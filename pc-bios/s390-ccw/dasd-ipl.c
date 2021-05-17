/*
 * S390 IPL (boot) from a real DASD device via vfio framework.
 *
 * Copyright (c) 2019 Jason J. Herne <jjherne@us.ibm.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or (at
 * your option) any later version. See the COPYING file in the top-level
 * directory.
 */

#include "libc.h"
#include "s390-ccw.h"
#include "s390-arch.h"
#include "dasd-ipl.h"
#include "helper.h"

static char prefix_page[PAGE_SIZE * 2]
            __attribute__((__aligned__(PAGE_SIZE * 2)));

static void enable_prefixing(void)
{
    memcpy(&prefix_page, lowcore, 4096);
    set_prefix(ptr2u32(&prefix_page));
}

static void disable_prefixing(void)
{
    set_prefix(0);
    /* Copy io interrupt info back to low core */
    memcpy((void *)&lowcore->subchannel_id, prefix_page + 0xB8, 12);
}

static bool is_read_tic_ccw_chain(Ccw0 *ccw)
{
    Ccw0 *next_ccw = ccw + 1;

    return ((ccw->cmd_code == CCW_CMD_DASD_READ ||
            ccw->cmd_code == CCW_CMD_DASD_READ_MT) &&
            ccw->chain && next_ccw->cmd_code == CCW_CMD_TIC);
}

static bool dynamic_cp_fixup(uint32_t ccw_addr, uint32_t  *next_cpa)
{
    Ccw0 *cur_ccw = (Ccw0 *)(uint64_t)ccw_addr;
    Ccw0 *tic_ccw;

    while (true) {
        /* Skip over inline TIC (it might not have the chain bit on)  */
        if (cur_ccw->cmd_code == CCW_CMD_TIC &&
            cur_ccw->cda == ptr2u32(cur_ccw) - 8) {
            cur_ccw += 1;
            continue;
        }

        if (!cur_ccw->chain) {
            break;
        }
        if (is_read_tic_ccw_chain(cur_ccw)) {
            /*
             * Breaking a chain of CCWs may alter the semantics or even the
             * validity of a channel program. The heuristic implemented below
             * seems to work well in practice for the channel programs
             * generated by zipl.
             */
            tic_ccw = cur_ccw + 1;
            *next_cpa = tic_ccw->cda;
            cur_ccw->chain = 0;
            return true;
        }
        cur_ccw += 1;
    }
    return false;
}

static int run_dynamic_ccw_program(SubChannelId schid, uint16_t cutype,
                                   uint32_t cpa)
{
    bool has_next;
    uint32_t next_cpa = 0;
    int rc;

    do {
        has_next = dynamic_cp_fixup(cpa, &next_cpa);

        print_int("executing ccw chain at ", cpa);
        enable_prefixing();
        rc = do_cio(schid, cutype, cpa, CCW_FMT0);
        disable_prefixing();

        if (rc) {
            break;
        }
        cpa = next_cpa;
    } while (has_next);

    return rc;
}

static void make_readipl(void)
{
    Ccw0 *ccwIplRead = (Ccw0 *)0x00;

    /* Clear out any existing data */
    memset(ccwIplRead, 0, sizeof(Ccw0));

    /* Create Read IPL ccw at address 0 */
    ccwIplRead->cmd_code = CCW_CMD_READ_IPL;
    ccwIplRead->cda = 0x00; /* Read into address 0x00 in main memory */
    ccwIplRead->chain = 0; /* Chain flag */
    ccwIplRead->count = 0x18; /* Read 0x18 bytes of data */
}

static void run_readipl(SubChannelId schid, uint16_t cutype)
{
    if (do_cio(schid, cutype, 0x00, CCW_FMT0)) {
        panic("dasd-ipl: Failed to run Read IPL channel program\n");
    }
}

/*
 * The architecture states that IPL1 data should consist of a psw followed by
 * format-0 READ and TIC CCWs. Let's sanity check.
 */
static void check_ipl1(void)
{
    Ccw0 *ccwread = (Ccw0 *)0x08;
    Ccw0 *ccwtic = (Ccw0 *)0x10;

    if (ccwread->cmd_code != CCW_CMD_DASD_READ ||
        ccwtic->cmd_code != CCW_CMD_TIC) {
        panic("dasd-ipl: IPL1 data invalid. Is this disk really bootable?\n");
    }
}

static void check_ipl2(uint32_t ipl2_addr)
{
    Ccw0 *ccw = u32toptr(ipl2_addr);

    if (ipl2_addr == 0x00) {
        panic("IPL2 address invalid. Is this disk really bootable?\n");
    }
    if (ccw->cmd_code == 0x00) {
        panic("IPL2 ccw data invalid. Is this disk really bootable?\n");
    }
}

static uint32_t read_ipl2_addr(void)
{
    Ccw0 *ccwtic = (Ccw0 *)0x10;

    return ccwtic->cda;
}

static void ipl1_fixup(void)
{
    Ccw0 *ccwSeek = (Ccw0 *) 0x08;
    Ccw0 *ccwSearchID = (Ccw0 *) 0x10;
    Ccw0 *ccwSearchTic = (Ccw0 *) 0x18;
    Ccw0 *ccwRead = (Ccw0 *) 0x20;
    CcwSeekData *seekData = (CcwSeekData *) 0x30;
    CcwSearchIdData *searchData = (CcwSearchIdData *) 0x38;

    /* move IPL1 CCWs to make room for CCWs needed to locate record 2 */
    memcpy(ccwRead, (void *)0x08, 16);

    /* Disable chaining so we don't TIC to IPL2 channel program */
    ccwRead->chain = 0x00;

    ccwSeek->cmd_code = CCW_CMD_DASD_SEEK;
    ccwSeek->cda = ptr2u32(seekData);
    ccwSeek->chain = 1;
    ccwSeek->count = sizeof(*seekData);
    seekData->reserved = 0x00;
    seekData->cyl = 0x00;
    seekData->head = 0x00;

    ccwSearchID->cmd_code = CCW_CMD_DASD_SEARCH_ID_EQ;
    ccwSearchID->cda = ptr2u32(searchData);
    ccwSearchID->chain = 1;
    ccwSearchID->count = sizeof(*searchData);
    searchData->cyl = 0;
    searchData->head = 0;
    searchData->record = 2;

    /* Go back to Search CCW if correct record not yet found */
    ccwSearchTic->cmd_code = CCW_CMD_TIC;
    ccwSearchTic->cda = ptr2u32(ccwSearchID);
}

static void run_ipl1(SubChannelId schid, uint16_t cutype)
 {
    uint32_t startAddr = 0x08;

    if (do_cio(schid, cutype, startAddr, CCW_FMT0)) {
        panic("dasd-ipl: Failed to run IPL1 channel program\n");
    }
}

static void run_ipl2(SubChannelId schid, uint16_t cutype, uint32_t addr)
{
    if (run_dynamic_ccw_program(schid, cutype, addr)) {
        panic("dasd-ipl: Failed to run IPL2 channel program\n");
    }
}

/*
 * Limitations in vfio-ccw support complicate the IPL process. Details can
 * be found in docs/devel/s390-dasd-ipl.rst
 */
void dasd_ipl(SubChannelId schid, uint16_t cutype)
{
    PSWLegacy *pswl = (PSWLegacy *) 0x00;
    uint32_t ipl2_addr;

    /* Construct Read IPL CCW and run it to read IPL1 from boot disk */
    make_readipl();
    run_readipl(schid, cutype);
    ipl2_addr = read_ipl2_addr();
    check_ipl1();

    /*
     * Fixup IPL1 channel program to account for vfio-ccw limitations, then run
     * it to read IPL2 channel program from boot disk.
     */
    ipl1_fixup();
    run_ipl1(schid, cutype);
    check_ipl2(ipl2_addr);

    /*
     * Run IPL2 channel program to read operating system code from boot disk
     */
    run_ipl2(schid, cutype, ipl2_addr);

    /* Transfer control to the guest operating system */
    pswl->mask |= PSW_MASK_EAMODE;   /* Force z-mode */
    pswl->addr |= PSW_MASK_BAMODE;   /* ...          */
    jump_to_low_kernel();
}
