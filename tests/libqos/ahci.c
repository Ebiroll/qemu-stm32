/*
 * libqos AHCI functions
 *
 * Copyright (c) 2014 John Snow <jsnow@redhat.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <glib.h>

#include "libqtest.h"
#include "libqos/ahci.h"
#include "libqos/pci-pc.h"

#include "qemu-common.h"
#include "qemu/host-utils.h"

#include "hw/pci/pci_ids.h"
#include "hw/pci/pci_regs.h"

typedef struct AHCICommandProp {
    uint8_t  cmd;        /* Command Code */
    bool     data;       /* Data transfer command? */
    bool     pio;
    bool     dma;
    bool     lba28;
    bool     lba48;
    bool     read;
    bool     write;
    bool     atapi;
    bool     ncq;
    uint64_t size;       /* Static transfer size, for commands like IDENTIFY. */
    uint32_t interrupts; /* Expected interrupts for this command. */
} AHCICommandProp;

AHCICommandProp ahci_command_properties[] = {
    { .cmd = CMD_READ_PIO,       .data = true,  .pio = true,
                                 .lba28 = true, .read = true },
    { .cmd = CMD_WRITE_PIO,      .data = true,  .pio = true,
                                 .lba28 = true, .write = true },
    { .cmd = CMD_READ_PIO_EXT,   .data = true,  .pio = true,
                                 .lba48 = true, .read = true },
    { .cmd = CMD_WRITE_PIO_EXT,  .data = true,  .pio = true,
                                 .lba48 = true, .write = true },
    { .cmd = CMD_READ_DMA,       .data = true,  .dma = true,
                                 .lba28 = true, .read = true },
    { .cmd = CMD_WRITE_DMA,      .data = true,  .dma = true,
                                 .lba28 = true, .write = true },
    { .cmd = CMD_READ_DMA_EXT,   .data = true,  .dma = true,
                                 .lba48 = true, .read = true },
    { .cmd = CMD_WRITE_DMA_EXT,  .data = true,  .dma = true,
                                 .lba48 = true, .write = true },
    { .cmd = CMD_IDENTIFY,       .data = true,  .pio = true,
                                 .size = 512,   .read = true },
    { .cmd = READ_FPDMA_QUEUED,  .data = true,  .dma = true,
                                 .lba48 = true, .read = true, .ncq = true },
    { .cmd = WRITE_FPDMA_QUEUED, .data = true,  .dma = true,
                                 .lba48 = true, .write = true, .ncq = true },
    { .cmd = CMD_READ_MAX,       .lba28 = true },
    { .cmd = CMD_READ_MAX_EXT,   .lba48 = true },
    { .cmd = CMD_FLUSH_CACHE,    .data = false }
};

struct AHCICommand {
    /* Test Management Data */
    uint8_t name;
    uint8_t port;
    uint8_t slot;
    uint32_t interrupts;
    uint64_t xbytes;
    uint32_t prd_size;
    uint64_t buffer;
    AHCICommandProp *props;
    /* Data to be transferred to the guest */
    AHCICommandHeader header;
    RegH2DFIS fis;
    void *atapi_cmd;
};

/**
 * Allocate space in the guest using information in the AHCIQState object.
 */
uint64_t ahci_alloc(AHCIQState *ahci, size_t bytes)
{
    g_assert(ahci);
    g_assert(ahci->parent);
    return qmalloc(ahci->parent, bytes);
}

void ahci_free(AHCIQState *ahci, uint64_t addr)
{
    g_assert(ahci);
    g_assert(ahci->parent);
    qfree(ahci->parent, addr);
}

/**
 * Locate, verify, and return a handle to the AHCI device.
 */
QPCIDevice *get_ahci_device(uint32_t *fingerprint)
{
    QPCIDevice *ahci;
    uint32_t ahci_fingerprint;
    QPCIBus *pcibus;

    pcibus = qpci_init_pc();

    /* Find the AHCI PCI device and verify it's the right one. */
    ahci = qpci_device_find(pcibus, QPCI_DEVFN(0x1F, 0x02));
    g_assert(ahci != NULL);

    ahci_fingerprint = qpci_config_readl(ahci, PCI_VENDOR_ID);

    switch (ahci_fingerprint) {
    case AHCI_INTEL_ICH9:
        break;
    default:
        /* Unknown device. */
        g_assert_not_reached();
    }

    if (fingerprint) {
        *fingerprint = ahci_fingerprint;
    }
    return ahci;
}

void free_ahci_device(QPCIDevice *dev)
{
    QPCIBus *pcibus = dev ? dev->bus : NULL;

    /* libqos doesn't have a function for this, so free it manually */
    g_free(dev);
    qpci_free_pc(pcibus);
}

/* Free all memory in-use by the AHCI device. */
void ahci_clean_mem(AHCIQState *ahci)
{
    uint8_t port, slot;

    for (port = 0; port < 32; ++port) {
        if (ahci->port[port].fb) {
            ahci_free(ahci, ahci->port[port].fb);
            ahci->port[port].fb = 0;
        }
        if (ahci->port[port].clb) {
            for (slot = 0; slot < 32; slot++) {
                ahci_destroy_command(ahci, port, slot);
            }
            ahci_free(ahci, ahci->port[port].clb);
            ahci->port[port].clb = 0;
        }
    }
}

/*** Logical Device Initialization ***/

/**
 * Start the PCI device and sanity-check default operation.
 */
void ahci_pci_enable(AHCIQState *ahci)
{
    uint8_t reg;

    start_ahci_device(ahci);

    switch (ahci->fingerprint) {
    case AHCI_INTEL_ICH9:
        /* ICH9 has a register at PCI 0x92 that
         * acts as a master port enabler mask. */
        reg = qpci_config_readb(ahci->dev, 0x92);
        reg |= 0x3F;
        qpci_config_writeb(ahci->dev, 0x92, reg);
        /* 0...0111111b -- bit significant, ports 0-5 enabled. */
        ASSERT_BIT_SET(qpci_config_readb(ahci->dev, 0x92), 0x3F);
        break;
    }

}

/**
 * Map BAR5/ABAR, and engage the PCI device.
 */
void start_ahci_device(AHCIQState *ahci)
{
    /* Map AHCI's ABAR (BAR5) */
    ahci->hba_base = qpci_iomap(ahci->dev, 5, &ahci->barsize);
    g_assert(ahci->hba_base);

    /* turns on pci.cmd.iose, pci.cmd.mse and pci.cmd.bme */
    qpci_device_enable(ahci->dev);
}

/**
 * Test and initialize the AHCI's HBA memory areas.
 * Initialize and start any ports with devices attached.
 * Bring the HBA into the idle state.
 */
void ahci_hba_enable(AHCIQState *ahci)
{
    /* Bits of interest in this section:
     * GHC.AE     Global Host Control / AHCI Enable
     * PxCMD.ST   Port Command: Start
     * PxCMD.SUD  "Spin Up Device"
     * PxCMD.POD  "Power On Device"
     * PxCMD.FRE  "FIS Receive Enable"
     * PxCMD.FR   "FIS Receive Running"
     * PxCMD.CR   "Command List Running"
     */
    uint32_t reg, ports_impl;
    uint16_t i;
    uint8_t num_cmd_slots;

    g_assert(ahci != NULL);

    /* Set GHC.AE to 1 */
    ahci_set(ahci, AHCI_GHC, AHCI_GHC_AE);
    reg = ahci_rreg(ahci, AHCI_GHC);
    ASSERT_BIT_SET(reg, AHCI_GHC_AE);

    /* Cache CAP and CAP2. */
    ahci->cap = ahci_rreg(ahci, AHCI_CAP);
    ahci->cap2 = ahci_rreg(ahci, AHCI_CAP2);

    /* Read CAP.NCS, how many command slots do we have? */
    num_cmd_slots = ((ahci->cap & AHCI_CAP_NCS) >> ctzl(AHCI_CAP_NCS)) + 1;
    g_test_message("Number of Command Slots: %u", num_cmd_slots);

    /* Determine which ports are implemented. */
    ports_impl = ahci_rreg(ahci, AHCI_PI);

    for (i = 0; ports_impl; ports_impl >>= 1, ++i) {
        if (!(ports_impl & 0x01)) {
            continue;
        }

        g_test_message("Initializing port %u", i);

        reg = ahci_px_rreg(ahci, i, AHCI_PX_CMD);
        if (BITCLR(reg, AHCI_PX_CMD_ST | AHCI_PX_CMD_CR |
                   AHCI_PX_CMD_FRE | AHCI_PX_CMD_FR)) {
            g_test_message("port is idle");
        } else {
            g_test_message("port needs to be idled");
            ahci_px_clr(ahci, i, AHCI_PX_CMD,
                        (AHCI_PX_CMD_ST | AHCI_PX_CMD_FRE));
            /* The port has 500ms to disengage. */
            usleep(500000);
            reg = ahci_px_rreg(ahci, i, AHCI_PX_CMD);
            ASSERT_BIT_CLEAR(reg, AHCI_PX_CMD_CR);
            ASSERT_BIT_CLEAR(reg, AHCI_PX_CMD_FR);
            g_test_message("port is now idle");
            /* The spec does allow for possibly needing a PORT RESET
             * or HBA reset if we fail to idle the port. */
        }

        /* Allocate Memory for the Command List Buffer & FIS Buffer */
        /* PxCLB space ... 0x20 per command, as in 4.2.2 p 36 */
        ahci->port[i].clb = ahci_alloc(ahci, num_cmd_slots * 0x20);
        qmemset(ahci->port[i].clb, 0x00, num_cmd_slots * 0x20);
        g_test_message("CLB: 0x%08" PRIx64, ahci->port[i].clb);
        ahci_px_wreg(ahci, i, AHCI_PX_CLB, ahci->port[i].clb);
        g_assert_cmphex(ahci->port[i].clb, ==,
                        ahci_px_rreg(ahci, i, AHCI_PX_CLB));

        /* PxFB space ... 0x100, as in 4.2.1 p 35 */
        ahci->port[i].fb = ahci_alloc(ahci, 0x100);
        qmemset(ahci->port[i].fb, 0x00, 0x100);
        g_test_message("FB: 0x%08" PRIx64, ahci->port[i].fb);
        ahci_px_wreg(ahci, i, AHCI_PX_FB, ahci->port[i].fb);
        g_assert_cmphex(ahci->port[i].fb, ==,
                        ahci_px_rreg(ahci, i, AHCI_PX_FB));

        /* Clear PxSERR, PxIS, then IS.IPS[x] by writing '1's. */
        ahci_px_wreg(ahci, i, AHCI_PX_SERR, 0xFFFFFFFF);
        ahci_px_wreg(ahci, i, AHCI_PX_IS, 0xFFFFFFFF);
        ahci_wreg(ahci, AHCI_IS, (1 << i));

        /* Verify Interrupts Cleared */
        reg = ahci_px_rreg(ahci, i, AHCI_PX_SERR);
        g_assert_cmphex(reg, ==, 0);

        reg = ahci_px_rreg(ahci, i, AHCI_PX_IS);
        g_assert_cmphex(reg, ==, 0);

        reg = ahci_rreg(ahci, AHCI_IS);
        ASSERT_BIT_CLEAR(reg, (1 << i));

        /* Enable All Interrupts: */
        ahci_px_wreg(ahci, i, AHCI_PX_IE, 0xFFFFFFFF);
        reg = ahci_px_rreg(ahci, i, AHCI_PX_IE);
        g_assert_cmphex(reg, ==, ~((uint32_t)AHCI_PX_IE_RESERVED));

        /* Enable the FIS Receive Engine. */
        ahci_px_set(ahci, i, AHCI_PX_CMD, AHCI_PX_CMD_FRE);
        reg = ahci_px_rreg(ahci, i, AHCI_PX_CMD);
        ASSERT_BIT_SET(reg, AHCI_PX_CMD_FR);

        /* AHCI 1.3 spec: if !STS.BSY, !STS.DRQ and PxSSTS.DET indicates
         * physical presence, a device is present and may be started. However,
         * PxSERR.DIAG.X /may/ need to be cleared a priori. */
        reg = ahci_px_rreg(ahci, i, AHCI_PX_SERR);
        if (BITSET(reg, AHCI_PX_SERR_DIAG_X)) {
            ahci_px_set(ahci, i, AHCI_PX_SERR, AHCI_PX_SERR_DIAG_X);
        }

        reg = ahci_px_rreg(ahci, i, AHCI_PX_TFD);
        if (BITCLR(reg, AHCI_PX_TFD_STS_BSY | AHCI_PX_TFD_STS_DRQ)) {
            reg = ahci_px_rreg(ahci, i, AHCI_PX_SSTS);
            if ((reg & AHCI_PX_SSTS_DET) == SSTS_DET_ESTABLISHED) {
                /* Device Found: set PxCMD.ST := 1 */
                ahci_px_set(ahci, i, AHCI_PX_CMD, AHCI_PX_CMD_ST);
                ASSERT_BIT_SET(ahci_px_rreg(ahci, i, AHCI_PX_CMD),
                               AHCI_PX_CMD_CR);
                g_test_message("Started Device %u", i);
            } else if ((reg & AHCI_PX_SSTS_DET)) {
                /* Device present, but in some unknown state. */
                g_assert_not_reached();
            }
        }
    }

    /* Enable GHC.IE */
    ahci_set(ahci, AHCI_GHC, AHCI_GHC_IE);
    reg = ahci_rreg(ahci, AHCI_GHC);
    ASSERT_BIT_SET(reg, AHCI_GHC_IE);

    /* TODO: The device should now be idling and waiting for commands.
     * In the future, a small test-case to inspect the Register D2H FIS
     * and clear the initial interrupts might be good. */
}

/**
 * Pick the first implemented and running port
 */
unsigned ahci_port_select(AHCIQState *ahci)
{
    uint32_t ports, reg;
    unsigned i;

    ports = ahci_rreg(ahci, AHCI_PI);
    for (i = 0; i < 32; ports >>= 1, ++i) {
        if (ports == 0) {
            i = 32;
        }

        if (!(ports & 0x01)) {
            continue;
        }

        reg = ahci_px_rreg(ahci, i, AHCI_PX_CMD);
        if (BITSET(reg, AHCI_PX_CMD_ST)) {
            break;
        }
    }
    g_assert(i < 32);
    return i;
}

/**
 * Clear a port's interrupts and status information prior to a test.
 */
void ahci_port_clear(AHCIQState *ahci, uint8_t port)
{
    uint32_t reg;

    /* Clear out this port's interrupts (ignore the init register d2h fis) */
    reg = ahci_px_rreg(ahci, port, AHCI_PX_IS);
    ahci_px_wreg(ahci, port, AHCI_PX_IS, reg);
    g_assert_cmphex(ahci_px_rreg(ahci, port, AHCI_PX_IS), ==, 0);

    /* Wipe the FIS-Receive Buffer */
    qmemset(ahci->port[port].fb, 0x00, 0x100);
}

/**
 * Check a port for errors.
 */
void ahci_port_check_error(AHCIQState *ahci, uint8_t port)
{
    uint32_t reg;

    /* The upper 9 bits of the IS register all indicate errors. */
    reg = ahci_px_rreg(ahci, port, AHCI_PX_IS);
    reg >>= 23;
    g_assert_cmphex(reg, ==, 0);

    /* The Sata Error Register should be empty. */
    reg = ahci_px_rreg(ahci, port, AHCI_PX_SERR);
    g_assert_cmphex(reg, ==, 0);

    /* The TFD also has two error sections. */
    reg = ahci_px_rreg(ahci, port, AHCI_PX_TFD);
    ASSERT_BIT_CLEAR(reg, AHCI_PX_TFD_STS_ERR);
    ASSERT_BIT_CLEAR(reg, AHCI_PX_TFD_ERR);
}

void ahci_port_check_interrupts(AHCIQState *ahci, uint8_t port,
                                uint32_t intr_mask)
{
    uint32_t reg;

    /* Check for expected interrupts */
    reg = ahci_px_rreg(ahci, port, AHCI_PX_IS);
    ASSERT_BIT_SET(reg, intr_mask);

    /* Clear expected interrupts and assert all interrupts now cleared. */
    ahci_px_wreg(ahci, port, AHCI_PX_IS, intr_mask);
    g_assert_cmphex(ahci_px_rreg(ahci, port, AHCI_PX_IS), ==, 0);
}

void ahci_port_check_nonbusy(AHCIQState *ahci, uint8_t port, uint8_t slot)
{
    uint32_t reg;

    /* Assert that the command slot is no longer busy (NCQ) */
    reg = ahci_px_rreg(ahci, port, AHCI_PX_SACT);
    ASSERT_BIT_CLEAR(reg, (1 << slot));

    /* Non-NCQ */
    reg = ahci_px_rreg(ahci, port, AHCI_PX_CI);
    ASSERT_BIT_CLEAR(reg, (1 << slot));

    /* And assert that we are generally not busy. */
    reg = ahci_px_rreg(ahci, port, AHCI_PX_TFD);
    ASSERT_BIT_CLEAR(reg, AHCI_PX_TFD_STS_BSY);
    ASSERT_BIT_CLEAR(reg, AHCI_PX_TFD_STS_DRQ);
}

void ahci_port_check_d2h_sanity(AHCIQState *ahci, uint8_t port, uint8_t slot)
{
    RegD2HFIS *d2h = g_malloc0(0x20);
    uint32_t reg;

    memread(ahci->port[port].fb + 0x40, d2h, 0x20);
    g_assert_cmphex(d2h->fis_type, ==, 0x34);

    reg = ahci_px_rreg(ahci, port, AHCI_PX_TFD);
    g_assert_cmphex((reg & AHCI_PX_TFD_ERR) >> 8, ==, d2h->error);
    g_assert_cmphex((reg & AHCI_PX_TFD_STS), ==, d2h->status);

    g_free(d2h);
}

void ahci_port_check_pio_sanity(AHCIQState *ahci, uint8_t port,
                                uint8_t slot, size_t buffsize)
{
    PIOSetupFIS *pio = g_malloc0(0x20);

    /* We cannot check the Status or E_Status registers, because
     * the status may have again changed between the PIO Setup FIS
     * and the conclusion of the command with the D2H Register FIS. */
    memread(ahci->port[port].fb + 0x20, pio, 0x20);
    g_assert_cmphex(pio->fis_type, ==, 0x5f);

    /* BUG: PIO Setup FIS as utilized by QEMU tries to fit the entire
     * transfer size in a uint16_t field. The maximum transfer size can
     * eclipse this; the field is meant to convey the size of data per
     * each Data FIS, not the entire operation as a whole. For now,
     * we will sanity check the broken case where applicable. */
    if (buffsize <= UINT16_MAX) {
        g_assert_cmphex(le16_to_cpu(pio->tx_count), ==, buffsize);
    }

    g_free(pio);
}

void ahci_port_check_cmd_sanity(AHCIQState *ahci, AHCICommand *cmd)
{
    AHCICommandHeader cmdh;

    ahci_get_command_header(ahci, cmd->port, cmd->slot, &cmdh);
    /* Physical Region Descriptor Byte Count is not required to work for NCQ. */
    if (!cmd->props->ncq) {
        g_assert_cmphex(cmd->xbytes, ==, cmdh.prdbc);
    }
}

/* Get the command in #slot of port #port. */
void ahci_get_command_header(AHCIQState *ahci, uint8_t port,
                             uint8_t slot, AHCICommandHeader *cmd)
{
    uint64_t ba = ahci->port[port].clb;
    ba += slot * sizeof(AHCICommandHeader);
    memread(ba, cmd, sizeof(AHCICommandHeader));

    cmd->flags = le16_to_cpu(cmd->flags);
    cmd->prdtl = le16_to_cpu(cmd->prdtl);
    cmd->prdbc = le32_to_cpu(cmd->prdbc);
    cmd->ctba = le64_to_cpu(cmd->ctba);
}

/* Set the command in #slot of port #port. */
void ahci_set_command_header(AHCIQState *ahci, uint8_t port,
                             uint8_t slot, AHCICommandHeader *cmd)
{
    AHCICommandHeader tmp = { .flags = 0 };
    uint64_t ba = ahci->port[port].clb;
    ba += slot * sizeof(AHCICommandHeader);

    tmp.flags = cpu_to_le16(cmd->flags);
    tmp.prdtl = cpu_to_le16(cmd->prdtl);
    tmp.prdbc = cpu_to_le32(cmd->prdbc);
    tmp.ctba = cpu_to_le64(cmd->ctba);

    memwrite(ba, &tmp, sizeof(AHCICommandHeader));
}

void ahci_destroy_command(AHCIQState *ahci, uint8_t port, uint8_t slot)
{
    AHCICommandHeader cmd;

    /* Obtain the Nth Command Header */
    ahci_get_command_header(ahci, port, slot, &cmd);
    if (cmd.ctba == 0) {
        /* No address in it, so just return -- it's empty. */
        goto tidy;
    }

    /* Free the Table */
    ahci_free(ahci, cmd.ctba);

 tidy:
    /* NULL the header. */
    memset(&cmd, 0x00, sizeof(cmd));
    ahci_set_command_header(ahci, port, slot, &cmd);
    ahci->port[port].ctba[slot] = 0;
    ahci->port[port].prdtl[slot] = 0;
}

void ahci_write_fis(AHCIQState *ahci, RegH2DFIS *fis, uint64_t addr)
{
    RegH2DFIS tmp = *fis;

    /* The auxiliary FIS fields are defined per-command and are not
     * currently implemented in libqos/ahci.o, but may or may not need
     * to be flipped. */

    /* All other FIS fields are 8 bit and do not need to be flipped. */
    tmp.count = cpu_to_le16(tmp.count);

    memwrite(addr, &tmp, sizeof(tmp));
}

unsigned ahci_pick_cmd(AHCIQState *ahci, uint8_t port)
{
    unsigned i;
    unsigned j;
    uint32_t reg;

    reg = ahci_px_rreg(ahci, port, AHCI_PX_CI);

    /* Pick the least recently used command slot that's available */
    for (i = 0; i < 32; ++i) {
        j = ((ahci->port[port].next + i) % 32);
        if (reg & (1 << j)) {
            continue;
        }
        ahci_destroy_command(ahci, port, j);
        ahci->port[port].next = (j + 1) % 32;
        return j;
    }

    g_test_message("All command slots were busy.");
    g_assert_not_reached();
}

inline unsigned size_to_prdtl(unsigned bytes, unsigned bytes_per_prd)
{
    /* Each PRD can describe up to 4MiB */
    g_assert_cmphex(bytes_per_prd, <=, 4096 * 1024);
    g_assert_cmphex(bytes_per_prd & 0x01, ==, 0x00);
    return (bytes + bytes_per_prd - 1) / bytes_per_prd;
}

/* Issue a command, expecting it to fail and STOP the VM */
AHCICommand *ahci_guest_io_halt(AHCIQState *ahci, uint8_t port,
                                uint8_t ide_cmd, uint64_t buffer,
                                size_t bufsize, uint64_t sector)
{
    AHCICommand *cmd;

    cmd = ahci_command_create(ide_cmd);
    ahci_command_adjust(cmd, sector, buffer, bufsize, 0);
    ahci_command_commit(ahci, cmd, port);
    ahci_command_issue_async(ahci, cmd);
    qmp_eventwait("STOP");

    return cmd;
}

/* Resume a previously failed command and verify/finalize */
void ahci_guest_io_resume(AHCIQState *ahci, AHCICommand *cmd)
{
    /* Complete the command */
    qmp_async("{'execute':'cont' }");
    qmp_eventwait("RESUME");
    ahci_command_wait(ahci, cmd);
    ahci_command_verify(ahci, cmd);
    ahci_command_free(cmd);
}

/* Given a guest buffer address, perform an IO operation */
void ahci_guest_io(AHCIQState *ahci, uint8_t port, uint8_t ide_cmd,
                   uint64_t buffer, size_t bufsize, uint64_t sector)
{
    AHCICommand *cmd;
    cmd = ahci_command_create(ide_cmd);
    ahci_command_set_buffer(cmd, buffer);
    ahci_command_set_size(cmd, bufsize);
    if (sector) {
        ahci_command_set_offset(cmd, sector);
    }
    ahci_command_commit(ahci, cmd, port);
    ahci_command_issue(ahci, cmd);
    ahci_command_verify(ahci, cmd);
    ahci_command_free(cmd);
}

static AHCICommandProp *ahci_command_find(uint8_t command_name)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(ahci_command_properties); i++) {
        if (ahci_command_properties[i].cmd == command_name) {
            return &ahci_command_properties[i];
        }
    }

    return NULL;
}

/* Given a HOST buffer, create a buffer address and perform an IO operation. */
void ahci_io(AHCIQState *ahci, uint8_t port, uint8_t ide_cmd,
             void *buffer, size_t bufsize, uint64_t sector)
{
    uint64_t ptr;
    AHCICommandProp *props;

    props = ahci_command_find(ide_cmd);
    g_assert(props);
    ptr = ahci_alloc(ahci, bufsize);
    g_assert(ptr);
    qmemset(ptr, 0x00, bufsize);

    if (props->write) {
        bufwrite(ptr, buffer, bufsize);
    }

    ahci_guest_io(ahci, port, ide_cmd, ptr, bufsize, sector);

    if (props->read) {
        bufread(ptr, buffer, bufsize);
    }

    ahci_free(ahci, ptr);
}

/**
 * Initializes a basic command header in memory.
 * We assume that this is for an ATA command using RegH2DFIS.
 */
static void command_header_init(AHCICommand *cmd)
{
    AHCICommandHeader *hdr = &cmd->header;
    AHCICommandProp *props = cmd->props;

    hdr->flags = 5;             /* RegH2DFIS is 5 DW long. Must be < 32 */
    hdr->flags |= CMDH_CLR_BSY; /* Clear the BSY bit when done */
    if (props->write) {
        hdr->flags |= CMDH_WRITE;
    }
    if (props->atapi) {
        hdr->flags |= CMDH_ATAPI;
    }
    /* Other flags: PREFETCH, RESET, and BIST */
    hdr->prdtl = size_to_prdtl(cmd->xbytes, cmd->prd_size);
    hdr->prdbc = 0;
    hdr->ctba = 0;
}

static void command_table_init(AHCICommand *cmd)
{
    RegH2DFIS *fis = &(cmd->fis);
    uint16_t sect_count = (cmd->xbytes / AHCI_SECTOR_SIZE);

    fis->fis_type = REG_H2D_FIS;
    fis->flags = REG_H2D_FIS_CMD; /* "Command" bit */
    fis->command = cmd->name;

    if (cmd->props->ncq) {
        NCQFIS *ncqfis = (NCQFIS *)fis;
        /* NCQ is weird and re-uses FIS frames for unrelated data.
         * See SATA 3.2, 13.6.4.1 READ FPDMA QUEUED for an example. */
        ncqfis->sector_low = sect_count & 0xFF;
        ncqfis->sector_hi = (sect_count >> 8) & 0xFF;
        ncqfis->device = NCQ_DEVICE_MAGIC;
        /* Force Unit Access is bit 7 in the device register */
        ncqfis->tag = 0;  /* bits 3-7 are the NCQ tag */
        ncqfis->prio = 0; /* bits 6,7 are a prio tag */
        /* RARC bit is bit 0 of TAG field */
    } else {
        fis->feature_low = 0x00;
        fis->feature_high = 0x00;
        if (cmd->props->lba28 || cmd->props->lba48) {
            fis->device = ATA_DEVICE_LBA;
        }
        fis->count = (cmd->xbytes / AHCI_SECTOR_SIZE);
    }
    fis->icc = 0x00;
    fis->control = 0x00;
    memset(fis->aux, 0x00, ARRAY_SIZE(fis->aux));
}

AHCICommand *ahci_command_create(uint8_t command_name)
{
    AHCICommandProp *props = ahci_command_find(command_name);
    AHCICommand *cmd;

    g_assert(props);
    cmd = g_malloc0(sizeof(AHCICommand));
    g_assert(!(props->dma && props->pio));
    g_assert(!(props->lba28 && props->lba48));
    g_assert(!(props->read && props->write));
    g_assert(!props->size || props->data);
    g_assert(!props->ncq || (props->ncq && props->lba48));

    /* Defaults and book-keeping */
    cmd->props = props;
    cmd->name = command_name;
    cmd->xbytes = props->size;
    cmd->prd_size = 4096;
    cmd->buffer = 0xabad1dea;

    if (!cmd->props->ncq) {
        cmd->interrupts = AHCI_PX_IS_DHRS;
    }
    /* BUG: We expect the DPS interrupt for data commands */
    /* cmd->interrupts |= props->data ? AHCI_PX_IS_DPS : 0; */
    /* BUG: We expect the DMA Setup interrupt for DMA commands */
    /* cmd->interrupts |= props->dma ? AHCI_PX_IS_DSS : 0; */
    cmd->interrupts |= props->pio ? AHCI_PX_IS_PSS : 0;
    cmd->interrupts |= props->ncq ? AHCI_PX_IS_SDBS : 0;

    command_header_init(cmd);
    command_table_init(cmd);

    return cmd;
}

void ahci_command_free(AHCICommand *cmd)
{
    g_free(cmd);
}

void ahci_command_set_flags(AHCICommand *cmd, uint16_t cmdh_flags)
{
    cmd->header.flags |= cmdh_flags;
}

void ahci_command_clr_flags(AHCICommand *cmd, uint16_t cmdh_flags)
{
    cmd->header.flags &= ~cmdh_flags;
}

void ahci_command_set_offset(AHCICommand *cmd, uint64_t lba_sect)
{
    RegH2DFIS *fis = &(cmd->fis);
    if (cmd->props->lba28) {
        g_assert_cmphex(lba_sect, <=, 0xFFFFFFF);
    } else if (cmd->props->lba48 || cmd->props->ncq) {
        g_assert_cmphex(lba_sect, <=, 0xFFFFFFFFFFFF);
    } else {
        /* Can't set offset if we don't know the format. */
        g_assert_not_reached();
    }

    /* LBA28 uses the low nibble of the device/control register for LBA24:27 */
    fis->lba_lo[0] = (lba_sect & 0xFF);
    fis->lba_lo[1] = (lba_sect >> 8) & 0xFF;
    fis->lba_lo[2] = (lba_sect >> 16) & 0xFF;
    if (cmd->props->lba28) {
        fis->device = (fis->device & 0xF0) | ((lba_sect >> 24) & 0x0F);
    }
    fis->lba_hi[0] = (lba_sect >> 24) & 0xFF;
    fis->lba_hi[1] = (lba_sect >> 32) & 0xFF;
    fis->lba_hi[2] = (lba_sect >> 40) & 0xFF;
}

void ahci_command_set_buffer(AHCICommand *cmd, uint64_t buffer)
{
    cmd->buffer = buffer;
}

void ahci_command_set_sizes(AHCICommand *cmd, uint64_t xbytes,
                            unsigned prd_size)
{
    uint16_t sect_count;

    /* Each PRD can describe up to 4MiB, and must not be odd. */
    g_assert_cmphex(prd_size, <=, 4096 * 1024);
    g_assert_cmphex(prd_size & 0x01, ==, 0x00);
    if (prd_size) {
        cmd->prd_size = prd_size;
    }
    cmd->xbytes = xbytes;
    sect_count = (cmd->xbytes / AHCI_SECTOR_SIZE);

    if (cmd->props->ncq) {
        NCQFIS *nfis = (NCQFIS *)&(cmd->fis);
        nfis->sector_low = sect_count & 0xFF;
        nfis->sector_hi = (sect_count >> 8) & 0xFF;
    } else {
        cmd->fis.count = sect_count;
    }
    cmd->header.prdtl = size_to_prdtl(cmd->xbytes, cmd->prd_size);
}

void ahci_command_set_size(AHCICommand *cmd, uint64_t xbytes)
{
    ahci_command_set_sizes(cmd, xbytes, cmd->prd_size);
}

void ahci_command_set_prd_size(AHCICommand *cmd, unsigned prd_size)
{
    ahci_command_set_sizes(cmd, cmd->xbytes, prd_size);
}

void ahci_command_adjust(AHCICommand *cmd, uint64_t offset, uint64_t buffer,
                         uint64_t xbytes, unsigned prd_size)
{
    ahci_command_set_sizes(cmd, xbytes, prd_size);
    ahci_command_set_buffer(cmd, buffer);
    ahci_command_set_offset(cmd, offset);
}

void ahci_command_commit(AHCIQState *ahci, AHCICommand *cmd, uint8_t port)
{
    uint16_t i, prdtl;
    uint64_t table_size, table_ptr, remaining;
    PRD prd;

    /* This command is now tied to this port/command slot */
    cmd->port = port;
    cmd->slot = ahci_pick_cmd(ahci, port);

    if (cmd->props->ncq) {
        NCQFIS *nfis = (NCQFIS *)&cmd->fis;
        nfis->tag = (cmd->slot << 3) & 0xFC;
    }

    /* Create a buffer for the command table */
    prdtl = size_to_prdtl(cmd->xbytes, cmd->prd_size);
    table_size = CMD_TBL_SIZ(prdtl);
    table_ptr = ahci_alloc(ahci, table_size);
    g_assert(table_ptr);
    /* AHCI 1.3: Must be aligned to 0x80 */
    g_assert((table_ptr & 0x7F) == 0x00);
    cmd->header.ctba = table_ptr;

    /* Commit the command header and command FIS */
    ahci_set_command_header(ahci, port, cmd->slot, &(cmd->header));
    ahci_write_fis(ahci, &(cmd->fis), table_ptr);

    /* Construct and write the PRDs to the command table */
    g_assert_cmphex(prdtl, ==, cmd->header.prdtl);
    remaining = cmd->xbytes;
    for (i = 0; i < prdtl; ++i) {
        prd.dba = cpu_to_le64(cmd->buffer + (cmd->prd_size * i));
        prd.res = 0;
        if (remaining > cmd->prd_size) {
            /* Note that byte count is 0-based. */
            prd.dbc = cpu_to_le32(cmd->prd_size - 1);
            remaining -= cmd->prd_size;
        } else {
            /* Again, dbc is 0-based. */
            prd.dbc = cpu_to_le32(remaining - 1);
            remaining = 0;
        }
        prd.dbc |= cpu_to_le32(0x80000000); /* Request DPS Interrupt */

        /* Commit the PRD entry to the Command Table */
        memwrite(table_ptr + 0x80 + (i * sizeof(PRD)),
                 &prd, sizeof(PRD));
    }

    /* Bookmark the PRDTL and CTBA values */
    ahci->port[port].ctba[cmd->slot] = table_ptr;
    ahci->port[port].prdtl[cmd->slot] = prdtl;
}

void ahci_command_issue_async(AHCIQState *ahci, AHCICommand *cmd)
{
    if (cmd->props->ncq) {
        ahci_px_wreg(ahci, cmd->port, AHCI_PX_SACT, (1 << cmd->slot));
    }

    ahci_px_wreg(ahci, cmd->port, AHCI_PX_CI, (1 << cmd->slot));
}

void ahci_command_wait(AHCIQState *ahci, AHCICommand *cmd)
{
    /* We can't rely on STS_BSY until the command has started processing.
     * Therefore, we also use the Command Issue bit as indication of
     * a command in-flight. */

#define RSET(REG, MASK) (BITSET(ahci_px_rreg(ahci, cmd->port, (REG)), (MASK)))

    while (RSET(AHCI_PX_TFD, AHCI_PX_TFD_STS_BSY) ||
           RSET(AHCI_PX_CI, 1 << cmd->slot) ||
           (cmd->props->ncq && RSET(AHCI_PX_SACT, 1 << cmd->slot))) {
        usleep(50);
    }

}

void ahci_command_issue(AHCIQState *ahci, AHCICommand *cmd)
{
    ahci_command_issue_async(ahci, cmd);
    ahci_command_wait(ahci, cmd);
}

void ahci_command_verify(AHCIQState *ahci, AHCICommand *cmd)
{
    uint8_t slot = cmd->slot;
    uint8_t port = cmd->port;

    ahci_port_check_error(ahci, port);
    ahci_port_check_interrupts(ahci, port, cmd->interrupts);
    ahci_port_check_nonbusy(ahci, port, slot);
    ahci_port_check_cmd_sanity(ahci, cmd);
    if (cmd->interrupts & AHCI_PX_IS_DHRS) {
        ahci_port_check_d2h_sanity(ahci, port, slot);
    }
    if (cmd->props->pio) {
        ahci_port_check_pio_sanity(ahci, port, slot, cmd->xbytes);
    }
}

uint8_t ahci_command_slot(AHCICommand *cmd)
{
    return cmd->slot;
}
