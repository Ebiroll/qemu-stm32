#ifndef XEN_PT_H
#define XEN_PT_H

#include "qemu-common.h"
#include "xen_common.h"
#include "pci.h"
#include "xen-host-pci-device.h"

void xen_pt_log(const PCIDevice *d, const char *f, ...) GCC_FMT_ATTR(2, 3);

#define XEN_PT_ERR(d, _f, _a...) xen_pt_log(d, "%s: Error: "_f, __func__, ##_a)

#ifdef XEN_PT_LOGGING_ENABLED
#  define XEN_PT_LOG(d, _f, _a...)  xen_pt_log(d, "%s: " _f, __func__, ##_a)
#  define XEN_PT_WARN(d, _f, _a...) \
    xen_pt_log(d, "%s: Warning: "_f, __func__, ##_a)
#else
#  define XEN_PT_LOG(d, _f, _a...)
#  define XEN_PT_WARN(d, _f, _a...)
#endif

#ifdef XEN_PT_DEBUG_PCI_CONFIG_ACCESS
#  define XEN_PT_LOG_CONFIG(d, addr, val, len) \
    xen_pt_log(d, "%s: address=0x%04x val=0x%08x len=%d\n", \
               __func__, addr, val, len)
#else
#  define XEN_PT_LOG_CONFIG(d, addr, val, len)
#endif


/* Helper */
#define XEN_PFN(x) ((x) >> XC_PAGE_SHIFT)

typedef struct XenPTRegInfo XenPTRegInfo;
typedef struct XenPTReg XenPTReg;

typedef struct XenPCIPassthroughState XenPCIPassthroughState;

/* function type for config reg */
typedef int (*xen_pt_conf_reg_init)
    (XenPCIPassthroughState *, XenPTRegInfo *, uint32_t real_offset,
     uint32_t *data);
typedef int (*xen_pt_conf_dword_write)
    (XenPCIPassthroughState *, XenPTReg *cfg_entry,
     uint32_t *val, uint32_t dev_value, uint32_t valid_mask);
typedef int (*xen_pt_conf_word_write)
    (XenPCIPassthroughState *, XenPTReg *cfg_entry,
     uint16_t *val, uint16_t dev_value, uint16_t valid_mask);
typedef int (*xen_pt_conf_byte_write)
    (XenPCIPassthroughState *, XenPTReg *cfg_entry,
     uint8_t *val, uint8_t dev_value, uint8_t valid_mask);
typedef int (*xen_pt_conf_dword_read)
    (XenPCIPassthroughState *, XenPTReg *cfg_entry,
     uint32_t *val, uint32_t valid_mask);
typedef int (*xen_pt_conf_word_read)
    (XenPCIPassthroughState *, XenPTReg *cfg_entry,
     uint16_t *val, uint16_t valid_mask);
typedef int (*xen_pt_conf_byte_read)
    (XenPCIPassthroughState *, XenPTReg *cfg_entry,
     uint8_t *val, uint8_t valid_mask);

#define XEN_PT_BAR_ALLF 0xFFFFFFFF
#define XEN_PT_BAR_UNMAPPED (-1)


typedef enum {
    XEN_PT_GRP_TYPE_HARDWIRED = 0,  /* 0 Hardwired reg group */
    XEN_PT_GRP_TYPE_EMU,            /* emul reg group */
} XenPTRegisterGroupType;

typedef enum {
    XEN_PT_BAR_FLAG_MEM = 0,        /* Memory type BAR */
    XEN_PT_BAR_FLAG_IO,             /* I/O type BAR */
    XEN_PT_BAR_FLAG_UPPER,          /* upper 64bit BAR */
    XEN_PT_BAR_FLAG_UNUSED,         /* unused BAR */
} XenPTBarFlag;


typedef struct XenPTRegion {
    /* BAR flag */
    XenPTBarFlag bar_flag;
    /* Translation of the emulated address */
    union {
        uint64_t maddr;
        uint64_t pio_base;
        uint64_t u;
    } access;
} XenPTRegion;

/* XenPTRegInfo declaration
 * - only for emulated register (either a part or whole bit).
 * - for passthrough register that need special behavior (like interacting with
 *   other component), set emu_mask to all 0 and specify r/w func properly.
 * - do NOT use ALL F for init_val, otherwise the tbl will not be registered.
 */

/* emulated register infomation */
struct XenPTRegInfo {
    uint32_t offset;
    uint32_t size;
    uint32_t init_val;
    /* reg read only field mask (ON:RO/ROS, OFF:other) */
    uint32_t ro_mask;
    /* reg emulate field mask (ON:emu, OFF:passthrough) */
    uint32_t emu_mask;
    /* no write back allowed */
    uint32_t no_wb;
    xen_pt_conf_reg_init init;
    /* read/write function pointer
     * for double_word/word/byte size */
    union {
        struct {
            xen_pt_conf_dword_write write;
            xen_pt_conf_dword_read read;
        } dw;
        struct {
            xen_pt_conf_word_write write;
            xen_pt_conf_word_read read;
        } w;
        struct {
            xen_pt_conf_byte_write write;
            xen_pt_conf_byte_read read;
        } b;
    } u;
};

/* emulated register management */
struct XenPTReg {
    QLIST_ENTRY(XenPTReg) entries;
    XenPTRegInfo *reg;
    uint32_t data; /* emulated value */
};

typedef struct XenPTRegGroupInfo XenPTRegGroupInfo;

/* emul reg group size initialize method */
typedef int (*xen_pt_reg_size_init_fn)
    (XenPCIPassthroughState *, const XenPTRegGroupInfo *,
     uint32_t base_offset, uint8_t *size);

/* emulated register group infomation */
struct XenPTRegGroupInfo {
    uint8_t grp_id;
    XenPTRegisterGroupType grp_type;
    uint8_t grp_size;
    xen_pt_reg_size_init_fn size_init;
    XenPTRegInfo *emu_regs;
};

/* emul register group management table */
typedef struct XenPTRegGroup {
    QLIST_ENTRY(XenPTRegGroup) entries;
    const XenPTRegGroupInfo *reg_grp;
    uint32_t base_offset;
    uint8_t size;
    QLIST_HEAD(, XenPTReg) reg_tbl_list;
} XenPTRegGroup;


#define XEN_PT_UNASSIGNED_PIRQ (-1)

struct XenPCIPassthroughState {
    PCIDevice dev;

    PCIHostDeviceAddress hostaddr;
    bool is_virtfn;
    XenHostPCIDevice real_device;
    XenPTRegion bases[PCI_NUM_REGIONS]; /* Access regions */
    QLIST_HEAD(, XenPTRegGroup) reg_grps;

    uint32_t machine_irq;

    MemoryRegion bar[PCI_NUM_REGIONS - 1];
    MemoryRegion rom;

    MemoryListener memory_listener;
};

int xen_pt_config_init(XenPCIPassthroughState *s);
void xen_pt_config_delete(XenPCIPassthroughState *s);
XenPTRegGroup *xen_pt_find_reg_grp(XenPCIPassthroughState *s, uint32_t address);
XenPTReg *xen_pt_find_reg(XenPTRegGroup *reg_grp, uint32_t address);
int xen_pt_bar_offset_to_index(uint32_t offset);

static inline pcibus_t xen_pt_get_emul_size(XenPTBarFlag flag, pcibus_t r_size)
{
    /* align resource size (memory type only) */
    if (flag == XEN_PT_BAR_FLAG_MEM) {
        return (r_size + XC_PAGE_SIZE - 1) & XC_PAGE_MASK;
    } else {
        return r_size;
    }
}

/* INTx */
/* The PCI Local Bus Specification, Rev. 3.0,
 * Section 6.2.4 Miscellaneous Registers, pp 223
 * outlines 5 valid values for the interrupt pin (intx).
 *  0: For devices (or device functions) that don't use an interrupt in
 *  1: INTA#
 *  2: INTB#
 *  3: INTC#
 *  4: INTD#
 *
 * Xen uses the following 4 values for intx
 *  0: INTA#
 *  1: INTB#
 *  2: INTC#
 *  3: INTD#
 *
 * Observing that these list of values are not the same, xen_pt_pci_read_intx()
 * uses the following mapping from hw to xen values.
 * This seems to reflect the current usage within Xen.
 *
 * PCI hardware    | Xen | Notes
 * ----------------+-----+----------------------------------------------------
 * 0               | 0   | No interrupt
 * 1               | 0   | INTA#
 * 2               | 1   | INTB#
 * 3               | 2   | INTC#
 * 4               | 3   | INTD#
 * any other value | 0   | This should never happen, log error message
 */

static inline uint8_t xen_pt_pci_read_intx(XenPCIPassthroughState *s)
{
    uint8_t v = 0;
    xen_host_pci_get_byte(&s->real_device, PCI_INTERRUPT_PIN, &v);
    return v;
}

static inline uint8_t xen_pt_pci_intx(XenPCIPassthroughState *s)
{
    uint8_t r_val = xen_pt_pci_read_intx(s);

    XEN_PT_LOG(&s->dev, "intx=%i\n", r_val);
    if (r_val < 1 || r_val > 4) {
        XEN_PT_LOG(&s->dev, "Interrupt pin read from hardware is out of range:"
                   " value=%i, acceptable range is 1 - 4\n", r_val);
        r_val = 0;
    } else {
        r_val -= 1;
    }

    return r_val;
}

#endif /* !XEN_PT_H */
