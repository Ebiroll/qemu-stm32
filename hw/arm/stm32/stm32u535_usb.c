
#include "qemu/osdep.h"
#include "stm32u535_usb.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"


/******************  Bits definition for USB_DRD_CNTR register  *******************/
#define USB_CNTR_HOST_Pos               (31U)
#define USB_CNTR_HOST_Msk               (0x1UL << USB_CNTR_HOST_Pos)    /*!< 0x80000000 */
#define USB_CNTR_HOST                   USB_CNTR_HOST_Msk               /*!< Host Mode  */
#define USB_CNTR_THR512M_Pos            (16U)
#define USB_CNTR_THR512M_Msk            (0x1UL << USB_CNTR_THR512M_Pos)  /*!< 0x00010000 */
#define USB_CNTR_THR512M                USB_CNTR_THR512M_Msk             /*!< 512byte Threshold interrupt mask */
#define USB_CNTR_CTRM_Pos               (15U)
#define USB_CNTR_CTRM_Msk               (0x1UL << USB_CNTR_CTRM_Pos)    /*!< 0x00008000 */
#define USB_CNTR_CTRM                   USB_CNTR_CTRM_Msk               /*!< Correct Transfer Mask */
#define USB_CNTR_PMAOVRM_Pos            (14U)
#define USB_CNTR_PMAOVRM_Msk            (0x1UL << USB_CNTR_PMAOVRM_Pos) /*!< 0x00004000 */
#define USB_CNTR_PMAOVRM                USB_CNTR_PMAOVRM_Msk            /*!< DMA OVeR/underrun Mask */
#define USB_CNTR_ERRM_Pos               (13U)
#define USB_CNTR_ERRM_Msk               (0x1UL << USB_CNTR_ERRM_Pos)    /*!< 0x00002000 */
#define USB_CNTR_ERRM                   USB_CNTR_ERRM_Msk               /*!< ERRor Mask */
#define USB_CNTR_WKUPM_Pos              (12U)
#define USB_CNTR_WKUPM_Msk              (0x1UL << USB_CNTR_WKUPM_Pos)   /*!< 0x00001000 */
#define USB_CNTR_WKUPM                  USB_CNTR_WKUPM_Msk              /*!< WaKe UP Mask */
#define USB_CNTR_SUSPM_Pos              (11U)
#define USB_CNTR_SUSPM_Msk              (0x1UL << USB_CNTR_SUSPM_Pos)   /*!< 0x00000800 */
#define USB_CNTR_SUSPM                  USB_CNTR_SUSPM_Msk              /*!< SUSPend Mask */
#define USB_CNTR_RESETM_Pos             (10U)
#define USB_CNTR_RESETM_Msk             (0x1UL << USB_CNTR_RESETM_Pos)  /*!< 0x00000400 */
#define USB_CNTR_RESETM                 USB_CNTR_RESETM_Msk             /*!< RESET Mask */
#define USB_CNTR_DCON                   USB_CNTR_RESETM_Msk             /*!< Disconnection Connection Mask */
#define USB_CNTR_SOFM_Pos               (9U)
#define USB_CNTR_SOFM_Msk               (0x1UL << USB_CNTR_SOFM_Pos)    /*!< 0x00000200 */
#define USB_CNTR_SOFM                   USB_CNTR_SOFM_Msk               /*!< Start Of Frame Mask */
#define USB_CNTR_ESOFM_Pos              (8U)
#define USB_CNTR_ESOFM_Msk              (0x1UL << USB_CNTR_ESOFM_Pos)   /*!< 0x00000100 */
#define USB_CNTR_ESOFM                  USB_CNTR_ESOFM_Msk              /*!< Expected Start Of Frame Mask */
#define USB_CNTR_L1REQM_Pos             (7U)
#define USB_CNTR_L1REQM_Msk             (0x1UL << USB_CNTR_L1REQM_Pos)  /*!< 0x00000080 */
#define USB_CNTR_L1REQM                 USB_CNTR_L1REQM_Msk             /*!< LPM L1 state request interrupt Mask */
#define USB_CNTR_L1XACT_Pos             (6U)
#define USB_CNTR_L1XACT_Msk             (0x1UL << USB_CNTR_L1XACT_Pos)  /*!< 0x00000040 */
#define USB_CNTR_L1XACT                 USB_CNTR_L1XACT_Msk             /*!< Host LPM L1 transaction request Mask */
#define USB_CNTR_L1RES_Pos              (5U)
#define USB_CNTR_L1RES_Msk              (0x1UL << USB_CNTR_L1RES_Pos)   /*!< 0x00000020 */
#define USB_CNTR_L1RES                  USB_CNTR_L1RES_Msk              /*!< LPM L1 Resume request/ Remote Wakeup Mask */
#define USB_CNTR_L2RES_Pos              (4U)
#define USB_CNTR_L2RES_Msk              (0x1UL << USB_CNTR_L2RES_Pos)   /*!< 0x00000010 */
#define USB_CNTR_L2RES                  USB_CNTR_L2RES_Msk              /*!< L2 Remote Wakeup / Resume driver Mask */
#define USB_CNTR_SUSPEN_Pos             (3U)
#define USB_CNTR_SUSPEN_Msk             (0x1UL << USB_CNTR_SUSPEN_Pos)  /*!< 0x00000008 */
#define USB_CNTR_SUSPEN                 USB_CNTR_SUSPEN_Msk             /*!< Suspend state enable Mask */
#define USB_CNTR_SUSPRDY_Pos            (2U)
#define USB_CNTR_SUSPRDY_Msk            (0x1UL << USB_CNTR_SUSPRDY_Pos) /*!< 0x00000004 */
#define USB_CNTR_SUSPRDY                USB_CNTR_SUSPRDY_Msk            /*!< Suspend state effective Mask */
#define USB_CNTR_PDWN_Pos               (1U)
#define USB_CNTR_PDWN_Msk               (0x1UL << USB_CNTR_PDWN_Pos)    /*!< 0x00000002 */
#define USB_CNTR_PDWN                   USB_CNTR_PDWN_Msk               /*!< Power DoWN Mask */
#define USB_CNTR_USBRST_Pos             (0U)
#define USB_CNTR_USBRST_Msk             (0x1UL << USB_CNTR_USBRST_Pos)  /*!< 0x00000001 */
#define USB_CNTR_USBRST                 USB_CNTR_USBRST_Msk             /*!< USB Reset Mask */


#define STM32U535_USB(obj) OBJECT_CHECK(STM32U535USBState, (obj), TYPE_STM32U535_USB)

// Forward declarations
static uint64_t stm32u535_usb_read(void *opaque, hwaddr offset, unsigned size);
static void stm32u535_usb_write(void *opaque, hwaddr offset, uint64_t value, unsigned size);

static const MemoryRegionOps stm32u535_usb_ops = {
    .read = stm32u535_usb_read,
    .write = stm32u535_usb_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void stm32u535_usb_reset(DeviceState *dev)
{
    STM32U535USBState *s = STM32U535_USB(dev);

    // Reset all registers to their default values
    memset(s->EP, 0, sizeof(s->EP));
    s->CNTR = 0;
    s->ISTR = 0;
    s->FNR = 0;
    s->DADDR = 0;
    // ... Reset other fields as needed ...
}

static uint64_t stm32u535_usb_read(void *opaque, hwaddr offset, unsigned size)
{
    STM32U535USBState *s = STM32U535_USB(opaque);
    uint64_t value = 0;

    switch (offset) {
    // Handle reading from various registers
    case USB_CNTR_OFFSET:
        value = s->CNTR;
        break;
    case USB_ISTR_OFFSET:
        value = s->ISTR;
        break;
    case USB_FNR_OFFSET:
        value = s->FNR;
        break;
    case USB_DADDR_OFFSET:
        value = s->DADDR;
        break;
    case USB_BCDR_OFFSET:
        value = s->BCDR;
        break;

    // ... Handle other offsets ...
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Invalid read offset 0x%x\n", __func__, (int)offset);
        break;
    }

    return value;
}

static void stm32u535_usb_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    STM32U535USBState *s = STM32U535_USB(opaque);

    switch (offset) {
    // Handle writing to various registers
    case USB_CNTR_OFFSET:
        s->CNTR = value;
        // ... Implement logic to handle CNTR register changes ...
        break;
    case USB_ISTR_OFFSET:
        s->ISTR = value;
        // ... Implement logic to handle ISTR register changes ...
        break;
    case USB_FNR_OFFSET:
        s->FNR = value;
        // ... Implement logic to handle FNR register changes ...
        break;
    case USB_DADDR_OFFSET:
        s->DADDR = value;
        // ... Implement logic to handle DADDR register changes ...
        break;

    case USB_BCDR_OFFSET:
        s->BCDR = value;
        break;

    // ... Handle other offsets ...
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Invalid write offset 0x%x\n", __func__, (int)offset);
        break;
    }

    // ... Add logic for handling USB data transfers and interrupts as needed ...
}

static void stm32u535_usb_init(Object *obj)
{
    STM32U535USBState *s = STM32U535_USB(obj);

    memory_region_init_io(&s->iomem, obj, &stm32u535_usb_ops, s, TYPE_STM32U535_USB, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);

    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
}

static void stm32u535_usb_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = stm32u535_usb_reset;
}

static TypeInfo stm32u535_usb_type_info = {
    .name = TYPE_STM32U535_USB,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32U535USBState),
    .instance_init = stm32u535_usb_init,
    .class_init = stm32u535_usb_class_init,
};

static void stm32u535_usb_register_types(void)
{
    type_register_static(&stm32u535_usb_type_info);
}

type_init(stm32u535_usb_register_types)
