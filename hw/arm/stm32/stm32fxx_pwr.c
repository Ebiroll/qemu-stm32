


#if 0


static uint64_t stm32fxx_powermgt_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    STM32PowerMgtState *s = (STM32PowerMgtState *)opaque;
    uint32_t res = 0;

    switch (offset) {
          case 0x0:
            res = s->cfg;
            static char bit_num=0;
            res= res | 1 |  (16384  |  (1 << bit_num));
            bit_num = bit_num +1;
            if (bit_num == 32) {
                bit_num=0;
            }

            qemu_log_mask(LOG_UNIMP,
                "stm32fxx_powermgt_read: cfg offset 0x%08"HWADDR_PRIx
                " %d\n", offset,res);
                break;
                // RCC_FLAG_HSERDY
        case 0x00000074:
                //s->csr = (1 << 0x61);
                res = s->csr;

                static char scr_bit_num=0;
                res= (res | (1 << scr_bit_num));

                scr_bit_num = scr_bit_num +1;
                if (scr_bit_num == 32) {
                    scr_bit_num=0;
                }

                qemu_log_mask(LOG_UNIMP,
                    "stm32fxx_powermgt_read: HSI-STATUS offset 0x%08"HWADDR_PRIx
                    " %d\n", offset,res);
                break;

            case 0x10:
                qemu_log_mask(LOG_UNIMP,
                        "stm32fxx_powermgt_read: cfgr 0x%08"HWADDR_PRIx
                        "\n", offset);
            res= s->cfg2;
          /*
             <name>SVOS</name>
              <description>System Stop mode voltage scaling
              selection These bits control the VCORE voltage level
              in system Stop mode, to obtain the best trade-off
              between power consumption and
              performance.</description>
              <bitOffset>14</bitOffset>
              <bitWidth>2</bitWidth>
              */
          break;
           case 0x28:
              res = s->rpcsr;
           break;
          case 0xe8:
              res = s->hsem_lock;
           break;


          case 0x00000418:
          /*
              voltage levels ready bit for currently
              used VOS and SDLEVEL This bit is set to 1 by hardware
              when the voltage regulator and the SD converter are
              both disabled and Bypass mode is selected in PWR
              control register 3 (PWR_CR3)
          */
            res=8192;
          break;

    default:
        qemu_log_mask(LOG_UNIMP,
                      "stm32fxx_powermgt_read: Unknown offset 0x%08"HWADDR_PRIx
                      "\n", offset);
        res = 0;
        break;
    }

    return res;
}

static void stm32fxx_powermgt_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    STM32PowerMgtState *s = (STM32PowerMgtState *)opaque;


    switch (offset) {
        case 0:
            qemu_log_mask(LOG_UNIMP,
            "stm32fxx_powermgt_write: CFG, offset 0x%08"HWADDR_PRIx
            " %d\n", offset,value);
            if (value & (1 << 24)) {
                       qemu_log_mask(LOG_UNIMP,
                      "stm32fxx_powermgt_write: PLL ON 0x%08"HWADDR_PRIx
                      "\n", offset);
                      value= value | (1 << 25);
            } else {
                qemu_log_mask(LOG_UNIMP,
                "stm32fxx_powermgt_write: PLL OFF 0x%08"HWADDR_PRIx
                "\n", offset);
                value=  value & ~(1 << 25);
            }

        s->cfg = value;
        break;
        case 0x10:
        {
            uint64_t source= value & ~0xfffffff8;
            s->cfg2=(value & 0xfffffff8 ) | source * 8 ;
            qemu_log_mask(LOG_UNIMP,
            "stm32fxx_powermgt_write: CFGR, offset 0x%08"HWADDR_PRIx
            " %d\n", offset,value);
            qemu_log_mask(LOG_UNIMP,
            "stm32fxx_powermgt_write: CFGR, offset 0x%08"HWADDR_PRIx
            " %d\n", offset,source);
        }
        break;

        /* 0x58024400
        // HSEM Read lock register
        stm32fxx_powermgt_write: Unknown offset 0x000000e8
        stm32fxx_powermgt_write: Unknown offset 0x000000e8


        RCC PLLs Clock Source Selection
        <resetValue>0x02020200</resetValue>
*/
    case 0xe8:
        s->hsem_lock = value;
        break;


    case 0x28:
        s->rpcsr = value;
        break;

    case 0x00000074:
        s->csr = value;
        qemu_log_mask(LOG_UNIMP,
            "stm32fxx_powermgt_write: RCC Clock Control and Status, offset 0x%08"HWADDR_PRIx
            " %d\n", offset,value);

        break;
    //case R_WDOG:
    //    qemu_log_mask(LOG_UNIMP,
    //                  "stm32fxx_powermgt_write: WDOG\n");
    //    s->wdog = value;
    //    break;

    default:
        qemu_log_mask(LOG_UNIMP,
                      "stm32fxx_powermgt_write: Unknown offset 0x%08"HWADDR_PRIx
                      "\n", offset);
        break;
    }
}

static const MemoryRegionOps stm32fxx_powermgt_ops = {
    .read = stm32fxx_powermgt_read,
    .write = stm32fxx_powermgt_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static const VMStateDescription vmstate_stm32fxx_powermgt = {
    .name = TYPE_STM32H7A3VIH_POWERMGT,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(cfg, STM32PowerMgtState),
        VMSTATE_END_OF_LIST()
    }
};

static void stm32fxx_powermgt_init(Object *obj)
{
    STM32PowerMgtState *s = STM32H7A3VIH_POWERMGT(obj);

    memory_region_init_io(&s->iomem, obj, &stm32fxx_powermgt_ops, s,
                          TYPE_STM32H7A3VIH_POWERMGT, 0xc00);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

static void stm32fxx_powermgt_reset(DeviceState *dev)
{
    STM32PowerMgtState *s = STM32H7A3VIH_POWERMGT(dev);

    s->cfg = 0x00000000;
    s->rpcsr = 0x02020200;
    s->hsem_lock=0;
}

static void stm32fxx_powermgt_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = stm32fxx_powermgt_reset;
    dc->vmsd = &vmstate_stm32fxx_powermgt;
}

static const TypeInfo stm32fxx_powermgt_info = {
    .name          = TYPE_STM32H7A3VIH_POWERMGT,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32PowerMgtState),
    .class_init    = stm32fxx_powermgt_class_init,
    .instance_init = stm32fxx_powermgt_init,
};

static void stm32fxx_powermgt_register_types(void)
{
    type_register_static(&stm32fxx_powermgt_info);
}

type_init(stm32fxx_powermgt_register_types)

#endif