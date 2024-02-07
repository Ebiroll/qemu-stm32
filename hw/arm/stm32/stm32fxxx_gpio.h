#pragma once 
#include "hw/arm/stm32fxxx.h"

#define STM32_NUM_GPIOS 5

#define GPIO_TRACE(fmt, ...) qemu_log_mask(LOG_TRACE, "stm32fxxx_gpio: " fmt, ##__VA_ARGS__)
#define GPIO_ERROR(fmt, ...) qemu_log_mask(LOG_TRACE, "stm32fxxx_gpio: ERROR: " fmt, ##__VA_ARGS__)


#define TYPE_STM32FXXX_GPIO "stm32fxxx-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(stm32fxxx_gpio, STM32FXXX_GPIO)

typedef  struct stm32fxxx_gpio_state_t {
        union {
            uint32_t regs[10];
            struct {
                uint32_t MODER;
                uint32_t OTYPER;
                uint32_t OSPEEDR;
                uint32_t PUPDR;
                uint32_t IDR;
                uint32_t ODR;
                uint32_t BSRR;
                uint32_t LCKR;
                uint32_t AFRL;
                uint32_t AFRH;
            };
        };
} stm32fxxx_gpio_state_t;

struct stm32fxxx_gpio {
    SysBusDevice parent;

    MemoryRegion mmio;
    qemu_irq irq;

    uint8_t port_id, _port_id;

    //struct stm32fxxx_state *state;
    stm32fxxx_gpio_state_t GPIO;
    qemu_irq pins[32];
    //struct stm32fxxx_gpio_state *regs;
};

