#ifndef HW_I8257_H
#define HW_I8257_H

#define TYPE_I8257 "i8257"

typedef struct I8257Regs {
    int now[2];
    uint16_t base[2];
    uint8_t mode;
    uint8_t page;
    uint8_t pageh;
    uint8_t dack;
    uint8_t eop;
    DMA_transfer_handler transfer_handler;
    void *opaque;
} I8257Regs;

typedef struct I8257State {
    /* <private> */
    ISADevice parent_obj;

    /* <public> */
    int32_t base;
    int32_t page_base;
    int32_t pageh_base;
    int32_t dshift;

    uint8_t status;
    uint8_t command;
    uint8_t mask;
    uint8_t flip_flop;
    I8257Regs regs[4];
    MemoryRegion channel_io;
    MemoryRegion cont_io;

    QEMUBH *dma_bh;
    bool dma_bh_scheduled;
    int running;
} I8257State;

#endif

