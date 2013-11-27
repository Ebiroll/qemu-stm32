#ifndef INPUT_H
#define INPUT_H

#include "qapi-types.h"

#define INPUT_EVENT_MASK_KEY   (1<<INPUT_EVENT_KIND_KEY)
#define INPUT_EVENT_MASK_BTN   (1<<INPUT_EVENT_KIND_BTN)
#define INPUT_EVENT_MASK_REL   (1<<INPUT_EVENT_KIND_REL)
#define INPUT_EVENT_MASK_ABS   (1<<INPUT_EVENT_KIND_ABS)

#define INPUT_EVENT_ABS_SIZE   0x8000

typedef struct QemuInputHandler QemuInputHandler;
typedef struct QemuInputHandlerState QemuInputHandlerState;

typedef void (*QemuInputHandlerEvent)(DeviceState *dev, QemuConsole *src,
                                      InputEvent *evt);
typedef void (*QemuInputHandlerSync)(DeviceState *dev);

struct QemuInputHandler {
    const char             *name;
    uint32_t               mask;
    QemuInputHandlerEvent  event;
    QemuInputHandlerSync   sync;
};

QemuInputHandlerState *qemu_input_handler_register(DeviceState *dev,
                                                   QemuInputHandler *handler);
void qemu_input_handler_activate(QemuInputHandlerState *s);
void qemu_input_handler_unregister(QemuInputHandlerState *s);
void qemu_input_event_send(QemuConsole *src, InputEvent *evt);
void qemu_input_event_sync(void);

InputEvent *qemu_input_event_new_key(KeyValue *key, bool down);
void qemu_input_event_send_key(QemuConsole *src, KeyValue *key, bool down);
void qemu_input_event_send_key_number(QemuConsole *src, int num, bool down);
void qemu_input_event_send_key_qcode(QemuConsole *src, QKeyCode q, bool down);

InputEvent *qemu_input_event_new_btn(InputButton btn, bool down);
void qemu_input_queue_btn(QemuConsole *src, InputButton btn, bool down);
void qemu_input_update_buttons(QemuConsole *src, uint32_t *button_map,
                               uint32_t button_old, uint32_t button_new);

int qemu_input_scale_axis(int value, int size_in, int size_out);
InputEvent *qemu_input_event_new_move(InputEventKind kind,
                                      InputAxis axis, int value);
void qemu_input_queue_rel(QemuConsole *src, InputAxis axis, int value);
void qemu_input_queue_abs(QemuConsole *src, InputAxis axis,
                          int value, int size);

#endif /* INPUT_H */
