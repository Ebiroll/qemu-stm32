#ifndef _QEMU_VIRTIO_INPUT_H
#define _QEMU_VIRTIO_INPUT_H

#include "ui/input.h"

/* ----------------------------------------------------------------- */
/* virtio input protocol                                             */

#include "standard-headers/linux/virtio_ids.h"
#include "standard-headers/linux/virtio_input.h"

typedef struct virtio_input_absinfo virtio_input_absinfo;
typedef struct virtio_input_config virtio_input_config;
typedef struct virtio_input_event virtio_input_event;

#if defined(HOST_WORDS_BIGENDIAN)
# define const_le32(_x) bswap32(_x)
# define const_le16(_x) bswap32(_x)
#else
# define const_le32(_x) (_x)
# define const_le16(_x) (_x)
#endif

/* ----------------------------------------------------------------- */
/* qemu internals                                                    */

#define TYPE_VIRTIO_INPUT "virtio-input-device"
#define VIRTIO_INPUT(obj) \
        OBJECT_CHECK(VirtIOInput, (obj), TYPE_VIRTIO_INPUT)
#define VIRTIO_INPUT_GET_PARENT_CLASS(obj) \
        OBJECT_GET_PARENT_CLASS(obj, TYPE_VIRTIO_INPUT)
#define VIRTIO_INPUT_GET_CLASS(obj) \
        OBJECT_GET_CLASS(VirtIOInputClass, obj, TYPE_VIRTIO_INPUT)
#define VIRTIO_INPUT_CLASS(klass) \
        OBJECT_CLASS_CHECK(VirtIOInputClass, klass, TYPE_VIRTIO_INPUT)

typedef struct VirtIOInput VirtIOInput;
typedef struct VirtIOInputClass VirtIOInputClass;
typedef struct VirtIOInputConfig VirtIOInputConfig;

struct virtio_input_conf {
    char *serial;
};

struct VirtIOInputConfig {
    virtio_input_config               config;
    QTAILQ_ENTRY(VirtIOInputConfig)   node;
};

struct VirtIOInput {
    VirtIODevice                      parent_obj;
    uint8_t                           cfg_select;
    uint8_t                           cfg_subsel;
    uint32_t                          cfg_size;
    QTAILQ_HEAD(, VirtIOInputConfig)  cfg_list;
    VirtQueue                         *evt, *sts;
    virtio_input_conf                 input;

    virtio_input_event                *queue;
    uint32_t                          qindex, qsize;

    bool                              active;
};

struct VirtIOInputClass {
    /*< private >*/
    VirtioDeviceClass parent;
    /*< public >*/

    DeviceRealize realize;
    DeviceUnrealize unrealize;
    void (*change_active)(VirtIOInput *vinput);
    void (*handle_status)(VirtIOInput *vinput, virtio_input_event *event);
};

void virtio_input_send(VirtIOInput *vinput, virtio_input_event *event);
void virtio_input_init_config(VirtIOInput *vinput,
                              virtio_input_config *config);
void virtio_input_add_config(VirtIOInput *vinput,
                             virtio_input_config *config);
void virtio_input_idstr_config(VirtIOInput *vinput,
                               uint8_t select, const char *string);

#endif /* _QEMU_VIRTIO_INPUT_H */
