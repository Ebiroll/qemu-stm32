#include "usb.h"
#include "usb-desc.h"
#include "trace.h"

/* ------------------------------------------------------------------ */

static uint8_t usb_lo(uint16_t val)
{
    return val & 0xff;
}

static uint8_t usb_hi(uint16_t val)
{
    return (val >> 8) & 0xff;
}

int usb_desc_device(const USBDescID *id, const USBDescDevice *dev,
                    uint8_t *dest, size_t len)
{
    uint8_t bLength = 0x12;

    if (len < bLength) {
        return -1;
    }

    dest[0x00] = bLength;
    dest[0x01] = USB_DT_DEVICE;

    dest[0x02] = usb_lo(dev->bcdUSB);
    dest[0x03] = usb_hi(dev->bcdUSB);
    dest[0x04] = dev->bDeviceClass;
    dest[0x05] = dev->bDeviceSubClass;
    dest[0x06] = dev->bDeviceProtocol;
    dest[0x07] = dev->bMaxPacketSize0;

    dest[0x08] = usb_lo(id->idVendor);
    dest[0x09] = usb_hi(id->idVendor);
    dest[0x0a] = usb_lo(id->idProduct);
    dest[0x0b] = usb_hi(id->idProduct);
    dest[0x0c] = usb_lo(id->bcdDevice);
    dest[0x0d] = usb_hi(id->bcdDevice);
    dest[0x0e] = id->iManufacturer;
    dest[0x0f] = id->iProduct;
    dest[0x10] = id->iSerialNumber;

    dest[0x11] = dev->bNumConfigurations;

    return bLength;
}

int usb_desc_config(const USBDescConfig *conf, uint8_t *dest, size_t len)
{
    uint8_t  bLength = 0x09;
    uint16_t wTotalLength = 0;
    int i, rc, count;

    if (len < bLength) {
        return -1;
    }

    dest[0x00] = bLength;
    dest[0x01] = USB_DT_CONFIG;
    dest[0x04] = conf->bNumInterfaces;
    dest[0x05] = conf->bConfigurationValue;
    dest[0x06] = conf->iConfiguration;
    dest[0x07] = conf->bmAttributes;
    dest[0x08] = conf->bMaxPower;
    wTotalLength += bLength;

    count = conf->nif ? conf->nif : conf->bNumInterfaces;
    for (i = 0; i < count; i++) {
        rc = usb_desc_iface(conf->ifs + i, dest + wTotalLength, len - wTotalLength);
        if (rc < 0) {
            return rc;
        }
        wTotalLength += rc;
    }

    dest[0x02] = usb_lo(wTotalLength);
    dest[0x03] = usb_hi(wTotalLength);
    return wTotalLength;
}

int usb_desc_iface(const USBDescIface *iface, uint8_t *dest, size_t len)
{
    uint8_t bLength = 0x09;
    int i, rc, pos = 0;

    if (len < bLength) {
        return -1;
    }

    dest[0x00] = bLength;
    dest[0x01] = USB_DT_INTERFACE;
    dest[0x02] = iface->bInterfaceNumber;
    dest[0x03] = iface->bAlternateSetting;
    dest[0x04] = iface->bNumEndpoints;
    dest[0x05] = iface->bInterfaceClass;
    dest[0x06] = iface->bInterfaceSubClass;
    dest[0x07] = iface->bInterfaceProtocol;
    dest[0x08] = iface->iInterface;
    pos += bLength;

    for (i = 0; i < iface->ndesc; i++) {
        rc = usb_desc_other(iface->descs + i, dest + pos, len - pos);
        if (rc < 0) {
            return rc;
        }
        pos += rc;
    }

    for (i = 0; i < iface->bNumEndpoints; i++) {
        rc = usb_desc_endpoint(iface->eps + i, dest + pos, len - pos);
        if (rc < 0) {
            return rc;
        }
        pos += rc;
    }

    return pos;
}

int usb_desc_endpoint(const USBDescEndpoint *ep, uint8_t *dest, size_t len)
{
    uint8_t bLength = 0x07;

    if (len < bLength) {
        return -1;
    }

    dest[0x00] = bLength;
    dest[0x01] = USB_DT_ENDPOINT;
    dest[0x02] = ep->bEndpointAddress;
    dest[0x03] = ep->bmAttributes;
    dest[0x04] = usb_lo(ep->wMaxPacketSize);
    dest[0x05] = usb_hi(ep->wMaxPacketSize);
    dest[0x06] = ep->bInterval;

    return bLength;
}

int usb_desc_other(const USBDescOther *desc, uint8_t *dest, size_t len)
{
    int bLength = desc->length ? desc->length : desc->data[0];

    if (len < bLength) {
        return -1;
    }

    memcpy(dest, desc->data, bLength);
    return bLength;
}

/* ------------------------------------------------------------------ */

void usb_desc_set_string(USBDevice *dev, uint8_t index, const char *str)
{
    USBDescString *s;

    QLIST_FOREACH(s, &dev->strings, next) {
        if (s->index == index) {
            break;
        }
    }
    if (s == NULL) {
        s = qemu_mallocz(sizeof(*s));
        s->index = index;
        QLIST_INSERT_HEAD(&dev->strings, s, next);
    }
    qemu_free(s->str);
    s->str = qemu_strdup(str);
}

const char *usb_desc_get_string(USBDevice *dev, uint8_t index)
{
    USBDescString *s;

    QLIST_FOREACH(s, &dev->strings, next) {
        if (s->index == index) {
            return s->str;
        }
    }
    return NULL;
}

int usb_desc_string(USBDevice *dev, int index, uint8_t *dest, size_t len)
{
    uint8_t bLength, pos, i;
    const char *str;

    if (len < 4) {
        return -1;
    }

    if (index == 0) {
        /* language ids */
        dest[0] = 4;
        dest[1] = USB_DT_STRING;
        dest[2] = 0x09;
        dest[3] = 0x04;
        return 4;
    }

    str = usb_desc_get_string(dev, index);
    if (str == NULL) {
        str = dev->info->usb_desc->str[index];
        if (str == NULL) {
            return 0;
        }
    }

    bLength = strlen(str) * 2 + 2;
    dest[0] = bLength;
    dest[1] = USB_DT_STRING;
    i = 0; pos = 2;
    while (pos+1 < bLength && pos+1 < len) {
        dest[pos++] = str[i++];
        dest[pos++] = 0;
    }
    return pos;
}

int usb_desc_get_descriptor(USBDevice *dev, int value, uint8_t *dest, size_t len)
{
    const USBDesc *desc = dev->info->usb_desc;
    uint8_t buf[256];
    uint8_t type = value >> 8;
    uint8_t index = value & 0xff;
    int ret = -1;

    switch(type) {
    case USB_DT_DEVICE:
        ret = usb_desc_device(&desc->id, desc->full, buf, sizeof(buf));
        trace_usb_desc_device(dev->addr, len, ret);
        break;
    case USB_DT_CONFIG:
        if (index < desc->full->bNumConfigurations) {
            ret = usb_desc_config(desc->full->confs + index, buf, sizeof(buf));
        }
        trace_usb_desc_config(dev->addr, index, len, ret);
        break;
    case USB_DT_STRING:
        ret = usb_desc_string(dev, index, buf, sizeof(buf));
        trace_usb_desc_string(dev->addr, index, len, ret);
        break;
    default:
        fprintf(stderr, "%s: %d unknown type %d (len %zd)\n", __FUNCTION__,
                dev->addr, type, len);
        break;
    }

    if (ret > 0) {
        if (ret > len) {
            ret = len;
        }
        memcpy(dest, buf, ret);
    }
    return ret;
}

int usb_desc_handle_control(USBDevice *dev, int request, int value,
                            int index, int length, uint8_t *data)
{
    const USBDesc *desc = dev->info->usb_desc;
    int ret = -1;

    assert(desc != NULL);
    switch(request) {
    case DeviceRequest | USB_REQ_GET_DESCRIPTOR:
        ret = usb_desc_get_descriptor(dev, value, data, length);
        break;
    }
    return ret;
}
