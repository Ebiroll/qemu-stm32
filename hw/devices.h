#ifndef QEMU_DEVICES_H
#define QEMU_DEVICES_H

/* Devices that have nowhere better to go.  */

/* smc91c111.c */
void smc91c111_init(NICInfo *, uint32_t, qemu_irq);

/* ssd0323.c */
int ssd0323_xfer_ssi(void *opaque, int data);
void *ssd0323_init(DisplayState *ds, qemu_irq *cmd_p);

/* ads7846.c */
struct ads7846_state_s;
uint32_t ads7846_read(void *opaque);
void ads7846_write(void *opaque, uint32_t value);
struct ads7846_state_s *ads7846_init(qemu_irq penirq);

/* tsc210x.c */
struct uwire_slave_s;
struct mouse_transform_info_s;
struct uwire_slave_s *tsc2102_init(qemu_irq pint, AudioState *audio);
struct uwire_slave_s *tsc2301_init(qemu_irq penirq, qemu_irq kbirq,
                qemu_irq dav, AudioState *audio);
struct i2s_codec_s *tsc210x_codec(struct uwire_slave_s *chip);
uint32_t tsc210x_txrx(void *opaque, uint32_t value);
void tsc210x_set_transform(struct uwire_slave_s *chip,
                struct mouse_transform_info_s *info);
void tsc210x_key_event(struct uwire_slave_s *chip, int key, int down);

/* stellaris_input.c */
void stellaris_gamepad_init(int n, qemu_irq *irq, const int *keycode);

/* blizzard.c */
void *s1d13745_init(qemu_irq gpio_int, DisplayState *ds);
void s1d13745_write(void *opaque, int dc, uint16_t value);
void s1d13745_write_block(void *opaque, int dc,
                void *buf, size_t len, int pitch);
uint16_t s1d13745_read(void *opaque, int dc);

/* cbus.c */
struct cbus_s {
    qemu_irq clk;
    qemu_irq dat;
    qemu_irq sel;
};
struct cbus_s *cbus_init(qemu_irq dat_out);
void cbus_attach(struct cbus_s *bus, void *slave_opaque);

void *retu_init(qemu_irq irq, int vilma);
void *tahvo_init(qemu_irq irq, int betty);

void retu_key_event(void *retu, int state);

/* tusb6010.c */
struct tusb_s;
struct tusb_s *tusb6010_init(qemu_irq intr);
int tusb6010_sync_io(struct tusb_s *s);
int tusb6010_async_io(struct tusb_s *s);
void tusb6010_power(struct tusb_s *s, int on);

#endif
