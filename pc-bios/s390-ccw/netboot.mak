
SLOF_DIR := $(SRC_PATH)/roms/SLOF

NETOBJS := start.o sclp.o virtio.o virtio-net.o netmain.o libc.a

LIBC_INC := -nostdinc -I$(SLOF_DIR)/lib/libc/include
LIBNET_INC := -I$(SLOF_DIR)/lib/libnet

NETLDFLAGS := $(LDFLAGS) -Ttext=0x7800000

$(NETOBJS): QEMU_CFLAGS += $(LIBC_INC) $(LIBNET_INC)

s390-netboot.elf: $(NETOBJS)
	$(call quiet-command,$(CC) $(NETLDFLAGS) -o $@ $(NETOBJS),"BUILD","$(TARGET_DIR)$@")

s390-netboot.img: s390-netboot.elf
	$(call quiet-command,$(STRIP) --strip-unneeded $< -o $@,"STRIP","$(TARGET_DIR)$@")

# libc files:

LIBC_CFLAGS :=  $(QEMU_CFLAGS) $(LIBC_INC) $(LIBNET_INC)

CTYPE_OBJS = isdigit.o isxdigit.o toupper.o
%.o : $(SLOF_DIR)/lib/libc/ctype/%.c
	$(call quiet-command,$(CC) $(LIBC_CFLAGS) -c -o $@ $<,"CC","$(TARGET_DIR)$@")

STRING_OBJS = strcat.o strchr.o strcmp.o strcpy.o strlen.o strncmp.o strncpy.o \
	      strstr.o memset.o memcpy.o memmove.o memcmp.o
%.o : $(SLOF_DIR)/lib/libc/string/%.c
	$(call quiet-command,$(CC) $(LIBC_CFLAGS) -c -o $@ $<,"CC","$(TARGET_DIR)$@")

STDLIB_OBJS = atoi.o atol.o strtoul.o strtol.o rand.o malloc.o free.o
%.o : $(SLOF_DIR)/lib/libc/stdlib/%.c
	$(call quiet-command,$(CC) $(LIBC_CFLAGS) -c -o $@ $<,"CC","$(TARGET_DIR)$@")

STDIO_OBJS = sprintf.o vfprintf.o vsnprintf.o vsprintf.o fprintf.o \
	     printf.o putc.o puts.o putchar.o stdchnls.o fileno.o
%.o : $(SLOF_DIR)/lib/libc/stdio/%.c
	$(call quiet-command,$(CC) $(LIBC_CFLAGS) -c -o $@ $<,"CC","$(TARGET_DIR)$@")

sbrk.o: $(SLOF_DIR)/slof/sbrk.c
	$(call quiet-command,$(CC) $(LIBC_CFLAGS) -c -o $@ $<,"CC","$(TARGET_DIR)$@")

LIBCOBJS := $(STRING_OBJS) $(CTYPE_OBJS) $(STDLIB_OBJS) $(STDIO_OBJS) sbrk.o

libc.a: $(LIBCOBJS)
	$(call quiet-command,$(AR) -rc $@ $^,"AR","$(TARGET_DIR)$@")
