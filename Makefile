# Enable kbuild verbose mode:
# $ make V=1

KSRC ?= /lib/modules/$(shell uname -r)/build
KMOD ?= /lib/modules/$(shell uname -r)

EXTRA_CFLAGS += -Wall
DIST_VERSION = $(shell grep MODULE_VERSION shuttle_leds.c | cut -d\" -f2)

CONFIG_SHUTTLE_VFD := m

obj-$(CONFIG_SHUTTLE_VFD)	+= shuttle_leds.o

all:
	$(MAKE) -C $(KSRC) M=$(PWD) modules

clean:
	$(MAKE) -C $(KSRC) M=$(PWD) clean

distclean: clean
	rm -rf cscope.* *~

dist: Makefile shuttle_vfd.c
	@tar -zcf shuttle_leds_driver-$(DIST_VERSION).tar.gz $?
cp:
	-cp $(obj-m:.o=.ko) $(KMOD)/misc

