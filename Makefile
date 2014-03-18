ifeq ($(KERNEL_SRC),)
	KERNEL_SRC ?= ~/linux/flir-yocto/build_pico/tmp-eglibc/work/neco-oe-linux-gnueabi/linux-boundary/3.0.35-r0/git
endif

EXTRA_CFLAGS = -I$(ALPHAREL)/SDK/FLIR/Include

obj-m := vcam.o
vcam-objs += vcamd.o
vcam-objs += MT9P111.o
vcam-objs += OV7740.o
vcam-objs += vcam_pico.o
vcam-objs += vcam_neco.o

SRC := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) clean

