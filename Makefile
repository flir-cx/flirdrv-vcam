ifeq ($(KERNEL_SRC),)
	KERNEL_SRC ?= ~/linux/flir-yocto/build_pico/tmp-eglibc/work/neco-oe-linux-gnueabi/linux-boundary/3.0.35-r0/git
endif

ifneq ($(KERNEL_PATH),)
       KERNEL_SRC = $(KERNEL_PATH)
endif

ifdef DEBUG
	DEBUG = -DDEBUG
endif

ifdef NOWERRROR
	ERROR=
else
	ERROR=-Werror
endif

ifeq ($(INCLUDE_SRC),)
       INCLUDE_SRC ?=$(ALPHAREL)/SDK/FLIR/Include
endif

EXTRA_CFLAGS = -I$(INCLUDE_SRC)
#EXTRA_CFLAGS = -I$(INCLUDE_SRC) $(DEBUG) $(ERROR)

obj-m := vcam.o
vcam-objs += vcamd.o

#ifndef CONFIG_OF
# vcam-objs += MT9P111.o
# vcam-objs += OV7740.o
# vcam-objs += vcam_pico.o
# vcam-objs += vcam_neco.o
vcam-objs += vcam_roco.o
#endif

#ifdef CONFIG_OF
vcam-objs += vcam_evco.o
vcam-objs += vcam_eoco.o
vcam-objs += OV5640.o
#endif

SRC := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) clean

