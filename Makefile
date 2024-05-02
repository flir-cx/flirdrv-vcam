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
vcam-objs += vcam_platform.o
vcam-objs += ov5640.o

SRC := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) clean

