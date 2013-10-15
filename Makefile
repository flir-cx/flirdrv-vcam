
# typically use the following to compile
# make ARCH=arm CROSS_COMPILE=/home/fredrik/mentor/arm-2011.03/bin/arm-none-linux-gnueabi
#
# Also modify 'KERNELDIR' to fit your system

EXTRA_CFLAGS = -I$(ALPHAREL)/SDK/FLIR/Include

	obj-m := vcam.o
	vcam-objs += vcamd.o
	vcam-objs += MT9P111.o
	vcam-objs += bspvcam.o
	
	KERNELDIR ?= /home/pfitger/linux-2.6-imx
	PWD := $(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

