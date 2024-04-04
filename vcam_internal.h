/* SPDX-License-Identifier: GPL-2.0-or-later */
/***********************************************************************
 *
 * Project: Balthazar
 *
 * Description of file:
 *   Visual Camera Driver Header file
 *
 * Copyright: FLIR Systems AB
 ***********************************************************************/
#ifndef _VCAM_INTERNAL_H_
#define _VCAM_INTERNAL_H_

#include "vcam_ioctl.h"
#include "flir_kernel_os.h"
#include <linux/miscdevice.h>

enum sensor_model {
	OV5640_STANDARD,
	OV5640_HIGH_K
};

// this structure keeps track of the device instance
struct vcam_ops {
	// Function pointers
	DWORD(*get_torchstate) (struct device *dev, VCAMIOCTLFLASH *pFlashData);
	DWORD(*set_torchstate) (struct device *dev, VCAMIOCTLFLASH *pFlashData);
	void (*set_power)(struct device *dev, bool enable);
	int (*do_iocontrol)(struct device *dev, int cmd, PUCHAR buf, PUCHAR userbuf);
	DWORD (*deinitialize_hw)(struct device *dev);
};

struct vcam_data {
	struct vcam_ops ops;
	struct miscdevice miscdev;
	struct device *dev;
	int i2c_address;
	struct i2c_adapter *i2c_bus;
	enum sensor_model sensor_model;
	struct work_struct nightmode_work;
	int flipped_sensor;	//if true the sensor is mounted upside/down.
	int edge_enhancement;	//enable increased edge enhancement in camera sensor

	int pwdn_gpio;
	int reset_gpio;
	int clk_en_gpio;

	struct regulator *reg_vcm1i2c;
	struct regulator *reg_vcm2i2c;
	struct regulator *reg_vcm;

	struct semaphore sem;	// serialize access to this device's state
};

DWORD PlatformInitHW(struct device *dev);

#endif //_VCAM_INTERNAL_H_
