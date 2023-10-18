/* SPDX-License-Identifier: GPL-2.0-or-later */

/***********************************************************************
 *
 * Project: Balthazar
 *
 * Description of file:
 *   Visual Camera Driver
 *
 * Copyright: FLIR Systems AB
 ***********************************************************************/

#include "flir_kernel_os.h"
#include "vcam_internal.h"
#include "i2cdev.h"
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include "OV5640.h"

static int ov5640_initmipicamera(struct device *dev, CAM_NO camera);
static int ov5640_initcsicamera(struct device *dev, CAM_NO camera);
static int ov5640_initcamera(struct device *dev, CAM_NO camera);

/* Definition of DT node name, construction is error prone, so avoiding some
 * error possibilities by using definition
 */
#define VCAM_PARALLELL_INTERFACE "vcam_parallell_interface"

static DWORD OV5640_mirror_enable(PCAM_HW_INDEP_INFO pInfo, CAM_NO camera, bool enable);
static void OV5640_autofocus_enable(PCAM_HW_INDEP_INFO pInfo, CAM_NO camera, bool enable);
static int OV5640_set_fov(PCAM_HW_INDEP_INFO pInfo, CAM_NO camera, int fov);
static void OV5640_Testpattern_Enable(struct device *dev, bool on);

#define OV5640_SETTING_HIGH_K_ELEMENTS 93
static struct reg_value ov5640_setting_High_K[OV5640_SETTING_HIGH_K_ELEMENTS] = {
	{ 0x5180, 0xff },
	{ 0x5181, 0xf2 },
	{ 0x5182, 0x0 },
	{ 0x5183, 0x14 },
	{ 0x5184, 0x25 },
	{ 0x5185, 0x24 },
	{ 0x5186, 0x10 },
	{ 0x5187, 0x10 },
	{ 0x5188, 0x10 },
	{ 0x5189, 0x6d },
	{ 0x518a, 0x53 },
	{ 0x518b, 0x90 },
	{ 0x518c, 0x8c },
	{ 0x518d, 0x3b },
	{ 0x518e, 0x2c },
	{ 0x518f, 0x59 },
	{ 0x5190, 0x42 },
	{ 0x5191, 0xf8 },
	{ 0x5192, 0x4 },
	{ 0x5193, 0x70 },
	{ 0x5194, 0xf0 },
	{ 0x5195, 0xf0 },
	{ 0x5196, 0x3 },
	{ 0x5197, 0x1 },
	{ 0x5198, 0x4 },
	{ 0x5199, 0x0 },
	{ 0x519a, 0x4 },
	{ 0x519b, 0x13 },
	{ 0x519c, 0x6 },
	{ 0x519d, 0x9e },
	{ 0x519e, 0x38 },
	{ 0x5800, 0x2a },
	{ 0x5801, 0x1a },
	{ 0x5802, 0x13 },
	{ 0x5803, 0x13 },
	{ 0x5804, 0x1b },
	{ 0x5805, 0x2b },
	{ 0x5806, 0x10 },
	{ 0x5807, 0x9 },
	{ 0x5808, 0x7 },
	{ 0x5809, 0x7 },
	{ 0x580a, 0xa },
	{ 0x580b, 0x10 },
	{ 0x580c, 0x8 },
	{ 0x580d, 0x3 },
	{ 0x580e, 0x0 },
	{ 0x580f, 0x0 },
	{ 0x5810, 0x4 },
	{ 0x5811, 0x9 },
	{ 0x5812, 0x9 },
	{ 0x5813, 0x3 },
	{ 0x5814, 0x0 },
	{ 0x5815, 0x0 },
	{ 0x5816, 0x4 },
	{ 0x5817, 0x9 },
	{ 0x5818, 0xd },
	{ 0x5819, 0x6 },
	{ 0x581a, 0x3 },
	{ 0x581b, 0x4 },
	{ 0x581c, 0x6 },
	{ 0x581d, 0xd },
	{ 0x581e, 0x21 },
	{ 0x581f, 0x11 },
	{ 0x5820, 0xa },
	{ 0x5821, 0xa },
	{ 0x5822, 0x13 },
	{ 0x5823, 0x23 },
	{ 0x5824, 0x13 },
	{ 0x5825, 0x24 },
	{ 0x5826, 0x24 },
	{ 0x5827, 0x22 },
	{ 0x5828, 0x4 },
	{ 0x5829, 0x10 },
	{ 0x582a, 0x22 },
	{ 0x582b, 0x22 },
	{ 0x582c, 0x22 },
	{ 0x582d, 0x22 },
	{ 0x582e, 0x10 },
	{ 0x582f, 0x22 },
	{ 0x5830, 0x42 },
	{ 0x5831, 0x22 },
	{ 0x5832, 0x22 },
	{ 0x5833, 0x10 },
	{ 0x5834, 0x22 },
	{ 0x5835, 0x22 },
	{ 0x5836, 0x22 },
	{ 0x5837, 0x0 },
	{ 0x5838, 0x12 },
	{ 0x5839, 0x12 },
	{ 0x583a, 0x10 },
	{ 0x583b, 0x10 },
	{ 0x583c, 0x2 },
	{ 0x583d, 0xce },
};


/*[11-14 17:20] Wistrand, Anders */
/* @@ MEG5_YUV 1.875fps */
/* 100 99 2592 1944 */
/* 100 98 0 00 */
/* 102 3601 bb */
/* ; */
/* ;OV5640 setting Version History */
/* ;dated 04/08/2010 A02 */
/* ;--Based on v08 release */
/* ; */
/* ;dated 04/20/2010 A03 */
/* ;--Based on V10 release */
/* ; */
/* ;dated 04/22/2010 A04 */
/* ;--Based on V10 release */
/* ;--updated ccr & awb setting */
/* ; */
/* ;dated 04/22/2010 A06 */
/* ;--Based on A05 release */
/* ;--Add pg setting */
/* ; */
/* ;dated 05/19/2011 A09 */
/* ;--changed pchg 3708 setting */
/* ; */
/* ;dated 07/06/2011 A10 */
/* ;--changed contrast setting */
/* ; */
/* ;dated 07/11/2013 A11 */
/* ;--change 3708/3709 align wt V15 */
#define OV5640_INIT_SETTING_5MP_ELEMENTS 260

static struct reg_value ov5640_init_setting_5MP[OV5640_INIT_SETTING_5MP_ELEMENTS] = {
	{ 0x3103, 0x11 },
	{ 0x3008, 0x82 },
	{ 0x3008, 0x42 },
	{ 0x3103, 0x03 },
	{ 0x3017, 0xff },
	{ 0x3018, 0xff },
	{ 0x3034, 0x1a },
	{ 0x3035, 0x21 },
	{ 0x3036, 0x69 },
	{ 0x3037, 0x13 },
	{ 0x3108, 0x01 },
	{ 0x3630, 0x36 },
	{ 0x3631, 0x0e },
	{ 0x3632, 0xe2 },
	{ 0x3633, 0x12 },
	{ 0x3621, 0xe0 },
	{ 0x3704, 0xa0 },
	{ 0x3703, 0x5a },
	{ 0x3715, 0x78 },
	{ 0x3717, 0x01 },
	{ 0x370b, 0x60 },
	{ 0x3705, 0x1a },
	{ 0x3905, 0x02 },
	{ 0x3906, 0x10 },
	{ 0x3901, 0x0a },
	{ 0x3731, 0x12 },
	{ 0x3600, 0x08 },
	{ 0x3601, 0x33 },
	{ 0x302d, 0x60 },
	{ 0x3620, 0x52 },
	{ 0x371b, 0x20 },
	{ 0x471c, 0x50 },
	{ 0x3a13, 0x43 },
	{ 0x3a18, 0x00 },
	{ 0x3a19, 0xf8 },
	{ 0x3635, 0x13 },
	{ 0x3636, 0x03 },
	{ 0x3634, 0x40 },
	{ 0x3622, 0x01 },
	{ 0x3c01, 0x34 },
	{ 0x3c04, 0x28 },
	{ 0x3c05, 0x98 },
	{ 0x3c06, 0x00 },
	{ 0x3c07, 0x07 },
	{ 0x3c08, 0x00 },
	{ 0x3c09, 0x1c },
	{ 0x3c0a, 0x9c },
	{ 0x3c0b, 0x40 },
	{ 0x3821, 0x06 },
	{ 0x3814, 0x11 },
	{ 0x3815, 0x11 },
	{ 0x3800, 0x00 },
	{ 0x3801, 0x00 },
	{ 0x3802, 0x00 },
	{ 0x3803, 0x00 },
	{ 0x3804, 0x0a },
	{ 0x3805, 0x3f },
	{ 0x3806, 0x07 },
	{ 0x3807, 0x9f },
	{ 0x3808, 0x0a },
	{ 0x3809, 0x20 },
	{ 0x380a, 0x07 },
	{ 0x380b, 0x98 },
	{ 0x380c, 0x0b },
	{ 0x380d, 0x1c },
	{ 0x380e, 0x07 },
	{ 0x380f, 0xb0 },
	{ 0x3810, 0x00 },
	{ 0x3811, 0x10 },
	{ 0x3812, 0x00 },
	{ 0x3813, 0x04 },
	{ 0x3618, 0x04 },
	{ 0x3612, 0x2b },
	{ 0x3708, 0x63 },
	{ 0x3709, 0x12 },
	{ 0x370c, 0x00 },
	{ 0x3a02, 0x07 },
	{ 0x3a03, 0xb0 },
	{ 0x3a08, 0x01 },
	{ 0x3a09, 0x27 },
	{ 0x3a0a, 0x00 },
	{ 0x3a0b, 0xf6 },
	{ 0x3a0e, 0x06 },
	{ 0x3a0d, 0x08 },
	{ 0x3a14, 0x07 },
	{ 0x3a15, 0xb0 },
	{ 0x4001, 0x02 },
	{ 0x4004, 0x06 },
	{ 0x4050, 0x6e },
	{ 0x4051, 0x8f },
	{ 0x3000, 0x00 },
	{ 0x3002, 0x1c },
	{ 0x3004, 0xff },
	{ 0x3006, 0xc3 },
	{ 0x300e, 0x58 },
	{ 0x302e, 0x00 },
	{ 0x4300, 0x30 },
	{ 0x4837, 0x2c },
	{ 0x501f, 0x00 },
	{ 0x5684, 0x0a },
	{ 0x5685, 0x20 },
	{ 0x5686, 0x07 },
	{ 0x5687, 0x98 },
	{ 0x440e, 0x00 },
	{ 0x5000, 0xa7 },
	{ 0x5001, 0x83 },
	{ 0x5180, 0xff },
	{ 0x5181, 0xf2 },
	{ 0x5182, 0x00 },
	{ 0x5183, 0x14 },
	{ 0x5184, 0x25 },
	{ 0x5185, 0x24 },
	{ 0x5186, 0x09 },
	{ 0x5187, 0x09 },
	{ 0x5188, 0x09 },
	{ 0x5189, 0x75 },
	{ 0x518a, 0x54 },
	{ 0x518b, 0xe0 },
	{ 0x518c, 0xb2 },
	{ 0x518d, 0x42 },
	{ 0x518e, 0x3d },
	{ 0x518f, 0x56 },
	{ 0x5190, 0x46 },
	{ 0x5191, 0xf8 },
	{ 0x5192, 0x04 },
	{ 0x5193, 0x70 },
	{ 0x5194, 0xf0 },
	{ 0x5195, 0xf0 },
	{ 0x5196, 0x03 },
	{ 0x5197, 0x01 },
	{ 0x5198, 0x04 },
	{ 0x5199, 0x12 },
	{ 0x519a, 0x04 },
	{ 0x519b, 0x00 },
	{ 0x519c, 0x06 },
	{ 0x519d, 0x82 },
	{ 0x519e, 0x38 },
	{ 0x5381, 0x1e },
	{ 0x5382, 0x5b },
	{ 0x5383, 0x08 },
	{ 0x5384, 0x0a },
	{ 0x5385, 0x7e },
	{ 0x5386, 0x88 },
	{ 0x5387, 0x7c },
	{ 0x5388, 0x6c },
	{ 0x5389, 0x10 },
	{ 0x538a, 0x01 },
	{ 0x538b, 0x98 },
	{ 0x5300, 0x08 },
	{ 0x5301, 0x30 },
	{ 0x5302, 0x10 },
	{ 0x5303, 0x00 },
	{ 0x5304, 0x08 },
	{ 0x5305, 0x30 },
	{ 0x5306, 0x08 },
	{ 0x5307, 0x16 },
	{ 0x5309, 0x08 },
	{ 0x530a, 0x30 },
	{ 0x530b, 0x04 },
	{ 0x530c, 0x06 },
	{ 0x5480, 0x01 },
	{ 0x5481, 0x08 },
	{ 0x5482, 0x14 },
	{ 0x5483, 0x28 },
	{ 0x5484, 0x51 },
	{ 0x5485, 0x65 },
	{ 0x5486, 0x71 },
	{ 0x5487, 0x7d },
	{ 0x5488, 0x87 },
	{ 0x5489, 0x91 },
	{ 0x548a, 0x9a },
	{ 0x548b, 0xaa },
	{ 0x548c, 0xb8 },
	{ 0x548d, 0xcd },
	{ 0x548e, 0xdd },
	{ 0x548f, 0xea },
	{ 0x5490, 0x1d },
	{ 0x5580, 0x02 },
	{ 0x5583, 0x40 },
	{ 0x5584, 0x10 },
	{ 0x5589, 0x10 },
	{ 0x558a, 0x00 },
	{ 0x558b, 0xf8 },
	{ 0x5800, 0x23 },
	{ 0x5801, 0x14 },
	{ 0x5802, 0x0f },
	{ 0x5803, 0x0f },
	{ 0x5804, 0x12 },
	{ 0x5805, 0x26 },
	{ 0x5806, 0x0c },
	{ 0x5807, 0x08 },
	{ 0x5808, 0x05 },
	{ 0x5809, 0x05 },
	{ 0x580a, 0x08 },
	{ 0x580b, 0x0d },
	{ 0x580c, 0x08 },
	{ 0x580d, 0x03 },
	{ 0x580e, 0x00 },
	{ 0x580f, 0x00 },
	{ 0x5810, 0x03 },
	{ 0x5811, 0x09 },
	{ 0x5812, 0x07 },
	{ 0x5813, 0x03 },
	{ 0x5814, 0x00 },
	{ 0x5815, 0x01 },
	{ 0x5816, 0x03 },
	{ 0x5817, 0x08 },
	{ 0x5818, 0x0d },
	{ 0x5819, 0x08 },
	{ 0x581a, 0x05 },
	{ 0x581b, 0x06 },
	{ 0x581c, 0x08 },
	{ 0x581d, 0x0e },
	{ 0x581e, 0x29 },
	{ 0x581f, 0x17 },
	{ 0x5820, 0x11 },
	{ 0x5821, 0x11 },
	{ 0x5822, 0x15 },
	{ 0x5823, 0x28 },
	{ 0x5824, 0x46 },
	{ 0x5825, 0x26 },
	{ 0x5826, 0x08 },
	{ 0x5827, 0x26 },
	{ 0x5828, 0x64 },
	{ 0x5829, 0x26 },
	{ 0x582a, 0x24 },
	{ 0x582b, 0x22 },
	{ 0x582c, 0x24 },
	{ 0x582d, 0x24 },
	{ 0x582e, 0x06 },
	{ 0x582f, 0x22 },
	{ 0x5830, 0x40 },
	{ 0x5831, 0x42 },
	{ 0x5832, 0x24 },
	{ 0x5833, 0x26 },
	{ 0x5834, 0x24 },
	{ 0x5835, 0x22 },
	{ 0x5836, 0x22 },
	{ 0x5837, 0x26 },
	{ 0x5838, 0x44 },
	{ 0x5839, 0x24 },
	{ 0x583a, 0x26 },
	{ 0x583b, 0x28 },
	{ 0x583c, 0x42 },
	{ 0x583d, 0xce },
	{ 0x5025, 0x00 },
	{ 0x3a0f, 0x30 },
	{ 0x3a10, 0x28 },
	{ 0x3a1b, 0x30 },
	{ 0x3a1e, 0x26 },
	{ 0x3a11, 0x60 },
	{ 0x3a1f, 0x14 },
	{ 0x3008, 0x02 },
	{ 0x471d, 0x00 }, //DVP VSYNC CTRL
	{ 0x4740, 0x21 }, //DVP POLARITY CTRL00
	{ 0x4300, 0x32 }, //FORMAT CONTROL b4:3 == 3 -> YUV422 -> b3:0 == 0x0 -> YUYV format
	{ 0x501f, 0x00 }, //FORMAT MUX CONTROL (Foramt select, b2:0 == 0 -> ISP YUV422
	{ 0x3820, 0x47 }, //VFLIP
	{ 0x3035, 0x21 },
	{ 0x3821, 0x01 }, //MIRROR
};

/* Settings from
 * Rocky/Elektronik/Komponenter/datablad/VCam/Settings/
 */
#define OV5640_INIT_SETTING_9FPS_5MP_ELEMENTS 252
static struct reg_value ov5640_init_setting_9fps_5MP[OV5640_INIT_SETTING_9FPS_5MP_ELEMENTS] = {
	{ 0x3103, 0x11 },
	{ 0x3008, 0x82 },
	{ 0x3008, 0x42 },
	{ 0x3103, 0x03 },
	{ 0x3017, 0x00 },
	{ 0x3018, 0x00 },
	{ 0x3034, 0x18 },
	{ 0x3035, 0x11 },
	{ 0x3036, 0x34 },
	{ 0x3037, 0x13 },
	{ 0x3108, 0x01 },
	{ 0x3630, 0x36 },
	{ 0x3631, 0x0e },
	{ 0x3632, 0xe2 },
	{ 0x3633, 0x12 },
	{ 0x3621, 0xe0 },
	{ 0x3704, 0xa0 },
	{ 0x3703, 0x5a },
	{ 0x3715, 0x78 },
	{ 0x3717, 0x01 },
	{ 0x370b, 0x60 },
	{ 0x3705, 0x1a },
	{ 0x3905, 0x02 },
	{ 0x3906, 0x10 },
	{ 0x3901, 0x0a },
	{ 0x3731, 0x12 },
	{ 0x3600, 0x08 },
	{ 0x3601, 0x33 },
	{ 0x302d, 0x60 },
	{ 0x3620, 0x52 },
	{ 0x371b, 0x20 },
	{ 0x471c, 0x50 },
	{ 0x3a13, 0x43 },
	{ 0x3a17, 0x03 },
	{ 0x3a18, 0x03 },
	{ 0x3a19, 0xc0 },	// Changed from ff to c0 by IQL
	{ 0x3635, 0x13 },
	{ 0x3636, 0x03 },
	{ 0x3634, 0x40 },
	{ 0x3622, 0x01 },
	{ 0x3c01, 0x34 },
	{ 0x3c04, 0x28 },
	{ 0x3c05, 0x98 },
	{ 0x3c06, 0x00 },
	{ 0x3c07, 0x07 },
	{ 0x3c08, 0x00 },
	{ 0x3c09, 0x1c },
	{ 0x3c0a, 0x9c },
	{ 0x3c0b, 0x40 },
	{ 0x3820, 0x40 },	//Sensor flip=0 and mirror=1
	{ 0x3821, 0x07 },
	{ 0x3814, 0x11 },
	{ 0x3815, 0x11 },
	{ 0x3800, 0x00 },
	{ 0x3801, 0x00 },
	{ 0x3802, 0x00 },
	{ 0x3803, 0x00 },
	{ 0x3804, 0x0a },
	{ 0x3805, 0x3f },
	{ 0x3806, 0x07 },
	{ 0x3807, 0x9f },
	{ 0x3808, 0x0a },
	{ 0x3809, 0x20 },
	{ 0x380a, 0x07 },
	{ 0x380b, 0x98 },
	{ 0x380c, 0x0b },
	{ 0x380d, 0x1c },
	{ 0x380e, 0x07 },
	{ 0x380f, 0xb0 },
	{ 0x3810, 0x00 },
	{ 0x3811, 0x10 },
	{ 0x3812, 0x00 },
	{ 0x3813, 0x04 },
	{ 0x3618, 0x04 },
	{ 0x3612, 0x2b },
	{ 0x3708, 0x64 },
	{ 0x3709, 0x12 },
	{ 0x370c, 0x00 },
	{ 0x3a00, 0x7c },	//night_mode
	{ 0x3a01, 0x01 },
	{ 0x3a02, 0x0f },
	{ 0x3a03, 0xff },
	{ 0x3a05, 0x70 },	//Sigma change from 30 to 70 (161130)
	{ 0x3a08, 0x01 },
	{ 0x3a09, 0x27 },
	{ 0x3a0a, 0x00 },
	{ 0x3a0b, 0xf6 },
	{ 0x3a0e, 0x06 },
	{ 0x3a0d, 0x08 },
	{ 0x3a14, 0xff },
	{ 0x3a15, 0xff },
	{ 0x4001, 0x02 },
	{ 0x4004, 0x06 },
	{ 0x3000, 0x20 },	//Leave MCU in reset
	{ 0x3002, 0x1c },
	{ 0x3004, 0xdf },	//Disable mcu clock
	{ 0x3006, 0xc3 },
	{ 0x300e, 0x45 },	/* MIPI enabled */
	{ 0x302e, 0x08 },
	{ 0x4300, 0x32 },
	{ 0x4837, 0x0a },
	{ 0x501f, 0x00 },
	{ 0x440e, 0x00 },
	{ 0x5000, 0xa7 },
	{ 0x5001, 0x83 },
	{ 0x5180, 0xff },
	{ 0x5181, 0xf2 },
	{ 0x5182, 0x00 },
	{ 0x5183, 0x14 },	// [START] Sigma AWB tuning (advanced mode 0x14) (161121)
	{ 0x5184, 0x25 },	//
	{ 0x5185, 0x24 },	//
	{ 0x5186, 0x0b },	//
	{ 0x5187, 0x0f },	//
	{ 0x5188, 0x0c },	//
	{ 0x5189, 0x72 },	//
	{ 0x518a, 0x63 },	//
	{ 0x518b, 0xbb },	//
	{ 0x518c, 0x8c },	//
	{ 0x518d, 0x3c },	//
	{ 0x518e, 0x3c },	//
	{ 0x518f, 0x48 },	//
	{ 0x5190, 0x45 },	//
	{ 0x5191, 0xf8 },	//
	{ 0x5192, 0x04 },	//
	{ 0x5193, 0x70 },	//
	{ 0x5194, 0xf0 },	//
	{ 0x5195, 0xf0 },	//
	{ 0x5196, 0x03 },	//
	{ 0x5197, 0x01 },	//
	{ 0x5198, 0x06 },	//
	{ 0x5199, 0x9b },	//
	{ 0x519a, 0x04 },	//
	{ 0x519b, 0x00 },	//
	{ 0x519c, 0x04 },	//
	{ 0x519d, 0x14 },	//
	{ 0x519e, 0x38 },	// [END] Sigma AWB tuning (advanced mode 0x14) (161121)
	{ 0x5381, 0x1e },	// [START] Sigma roll back to default CCM, to be finished (161121)
	{ 0x5382, 0x5b },	//
	{ 0x5383, 0x08 },	//
	{ 0x5384, 0x0a },	//
	{ 0x5385, 0x7e },	//
	{ 0x5386, 0x88 },	//
	{ 0x5387, 0x7c },	//
	{ 0x5388, 0x6c },	//
	{ 0x5389, 0x10 },	//
	{ 0x538a, 0x01 },	//
	{ 0x538b, 0x98 },	// [START] Sigma roll back to default CCM, to be finished (161121)
	{ 0x5300, 0x08 },
	{ 0x5301, 0x30 },
	{ 0x5302, 0x10 },
	{ 0x5303, 0x00 },
	{ 0x5304, 0x08 },
	{ 0x5305, 0x30 },
	{ 0x5306, 0x08 },
	{ 0x5307, 0x16 },
	{ 0x5309, 0x08 },
	{ 0x530a, 0x30 },
	{ 0x530b, 0x04 },
	{ 0x530c, 0x06 },
	{ 0x5480, 0x01 },
	{ 0x5481, 0x1f },	// [START] Sigma gamma settings (161130)
	{ 0x5482, 0x2a },	//
	{ 0x5483, 0x3c },	//
	{ 0x5484, 0x61 },	//
	{ 0x5485, 0x73 },	//
	{ 0x5486, 0x7e },	//
	{ 0x5487, 0x88 },	//
	{ 0x5488, 0x91 },	//
	{ 0x5489, 0x9a },	//
	{ 0x548a, 0xa2 },	//
	{ 0x548b, 0xb1 },	//
	{ 0x548c, 0xbd },	//
	{ 0x548d, 0xd0 },	//
	{ 0x548e, 0xdf },	//
	{ 0x548f, 0xea },	//
	{ 0x5490, 0x1d },	// [END] Sigma gamma settings (161130)
	{ 0x5580, 0x02 },
	{ 0x5583, 0x40 },
	{ 0x5584, 0x10 },
	{ 0x5589, 0x10 },
	{ 0x558a, 0x00 },
	{ 0x558b, 0xf8 },
	{ 0x5800, 0x23 },	// [START] Sigma LENC table (161121)
	{ 0x5801, 0x14 },	//
	{ 0x5802, 0x0f },	//
	{ 0x5803, 0x0f },	//
	{ 0x5804, 0x12 },	//
	{ 0x5805, 0x26 },	//
	{ 0x5806, 0x0c },	//
	{ 0x5807, 0x08 },	//
	{ 0x5808, 0x07 },	//
	{ 0x5809, 0x07 },	//
	{ 0x580a, 0x08 },	//
	{ 0x580b, 0x0d },	//
	{ 0x580c, 0x08 },	//
	{ 0x580d, 0x03 },	//
	{ 0x580e, 0x01 },	//
	{ 0x580f, 0x01 },	//
	{ 0x5810, 0x07 },	//
	{ 0x5811, 0x09 },	//
	{ 0x5812, 0x07 },	//
	{ 0x5813, 0x03 },	//
	{ 0x5814, 0x01 },	//
	{ 0x5815, 0x01 },	//
	{ 0x5816, 0x07 },	//
	{ 0x5817, 0x08 },	//
	{ 0x5818, 0x0d },	//
	{ 0x5819, 0x08 },	//
	{ 0x581a, 0x05 },	//
	{ 0x581b, 0x06 },	//
	{ 0x581c, 0x08 },	//
	{ 0x581d, 0x0e },	//
	{ 0x581e, 0x29 },	//
	{ 0x581f, 0x17 },	//
	{ 0x5820, 0x11 },	//
	{ 0x5821, 0x11 },	//
	{ 0x5822, 0x15 },	//
	{ 0x5823, 0x28 },	//
	{ 0x5824, 0x46 },	//
	{ 0x5825, 0x26 },	//
	{ 0x5826, 0x08 },	//
	{ 0x5827, 0x26 },	//
	{ 0x5828, 0x64 },	//
	{ 0x5829, 0x26 },	//
	{ 0x582a, 0x24 },	//
	{ 0x582b, 0x21 },	//
	{ 0x582c, 0x02 },	//
	{ 0x582d, 0x24 },	//
	{ 0x582e, 0x06 },	//
	{ 0x582f, 0x21 },	//
	{ 0x5830, 0x30 },	//
	{ 0x5831, 0x21 },	//
	{ 0x5832, 0x24 },	//
	{ 0x5833, 0x26 },	//
	{ 0x5834, 0x24 },	//
	{ 0x5835, 0x21 },	//
	{ 0x5836, 0x22 },	//
	{ 0x5837, 0x26 },	//
	{ 0x5838, 0x44 },	//
	{ 0x5839, 0x24 },	//
	{ 0x583a, 0x26 },	//
	{ 0x583b, 0x28 },	//
	{ 0x583c, 0x42 },	//
	{ 0x583d, 0xff },	// [END] Sigma LENC table (161121)
	{ 0x5025, 0x00 },
	{ 0x3a0f, 0x30 },	// Exposure target
	{ 0x3a10, 0x28 },	//
	{ 0x3a1b, 0x30 },	//
	{ 0x3a1e, 0x26 },	//
	{ 0x3a11, 0x60 },
	{ 0x3a1f, 0x14 },
	{ 0x3008, 0x02 }
};

static struct reg_value ov5640_edge_enhancement = { 0x5302, 0x24 };	// Sigma increase edge enhancement from 10 to 24 (161025)

/*
 * settings based on ov5640_setting_30fps_720P_1280_720
 *
 * mode timings from https://confluence-se.flir.net/display/IN/vcam+modes
 *
 * vcam fov=55 used with IR lens fov=45
 */
#define OV5640_SETTING_30FPS_1280_960_HFOV54_ELEMENTS 75
static struct reg_value ov5640_setting_30fps_1280_960_HFOV54[OV5640_SETTING_30FPS_1280_960_HFOV54_ELEMENTS] = {
	{ 0x3008, 0x42 },
	{ 0x3035, 0x21 }, { 0x3036, 0x5c }, { 0x3c07, 0x07 },
	{ 0x3c09, 0x1c }, { 0x3c0a, 0x9c }, { 0x3c0b, 0x40 },
	{ 0x3814, 0x31 },	//Horizontal subsamble increment
	{ 0x3815, 0x31 },	//Vertical   subsamble increment
	{ 0x3800, 0x00 }, { 0x3801, 0x00 },	//X address start = 0x0
	{ 0x3802, 0x00 }, { 0x3803, 0x04 },	//Y address start = 0x4
	{ 0x3804, 0x0a }, { 0x3805, 0x3f },	//X address end   = 0xa3f
	{ 0x3806, 0x07 }, { 0x3807, 0x9b },	//Y address end   = 0x79b
	{ 0x3808, 0x05 }, { 0x3809, 0x00 },	//DVP width  output size = 0x500   (1280)
	{ 0x380a, 0x03 }, { 0x380b, 0xc0 },	//DVP height output size = 0x3c0   (960)
	{ 0x380c, 0x06 }, { 0x380d, 0x40 },	// Total horizontal size = 0x640   ()
	{ 0x380e, 0x03 }, { 0x380f, 0xd8 },	// Total vertical size  =  0x3d8   (984)
	{ 0x3810, 0x00 }, { 0x3811, 0x10 },	// ISP horizontal offset = 0x10
	{ 0x3812, 0x00 }, { 0x3813, 0x00 },	// ISP vertical   offset = 0x0
	{ 0x3618, 0x00 }, { 0x3612, 0x29 }, { 0x3708, 0x64 },
	{ 0x3709, 0x52 }, { 0x370c, 0x03 }, { 0x3a02, 0x0f },
	{ 0x3a03, 0xff }, { 0x3a08, 0x01 }, { 0x3a09, 0xbc },
	{ 0x3a0a, 0x01 }, { 0x3a0b, 0x72 }, { 0x3a0e, 0x06 },
	{ 0x3a0d, 0x02 }, { 0x3a14, 0x0f }, { 0x3a15, 0xff },
	{ 0x4001, 0x02 }, { 0x4004, 0x02 }, { 0x4713, 0x02 },
	{ 0x4407, 0x04 }, { 0x460b, 0x37 }, { 0x460c, 0x20 },
	{ 0x3824, 0x04 }, { 0x5001, 0x83 }, { 0x4005, 0x1a },
	{ 0x3008, 0x02 }, { 0x3503, 0 },
	{ 0x5688, 0x11 },	// [START] Sigma exposure weights (161130)
	{ 0x5689, 0x11 },	// keep 0x11 for HFOV28 and HFOV39
	{ 0x568a, 0x11 },	//
	{ 0x568b, 0x11 },	//
	{ 0x568c, 0x11 },	//
	{ 0x568d, 0x11 },	//
	{ 0x568e, 0x11 },	//
	{ 0x568f, 0x11 },	// [END] Sigma exposure weights (161130)
	{ 0x5186, 0x0b },	// [START] Sigma HFOV54/HFOV28 AWB (161202)
	{ 0x5187, 0x0f },	//
	{ 0x5188, 0x0c },	//
	{ 0x5189, 0x72 },	//
	{ 0x518a, 0x63 },	//
	{ 0x518e, 0x3c },	//
	{ 0x518f, 0x48 },	//
	{ 0x5190, 0x45 },	//
	{ 0x5198, 0x06 },	//
	{ 0x5199, 0x9b },	//
	{ 0x519c, 0x04 },	//
	{ 0x519d, 0x14 },	// [END] Sigma HFOV54/HFOV28 AWB (161202)
};

/*
 *
 *
 * settings based on ov5640_setting_30fps_720P_1280_720
 *
 * mode timings from https://confluence-se.flir.net/display/IN/vcam+modes
 *
 * vcam fov=39 used with IR lens fov=28
 */
#define OV5640_SETTING_30FPS_1280_960_HFOV39_ELEMENTS 75
static struct reg_value ov5640_setting_30fps_1280_960_HFOV39[OV5640_SETTING_30FPS_1280_960_HFOV39_ELEMENTS] = {
	{ 0x3008, 0x42 },
	{ 0x3035, 0x12 }, { 0x3036, 0x60 }, { 0x3c07, 0x07 },
	{ 0x3c09, 0x1c }, { 0x3c0a, 0x9c }, { 0x3c0b, 0x40 },
	{ 0x3814, 0x11 },	//Horizontal subsamble increment
	{ 0x3815, 0x11 },	//Vertical   subsamble increment
	{ 0x3800, 0x01 }, { 0x3801, 0x8c },	//X address start = 0x18c
	{ 0x3802, 0x01 }, { 0x3803, 0x26 },	//Y address start = 0x126
	{ 0x3804, 0x08 }, { 0x3805, 0xb3 },	//X address end   = 0x8b3
	{ 0x3806, 0x06 }, { 0x3807, 0x77 },	//Y address end   = 0x677
	{ 0x3808, 0x05 }, { 0x3809, 0x00 },	//DVP width  output size = 0x500   (1280)
	{ 0x380a, 0x03 }, { 0x380b, 0xc0 },	//DVP height output size = 0x3c0   (960)
	{ 0x380c, 0x08 }, { 0x380d, 0x00 },	// Total horizontal size = 0x800   (2048)
	{ 0x380e, 0x06 }, { 0x380f, 0x00 },	// Total vertical size  =  0x600   (1536)
	{ 0x3810, 0x00 }, { 0x3811, 0x10 },	// ISP horizontal offset = 0x10
	{ 0x3812, 0x00 }, { 0x3813, 0x06 },	// ISP vertical   offset = 0x6
	{ 0x3618, 0x00 }, { 0x3612, 0x29 }, { 0x3708, 0x64 },
	{ 0x3709, 0x52 }, { 0x370c, 0x03 }, { 0x3a02, 0x0f },
	{ 0x3a03, 0xff }, { 0x3a08, 0x01 }, { 0x3a09, 0xbc },
	{ 0x3a0a, 0x01 }, { 0x3a0b, 0x72 }, { 0x3a0e, 0x06 },
	{ 0x3a0d, 0x02 }, { 0x3a14, 0x0f }, { 0x3a15, 0xff },
	{ 0x4001, 0x02 }, { 0x4004, 0x02 }, { 0x4713, 0x02 },
	{ 0x4407, 0x04 }, { 0x460b, 0x37 }, { 0x460c, 0x20 },
	{ 0x3824, 0x04 }, { 0x5001, 0xa3 }, { 0x4005, 0x1a },
	{ 0x3008, 0x02 }, { 0x3503, 0 },
	{ 0x5688, 0x11 },	// [START] Sigma exposure weights (161130)
	{ 0x5689, 0x11 },	// keep 0x11 for HFOV28 and HFOV39
	{ 0x568a, 0x11 },	//
	{ 0x568b, 0x11 },	//
	{ 0x568c, 0x11 },	//
	{ 0x568d, 0x11 },	//
	{ 0x568e, 0x11 },	//
	{ 0x568f, 0x11 },	// [END] Sigma exposure weights (161130)
	{ 0x5186, 0x10 },	// [START] Sigma HFOV39 AWB (161202)
	{ 0x5187, 0x14 },	//
	{ 0x5188, 0x10 },	//
	{ 0x5189, 0x7d },	//
	{ 0x518a, 0x6b },	//
	{ 0x518e, 0x3a },	//
	{ 0x518f, 0x4d },	//
	{ 0x5190, 0x47 },	//
	{ 0x5198, 0x04 },	//
	{ 0x5199, 0x4d },	//
	{ 0x519c, 0x08 },	//
	{ 0x519d, 0x82 },	// [END] Sigma HFOV39 AWB (161202)
};

/*
 *
 *
 * settings based on ov5640_setting_30fps_720P_1280_720
 *
 * mode timings from https://confluence-se.flir.net/display/IN/vcam+modes
 *
 * vcam fov=28 used with IR lens fov=12
 *
 */
#define OV5640_SETTING_30FPS_1280_960_HFOV28_ELEMENTS 75
static struct reg_value ov5640_setting_30fps_1280_960_HFOV28[OV5640_SETTING_30FPS_1280_960_HFOV28_ELEMENTS] = {
	{ 0x3008, 0x42 },
	{ 0x3035, 0x21 }, { 0x3036, 0x5c }, { 0x3c07, 0x07 },
	{ 0x3c09, 0x1c }, { 0x3c0a, 0x9c }, { 0x3c0b, 0x40 },
	{ 0x3814, 0x11 },	//Horizontal subsamble increment
	{ 0x3815, 0x11 },	//Vertical   subsamble increment
	{ 0x3800, 0x02 }, { 0x3801, 0x90 },	//X address start = 0x290
	{ 0x3802, 0x01 }, { 0x3803, 0xec },	//Y address start = 0x1ec
	{ 0x3804, 0x07 }, { 0x3805, 0xaf },	//X address end   = 0x7af
	{ 0x3806, 0x05 }, { 0x3807, 0xb3 },	//Y address end   = 0x5b3
	{ 0x3808, 0x05 }, { 0x3809, 0x00 },	//DVP width  output size = 0x500   (1280)
	{ 0x380a, 0x03 }, { 0x380b, 0xc0 },	//DVP height output size = 0x3c0   (960)
	{ 0x380c, 0x06 }, { 0x380d, 0x00 },	// Total horizontal size = 0x600   (1536)
	{ 0x380e, 0x03 }, { 0x380f, 0xd8 },	// Total vertical size  =  0x3d8   (984)
	{ 0x3810, 0x00 }, { 0x3811, 0x10 },	// ISP horizontal offset = 0x10
	{ 0x3812, 0x00 }, { 0x3813, 0x04 },	// ISP vertical   offset = 0x4
	{ 0x3618, 0x00 }, { 0x3612, 0x29 }, { 0x3708, 0x64 },
	{ 0x3709, 0x52 }, { 0x370c, 0x03 }, { 0x3a02, 0x0f },
	{ 0x3a03, 0xff }, { 0x3a08, 0x01 }, { 0x3a09, 0xbc },
	{ 0x3a0a, 0x01 }, { 0x3a0b, 0x72 }, { 0x3a0e, 0x06 },
	{ 0x3a0d, 0x02 }, { 0x3a14, 0x0f }, { 0x3a15, 0xff },
	{ 0x4001, 0x02 }, { 0x4004, 0x02 }, { 0x4713, 0x02 },
	{ 0x4407, 0x04 }, { 0x460b, 0x37 }, { 0x460c, 0x20 },
	{ 0x3824, 0x04 }, { 0x5001, 0x83 }, { 0x4005, 0x1a },
	{ 0x3008, 0x02 }, { 0x3503, 0 },
	{ 0x5688, 0x33 },	// [START] Sigma exposure weights (161130)
	{ 0x5689, 0x33 },	// Tuned for HFOV54
	{ 0x568a, 0x53 },	//
	{ 0x568b, 0x35 },	//
	{ 0x568c, 0x53 },	//
	{ 0x568d, 0x35 },	//
	{ 0x568e, 0x33 },	//
	{ 0x568f, 0x33 },	// [END] Sigma exposure weights (161130)
	{ 0x5186, 0x0b },	// [START] Sigma HFOV54/HFOV28 AWB (161202)
	{ 0x5187, 0x0f },	//
	{ 0x5188, 0x0c },	//
	{ 0x5189, 0x72 },	//
	{ 0x518a, 0x63 },	//
	{ 0x518e, 0x3c },	//
	{ 0x518f, 0x48 },	//
	{ 0x5190, 0x45 },	//
	{ 0x5198, 0x06 },	//
	{ 0x5199, 0x9b },	//
	{ 0x519c, 0x04 },	//
	{ 0x519d, 0x14 },	// [END] Sigma HFOV54/HFOV28 AWB (161202)
};

static struct reg_value stream_on = { 0x4202, 0x00 };	//stream on
static struct reg_value stream_off = { 0x4202, 0x0f };	//stream off


static struct reg_value ov5640_mirror_on_reg = { 0x3821, 0x01 };
static struct reg_value ov5640_mirror_off_reg = { 0x3821, 0x07 };
static struct reg_value ov5640_flip_on_reg = { 0x3820, 0x46 };
static struct reg_value ov5640_flip_off_reg = { 0x3820, 0x40 };

static struct reg_value ov5640_testimage_on_reg = { 0x503d, 0x80 };
static struct reg_value ov5640_testimage_off_reg = { 0x503d, 0x00 };

static struct reg_value night_mode_on = { 0x3a00, 0x7c };
static struct reg_value night_mode_off = { 0x3a00, 0x78 };
static struct reg_value autofocus_on = { 0x3022, 0x04 };
static struct reg_value autofocus_off = { 0x3022, 0x00 };

static int CamActive[CAM_ALL] = { TRUE, FALSE };

static int g_vcamFOV = 54;
static CAM_NO g_camera = CAM_ALL;


/* OV5640 Configurations copied from WINCE Gas Camera */
/*
 * General initialization executed once at power on
 *
 *
 */
#define OV5640_INIT_INTERFACE_CSI_ELEMENTS 212
static struct reg_value ov5640_init_interface_csi[OV5640_INIT_INTERFACE_CSI_ELEMENTS] = {
	{0x3103, 0x11}, //SCCB system control
	{0x3008, 0x42}, //System root divider
	{0x3103, 0x03}, //SCCB system control
	{0x3017, 0xff}, //VSYNC I/O control
	{0x3018, 0xff}, //Pad output enable
	{0x3034, 0x1a}, //SC PLL control
	{0x3035, 0x21}, //System clock divider
	{0x3037, 0x13}, //PLL pre-divider
	{0x3108, 0x01}, //System root divider
	{0x302d, 0x60}, //System control

	{0x3630, 0x36}, {0x3631, 0x0e}, //undoc start
	{0x3632, 0xe2}, {0x3633, 0x12},
	{0x3621, 0xe0}, {0x3704, 0xa0},
	{0x3703, 0x5a}, {0x3715, 0x78},
	{0x3717, 0x01}, {0x370b, 0x60},
	{0x3705, 0x1a}, {0x3905, 0x02},
	{0x3906, 0x10}, {0x3901, 0x0a},
	{0x3731, 0x12}, {0x3620, 0x52},
	{0x371b, 0x20}, {0x3635, 0x13},
	{0x3636, 0x03}, {0x3634, 0x40},
	{0x3622, 0x01}, {0x471c, 0x50},
	{0x4050, 0x6e}, {0x4051, 0x8f},
	{0x302e, 0x00}, {0x5025, 0x00},

	{0x3824, 0x06}, //[AW change 02 to 06] PCLK divider
	{0x3a13, 0x43}, //AEC ctrl
	{0x3a18, 0x00}, {0x3a19, 0xf8}, //AEC Gain ceiling

	{0x3c01, 0x34}, {0x3c05, 0x98}, //5060HZ ctrl
	{0x3c06, 0x00}, {0x3c08, 0x00},
	{0x3c09, 0x1c}, {0x3c0a, 0x9c},
	{0x3c0b, 0x40},

	{0x3820, 0x47}, //VFLIP
	{0x3a08, 0x01}, {0x3a09, 0x27}, //AEC B50 step
	{0x3a0a, 0x00}, {0x3a0b, 0xf6},
	{0x4001, 0x02}, {0x4004, 0x02}, //BLC ctrl

	{0x300e, 0x58}, //MIPI ctrl
	{0x4300, 0x32}, //YUV422, YUYV
	{0x501f, 0x00}, //ISP YUV422
	{0x4713, 0x02}, //Jpeg mode 2
	{0x4407, 0x04}, {0x440e, 0x00}, //JPG ctrl
	{0x460b, 0x35}, //Debug
	{0x460c, 0x22}, //DVP PCLK divider control by 0x3824
	{0x471d, 0x00}, //[EVAL 01] VSYNC_mode according to FPGA
	{0x4740, 0x21}, //[EVAL 20] VSYNC active high, HREF active low
	{0x5000, 0xa7}, //ISP ctrl
	{0x5180, 0xff}, {0x5181, 0xf2}, //AWB
	{0x5182, 0x00}, {0x5183, 0x14},
	{0x5184, 0x25}, {0x5185, 0x24},
	{0x5186, 0x09}, {0x5187, 0x09},
	{0x5188, 0x09}, {0x5189, 0x75},
	{0x518a, 0x54}, {0x518b, 0xe0},
	{0x518c, 0xb2}, {0x518d, 0x42},
	{0x518e, 0x3d}, {0x518f, 0x56},
	{0x5190, 0x46}, {0x5191, 0xf8},
	{0x5192, 0x04}, {0x5193, 0x70},
	{0x5194, 0xf0}, {0x5195, 0xf0},
	{0x5196, 0x03}, {0x5197, 0x01},
	{0x5198, 0x04}, {0x5199, 0x12},
	{0x519a, 0x04}, {0x519b, 0x00},
	{0x519c, 0x06}, {0x519d, 0x82},
	{0x519e, 0x38},

	{0x5381, 0x1e}, {0x5382, 0x5b}, //CMX
	{0x5383, 0x08}, {0x5384, 0x0a},
	{0x5385, 0x7e}, {0x5386, 0x88},
	{0x5387, 0x7c}, {0x5388, 0x6c},
	{0x5389, 0x10}, {0x538a, 0x01},
	{0x538b, 0x98},

	{0x5300, 0x08}, {0x5301, 0x30}, //CIP
	{0x5302, 0x10}, {0x5303, 0x00},
	{0x5304, 0x08}, {0x5305, 0x30},
	{0x5306, 0x08}, {0x5307, 0x16},
	{0x5309, 0x08}, {0x530a, 0x30},
	{0x530b, 0x04}, {0x530c, 0x06},

	{0x5480, 0x01}, {0x5481, 0x08}, //Gamma
	{0x5482, 0x14}, {0x5483, 0x28},
	{0x5484, 0x51}, {0x5485, 0x65},
	{0x5486, 0x71}, {0x5487, 0x7d},
	{0x5488, 0x87}, {0x5489, 0x91},
	{0x548a, 0x9a}, {0x548b, 0xaa},
	{0x548c, 0xb8}, {0x548d, 0xcd},
	{0x548e, 0xdd}, {0x548f, 0xea},
	{0x5490, 0x1d},

	{0x5580, 0x02}, {0x5583, 0x40}, //SDE
	{0x5584, 0x10}, {0x5589, 0x10},
	{0x558a, 0x00}, {0x558b, 0xf8},

	{0x5800, 0x3f}, {0x5801, 0x21}, //LENC
	{0x5802, 0x13}, {0x5803, 0x11},
	{0x5804, 0x1a}, {0x5805, 0x29},
	{0x5806, 0x19}, {0x5807, 0x0c},
	{0x5808, 0x06}, {0x5809, 0x04},
	{0x580a, 0x08}, {0x580b, 0x18},
	{0x580c, 0x12}, {0x580d, 0x06},
	{0x580e, 0x01}, {0x580f, 0x00},
	{0x5810, 0x04}, {0x5811, 0x0e},
	{0x5812, 0x13}, {0x5813, 0x07},
	{0x5814, 0x01}, {0x5815, 0x01},
	{0x5816, 0x05}, {0x5817, 0x11},
	{0x5818, 0x1e}, {0x5819, 0x10},
	{0x581a, 0x0a}, {0x581b, 0x09},
	{0x581c, 0x0f}, {0x581d, 0x1d},
	{0x581e, 0x3f}, {0x581f, 0x2d},
	{0x5820, 0x1e}, {0x5821, 0x1f},
	{0x5822, 0x29}, {0x5823, 0x3f},
	{0x5824, 0x16}, {0x5825, 0x18},
	{0x5826, 0x09}, {0x5827, 0x17},
	{0x5828, 0x1a}, {0x5829, 0x29},
	{0x582a, 0x27}, {0x582b, 0x25},
	{0x582c, 0x27}, {0x582d, 0x17},
	{0x582e, 0x19}, {0x582f, 0x43},
	{0x5830, 0x50}, {0x5831, 0x32},
	{0x5832, 0x29}, {0x5833, 0x2b},
	{0x5834, 0x38}, {0x5835, 0x36},
	{0x5836, 0x27}, {0x5837, 0x28},
	{0x5838, 0x28}, {0x5839, 0x2a},
	{0x583a, 0x1e}, {0x583b, 0x2a},
	{0x583c, 0x47}, {0x583d, 0xce},

	{0x3a0f, 0x30}, {0x3a10, 0x28}, //AEC/AGC control start
	{0x3a1b, 0x30}, {0x3a1e, 0x26},
	{0x3a11, 0x60}, {0x3a1f, 0x14},

	{0x5001, 0xa3}, //ISP ctrl, Scale enable
	{0x3008, 0x02}, //Enable PCLK
};

/* attribute sysfs files */
static ssize_t enable_stream_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	PCAM_HW_INDEP_INFO pInfo = (PCAM_HW_INDEP_INFO)dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &val) < 0)
		return -EINVAL;

	OV5640_enable_stream(pInfo, CAM_1, val);
	return count;
}

static ssize_t flip_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	PCAM_HW_INDEP_INFO pInfo = (PCAM_HW_INDEP_INFO)dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &val) < 0)
		return -EINVAL;

	OV5640_FlipImage(pInfo, val);
	return count;
}

static ssize_t testpattern_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val) < 0)
		return -EINVAL;

	OV5640_Testpattern_Enable(dev, val);
	return count;
}


static ssize_t mirror_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	PCAM_HW_INDEP_INFO pInfo = (PCAM_HW_INDEP_INFO)dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &val) < 0)
		return -EINVAL;

	OV5640_mirror_enable(pInfo, CAM_1, val);
	return count;
}

static ssize_t autofocus_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	PCAM_HW_INDEP_INFO pInfo = (PCAM_HW_INDEP_INFO)dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &val) < 0)
		return -EINVAL;

	OV5640_autofocus_enable(pInfo, CAM_1, val);
	return count;
}


static ssize_t fov_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	PCAM_HW_INDEP_INFO pInfo = (PCAM_HW_INDEP_INFO)dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &val) < 0)
		return -EINVAL;
	OV5640_set_fov(pInfo, CAM_1, val);
	return count;
}

static ssize_t fov_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	sprintf(buf, "VCAM OV5640 FOV: (54 39 28) %i\n", g_vcamFOV);
	return strlen(buf);
}

static DEVICE_ATTR(enable_stream, 0200, NULL, enable_stream_store);
static DEVICE_ATTR(flip, 0200, NULL, flip_store);
static DEVICE_ATTR(testpattern, 0200, NULL, testpattern_store);
static DEVICE_ATTR(mirror_enable, 0200, NULL, mirror_enable_store);
static DEVICE_ATTR(autofocus_enable, 0200, NULL, autofocus_enable_store);
static DEVICE_ATTR(fov, 0644, fov_show, fov_store);

static struct attribute *ov5640_attrs[] = {
	&dev_attr_enable_stream.attr,
	&dev_attr_flip.attr,
	&dev_attr_testpattern.attr,
	&dev_attr_mirror_enable.attr,
	&dev_attr_autofocus_enable.attr,
	&dev_attr_fov.attr,
	NULL
};

static const struct attribute_group ov5640_groups = {
	.attrs = ov5640_attrs,
};


int OV5640_create_sysfs_attributes(struct device *dev)
{
	int ret;
	PCAM_HW_INDEP_INFO pInfo = (PCAM_HW_INDEP_INFO)dev_get_drvdata(dev);
	struct platform_device *pdev = pInfo->pLinuxDevice;

	ret = sysfs_create_group(&pdev->dev.kobj, &ov5640_groups);
	if (ret)
		pr_err("failed to add sys fs entry\n");
	return ret;

}

void OV5640_remove_sysfs_attributes(struct device *dev)
{
  PCAM_HW_INDEP_INFO pInfo = (PCAM_HW_INDEP_INFO)dev_get_drvdata(dev);
	struct platform_device *pdev = pInfo->pLinuxDevice;
	sysfs_remove_group(&pdev->dev.kobj, &ov5640_groups);
}


/* end sysfs attributes */


/* ov5640_write_reg
 *
 * Returns negative errno, else 0 on success
 */
static int ov5640_write_reg(PCAM_HW_INDEP_INFO pInfo, u16 reg, u8 val, CAM_NO cam)
{
	struct platform_device *pdev = pInfo->pLinuxDevice;
	struct device *dev = &pdev->dev;
	u8 buf[3] = { 0 };
	struct i2c_msg msgs[1];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = val;

	msgs[0].addr = pInfo->cameraI2CAddress[cam] >> 1;
	msgs[0].flags = I2C_M_TEN;
	msgs[0].buf = buf;
	msgs[0].len = 3;

	ret = i2c_transfer(pInfo->hI2C, msgs, 1);
	if (ret <= 0)
		return ret;

	return 0;
}

/* ov5640_read_reg
 * Returns negative errno, else 0 on success
 */
static int ov5640_read_reg(PCAM_HW_INDEP_INFO pInfo, u16 reg, u8 *val, CAM_NO cam)
{
	u8 buf[2] = { 0 };
	struct i2c_msg msgs[1];
	int ret;

	msgs[0].addr = pInfo->cameraI2CAddress[cam] >> 1;
	msgs[0].flags = I2C_M_TEN;
	msgs[0].len = 2;
	msgs[0].buf = buf;

	/* register to read */
	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	ret = i2c_transfer(pInfo->hI2C, msgs, 1);
	if (ret <= 0)
		return ret;

	/* Send in master receive mode. */
	msgs[0].flags |= I2C_M_RD;	/* see i2c_master_recv() */
	msgs[0].len = 1;
	msgs[0].buf = val;

	ret = i2c_transfer(pInfo->hI2C, msgs, 1);
	if (ret <= 0)
		return ret;

	return 0;
}

/* ov5640_read_reg
 * Returns negative errno, else 0 on success
 */
static int ov5640_mod_reg(PCAM_HW_INDEP_INFO pInfo, u16 reg, u8 mask, u8 val, CAM_NO cam)
{
	u8 readval;
	int ret;

	ret = ov5640_read_reg(pInfo, reg, &readval, cam);
	if (ret < 0)
		return ret;

	readval &= ~mask;
	val &= mask;
	val |= readval;

	return ov5640_write_reg(pInfo, reg, val, cam);
}

/* ov5640_get_otp_memory
 * Returns negative errno, else 0 on success
 */
static int ov5640_get_otp_memory(PCAM_HW_INDEP_INFO pInfo, u8 *otp_memory, int n, CAM_NO cam)
{
	struct platform_device *pdev = pInfo->pLinuxDevice;
	struct device *dev = &pdev->dev;
	int ret;
	int i;

	/* Enable OTP block and OPT clock */
	ret = ov5640_mod_reg(pInfo, OV5640_SYSTEM_RESET00, BIT(4), 0, cam);
	if (ret < 0) {
		dev_err(dev, "ov5640: failed to enable OTP module\n");
		return ret;
	}

	ret = ov5640_mod_reg(pInfo, OV5640_CLOCK_ENABLE00, BIT(4), BIT(4), cam);
	if (ret < 0) {
		dev_err(dev, "ov5640: failed to enable OTP clock\n");
		return ret;
	}

	/* According to OTP read example in datasheet */
	ret = ov5640_write_reg(pInfo, OV5640_OTP_PROGRAM_CTRL, 0, cam);
	if (ret < 0) {
		dev_err(dev, "ov5640: failed to disable OTP programming\n");
		return ret;
	}

	ret = ov5640_write_reg(pInfo, OV5640_OTP_READ_CTRL, 0, cam);
	if (ret < 0) {
		dev_err(dev, "ov5640: failed to disable OTP read\n");
		return ret;
	}

	ret = ov5640_write_reg(pInfo, OV5640_OTP_READ_CTRL, 1, cam);
	if (ret < 0) {
		dev_err(dev, "ov5640: failed to enable OTP read\n");
		return ret;
	}

	/* delay 1ms according to datasheet */
	msleep(1);

	ret = 0;
	for (i = 0; i < n && ret >= 0; i++) {
		ret = ov5640_read_reg(pInfo, OV5640_OTP_START_ADDR + i,
				    &otp_memory[i], cam);
		dev_dbg(dev, "otp[0x%x] 0x%x %c\n", OV5640_OTP_START_ADDR + i,
			otp_memory[i], otp_memory[i]);
	}

	if (ret < 0) {
		dev_err(dev, "ov5640_read_reg() failed\n");
		return ret;
	}

	/* delay 10ms according to datasheet */
	msleep(10);
	ret = ov5640_write_reg(pInfo, OV5640_OTP_READ_CTRL, 0, cam);
	if (ret < 0) {
		dev_err(dev, "ov5640: failed to disable OTP read\n");
		return ret;
	}

	/* Disable OTP block and OPT clock. Letting the block and
	 * clock enabled can cause the sensor to fail to start
	 * streaming frames.
	 */
	ret = ov5640_mod_reg(pInfo, OV5640_SYSTEM_RESET00, BIT(4), BIT(4), cam);
	if (ret < 0) {
		pr_err("ov5640: failed to disable OTP module\n");
		return ret;
	}

	ret = ov5640_mod_reg(pInfo, OV5640_CLOCK_ENABLE00, BIT(4), 0, cam);
	if (ret < 0) {
		pr_err("ov5640: failed to disable OTP clock\n");
		return ret;
	}

	return 0;
}

/* ov5640_get_sensor_models
 * Returns 0 on success, Unclear output on failure though...
 * Unclear what is returned though, since we are able to iterate over multiple cameras
 * If camera is set to CAM_ALL:
 * If ov5640_get_otp_memory fails on first camera, but succeeds on second camera output is 0
 * If ov5640_get_otp_memory succeeds  on first camera, but fails on second camera output is error from ov5640_get_otp_memory on second camera
 * If ov5640_get_otp_memory fails  on first camera and fails on second camera output is error from ov5640_get_otp_memory on second camera
 * Secondly, If ov5640_get_otp_memory fails on one of the cameras, pInfo->sensor_model[camera] is not changed (for that camera)...
 *  The sensor_model will probably implicitly end up as OV5640_STANDARD, as that could be 0 in the defined enum... and gpDev is allocated with kzalloc in vcamd.c
 *
 *
 * If camera is set to CAM_1 or CAM_2:
 * Return 0 on success, and error from ov5640_get_otp_memory on failure
 * Secondly, If ov5640_get_otp_memory fails on one of the cameras, pInfo->sensor_model[camera] is not changed (for that camera)...
 * The sensor_model will probably implicitly end up as OV5640_STANDARD, as that could be 0 in the defined enum... and gpDev is allocated with kzalloc in vcamd.c
 *
 * (if camera is set to CAM_ALL, we iterate from CAM_1 to CAM_2, else if
 * camera is set to CAM_1 we iterate from CAM_1 to CAM_1, and if camera is set
 * to CAM_2 we iterate from CAM_2 to CAM_2
 * this is setup a little strangely
 * and it would probably be better to cal a new function, ov5640_get_sensor_model once per camera instead...
 *
 */

static int ov5640_get_sensor_models(PCAM_HW_INDEP_INFO pInfo, CAM_NO camera)
{
	struct platform_device *pdev = pInfo->pLinuxDevice;
	struct device *dev = &pdev->dev;
	u8 otp_memory[OV5640_OTP_END_ADDR - OV5640_OTP_START_ADDR + 1];
	DWORD cam;
	DWORD cam_first = (camera == CAM_2) ? CAM_2 : CAM_1;
	DWORD cam_last = (camera == CAM_1) ? CAM_1 : CAM_2;
	int ret = 0;

	for (cam = cam_first; cam <= cam_last; cam++) {
		/* Read content in OTP memory */
		ret = ov5640_get_otp_memory(pInfo, otp_memory, OV5640_OTP_END_ADDR - OV5640_OTP_START_ADDR + 1, cam);
		if (ret) {
			dev_err(dev, "ov5640_get_otp_memory() failed\n");
			return ret;
		}

		/* The sensor model can be determined by the content of the
		 * OTP memory. There are two ways the memory can be programmed
		 * from the vendor. The memory can either contain a string
		 * specifying the model or a specific register can contain an
		 * integer value.
		 */

		/* Test if sensor is programmed with a string specifying model */
		if (strncmp(otp_memory, OV5640_SENSOR_MODEL_HIGH_K, strlen(OV5640_SENSOR_MODEL_HIGH_K)) == 0) {
			dev_info(dev, "ov5640: Sensor model id: \"%s\" (High K)\n", OV5640_SENSOR_MODEL_HIGH_K);
			pInfo->sensor_model[cam] = OV5640_HIGH_K;
			continue;
		}

		/* Test if sensor is programmed with a sensor id integer */
		if (otp_memory[OV5640_SENSOR_MODEL_ID_ADDR - OV5640_OTP_START_ADDR] == OV5640_SENSOR_MODEL_HIGH_K_ID) {
			dev_info(dev, "ov5640: Sensor model id: High K\n");
			pInfo->sensor_model[cam] = OV5640_HIGH_K;
			continue;
		}

		/* Assume the sensor is a standard model */
		dev_info(dev, "ov5640: cam %lu Standard sensor model\n", cam);
		pInfo->sensor_model[cam] = OV5640_STANDARD;
	}

	return ret;
}

/* ov5640_set_sensor_model_conf
 *
 * configures the camera with setting acquired from ov5640_get_sensor_model_conf
 * currently (2022) only ov5640_setting_High_K available...
 *
 * Iterates over cameras if camera is CAM_ALL
 * else run for CAM_1 or CAM_2
 * Fetch config from ov56450_get_sensor_model_conf
 * If config exists, send config to each camera
 *
 * If OV5640_DoI2CWrite fails, depends on what camera is set to
 * camera = CAM_ALL, silently drops error code from OV5640_DoI2CWrite to CAM_1, returns failure code from call with CAM_2
 *
 * if camera == CAM_1 || CAM_2
 *   return error code from OV5640_DoI2CWrite
 *
 */
static int ov5640_set_sensor_model_conf(PCAM_HW_INDEP_INFO pInfo, CAM_NO camera)
{
	struct platform_device *pdev = pInfo->pLinuxDevice;
	struct device *dev = &pdev->dev;
	DWORD cam;
	DWORD cam_first = (camera == CAM_2) ? CAM_2 : CAM_1;
	DWORD cam_last = (camera == CAM_1) ? CAM_1 : CAM_2;
	int ret = 0;

	for (cam = cam_first; cam <= cam_last; cam++) {
		if (pInfo->sensor_model[camera] == OV5640_HIGH_K) {
			dev_info(dev, "Selecting High_K config\n");
			ret = OV5640_DoI2CWrite(pInfo, ov5640_setting_High_K, OV5640_SETTING_HIGH_K_ELEMENTS, cam);
			if (ret) {
				dev_err(dev, "OV5640_DoI2CWrite() failed for camera %lu\n", cam);
				continue;
			}
		} else if (pInfo->sensor_model[camera] == OV5640_STANDARD) {
			dev_info(dev, "Selecting Standard OV5640 config\n");
		} else {
			dev_info(dev, "Unknown OV5640\n");
			return -1;
		}
	}

	return ret;
}

/* ov5640_set_sensor_model_conf
 * returns 0 on success
 *        negative error when either i2c_transfer to respective camera fails
 *
 * detects of camera is set as CAM_ALL, CAM_1 or CAM_2
 * iterates on CAM_1 -- CAM_2 if camera is CAM_ALL
 * iterates on CAM_1 -- CAM_1 if camera is CAM_1
 * iterates on CAM_2 -- CAM_2 if camera is CAM_2
 *
 */
int OV5640_DoI2CWrite(PCAM_HW_INDEP_INFO pInfo, struct reg_value *pMode, USHORT elements, CAM_NO camera)
{
	struct platform_device *pdev = pInfo->pLinuxDevice;
	struct device *dev = &pdev->dev;
	struct i2c_msg msgs[1];
	int i, retval = 0;
	DWORD cam;
	DWORD cam_first = (camera == CAM_2) ? CAM_2 : CAM_1;
	DWORD cam_last = (camera == CAM_1) ? CAM_1 : CAM_2;
	u8 buf[3] = { 0 };
	u16 RegAddr = 0;
	u8 Val = 0;
	for (cam = cam_first; cam <= cam_last; cam++) {
		/*  Check if camera in use */
		msgs[0].addr = pInfo->cameraI2CAddress[cam] >> 1;
		if (msgs[0].addr == 0)
			continue;
		msgs[0].flags = 0;
		msgs[0].buf = buf;
		msgs[0].len = 3;

		for (i = 0; i < elements; ++i) {
			RegAddr = pMode[i].u16RegAddr;
			Val = pMode[i].u8Val;

			buf[0] = RegAddr >> 8;
			buf[1] = RegAddr & 0xff;
			buf[2] = Val;

			retval = i2c_transfer(pInfo->hI2C, msgs, 1);

			if (retval == -EAGAIN) {
				msleep(100);
				retval = i2c_transfer(pInfo->hI2C, msgs, 1);
			}

			if (retval <= 0) {
				dev_err(dev, "failed on index i=%i with error %i (data 0x%x:0x%x)\n",
					i, retval, RegAddr, Val);
				return retval;
			}
		}
	}

	return 0;
}

/* OV640_enable_stream
 * returns void
 *
 *
 */
void OV5640_enable_stream(PCAM_HW_INDEP_INFO pInfo, CAM_NO camera, bool enable)
{
	if (enable)
		OV5640_DoI2CWrite(pInfo, &stream_on, 1, camera);
	else
		OV5640_DoI2CWrite(pInfo, &stream_off, 1, camera);
}

/* OV5640_nightmode_enable
 * returns void
 *
 *
 */
static void OV5640_nightmode_enable(PCAM_HW_INDEP_INFO pInfo, CAM_NO camera, bool enable)
{
	if (enable)
		OV5640_DoI2CWrite(pInfo, &night_mode_on, 1, camera);
	else
		OV5640_DoI2CWrite(pInfo, &night_mode_off, 1, camera);
}

/* OV5640_nightmode_enable
 * returns DWORD on error (dword is unsigned long?)
 *
 * Return value is from output OV5640_DoI2CWrite, which returns an integer error value
 * that is 0 on success, and negative on error, this is casted? to a DWORD, should end up as
 * 0 on success and positive value on error...
 *
 *
 */
static DWORD OV5640_mirror_enable(PCAM_HW_INDEP_INFO pInfo, CAM_NO camera, bool enable)
{
	int ret;

	if (enable)
		ret = OV5640_DoI2CWrite(pInfo, &ov5640_mirror_on_reg, 1, camera);
	else
		ret = OV5640_DoI2CWrite(pInfo, &ov5640_mirror_off_reg, 1, camera);
	return ret;
}

/* OV5640_autofocus_enable
 * returns void
 *
 *
 */
static void OV5640_autofocus_enable(PCAM_HW_INDEP_INFO pInfo, CAM_NO camera, bool enable)
{
	if (enable)
		OV5640_DoI2CWrite(pInfo, &autofocus_on, 1, camera);
	else
		OV5640_DoI2CWrite(pInfo, &autofocus_off, 1, camera);
}

/* OV5640_set_exposure
 * Set vcam exposure value
 *
 * returns void
 */
static void OV5640_set_exposure(PCAM_HW_INDEP_INFO pInfo, CAM_NO camera, int exp)
{
	struct reg_value temp;

	temp.u16RegAddr = 0x3500;
	temp.u8Val = ((exp >> 16) & 0x0f);
	OV5640_DoI2CWrite(pInfo, &temp, 1, camera);

	temp.u16RegAddr = 0x3501;
	temp.u8Val = ((exp >> 8) & 0xff);
	OV5640_DoI2CWrite(pInfo, &temp, 1, camera);

	temp.u16RegAddr = 0x3502;
	temp.u8Val = (exp & 0xf0);
	OV5640_DoI2CWrite(pInfo, &temp, 1, camera);
}

/* nightmode_on_off_work
 *
 * workqueue work thingy...
 */
static void nightmode_on_off_work(struct work_struct *work)
{
	CAM_HW_INDEP_INFO *pInfo = container_of(work, CAM_HW_INDEP_INFO, nightmode_work);

	msleep(1000);
	OV5640_nightmode_enable(pInfo, pInfo->cam, FALSE);
	msleep(1000);
	OV5640_nightmode_enable(pInfo, pInfo->cam, TRUE);
}


/* OV5640_set_5MP
 *
 * returns 0 on success
 *         <0, (or >0) on  error...
 *
 * sets modes ov5640_init_settings_9fps_5MP
 *            ov5640_edge_enhancement
 *            OV5640_FlipImage      - default flip of sensor??
 *            OV5640_mirror_enable  - default mirror of sensor??
 *                image flip and mirror enable, might also be set in ov5640_init_settings_9fps_5MP and similar structs...
 *
 */
static int OV5640_set_5MP(PCAM_HW_INDEP_INFO pInfo, CAM_NO camera)
{
	int ret;
	struct platform_device *pdev = pInfo->pLinuxDevice;
	struct device *dev = &pdev->dev;

	bool ov5640_using_mipi_interface = !of_find_property(pInfo->node, VCAM_PARALLELL_INTERFACE, NULL);

	OV5640_enable_stream(pInfo, camera, FALSE);

	/* Initialize camera settings */
	if (ov5640_using_mipi_interface)
		ret = OV5640_DoI2CWrite(pInfo, ov5640_init_setting_9fps_5MP, OV5640_INIT_SETTING_9FPS_5MP_ELEMENTS, camera);
	else
		ret = OV5640_DoI2CWrite(pInfo, ov5640_init_setting_5MP, OV5640_INIT_SETTING_5MP_ELEMENTS, camera);

	if (ret) {
		dev_err(dev, "Failed to set %s 5MP mode\n", ov5640_using_mipi_interface ? "MIPI" : "parallell");
		return ret;
	}

	/* Write model specific configuration */
	ov5640_set_sensor_model_conf(pInfo, camera);

	if (pInfo->edge_enhancement) {
		ret = OV5640_DoI2CWrite(pInfo, &ov5640_edge_enhancement, 1, camera);
		if (ret) {
			dev_err(dev, "Failed to enable edge enhancement\n");
			return ret;
		}
	}

	if (ov5640_using_mipi_interface) {
		/* Set default flip */
		ret = OV5640_FlipImage(pInfo, FALSE);
		if (ret) {
			dev_err(dev, "Failed to call OV5640_FlipImage\n");
			return ret;
		}

		ret = OV5640_mirror_enable(pInfo, camera, pInfo->flipped_sensor);
		if (ret) {
			dev_err(dev, "Failed to call OV5640_mirror_enable\n");
			return ret;
		}
	}

	OV5640_enable_stream(pInfo, camera, TRUE);
	return 0;
}


/* OV5640_set_fov
 *
 *
 * returns 0 on success
 *         <0, (or >0) on  error...
 *         ERROR_NOT_SUPPORTED, setting not allowed..
 *
 */
static int OV5640_set_fov(PCAM_HW_INDEP_INFO pInfo, CAM_NO camera, int fov)
{
	int ret = ERROR_NOT_SUPPORTED;
	int elements;
	struct reg_value *setting;
	struct platform_device *pdev = pInfo->pLinuxDevice;
	struct device *dev = &pdev->dev;

	switch (fov) {
	case 54:
		setting = ov5640_setting_30fps_1280_960_HFOV54;
		elements = OV5640_SETTING_30FPS_1280_960_HFOV54_ELEMENTS;
		ret = 0;
		break;

	case 39:
		setting = ov5640_setting_30fps_1280_960_HFOV39;
		elements = OV5640_SETTING_30FPS_1280_960_HFOV39_ELEMENTS;
		ret = 0;
		break;

	case 28:
		setting = ov5640_setting_30fps_1280_960_HFOV28;
		elements = OV5640_SETTING_30FPS_1280_960_HFOV28_ELEMENTS;
		ret = 0;
		break;

	default:
		dev_err(dev, "VCAM: Unsupported fov: %d\n", fov);
		ret = ERROR_NOT_SUPPORTED;
		break;
	}
	if (ret == 0) {
		dev_info(dev, "Change fov to %i\n", fov);
		ov5640_set_sensor_model_conf(pInfo, camera);
		OV5640_enable_stream(pInfo, camera, FALSE);
		ret = OV5640_DoI2CWrite(pInfo, setting, elements, camera);

		OV5640_enable_stream(pInfo, camera, TRUE);

		if (ret == 0) {
			g_vcamFOV = fov;
			schedule_work(&pInfo->nightmode_work);
		}
	}

	return ret;
}

/* OV5640_Testpattern_Enable
 *
 * Enables testpattern output (on CAM_1 only)
 *
 * returns void
 *
 */
static void OV5640_Testpattern_Enable(struct device *dev, bool on)
{
  PCAM_HW_INDEP_INFO pInfo = (PCAM_HW_INDEP_INFO)dev_get_drvdata(dev);

	if (on) {
		dev_info(dev, "Enable testpattern\n");
		OV5640_DoI2CWrite(pInfo, &ov5640_testimage_on_reg, 1, CAM_1);
	} else {
		dev_info(dev, "Disable testpattern\n");
		OV5640_DoI2CWrite(pInfo, &ov5640_testimage_off_reg, 1, CAM_1);
	}
}

/* OV5640_FlipImage
 * returns output of OV5640_DoI2CWrite (integer)
 * returns DWORD on error (dword is unsigned long?)
 *
 * Return value is from output OV5640_DoI2CWrite, which returns an integer error value
 * that is 0 on success, and negative on error, this is casted? to a DWORD, should end up as
 * 0 on success and positive value on error...
 *
 */
DWORD OV5640_FlipImage(PCAM_HW_INDEP_INFO pInfo, bool flip)
{
	struct reg_value *regval;

	if ((pInfo->flipped_sensor && !flip) ||
	    ((!pInfo->flipped_sensor && flip)))
		regval = &ov5640_flip_on_reg;
	else
		regval = &ov5640_flip_off_reg;

	return OV5640_DoI2CWrite(pInfo, regval, 1, g_camera);
}


/* ov5640_initmipicamera
 * Initialize MIPI attached camera (MIPI interface between OV5640 and FPGA)
 *
 * returns 0 on succes
 *         else failure
 */
static int ov5640_initmipicamera(struct device *dev, CAM_NO camera)
{
	PCAM_HW_INDEP_INFO pInfo = dev_get_drvdata(dev);
	int ret = 0;

	dev_info(dev, "MIPI interface used\n");
	ret = OV5640_set_5MP(pInfo, camera);
	if (ret) {
		dev_err(dev, "Failed to configure MIPI camera interface\n");
		return ret;
	}

	return ret;
}

/* ov5640_initcsicamera
 * Initialize CSI attached camera (CSI interface between OV5640 and FPGA)
 *
 * returns 0 on succes
 *         else failure
 */
static int ov5640_initcsicamera(struct device *dev, CAM_NO camera)
{
	PCAM_HW_INDEP_INFO pInfo = dev_get_drvdata(dev);
	int ret = 0;

	dev_info(dev, "cam %u, Parallell interface\n", camera);
	ret = OV5640_DoI2CWrite(pInfo, ov5640_init_interface_csi, OV5640_INIT_INTERFACE_CSI_ELEMENTS, camera);
	if (ret) {
		dev_err(dev, "Failed to configure parallell csi camera interface\n");
		return ret;
	}

	ret = OV5640_mirror_enable(pInfo, g_camera, true);
	if (ret < 0) {
		dev_err(dev, "Failed to enable mirror on sensor\n");
		return ret;
	}

	return 0;
}

/* ov5640_initcamera
 * initialize the OV5640 camera
 *
 * returns 0 on success
 *       else failed
 *
 */
static int ov5640_initcamera(struct device *dev, CAM_NO camera)
{
	PCAM_HW_INDEP_INFO pInfo = dev_get_drvdata(dev);
	int ret = 0;

	if (of_find_property(pInfo->node, VCAM_PARALLELL_INTERFACE, NULL)) {
		ret = ov5640_initcsicamera(dev, camera);
		if (ret < 0) {
			dev_err(dev, "Failed to initialise parallell camera interface\n");
			return ret;
		}
	} else {
		ret = ov5640_initmipicamera(dev, camera);
		if (ret < 0) {
			dev_err(dev, "Failed to initialise MIPI camera interface\n");
			return ret;
		}
	}

	ret = OV5640_set_fov(pInfo, g_camera, g_vcamFOV);
	if (ret)
		return ret;

	return ret;
}
/* OV5640_Init
 *
 * Start initializing cameras...
 * Would prefer if we remove the CAM_ALL setting...
 *
 */
int OV5640_Init(struct device *dev)
{
	PCAM_HW_INDEP_INFO pInfo = dev_get_drvdata(dev);
	int ret = 0;

	INIT_WORK(&pInfo->nightmode_work, nightmode_on_off_work);

	if (pInfo->cameraI2CAddress[1] == 0)	/* Only 1 active camera */
		g_camera = CAM_1;

	return ret;
}

/* OV5640_IOControl
 *
 */
DWORD OV5640_IOControl(PCAM_HW_INDEP_INFO pInfo, DWORD Ioctl, PUCHAR pBuf, PUCHAR pUserBuf)
{
	DWORD dwErr = ERROR_INVALID_PARAMETER;
	static int TestActive;
	struct platform_device *pdev = pInfo->pLinuxDevice;
	struct device *dev = &pdev->dev;

	switch (Ioctl) {
	case IOCTL_CAM_GET_TEST:
		{
			LOCK(pInfo);
			((VCAMIOCTLTEST *) pBuf)->bTestMode = TestActive;
			dwErr = 0;
			UNLOCK(pInfo);
		}
		break;

	case IOCTL_CAM_SET_TEST:
		{
			LOCK(pInfo);
			TestActive = (((VCAMIOCTLTEST *) pBuf)->bTestMode != 0);
			dwErr = 0;
			UNLOCK(pInfo);
		}
		break;

	case IOCTL_CAM_GET_ACTIVE:
		{
			VCAMIOCTLACTIVE *pVcamIoctl = (VCAMIOCTLACTIVE *) pBuf;
			LOCK(pInfo);
			pVcamIoctl->bActive = CamActive[CAM_1] || CamActive[CAM_2];
			dwErr = 0;
			UNLOCK(pInfo);
		}
		break;

	case IOCTL_CAM_INIT:

		/* Read the OTP memory before the initial configuration. This
		 * is the only time the otp memory is read. If read after the
		 * initial settings configuration is loaded the sensor can
		 * fail to start to stream frames.
		 */
		dwErr = ov5640_get_sensor_models(pInfo, g_camera);
		if (dwErr) {
			dev_err(dev, "Failed to get sensor models\n");
			break;
		}

		dwErr = ov5640_initcamera(dev, g_camera);
		break;
	case IOCTL_CAM_SET_ACTIVE:
	case IOCTL_CAM_SET_2ND_ACTIVE:
		{
			int NewActive;
			int res = TRUE;
			CAM_NO cam = (Ioctl == IOCTL_CAM_SET_ACTIVE) ? CAM_1 : CAM_2;
			LOCK(pInfo);
			NewActive = (((VCAMIOCTLACTIVE *) pBuf)->bActive != 0);

			if (res) {
				CamActive[cam] = NewActive;
				dwErr = ERROR_SUCCESS;
			}
			UNLOCK(pInfo);
		}
		break;

	case IOCTL_CAM_SET_FLASH:
		{
			VCAMIOCTLFLASH *pFlashData = (VCAMIOCTLFLASH *) pBuf;

			LOCK(pInfo);

			dwErr = pInfo->pSetTorchState(pInfo, pFlashData);
			/* set fast exposure to compensate for led brightness */
			if (pFlashData->bTorchOn)
				OV5640_set_exposure(pInfo, g_camera, 0x2000);

			UNLOCK(pInfo);
		}
		break;

	case IOCTL_CAM_SET_CAMMODE:
		{
			VCAMIOCTLCAMMODE *pMode = (VCAMIOCTLCAMMODE *) pBuf;

			LOCK(pInfo);
			switch (pMode->eCamMode) {
			case VCAM_STILL:
				/* set camera to 5MP full size mode */
				dwErr = OV5640_set_5MP(pInfo, g_camera);
				msleep(800);
				break;

			case VCAM_DRAFT:
				/* restore last known fov */
				dwErr = ov5640_initcamera(dev, g_camera);
				msleep(500);

				break;

			case VCAM_UNDEFINED:
			case VCAM_RESET:
			default:
				dev_err(dev, "VCAM Unsupported IOCTL_CAM_SET_CAMMODE %d\n", pMode->eCamMode);
				break;
			}
			UNLOCK(pInfo);
		}
		break;

	case IOCTL_CAM_SET_FOV:
		{
			VCAMIOCTLFOV *pVcamFOV = (VCAMIOCTLFOV *) pBuf;
			LOCK(pInfo);
			dwErr = OV5640_set_fov(pInfo, g_camera, pVcamFOV->fov);
			UNLOCK(pInfo);
		}
		break;
	case IOCTL_CAM_GET_FOV:
		LOCK(pInfo);
		((VCAMIOCTLFOV *) pBuf)->fov = g_vcamFOV;
		dwErr = 0;
		UNLOCK(pInfo);
		break;


	case IOCTL_CAM_MIRROR_ON:
		dwErr = OV5640_mirror_enable(pInfo, g_camera, TRUE);
		break;
	case IOCTL_CAM_MIRROR_OFF:
		dwErr = OV5640_mirror_enable(pInfo, g_camera, FALSE);
		break;
	case IOCTL_CAM_FLIP_ON:
		LOCK(pInfo);
		dwErr = OV5640_FlipImage(pInfo, true);
		UNLOCK(pInfo);
		break;
	case IOCTL_CAM_FLIP_OFF:
		LOCK(pInfo);
		dwErr = OV5640_FlipImage(pInfo, false);
		UNLOCK(pInfo);
		break;
	default:
		dev_err(dev, "VCAM Unsupported IOCTL code %lu\n", Ioctl);
		dwErr = ERROR_NOT_SUPPORTED;
		break;
	}
	return dwErr;
}
