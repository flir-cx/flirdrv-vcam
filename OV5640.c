/***********************************************************************
 *                                                                     
 * Project: Balthazar
 * $Date: 2013/10/04 $
 * $Author: pfitger $
 *
 * $Id: //depot/Balthazar/Camera/OS/Experimental/linux/drivers/vcam/MT9P111.c#1 $
 *
 * Description of file:
 *   Visual Camera Driver
 *
 * Last check-in changelist:
 * $Change: 179282 $
 *
 * Copyright: FLIR Systems AB
 ***********************************************************************/

#include "flir_kernel_os.h"
#include "vcam_internal.h"
#include "i2cdev.h"
#include <linux/i2c.h>

// Local typedefs

typedef enum { CAM_1, CAM_2, CAM_ALL } CAM_NO;

// Local definitions

#define DEBUG_TMO           0

// Local functions

// Local variables
static BOOL bCamActive[CAM_ALL] = { TRUE, FALSE };

static BOOL init;

static int g_vcamFOV;
static CAM_NO g_camera = CAM_ALL;
struct reg_value {
	u16 u16RegAddr;
	u8 u8Val;
};

/*Settings from
/Rocky/Elektronik/Komponenter/datablad/VCam/Settings/
*/
static struct reg_value ov5640_init_setting_9fps_5MP[] = {
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

static struct reg_value ov5640_flip_reg[] = {
	{ 0x3820, 0x46 },	//Sensor flip=1 and mirror=0
	{ 0x3821, 0x01 },
	{ 0x4300, 0x32 },	//Change CbCr order
};

static struct reg_value ov5640_edge_enhancement[] = {
	{ 0x5302, 0x24 },	// Sigma increase edge enhancement from 10 to 24 (161025)
};

/*
 *
 *
 * settings based on ov5640_setting_30fps_720P_1280_720
 *
 * mode timings from https://confluence-se.flir.net/display/IN/vcam+modes
 *
 * vcam fov=55 used with IR lens fov=45
*/
static struct reg_value ov5640_setting_30fps_1280_960_HFOV54[] = {
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
static struct reg_value ov5640_setting_30fps_1280_960_HFOV39[] = {
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
static struct reg_value ov5640_setting_30fps_1280_960_HFOV28[] = {
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

static BOOL DoI2CWrite(PCAM_HW_INDEP_INFO pInfo,
		       struct reg_value *pMode, USHORT elements, CAM_NO camera)
{
	struct i2c_msg msgs[1];
	int i, retval = 0;
	int retries = 50;
	DWORD cam;
	DWORD cam_first = (camera == CAM_2) ? CAM_2 : CAM_1;
	DWORD cam_last = (camera == CAM_1) ? CAM_1 : CAM_2;
	u8 buf[3] = { 0 };
	u16 RegAddr = 0;
	u8 Val = 0;

	for (cam = cam_first; cam <= cam_last; cam++) {
		// Check if camera in use
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

			if (retval <= 0) {
				if (retries-- <= 0) {
					pr_err
						("VCAM: %s failing on element %d of %d\n",
						 __func__,
						 i, elements);
					return ERROR_NOT_SUPPORTED;	// Too many errors, give up
				}
				usleep_range(10000, 20000);
				i--;
				continue;
			}
		}
	}

	return ERROR_SUCCESS;
}

static struct reg_value stream_on[] = {
	{ 0x4202, 0x00 },	//stream on
};

static struct reg_value stream_off[] = {
	{ 0x4202, 0x0f },	//stream off
};

void OV5640_stream_on(PCAM_HW_INDEP_INFO pInfo, CAM_NO cam)
{
	DoI2CWrite(pInfo, stream_on, dim(stream_on), cam);
}

void OV5640_stream_off(PCAM_HW_INDEP_INFO pInfo, CAM_NO cam)
{
	DoI2CWrite(pInfo, stream_off, dim(stream_off), cam);
}

void OV5640_MipiSuspend(PCAM_HW_INDEP_INFO pInfo, BOOL bSuspend)
{
	struct reg_value mipi_pwdn = { 0x300e, 0x45 };

	if (bSuspend) {
		/* Set register 0x300E[4:3] to 2'b11 before the PWDN pin is set high */
		mipi_pwdn.u8Val |= 0x18;
	}
	DoI2CWrite(pInfo, &mipi_pwdn, 1, g_camera);
}

void OV5640_nightmode_off(PCAM_HW_INDEP_INFO pInfo, CAM_NO cam)
{
	struct reg_value night_mode_off = { 0x3a00, 0x78 };	//night mode off

	DoI2CWrite(pInfo, &night_mode_off, 1, cam);
}

void OV5640_nightmode_on(PCAM_HW_INDEP_INFO pInfo, CAM_NO cam)
{
	struct reg_value night_mode_on = { 0x3a00, 0x7c };	//night mode on

	DoI2CWrite(pInfo, &night_mode_on, 1, cam);
}

/*Turn vcam autofocus on*/
void OV5640_autofocus_on(PCAM_HW_INDEP_INFO pInfo, CAM_NO camera)
{
	struct reg_value buff = { 0x3022, 0x04 };

	DoI2CWrite(pInfo, &buff, 1, camera);
}

/*Turn vcam autofocus off*/
void OV5640_autofocus_off(PCAM_HW_INDEP_INFO pInfo, CAM_NO camera)
{
	struct reg_value buff = { 0x3022, 0x00 };

	DoI2CWrite(pInfo, &buff, 1, camera);
}

/*Set vcam exposure value*/
static void OV5640_set_exposure(PCAM_HW_INDEP_INFO pInfo, CAM_NO camera,
				int exp)
{
	struct reg_value temp;

	temp.u16RegAddr = 0x3500;
	temp.u8Val = ((exp >> 16) & 0x0f);
	DoI2CWrite(pInfo, &temp, 1, camera);

	temp.u16RegAddr = 0x3501;
	temp.u8Val = ((exp >> 8) & 0xff);
	DoI2CWrite(pInfo, &temp, 1, camera);

	temp.u16RegAddr = 0x3502;
	temp.u8Val = (exp & 0xf0);
	DoI2CWrite(pInfo, &temp, 1, camera);

}

static void nightmode_on_off_work(struct work_struct *work)
{
	CAM_HW_INDEP_INFO *pInfo =
	    container_of(work, CAM_HW_INDEP_INFO, nightmode_work);

	// pr_err("VCAM turning nightmode off and on\n");
	msleep(1000);
	OV5640_nightmode_off(pInfo, pInfo->cam);
	msleep(1000);
	OV5640_nightmode_on(pInfo, pInfo->cam);

}

static BOOL OV5640_set_5MP(PCAM_HW_INDEP_INFO pInfo, CAM_NO cam)
{
	int ret;

	OV5640_stream_off(pInfo, cam);
	ret =
	    DoI2CWrite(pInfo, ov5640_init_setting_9fps_5MP,
		       dim(ov5640_init_setting_9fps_5MP), cam);
	if (ret)
		return ret;

	if (pInfo->edge_enhancement) {
		ret =
		    DoI2CWrite(pInfo, ov5640_edge_enhancement,
			       dim(ov5640_edge_enhancement), cam);
		if (ret)
			return ret;
	}

	if (pInfo->flip_image) {
		ret =
		    DoI2CWrite(pInfo, ov5640_flip_reg, dim(ov5640_flip_reg),
			       cam);
		if (ret)
			return ret;
	}
	OV5640_stream_on(pInfo, cam);
	return ret;
}

static BOOL OV5640_set_fov(PCAM_HW_INDEP_INFO pInfo, CAM_NO cam, int fov)
{
	int ret;

	OV5640_stream_off(pInfo, cam);

	switch (fov) {
	case 54:
		ret =
		    DoI2CWrite(pInfo, ov5640_setting_30fps_1280_960_HFOV54,
			       dim(ov5640_setting_30fps_1280_960_HFOV54), cam);
		g_vcamFOV = fov;
		break;

	case 39:
		ret =
		    DoI2CWrite(pInfo, ov5640_setting_30fps_1280_960_HFOV39,
			       dim(ov5640_setting_30fps_1280_960_HFOV39), cam);
		g_vcamFOV = fov;
		break;

	case 28:
		ret =
		    DoI2CWrite(pInfo, ov5640_setting_30fps_1280_960_HFOV28,
			       dim(ov5640_setting_30fps_1280_960_HFOV28), cam);
		g_vcamFOV = fov;
		break;

	default:
		pr_err("VCAM: Unsupported fov: %d\n", fov);
		ret = ERROR_NOT_SUPPORTED;
	}

	OV5640_stream_on(pInfo, cam);

	pInfo->cam = cam;
	schedule_work(&pInfo->nightmode_work);
	return ret;
}

static BOOL initCamera(PCAM_HW_INDEP_INFO pInfo, BOOL fullInit, CAM_NO cam)
{
	BOOL ret = ERROR_SUCCESS;

	ret =
	    DoI2CWrite(pInfo, ov5640_init_setting_9fps_5MP,
		       dim(ov5640_init_setting_9fps_5MP), cam);
	if (ret)
		return ret;

	if (pInfo->flip_image) {
		ret =
		    DoI2CWrite(pInfo, ov5640_flip_reg, dim(ov5640_flip_reg),
			       cam);
		if (ret)
			return ret;
	}

	if (pInfo->edge_enhancement) {
		ret =
		    DoI2CWrite(pInfo, ov5640_edge_enhancement,
			       dim(ov5640_edge_enhancement), cam);
		if (ret)
			return ret;
	}

	ret = OV5640_set_fov(pInfo, cam, 54);
	if (ret)
		return ret;

	return ret;
}

BOOL OV5640_reinit(PCAM_HW_INDEP_INFO pInfo)
{
	if (!init)
		return FALSE;

	OV5640_set_5MP(pInfo, g_camera);

	OV5640_set_fov(pInfo, g_camera, g_vcamFOV);

	return TRUE;
}

BOOL OV5640_Init(PCAM_HW_INDEP_INFO pInfo)
{
	INIT_WORK(&pInfo->nightmode_work, nightmode_on_off_work);

	if (pInfo->cameraI2CAddress[1] == 0)	//Only 1 active camera
		g_camera = CAM_1;
	initCamera(pInfo, TRUE, g_camera);
	init = 1;
	return TRUE;
}

DWORD OV5640_IOControl(PCAM_HW_INDEP_INFO pInfo,
		       DWORD Ioctl, PUCHAR pBuf, PUCHAR pUserBuf)
{
	DWORD dwErr = ERROR_INVALID_PARAMETER;
	static BOOL bTestActive;

	switch (Ioctl) {
	case IOCTL_CAM_GET_TEST:
		{
			LOCK(pInfo);
			((VCAMIOCTLTEST *) pBuf)->bTestMode = bTestActive;
			dwErr = ERROR_SUCCESS;
			UNLOCK(pInfo);
		}
		break;

	case IOCTL_CAM_SET_TEST:
		{
			LOCK(pInfo);
			bTestActive =
			    (((VCAMIOCTLTEST *) pBuf)->bTestMode != 0);
			dwErr = ERROR_SUCCESS;
			UNLOCK(pInfo);
		}
		break;

	case IOCTL_CAM_GET_ACTIVE:
		{
			VCAMIOCTLACTIVE *pVcamIoctl = (VCAMIOCTLACTIVE *) pBuf;

			LOCK(pInfo);
			pVcamIoctl->bActive = bCamActive[CAM_1]
			    || bCamActive[CAM_2];
			dwErr = ERROR_SUCCESS;
			UNLOCK(pInfo);
		}
		break;

	case IOCTL_CAM_INIT:
		dwErr = ERROR_SUCCESS;
		break;

	case IOCTL_CAM_SET_ACTIVE:
	case IOCTL_CAM_SET_2ND_ACTIVE:
		{
			BOOL bNewActive;
			BOOL res = TRUE;
			CAM_NO cam =
			    (Ioctl == IOCTL_CAM_SET_ACTIVE) ? CAM_1 : CAM_2;
			LOCK(pInfo);
			bNewActive = (((VCAMIOCTLACTIVE *) pBuf)->bActive != 0);

			if (res) {
				bCamActive[cam] = bNewActive;
				pr_err("bCamActive for cam %d now %d\n", cam,
				       bCamActive[cam]);
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
			//set fast exposure to compensate for led brightness
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
				//set camera to 5MP full size mode
				dwErr = OV5640_set_5MP(pInfo, g_camera);
				msleep(800);
				break;

			case VCAM_DRAFT:
				//restore last known fov
				dwErr =
				    OV5640_set_fov(pInfo, g_camera, g_vcamFOV);
				// must wait for auto exposure control (AEC)
				// and auto gain control (AGC) to adjust image brightness
				msleep(500);
				break;

			case VCAM_UNDEFINED:
			case VCAM_RESET:
			default:
				pr_err
				    ("VCAM Unsupported IOCTL_CAM_SET_CAMMODE %d\n",
				     pMode->eCamMode);
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
		{
			LOCK(pInfo);
			((VCAMIOCTLFOV *) pBuf)->fov = g_vcamFOV;
			dwErr = ERROR_SUCCESS;
			UNLOCK(pInfo);
		}
		break;

	default:
		pr_err("VCAM Unsupported IOCTL code %lu\n", Ioctl);
		dwErr = ERROR_NOT_SUPPORTED;
		break;
	}

	return dwErr;
}
