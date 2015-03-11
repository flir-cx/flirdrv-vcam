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

typedef enum {CAM_1, CAM_2, CAM_ALL} CAM_NO;

// Local definitions

#define DEBUG_TMO           0

// Local functions

// Local variables
static BOOL bCamActive[CAM_ALL] = { TRUE, FALSE };

/* dataformat to send to camera over i2c, 16 bits register and 16 bits of data
  nr of bytes to send, register high byte, register low byte, data high byte, data low byte*/
static const UCHAR I2CDataInitPart1[][4] =
{
    { 3, 0x31, 0x03,0x11 },
    { 3, 0x30, 0x08,0x82 },
    { 3, 0x30, 0x08,0x42 },
    { 3, 0x31, 0x03,0x03 },
    { 3, 0x30, 0x17,0x00 },
    { 3, 0x30, 0x18,0x00 },
    { 3, 0x30, 0x34,0x18 },
    { 3, 0x30, 0x35,0x11 },
    { 3, 0x30, 0x36,0x54 },
    { 3, 0x30, 0x37,0x13 },
    { 3, 0x31, 0x08,0x01 },
    { 3, 0x36, 0x30,0x36 },
    { 3, 0x36, 0x31,0x0e },
    { 3, 0x36, 0x32,0xe2 },
    { 3, 0x36, 0x33,0x12 },
    { 3, 0x36, 0x21,0xe0 },
    { 3, 0x37, 0x04,0xa0 },
    { 3, 0x37, 0x03,0x5a },
    { 3, 0x37, 0x15,0x78 },
    { 3, 0x37, 0x17,0x01 },
    { 3, 0x37, 0x0b,0x60 },
    { 3, 0x37, 0x05,0x1a },
    { 3, 0x39, 0x05,0x02 },
    { 3, 0x39, 0x06,0x10 },
    { 3, 0x39, 0x01,0x0a },
    { 3, 0x37, 0x31,0x12 },
    { 3, 0x36, 0x00,0x08 },
    { 3, 0x36, 0x01,0x33 },
    { 3, 0x30, 0x2d,0x60 },
    { 3, 0x36, 0x20,0x52 },
    { 3, 0x37, 0x1b,0x20 },
    { 3, 0x47, 0x1c,0x50 },
    { 3, 0x3a, 0x13,0x43 },
    { 3, 0x3a, 0x18,0x00 },
    { 3, 0x3a, 0x19,0xf8 },
    { 3, 0x36, 0x35,0x13 },
    { 3, 0x36, 0x36,0x03 },
    { 3, 0x36, 0x34,0x40 },
    { 3, 0x36, 0x22,0x01 },
    { 3, 0x3c, 0x01,0x34 },
    { 3, 0x3c, 0x04,0x28 },
    { 3, 0x3c, 0x05,0x98 },
    { 3, 0x3c, 0x06,0x00 },
    { 3, 0x3c, 0x07,0x07 },
    { 3, 0x3c, 0x08,0x00 },
    { 3, 0x3c, 0x09,0x1c },
    { 3, 0x3c, 0x0a,0x9c },
    { 3, 0x3c, 0x0b,0x40 },
    { 3, 0x38, 0x20,0x40 },
    { 3, 0x38, 0x21,0x06 },
    { 3, 0x38, 0x14,0x11 },
    { 3, 0x38, 0x15,0x11 },
    { 3, 0x38, 0x00,0x00 },
    { 3, 0x38, 0x01,0x00 },
    { 3, 0x38, 0x02,0x00 },
    { 3, 0x38, 0x03,0x00 },
    { 3, 0x38, 0x04,0x0a },
    { 3, 0x38, 0x05,0x3f },
    { 3, 0x38, 0x06,0x07 },
    { 3, 0x38, 0x07,0x9f },
    { 3, 0x38, 0x08,0x0a },
    { 3, 0x38, 0x09,0x20 },
    { 3, 0x38, 0x0a,0x07 },
    { 3, 0x38, 0x0b,0x98 },
    { 3, 0x38, 0x0c,0x0b },
    { 3, 0x38, 0x0d,0x1c },
    { 3, 0x38, 0x0e,0x07 },
    { 3, 0x38, 0x0f,0xb0 },
    { 3, 0x38, 0x10,0x00 },
    { 3, 0x38, 0x11,0x10 },
    { 3, 0x38, 0x12,0x00 },
    { 3, 0x38, 0x13,0x04 },
    { 3, 0x36, 0x18,0x04 },
    { 3, 0x36, 0x12,0x2b },
    { 3, 0x37, 0x08,0x64 },
    { 3, 0x37, 0x09,0x12 },
    { 3, 0x37, 0x0c,0x00 },
    { 3, 0x3a, 0x02,0x07 },
    { 3, 0x3a, 0x03,0xb0 },
    { 3, 0x3a, 0x08,0x01 },
    { 3, 0x3a, 0x09,0x27 },
    { 3, 0x3a, 0x0a,0x00 },
    { 3, 0x3a, 0x0b,0xf6 },
    { 3, 0x3a, 0x0e,0x06 },
    { 3, 0x3a, 0x0d,0x08 },
    { 3, 0x3a, 0x14,0x07 },
    { 3, 0x3a, 0x15,0xb0 },
    { 3, 0x40, 0x01,0x02 },
    { 3, 0x40, 0x04,0x06 },
    { 3, 0x30, 0x00,0x00 },
    { 3, 0x30, 0x02,0x1c },
    { 3, 0x30, 0x04,0xff },
    { 3, 0x30, 0x06,0xc3 },
    { 3, 0x30, 0x0e,0x45 },
    { 3, 0x30, 0x2e,0x08 },
    { 3, 0x43, 0x00,0x30 },
    { 3, 0x48, 0x37,0x0a },
    { 3, 0x50, 0x1f,0x00 },
    { 3, 0x44, 0x0e,0x00 },
    { 3, 0x50, 0x00,0xa7 },
    { 3, 0x50, 0x01,0x83 },
    { 3, 0x51, 0x80,0xff },
    { 3, 0x51, 0x81,0xf2 },
    { 3, 0x51, 0x82,0x00 },
    { 3, 0x51, 0x83,0x14 },
    { 3, 0x51, 0x84,0x25 },
    { 3, 0x51, 0x85,0x24 },
    { 3, 0x51, 0x86,0x09 },
    { 3, 0x51, 0x87,0x09 },
    { 3, 0x51, 0x88,0x09 },
    { 3, 0x51, 0x89,0x75 },
    { 3, 0x51, 0x8a,0x54 },
    { 3, 0x51, 0x8b,0xe0 },
    { 3, 0x51, 0x8c,0xb2 },
    { 3, 0x51, 0x8d,0x42 },
    { 3, 0x51, 0x8e,0x3d },
    { 3, 0x51, 0x8f,0x56 },
    { 3, 0x51, 0x90,0x46 },
    { 3, 0x51, 0x91,0xf8 },
    { 3, 0x51, 0x92,0x04 },
    { 3, 0x51, 0x93,0x70 },
    { 3, 0x51, 0x94,0xf0 },
    { 3, 0x51, 0x95,0xf0 },
    { 3, 0x51, 0x96,0x03 },
    { 3, 0x51, 0x97,0x01 },
    { 3, 0x51, 0x98,0x04 },
    { 3, 0x51, 0x99,0x12 },
    { 3, 0x51, 0x9a,0x04 },
    { 3, 0x51, 0x9b,0x00 },
    { 3, 0x51, 0x9c,0x06 },
    { 3, 0x51, 0x9d,0x82 },
    { 3, 0x51, 0x9e,0x38 },
    { 3, 0x53, 0x81,0x1e },
    { 3, 0x53, 0x82,0x5b },
    { 3, 0x53, 0x83,0x08 },
    { 3, 0x53, 0x84,0x0a },
    { 3, 0x53, 0x85,0x7e },
    { 3, 0x53, 0x86,0x88 },
    { 3, 0x53, 0x87,0x7c },
    { 3, 0x53, 0x88,0x6c },
    { 3, 0x53, 0x89,0x10 },
    { 3, 0x53, 0x8a,0x01 },
    { 3, 0x53, 0x8b,0x98 },
    { 3, 0x53, 0x00,0x08 },
    { 3, 0x53, 0x01,0x30 },
    { 3, 0x53, 0x02,0x10 },
    { 3, 0x53, 0x03,0x00 },
    { 3, 0x53, 0x04,0x08 },
    { 3, 0x53, 0x05,0x30 },
    { 3, 0x53, 0x06,0x08 },
    { 3, 0x53, 0x07,0x16 },
    { 3, 0x53, 0x09,0x08 },
    { 3, 0x53, 0x0a,0x30 },
    { 3, 0x53, 0x0b,0x04 },
    { 3, 0x53, 0x0c,0x06 },
    { 3, 0x54, 0x80,0x01 },
    { 3, 0x54, 0x81,0x08 },
    { 3, 0x54, 0x82,0x14 },
    { 3, 0x54, 0x83,0x28 },
    { 3, 0x54, 0x84,0x51 },
    { 3, 0x54, 0x85,0x65 },
    { 3, 0x54, 0x86,0x71 },
    { 3, 0x54, 0x87,0x7d },
    { 3, 0x54, 0x88,0x87 },
    { 3, 0x54, 0x89,0x91 },
    { 3, 0x54, 0x8a,0x9a },
    { 3, 0x54, 0x8b,0xaa },
    { 3, 0x54, 0x8c,0xb8 },
    { 3, 0x54, 0x8d,0xcd },
    { 3, 0x54, 0x8e,0xdd },
    { 3, 0x54, 0x8f,0xea },
    { 3, 0x54, 0x90,0x1d },
    { 3, 0x55, 0x80,0x02 },
    { 3, 0x55, 0x83,0x40 },
    { 3, 0x55, 0x84,0x10 },
    { 3, 0x55, 0x89,0x10 },
    { 3, 0x55, 0x8a,0x00 },
    { 3, 0x55, 0x8b,0xf8 },
    { 3, 0x58, 0x00,0x23 },
    { 3, 0x58, 0x01,0x14 },
    { 3, 0x58, 0x02,0x0f },
    { 3, 0x58, 0x03,0x0f },
    { 3, 0x58, 0x04,0x12 },
    { 3, 0x58, 0x05,0x26 },
    { 3, 0x58, 0x06,0x0c },
    { 3, 0x58, 0x07,0x08 },
    { 3, 0x58, 0x08,0x05 },
    { 3, 0x58, 0x09,0x05 },
    { 3, 0x58, 0x0a,0x08 },
    { 3, 0x58, 0x0b,0x0d },
    { 3, 0x58, 0x0c,0x08 },
    { 3, 0x58, 0x0d,0x03 },
    { 3, 0x58, 0x0e,0x00 },
    { 3, 0x58, 0x0f,0x00 },
    { 3, 0x58, 0x10,0x03 },
    { 3, 0x58, 0x11,0x09 },
    { 3, 0x58, 0x12,0x07 },
    { 3, 0x58, 0x13,0x03 },
    { 3, 0x58, 0x14,0x00 },
    { 3, 0x58, 0x15,0x01 },
    { 3, 0x58, 0x16,0x03 },
    { 3, 0x58, 0x17,0x08 },
    { 3, 0x58, 0x18,0x0d },
    { 3, 0x58, 0x19,0x08 },
    { 3, 0x58, 0x1a,0x05 },
    { 3, 0x58, 0x1b,0x06 },
    { 3, 0x58, 0x1c,0x08 },
    { 3, 0x58, 0x1d,0x0e },
    { 3, 0x58, 0x1e,0x29 },
    { 3, 0x58, 0x1f,0x17 },
    { 3, 0x58, 0x20,0x11 },
    { 3, 0x58, 0x21,0x11 },
    { 3, 0x58, 0x22,0x15 },
    { 3, 0x58, 0x23,0x28 },
    { 3, 0x58, 0x24,0x46 },
    { 3, 0x58, 0x25,0x26 },
    { 3, 0x58, 0x26,0x08 },
    { 3, 0x58, 0x27,0x26 },
    { 3, 0x58, 0x28,0x64 },
    { 3, 0x58, 0x29,0x26 },
    { 3, 0x58, 0x2a,0x24 },
    { 3, 0x58, 0x2b,0x22 },
    { 3, 0x58, 0x2c,0x24 },
    { 3, 0x58, 0x2d,0x24 },
    { 3, 0x58, 0x2e,0x06 },
    { 3, 0x58, 0x2f,0x22 },
    { 3, 0x58, 0x30,0x40 },
    { 3, 0x58, 0x31,0x42 },
    { 3, 0x58, 0x32,0x24 },
    { 3, 0x58, 0x33,0x26 },
    { 3, 0x58, 0x34,0x24 },
    { 3, 0x58, 0x35,0x22 },
    { 3, 0x58, 0x36,0x22 },
    { 3, 0x58, 0x37,0x26 },
    { 3, 0x58, 0x38,0x44 },
    { 3, 0x58, 0x39,0x24 },
    { 3, 0x58, 0x3a,0x26 },
    { 3, 0x58, 0x3b,0x28 },
    { 3, 0x58, 0x3c,0x42 },
    { 3, 0x58, 0x3d,0xce },
    { 3, 0x50, 0x25,0x00 },
    { 3, 0x3a, 0x0f,0x30 },
    { 3, 0x3a, 0x10,0x28 },
    { 3, 0x3a, 0x1b,0x30 },
    { 3, 0x3a, 0x1e,0x26 },
    { 3, 0x3a, 0x11,0x60 },
    { 3, 0x3a, 0x1f,0x14 },
    { 3, 0x30, 0x08,0x02 }
};
static  const UCHAR ov5640_init_setting_30fps_VGA[][4] = {

    {3, 0x31,0x03, 0x11}, {3,0x30,0x08, 0x82}, {3,0x30,0x08, 0x42},
    {3, 0x31,0x03, 0x03}, {3,0x30,0x17, 0x00}, {3,0x30,0x18, 0x00},
    {3, 0x30,0x34, 0x18}, {3,0x30,0x35, 0x14}, {3,0x30,0x36, 0x38},
    {3, 0x30,0x37, 0x13}, {3,0x31,0x08, 0x01}, {3,0x36,0x30, 0x36},
    {3, 0x36,0x31, 0x0e}, {3,0x36,0x32, 0xe2}, {3,0x36,0x33, 0x12},
    {3, 0x36,0x21, 0xe0}, {3,0x37,0x04, 0xa0}, {3,0x37,0x03, 0x5a},
    {3, 0x37,0x15, 0x78}, {3,0x37,0x17, 0x01}, {3,0x37,0x0b, 0x60},
    {3, 0x37,0x05, 0x1a}, {3,0x39,0x05, 0x02}, {3,0x39,0x06, 0x10},
    {3, 0x39,0x01, 0x0a}, {3,0x37,0x31, 0x12}, {3,0x36,0x00, 0x08},
    {3, 0x36,0x01, 0x33}, {3,0x30,0x2d, 0x60}, {3,0x36,0x20, 0x52},
    {3, 0x37,0x1b, 0x20}, {3,0x47,0x1c, 0x50}, {3,0x3a,0x13, 0x43},
    {3, 0x3a,0x18, 0x00}, {3,0x3a,0x19, 0xf8}, {3,0x36,0x35, 0x13},
    {3, 0x36,0x36, 0x03}, {3,0x36,0x34, 0x40}, {3,0x36,0x22, 0x01},
    {3, 0x3c,0x01, 0xa4}, {3,0x3c,0x04, 0x28}, {3,0x3c,0x05, 0x98},
    {3, 0x3c,0x06, 0x00}, {3,0x3c,0x07, 0x08}, {3,0x3c,0x08, 0x00},
    {3, 0x3c,0x09, 0x1c}, {3,0x3c,0x0a, 0x9c}, {3,0x3c,0x0b, 0x40},
    {3, 0x38,0x20, 0x41}, {3,0x38,0x21, 0x07}, {3,0x38,0x14, 0x31},
    {3, 0x38,0x15, 0x31}, {3,0x38,0x00, 0x00}, {3,0x38,0x01, 0x00},
    {3, 0x38,0x02, 0x00}, {3,0x38,0x03, 0x04}, {3,0x38,0x04, 0x0a},
    {3, 0x38,0x05, 0x3f}, {3,0x38,0x06, 0x07}, {3,0x38,0x07, 0x9b},
    {3, 0x38,0x08, 0x02}, {3,0x38,0x09, 0x80}, {3,0x38,0x0a, 0x01},
    {3, 0x38,0x0b, 0xe0}, {3,0x38,0x0c, 0x07}, {3,0x38,0x0d, 0x68},
    {3, 0x38,0x0e, 0x03}, {3,0x38,0x0f, 0xd8}, {3,0x38,0x10, 0x00},
    {3, 0x38,0x11, 0x10}, {3,0x38,0x12, 0x00}, {3,0x38,0x13, 0x06},
    {3, 0x36,0x18, 0x00}, {3,0x36,0x12, 0x29}, {3,0x37,0x08, 0x64},
    {3, 0x37,0x09, 0x52}, {3,0x37,0x0c, 0x03}, {3,0x3a,0x02, 0x03},
    {3, 0x3a,0x03, 0xd8}, {3,0x3a,0x08, 0x01}, {3,0x3a,0x09, 0x27},
    {3, 0x3a,0x0a, 0x00}, {3,0x3a,0x0b, 0xf6}, {3,0x3a,0x0e, 0x03},
    {3, 0x3a,0x0d, 0x04}, {3,0x3a,0x14, 0x03}, {3,0x3a,0x15, 0xd8},
    {3, 0x40,0x01, 0x02}, {3,0x40,0x04, 0x02}, {3,0x30,0x00, 0x00},
    {3, 0x30,0x02, 0x1c}, {3,0x30,0x04, 0xff}, {3,0x30,0x06, 0xc3},
    {3, 0x30,0x0e, 0x45}, {3,0x30,0x2e, 0x08}, {3,0x43,0x00, 0x3f},
    {3, 0x50,0x1f, 0x00}, {3,0x47,0x13, 0x03}, {3,0x44,0x07, 0x04},
    {3, 0x44,0x0e, 0x00}, {3,0x46,0x0b, 0x35}, {3,0x46,0x0c, 0x22},
    {3, 0x48,0x37, 0x0a}, {3,0x48,0x00, 0x04}, {3,0x38,0x24, 0x02},
    {3, 0x50,0x00, 0xa7}, {3,0x50,0x01, 0xa3}, {3,0x51,0x80, 0xff},
    {3, 0x51,0x81, 0xf2}, {3,0x51,0x82, 0x00}, {3,0x51,0x83, 0x14},
    {3, 0x51,0x84, 0x25}, {3,0x51,0x85, 0x24}, {3,0x51,0x86, 0x09},
    {3, 0x51,0x87, 0x09}, {3,0x51,0x88, 0x09}, {3,0x51,0x89, 0x88},
    {3, 0x51,0x8a, 0x54}, {3,0x51,0x8b, 0xee}, {3,0x51,0x8c, 0xb2},
    {3, 0x51,0x8d, 0x50}, {3,0x51,0x8e, 0x34}, {3,0x51,0x8f, 0x6b},
    {3, 0x51,0x90, 0x46}, {3,0x51,0x91, 0xf8}, {3,0x51,0x92, 0x04},
    {3, 0x51,0x93, 0x70}, {3,0x51,0x94, 0xf0}, {3,0x51,0x95, 0xf0},
    {3, 0x51,0x96, 0x03}, {3,0x51,0x97, 0x01}, {3,0x51,0x98, 0x04},
    {3, 0x51,0x99, 0x6c}, {3,0x51,0x9a, 0x04}, {3,0x51,0x9b, 0x00},
    {3, 0x51,0x9c, 0x09}, {3,0x51,0x9d, 0x2b}, {3,0x51,0x9e, 0x38},
    {3, 0x53,0x81, 0x1e}, {3,0x53,0x82, 0x5b}, {3,0x53,0x83, 0x08},
    {3, 0x53,0x84, 0x0a}, {3,0x53,0x85, 0x7e}, {3,0x53,0x86, 0x88},
    {3, 0x53,0x87, 0x7c}, {3,0x53,0x88, 0x6c}, {3,0x53,0x89, 0x10},
    {3, 0x53,0x8a, 0x01}, {3,0x53,0x8b, 0x98}, {3,0x53,0x00, 0x08},
    {3, 0x53,0x01, 0x30}, {3,0x53,0x02, 0x10}, {3,0x53,0x03, 0x00},
    {3, 0x53,0x04, 0x08}, {3,0x53,0x05, 0x30}, {3,0x53,0x06, 0x08},
    {3, 0x53,0x07, 0x16}, {3,0x53,0x09, 0x08}, {3,0x53,0x0a, 0x30},
    {3, 0x53,0x0b, 0x04}, {3,0x53,0x0c, 0x06}, {3,0x54,0x80, 0x01},
    {3, 0x54,0x81, 0x08}, {3,0x54,0x82, 0x14}, {3,0x54,0x83, 0x28},
    {3, 0x54,0x84, 0x51}, {3,0x54,0x85, 0x65}, {3,0x54,0x86, 0x71},
    {3, 0x54,0x87, 0x7d}, {3,0x54,0x88, 0x87}, {3,0x54,0x89, 0x91},
    {3, 0x54,0x8a, 0x9a}, {3,0x54,0x8b, 0xaa}, {3,0x54,0x8c, 0xb8},
    {3, 0x54,0x8d, 0xcd}, {3,0x54,0x8e, 0xdd}, {3,0x54,0x8f, 0xea},
    {3, 0x54,0x90, 0x1d}, {3,0x55,0x80, 0x02}, {3,0x55,0x83, 0x40},
    {3, 0x55,0x84, 0x10}, {3,0x55,0x89, 0x10}, {3,0x55,0x8a, 0x00},
    {3, 0x55,0x8b, 0xf8}, {3,0x58,0x00, 0x23}, {3,0x58,0x01, 0x14},
    {3, 0x58,0x02, 0x0f}, {3,0x58,0x03, 0x0f}, {3,0x58,0x04, 0x12},
    {3, 0x58,0x05, 0x26}, {3,0x58,0x06, 0x0c}, {3,0x58,0x07, 0x08},
    {3, 0x58,0x08, 0x05}, {3,0x58,0x09, 0x05}, {3,0x58,0x0a, 0x08},
    {3, 0x58,0x0b, 0x0d}, {3,0x58,0x0c, 0x08}, {3,0x58,0x0d, 0x03},
    {3, 0x58,0x0e, 0x00}, {3,0x58,0x0f, 0x00}, {3,0x58,0x10, 0x03},
    {3, 0x58,0x11, 0x09}, {3,0x58,0x12, 0x07}, {3,0x58,0x13, 0x03},
    {3, 0x58,0x14, 0x00}, {3,0x58,0x15, 0x01}, {3,0x58,0x16, 0x03},
    {3, 0x58,0x17, 0x08}, {3,0x58,0x18, 0x0d}, {3,0x58,0x19, 0x08},
    {3, 0x58,0x1a, 0x05}, {3,0x58,0x1b, 0x06}, {3,0x58,0x1c, 0x08},
    {3, 0x58,0x1d, 0x0e}, {3,0x58,0x1e, 0x29}, {3,0x58,0x1f, 0x17},
    {3, 0x58,0x20, 0x11}, {3,0x58,0x21, 0x11}, {3,0x58,0x22, 0x15},
    {3, 0x58,0x23, 0x28}, {3,0x58,0x24, 0x46}, {3,0x58,0x25, 0x26},
    {3, 0x58,0x26, 0x08}, {3,0x58,0x27, 0x26}, {3,0x58,0x28, 0x64},
    {3, 0x58,0x29, 0x26}, {3,0x58,0x2a, 0x24}, {3,0x58,0x2b, 0x22},
    {3, 0x58,0x2c, 0x24}, {3,0x58,0x2d, 0x24}, {3,0x58,0x2e, 0x06},
    {3, 0x58,0x2f, 0x22}, {3,0x58,0x30, 0x40}, {3,0x58,0x31, 0x42},
    {3, 0x58,0x32, 0x24}, {3,0x58,0x33, 0x26}, {3,0x58,0x34, 0x24},
    {3, 0x58,0x35, 0x22}, {3,0x58,0x36, 0x22}, {3,0x58,0x37, 0x26},
    {3, 0x58,0x38, 0x44}, {3,0x58,0x39, 0x24}, {3,0x58,0x3a, 0x26},
    {3, 0x58,0x3b, 0x28}, {3,0x58,0x3c, 0x42}, {3,0x58,0x3d, 0xce},
    {3, 0x50,0x25, 0x00}, {3,0x3a,0x0f, 0x30}, {3,0x3a,0x10, 0x28},
    {3, 0x3a,0x1b, 0x30}, {3,0x3a,0x1e, 0x26}, {3,0x3a,0x11, 0x60},
    {3, 0x3a,0x1f, 0x14}, {3,0x30,0x08, 0x02}, {3,0x3c,0x00, 0x04},
};

static const UCHAR ov5640_setting_30fps_VGA_640_480[][4] = {

    {3, 0x30,0x35, 0x14}, {3,0x30,0x36, 0x38}, {3,0x3c,0x07, 0x08},
    {3, 0x3c,0x09, 0x1c}, {3,0x3c,0x0a, 0x9c}, {3,0x3c,0x0b, 0x40},
    {3, 0x38,0x20, 0x41}, {3,0x38,0x21, 0x07}, {3,0x38,0x14, 0x31},
    {3, 0x38,0x15, 0x31}, {3,0x38,0x00, 0x00}, {3,0x38,0x01, 0x00},
    {3, 0x38,0x02, 0x00}, {3,0x38,0x03, 0x04}, {3,0x38,0x04, 0x0a},
    {3, 0x38,0x05, 0x3f}, {3,0x38,0x06, 0x07}, {3,0x38,0x07, 0x9b},
    {3, 0x38,0x08, 0x02}, {3,0x38,0x09, 0x80}, {3,0x38,0x0a, 0x01},
    {3, 0x38,0x0b, 0xe0}, {3,0x38,0x0c, 0x07}, {3,0x38,0x0d, 0x68},
    {3, 0x38,0x0e, 0x04}, {3,0x38,0x0f, 0x38}, {3,0x38,0x10, 0x00},
    {3, 0x38,0x11, 0x10}, {3,0x38,0x12, 0x00}, {3,0x38,0x13, 0x06},
    {3, 0x36,0x18, 0x00}, {3,0x36,0x12, 0x29}, {3,0x37,0x08, 0x64},
    {3, 0x37,0x09, 0x52}, {3,0x37,0x0c, 0x03}, {3,0x3a,0x02, 0x03},
    {3, 0x3a,0x03, 0xd8}, {3,0x3a,0x08, 0x01}, {3,0x3a,0x09, 0x0e},
    {3, 0x3a,0x0a, 0x00}, {3,0x3a,0x0b, 0xf6}, {3,0x3a,0x0e, 0x03},
    {3, 0x3a,0x0d, 0x04}, {3,0x3a,0x14, 0x03}, {3,0x3a,0x15, 0xd8},
    {3, 0x40,0x01, 0x02}, {3,0x40,0x04, 0x02}, {3,0x47,0x13, 0x03},
    {3, 0x44,0x07, 0x04}, {3,0x46,0x0b, 0x35}, {3,0x46,0x0c, 0x22},
    {3, 0x38,0x24, 0x02}, {3,0x50,0x01, 0xa3}, {3,0x35,0x03, 0x00},
};


static BOOL DoI2CWrite (PCAM_HW_INDEP_INFO pInfo,
						const UCHAR *buf,
						USHORT elements,
						CAM_NO camera)
{
	struct i2c_msg msgs[2];
    DWORD ret;
    int i;
    int retries = 50;
    DWORD cam;
    const UCHAR *ptr;
    DWORD cam_first = (camera == CAM_2) ? CAM_2 : CAM_1;
    DWORD cam_last = (camera == CAM_1) ? CAM_1 : CAM_2;

    for (cam=cam_first; cam<=cam_last; cam++)
    {
        // Check if camera in use
        msgs[0].addr = pInfo->cameraI2CAddress[cam] >> 1;
        if (msgs[0].addr == 0)
            continue;
        msgs[1].addr = msgs[0].addr;

        ptr = buf;
        msgs[0].flags = 0;

        for (i=0; i<elements; i++)
        {
            msgs[0].len = ptr[0];
            msgs[0].buf = (UCHAR *)(ptr+1);
            ret = i2c_transfer(pInfo->hI2C, msgs, 1);

            if (ret <= 0)
            {
                if (retries-- <= 0)
                {
                    pr_err("DoI2CWrite failing on element %d of %d\n", i, elements);
                    return FALSE;       // Too many errors, give up
                }
                msleep (10);
                i--;
                continue;
            }
            ptr+=4;
        }
    }

    return TRUE;
}
#if 0
static BOOL DoI2CRead (PCAM_HW_INDEP_INFO pInfo, USHORT *result, USHORT reg, CAM_NO camera)
{
	struct i2c_msg msgs[2];
    DWORD ret = 0;
    UCHAR cmd[2];
    UCHAR stat[2];

    // Check if camera in use
    msgs[0].addr = BSPGetCameraI2CAddress(camera) >> 1;
    if (msgs[0].addr == 0)
        return TRUE;
    msgs[1].addr = msgs[0].addr;

    msgs[0].flags = 0;
    msgs[0].len = 2;
    msgs[0].buf = cmd;
    msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
    msgs[1].len = 2;
    msgs[1].buf = stat;

    cmd[0] = (UCHAR)(reg >> 8);
    cmd[1] = (UCHAR)(reg & 0xFF);

    ret = i2c_transfer(pInfo->hI2C, msgs, 2);

    if (ret > 0)
    {
        *result = (stat[0] << 8) | stat[1];
    }
    else
    {
        pr_err("DoI2CRead failing reading reg %d\n", reg);
    }
    return ret;
}
#endif

static BOOL initCamera (PCAM_HW_INDEP_INFO pInfo, BOOL fullInit, CAM_NO cam)
{
    BOOL ret = TRUE;

   // ret = DoI2CWrite(pInfo, I2CDataInitPart1[0], dim(I2CDataInitPart1), cam);

    ret = DoI2CWrite(pInfo, ov5640_init_setting_30fps_VGA[0], dim(ov5640_init_setting_30fps_VGA), cam);
    ret = DoI2CWrite(pInfo, ov5640_setting_30fps_VGA_640_480[0], dim(ov5640_setting_30fps_VGA_640_480), cam);


	return ret;
}

BOOL OV5640_Init(PCAM_HW_INDEP_INFO pInfo)
{
    initCamera(pInfo, TRUE, CAM_ALL);
    /*if (bCamActive[CAM_1] == FALSE)
		DoI2CWrite(pInfo, I2CDataStandByEnter[0], dim(I2CDataStandByEnter), 0, 0, 0, CAM_1);
	if (bCamActive[CAM_2] == FALSE)
		DoI2CWrite(pInfo, I2CDataStandByEnter[0], dim(I2CDataStandByEnter), 0, 0, 0, CAM_2);
    */
    return TRUE;
}

DWORD OV5640_IOControl(PCAM_HW_INDEP_INFO pInfo,
                        DWORD  Ioctl,
                        PUCHAR pBuf,
                        PUCHAR pUserBuf)
{
    DWORD  dwErr = ERROR_INVALID_PARAMETER;
    static BOOL bTestActive;

    switch (Ioctl) 
	{
        case IOCTL_CAM_GET_TEST:   
			{
	   			LOCK(pInfo);
                ((VCAMIOCTLTEST *)pBuf)->bTestMode = bTestActive;
				dwErr = ERROR_SUCCESS;
        		UNLOCK(pInfo);
   			}
            break;

		case IOCTL_CAM_SET_TEST:
            {
			    LOCK(pInfo);
				bTestActive = (((VCAMIOCTLTEST *)pBuf)->bTestMode != 0);
				dwErr = ERROR_SUCCESS;
			    UNLOCK(pInfo);
            }
            break;

		case IOCTL_CAM_GET_ACTIVE:   
            {
				VCAMIOCTLACTIVE * pVcamIoctl = (VCAMIOCTLACTIVE *)pBuf;
    			LOCK(pInfo);
				pVcamIoctl->bActive = bCamActive[CAM_1] || bCamActive[CAM_2];
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
				CAM_NO cam = (Ioctl == IOCTL_CAM_SET_ACTIVE) ? CAM_1 : CAM_2;
    			LOCK(pInfo);
				bNewActive = (((VCAMIOCTLACTIVE *)pBuf)->bActive != 0);
				if (bNewActive != bCamActive[cam])
				{
					if (bNewActive)
					{
                    //	res = DoI2CWrite(pInfo, I2CDataStandByExit[0], dim(I2CDataStandByExit), 0, 0, 0, cam);

					}
					else
					{
                    //	res = DoI2CWrite(pInfo, I2CDataStandByEnter[0], dim(I2CDataStandByEnter), 0, 0, 0, cam);
					}
				}

				if (res)
				{
					bCamActive[cam] = bNewActive;
					pr_err("bCamActive for cam %d now %d\n", cam, bCamActive[cam]);
					dwErr = ERROR_SUCCESS;
				}
        		UNLOCK(pInfo);
            }
            break;

		case IOCTL_CAM_SET_FLASH:
            {
				VCAMIOCTLFLASH * pFlashData = (VCAMIOCTLFLASH *) pBuf;
			    LOCK(pInfo);

				if (pFlashData->bFlashOn)
				{
				}

				dwErr = pInfo->pSetTorchState(pInfo, pFlashData);
			    UNLOCK(pInfo);
            }
            break;

        case IOCTL_CAM_GRAB_STILL:
            {
              //  CAM_NO cam = (bCamActive[CAM_1] == TRUE) ? CAM_1 : CAM_2;

    			LOCK(pInfo);
                // Tell camera to switch to context B
                //DoI2CWrite(pInfo, I2CDataGrab[0], dim(I2CDataGrab), 0, 0, 0, cam);

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



