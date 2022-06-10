#ifndef OV5640_PARALELL_CSI_H
#define OV5640_PARALELL_CSI_H
/*
 * General initialization executed once at power on
 *
 *
 */
static struct reg_value ov5640_init_settings_wince[] = {
	{0x3103, 0x11}, //SCCB system ctrl
	{0x3008, 0x42}, //System root divider
	{0x3103, 0x03}, //SCCB system ctrl
	{0x3017, 0xff}, //VSYNC I/O contril
	{0x3018, 0xff}, //Pad output enable
	{0x3034, 0x1a}, //SC PLL control
	{0x3035, 0x21}, //System clock divider
	{0x3037, 0x13}, //PLL pre-divider
	{0x3108, 0x01}, //System root divider
	{0x302d, 0x60}, //System ctrl

	{0x3630, 0x36}, {0x3631, 0x0e},//undoc start
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

static struct reg_value ov5640_setting_15fps_640_480[] = {
    //VGA(YUV) 15fps
	//Crop 2304 x 1536 output 640 x 480 setup
	{0x3212, 0x01}, //Enable group 1
	{0x3036, 0xbf}, //PLL multiplier
	{0x3a00, 0x7c}, //AEC ctrl, night mode
	{0x3821, 0x01}, //Mirror and binning
	{0x3814, 0x31}, {0x3815, 0x31}, //Timing x and y inc
	{0x3800, 0x00}, {0x3801, 0x99}, // X address start = 0x99
	{0x3802, 0x00}, {0x3803, 0x6c}, // Y address start = 0x6c
	{0x3804, 0x09}, {0x3805, 0x98}, // X address end   = 0x998
	{0x3806, 0x07}, {0x3807, 0x2b}, // Y address end   = 0x72b
	{0x3808, 0x02}, {0x3809, 0x80}, // DVP width  output size = 0x280 (640)
	{0x380a, 0x01}, {0x380b, 0xe0}, // DVP height output size = 0x1e0 (480)
	{0x380c, 0x0b}, {0x380d, 0x00}, // Total horizontal size = 0x768
	{0x380e, 0x07}, {0x380f, 0x00}, // Total vertical size  =  0x3d8
	{0x3810, 0x00}, {0x3811, 0x08}, // ISP horizontal offset = 0x8
	{0x3812, 0x00}, {0x3813, 0x04}, // ISP vertical   offset = 0x4

	{0x3618, 0x00}, {0x3612, 0x29}, //undoc
	{0x3708, 0x66}, {0x3709, 0x12},
	{0x370c, 0x03}, {0x4837, 0x22},

	{0x3a02, 0x09}, {0x3a03, 0xff}, //AEC Max Expo (60 Hz)
	{0x3a0e, 0x03}, {0x3a0d, 0x04}, //AEC ctrl
	{0x3a14, 0x09}, {0x3a15, 0xff}, //AEC Max Expo (50 Hz)
	{0x3000, 0x00}, {0x3002, 0x1c}, //System reset
	{0x3004, 0xff}, {0x3006, 0xc3}, //Clock enable
	{0x5684, 0x05}, {0x5685, 0x00}, //AVG x-window reg
	{0x5686, 0x03}, {0x5687, 0xc0}, //AVG y-window reg
	{0x3212, 0x11}, //End group 1
	{0x3212, 0xa1}, //Lanch group 1
};
static struct reg_value ov5640_setting_9fps_3MP[] =
{
    //MEG3_JPG 15fps (Mode2)
	//crop 2304 x 1536 output 2048 x 1536 setup
	{0x3212, 0x00}, //Enable group 0
	{0x3036, 0x69}, //PLL multiplier
	{0x3a00, 0x78}, //AEC ctrl
	{0x3821, 0x21}, //Mirror, binning, Jpeg enable
	{0x3814, 0x11}, {0x3815, 0x11}, //Timing x and y inc
	{0x3800, 0x00}, {0x3801, 0x90}, // X address start = 0x090
	{0x3802, 0x00}, {0x3803, 0x66}, // Y address start = 0x066
	{0x3804, 0x09}, {0x3805, 0xaF}, // X address end   = 0x9af
	{0x3806, 0x07}, {0x3807, 0x31}, // Y address end   = 0x731
	{0x3808, 0x08}, {0x3809, 0x00}, // DVP width  output size = 0x800 (2048)
	{0x380a, 0x06}, {0x380b, 0x00}, // DVP height output size = 0x600 (1536)
	{0x380c, 0x0b}, {0x380d, 0x1c}, // Total horizontal size = 0xb1c
	{0x380e, 0x07}, {0x380f, 0xb0}, // Total vertical size  =  0x7b0
	{0x3810, 0x00}, {0x3811, 0x1e}, // ISP horizontal offset = 0x1e
	{0x3812, 0x00}, {0x3813, 0x0f}, // ISP vertical   offset = 0xf

	{0x3618, 0x04}, {0x3612, 0x2b}, //undoc
	{0x3708, 0x63}, {0x3709, 0x12},
	{0x370c, 0x00}, {0x3824, 0x04},

	{0x3a02, 0x07}, {0x3a03, 0xb0}, // AEC Max Expo (60 Hz)
	{0x3a0e, 0x06}, {0x3a0d, 0x08}, //AEC ctrl
	{0x3a14, 0x07}, {0x3a15, 0xb0}, //AEC Max Expo (50 Hz)
	{0x3000, 0x00}, {0x3002, 0x00}, //System reset
	{0x3004, 0xff}, {0x3006, 0xff}, //Clock enable
	{0x5684, 0x0a}, {0x5685, 0x20}, //AVG x-window reg
	{0x5686, 0x07}, {0x5687, 0x98}, //AVG y-window reg
	{0x3212, 0x10}, //End group 0
	{0x3212, 0xa0}, //Lanch group 0
};

#endif // OV5640_PARALELL_CSI_H
