/***********************************************************************
 *                                                                     
 * Project: Balthazar
 * $Date: 2013/05/06 $
 * $Author: fhammars $
 *
 * $Id: //depot/Balthazar/Camera/OS/main/WINCE600/PLATFORM/COMMON/SRC/FLIR/VCAM/OV7740.c#4 $
 *
 * Description of file:
 *   Visual Camera Driver for OV7740 (Astra)
 *
 * Last check-in changelist:
 * $Change: 172358 $
 *
 * Copyright: FLIR Systems AB
 ***********************************************************************/

#include "flir_kernel_os.h"
#include "vcam_internal.h"
#include "i2cdev.h"
#include <linux/i2c.h>

// Local typedefs

// Local definitions

// Local functions

// Local variables

static BOOL bCamActive = TRUE;

// VCAM settings

static const UCHAR I2CDataInitPart1[][2] =
{
    { 0x12, 0x80 },
    { 0x13, 0x00 },
    { 0x11, 0x01 },
    { 0x12, 0x00 },
    { 0xD5, 0x10 },
    { 0x0C, 0xC2 }, /*mirror and flip function control, enable flip and mirror*/

/*
0x0C REG0C 0x02 RW
Bit[7]: Flip enable
0: Disable flip
1: Enable flip
Bit[6]: Mirror enable
0: Mirror disable
1: Mirror enable
Bit[5]: Not used
Bit[4]: YUV output, Y ?UV swap
0: YUYVYUYV
1: UYVYUYVY
Bit[3]: High 8-bit MSB and LSB swap
0: Output 
[Y9,Y8�Y3,Y2,Y1,Y0]
1: Output 
[Y2,Y3�Y8,Y9,Y1,Y0]
Bit[2:1]: Max exposure =
frame length � limit � 2
Bit[0]: Array color bar*/

    { 0x0D, 0x34 },
	{ 0x16, 0x01 }, /*move horizontal start position 1 byte*/
/*
0x16 REG16 0x00 RW
Bit[7]: Analog setting
Changing this value is not 
recommended
Bit[6]: Not used
Bit[5]: Sensor vertical output size
1 LSBs
Bit[4:3]: Sensor horizontal output size 2 
LSB
Bit[2]: Sensor vertical output start point 1 
LSB
Bit[1:0]: Sensor horizontal output start point 
2 LSBs
*/
    { 0x17, 0x25 },
	/*
0x17 AHSTART 0x2A RW
Sensor Horizontal Output Start Point 8 MSBs 
(LSBs in REG16[1:0] (0x16))
*/
    { 0x18, 0xA0 },
    { 0x19, 0x03 },
    { 0x1A, 0xF0 },
    { 0x1B, 0x89 },
    { 0x22, 0x03 },
    { 0x29, 0x17 },
    { 0x2B, 0xF8 },
    { 0x2C, 0x01 },
    { 0x31, 0xA0 },
    { 0x32, 0xF0 },
    { 0x33, 0xC4 },
    { 0x35, 0x05 },
    { 0x36, 0x3F },
    { 0x04, 0x60 },
    { 0x27, 0x80 },
    { 0x3D, 0x0F },
    { 0x3E, 0x81 },
    { 0x3F, 0x40 },
    { 0x40, 0x7F },
    { 0x41, 0x6A },
    { 0x42, 0x29 },
    { 0x44, 0xE5 },
    { 0x45, 0x41 },
    { 0x47, 0x42 },
    { 0x48, 0x00 },
    { 0x49, 0x61 },
    { 0x4A, 0xA1 },
    { 0x4B, 0x5E },
    { 0x4C, 0x18 },
    { 0x4D, 0x50 },
    { 0x4E, 0x13 },
    { 0x64, 0x00 },
    { 0x67, 0x88 },
    { 0x68, 0x1A },
    { 0x14, 0x38 },
    { 0x24, 0x4B },
    { 0x25, 0x3F },
    { 0x26, 0x72 },
    { 0x50, 0x97 },
    { 0x51, 0x7E },
    { 0x52, 0x00 },
    { 0x53, 0x00 },
    { 0x20, 0x00 },
    { 0x21, 0x23 },
    { 0x38, 0x14 },
    { 0xE9, 0x00 },
    { 0x56, 0x55 },
    { 0x57, 0xFF },
    { 0x58, 0xFF },
    { 0x59, 0xFF },
    { 0x5F, 0x04 },
    { 0xEC, 0x00 },
    { 0x13, 0xFF },
    { 0x80, 0x7D },
    { 0x81, 0x3F },
    { 0x82, 0x32 },
    { 0x83, 0x01 },
    { 0x38, 0x11 },
    { 0x84, 0x70 },
    { 0x85, 0x00 },
    { 0x86, 0x03 },
    { 0x87, 0x01 },
    { 0x88, 0x05 },
    { 0x89, 0x30 },
    { 0x8D, 0x30 },
    { 0x8F, 0x85 },
    { 0x93, 0x30 },
    { 0x95, 0x85 },
    { 0x99, 0x30 },
    { 0x9B, 0x85 },
    { 0x9C, 0x08 },
    { 0x9D, 0x12 },
    { 0x9E, 0x23 },
    { 0x9F, 0x45 },
    { 0xA0, 0x55 },
    { 0xA1, 0x64 },
    { 0xA2, 0x72 },
    { 0xA3, 0x7F },
    { 0xA4, 0x8B },
    { 0xA5, 0x95 },
    { 0xA6, 0xA7 },
    { 0xA7, 0xB5 },
    { 0xA8, 0xCB },
    { 0xA9, 0xDD },
    { 0xAA, 0xEC },
    { 0xAB, 0x1A },
    { 0xCE, 0x78 },
    { 0xCF, 0x6E },
    { 0xD0, 0x0A },
    { 0xD1, 0x0C },
    { 0xD2, 0x84 },
    { 0xD3, 0x90 },
    { 0xD4, 0x1E },
    { 0x5A, 0x24 },
    { 0x5B, 0x1F },
    { 0x5C, 0x88 },
    { 0x5D, 0x60 },
    { 0xAC, 0x6E },
    { 0xBE, 0xFF },
    { 0xBF, 0x00 },
    { 0x70, 0x00 },
    { 0x71, 0x34 },
    { 0x74, 0x28 },
    { 0x75, 0x98 },
    { 0x76, 0x00 },
    { 0x77, 0x08 },
    { 0x78, 0x01 },
    { 0x79, 0xC2 },
    { 0x7D, 0x02 },
    { 0x7A, 0x4E },
    { 0x7B, 0x1F },
    { 0xEC, 0x00 },
    { 0x7C, 0x0C },
    { 0x31, 0xA0 },
    { 0x32, 0xF0 },
    { 0x82, 0x32 },
    { 0x11, 0x03 },
    { 0x50, 0xFF },
    { 0x51, 0x7E },
    { 0x52, 0xF0 },
    { 0x53, 0x00 },
    { 0x20, 0x00 },
    { 0x21, 0x84 },
    { 0xEC, 0xC0 },
    { 0x8D, 0x7F },
    { 0x8F, 0x84 },
    { 0x93, 0x60 },
    { 0x95, 0x84 },
    { 0x99, 0x50 },
    { 0x9B, 0x84 },
    { 0xEC, 0x40 },
    { 0x11, 0x01 },
    { 0x55, 0x40 },
    { 0x2B, 0x5E },
    { 0x2C, 0x02 },
    { 0x13, 0xFF },
    { 0x50, 0x97 },
    { 0x51, 0x7E },
    { 0x52, 0x00 },
    { 0x21, 0x23 },
    { 0xEC, 0x40 },
    { 0x81, 0x3F },
    { 0xDA, 0x04 },
    { 0xE4, 0x0E },
    { 0xE3, 0x10 },
    { 0x13, 0xFF },
    { 0x01, 0x84 },
    { 0x02, 0x4C },
    { 0x03, 0x40 },
    { 0x15, 0x00 },
    { 0x2D, 0x00 },
    { 0x2E, 0x00 },
    { 0x80, 0x7F },
    { 0x81, 0x7F },
    { 0xDA, 0x06 },
    { 0xDD, 0x50 }, 
    { 0xDE, 0x50 },	
  
};

static const UCHAR I2CDataStandByEnter[][2] = {
    { 0x0E, 0xE8 },
};

static const UCHAR I2CDataStandByExit[][2] = {
    { 0x0E, 0xE0 },
};

// Local functions

static BOOL DoI2CWrite (PCAM_HW_INDEP_INFO pInfo,
						  const UCHAR *buf,
						  USHORT elements)
{
    struct i2c_msg msg;
    int ret;
    int i;
    int retries = 10;
    const UCHAR *ptr;

	// Check if camera in use
	msg.addr = pInfo->cameraI2CAddress[0] >> 1;
	ptr = buf;
	msg.flags = 0;
	msg.len = 2;

	for (i=0; i<elements; i++)
	{
		msg.buf = (UCHAR *)ptr;
		ret = i2c_transfer(pInfo->hI2C, &msg, 1);

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
		ptr+=2;
	}

    return TRUE;
}
#if 0
static BOOL DoI2CRead (PCAM_HW_INDEP_INFO pInfo, USHORT *result, USHORT reg)
{
	struct i2c_msg msgs[2];
    DWORD ret = 0;
    UCHAR cmd[2];
    UCHAR stat[2];

    msgs[0].addr = pInfo->cameraI2CAddress[0] >> 1;
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

BOOL OV7740_Init(PCAM_HW_INDEP_INFO pInfo)
{
    // Camera can not be initialized until FPGA up and running
	return TRUE;
}

DWORD OV7740_IOControl(PCAM_HW_INDEP_INFO pInfo,
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

	case IOCTL_CAM_INIT:
		LOCK(pInfo);
		dwErr = (DoI2CWrite(pInfo, I2CDataInitPart1[0], dim(I2CDataInitPart1))) ? ERROR_SUCCESS : -EIO;
		if (bCamActive == FALSE)
			DoI2CWrite(pInfo, I2CDataStandByEnter[0], dim(I2CDataStandByEnter));
		UNLOCK(pInfo);
		break;

	case IOCTL_CAM_GET_ACTIVE:
		{
			VCAMIOCTLACTIVE * pVcamIoctl = (VCAMIOCTLACTIVE *)pBuf;
			LOCK(pInfo);
			pVcamIoctl->bActive = bCamActive;
			dwErr = ERROR_SUCCESS;
			UNLOCK(pInfo);
		}
		break;

	case IOCTL_CAM_SET_ACTIVE:
		{
			BOOL bNewActive;
			BOOL res = TRUE;
			LOCK(pInfo);
			bNewActive = (((VCAMIOCTLACTIVE *)pBuf)->bActive != 0);
			if (bNewActive != bCamActive)
			{
				if (bNewActive)
				{
					res = DoI2CWrite(pInfo, I2CDataStandByExit[0], dim(I2CDataStandByExit));

				}
				else
				{
					res = DoI2CWrite(pInfo, I2CDataStandByEnter[0], dim(I2CDataStandByEnter));
				}
			}

			if (res)
			{
				bCamActive = bNewActive;
				pr_err("bCamActive now %d\n", bCamActive);
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
			dwErr = ERROR_SUCCESS;
		}
		break;
		
	default:
		pr_err("VCAM Unsupported IOCTL code %lu\n", Ioctl);
		dwErr = ERROR_NOT_SUPPORTED;
		break;
    }

    return dwErr;
}
