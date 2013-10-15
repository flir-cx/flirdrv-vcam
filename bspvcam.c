//------------------------------------------------------------------------------
//
//  File:  bspvcam.c
//
//  Provides BSP-specific configuration routines for visual camera peripheral.
//
//-----------------------------------------------------------------------------

#include "flir_kernel_os.h"
#include "vcam.h"
#include "vcam_internal.h"
#include "i2cdev.h"
#include "faddev.h"
#include <linux/i2c.h>

// Definitions

#define IOPORT_I2C_ADDR     0x46

#define VCM_LED_EN          0
#define VCM_PWR_EN          1
#define VCM_RESET           4
#define VCM_CLK_EN          6
#define VCM_I2C_EN          7

// Local variables

// Function prototypes

//-----------------------------------------------------------------------------
//
// Function: InitI2CIoport
//
// This function will set up the ioport on PIRI as outputs
//
// Parameters:
//
// Returns:
//      Returns status of init code.
//
//-----------------------------------------------------------------------------

static BOOL InitI2CIoport (PCAM_HW_INDEP_INFO pInfo)
{
	struct i2c_msg msgs[2];
    int res;
    UCHAR buf[2];
    UCHAR cmd;
    msgs[0].addr = IOPORT_I2C_ADDR >> 1;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &cmd;
    msgs[1].addr = IOPORT_I2C_ADDR >> 1;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = buf;

    cmd = 3;    // Read config register
	res = i2c_transfer(pInfo->hI2C, msgs, 2);

    if (res > 0)
    {
    	msgs[0].len = 2;
    	msgs[0].buf = buf;
        buf[1] = buf[0] & ~((1 << VCM_PWR_EN) |
                            (1 << VCM_RESET) |
                            (1 << VCM_CLK_EN) |
                            (1 << VCM_I2C_EN));  
//        pr_err("VCAM: IOPORT %02X -> %02X\n", buf[0], buf[1]);
        buf[0] = 3;
    	res = i2c_transfer(pInfo->hI2C, msgs, 1);
    }

    return (res > 0);
}

//-----------------------------------------------------------------------------
//
// Function: SetI2CIoport
//
// This function will set one bit of the ioport on PIRI
//
// Parameters:
//
// Returns:
//      Returns status of set operation.
//
//-----------------------------------------------------------------------------

static BOOL SetI2CIoport (PCAM_HW_INDEP_INFO pInfo, UCHAR bit, BOOL value)
{
	struct i2c_msg msgs[2];
    int res;
    UCHAR buf[2];
    UCHAR cmd;

    msgs[0].addr = IOPORT_I2C_ADDR >> 1;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &cmd;
    msgs[1].addr = IOPORT_I2C_ADDR >> 1;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = buf;

    cmd = 1;    // Read output register
	res = i2c_transfer(pInfo->hI2C, msgs, 2);

    if (res > 0)
    {
    	msgs[0].len = 2;
    	msgs[0].buf = buf;
        buf[1] = buf[0];  // Initial value before changes
        if (value)
            buf[1] |= (1 << bit);
        else
            buf[1] &= ~(1 << bit);
//        pr_err("VCAM: IO Set %02X -> %02X\n", buf[0], buf[1]);
        buf[0] = 1;       // Set output value

    	res = i2c_transfer(pInfo->hI2C, msgs, 1);
    }

    return (res > 0);
}

//-----------------------------------------------------------------------------
//
// Function: GetI2CIoport
//
// This function will get one bit of the ioport on PIRI
//
// Parameters:
//
// Returns:
//      Returns status of bit.
//
//-----------------------------------------------------------------------------

static BOOL GetI2CIoport (PCAM_HW_INDEP_INFO pInfo, UCHAR bit)
{
	struct i2c_msg msgs[2];
    int res;
    UCHAR buf[2];
    UCHAR cmd;

    msgs[0].addr = IOPORT_I2C_ADDR >> 1;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &cmd;
    msgs[1].addr = IOPORT_I2C_ADDR >> 1;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = buf;

    cmd = 0;    // Read port register
    buf[0] = 0;
	res = i2c_transfer(pInfo->hI2C, msgs, 2);

    return ((buf[0] & (1 << bit)) != 0);
}

//-----------------------------------------------------------------------------
//
// Function: BSPInitHW
//
// This function will perform BSP specific initialization.
//
// Parameters:
//
// Returns:
//      Returns status of init code.
//
//-----------------------------------------------------------------------------

DWORD BSPInitHW(PCAM_HW_INDEP_INFO pInfo)
{
    BOOL ret;

    ret = SetI2CIoport(pInfo, VCM_PWR_EN, FALSE);
    if (ret)
        ret = SetI2CIoport(pInfo, VCM_RESET, TRUE);
    if (ret)
        ret = SetI2CIoport(pInfo, VCM_CLK_EN, FALSE);
    if (ret)
        ret = SetI2CIoport(pInfo, VCM_I2C_EN, FALSE);
    if (ret)
        ret = InitI2CIoport(pInfo);

    if (ret)
        BspEnablePower(pInfo, TRUE);

    return ret;
}

//-----------------------------------------------------------------------------
//
// Function:  BSPDeinitHW
//
// This function will perform BSP specific deinitialization.
//
// Parameters:
//
// Returns:
//
//-----------------------------------------------------------------------------
DWORD BSPDeinitHW()
{
    return ERROR_SUCCESS;
}

//-----------------------------------------------------------------------------
//
// Function:  BSPGetCameraModel
//
// This function will return type of visual camera used.
//
// Parameters:
//
// Returns:
//
//-----------------------------------------------------------------------------
VCAM_CamModel BSPGetCameraModel()
{
	return MT9P111;
}

//-----------------------------------------------------------------------------
//
// Function:  BSPGetTorchState
//
// This function will return torch and flash state.
//
// Parameters:
//
// Returns:
//
//-----------------------------------------------------------------------------
DWORD BSPGetTorchState(VCAMIOCTLFLASH * pFlashData)
{
    pFlashData->bTorchOn = FALSE;
    pFlashData->bFlashOn = FALSE;

	return ERROR_SUCCESS;
}

//-----------------------------------------------------------------------------
//
// Function:  BSPSetTorchState
//
// This function will set torch and flash state.
//
// Parameters:
//
// Returns:
//
//-----------------------------------------------------------------------------
DWORD BSPSetTorchState(VCAMIOCTLFLASH * pFlashData)
{
	return ERROR_SUCCESS;
}

//-----------------------------------------------------------------------------
//
// Function:  BSPGetCameraI2CAddress
//
// This function will return visual camera I2C address
//
// Parameters:
//
// Returns:
//
//-----------------------------------------------------------------------------
UCHAR BSPGetCameraI2CAddress(DWORD dwCamNo)
{
    UCHAR addr;

    switch (dwCamNo)
    {
        case 0:
            addr = 0x78;    // CAM1
            break;
        
        case 1:
            addr = 0x7A;    // CAM2
            break;

        default:
            addr = 0;
            break;
    }
    return addr;
}

//-----------------------------------------------------------------------------
//
// Function:  BspReinitAfterStandby
//
// This function returns true if we need to reinitialize the camera module 
// after standby.
//
// Parameters:
//
// Returns:
//
//-----------------------------------------------------------------------------
BOOL BspGetReinitAfterStandby(void)
{
    return FALSE;
}

//-----------------------------------------------------------------------------
//
// Function:  BSPSetLightLimit
//
// This function will ....
//
// Parameters:
//
// Returns:
//
//-----------------------------------------------------------------------------
BOOL BSPSetLightLimit(void)
{
    return TRUE;
}

//-----------------------------------------------------------------------------
//
// Function:  BspEnablePower
//
// This function will control standby/on GPIO usage
//
// Parameters:
//
// Returns:
//
//-----------------------------------------------------------------------------
void BspEnablePower(PCAM_HW_INDEP_INFO pInfo, BOOL bEnable)
{
    if (bEnable)
    {
        SetI2CIoport(pInfo, VCM_PWR_EN, TRUE);
        msleep(20);
        SetI2CIoport(pInfo, VCM_CLK_EN, TRUE);
        msleep(1);
        SetI2CIoport(pInfo, VCM_RESET, FALSE);
        msleep(1);
        SetI2CIoport(pInfo, VCM_I2C_EN, TRUE);
    }
    else
    {
        SetI2CIoport(pInfo, VCM_I2C_EN, FALSE);
        msleep(1);
        SetI2CIoport(pInfo, VCM_RESET, TRUE);
        msleep(1);
        SetI2CIoport(pInfo, VCM_CLK_EN, FALSE);
        msleep(1);
        SetI2CIoport(pInfo, VCM_PWR_EN, FALSE);
    }
}
