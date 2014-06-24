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
static BOOL InitI2CIoport (PCAM_HW_INDEP_INFO pInfo);
static BOOL SetI2CIoport (PCAM_HW_INDEP_INFO pInfo, UCHAR bit, BOOL value);
// static BOOL GetI2CIoport (PCAM_HW_INDEP_INFO pInfo, UCHAR bit);

static DWORD GetTorchState(PCAM_HW_INDEP_INFO pInfo, VCAMIOCTLFLASH * pFlashData);
static DWORD SetTorchState(PCAM_HW_INDEP_INFO pInfo, VCAMIOCTLFLASH * pFlashData);
static void EnablePower(PCAM_HW_INDEP_INFO pInfo, BOOL bEnable);

//-----------------------------------------------------------------------------
//
// Function: PicoInitHW
//
// This function will perform BSP specific initialization.
//
// Parameters:
//
// Returns:
//      Returns status of init code.
//
//-----------------------------------------------------------------------------

DWORD PicoInitHW(PCAM_HW_INDEP_INFO pInfo)
{
    BOOL ret = TRUE;

    pInfo->eCamModel = MT9P111;
    pInfo->pGetTorchState = GetTorchState;
    pInfo->pSetTorchState = SetTorchState;
    pInfo->pEnablePower = EnablePower;
    pInfo->cameraI2CAddress[0] = 0x78;
    pInfo->cameraI2CAddress[1] = 0x7A;

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
        EnablePower(pInfo, TRUE);
    return ret;
}



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

BOOL InitI2CIoport (PCAM_HW_INDEP_INFO pInfo)
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
        pr_err("VCAM: IOPORT %02X -> %02X\n", buf[0], buf[1]);
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

BOOL SetI2CIoport (PCAM_HW_INDEP_INFO pInfo, UCHAR bit, BOOL value)
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
        pr_err("VCAM: IO Set %02X -> %02X\n", buf[0], buf[1]);
        buf[0] = 1;       // Set output value

    	res = i2c_transfer(pInfo->hI2C, msgs, 1);
    }

    return (res > 0);
}

#if 0
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

BOOL GetI2CIoport (PCAM_HW_INDEP_INFO pInfo, UCHAR bit)
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
#endif

//-----------------------------------------------------------------------------
//
// Function:  GetTorchState
//
// This function will return torch and flash state.
//
// Parameters:
//
// Returns:
//
//-----------------------------------------------------------------------------
DWORD GetTorchState(PCAM_HW_INDEP_INFO pInfo, VCAMIOCTLFLASH * pFlashData)
{
    pFlashData->bTorchOn = FALSE;
    pFlashData->bFlashOn = FALSE;

	return ERROR_SUCCESS;
}

//-----------------------------------------------------------------------------
//
// Function:  SetTorchState
//
// This function will set torch and flash state.
//
// Parameters:
//
// Returns:
//
//-----------------------------------------------------------------------------
DWORD SetTorchState(PCAM_HW_INDEP_INFO pInfo, VCAMIOCTLFLASH * pFlashData)
{
	return ERROR_SUCCESS;
}

//-----------------------------------------------------------------------------
//
// Function:  EnablePower
//
// This function will control standby/on GPIO usage
//
// Parameters:
//
// Returns:
//
//-----------------------------------------------------------------------------
void EnablePower(PCAM_HW_INDEP_INFO pInfo, BOOL bEnable)
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
