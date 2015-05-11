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
#include <linux/leds.h>

// Definitions

#define IOPORT_I2C_ADDR     0x46


#define VCM_PWR_EN          0
#define VCM_1_I2C_EN        1
//Laser switch on           2
#define VCM_PWDN            3
#define VCM_RESET           4
//Laser Soft on             5
//Optics 5V0                6
#define VCM_2_I2C_EN        7


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
// Function: RocoInitHW
//
// This function will perform BSP specific initialization.
//
// Parameters:
//
// Returns:
//      Returns status of init code.
//
//-----------------------------------------------------------------------------

DWORD RocoInitHW(PCAM_HW_INDEP_INFO pInfo)
{
    BOOL ret = TRUE;
    extern struct list_head leds_list;
    extern struct rw_semaphore leds_list_lock;
    struct led_classdev *led_cdev;

    pInfo->hI2C = i2c_get_adapter(1);
    pInfo->eCamModel = OV5640;
    pInfo->pGetTorchState = GetTorchState;
    pInfo->pSetTorchState = SetTorchState;
    pInfo->pEnablePower = EnablePower;
    pInfo->cameraI2CAddress[0] = 0x78;  //At power on vcam modules will share 0x78 i2c address
    pInfo->cameraI2CAddress[1] = 0x7A;

    // Find torch
    down_read(&leds_list_lock);
    list_for_each_entry(led_cdev, &leds_list, node) {
	    if (strcmp(led_cdev->name, "torch") == 0){
		    pr_err("*** Found led with name torch\n");
		    pInfo->torch_cdev = led_cdev;
	    } else {
		    pr_err("Found led with name %s\n", led_cdev->name);
	    }
    }
    up_read(&leds_list_lock);

    ret = SetI2CIoport(pInfo, VCM_PWR_EN, FALSE);
    if (ret)
        ret = SetI2CIoport(pInfo, VCM_RESET, FALSE);
    if (ret)
        ret = SetI2CIoport(pInfo, VCM_1_I2C_EN, FALSE);
    if (ret)
        ret = SetI2CIoport(pInfo, VCM_2_I2C_EN, FALSE);
    if (ret)
        ret = SetI2CIoport(pInfo, VCM_PWDN, TRUE);
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
                            (1 << VCM_1_I2C_EN) |
                            (1 << VCM_2_I2C_EN) |
                            (1 << VCM_PWDN) |
                            (1 << VCM_RESET));
        pr_debug("VCAM: IOPORT %02X -> %02X\n", buf[0], buf[1]);
        buf[0] = 3;
    	res = i2c_transfer(pInfo->hI2C, msgs, 1);
    }

    return (res > 0);
}

//-----------------------------------------------------------------------------
//
// Function: SetI2CIoport
//
// This function will set one bit of the ioport on RORI
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
        pr_debug("VCAM: IO Set %02X -> %02X\n", buf[0], buf[1]);
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
// This function will get one bit of the ioport on RORI
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
    if (pInfo->torch_cdev)
        pFlashData->bTorchOn = (pInfo->torch_cdev->brightness) ? TRUE : FALSE;
    else
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
    if (pInfo->torch_cdev)
    {
        pInfo->torch_cdev->brightness = pFlashData->bTorchOn ? 1 : 0;
        pInfo->torch_cdev->brightness_set(pInfo->torch_cdev, pInfo->torch_cdev->brightness);
    }

	return ERROR_SUCCESS;
}

//-----------------------------------------------------------------------------
//
// Function:  WriteVcam
//
// This function will
//
// Parameters:
//
// Returns:
//
//-----------------------------------------------------------------------------
DWORD WriteVcam(PCAM_HW_INDEP_INFO pInfo,u8 i2cAddress,u16 address,u8 data)
{
    struct i2c_msg msgs[1];
    UCHAR buf[3];

    msgs[0].addr = i2cAddress >> 1;
    msgs[0].flags = 0;
    msgs[0].len = 3;
    msgs[0].buf = buf;
    buf[0] = (address >>8) & 0xff;
    buf[1] = address & 0xff;
    buf[2] = data;

    return i2c_transfer(pInfo->hI2C, msgs, 1);
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
        SetI2CIoport(pInfo, VCM_PWDN, FALSE);
        msleep(1);
        SetI2CIoport(pInfo, VCM_RESET, TRUE);
        msleep(1);
        SetI2CIoport(pInfo, VCM_2_I2C_EN, TRUE);
        msleep(1); //Change vcam2 i2c address from 0x78 to 0x7a
        WriteVcam(pInfo,pInfo->cameraI2CAddress[0],0x3100,pInfo->cameraI2CAddress[1]);
        SetI2CIoport(pInfo, VCM_1_I2C_EN, TRUE);
    }
    else
    {
        SetI2CIoport(pInfo, VCM_2_I2C_EN, FALSE);
        msleep(1);
        SetI2CIoport(pInfo, VCM_1_I2C_EN, FALSE);
        msleep(1);
        SetI2CIoport(pInfo, VCM_RESET, FALSE);
        msleep(1);
        SetI2CIoport(pInfo, VCM_PWDN, TRUE);
        msleep(1);
        SetI2CIoport(pInfo, VCM_PWR_EN, FALSE);
    }
}
