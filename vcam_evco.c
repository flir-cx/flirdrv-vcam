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
#include <linux/platform_device.h>

#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/of_regulator.h>
#endif

// Definitions

// Local variables

// Function prototypes
static DWORD GetTorchState(PCAM_HW_INDEP_INFO pInfo, VCAMIOCTLFLASH * pFlashData);
static DWORD SetTorchState(PCAM_HW_INDEP_INFO pInfo, VCAMIOCTLFLASH * pFlashData);
static void EnablePower(PCAM_HW_INDEP_INFO pInfo, BOOL bEnable);
static void Suspend(PCAM_HW_INDEP_INFO pInfo, BOOL bSuspend);

#ifdef CONFIG_OF
static int requestGPIOpin(PCAM_HW_INDEP_INFO pInfo, int * ppin, char * of_name, int value );
static int requestRegulator(PCAM_HW_INDEP_INFO pInfo,struct regulator ** reg, char * of_name,int enable);
#endif

//-----------------------------------------------------------------------------
//
// Function: EvcoInitHW
//
// This function will perform BSP specific initialization.
//
// Parameters:
//
// Returns:
//      Returns status of init code.
//
//-----------------------------------------------------------------------------

DWORD EvcoInitHW(PCAM_HW_INDEP_INFO pInfo)
{
    BOOL ret = TRUE;
    extern struct list_head leds_list;
    extern struct rw_semaphore leds_list_lock;
    struct led_classdev *led_cdev;

    pInfo->hI2C = i2c_get_adapter(2);
    pInfo->eCamModel = OV5640;
    pInfo->pGetTorchState = GetTorchState;
    pInfo->pSetTorchState = SetTorchState;
    pInfo->pEnablePower = EnablePower;
    pInfo->pSuspend = Suspend;
    pInfo->cameraI2CAddress[0] = 0x78;
    pInfo->flip_image= 0;

 #ifdef CONFIG_OF
    // Find torch
    down_read(&leds_list_lock);
    list_for_each_entry(led_cdev, &leds_list, node) {
		if (strcmp(led_cdev->name, "torch") == 0){
		    pr_err("*** Found led with name torch\n");
		    pInfo->torch_cdev = led_cdev;
	    }
		else {
		  //  pr_err("Found led with name %s\n", led_cdev->name);
	    }
    }
    up_read(&leds_list_lock);

    ret =requestGPIOpin(pInfo,&pInfo->reset_gpio,"vcam_reset-gpio",0);
    if(!ret)
        ret = requestRegulator(pInfo,&pInfo->reg_vcm,"VCM_DOVDD",0);
    if(!ret)
        ret = requestGPIOpin(pInfo,&pInfo->pwdn_gpio,"vcam_pwdn-gpio",1);
    if (!ret)
        EnablePower(pInfo, TRUE);
#endif
    return TRUE;
}


 #ifdef CONFIG_OF
int requestGPIOpin(PCAM_HW_INDEP_INFO pInfo, int * ppin, char * of_name, int value )
{
    int pin,retval=-1;
    pin = of_get_named_gpio_flags(pInfo->node, of_name, 0, NULL);
    if (gpio_is_valid(pin) == 0){
        pr_err("VCAM: %s  can not be used\n",of_name);
    } else {
        *ppin = pin;
        retval = gpio_request(pin, of_name);
        if(retval){
            pr_err("VCAM: Fail registering %s",of_name);
        }
        retval = gpio_direction_output(pin, value);
        if(retval){
            pr_err("VCAM: Fail setting direction for %s",of_name);
        }
    }
    return retval;

}

int requestRegulator(PCAM_HW_INDEP_INFO pInfo,struct regulator ** reg, char * of_name,int enable)
{
    int retval = -1;
    *reg = regulator_get(&pInfo->pLinuxDevice->dev, of_name);
	if(IS_ERR(*reg))
	{
		pr_err("VCAM: Error on %s get\n",of_name);
	}
	else
	{
        retval =0;
		if(enable)
            retval = regulator_enable(*reg);
        else if(regulator_is_enabled(*reg))
            retval = regulator_disable(*reg);

		if (retval){
			pr_err("VCAM: Could not %s %s regulator\n",enable?"enable":"disable",of_name);
		}
	}

    return retval;
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
        pInfo->torch_cdev->brightness = pFlashData->bTorchOn ? pInfo->torch_cdev->max_brightness : 0;
        pInfo->torch_cdev->brightness_set(pInfo->torch_cdev, pInfo->torch_cdev->brightness);
    }

	return ERROR_SUCCESS;
}

#if 0
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
static DWORD WriteVcam(PCAM_HW_INDEP_INFO pInfo,u8 i2cAddress,u16 address,u8 data)
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
#endif

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
static void EnablePower(PCAM_HW_INDEP_INFO pInfo, BOOL bEnable)
{
   int ret=0;
#ifdef CONFIG_OF
    if (bEnable)
    {
        ret=regulator_enable(pInfo->reg_vcm);
        msleep(20);
        gpio_direction_output(pInfo->pwdn_gpio, 0);
        msleep(1);
        gpio_direction_output(pInfo->reset_gpio, 1);
    }
    else
    {
        gpio_direction_output(pInfo->reset_gpio, 0);
        msleep(1);
        gpio_direction_output(pInfo->pwdn_gpio, 1);
        msleep(1);
        ret=regulator_disable(pInfo->reg_vcm);
    }
#endif
}

//-----------------------------------------------------------------------------
//
// Function: Suspend
//
// This function will handle suspend and resume
//
// Parameters:
//
// Returns:
//
//-----------------------------------------------------------------------------
static void Suspend(PCAM_HW_INDEP_INFO pInfo, BOOL bSuspend)
{
#ifdef CONFIG_OF
    if (bSuspend) {
        OV5640_MipiSuspend(pInfo, bSuspend);
        gpio_direction_output(pInfo->pwdn_gpio, 1);
    } else {
        gpio_direction_output(pInfo->pwdn_gpio, 0);
        msleep(20);
        OV5640_MipiSuspend(pInfo, bSuspend);
    }
#endif
}
