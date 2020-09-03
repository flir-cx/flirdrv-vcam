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
static DWORD GetTorchState(PCAM_HW_INDEP_INFO pInfo,
			   VCAMIOCTLFLASH *pFlashData);
static DWORD SetTorchState(PCAM_HW_INDEP_INFO pInfo,
			   VCAMIOCTLFLASH *pFlashData);
static void EnablePower(PCAM_HW_INDEP_INFO pInfo, BOOL bEnable);
static void Suspend(PCAM_HW_INDEP_INFO pInfo, BOOL bSuspend);

#ifdef CONFIG_OF
static int requestGPIOpin(PCAM_HW_INDEP_INFO pInfo, int *ppin, char *of_name,
			  int value);
static int requestRegulator(PCAM_HW_INDEP_INFO pInfo, struct regulator **reg,
			    char *of_name, int enable);
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

	pInfo->hI2C = i2c_get_adapter(2);
	pInfo->eCamModel = OV5640;
	pInfo->pGetTorchState = GetTorchState;
	pInfo->pSetTorchState = SetTorchState;
	pInfo->pEnablePower = EnablePower;
	pInfo->pSuspend = Suspend;
	pInfo->cameraI2CAddress[0] = 0x78;
	pInfo->mirror_image = 0;
	pInfo->flip_image = 0;
	pInfo->flip_image_hw = 0;
	pInfo->edge_enhancement = 1;

#ifdef CONFIG_OF

	if (of_find_property(pInfo->node, "flip-image", NULL))
		pInfo->flip_image = 1;

	ret = requestGPIOpin(pInfo, &pInfo->reset_gpio, "vcam_reset-gpio", 0);
	if (!ret)
		ret = requestRegulator(pInfo, &pInfo->reg_vcm, "VCM_DOVDD", 0);
	if (!ret)
		ret =
		    requestGPIOpin(pInfo, &pInfo->pwdn_gpio, "vcam_pwdn-gpio",
				   1);
	if (!ret)
		ret =
		    requestGPIOpin(pInfo, &pInfo->clk_en_gpio,
				   "vcam_clk_en-gpio", 1);
	if (!ret)
		EnablePower(pInfo, TRUE);

#endif
	return TRUE;
}

#ifdef CONFIG_OF
int requestGPIOpin(PCAM_HW_INDEP_INFO pInfo, int *ppin, char *of_name,
		   int value)
{
	int pin, retval = -1;

	pin = of_get_named_gpio_flags(pInfo->node, of_name, 0, NULL);
	if (gpio_is_valid(pin) == 0) {
		pr_err("VCAM: %s  can not be used\n", of_name);
	} else {
		*ppin = pin;
		retval = gpio_request(pin, of_name);
		if (retval)
			pr_err("VCAM: Fail registering %s", of_name);

		retval = gpio_direction_output(pin, value);
		if (retval)
			pr_err("VCAM: Fail setting direction for %s", of_name);

	}
	return retval;

}

int requestRegulator(PCAM_HW_INDEP_INFO pInfo, struct regulator **reg,
		     char *of_name, int enable)
{
	int retval = -1;
	*reg = regulator_get(&pInfo->pLinuxDevice->dev, of_name);
	if (IS_ERR(*reg)) {
		pr_err("VCAM: Error on %s get\n", of_name);
	} else {
		retval = 0;
		if (enable)
			retval = regulator_enable(*reg);
		else if (regulator_is_enabled(*reg))
			retval = regulator_disable(*reg);

		if (retval) {
			pr_err("VCAM: Could not %s %s regulator\n",
			       enable ? "enable" : "disable", of_name);
		}
	}

	return retval;
}
#endif

//-----------------------------------------------------------------------------
//
// Function:  FindTorch
//
// This function will return LED named "torch" from the kernel list of LEDS
//
// Parameters: None
//
// Returns: struct *led_cdev NULL if not found
//
//-----------------------------------------------------------------------------
struct led_classdev *FindTorch(void)
{
	extern struct list_head leds_list;
	extern struct rw_semaphore leds_list_lock;
	/* Find torch */
	struct led_classdev *led_cdev, *led = NULL;
	down_read(&leds_list_lock);
	list_for_each_entry(led_cdev, &leds_list, node) {
		if (strcmp(led_cdev->name, "torch") == 0) {
			led = led_cdev;
			break;
		}
	}
	up_read(&leds_list_lock);

	return led;
}

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
DWORD GetTorchState(PCAM_HW_INDEP_INFO pInfo, VCAMIOCTLFLASH *pFlashData)
{
	int ret;
	struct led_classdev *led = FindTorch();

	if (led) {
		pFlashData->bTorchOn = (led->brightness) ? TRUE : FALSE;
		ret = ERROR_SUCCESS;
	} else {
		pFlashData->bTorchOn = FALSE;
		pr_err_once("Failed to find LED Torch\n");
		//Here we want to return ERROR_INVALID_HANDLE, but due to appcore and webapplications, we need
		//this to succeed, until underlying software is able to handle fails here!
		// ret = ERROR_INVALID_HANDLE;
		ret = ERROR_SUCCESS;
	}

	pFlashData->bFlashOn = FALSE;
	return ret;
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
DWORD SetTorchState(PCAM_HW_INDEP_INFO pInfo, VCAMIOCTLFLASH *pFlashData)
{
	int ret;
	struct led_classdev *led = FindTorch();

	if (led) {
		led->brightness =
		    pFlashData->bTorchOn ? led->max_brightness : 0;
		led->brightness_set(led, led->brightness);
		ret = ERROR_SUCCESS;
	} else {

		pr_err_once("Failed to find LED Flash\n");
		//Here we want to return ERROR_INVALID_HANDLE, but due to appcore and webapplications, we need
		//this to succeed, until underlying software is able to handle fails here!
		// ret = ERROR_INVALID_HANDLE;
		ret = ERROR_SUCCESS;
	}

	return ret;
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
static void EnablePower(PCAM_HW_INDEP_INFO pInfo, BOOL bEnable)
{
	int ret = 0;
#ifdef CONFIG_OF
	if (bEnable) {
		ret = regulator_enable(pInfo->reg_vcm);
		msleep(20);
		gpio_direction_output(pInfo->clk_en_gpio, 0);
		gpio_direction_output(pInfo->pwdn_gpio, 0);
		usleep_range(10000, 20000);
		gpio_direction_output(pInfo->reset_gpio, 1);
	} else {
		gpio_direction_output(pInfo->reset_gpio, 0);
		usleep_range(10000, 20000);
		gpio_direction_output(pInfo->pwdn_gpio, 1);
		gpio_direction_output(pInfo->clk_en_gpio, 1);
		usleep_range(10000, 20000);
		ret = regulator_disable(pInfo->reg_vcm);
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
		EnablePower(pInfo, false);
	} else {
		EnablePower(pInfo, true);
		OV5640_reinit(pInfo);
	}

#endif
}
