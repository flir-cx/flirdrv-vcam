// SPDX-License-Identifier: GPL-2.0-or-later
//------------------------------------------------------------------------------
//
//  File:  bspvcam.c
//
//  Provides BSP-specific configuration routines for visual camera peripheral.
//
//-----------------------------------------------------------------------------

#include "flir_kernel_os.h"
#include "vcam_ioctl.h"
#include "vcam_internal.h"
#include "i2cdev.h"
#include "faddev.h"
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/platform_device.h>

#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/of_regulator.h>
#include "OV5640.h"

// Definitions

// Local variables

// Function prototypes
static DWORD GetTorchState(PCAM_HW_INDEP_INFO pInfo,
			   VCAMIOCTLFLASH * pFlashData);
static DWORD SetTorchState(PCAM_HW_INDEP_INFO pInfo,
			   VCAMIOCTLFLASH *pFlashData);
static void EnablePower(PCAM_HW_INDEP_INFO pInfo, BOOL bEnable);
static void Suspend(PCAM_HW_INDEP_INFO pInfo, BOOL bSuspend);

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
	int ret;

	struct platform_device *pdev = pInfo->pLinuxDevice;
	struct device *dev = &pdev->dev;

	pInfo->hI2C = i2c_get_adapter(2);
	pInfo->eCamModel = OV5640;
	pInfo->pGetTorchState = GetTorchState;
	pInfo->pSetTorchState = SetTorchState;
	pInfo->pEnablePower = EnablePower;
	pInfo->pSuspend = Suspend;
	pInfo->cameraI2CAddress[0] = 0x78;
	pInfo->edge_enhancement = 1;

	pInfo->reset_gpio =
	    of_get_named_gpio_flags(pInfo->node, "vcam_reset-gpio", 0, NULL);
	ret =
	    gpio_request_one(pInfo->reset_gpio, GPIOF_OUT_INIT_LOW,
			     "vcam_reset-gpio");
	if (ret) {
		dev_err(dev,
			"Failed registering pin vcam_reset-gpio (err %i)\n",
			ret);
		return -EIO;
	}

	dev_info(dev, "Registered vcam_reset-gpio\n");

	pInfo->reg_vcm = regulator_get(&pInfo->pLinuxDevice->dev, "VCM_DOVDD");
	if (IS_ERR(pInfo->reg_vcm)) {
		dev_err(dev, "VCAM: Error on %s get\n", "VCM_DOVDD");
		gpio_free(pInfo->reset_gpio);
		return -EIO;
	}

	pInfo->pwdn_gpio =
	    of_get_named_gpio_flags(pInfo->node, "vcam_pwdn-gpio", 0, NULL);
	ret =
	    gpio_request_one(pInfo->pwdn_gpio, GPIOF_OUT_INIT_HIGH,
			     "vcam_pwdn-gpio");
	if (ret) {
		dev_err(dev, "Failed registering pin vcam_pwdn_gpio (err %i)\n",
			ret);
		gpio_free(pInfo->reset_gpio);
		return -EIO;
	}

	dev_err(dev, "Registered vcam_pwdn_gpio\n");

	pInfo->clk_en_gpio =
	    of_get_named_gpio_flags(pInfo->node, "vcam_clk_en-gpio", 0, NULL);
	ret =
	    gpio_request_one(pInfo->clk_en_gpio, GPIOF_OUT_INIT_HIGH,
			     "vcam_clk_en-gpio");
	if (ret) {
		dev_err(dev,
			"Failed registering pin vcam_clk_en-gpio (err %i)\n",
			ret);
		gpio_free(pInfo->reset_gpio);
		gpio_free(pInfo->pwdn_gpio);
		return -EIO;
	}

	dev_err(dev, "Registered vcam_clk_en-gpio\n");
	EnablePower(pInfo, TRUE);
	ret = OV5640_create_sysfs_attributes(dev);
	return ret;
}

DWORD EvcoDeInitHW(PCAM_HW_INDEP_INFO pInfo)
{
	struct platform_device *pdev = pInfo->pLinuxDevice;
	struct device *dev = &pdev->dev;

	OV5640_remove_sysfs_attributes(dev);
	EnablePower(pInfo, FALSE);
	gpio_free(pInfo->clk_en_gpio);
	gpio_free(pInfo->pwdn_gpio);
	regulator_put(pInfo->reg_vcm);
	gpio_free(pInfo->reset_gpio);
	i2c_put_adapter(pInfo->hI2C);
	return 0;
}

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
		if (led_cdev && led_cdev->name &&
		    strcmp(led_cdev->name, "torch") == 0) {
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
	struct platform_device *pdev = pInfo->pLinuxDevice;
	struct device *dev = &pdev->dev;

	if (led) {
		pFlashData->bTorchOn = (led->brightness) ? TRUE : FALSE;
		ret = ERROR_SUCCESS;
	} else {
		pFlashData->bTorchOn = FALSE;
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 19, 0)
		pr_err_once("Failed to find LED Torch\n");
#else
		dev_err_once(dev, "Failed to find LED Torch\n");
#endif
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
	struct platform_device *pdev = pInfo->pLinuxDevice;
	struct device *dev = &pdev->dev;

	if (led) {
		led->brightness =
		    pFlashData->bTorchOn ? led->max_brightness : 0;
		led->brightness_set(led, led->brightness);
		ret = ERROR_SUCCESS;
	} else {
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 19, 0)
		pr_err_once("Failed to find LED Torch\n");
#else
		dev_err_once(dev, "Failed to find LED Flash\n");
#endif

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
	if (bSuspend) {
		EnablePower(pInfo, false);
	} else {
		EnablePower(pInfo, true);
		OV5640_reinit(pInfo);
	}
}
