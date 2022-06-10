// SPDX-License-Identifier: GPL-2.0-or-later
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

#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/of_regulator.h>

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
// Function: EocoInitHW
//
// This function will perform BSP specific initialization.
//
// Parameters:
//
// Returns:
//      Returns status of init code.
//
//-----------------------------------------------------------------------------

DWORD EocoInitHW(struct device *dev)
{
	int ret = 0;

	PCAM_HW_INDEP_INFO pInfo = dev->driver_data;
	struct platform_device *pdev = pInfo->pLinuxDevice;

	pInfo->hI2C = i2c_get_adapter(0);
	pInfo->eCamModel = OV5640;
	pInfo->pGetTorchState = GetTorchState;
	pInfo->pSetTorchState = SetTorchState;
	pInfo->pEnablePower = EnablePower;
	pInfo->pSuspend = Suspend;
	pInfo->cameraI2CAddress[0] = 0x78;
	pInfo->edge_enhancement = 1;

	pInfo->reset_gpio = of_get_named_gpio_flags(pInfo->node, "vcam_reset-gpio", 0, NULL);
	if (gpio_is_valid(pInfo->reset_gpio)) {
		ret = devm_gpio_request_one(&pdev->dev, pInfo->reset_gpio, GPIOF_OUT_INIT_LOW,
					    "vcam_reset-gpio");
		if (ret) {
			dev_err(dev, "Failed registering pin vcam_reset-gpio (err %i)\n", ret);
			return ret;
		}
	} else {
		dev_warn(&pdev->dev, "vcam_reset not detected\n");
		pInfo->reset_gpio = 0;
	}

	pInfo->reg_vcm = devm_regulator_get(&pInfo->pLinuxDevice->dev, "eodc_dovdd");
	if (IS_ERR(pInfo->reg_vcm)) {
		dev_err(dev, "VCAM: Error fetching regulator VCM_DOVDD\n");
		return -EIO;
	}

	pInfo->pwdn_gpio = of_get_named_gpio_flags(pInfo->node, "vcam_pwdn-gpio", 0, NULL);
	if (gpio_is_valid(pInfo->pwdn_gpio)) {
		ret = devm_gpio_request_one(&pdev->dev, pInfo->pwdn_gpio, GPIOF_OUT_INIT_HIGH, "vcam_pwdn-gpio");
		if (ret) {
			dev_err(dev, "Failed registering pin vcam_pwdn-gpio (err %i)\n", ret);
			return ret;
		}
	} else {
		dev_warn(&pdev->dev, "vcam_pwdn not detected\n");
		pInfo->pwdn_gpio = 0;
	}
	pInfo->clk_en_gpio = of_get_named_gpio_flags(pInfo->node, "vcam_clk_en-gpio", 0, NULL);
	if (gpio_is_valid(pInfo->clk_en_gpio)) {
		ret = devm_gpio_request_one(&pdev->dev, pInfo->clk_en_gpio, GPIOF_OUT_INIT_HIGH, "vcam_clk_en-gpio");
		if (ret) {
			dev_err(dev, "Failed registering pin vcam_clk_en-gpio (err %i)\n", ret);
			return ret;
		}
	} else {
		dev_warn(&pdev->dev, "vcam_clk_en-gpio not detected\n");
		pInfo->clk_en_gpio = 0;
	}

	EnablePower(pInfo, TRUE);
	OV5640_Init(pInfo);
	return ret;
}

DWORD EocoDeInitHW(struct device *dev)
{
	PCAM_HW_INDEP_INFO pInfo = dev->driver_data;

	dev_err(dev, "%s", __func__);
	/* OV5640_DeInit(pInfo); */
	EnablePower(pInfo, FALSE);
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
static struct led_classdev *FindTorch(void)
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


static int eodp_vcam_enable_clk(struct device *dev, int on)
{
	PCAM_HW_INDEP_INFO pInfo = (PCAM_HW_INDEP_INFO)dev->driver_data;
	return gpio_direction_output(pInfo->clk_en_gpio, on);
}

static int eodp_vcam_powerdown(struct device *dev, int on)
{
	PCAM_HW_INDEP_INFO pInfo = (PCAM_HW_INDEP_INFO)dev->driver_data;
	return gpio_direction_output(pInfo->pwdn_gpio, on);
}


static int eodp_vcam_reset(struct device *dev, int on)
{
	PCAM_HW_INDEP_INFO pInfo = (PCAM_HW_INDEP_INFO)dev->driver_data;
	return gpio_direction_output(pInfo->reset_gpio, on);
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
	struct platform_device *pdev = pInfo->pLinuxDevice;
	struct device *dev = &pdev->dev;

	if (bEnable) {

		ret = regulator_enable(pInfo->reg_vcm);
		usleep_range(1000, 10000);
		eodp_vcam_enable_clk(dev, 1); //setting changed compared to evco...
		eodp_vcam_powerdown(dev, 0);
		usleep_range(1000, 10000);
		eodp_vcam_reset(dev, 0); //setting changed compared to evco...
	} else {
		eodp_vcam_reset(dev, 1); //setting changed compared to evco...
		usleep_range(10000, 10000);
		eodp_vcam_powerdown(dev, 1);
		eodp_vcam_enable_clk(dev, 0); //setting changed compared to evco...
		usleep_range(10000, 10000);
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
