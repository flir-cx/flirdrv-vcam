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
static DWORD GetTorchState(PCAM_HW_INDEP_INFO pInfo,
			   VCAMIOCTLFLASH *pFlashData);
static DWORD SetTorchState(PCAM_HW_INDEP_INFO pInfo,
			   VCAMIOCTLFLASH *pFlashData);
static void EnablePower(PCAM_HW_INDEP_INFO pInfo, BOOL bEnable);
static void Suspend(PCAM_HW_INDEP_INFO pInfo, BOOL bEnable);

#ifdef CONFIG_OF
static int requestGPIOpin(PCAM_HW_INDEP_INFO pInfo, int *ppin, char *of_name,
			  int value);
static int requestRegulator(PCAM_HW_INDEP_INFO pInfo, struct regulator **reg,
			    char *of_name, int enable);
#endif

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
	pInfo->pSuspend = Suspend;
	pInfo->cameraI2CAddress[0] = 0x78;	// At power on vcam modules will
						// share 0x78 i2c address
	pInfo->cameraI2CAddress[1] = 0x7A;
	pInfo->flip_image = 1;
	pInfo->mirror_image = 0;
	pInfo->edge_enhancement = 0;	// Rocky mipi to parallell IC will
					// introduce artifacts if enabled

#ifdef CONFIG_OF
	// Find torch
	down_read(&leds_list_lock);
	list_for_each_entry(led_cdev, &leds_list, node) {
		if (strcmp(led_cdev->name, "torch-rori-c") == 0) {
			pr_err("*** Found led with name torch-rori-c\n");
			pInfo->torch_cdev = led_cdev;
			break;
		}
		if (strcmp(led_cdev->name, "torch") == 0) {
			pr_err("*** Found led with name torch\n");
			pInfo->torch_cdev = led_cdev;
		}
	}
	up_read(&leds_list_lock);

	ret = requestGPIOpin(pInfo, &pInfo->reset_gpio, "vcam_reset-gpio", 0);
	if (!ret)
		ret = requestRegulator(pInfo, &pInfo->reg_vcm, "rori_vcm", 0);
	if (!ret)
		ret =
		    requestRegulator(pInfo, &pInfo->reg_vcm1i2c,
				     "rori_vcm1_i2c_en", 0);
	if (!ret)
		ret =
		    requestRegulator(pInfo, &pInfo->reg_vcm2i2c,
				     "rori_vcm2_i2c_en", 0);
	if (!ret)
		ret =
		    requestGPIOpin(pInfo, &pInfo->pwdn_gpio, "vcam_pwdn-gpio",
				   1);

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
	if (pInfo->torch_cdev)
		pFlashData->bTorchOn =
		    (pInfo->torch_cdev->brightness) ? TRUE : FALSE;
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
DWORD SetTorchState(PCAM_HW_INDEP_INFO pInfo, VCAMIOCTLFLASH *pFlashData)
{
	if (pInfo->torch_cdev) {
		pInfo->torch_cdev->brightness =
		    pFlashData->bTorchOn ? pInfo->torch_cdev->max_brightness : 0;
		pInfo->torch_cdev->brightness_set(pInfo->torch_cdev,
						  pInfo->torch_cdev->brightness);
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
DWORD WriteVcam(PCAM_HW_INDEP_INFO pInfo, u8 i2cAddress, u16 address, u8 data)
{
	struct i2c_msg msgs[1];
	UCHAR buf[3];

	msgs[0].addr = i2cAddress >> 1;
	msgs[0].flags = 0;
	msgs[0].len = 3;
	msgs[0].buf = buf;
	buf[0] = (address >> 8) & 0xff;
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
	int ret = 0;
#ifdef CONFIG_OF
	if (bEnable) {
		ret = regulator_enable(pInfo->reg_vcm);
		msleep(20);
		gpio_direction_output(pInfo->pwdn_gpio, 0);
		usleep_range(10000, 20000);
		gpio_direction_output(pInfo->reset_gpio, 1);
		usleep_range(10000, 20000);
		ret = regulator_enable(pInfo->reg_vcm2i2c);
		usleep_range(10000, 20000);	//Change vcam2 i2c address from 0x78 to 0x7a
		WriteVcam(pInfo, pInfo->cameraI2CAddress[0], 0x3100,
			  pInfo->cameraI2CAddress[1]);
		ret = regulator_enable(pInfo->reg_vcm1i2c);
	} else {
		ret = regulator_disable(pInfo->reg_vcm1i2c);
		usleep_range(10000, 20000);
		ret = regulator_disable(pInfo->reg_vcm2i2c);
		usleep_range(10000, 20000);
		gpio_direction_output(pInfo->reset_gpio, 0);
		usleep_range(10000, 20000);
		gpio_direction_output(pInfo->pwdn_gpio, 1);
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
}
