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

// Local variables

// Function prototypes

static DWORD GetTorchState(PCAM_HW_INDEP_INFO pInfo,
			   VCAMIOCTLFLASH * pFlashData);
static DWORD SetTorchState(PCAM_HW_INDEP_INFO pInfo,
			   VCAMIOCTLFLASH * pFlashData);
static void EnablePower(PCAM_HW_INDEP_INFO pInfo, BOOL bEnable);
static void Suspend(PCAM_HW_INDEP_INFO pInfo, BOOL bEnable);

//-----------------------------------------------------------------------------
//
// Function: NecoInitHW
//
// This function will perform BSP specific initialization.
//
// Parameters:
//
// Returns:
//      Returns status of init code.
//
//-----------------------------------------------------------------------------

DWORD NecoInitHW(PCAM_HW_INDEP_INFO pInfo)
{
	BOOL ret = TRUE;
	extern struct list_head leds_list;
	extern struct rw_semaphore leds_list_lock;
	struct led_classdev *led_cdev;

	pInfo->hI2C = i2c_get_adapter(2);
	pInfo->eCamModel = OV7740;
	pInfo->pGetTorchState = GetTorchState;
	pInfo->pSetTorchState = SetTorchState;
	pInfo->pEnablePower = EnablePower;
	pInfo->pSuspend = Suspend;

	pInfo->cameraI2CAddress[0] = 0x42;
	pInfo->cameraI2CAddress[1] = 0;

	// Find torch
	down_read(&leds_list_lock);
	list_for_each_entry(led_cdev, &leds_list, node) {
		if (strcmp(led_cdev->name, "torch") == 0)
			pInfo->torch_cdev = led_cdev;
	}
	up_read(&leds_list_lock);
	return ret;
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
DWORD GetTorchState(PCAM_HW_INDEP_INFO pInfo, VCAMIOCTLFLASH * pFlashData)
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
// Function:  BSPSetTorchState
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
	if (pInfo->torch_cdev) {
		pInfo->torch_cdev->brightness = pFlashData->bTorchOn ? 1 : 0;
		pInfo->torch_cdev->brightness_set(pInfo->torch_cdev,
						  pInfo->torch_cdev->
						  brightness);
	}
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
	if (bEnable) {
	} else {
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
}
