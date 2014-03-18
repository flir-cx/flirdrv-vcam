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

// Local variables

// Function prototypes

static DWORD GetTorchState(VCAMIOCTLFLASH * pFlashData);
static DWORD SetTorchState(VCAMIOCTLFLASH * pFlashData);
static void EnablePower(PCAM_HW_INDEP_INFO pInfo, BOOL bEnable);

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

    pInfo->eCamModel = OV7740;
    pInfo->pGetTorchState = GetTorchState;
    pInfo->pSetTorchState = SetTorchState;
    pInfo->pEnablePower = EnablePower;

    pInfo->cameraI2CAddress[0] = 0x42;
    pInfo->cameraI2CAddress[1] = 0;

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
DWORD GetTorchState(VCAMIOCTLFLASH * pFlashData)
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
DWORD SetTorchState(VCAMIOCTLFLASH * pFlashData)
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
    }
    else
    {
    }
}
