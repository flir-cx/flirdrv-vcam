/***********************************************************************
 *                                                                     
 * Project: Balthazar
 * $Date: 2013/05/06 $
 * $Author: fhammars $
 *
 * $Id: //depot/Balthazar/Camera/OS/main/WINCE600/PLATFORM/COMMON/SRC/FLIR/VCAM/OV7740.c#4 $
 *
 * Description of file:
 *   Visual Camera Driver for OV7740 (Astra)
 *
 * Last check-in changelist:
 * $Change: 172358 $
 *
 * Copyright: FLIR Systems AB
 ***********************************************************************/

#include "flir_kernel_os.h"
#include "vcam_internal.h"
#include "i2cdev.h"
#include <linux/i2c.h>
#include "OV7740.h"

static BOOL bCamActive = TRUE;

// Local functions

static BOOL DoI2CWrite(PCAM_HW_INDEP_INFO pInfo,
		       const UCHAR * buf, USHORT elements)
{
	struct i2c_msg msg;
	int ret;
	int i;
	int retries = 10;
	const UCHAR *ptr;

	// Check if camera in use
	msg.addr = pInfo->cameraI2CAddress[0] >> 1;
	ptr = buf;
	msg.flags = 0;
	msg.len = 2;

	for (i = 0; i < elements; i++) {
		msg.buf = (UCHAR *) ptr;
		ret = i2c_transfer(pInfo->hI2C, &msg, 1);

		if (ret <= 0) {
			if (retries-- <= 0) {
				pr_err
				    ("%s failing on element %d of %d\n",
				     __func__, i, elements);
				return FALSE;	// Too many errors, give up
			}
			usleep_range(10000, 20000);
			i--;
			continue;
		}
		ptr += 2;
	}

	return TRUE;
}

BOOL OV7740_Init(PCAM_HW_INDEP_INFO pInfo)
{
	// Camera can not be initialized until FPGA up and running
	return TRUE;
}

static int OV7740_mirror(PCAM_HW_INDEP_INFO pInfo, bool on)
{
	int ret;

	if (on)
		ret =
		    DoI2CWrite(pInfo, ov7740_mirror_on_reg,
			       dim(ov7740_mirror_on_reg));
	else
		ret =
		    DoI2CWrite(pInfo, ov7740_mirror_off_reg,
			       dim(ov7740_mirror_off_reg));
	return ret;
}

DWORD OV7740_IOControl(PCAM_HW_INDEP_INFO pInfo,
		       DWORD Ioctl, PUCHAR pBuf, PUCHAR pUserBuf)
{
	DWORD dwErr = ERROR_INVALID_PARAMETER;
	static BOOL bTestActive;

	switch (Ioctl) {
	case IOCTL_CAM_GET_TEST:
		{
			LOCK(pInfo);
			((VCAMIOCTLTEST *) pBuf)->bTestMode = bTestActive;
			dwErr = ERROR_SUCCESS;
			UNLOCK(pInfo);
		}
		break;

	case IOCTL_CAM_SET_TEST:
		{
			LOCK(pInfo);
			bTestActive =
			    (((VCAMIOCTLTEST *) pBuf)->bTestMode != 0);
			dwErr = ERROR_SUCCESS;
			UNLOCK(pInfo);
		}
		break;

	case IOCTL_CAM_INIT:
		LOCK(pInfo);
		dwErr =
		    (DoI2CWrite
		     (pInfo, I2CDataInitPart1[0],
		      dim(I2CDataInitPart1))) ? ERROR_SUCCESS : -EIO;
		if (bCamActive == FALSE)
			DoI2CWrite(pInfo, I2CDataStandByEnter[0],
				   dim(I2CDataStandByEnter));
		UNLOCK(pInfo);
		break;

	case IOCTL_CAM_GET_ACTIVE:
		{
			VCAMIOCTLACTIVE *pVcamIoctl = (VCAMIOCTLACTIVE *) pBuf;
			LOCK(pInfo);
			pVcamIoctl->bActive = bCamActive;
			dwErr = ERROR_SUCCESS;
			UNLOCK(pInfo);
		}
		break;

	case IOCTL_CAM_SET_ACTIVE:
		{
			BOOL bNewActive;
			BOOL res = TRUE;
			LOCK(pInfo);
			bNewActive = (((VCAMIOCTLACTIVE *) pBuf)->bActive != 0);
			if (bNewActive != bCamActive) {
				if (bNewActive) {
					res =
					    DoI2CWrite(pInfo,
						       I2CDataStandByExit[0],
						       dim(I2CDataStandByExit));

				} else {
					res =
					    DoI2CWrite(pInfo,
						       I2CDataStandByEnter[0],
						       dim
						       (I2CDataStandByEnter));
				}
			}

			if (res) {
				bCamActive = bNewActive;
				pr_err("bCamActive now %d\n", bCamActive);
				dwErr = ERROR_SUCCESS;
			}
			UNLOCK(pInfo);
		}
		break;

	case IOCTL_CAM_SET_FLASH:
		{
			VCAMIOCTLFLASH *pFlashData = (VCAMIOCTLFLASH *) pBuf;

			LOCK(pInfo);

			dwErr = pInfo->pSetTorchState(pInfo, pFlashData);
			UNLOCK(pInfo);
		}
		break;

	case IOCTL_CAM_GRAB_STILL:
		{
			dwErr = ERROR_SUCCESS;
		}
		break;
	case IOCTL_CAM_MIRROR_ON:
		OV7740_mirror(pInfo, TRUE);
		break;
	case IOCTL_CAM_MIRROR_OFF:
		OV7740_mirror(pInfo, FALSE);
		break;
	default:
		pr_err("VCAM Unsupported IOCTL code %lu\n", Ioctl);
		dwErr = ERROR_NOT_SUPPORTED;
		break;
	}

	return dwErr;
}
