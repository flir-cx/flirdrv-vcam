/***********************************************************************
 *                                                                     
 * Project: Balthazar
 * $Date: 2013/10/04 $
 * $Author: pfitger $
 *
 * $Id: //depot/Balthazar/Camera/OS/Experimental/linux/drivers/vcam/MT9P111.c#1 $
 *
 * Description of file:
 *   Visual Camera Driver
 *
 * Last check-in changelist:
 * $Change: 179282 $
 *
 * Copyright: FLIR Systems AB
 ***********************************************************************/

#include "flir_kernel_os.h"
#include "vcam_internal.h"
#include "i2cdev.h"
#include <linux/i2c.h>
#include "MT9P11.h"
// Local functions

static unsigned long GetTickCount(void)
{
	struct timeval tv;

	do_gettimeofday(&tv);

	return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

static BOOL DoI2CWrite(PCAM_HW_INDEP_INFO pInfo,
		       const UCHAR * buf,
		       USHORT elements,
		       UCHAR pollReg,
		       USHORT pollValue, USHORT timeout, CAM_NO camera)
{
	struct i2c_msg msgs[2];
	DWORD ret;
	int i;
	int retries = 50;
	DWORD cam;
	const UCHAR *ptr;
	BOOL ready;
	DWORD start;
	UCHAR cmd[2];
	UCHAR stat[2];
	USHORT status = 0;
	DWORD cam_first = (camera == CAM_2) ? CAM_2 : CAM_1;
	DWORD cam_last = (camera == CAM_1) ? CAM_1 : CAM_2;

	for (cam = cam_first; cam <= cam_last; cam++) {
		// Check if camera in use
		msgs[0].addr = pInfo->cameraI2CAddress[cam] >> 1;
		if (msgs[0].addr == 0)
			continue;
		msgs[1].addr = msgs[0].addr;

		if (pollReg) {
			// Poll camera to be ready for transfer
			ready = FALSE;
			start = GetTickCount();
			while (!ready) {
				usleep_range(10000, 20000);

				msgs[0].flags = 0;
				msgs[0].len = 2;
				msgs[0].buf = cmd;
				msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
				msgs[1].len = 2;
				msgs[1].buf = stat;

				cmd[0] = 0;
				cmd[1] = pollReg;
				ret = i2c_transfer(pInfo->hI2C, msgs, 2);

				if (ret > 0) {
					status = (stat[0] << 8) | stat[1];
					if (status == pollValue) {
						pr_err
							("%s cam %lu got result %X in %lu ms\n",
							 __func__,
							 cam, status,
							 GetTickCount() - start);
						ready = TRUE;
					}
				}
				if ((GetTickCount() - start) > timeout)
					break;
			}
			if (!ready) {
				pr_err
					("%s cam %lu timeout with result %X\n",
					 __func__,
					 cam, status);
			}
		} else if (cam == cam_first) {
			// Fixed delay
			msleep(timeout);
		}

		ptr = buf;
		msgs[0].flags = 0;

		for (i = 0; i < elements; i++) {
			msgs[0].len = ptr[0];
			msgs[0].buf = (UCHAR *) (ptr + 1);
			ret = i2c_transfer(pInfo->hI2C, msgs, 1);

			if (ret <= 0) {
				if (retries-- <= 0) {
					pr_err
						("%s failing on element %d of %d\n",
						 __func__,
						 i, elements);
					return FALSE;	// Too many errors, give up
				}
				usleep_range(10000, 20000);
				i--;
				continue;
			}
			ptr += 5;
		}
	}

	return TRUE;
}

static BOOL initCamera(PCAM_HW_INDEP_INFO pInfo, BOOL fullInit, CAM_NO cam)
{
	BOOL ret = TRUE;

	ret =
	    DoI2CWrite(pInfo, I2CDataInitPart1[0], dim(I2CDataInitPart1), 0x18,
		       0x4009, 100, cam);
	if (ret)
		ret =
		    DoI2CWrite(pInfo, I2CDataInitPart2[0],
			       dim(I2CDataInitPart2), 0x18, 0x2008, 1000, cam);
	if (ret)
		ret =
		    DoI2CWrite(pInfo, I2CDataInitPart3[0],
			       dim(I2CDataInitPart3), 0x18, 0x4009, 1000, cam);
	if (ret)
		ret =
		    DoI2CWrite(pInfo, I2CDataInitPart4[0],
			       dim(I2CDataInitPart4), 0, 0, 100, cam);
	if (ret)
		ret =
		    DoI2CWrite(pInfo, I2CDataInitPart5[0],
			       dim(I2CDataInitPart5), 0, 0, 200, cam);
	if (ret)
		ret =
		    DoI2CWrite(pInfo, I2CDataInitPart6[0],
			       dim(I2CDataInitPart6), 0, 0, 200, cam);
	if (ret)
		ret =
		    DoI2CWrite(pInfo, I2CDataInitPart8[0],
			       dim(I2CDataInitPart8), 0, 0, 200, cam);
	if (ret)
		ret =
		    DoI2CWrite(pInfo, I2CDataInitPart9[0],
			       dim(I2CDataInitPart9), 0, 0, 200, cam);
	if (ret)
		ret =
		    DoI2CWrite(pInfo, I2CDataInitPart10[0],
			       dim(I2CDataInitPart10), 0x16, 0x0447, 1000, cam);
	if (ret)
		ret =
		    DoI2CWrite(pInfo, I2CDataInitPart7[0],
			       dim(I2CDataInitPart7), 0, 0, 200, cam);

	return ret;
}

BOOL MT9P111_Init(PCAM_HW_INDEP_INFO pInfo)
{
	initCamera(pInfo, TRUE, CAM_ALL);
	if (bCamActive[CAM_1] == FALSE)
		DoI2CWrite(pInfo, I2CDataStandByEnter[0],
			   dim(I2CDataStandByEnter), 0, 0, 0, CAM_1);
	if (bCamActive[CAM_2] == FALSE)
		DoI2CWrite(pInfo, I2CDataStandByEnter[0],
			   dim(I2CDataStandByEnter), 0, 0, 0, CAM_2);
	return TRUE;
}

DWORD MT9P111_IOControl(PCAM_HW_INDEP_INFO pInfo,
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

	case IOCTL_CAM_GET_ACTIVE:
		{
			VCAMIOCTLACTIVE *pVcamIoctl = (VCAMIOCTLACTIVE *) pBuf;

			LOCK(pInfo);
			pVcamIoctl->bActive = bCamActive[CAM_1]
			    || bCamActive[CAM_2];
			dwErr = ERROR_SUCCESS;
			UNLOCK(pInfo);
		}
		break;

	case IOCTL_CAM_SET_ACTIVE:
	case IOCTL_CAM_SET_2ND_ACTIVE:
		{
			BOOL bNewActive;
			BOOL res = TRUE;
			CAM_NO cam =
			    (Ioctl == IOCTL_CAM_SET_ACTIVE) ? CAM_1 : CAM_2;
			LOCK(pInfo);
			bNewActive = (((VCAMIOCTLACTIVE *) pBuf)->bActive != 0);
			if (bNewActive != bCamActive[cam]) {
				if (bNewActive) {
					res =
					    DoI2CWrite(pInfo,
						       I2CDataStandByExit[0],
						       dim(I2CDataStandByExit),
						       0, 0, 0, cam);

				} else {
					res =
					    DoI2CWrite(pInfo,
						       I2CDataStandByEnter[0],
						       dim(I2CDataStandByEnter),
						       0, 0, 0, cam);
				}
			}

			if (res) {
				bCamActive[cam] = bNewActive;
				pr_err("bCamActive for cam %d now %d\n", cam,
				       bCamActive[cam]);
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
			CAM_NO cam =
			    (bCamActive[CAM_1] == TRUE) ? CAM_1 : CAM_2;

			LOCK(pInfo);
			// Tell camera to switch to context B
			DoI2CWrite(pInfo, I2CDataGrab[0], dim(I2CDataGrab), 0,
				   0, 0, cam);

			dwErr = ERROR_SUCCESS;
			UNLOCK(pInfo);
		}
		break;

	default:
		pr_err("VCAM Unsupported IOCTL code %lu\n", Ioctl);
		dwErr = ERROR_NOT_SUPPORTED;
		break;
	}

	return dwErr;
}

