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
#include "OV5640.h"

static BOOL DoI2CWrite(PCAM_HW_INDEP_INFO pInfo,
		       struct reg_value *pMode, USHORT elements, CAM_NO camera)
{
	struct i2c_msg msgs[1];
	int i, retval = 0;
	int retries = 50;
	DWORD cam;
	DWORD cam_first = (camera == CAM_2) ? CAM_2 : CAM_1;
	DWORD cam_last = (camera == CAM_1) ? CAM_1 : CAM_2;
	u8 buf[3] = { 0 };
	u16 RegAddr = 0;
	u8 Val = 0;

	for (cam = cam_first; cam <= cam_last; cam++) {
		// Check if camera in use
		msgs[0].addr = pInfo->cameraI2CAddress[cam] >> 1;
		if (msgs[0].addr == 0)
			continue;
		msgs[0].flags = 0;
		msgs[0].buf = buf;
		msgs[0].len = 3;

		for (i = 0; i < elements; ++i) {
			RegAddr = pMode[i].u16RegAddr;
			Val = pMode[i].u8Val;

			buf[0] = RegAddr >> 8;
			buf[1] = RegAddr & 0xff;
			buf[2] = Val;

			retval = i2c_transfer(pInfo->hI2C, msgs, 1);

			if (retval <= 0) {
				if (retries-- <= 0) {
					pr_err
						("VCAM: %s failing on element %d of %d\n",
						 __func__,
						 i, elements);
					return ERROR_NOT_SUPPORTED;	// Too many errors, give up
				}
				usleep_range(10000, 20000);
				i--;
				continue;
			}
		}
	}

	return ERROR_SUCCESS;
}

void OV5640_enable_stream(PCAM_HW_INDEP_INFO pInfo, CAM_NO cam, bool enable)
{
	if (enable)
		DoI2CWrite(pInfo, stream_on, dim(stream_on), cam);
	else
		DoI2CWrite(pInfo, stream_off, dim(stream_off), cam);
}

void OV5640_MipiSuspend(PCAM_HW_INDEP_INFO pInfo, BOOL bSuspend)
{
	struct reg_value mipi_pwdn = { 0x300e, 0x45 };

	if (bSuspend) {
		/* Set register 0x300E[4:3] to 2'b11 before the PWDN pin is set high */
		mipi_pwdn.u8Val |= 0x18;
	}
	DoI2CWrite(pInfo, &mipi_pwdn, 1, g_camera);
}

void OV5640_nightmode_off(PCAM_HW_INDEP_INFO pInfo, CAM_NO cam)
{
	struct reg_value night_mode_off = { 0x3a00, 0x78 };	//night mode off

	DoI2CWrite(pInfo, &night_mode_off, 1, cam);
}

void OV5640_nightmode_on(PCAM_HW_INDEP_INFO pInfo, CAM_NO cam)
{
	struct reg_value night_mode_on = { 0x3a00, 0x7c };	//night mode on

	DoI2CWrite(pInfo, &night_mode_on, 1, cam);
}

/*Turn vcam autofocus on*/
void OV5640_autofocus_on(PCAM_HW_INDEP_INFO pInfo, CAM_NO camera)
{
	struct reg_value buff = { 0x3022, 0x04 };

	DoI2CWrite(pInfo, &buff, 1, camera);
}

/*Turn vcam autofocus off*/
void OV5640_autofocus_off(PCAM_HW_INDEP_INFO pInfo, CAM_NO camera)
{
	struct reg_value buff = { 0x3022, 0x00 };

	DoI2CWrite(pInfo, &buff, 1, camera);
}

/*Set vcam exposure value*/
static void OV5640_set_exposure(PCAM_HW_INDEP_INFO pInfo, CAM_NO camera,
				int exp)
{
	struct reg_value temp;

	temp.u16RegAddr = 0x3500;
	temp.u8Val = ((exp >> 16) & 0x0f);
	DoI2CWrite(pInfo, &temp, 1, camera);

	temp.u16RegAddr = 0x3501;
	temp.u8Val = ((exp >> 8) & 0xff);
	DoI2CWrite(pInfo, &temp, 1, camera);

	temp.u16RegAddr = 0x3502;
	temp.u8Val = (exp & 0xf0);
	DoI2CWrite(pInfo, &temp, 1, camera);

}

static void nightmode_on_off_work(struct work_struct *work)
{
	CAM_HW_INDEP_INFO *pInfo =
	    container_of(work, CAM_HW_INDEP_INFO, nightmode_work);

	// pr_err("VCAM turning nightmode off and on\n");
	msleep(1000);
	OV5640_nightmode_off(pInfo, pInfo->cam);
	msleep(1000);
	OV5640_nightmode_on(pInfo, pInfo->cam);

}

static BOOL OV5640_set_5MP(PCAM_HW_INDEP_INFO pInfo, CAM_NO cam)
{
	int ret;

	OV5640_enable_stream(pInfo, cam, FALSE);
	ret =
	    DoI2CWrite(pInfo, ov5640_init_setting_9fps_5MP,
		       dim(ov5640_init_setting_9fps_5MP), cam);
	if (ret)
		return ret;

	if (pInfo->edge_enhancement) {
		ret =
		    DoI2CWrite(pInfo, ov5640_edge_enhancement,
			       dim(ov5640_edge_enhancement), cam);
		if (ret)
			return ret;
	}

	if (pInfo->flip_image) {
		ret =
		  DoI2CWrite(pInfo, ov5640_flip_reg, dim(ov5640_flip_reg),
				 cam);
		if (ret)
			return ret;
	}
	if (pInfo->mirror_image) {
		ret =
		  DoI2CWrite(pInfo, ov5640_mirror_on_reg, dim(ov5640_mirror_on_reg),
				 cam);
		if (ret)
			return ret;
	}

	OV5640_enable_stream(pInfo, cam, TRUE);
	return ret;
}

static BOOL OV5640_set_fov(PCAM_HW_INDEP_INFO pInfo, CAM_NO cam, int fov)
{
	int ret;

	OV5640_enable_stream(pInfo, cam, FALSE);

	switch (fov) {
	case 54:
		ret =
		    DoI2CWrite(pInfo, ov5640_setting_30fps_1280_960_HFOV54,
			       dim(ov5640_setting_30fps_1280_960_HFOV54), cam);
		g_vcamFOV = fov;
		break;

	case 39:
		ret =
		    DoI2CWrite(pInfo, ov5640_setting_30fps_1280_960_HFOV39,
			       dim(ov5640_setting_30fps_1280_960_HFOV39), cam);
		g_vcamFOV = fov;
		break;

	case 28:
		ret =
		    DoI2CWrite(pInfo, ov5640_setting_30fps_1280_960_HFOV28,
			       dim(ov5640_setting_30fps_1280_960_HFOV28), cam);
		g_vcamFOV = fov;
		break;

	default:
		pr_err("VCAM: Unsupported fov: %d\n", fov);
		ret = ERROR_NOT_SUPPORTED;
	}

	OV5640_enable_stream(pInfo, cam, TRUE);

	pInfo->cam = cam;
	schedule_work(&pInfo->nightmode_work);
	return ret;
}

static BOOL initCamera(PCAM_HW_INDEP_INFO pInfo, BOOL fullInit, CAM_NO cam)
{
	BOOL ret = ERROR_SUCCESS;

	ret =
	    DoI2CWrite(pInfo, ov5640_init_setting_9fps_5MP,
		       dim(ov5640_init_setting_9fps_5MP), cam);
	if (ret)
		return ret;

	if (pInfo->flip_image) {
		ret =
		    DoI2CWrite(pInfo, ov5640_flip_reg, dim(ov5640_flip_reg),
				 cam);
		if (ret)
			return ret;
	}

	if (pInfo->mirror_image) {
		ret =
			DoI2CWrite(pInfo, ov5640_mirror_on_reg, dim(ov5640_mirror_on_reg),
				 camera);
		if (ret)
			return ret;
	}

	if (pInfo->edge_enhancement) {
		ret =
			DoI2CWrite(pInfo, ov5640_edge_enhancement,
				   dim(ov5640_edge_enhancement), cam);
		if (ret)
			return ret;
	}

	ret = OV5640_set_fov(pInfo, cam, 54);
	if (ret)
		return ret;

	return ret;
}

BOOL OV5640_reinit(PCAM_HW_INDEP_INFO pInfo)
{
	if (!init)
		return FALSE;

	OV5640_set_5MP(pInfo, g_camera);

	OV5640_set_fov(pInfo, g_camera, g_vcamFOV);

	return TRUE;
}

BOOL OV5640_Init(PCAM_HW_INDEP_INFO pInfo)
{
	INIT_WORK(&pInfo->nightmode_work, nightmode_on_off_work);

	if (pInfo->cameraI2CAddress[1] == 0)	//Only 1 active camera
		g_camera = CAM_1;
	initCamera(pInfo, TRUE, g_camera);
	init = 1;
	return TRUE;
}

DWORD OV5640_IOControl(PCAM_HW_INDEP_INFO pInfo,
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

	case IOCTL_CAM_INIT:
		dwErr = ERROR_SUCCESS;
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
			//set fast exposure to compensate for led brightness
			if (pFlashData->bTorchOn)
				OV5640_set_exposure(pInfo, g_camera, 0x2000);

			UNLOCK(pInfo);
		}
		break;

	case IOCTL_CAM_SET_CAMMODE:
		{
			VCAMIOCTLCAMMODE *pMode = (VCAMIOCTLCAMMODE *) pBuf;

			LOCK(pInfo);
			switch (pMode->eCamMode) {
			case VCAM_STILL:
				//set camera to 5MP full size mode
				dwErr = OV5640_set_5MP(pInfo, g_camera);
				msleep(800);
				break;

			case VCAM_DRAFT:
				//restore last known fov
				dwErr =
				    OV5640_set_fov(pInfo, g_camera, g_vcamFOV);
				// must wait for auto exposure control (AEC)
				// and auto gain control (AGC) to adjust image brightness
				msleep(500);
				break;

			case VCAM_UNDEFINED:
			case VCAM_RESET:
			default:
				pr_err
				    ("VCAM Unsupported IOCTL_CAM_SET_CAMMODE %d\n",
				     pMode->eCamMode);
				break;
			}
			UNLOCK(pInfo);
		}
		break;

	case IOCTL_CAM_SET_FOV:
		{
			VCAMIOCTLFOV *pVcamFOV = (VCAMIOCTLFOV *) pBuf;

			LOCK(pInfo);
			dwErr = OV5640_set_fov(pInfo, g_camera, pVcamFOV->fov);
			UNLOCK(pInfo);
		}
		break;
	case IOCTL_CAM_GET_FOV:
		LOCK(pInfo);
		((VCAMIOCTLFOV *) pBuf)->fov = g_vcamFOV;
		dwErr = ERROR_SUCCESS;
		UNLOCK(pInfo);
		break;

	case IOCTL_CAM_MIRROR_ON:
		dwErr = OV5640_mirror_enable(pInfo, g_camera, TRUE);
		break;
	case IOCTL_CAM_MIRROR_OFF:
		dwErr = OV5640_mirror_enable(pInfo, g_camera, FALSE);
		break;
	default:
		pr_err("VCAM Unsupported IOCTL code %lu\n", Ioctl);
		dwErr = ERROR_NOT_SUPPORTED;
		break;
	}
	return dwErr;
}
