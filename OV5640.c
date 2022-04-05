// SPDX-License-Identifier: GPL-2.0-or-later
/***********************************************************************
 *
 * Project: Balthazar
 *
 * Description of file:
 *   Visual Camera Driver
 *
 * Copyright: FLIR Systems AB
 ***********************************************************************/

#include "flir_kernel_os.h"
#include "vcam_internal.h"
#include "i2cdev.h"
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include "OV5640.h"

static s32 ov5640_write_reg(PCAM_HW_INDEP_INFO pInfo, u16 reg, u8 val,
			    CAM_NO cam)
{
	struct platform_device *pdev = pInfo->pLinuxDevice;
	struct device *dev = &pdev->dev;
	u8 buf[3] = { 0 };
	int i, retries = 50;
	struct i2c_msg msgs[1];
	int retval;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = val;

	msgs[0].addr = pInfo->cameraI2CAddress[cam] >> 1;
	msgs[0].flags = I2C_M_TEN;
	msgs[0].buf = buf;
	msgs[0].len = 3;

	for (i = 0; i < retries; ++i) {
		retval = i2c_transfer(pInfo->hI2C, msgs, 1);
		if (retval <= 0) {
			dev_err(dev, "i2c_transfer() failed, try no. %d\n", i);
			usleep_range(10000, 20000);
		} else {
			break;
		}
	}

	if (retval <= 0) {
		return ERROR_NOT_SUPPORTED;
	}

	return 0;
}

static s32 ov5640_read_reg(PCAM_HW_INDEP_INFO pInfo, u16 reg, u8 *val,
			   CAM_NO cam)
{
	struct platform_device *pdev = pInfo->pLinuxDevice;
	struct device *dev = &pdev->dev;
	u8 buf[2] = { 0 };
	struct i2c_msg msgs[1];
	int i, retval, retries = 50;

	msgs[0].addr = pInfo->cameraI2CAddress[cam] >> 1;
	msgs[0].flags = I2C_M_TEN;
	msgs[0].len = 2;
	msgs[0].buf = buf;

	/* register to read */
	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	for (i = 0; i < retries; ++i) {
		retval = i2c_transfer(pInfo->hI2C, msgs, 1);
		if (retval <= 0) {
			dev_err(dev, "i2c_transfer() failed, try %d\n", i);
			usleep_range(10000, 20000);
		} else {
			break;
		}
	}

	if (retval <= 0) {
		return ERROR_NOT_SUPPORTED;
	}

	/* Send in master receive mode. */
	msgs[0].flags |= I2C_M_RD;	/* see i2c_master_recv() */
	msgs[0].len = 1;
	msgs[0].buf = val;

	for (i = 0; i < retries; ++i) {
		retval = i2c_transfer(pInfo->hI2C, msgs, 1);
		if (retval <= 0) {
			dev_err(dev, "i2c_transfer() failed, try %d\n", i);
			usleep_range(10000, 20000);
		} else {
			break;
		}
	}

	if (retval <= 0) {
		return ERROR_NOT_SUPPORTED;
	}

	return 0;
}

static s32 ov5640_mod_reg(PCAM_HW_INDEP_INFO pInfo, u16 reg, u8 mask, u8 val,
			  CAM_NO cam)
{
	u8 readval;
	s32 ret;

	ret = ov5640_read_reg(pInfo, reg, &readval, cam);
	if (ret) {
		return ret;
	}

	readval &= ~mask;
	val &= mask;
	val |= readval;

	return ov5640_write_reg(pInfo, reg, val, cam);
}

static int ov5640_get_otp_memory(PCAM_HW_INDEP_INFO pInfo, u8 *otp_memory,
				 int n, CAM_NO cam)
{
	struct platform_device *pdev = pInfo->pLinuxDevice;
	struct device *dev = &pdev->dev;
	int retval;
	int i;

	/* Enable OTP block and OPT clock */
	retval = ov5640_mod_reg(pInfo, OV5640_SYSTEM_RESET00, BIT(4), 0, cam);
	if (retval < 0) {
		dev_err(dev, "ov5640: failed to enable OTP module\n");
		return -1;
	}

	retval =
	    ov5640_mod_reg(pInfo, OV5640_CLOCK_ENABLE00, BIT(4), BIT(4), cam);
	if (retval < 0) {
		dev_err(dev, "ov5640: failed to enable OTP clock\n");
		return -1;
	}

	/* According to OTP read example in datasheet */
	retval = ov5640_write_reg(pInfo, OV5640_OTP_PROGRAM_CTRL, 0, cam);
	if (retval < 0) {
		dev_err(dev, "ov5640: failed to disable OTP programming\n");
		return -1;
	}

	retval = ov5640_write_reg(pInfo, OV5640_OTP_READ_CTRL, 0, cam);
	if (retval < 0) {
		dev_err(dev, "ov5640: failed to disable OTP read\n");
		return -1;
	}

	retval = ov5640_write_reg(pInfo, OV5640_OTP_READ_CTRL, 1, cam);
	if (retval < 0) {
		dev_err(dev, "ov5640: failed to enable OTP read\n");
		return -1;
	}

	/* delay 1ms according to datasheet */
	msleep(1);
	for (i = 0; i < n && retval >= 0; i++) {
		retval =
		    ov5640_read_reg(pInfo, OV5640_OTP_START_ADDR + i,
				    &otp_memory[i], cam);
		dev_dbg(dev, "otp[0x%x] 0x%x %c\n", OV5640_OTP_START_ADDR + i,
			otp_memory[i], otp_memory[i]);
	}

	if (retval < 0) {
		dev_err(dev, "ov5640_read_reg() failed\n");
		return -1;
	}

	/* delay 10ms according to datasheet */
	msleep(10);
	retval = ov5640_write_reg(pInfo, OV5640_OTP_READ_CTRL, 0, cam);
	if (retval < 0) {
		dev_err(dev, "ov5640: failed to disable OTP read\n");
		return -1;
	}

	/* Disable OTP block and OPT clock. Letting the block and
	 * clock enabled can cause the sensor to fail to start
	 * streaming frames. */
	retval =
	    ov5640_mod_reg(pInfo, OV5640_SYSTEM_RESET00, BIT(4), BIT(4), cam);
	if (retval < 0) {
		pr_err("ov5640: failed to disable OTP module\n");
		return -1;
	}

	retval = ov5640_mod_reg(pInfo, OV5640_CLOCK_ENABLE00, BIT(4), 0, cam);
	if (retval < 0) {
		pr_err("ov5640: failed to disable OTP clock\n");
		return -1;
	}

	return 0;
}

static int ov5640_get_sensor_models(PCAM_HW_INDEP_INFO pInfo, CAM_NO camera)
{
	struct platform_device *pdev = pInfo->pLinuxDevice;
	struct device *dev = &pdev->dev;
	u8 otp_memory[OV5640_OTP_END_ADDR - OV5640_OTP_START_ADDR + 1];
	DWORD cam;
	DWORD cam_first = (camera == CAM_2) ? CAM_2 : CAM_1;
	DWORD cam_last = (camera == CAM_1) ? CAM_1 : CAM_2;
	int ret = 0;

	for (cam = cam_first; cam <= cam_last; cam++) {
		/* Read content in OTP memory */
		ret =
		    ov5640_get_otp_memory(pInfo, otp_memory,
					  OV5640_OTP_END_ADDR -
					  OV5640_OTP_START_ADDR + 1, cam);
		if (ret) {
			dev_err(dev, "ov5640_get_otp_memory() failed\n");
			continue;
		}

		/* The sensor model can be determined by the content of the
		 * OTP memory. There are two ways the memory can be programmed
		 * from the vendor. The memory can either contain a string
		 * specifying the model or a specific register can contain an
		 * integer value. */

		/* Test if sensor is programmed with a string specifying model */
		if (strncmp
		    (otp_memory, OV5640_SENSOR_MODEL_HIGH_K,
		     strlen(OV5640_SENSOR_MODEL_HIGH_K)) == 0) {
			dev_err(dev,
				"ov5640: Sensor model id: \"%s\" (High K)\n",
				OV5640_SENSOR_MODEL_HIGH_K);
			pInfo->sensor_model[cam] = OV5640_HIGH_K;
			continue;
		}

		/* Test if sensor is programmed with a sensor id integer */
		if (otp_memory
		    [OV5640_SENSOR_MODEL_ID_ADDR - OV5640_OTP_START_ADDR] ==
		    OV5640_SENSOR_MODEL_HIGH_K_ID) {
			dev_err(dev, "ov5640: Sensor model id: High K\n");
			pInfo->sensor_model[cam] = OV5640_HIGH_K;
			continue;
		}

		/* Assume the sensor is a standard model */
		dev_info(dev, "ov5640: Standard sensor model\n");
		pInfo->sensor_model[cam] = OV5640_STANDARD;
	}

	return ret;
}

static int ov5640_get_sensor_model_conf(PCAM_HW_INDEP_INFO pInfo,
					struct reg_value **value, CAM_NO camera)
{
	struct platform_device *pdev = pInfo->pLinuxDevice;
	struct device *dev = &pdev->dev;
	int ret;

	*value = NULL;
	if (pInfo->sensor_model[camera] == OV5640_HIGH_K) {
		*value = ov5640_setting_High_K;
	}

	return 0;
}

static int ov5640_set_sensor_model_conf(PCAM_HW_INDEP_INFO pInfo, CAM_NO camera)
{
	struct platform_device *pdev = pInfo->pLinuxDevice;
	struct device *dev = &pdev->dev;
	DWORD cam;
	DWORD cam_first = (camera == CAM_2) ? CAM_2 : CAM_1;
	DWORD cam_last = (camera == CAM_1) ? CAM_1 : CAM_2;
	struct reg_value *pModeSetting = NULL;
	int ret = 0;

	for (cam = cam_first; cam <= cam_last; cam++) {
		ov5640_get_sensor_model_conf(pInfo, &pModeSetting, cam);
		if (pModeSetting == NULL) {
			continue;
		}

		ret =
		    OV5640_DoI2CWrite(pInfo, pModeSetting, dim(pModeSetting),
				      cam);
		if (ret) {
			dev_err(dev,
				"OV5640_DoI2CWrite() failed for camera %lu\n",
				cam);
			continue;
		}
	}

	return ret;
}

BOOL OV5640_DoI2CWrite(PCAM_HW_INDEP_INFO pInfo,
		       struct reg_value *pMode, USHORT elements, CAM_NO camera)
{
	struct i2c_msg msgs[1];
	struct platform_device *pdev = pInfo->pLinuxDevice;
	struct device *dev = &pdev->dev;
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
					dev_err(dev,
						"failing on element %d of %d\n",
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

void OV5640_enable_stream(PCAM_HW_INDEP_INFO pInfo, CAM_NO camera, bool enable)
{
	if (enable)
		OV5640_DoI2CWrite(pInfo, stream_on, dim(stream_on), camera);
	else
		OV5640_DoI2CWrite(pInfo, stream_off, dim(stream_off), camera);
}

void OV5640_MipiSuspend(PCAM_HW_INDEP_INFO pInfo, BOOL bSuspend)
{
	struct reg_value mipi_pwdn = { 0x300e, 0x45 };

	if (bSuspend) {
		/* Set register 0x300E[4:3] to 2'b11 before the PWDN pin is set high */
		mipi_pwdn.u8Val |= 0x18;
	}
	OV5640_DoI2CWrite(pInfo, &mipi_pwdn, 1, g_camera);
}

static int OV5640_nightmode_enable(PCAM_HW_INDEP_INFO pInfo, CAM_NO camera,
				   bool enable)
{
	int ret;

	if (enable)
		ret = OV5640_DoI2CWrite(pInfo, &night_mode_on, 1, camera);
	else
		ret = OV5640_DoI2CWrite(pInfo, &night_mode_off, 1, camera);
	return ret;
}

static DWORD OV5640_mirror_enable(PCAM_HW_INDEP_INFO pInfo, CAM_NO camera,
				  bool enable)
{
	DWORD ret;

	if (enable)
		ret = OV5640_DoI2CWrite(pInfo,
					ov5640_mirror_on_reg,
					dim(ov5640_mirror_on_reg), camera);
	else
		ret = OV5640_DoI2CWrite(pInfo,
					ov5640_mirror_off_reg,
					dim(ov5640_mirror_off_reg), camera);
	return ret;
}

static int OV5640_autofocus_enable(PCAM_HW_INDEP_INFO pInfo, CAM_NO camera,
				   bool enable)
{
	int ret;

	if (enable)
		ret = OV5640_DoI2CWrite(pInfo, &autofocus_on, 1, camera);
	else
		ret = OV5640_DoI2CWrite(pInfo, &autofocus_off, 1, camera);
	return ret;
}

/*Set vcam exposure value*/
static void OV5640_set_exposure(PCAM_HW_INDEP_INFO pInfo, CAM_NO camera,
				int exp)
{
	struct reg_value temp;

	temp.u16RegAddr = 0x3500;
	temp.u8Val = ((exp >> 16) & 0x0f);
	OV5640_DoI2CWrite(pInfo, &temp, 1, camera);

	temp.u16RegAddr = 0x3501;
	temp.u8Val = ((exp >> 8) & 0xff);
	OV5640_DoI2CWrite(pInfo, &temp, 1, camera);

	temp.u16RegAddr = 0x3502;
	temp.u8Val = (exp & 0xf0);
	OV5640_DoI2CWrite(pInfo, &temp, 1, camera);

}

static void nightmode_on_off_work(struct work_struct *work)
{
	CAM_HW_INDEP_INFO *pInfo =
	    container_of(work, CAM_HW_INDEP_INFO, nightmode_work);

	msleep(1000);
	OV5640_nightmode_enable(pInfo, pInfo->cam, FALSE);
	msleep(1000);
	OV5640_nightmode_enable(pInfo, pInfo->cam, TRUE);
}

static BOOL OV5640_set_5MP(PCAM_HW_INDEP_INFO pInfo, CAM_NO camera)
{
	int ret;
	struct platform_device *pdev = pInfo->pLinuxDevice;
	struct device *dev = &pdev->dev;

	OV5640_enable_stream(pInfo, camera, FALSE);
	ret = OV5640_DoI2CWrite(pInfo, ov5640_init_setting_9fps_5MP,
				dim(ov5640_init_setting_9fps_5MP), camera);
	if (ret) {
		dev_err(dev, "Failed to call OV5640_DoI2CWrite\n");
		return ret;
	}

	/* Write model specific configuration */
	ov5640_set_sensor_model_conf(pInfo, camera);

	if (pInfo->edge_enhancement) {
		ret =
		    OV5640_DoI2CWrite(pInfo, ov5640_edge_enhancement,
				      dim(ov5640_edge_enhancement), camera);
		if (ret) {
			dev_err(dev, "Failed to call OV5640_DoI2CWrite\n");
			return ret;
		}
	}
	// Set default flip
	ret = OV5640_FlipImage(pInfo, FALSE);
	if (ret) {
		dev_err(dev, "Failed to call OV5640_FlipImage\n");
		return ret;
	}

	ret = OV5640_mirror_enable(pInfo, camera, pInfo->flipped_sensor);
	if (ret) {
		dev_err(dev, "Failed to call OV5640_mirror_enable\n");
		return ret;
	}

	OV5640_enable_stream(pInfo, camera, TRUE);
	return ret;
}

static BOOL OV5640_set_fov(PCAM_HW_INDEP_INFO pInfo, CAM_NO camera, int fov)
{
	int ret;
	struct platform_device *pdev = pInfo->pLinuxDevice;
	struct device *dev = &pdev->dev;

	OV5640_enable_stream(pInfo, camera, FALSE);

	switch (fov) {
	case 54:
		ret =
		    OV5640_DoI2CWrite(pInfo,
				      ov5640_setting_30fps_1280_960_HFOV54,
				      dim(ov5640_setting_30fps_1280_960_HFOV54),
				      camera);
		g_vcamFOV = fov;
		break;

	case 39:
		ret =
		    OV5640_DoI2CWrite(pInfo,
				      ov5640_setting_30fps_1280_960_HFOV39,
				      dim(ov5640_setting_30fps_1280_960_HFOV39),
				      camera);
		g_vcamFOV = fov;
		break;

	case 28:
		ret =
		    OV5640_DoI2CWrite(pInfo,
				      ov5640_setting_30fps_1280_960_HFOV28,
				      dim(ov5640_setting_30fps_1280_960_HFOV28),
				      camera);
		g_vcamFOV = fov;
		break;

	default:
		dev_err(dev, "VCAM: Unsupported fov: %d\n", fov);
		ret = ERROR_NOT_SUPPORTED;
	}

	OV5640_enable_stream(pInfo, camera, TRUE);

	pInfo->cam = camera;
	schedule_work(&pInfo->nightmode_work);
	return ret;
}

DWORD OV5640_FlipImage(PCAM_HW_INDEP_INFO pInfo, bool flip)
{
	DWORD dwErr = ERROR_SUCCESS;

	if (flip) {
		if (pInfo->flipped_sensor) {
			dwErr = OV5640_DoI2CWrite(pInfo,
						  ov5640_flip_off_reg,
						  dim(ov5640_flip_off_reg),
						  g_camera);
		} else {
			dwErr = OV5640_DoI2CWrite(pInfo,
						  ov5640_flip_on_reg,
						  dim(ov5640_flip_on_reg),
						  g_camera);
		}
	} else {
		if (pInfo->flipped_sensor) {
			dwErr = OV5640_DoI2CWrite(pInfo,
						  ov5640_flip_on_reg,
						  dim(ov5640_flip_on_reg),
						  g_camera);
		} else {
			dwErr = OV5640_DoI2CWrite(pInfo,
						  ov5640_flip_off_reg,
						  dim(ov5640_flip_off_reg),
						  g_camera);
		}
	}

	return dwErr;
}

static BOOL initCamera(PCAM_HW_INDEP_INFO pInfo, BOOL fullInit, CAM_NO camera)
{
	BOOL ret = ERROR_SUCCESS;
	struct platform_device *pdev = pInfo->pLinuxDevice;
	struct device *dev = &pdev->dev;

	dev_info(dev, "initCamera\n");

	/* Read the OTP memory before the initial configuration. This
	 * is the only time the otp memory is read. If read after the
	 * initial settings configuration is loaded the sensor can
	 * fail to start to stream frames. */
	ov5640_get_sensor_models(pInfo, camera);

	ret = OV5640_DoI2CWrite(pInfo, ov5640_init_setting_9fps_5MP,
				dim(ov5640_init_setting_9fps_5MP), camera);
	if (ret) {
		dev_err(dev, "Failed to call OV5640_DoI2CWrite\n");
		return ret;
	}

	/* Write model specific configuration */
	ov5640_set_sensor_model_conf(pInfo, camera);

	// Set default flip
	ret = OV5640_FlipImage(pInfo, FALSE);
	if (ret) {
		dev_err(dev, "Failed in call OV5640_FlipImage\n");
		return ret;
	}

	ret = OV5640_mirror_enable(pInfo, camera, pInfo->flipped_sensor);
	if (ret) {
		dev_err(dev, "Failed in call OV5640_mirror_enable\n");
		return ret;
	}

	if (pInfo->edge_enhancement) {
		ret = OV5640_DoI2CWrite(pInfo, ov5640_edge_enhancement,
					dim(ov5640_edge_enhancement), camera);
		if (ret) {
			dev_err(dev, "Failed in call OV5640_DoI2CWrite..\n");
			return ret;
		}
	}

	ret = OV5640_set_fov(pInfo, camera, 54);
	if (ret) {
		dev_err(dev, "Failed in call OV5640_set_fov\n");
		return ret;
	}

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
	struct platform_device *pdev = pInfo->pLinuxDevice;
	struct device *dev = &pdev->dev;

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
				dev_err(dev, "bCamActive for cam %d now %d\n",
					cam, bCamActive[cam]);
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
				dev_err(dev,
					"VCAM Unsupported IOCTL_CAM_SET_CAMMODE %d\n",
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
	case IOCTL_CAM_FLIP_ON:
		LOCK(pInfo);
		dwErr = OV5640_FlipImage(pInfo, true);
		UNLOCK(pInfo);
		break;
	case IOCTL_CAM_FLIP_OFF:
		LOCK(pInfo);
		dwErr = OV5640_FlipImage(pInfo, false);
		UNLOCK(pInfo);
		break;
	default:
		dev_err(dev, "VCAM Unsupported IOCTL code %lu\n", Ioctl);
		dwErr = ERROR_NOT_SUPPORTED;
		break;
	}
	return dwErr;
}
