/* SPDX-License-Identifier: GPL-2.0-or-later */
/***********************************************************************
 *
 * Project: Balthazar
 *
 * Description of file:
 *   Visual Camera Driver Header file
 *
 * Copyright: FLIR Systems AB
 ***********************************************************************/
#ifndef _VCAM_INTERNAL_H_
#define _VCAM_INTERNAL_H_

#include "vcam.h"
#include "flir_kernel_os.h"

// Definitions
#define	LOCK(pd)		down(&pd->semDevice)
#define	UNLOCK(pd)		up(&pd->semDevice)
#define	dim(x)			(sizeof(x) / sizeof((x)[0]))

typedef enum { CAM_1, CAM_2, CAM_ALL } CAM_NO;

enum sensor_model {
	OV5640_STANDARD,
	OV5640_HIGH_K
};

// this structure keeps track of the device instance
typedef struct __CAM_HW_INDEP_INFO {
	struct platform_device *pLinuxDevice;
	struct semaphore semDevice;	// serialize access to this device's state
	VCAM_CamModel eCamModel;	// type/model of visual camera module
	enum sensor_model sensor_model[2];
	struct i2c_adapter *hI2C;
	struct led_classdev *torch_cdev;
	UCHAR cameraI2CAddress[2];
	int flipped_sensor;	//if true the sensor is mounted upside/down.
	int edge_enhancement;	//enable increased edge enhancement in camera sensor
#ifdef CONFIG_OF
	struct device_node *node;
	int pwdn_gpio;
	int reset_gpio;
	int clk_en_gpio;

	struct regulator *reg_vcm1i2c;
	struct regulator *reg_vcm2i2c;
	struct regulator *reg_vcm;
#endif

	// Function pointers
	DWORD(*pGetTorchState) (struct __CAM_HW_INDEP_INFO *pInfo, VCAMIOCTLFLASH * pFlashData);
	DWORD(*pSetTorchState) (struct __CAM_HW_INDEP_INFO *pInfo, VCAMIOCTLFLASH * pFlashData);
	void (*pEnablePower)(struct __CAM_HW_INDEP_INFO *pInfo, BOOL bEnable);
	void (*pSuspend)(struct __CAM_HW_INDEP_INFO *pInfo, BOOL bSuspend);

	struct work_struct nightmode_work;
	int cam;
	int camera;
	int fov;
	int CamActive[CAM_ALL];
} CAM_HW_INDEP_INFO, *PCAM_HW_INDEP_INFO;

// Hardware specific functions
DWORD NecoInitHW(PCAM_HW_INDEP_INFO pInfo);
DWORD PicoInitHW(PCAM_HW_INDEP_INFO pInfo);
DWORD RocoInitHW(PCAM_HW_INDEP_INFO pInfo);
DWORD EvcoInitHW(PCAM_HW_INDEP_INFO pInfo);
DWORD EvcoDeInitHW(PCAM_HW_INDEP_INFO pInfo);
DWORD EocoInitHW(struct device *dev);
DWORD EocoDeInitHW(struct device *dev);
// Visual camera specific functions
BOOL MT9P111_Init(PCAM_HW_INDEP_INFO pInfo);
DWORD MT9P111_IOControl(PCAM_HW_INDEP_INFO pInfo, DWORD Ioctl, PUCHAR pBuf, PUCHAR pUserBuf);
BOOL OV7740_Init(PCAM_HW_INDEP_INFO pInfo);
DWORD OV7740_IOControl(PCAM_HW_INDEP_INFO pInfo, DWORD Ioctl, PUCHAR pBuf, PUCHAR pUserBuf);

int OV5640_Init(struct device *dev);
DWORD OV5640_IOControl(PCAM_HW_INDEP_INFO pInfo, DWORD Ioctl, PUCHAR pBuf, PUCHAR pUserBuf);
void OV5640_MipiSuspend(PCAM_HW_INDEP_INFO pInfo, BOOL bSuspend);
int OV5640_reinit(PCAM_HW_INDEP_INFO pInfo);

#endif //_VCAM_INTERNAL_H_
