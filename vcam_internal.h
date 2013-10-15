/***********************************************************************
 *                                                                     
 * Project: Balthazar
 * $Date: 2013/10/04 $
 * $Author: pfitger $
 *
 * $Id: //depot/Balthazar/Camera/OS/Experimental/linux/drivers/vcam/vcam_internal.h#1 $
 *
 * Description of file:
 *   Visual Camera Driver Header file
 *
 * Last check-in changelist:
 * $Change: 179282 $
 *
 * Copyright: FLIR Systems AB
 ***********************************************************************/
#ifndef _VCAM_INTERNAL_H_
#define _VCAM_INTERNAL_H_

#include "vcam.h"

// Definitions
#define	LOCK(pd)			down(&pd->semDevice)
#define	UNLOCK(pd)			up(&pd->semDevice)

#define	dim(x)				(sizeof(x) / sizeof(x[0]))

// this structure keeps track of the device instance
typedef struct __CAM_HW_INDEP_INFO {
	struct platform_device *pLinuxDevice;
    struct cdev 			vcam_cdev;			// Linux character device
    dev_t 					vcam_dev;			// Major.Minor device number
	struct semaphore		semDevice;			// serialize access to this device's state
	VCAM_CamModel			eCamModel;			// type/model of visual camera module
    struct i2c_adapter 	   *hI2C;
} CAM_HW_INDEP_INFO, *PCAM_HW_INDEP_INFO;

// Hardware specific functions
DWORD			BSPInitHW(PCAM_HW_INDEP_INFO pInfo);
DWORD			BSPDeinitHW(void);
VCAM_CamModel	BSPGetCameraModel(void);
UCHAR			BSPGetCameraI2CAddress(DWORD dwCamNo);
BOOL            BspGetReinitAfterStandby(void);
DWORD			BSPGetTorchState(VCAMIOCTLFLASH * pFlashData);
DWORD			BSPSetTorchState(VCAMIOCTLFLASH * pFlashData);
BOOL			BSPSetLightLimit(void);
void            BspEnablePower(PCAM_HW_INDEP_INFO pInfo, BOOL bEnable);

// Visual camera specific functions
BOOL MT9P111_Init(PCAM_HW_INDEP_INFO pInfo);
DWORD MT9P111_IOControl(PCAM_HW_INDEP_INFO pInfo,
                        DWORD  Ioctl,
                        PUCHAR pBuf,
                        PUCHAR pUserBuf);


#endif //_VCAM_INTERNAL_H_
