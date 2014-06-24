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
    struct class			*vcam_class;
    dev_t 					vcam_dev;			// Major.Minor device number
	struct semaphore		semDevice;			// serialize access to this device's state
	VCAM_CamModel			eCamModel;			// type/model of visual camera module
    struct i2c_adapter 	    *hI2C;
    struct led_classdev     *torch_cdev;

    UCHAR					cameraI2CAddress[2];

    // Function pointers
    DWORD (* pGetTorchState) (struct __CAM_HW_INDEP_INFO * pInfo, VCAMIOCTLFLASH * pFlashData);
    DWORD (* pSetTorchState) (struct __CAM_HW_INDEP_INFO * pInfo, VCAMIOCTLFLASH * pFlashData);
    void  (* pEnablePower) (struct __CAM_HW_INDEP_INFO * pInfo, BOOL bEnable);

} CAM_HW_INDEP_INFO, *PCAM_HW_INDEP_INFO;

// Hardware specific functions
DWORD			NecoInitHW(PCAM_HW_INDEP_INFO pInfo);
DWORD			PicoInitHW(PCAM_HW_INDEP_INFO pInfo);

// Visual camera specific functions
BOOL MT9P111_Init(PCAM_HW_INDEP_INFO pInfo);
DWORD MT9P111_IOControl(PCAM_HW_INDEP_INFO pInfo,
                        DWORD  Ioctl,
                        PUCHAR pBuf,
                        PUCHAR pUserBuf);
BOOL OV7740_Init(PCAM_HW_INDEP_INFO pInfo);
DWORD OV7740_IOControl(PCAM_HW_INDEP_INFO pInfo,
                        DWORD  Ioctl,
                        PUCHAR pBuf,
                        PUCHAR pUserBuf);


#endif //_VCAM_INTERNAL_H_
