/***********************************************************************
 *                                                                     
 * Project: Balthazar
 * $Date: 2013/10/04 $
 * $Author: pfitger $
 *
 * $Id: //depot/Balthazar/Camera/OS/Experimental/linux/drivers/vcam/vcamd.c#1 $
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
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/version.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>

#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/of_regulator.h>
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
#include "../arch/arm/mach-imx/hardware.h"
#define cpu_is_imx6s   cpu_is_imx6dl
#else // LINUX_VERSION_CODE
#include "mach/mx6.h"
#define cpu_is_imx6s   cpu_is_mx6dl
#define cpu_is_imx6q   cpu_is_mx6q
#endif

// Function prototypes
static long VCAM_IOControl(struct file *filep,
			   unsigned int cmd, unsigned long arg);
static int VCAM_Open(struct inode *inode, struct file *filp);
static DWORD DoIOControl(PCAM_HW_INDEP_INFO pInfo,
			 DWORD Ioctl, PUCHAR pBuf, PUCHAR pUserBuf);

static struct platform_device *vcam_platform_device;
static PCAM_HW_INDEP_INFO gpDev;
static const struct file_operations vcam_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = VCAM_IOControl,
	.open = VCAM_Open,
};

static struct miscdevice vcam_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "vcam0",
	.fops = &vcam_fops
};

static int vcam_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;

	//Kludge to avoid two calls to probe function..
	if(gpDev){
		//dev_err(dev, "Gpdev already allocated...\n");
		return 0;
	}

	dev_info(dev, "%s\n", __func__);
	// Allocate (and zero-initiate) our control structure.
	gpDev = (PCAM_HW_INDEP_INFO) kzalloc(sizeof(CAM_HW_INDEP_INFO), GFP_KERNEL);
	if (!gpDev) {
		dev_err(dev, "Error allocating memory for pDev, VCAM_Init failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, gpDev);
	gpDev->pLinuxDevice = pdev;

	ret = misc_register(&vcam_miscdev);
	if (ret) {
		dev_err(dev, "Failed to register miscdev for VCAM driver (error %i\n)\n", ret);
		return ret;
	}

	// initialize this device instance
	sema_init(&gpDev->semDevice, 1);

	gpDev->flipped_sensor = 0; //Default, set to 0 if property does not exist or is empty

	// Init hardware
#ifdef CONFIG_OF
	gpDev->node = of_find_compatible_node(NULL, NULL, "flir,vcam");

	if (of_find_property(gpDev->node, "flip-image", NULL))
		gpDev->flipped_sensor = 1;

	if ((of_machine_is_compatible("fsl,imx6dl-ec101")) ||
	    (of_machine_is_compatible("fsl,imx6dl-ec501"))) {
		ret = EvcoInitHW(gpDev);
	} else {
#endif
	/* if (cpu_is_mx51()) */
	/* 	ret = PicoInitHW(gpDev); */
	/* else if (cpu_is_imx6s()) */
	/* 	ret = NecoInitHW(gpDev); */
		if (cpu_is_imx6q()) {
			ret = RocoInitHW(gpDev);
		} else {
			dev_err(dev, "VCAM: Error: Unknown Hardware\n");
			ret = 0;
		}
#ifdef CONFIG_OF
	}
#endif
	if (ret)  
		dev_err(dev, "failed to init hardware!\n");

	return ret;
}

static int vcam_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	//Kludge to avoid two calls to probe/remove function..
	if(!gpDev){
		/* pr_err("gpDev has already been removed...\n"); */
		return 0;
	}
	dev_info(dev, "Removing VCAM driver\n");

	if ((of_machine_is_compatible("fsl,imx6dl-ec101")) ||
	    (of_machine_is_compatible("fsl,imx6dl-ec501"))) {
		EvcoDeInitHW(gpDev);
	}
	misc_deregister(&vcam_miscdev);

	//Deinitialization has moved to EvcoDeInitHW
	//When reading rocky/pico etc, be sure to add deinitialization

	if (gpDev->node)
		of_node_put(gpDev->node);
	kfree(gpDev);
	gpDev=0;
	return 0;
}

static int vcam_suspend(struct platform_device *pdev, pm_message_t state)
{
	if (gpDev->pSuspend)
		gpDev->pSuspend(gpDev, TRUE);
	return 0;
}

static void vcam_shutdown(struct platform_device *pdev)
{
	vcam_remove(pdev);
}

static int vcam_resume(struct platform_device *pdev)
{
	if (gpDev->pSuspend)
		gpDev->pSuspend(gpDev, FALSE);
	return 0;
}

static struct platform_driver vcam_device_driver = {
	.probe = vcam_probe,
	.remove = vcam_remove,
	.suspend = vcam_suspend,
	.resume = vcam_resume,
	.shutdown = vcam_shutdown,
	.driver = {
		   .name = "vcam",
		    },
};

static int VCAM_Init(void)
{
	int ret = -EIO;

	pr_info("VCAM_Init\n");

	// Register linux driver
	vcam_platform_device = platform_device_alloc("vcam", 1);
	if (! vcam_platform_device) {
		pr_err("VCAM: Error adding allocating device\n");
		return -ENOMEM;
	}

	ret = platform_device_add(vcam_platform_device);
	if (ret < 0) {
		pr_err("VCAM: Error adding platform device\n");
		platform_device_put(vcam_platform_device);
		return ret;
	}

	ret = platform_driver_register(&vcam_device_driver);
	if (ret < 0) {
		pr_err("VCAM: Error adding platform driver\n");
		platform_device_unregister(vcam_platform_device);
	}

	return ret;
}

static void VCAM_Deinit(void)
{
	platform_driver_unregister(&vcam_device_driver);
	platform_device_unregister(vcam_platform_device);
}

static DWORD DoIOControl(PCAM_HW_INDEP_INFO pInfo,
			 DWORD Ioctl, PUCHAR pBuf, PUCHAR pUserBuf)
{
	DWORD dwErr = ERROR_INVALID_PARAMETER;
	struct platform_device *pdev = pInfo->pLinuxDevice;
	struct device *dev = &pdev->dev;

	switch (Ioctl) {
	case IOCTL_CAM_GET_TEST:
	case IOCTL_CAM_SET_TEST:
	case IOCTL_CAM_GET_ACTIVE:
	case IOCTL_CAM_SET_ACTIVE:
	case IOCTL_CAM_SET_2ND_ACTIVE:
	case IOCTL_CAM_SET_FLASH:
	case IOCTL_CAM_SET_FOV:
	case IOCTL_CAM_GET_FOV:
	case IOCTL_CAM_SET_CAMMODE:
	case IOCTL_CAM_INIT:
	case IOCTL_CAM_GRAB_STILL:
	case IOCTL_CAM_MIRROR_ON:
	case IOCTL_CAM_MIRROR_OFF:
	case IOCTL_CAM_FLIP_ON:
	case IOCTL_CAM_FLIP_OFF:
		switch (pInfo->eCamModel) {
		/* case MT9P111: */
		/* 	return MT9P111_IOControl(pInfo, Ioctl, pBuf, pUserBuf); */

		/* case OV7740: */
		/* 	return OV7740_IOControl(pInfo, Ioctl, pBuf, pUserBuf); */

		case OV5640:
			return OV5640_IOControl(pInfo, Ioctl, pBuf, pUserBuf);

		default:
			dwErr = ERROR_NOT_SUPPORTED;
			dev_err(dev, "CAM_Init - camera model undetermined!\n");
			break;
		}
		break;

	case IOCTL_CAM_GET_FLASH:
		{
			VCAMIOCTLFLASH *pFlashData = (VCAMIOCTLFLASH *) pBuf;

			LOCK(pInfo);

			// Callback to platform code to get torch/flash state
			dwErr = pInfo->pGetTorchState(pInfo, pFlashData);
			UNLOCK(pInfo);
		}
		break;

	case IOCTL_CAM_GET_CAM_MODEL:
		{
			VCAMIOCTLCAMMODEL *pModel = (VCAMIOCTLCAMMODEL *) pBuf;

			LOCK(pInfo);
			pModel->eCamModel = pInfo->eCamModel;

			dwErr = ERROR_SUCCESS;
			UNLOCK(pInfo);
		}
		break;

	case IOCTL_CAM_SUSPEND:
		{
			if (gpDev->pSuspend)
				gpDev->pSuspend(gpDev, TRUE);
		}
		dwErr = ERROR_SUCCESS;
		break;

	case IOCTL_CAM_RESUME:
		{
			if (gpDev->pSuspend)
				gpDev->pSuspend(gpDev, FALSE);
		}
		dwErr = ERROR_SUCCESS;
		break;

	default:
		dev_err(dev, "FAD: Unsupported IOCTL code %lX\n", Ioctl);
		dwErr = ERROR_NOT_SUPPORTED;
		break;
	}

	// pass back appropriate response codes
	return dwErr;
}

////////////////////////////////////////////////////////
//
// VCAM_IOControl
//
////////////////////////////////////////////////////////
static long VCAM_IOControl(struct file *filep,
			   unsigned int cmd, unsigned long arg)
{
	DWORD dwErr = ERROR_SUCCESS;
	char *tmp;
	struct platform_device *pdev = gpDev->pLinuxDevice;
	struct device *dev = &pdev->dev;

	tmp = kzalloc(_IOC_SIZE(cmd), GFP_KERNEL);
	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		dev_dbg(dev, "VCAM Ioctl %X copy from user: %d\n", cmd,
			_IOC_SIZE(cmd));
		dwErr = copy_from_user(tmp, (void *)arg, _IOC_SIZE(cmd));
		if (dwErr)
			dev_err(dev, "VCAM Copy from user failed: %lu\n", dwErr);
	}

	if (dwErr == ERROR_SUCCESS) {
		dwErr = DoIOControl(gpDev, cmd, tmp, (PUCHAR) arg);
		if (dwErr)
			dev_err(dev, "VCAM Ioctl failed: %X %ld %d\n", cmd, dwErr,
				_IOC_NR(cmd));
	}

	if ((dwErr == ERROR_SUCCESS) && (_IOC_DIR(cmd) & _IOC_READ)) {
		dev_dbg(dev, "VCAM Ioctl %X copy to user: %u\n", cmd,
			_IOC_SIZE(cmd));
		dwErr = copy_to_user((void *)arg, tmp, _IOC_SIZE(cmd));
		if (dwErr)
			dev_err(dev, "VCAM Copy to user failed: %ld\n", dwErr);
	}
	kfree(tmp);

	return dwErr;
}

static int VCAM_Open(struct inode *inode, struct file *filp)
{
	static BOOL init;
	struct platform_device *pdev = gpDev->pLinuxDevice;
	struct device *dev = &pdev->dev;

	dev_info(dev, "%s\n",__func__);
	LOCK(gpDev);
	if (!init) {
		// Detect and Init Visual Camera
		switch (gpDev->eCamModel) {
		/* case MT9P111: */
		/* 	MT9P111_Init(gpDev); */
		/* 	break; */

		/* case OV7740: */
		/* 	OV7740_Init(gpDev); */
		/* 	break; */

		case OV5640:
			OV5640_Init(gpDev);
			break;

		default:
			dev_err(dev, "CAM_Init - camera model undetermined!\n");
			break;
		}
		init = TRUE;
	}
	UNLOCK(gpDev);

	return 0;
}

module_init(VCAM_Init);
module_exit(VCAM_Deinit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Visual Camera Driver");
MODULE_AUTHOR("Peter Fitger, FLIR Systems AB");
MODULE_AUTHOR("Bo Svangård, FLIR Systems AB");
