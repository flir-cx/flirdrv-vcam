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
#include <linux/version.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>

#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/of_regulator.h>
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
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
static DWORD DoIOControl(PCAM_HW_INDEP_INFO pInfo,
			 DWORD Ioctl, PUCHAR pBuf, PUCHAR pUserBuf);

#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 19, 0)
static struct platform_device *vcam_platform_device;
#endif
static PCAM_HW_INDEP_INFO gpDev;
static const struct file_operations vcam_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = VCAM_IOControl,
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

	// Allocate (and zero-initiate) our control structure.
	gpDev = (PCAM_HW_INDEP_INFO) devm_kzalloc(dev, sizeof(CAM_HW_INDEP_INFO), GFP_KERNEL);
	if (!gpDev) {
		dev_err(dev, "Error allocating memory for pDev, VCAM_Init failed\n");
		return -ENOMEM;
	}

	dev_set_drvdata(dev, gpDev);
	platform_set_drvdata(pdev, gpDev);
	gpDev->pLinuxDevice = pdev;

	ret = misc_register(&vcam_miscdev);
	if (ret) {
		dev_err(dev, "Failed to register miscdev for VCAM driver (error %i\n)\n", ret);
		return ret;
	}

	// initialize this device instance
	sema_init(&gpDev->semDevice, 1);
	gpDev->flipped_sensor = 0;	//Default, set to 0 if property does not exist or is empty

	// Init hardware
#ifdef CONFIG_OF
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 19, 0)
	gpDev->node = of_find_compatible_node(NULL, NULL, "flir,vcam");
#else
	gpDev->node = dev->of_node;
#endif


	if (of_find_property(gpDev->node, "flip-image", NULL))
		gpDev->flipped_sensor = 1;

	if (of_machine_is_compatible("fsl,imx6qp-eoco")) {
		ret = EocoInitHW(dev);
	} else if ((of_machine_is_compatible("fsl,imx6dl-ec101")) ||
		   (of_machine_is_compatible("fsl,imx6dl-ec501"))) {
		ret = EvcoInitHW(gpDev);
	} else
#endif
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 19, 0)
		/* if (cpu_is_mx51()) */
		/*      ret = PicoInitHW(gpDev); */
		/* else if (cpu_is_imx6s()) */
		/*      ret = NecoInitHW(gpDev); */
		if (cpu_is_imx6q()) {
			ret = RocoInitHW(gpDev);
		} else
#endif
		{
			dev_err(dev, "VCAM: Error: Unknown Hardware\n");
			ret = 0;
		}

	if (ret) {
		dev_err(dev, "failed to init hardware!\n");
		misc_deregister(&vcam_miscdev);
	}
	return ret;
}

static int vcam_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	PCAM_HW_INDEP_INFO pInfo = dev_get_drvdata(dev);

	if (pInfo->deinitialize_hw)
		pInfo->deinitialize_hw(dev);

	misc_deregister(&vcam_miscdev);

	return 0;
}

static int vcam_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct device *dev = &pdev->dev;

	if (gpDev->do_iocontrol)
		gpDev->do_iocontrol(dev, IOCTL_CAM_SUSPEND, NULL, NULL);
	return 0;
}

static void vcam_shutdown(struct platform_device *pdev)
{
	vcam_remove(pdev);
}

static int vcam_resume(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	if (gpDev->do_iocontrol)
		gpDev->do_iocontrol(dev, IOCTL_CAM_RESUME, NULL, NULL);
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

#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 19, 0)
	vcam_platform_device = platform_device_alloc("vcam", 1);
	if (!vcam_platform_device) {
		pr_err("VCAM: Error adding allocating device\n");
		return -ENOMEM;
	}

	ret = platform_device_add(vcam_platform_device);
	if (ret < 0) {
		pr_err("VCAM: Error adding platform device\n");
		platform_device_put(vcam_platform_device);
		return ret;
	}
#endif

	ret = platform_driver_register(&vcam_device_driver);
	if (ret < 0) {
		pr_err("VCAM: Error adding platform driver\n");
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 19, 0)
		platform_device_unregister(vcam_platform_device);
#endif
	}

	return ret;
}

static void VCAM_Deinit(void)
{
	platform_driver_unregister(&vcam_device_driver);
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 19, 0)
	platform_device_unregister(vcam_platform_device);
#endif
}

static DWORD DoIOControl(PCAM_HW_INDEP_INFO pInfo, DWORD Ioctl, PUCHAR pBuf, PUCHAR pUserBuf)
{
	DWORD dwErr = ERROR_INVALID_PARAMETER;
	struct platform_device *pdev = pInfo->pLinuxDevice;
	struct device *dev = &pdev->dev;

	switch (Ioctl) {
	case IOCTL_CAM_GET_FLASH:
		LOCK(pInfo);
		if (pInfo->pGetTorchState)
			dwErr = pInfo->pGetTorchState(pInfo, (VCAMIOCTLFLASH *)pBuf);
		UNLOCK(pInfo);
		break;

	case IOCTL_CAM_GET_CAM_MODEL:
		LOCK(pInfo);
		((VCAMIOCTLCAMMODEL *) pBuf)->eCamModel = pInfo->eCamModel;
		dwErr = ERROR_SUCCESS;
		UNLOCK(pInfo);
		break;
	default:
		if (gpDev->do_iocontrol)
			dwErr = pInfo->do_iocontrol(dev, Ioctl, pBuf, pUserBuf);
		break;
	}
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
			dev_err(dev, "VCAM Copy from user failed: %lu\n",
				dwErr);
	}

	if (dwErr == ERROR_SUCCESS) {
		dwErr = DoIOControl(gpDev, cmd, tmp, (PUCHAR) arg);
		if (dwErr)
			dev_err(dev, "VCAM Ioctl failed: %X %ld %d\n", cmd,
				dwErr, _IOC_NR(cmd));
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

module_init(VCAM_Init);
module_exit(VCAM_Deinit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Visual Camera Driver");
MODULE_AUTHOR("Peter Fitger, FLIR Systems AB");
MODULE_AUTHOR("Bo Svangård, FLIR Systems AB");
