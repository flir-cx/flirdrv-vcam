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

	ret = misc_register(&vcam_miscdev);
	if (ret) {
		pr_err("Failed to register miscdev for VCAM driver\n");
		return ret;
	}

	// initialize this device instance
	sema_init(&gpDev->semDevice, 1);

	// Init hardware
#ifdef CONFIG_OF
	gpDev->node = of_find_compatible_node(NULL, NULL, "flir,vcam");
	if (of_machine_is_compatible("fsl,imx6dl-ec101") ||
	    of_machine_is_compatible("fsl,imx6dl-ec501"))
		ret = EvcoInitHW(gpDev);
	else
#endif
	if (cpu_is_mx51())
		ret = PicoInitHW(gpDev);
	else if (cpu_is_imx6s())
		ret = NecoInitHW(gpDev);
	else if (cpu_is_imx6q())
		ret = RocoInitHW(gpDev);
	else {
		pr_err("VCAM: Error: Unknown Hardware\n");
		ret = 0;
	}

	if (TRUE != ret)
		pr_err("VCAM_Init - failed to init hardware!\n");

	return (ret) ? 0 : -EIO;
}

static int vcam_remove(struct platform_device *pdev)
{
	pr_info("Removing VCAM driver\n");
	misc_deregister(&vcam_miscdev);

	// make sure this is a valid context
	// if the device is running, stop it
	if (gpDev != NULL) {
		i2c_put_adapter(gpDev->hI2C);

#ifdef CONFIG_OF
		if (gpDev->reg_vcm1i2c)
			regulator_put(gpDev->reg_vcm1i2c);

		if (gpDev->reg_vcm2i2c)
			regulator_put(gpDev->reg_vcm2i2c);

		if (gpDev->reg_vcm)
			regulator_put(gpDev->reg_vcm);

		if (gpDev->pwdn_gpio)
			gpio_free(gpDev->pwdn_gpio);

		if (gpDev->reset_gpio)
			gpio_free(gpDev->reset_gpio);

		if (gpDev->clk_en_gpio)
			gpio_free(gpDev->clk_en_gpio);

		if (gpDev->node)
			of_node_put(gpDev->node);
#endif
	}
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
	if (gpDev->pSuspend)
		gpDev->pSuspend(gpDev, TRUE);
}

static int vcam_resume(struct platform_device *pdev)
{
	if (gpDev->pSuspend)
		gpDev->pSuspend(gpDev, FALSE);
	return 0;
}

static struct platform_driver vcam_driver = {
	.probe = vcam_probe,
	.remove = vcam_remove,
	.suspend = vcam_suspend,
	.shutdown = vcam_shutdown,
	.resume = vcam_resume,
	.driver = {
		   .name = "vcam",
		   .owner = THIS_MODULE,
		    },
};

// Code
static int VCAM_Init(void)
{
	int ret = -EIO;

	pr_info("VCAM_Init\n");

	// Check that we are not already initiated
	if (gpDev) {
		pr_warn("VCAM already initialized\n");
		return 0;
	}

	// Allocate (and zero-initiate) our control structure.
	gpDev =
	    (PCAM_HW_INDEP_INFO) kzalloc(sizeof(CAM_HW_INDEP_INFO), GFP_KERNEL);
	if (!gpDev) {
		pr_err("Error allocating memory for pDev, VCAM_Init failed\n");
		goto err_alloc;
	}

	// Register linux driver
	gpDev->pLinuxDevice = platform_device_alloc("vcam", 1);
	if (gpDev->pLinuxDevice == NULL) {
		pr_err("VCAM: Error adding allocating device\n");
		goto err_platform;
	}

	ret = platform_device_add(gpDev->pLinuxDevice);
	if (ret) {
		pr_err("VCAM: Error adding platform device\n");
		goto err_device;
	}

	ret = platform_driver_register(&vcam_driver);
	if (ret < 0) {
		goto err_driver;
		pr_err("VCAM: Error adding platform driver\n");
	}

	return ret;

err_driver:
	platform_device_unregister(gpDev->pLinuxDevice);
err_device:
	platform_device_put(gpDev->pLinuxDevice);
err_platform:
	kfree(gpDev);
	gpDev = NULL;
err_alloc:
	return ret;
}

static void VCAM_Deinit(void)
{
	pr_err("VCAM_Deinit\n");

	platform_driver_unregister(&vcam_driver);
	platform_device_unregister(gpDev->pLinuxDevice);
	kfree(gpDev);
	gpDev = NULL;
}

static DWORD DoIOControl(PCAM_HW_INDEP_INFO pInfo,
			 DWORD Ioctl, PUCHAR pBuf, PUCHAR pUserBuf)
{
	DWORD dwErr = ERROR_INVALID_PARAMETER;

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
		switch (pInfo->eCamModel) {
		case MT9P111:
			return MT9P111_IOControl(pInfo, Ioctl, pBuf, pUserBuf);

		case OV7740:
			return OV7740_IOControl(pInfo, Ioctl, pBuf, pUserBuf);

		case OV5640:
			return OV5640_IOControl(pInfo, Ioctl, pBuf, pUserBuf);

		default:
			dwErr = ERROR_NOT_SUPPORTED;
			pr_err("CAM_Init - camera model undetermined!\n");
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
		pr_err("FAD: Unsupported IOCTL code %lX\n", Ioctl);
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

	tmp = kzalloc(_IOC_SIZE(cmd), GFP_KERNEL);
	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		pr_debug("VCAM Ioctl %X copy from user: %d\n", cmd,
			 _IOC_SIZE(cmd));
		dwErr = copy_from_user(tmp, (void *)arg, _IOC_SIZE(cmd));
		if (dwErr)
			pr_err("VCAM Copy from user failed: %lu\n", dwErr);
	}

	if (dwErr == ERROR_SUCCESS) {
		dwErr = DoIOControl(gpDev, cmd, tmp, (PUCHAR) arg);
		if (dwErr)
			pr_err("VCAM Ioctl failed: %X %ld %d\n", cmd, dwErr,
			       _IOC_NR(cmd));
	}

	if ((dwErr == ERROR_SUCCESS) && (_IOC_DIR(cmd) & _IOC_READ)) {
		pr_debug("VCAM Ioctl %X copy to user: %u\n", cmd,
			 _IOC_SIZE(cmd));
		dwErr = copy_to_user((void *)arg, tmp, _IOC_SIZE(cmd));
		if (dwErr)
			pr_err("VCAM Copy to user failed: %ld\n", dwErr);
	}
	kfree(tmp);

	return dwErr;
}

static int VCAM_Open(struct inode *inode, struct file *filp)
{
	static BOOL init;

	LOCK(gpDev);
	if (!init) {
		// Detect and Init Visual Camera
		switch (gpDev->eCamModel) {
		case MT9P111:
			MT9P111_Init(gpDev);
			break;

		case OV7740:
			OV7740_Init(gpDev);
			break;

		case OV5640:
			OV5640_Init(gpDev);
			break;

		default:
			pr_err("CAM_Init - camera model undetermined!\n");
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
