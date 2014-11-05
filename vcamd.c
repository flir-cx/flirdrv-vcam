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

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
#include "../arch/arm/mach-imx/hardware.h"
#ifndef __devexit
#define __devexit
#endif
#endif


// Function prototypes
static long VCAM_IOControl(struct file *filep,
		unsigned int cmd, unsigned long arg);
static ssize_t dummyWrite (struct file *filp, const char __user  *buf, size_t count, loff_t *f_pos);
static ssize_t dummyRead (struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
static int dummyRelease (struct inode *inode, struct file *filp);
static int dummyOpen (struct inode *inode, struct file *filp);
static DWORD DoIOControl(PCAM_HW_INDEP_INFO pInfo,
                         DWORD  Ioctl,
                         PUCHAR pBuf,
                         PUCHAR pUserBuf);

static PCAM_HW_INDEP_INFO gpDev;

static struct file_operations vcam_fops =
{
		.owner = THIS_MODULE,
		.unlocked_ioctl = VCAM_IOControl,
		.write = dummyWrite,
		.read = dummyRead,
		.open = dummyOpen,
		.release = dummyRelease,
};

// Code
static int __init VCAM_Init(void)
{
	int i;
	int ret;

    pr_err("VCAM_Init\n");

    // Check that we are not already initiated
    if (gpDev) {
    	pr_err("VCAM already initialized\n");
        return 0;
    }

    // Allocate (and zero-initiate) our control structure.
    gpDev = (PCAM_HW_INDEP_INFO)kmalloc(sizeof(CAM_HW_INDEP_INFO), GFP_KERNEL);
    if ( !gpDev ) {
    	pr_err("Error allocating memory for pDev, VCAM_Init failed\n");
        return -2;
    }

    // Reset all data
    memset (gpDev, 0, sizeof(*gpDev));

    // Register linux driver
    alloc_chrdev_region(&gpDev->vcam_dev, 0, 1, "vcam");
    cdev_init(&gpDev->vcam_cdev, &vcam_fops);
    gpDev->vcam_cdev.owner = THIS_MODULE;
    gpDev->vcam_cdev.ops = &vcam_fops;
    i = cdev_add(&gpDev->vcam_cdev, gpDev->vcam_dev, 1);
    if (i)
    {
    	pr_err("Error adding device driver\n");
        return -3;
    }
    gpDev->pLinuxDevice = platform_device_alloc("vcam", 1);
    if (gpDev->pLinuxDevice == NULL)
    {
    	pr_err("Error adding allocating device\n");
        return -4;
    }
    platform_device_add(gpDev->pLinuxDevice);
	pr_debug("VCAM driver device id %d.%d added\n", MAJOR(gpDev->vcam_dev), MINOR(gpDev->vcam_dev));
	gpDev->vcam_class = class_create(THIS_MODULE, "vcam");
    device_create(gpDev->vcam_class, NULL, gpDev->vcam_dev, NULL, "vcam0");

	// initialize this device instance
    sema_init(&gpDev->semDevice, 1);

    gpDev->hI2C = i2c_get_adapter(2);

    pr_debug("VCAM I2C driver %p\n", gpDev->hI2C);

	// Init hardware
    if (cpu_is_mx51())
    	ret = PicoInitHW(gpDev);
    else
    	ret = NecoInitHW(gpDev);

    if (TRUE != ret)
	{
		pr_err ("CAM_Init - falied to init hardware!\n");
		return -5;
	}

	return 0;
}

static void __devexit VCAM_Deinit(void)
{
    pr_err("VCAM_Deinit\n");

    // make sure this is a valid context
    // if the device is running, stop it
    if (gpDev != NULL)
    {
        i2c_put_adapter(gpDev->hI2C);

        device_destroy(gpDev->vcam_class, gpDev->vcam_dev);
    	class_destroy(gpDev->vcam_class);
        unregister_chrdev_region(gpDev->vcam_dev, 1);
    	platform_device_unregister(gpDev->pLinuxDevice);
       	kfree(gpDev);
		gpDev = NULL;
    }
}

static DWORD DoIOControl(PCAM_HW_INDEP_INFO pInfo,
                         DWORD  Ioctl,
                         PUCHAR pBuf,
                         PUCHAR pUserBuf)
{
    DWORD  dwErr = ERROR_INVALID_PARAMETER;
//    static ULONG ulWdogTime = 5000;    // 5 seconds
//    static BOOL bGPSEnable = FALSE;

    switch (Ioctl)
	{
    case IOCTL_CAM_GET_TEST:
	case IOCTL_CAM_SET_TEST:
    case IOCTL_CAM_GET_ACTIVE:
	case IOCTL_CAM_SET_ACTIVE:
    case IOCTL_CAM_SET_2ND_ACTIVE:
	case IOCTL_CAM_SET_FLASH:

	case IOCTL_CAM_INIT:
	case IOCTL_CAM_GRAB_STILL:
		switch (pInfo->eCamModel)
		{
			case MT9P111:
				return MT9P111_IOControl(pInfo,
										Ioctl,
										pBuf,
										pUserBuf);

			case OV7740:
				return OV7740_IOControl(pInfo,
						Ioctl,
						pBuf,
						pUserBuf);

			default:
				dwErr = ERROR_NOT_SUPPORTED;
				pr_err ("CAM_Init - camera model undetermined!\n");
				break;
		}

        break;

    case IOCTL_CAM_GET_FLASH:
		{
			VCAMIOCTLFLASH * pFlashData = (VCAMIOCTLFLASH *) pBuf;
       		LOCK(pInfo);

			// Callback to platform code to get torch/flash state
			dwErr = pInfo->pGetTorchState(pInfo, pFlashData);
	   		UNLOCK(pInfo);
		}
        break;

	case IOCTL_CAM_GET_CAM_MODEL:
        {
			VCAMIOCTLCAMMODEL * pModel = (VCAMIOCTLCAMMODEL *) pBuf;
       		LOCK(pInfo);
			pModel->eCamModel = pInfo->eCamModel;

			dwErr = ERROR_SUCCESS;
	   		UNLOCK(pInfo);
		}
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
    if (_IOC_DIR(cmd) & _IOC_WRITE)
    {
		pr_debug("VCAM Ioctl %X copy from user: %d\n", cmd, _IOC_SIZE(cmd));
    	dwErr = copy_from_user(tmp, (void *)arg, _IOC_SIZE(cmd));
    	if (dwErr)
    		pr_err("VCAM Copy from user failed: %lu\n", dwErr);
    }

    if (dwErr == ERROR_SUCCESS)
    {
#ifdef DEBUG_VCAM_IOCTL
		pr_err("VCAM Ioctl %X\n", cmd);
#endif
    	dwErr = DoIOControl(gpDev, cmd, tmp, (PUCHAR)arg);
    	if (dwErr)
    		pr_err("VCAM Ioctl failed: %X %ld %d\n", cmd, dwErr, _IOC_NR(cmd));
    }

    if ((dwErr == ERROR_SUCCESS) && (_IOC_DIR(cmd) & _IOC_READ))
    {
        pr_debug("VCAM Ioctl %X copy to user: %u\n", cmd, _IOC_SIZE(cmd));
    	dwErr = copy_to_user((void *)arg, tmp, _IOC_SIZE(cmd));
    	if (dwErr)
    		pr_err("VCAM Copy to user failed: %ld\n", dwErr);
    }
    kfree(tmp);

    return dwErr;
}

static ssize_t dummyWrite (struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
	return count;
}

static ssize_t dummyRead (struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	return count;
}

static int dummyRelease (struct inode *inode, struct file *filp)
{
	return 0;
}

static int dummyOpen (struct inode *inode, struct file *filp)
{
	static BOOL init;

	LOCK(gpDev);
	if (!init)
	{
		// Detect and Init Visual Camera
		switch (gpDev->eCamModel)
		{
			case MT9P111:
				MT9P111_Init(gpDev);
				break;

			case OV7740:
				OV7740_Init(gpDev);
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

module_init (VCAM_Init);
module_exit (VCAM_Deinit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Visual Camera Driver");
MODULE_AUTHOR("Peter Fitger, FLIR Systems AB");
