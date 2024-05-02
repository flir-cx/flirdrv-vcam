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
#include <linux/miscdevice.h>
// Function prototypes
static long vcam_iocontrol(struct file *filep, unsigned int cmd, unsigned long arg);

static const struct file_operations vcam_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = vcam_iocontrol,
};

static int vcam_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct vcam_data *data = devm_kzalloc(dev, sizeof(struct vcam_data), GFP_KERNEL);

	if (!data)
		return -ENOMEM;

	if (!of_device_is_available(dev->of_node)) {
		dev_info(dev, "vcam DISABLED in device-tree");
		return -ENODEV;
	}

	data->miscdev.minor = MISC_DYNAMIC_MINOR;
	data->miscdev.name = devm_kasprintf(dev, GFP_KERNEL, "vcam0");
	data->miscdev.fops = &vcam_fops;
	data->miscdev.parent = dev;

	data->dev = dev;
	dev_set_drvdata(dev, data);
	platform_set_drvdata(pdev, data);

	ret = misc_register(&data->miscdev);
	if (ret) {
		dev_err(dev, "Failed to register miscdev for VCAM driver (error %i\n)\n", ret);
		return ret;
	}

	// initialize this device instance
	sema_init(&data->sem, 1);
	data->flipped_sensor = 0;	//Default, set to 0 if property does not exist or is empty

	if (of_find_property(dev->of_node, "flip-image", NULL))
		data->flipped_sensor = 1;

	ret = platform_inithw(dev);

	if (ret) {
		dev_err(dev, "failed to init hardware!\n");
		goto err_init_failed;
	}

	return ret;

err_init_failed:
	misc_deregister(&data->miscdev);
	return ret;
}

static int vcam_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vcam_data *data = dev_get_drvdata(dev);

	if (data->ops.deinitialize_hw)
		data->ops.deinitialize_hw(dev);

	misc_deregister(&data->miscdev);

	return 0;
}

static int vcam_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct device *dev = &pdev->dev;
	struct vcam_data *data = platform_get_drvdata(pdev);

	if (data->ops.do_iocontrol)
		data->ops.do_iocontrol(dev, IOCTL_CAM_SUSPEND, NULL, NULL);
	return 0;
}

static void vcam_shutdown(struct platform_device *pdev)
{
	vcam_remove(pdev);
}

static int vcam_resume(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vcam_data *data = platform_get_drvdata(pdev);

	if (data->ops.do_iocontrol)
		data->ops.do_iocontrol(dev, IOCTL_CAM_RESUME, NULL, NULL);
	return 0;
}

static const struct of_device_id vcam_match_table[] = {
	{ .compatible = "flir,vcam", },
	{}
};

static struct platform_driver vcam_driver = {
	.probe = vcam_probe,
	.remove = vcam_remove,
	.suspend = vcam_suspend,
	.resume = vcam_resume,
	.shutdown = vcam_shutdown,
	.driver = {
		.of_match_table	= vcam_match_table,
		.name = "vcam",
		.owner = THIS_MODULE,
		/* .pm = &vcam_pm_ops, */
	},
};

static long vcam_iocontrol(struct file *filep, unsigned int cmd, unsigned long arg)
{
	int ret;
	char *tmp;
	struct vcam_data *data = container_of(filep->private_data, struct vcam_data, miscdev);
	struct device *dev = data->dev;

	tmp = kzalloc(_IOC_SIZE(cmd), GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		dev_dbg(dev, "VCAM Ioctl %X copy from user: %d\n", cmd, _IOC_SIZE(cmd));
		ret = copy_from_user(tmp, (void *)arg, _IOC_SIZE(cmd));
		if (ret) {
			dev_err(dev, "VCAM Copy from user failed: %i\n", ret);
			goto err_out;
		}
	}

	switch (cmd) {
	case IOCTL_CAM_GET_FLASH:
		down(&data->sem);
		if (data->ops.get_torchstate)
			ret = data->ops.get_torchstate(dev, (VCAMIOCTLFLASH *)tmp);
		up(&data->sem);
		break;

	case IOCTL_CAM_GET_CAM_MODEL:
		down(&data->sem);
		((VCAMIOCTLCAMMODEL *) tmp)->eCamModel = OV5640;
		ret = ERROR_SUCCESS;
		up(&data->sem);
		break;
	default:
		if (data->ops.do_iocontrol)
			ret = data->ops.do_iocontrol(dev, cmd, tmp, (PUCHAR)arg);
		break;
	}

	if (ret) {
		dev_err(dev, "VCAM Ioctl failed: %X %i %d\n", cmd, ret, _IOC_NR(cmd));
		goto err_out;
	}

	if (_IOC_DIR(cmd) & _IOC_READ) {
		dev_dbg(dev, "VCAM Ioctl %X copy to user: %u\n", cmd, _IOC_SIZE(cmd));
		ret = copy_to_user((void *)arg, tmp, _IOC_SIZE(cmd));
		if (ret) {
			dev_err(dev, "VCAM Copy to user failed: %i\n", ret);
			goto err_out;
		}
	}

err_out:
	kfree(tmp);
	return ret;
}

module_platform_driver(vcam_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Visual Camera Driver");
MODULE_AUTHOR("Peter Fitger, FLIR Systems AB");
MODULE_AUTHOR("Bo Svangård, FLIR Systems AB");
