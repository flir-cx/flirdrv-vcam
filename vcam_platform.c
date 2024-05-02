#include "flir_kernel_os.h"
#include "vcam_ioctl.h"
#include "vcam_internal.h"
#include "i2cdev.h"
#include "faddev.h"
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/platform_device.h>

#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/of_regulator.h>
#include "ov5640.h"
// Function prototypes
static void set_power(struct device *dev, bool enable);
static DWORD get_torchstate(struct device *dev, VCAMIOCTLFLASH *pFlashData);
static DWORD set_torchstate(struct device *dev, VCAMIOCTLFLASH *pFlashData);
static void set_suspend(struct device *dev, bool enable);
static int do_iocontrol(struct device *dev, int cmd, PUCHAR buf, PUCHAR userbuf);
struct led_classdev *find_torch(void);
static DWORD deinitialize_hw(struct device *dev);

static ssize_t vcam_eoco_power_store(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct vcam_data *data = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 0, &val) < 0)
		return -EINVAL;
	data->ops.set_power(dev, val);
	if (val) {
		ov5640_init(dev);
	}
	return count;
}

static ssize_t vcam_eoco_power_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct vcam_data *data = dev_get_drvdata(dev);
	if (of_machine_is_compatible("fsl,imx6qp-eoco")) {
		sprintf(buf, "VCAM OV5640 Power state %s\n",
			regulator_is_enabled(data->reg_vcm1i2c) ? "on":"off");
	}
	return strlen(buf);
}


static DEVICE_ATTR(vcam_eoco_power, 0644, vcam_eoco_power_show, vcam_eoco_power_store);

static struct attribute *vcam_eoco_attrs[] = {
	&dev_attr_vcam_eoco_power.attr,
	NULL
};

static const struct attribute_group vcam_eoco_groups = {
	.attrs = vcam_eoco_attrs,
};

int vcam_eoco_create_sysfs_attributes(struct device *dev)
{
	int ret = -EIO;

	ret = sysfs_create_group(&dev->kobj, &vcam_eoco_groups);
	if (ret)
		dev_err(dev, "failed to add sys fs entry\n");
	return ret;
}

void vcam_eoco_remove_sysfs_attributes(struct device *dev)
{
	sysfs_remove_group(&dev->kobj, &vcam_eoco_groups);
}





//-----------------------------------------------------------------------------
//
// Function:  set_power
//
// This function will control standby/on GPIO usage
//
// Parameters:
//
// Returns:
//
//-----------------------------------------------------------------------------
static void set_power(struct device *dev, bool enable)
{
	int ret = 0;
	struct vcam_data *data = dev_get_drvdata(dev);
	if (enable) {
		ret = regulator_enable(data->reg_vcm);
		usleep_range(1000, 10000);
		if (of_machine_is_compatible("fsl,imx6qp-eoco")) {
			gpio_direction_output(data->clk_en_gpio, 1);
		} else {
			gpio_direction_output(data->clk_en_gpio, 0);
		} 
		gpio_direction_output(data->pwdn_gpio, 0);
		usleep_range(1000, 10000);
		if (of_machine_is_compatible("fsl,imx6qp-eoco")) {
			gpio_direction_output(data->reset_gpio, 0);
		} else {
			gpio_direction_output(data->reset_gpio, 1);
		}
		if (of_machine_is_compatible("fsl,imx6qp-eoco")) {
			ret = regulator_enable(data->reg_vcm1i2c);
		}
	} else {
		if (of_machine_is_compatible("fsl,imx6qp-eoco")) {
			regulator_disable(data->reg_vcm1i2c);
		}
		if (of_machine_is_compatible("fsl,imx6qp-eoco")) {
			gpio_direction_output(data->reset_gpio, 1);
		} else {
			gpio_direction_output(data->reset_gpio, 0);
		}			
		usleep_range(10000, 20000);
		gpio_direction_output(data->pwdn_gpio, 1);
		if (of_machine_is_compatible("fsl,imx6qp-eoco")) {
			gpio_direction_output(data->clk_en_gpio, 0);
		} else {
			gpio_direction_output(data->clk_en_gpio, 1);
		}
		usleep_range(10000, 20000);
		ret = regulator_disable(data->reg_vcm);
	}
}



//-----------------------------------------------------------------------------
//
// Function: EocoInitHW
//
// This function will perform BSP specific initialization.
//
// Parameters:
//
// Returns:
//      Returns status of init code.
//
//-----------------------------------------------------------------------------

DWORD PlatformInitHW(struct device *dev)
{
	int ret = 0;
	struct vcam_data *data = dev_get_drvdata(dev);


	data->i2c_address = 0x78;
	data->edge_enhancement = 1;
	data->ops.get_torchstate = get_torchstate;
	data->ops.set_torchstate = set_torchstate;
	data->ops.do_iocontrol = do_iocontrol;
	data->ops.set_power = set_power;
	data->ops.deinitialize_hw = deinitialize_hw;

	if (of_machine_is_compatible("fsl,imx6qp-eoco")) {
		data->i2c_bus = i2c_get_adapter(0);
	} else {
		data->i2c_bus = i2c_get_adapter(2);
	}

	data->reset_gpio = of_get_named_gpio_flags(dev->of_node, "vcam_reset-gpio", 0, NULL);
	if (gpio_is_valid(data->reset_gpio)) {
		ret = devm_gpio_request_one(dev, data->reset_gpio, GPIOF_OUT_INIT_LOW, "vcam_reset-gpio");
		if (ret) {
			dev_err(dev, "Failed registering pin vcam_reset-gpio (err %i)\n", ret);
			return ret;
		}
	} else {
		dev_err(dev, "vcam_reset not detected\n");
		return -EIO;
	}

	data->pwdn_gpio = of_get_named_gpio_flags(dev->of_node, "vcam_pwdn-gpio", 0, NULL);
	if (gpio_is_valid(data->pwdn_gpio)) {
		ret = devm_gpio_request_one(dev, data->pwdn_gpio, GPIOF_OUT_INIT_HIGH, "vcam_pwdn-gpio");
		if (ret) {
			dev_err(dev, "Failed registering pin vcam_pwdn-gpio (err %i)\n", ret);
			return ret;
		}
	} else {
		dev_err(dev, "vcam_pwdn not detected\n");
		return -EIO;
	}

	data->clk_en_gpio = of_get_named_gpio_flags(dev->of_node, "vcam_clk_en-gpio", 0, NULL);
	if (gpio_is_valid(data->clk_en_gpio)) {
		ret = devm_gpio_request_one(dev, data->clk_en_gpio, GPIOF_OUT_INIT_HIGH, "vcam_clk_en-gpio");
		if (ret) {
			dev_err(dev, "Failed registering pin vcam_clk_en-gpio (err %i)\n", ret);
			return ret;
		}
	} else {
		dev_err(dev, "vcam_clk_en-gpio not detected\n");
		return -EIO;
	}

	if (of_machine_is_compatible("fsl,imx6qp-eoco")) {
		data->reg_vcm = devm_regulator_get(dev, "eodc_dovdd");
		if (IS_ERR(data->reg_vcm)) {
			dev_err(dev, "VCAM: Error fetching regulator VCM_DOVDD\n");
			return -EIO;
		}
	} else {
		data->reg_vcm = regulator_get(dev, "VCM_DOVDD");
		if (IS_ERR(data->reg_vcm)) {
			dev_err(dev, "VCAM: Error on %s get\n", "VCM_DOVDD");
			return -EIO;
		}
	}


	if (of_machine_is_compatible("fsl,imx6qp-eoco")) {
		data->reg_vcm1i2c = devm_regulator_get(dev, "EODC_I2C_ENABLE");
		if (IS_ERR(data->reg_vcm1i2c)) {
			dev_err(dev, "VCAM: Error fetching regulator EODC_I2C_ENABLE\n");
			return -EIO;
		}
	}

	data->ops.set_power(dev, true);
	ret = ov5640_init(dev);
	if (ret) {
		dev_err(dev, "error during initialization of OV5640\n");
	}

	ret = vcam_eoco_create_sysfs_attributes(dev);
	if (ret)
		goto out_eoco_sysfs;
	
	ret = ov5640_create_sysfs_attributes(dev);
	if (ret)
		goto out_sysfs;

	return ret;
	
out_sysfs:
	vcam_eoco_remove_sysfs_attributes(dev);
	
out_eoco_sysfs:
	return ret;
}


//-----------------------------------------------------------------------------
//
// Function:  get_torchstate
//
// This function will return torch and flash state.
//
// Parameters:
//
// Returns:
//
//-----------------------------------------------------------------------------
DWORD get_torchstate(struct device *dev, VCAMIOCTLFLASH *pFlashData)
{
	int ret;
	struct led_classdev *led = find_torch();

	if (led) {
		pFlashData->bTorchOn = (led->brightness) ? TRUE : FALSE;
		ret = ERROR_SUCCESS;
	} else {
		pFlashData->bTorchOn = FALSE;
		dev_err_once(dev, "Failed to find LED Torch\n");

		//Here we want to return ERROR_INVALID_HANDLE, but due to appcore and webapplications, we need
		//this to succeed, until underlying software is able to handle fails here!
		// ret = ERROR_INVALID_HANDLE;
		ret = ERROR_SUCCESS;
	}

	pFlashData->bFlashOn = FALSE;
	return ret;
}


//-----------------------------------------------------------------------------
//
// Function:  set_torchstate
//
// This function will set torch and flash state.
//
// Parameters:
//
// Returns:
//
//-----------------------------------------------------------------------------
DWORD set_torchstate(struct device *dev, VCAMIOCTLFLASH *pFlashData)
{
	int ret;
	struct led_classdev *led = find_torch();

	if (led) {
		led->brightness =
		    pFlashData->bTorchOn ? led->max_brightness : 0;
		led->brightness_set(led, led->brightness);
		ret = ERROR_SUCCESS;
	} else {
		dev_err_once(dev, "Failed to find LED Flash\n");

		//Here we want to return ERROR_INVALID_HANDLE, but due to appcore and webapplications, we need
		//this to succeed, until underlying software is able to handle fails here!
		// ret = ERROR_INVALID_HANDLE;
		ret = ERROR_SUCCESS;
	}

	return ret;
}

//-----------------------------------------------------------------------------
//
// Function: set_supend
//
// This function will handle suspend and resume
//
// Parameters:
//
// Returns:
//
//-----------------------------------------------------------------------------
static void set_suspend(struct device *dev, bool enable)
{
	if (enable) {
		set_power(dev, false);
	} else {
		set_power(dev, true);
		ov5640_init(dev);
	}
}

static int do_iocontrol(struct device *dev, int cmd, PUCHAR buf, PUCHAR userbuf)
{
	int ret;

	switch (cmd) {
	case IOCTL_CAM_GET_ACTIVE:
		((VCAMIOCTLACTIVE *) buf)->bActive = true;
		ret = ERROR_SUCCESS;
		break;
	case IOCTL_CAM_SET_ACTIVE:
	case IOCTL_CAM_SET_2ND_ACTIVE:
		dev_warn(0, "IOCTL_CAM_SET_ACTIVE inactivated, only one camera..\n");
		ret = ERROR_NOT_SUPPORTED;
		break;
	case IOCTL_CAM_SUSPEND:
	case IOCTL_CAM_RESUME:
		set_suspend(dev, (cmd == IOCTL_CAM_SUSPEND));
		ret = 0;
		break;
	default:
		ret = ov5640_ioctl(dev, cmd, buf, userbuf);
	}
	return ret;
}

//-----------------------------------------------------------------------------
//
// Function:  find_torch
//
// This function will return LED named "torch" from the kernel list of LEDS
//
// Parameters: None
//
// Returns: struct *led_cdev NULL if not found
//
//-----------------------------------------------------------------------------
struct led_classdev *find_torch(void)
{
	extern struct list_head leds_list;
	extern struct rw_semaphore leds_list_lock;
	/* Find torch */
	struct led_classdev *led_cdev, *led = NULL;

	down_read(&leds_list_lock);
	list_for_each_entry(led_cdev, &leds_list, node) {
		if (led_cdev && led_cdev->name &&
		    strcmp(led_cdev->name, "torch") == 0) {
			led = led_cdev;
			break;
		}
	}
	up_read(&leds_list_lock);

	return led;
}


static DWORD deinitialize_hw(struct device *dev)
{
	struct vcam_data *data = dev_get_drvdata(dev);

	ov5640_remove_sysfs_attributes(dev);
	vcam_eoco_remove_sysfs_attributes(dev);
		
	data->ops.set_power(dev, false);
	
	i2c_put_adapter(data->i2c_bus);
	return 0;
}
