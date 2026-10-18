// SPDX-License-Identifier: GPL-2.0
/*
 * Power key of the Samsung S2MPU06 PMIC
 *
 * The PMIC reports the falling (pressed) and rising (released) edge of the key as
 * two interrupts.
 */

#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>

static irqreturn_t s2mpu06_pwrkey_press(int irq, void *data)
{
	struct input_dev *input = data;

	input_report_key(input, KEY_POWER, 1);
	input_sync(input);

	return IRQ_HANDLED;
}

static irqreturn_t s2mpu06_pwrkey_release(int irq, void *data)
{
	struct input_dev *input = data;

	input_report_key(input, KEY_POWER, 0);
	input_sync(input);

	return IRQ_HANDLED;
}

static int s2mpu06_pwrkey_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct input_dev *input;
	int irq_f, irq_r, ret;

	irq_f = platform_get_irq_byname(pdev, "pwronf");
	if (irq_f < 0)
		return irq_f;
	irq_r = platform_get_irq_byname(pdev, "pwronr");
	if (irq_r < 0)
		return irq_r;

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	input->name = "s2mpu06-pwrkey";
	input->phys = "s2mpu06-pwrkey/input0";
	input_set_capability(input, EV_KEY, KEY_POWER);

	ret = devm_request_threaded_irq(dev, irq_f, NULL, s2mpu06_pwrkey_press,
					IRQF_ONESHOT, "pwronf", input);
	if (ret)
		return dev_err_probe(dev, ret, "cannot request the press irq\n");

	ret = devm_request_threaded_irq(dev, irq_r, NULL, s2mpu06_pwrkey_release,
					IRQF_ONESHOT, "pwronr", input);
	if (ret)
		return dev_err_probe(dev, ret, "cannot request the release irq\n");

	ret = input_register_device(input);
	if (ret)
		return dev_err_probe(dev, ret, "cannot register the input device\n");

	device_init_wakeup(dev, true);
	return 0;
}

static const struct platform_device_id s2mpu06_pwrkey_id[] = {
	{ "s2mpu06-pwrkey" },
	{ }
};
MODULE_DEVICE_TABLE(platform, s2mpu06_pwrkey_id);

static struct platform_driver s2mpu06_pwrkey_driver = {
	.probe = s2mpu06_pwrkey_probe,
	.driver = {
		.name = "s2mpu06-pwrkey",
	},
	.id_table = s2mpu06_pwrkey_id,
};
module_platform_driver(s2mpu06_pwrkey_driver);

MODULE_DESCRIPTION("Samsung S2MPU06 PMIC power key driver");
MODULE_LICENSE("GPL");
