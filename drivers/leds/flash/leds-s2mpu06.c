// SPDX-License-Identifier: GPL-2.0
/*
 * Flash LED block of the Samsung S2MPU06 PMIC (Galaxy J4 rear camera flash)
 *
 * The register layout comes from Samsung's downstream fimc-is-flash-i2c
 * driver (the only user of this block there):
 *
 *   FLED_CTRL0 0x03: bit 1 AUTO_BST_EN, bit 2 CHGVIN_EN, bit 5 TORCH_EN
 *   FLED_CTRL1 0x04: [7:4] flash current, 100 mA steps from 100 mA
 *                    [3:0] torch current, 20 mA steps from 20 mA
 *   FLED_CTRL3 0x06: written 0xff before every flash (meaning undocumented)
 *   FLED_CTRL4 0x07: bit 7 TORCH_TMR_MODE, [3:0] timeout
 *
 * Torch mode is switched with TORCH_EN. The flash is fired by a GPIO from the
 * AP (gpa0-0 on the J4) at the flash current; it is turned off again by a
 * software timer here.
 *
 * Without TORCH_TMR_MODE and a timeout the torch goes out after well under a
 * second (seen on the J4), so both are set at probe, as downstream does.
 *
 * Downstream picks the LED input after the cable: with a charger or USB host
 * attached CHGVIN_EN is set and AUTO_BST_EN cleared, otherwise the reverse.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/extcon.h>
#include <linux/gpio/consumer.h>
#include <linux/led-class-flash.h>
#include <linux/mfd/samsung/s2mpu06.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#define S2MPU06_FLED_CTRL0		0x03
#define S2MPU06_FLED_CTRL0_AUTO_BST_EN	BIT(1)
#define S2MPU06_FLED_CTRL0_CHGVIN_EN	BIT(2)
#define S2MPU06_FLED_CTRL0_TORCH_EN	BIT(5)

#define S2MPU06_FLED_CTRL1		0x04
#define S2MPU06_FLED_CTRL1_FLASH_ISET	GENMASK(7, 4)
#define S2MPU06_FLED_CTRL1_TORCH_ISET	GENMASK(3, 0)

#define S2MPU06_FLED_CTRL3		0x06
#define S2MPU06_FLED_CTRL4		0x07
#define S2MPU06_FLED_CTRL4_TORCH_TMR	BIT(7)
#define S2MPU06_FLED_CTRL4_TIMEOUT	GENMASK(3, 0)

#define S2MPU06_TORCH_STEP_UA		20000
#define S2MPU06_TORCH_MAX_UA		300000
#define S2MPU06_FLASH_STEP_UA		100000
#define S2MPU06_FLASH_MAX_UA		1500000

#define S2MPU06_TIMEOUT_MIN_US		50000
#define S2MPU06_TIMEOUT_STEP_US		50000

static const unsigned int s2mpu06_fled_cables[] = {
	EXTCON_USB,
	EXTCON_CHG_USB_SDP,
	EXTCON_CHG_USB_DCP,
	EXTCON_CHG_USB_CDP,
	EXTCON_CHG_USB_FAST,
	EXTCON_CHG_USB_SLOW,
};

struct s2mpu06_fled {
	struct device *dev;
	struct regmap *map;
	struct gpio_desc *flash_gpio;
	struct led_classdev_flash fled_cdev;
	struct mutex lock;		/* registers, strobe state, cable state */

	struct delayed_work strobe_off;
	bool strobing;

	struct extcon_dev *edev;
	struct notifier_block cable_nb;
	struct work_struct cable_work;
	bool cable;
};

static struct s2mpu06_fled *fled_from_cdev(struct led_classdev_flash *fled_cdev)
{
	return container_of(fled_cdev, struct s2mpu06_fled, fled_cdev);
}

/* caller holds the lock */
static int s2mpu06_fled_set_input(struct s2mpu06_fled *f)
{
	unsigned int val = f->cable ? S2MPU06_FLED_CTRL0_CHGVIN_EN :
				      S2MPU06_FLED_CTRL0_AUTO_BST_EN;

	return regmap_update_bits(f->map, S2MPU06_FLED_CTRL0,
				  S2MPU06_FLED_CTRL0_CHGVIN_EN |
				  S2MPU06_FLED_CTRL0_AUTO_BST_EN, val);
}

static int s2mpu06_fled_brightness_set(struct led_classdev *led_cdev,
				       enum led_brightness brightness)
{
	struct s2mpu06_fled *f = fled_from_cdev(lcdev_to_flcdev(led_cdev));
	int ret;

	mutex_lock(&f->lock);
	if (!brightness) {
		ret = regmap_update_bits(f->map, S2MPU06_FLED_CTRL0,
					 S2MPU06_FLED_CTRL0_TORCH_EN, 0);
		goto out;
	}

	ret = s2mpu06_fled_set_input(f);
	if (ret)
		goto out;
	ret = regmap_update_bits(f->map, S2MPU06_FLED_CTRL1,
				 S2MPU06_FLED_CTRL1_TORCH_ISET,
				 FIELD_PREP(S2MPU06_FLED_CTRL1_TORCH_ISET, brightness - 1));
	if (ret)
		goto out;
	ret = regmap_update_bits(f->map, S2MPU06_FLED_CTRL0,
				 S2MPU06_FLED_CTRL0_TORCH_EN,
				 S2MPU06_FLED_CTRL0_TORCH_EN);
out:
	mutex_unlock(&f->lock);
	return ret;
}

static int s2mpu06_fled_flash_brightness_set(struct led_classdev_flash *fled_cdev,
					     u32 brightness)
{
	struct s2mpu06_fled *f = fled_from_cdev(fled_cdev);
	unsigned int code = brightness / S2MPU06_FLASH_STEP_UA - 1;
	int ret;

	mutex_lock(&f->lock);
	ret = regmap_update_bits(f->map, S2MPU06_FLED_CTRL1,
				 S2MPU06_FLED_CTRL1_FLASH_ISET,
				 FIELD_PREP(S2MPU06_FLED_CTRL1_FLASH_ISET, code));
	mutex_unlock(&f->lock);
	return ret;
}

/* caller holds the lock */
static void s2mpu06_fled_flash_off(struct s2mpu06_fled *f)
{
	gpiod_set_value_cansleep(f->flash_gpio, 0);
	f->strobing = false;
}

static void s2mpu06_fled_strobe_off_work(struct work_struct *work)
{
	struct s2mpu06_fled *f = container_of(to_delayed_work(work),
					      struct s2mpu06_fled, strobe_off);

	mutex_lock(&f->lock);
	s2mpu06_fled_flash_off(f);
	mutex_unlock(&f->lock);
}

static int s2mpu06_fled_strobe_set(struct led_classdev_flash *fled_cdev, bool state)
{
	struct s2mpu06_fled *f = fled_from_cdev(fled_cdev);
	int ret = 0;

	cancel_delayed_work_sync(&f->strobe_off);

	mutex_lock(&f->lock);
	if (!state) {
		s2mpu06_fled_flash_off(f);
		goto out;
	}

	ret = s2mpu06_fled_set_input(f);
	if (ret)
		goto out;
	/* downstream writes this before every flash */
	ret = regmap_write(f->map, S2MPU06_FLED_CTRL3, 0xff);
	if (ret)
		goto out;

	gpiod_set_value_cansleep(f->flash_gpio, 1);
	f->strobing = true;
	schedule_delayed_work(&f->strobe_off, usecs_to_jiffies(fled_cdev->timeout.val));
out:
	mutex_unlock(&f->lock);
	return ret;
}

static int s2mpu06_fled_strobe_get(struct led_classdev_flash *fled_cdev, bool *state)
{
	struct s2mpu06_fled *f = fled_from_cdev(fled_cdev);

	mutex_lock(&f->lock);
	*state = f->strobing;
	mutex_unlock(&f->lock);
	return 0;
}

static int s2mpu06_fled_timeout_set(struct led_classdev_flash *fled_cdev, u32 timeout)
{
	/* only used by the software timer when the next strobe starts */
	return 0;
}

static const struct led_flash_ops s2mpu06_fled_ops = {
	.flash_brightness_set = s2mpu06_fled_flash_brightness_set,
	.strobe_set = s2mpu06_fled_strobe_set,
	.strobe_get = s2mpu06_fled_strobe_get,
	.timeout_set = s2mpu06_fled_timeout_set,
};

static bool s2mpu06_fled_cable_present(struct s2mpu06_fled *f)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(s2mpu06_fled_cables); i++)
		if (extcon_get_state(f->edev, s2mpu06_fled_cables[i]) > 0)
			return true;
	return false;
}

static void s2mpu06_fled_cable_work(struct work_struct *work)
{
	struct s2mpu06_fled *f = container_of(work, struct s2mpu06_fled, cable_work);
	bool cable = s2mpu06_fled_cable_present(f);
	unsigned int ctrl0;

	mutex_lock(&f->lock);
	if (cable == f->cable)
		goto out;
	f->cable = cable;
	s2mpu06_fled_set_input(f);

	/* downstream restarts a lit torch when the cable goes away */
	if (!cable && !regmap_read(f->map, S2MPU06_FLED_CTRL0, &ctrl0) &&
	    (ctrl0 & S2MPU06_FLED_CTRL0_TORCH_EN)) {
		regmap_update_bits(f->map, S2MPU06_FLED_CTRL0,
				   S2MPU06_FLED_CTRL0_TORCH_EN, 0);
		regmap_update_bits(f->map, S2MPU06_FLED_CTRL0,
				   S2MPU06_FLED_CTRL0_TORCH_EN,
				   S2MPU06_FLED_CTRL0_TORCH_EN);
	}
out:
	mutex_unlock(&f->lock);
}

static int s2mpu06_fled_cable_event(struct notifier_block *nb,
				    unsigned long event, void *ptr)
{
	struct s2mpu06_fled *f = container_of(nb, struct s2mpu06_fled, cable_nb);

	schedule_work(&f->cable_work);
	return NOTIFY_OK;
}

static void s2mpu06_fled_shutdown_hw(void *data)
{
	struct s2mpu06_fled *f = data;

	cancel_delayed_work_sync(&f->strobe_off);
	cancel_work_sync(&f->cable_work);
	gpiod_set_value_cansleep(f->flash_gpio, 0);
	regmap_update_bits(f->map, S2MPU06_FLED_CTRL0, S2MPU06_FLED_CTRL0_TORCH_EN, 0);
}

static int s2mpu06_fled_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s2mpu06 *s2m = dev_get_drvdata(dev->parent);
	struct led_init_data init_data = {};
	struct led_classdev_flash *fled_cdev;
	struct led_classdev *led_cdev;
	struct led_flash_setting *s;
	struct s2mpu06_fled *f;
	u32 torch_max, flash_max, timeout_max;
	int ret;

	if (!s2m || !s2m->fled)
		return -ENODEV;

	f = devm_kzalloc(dev, sizeof(*f), GFP_KERNEL);
	if (!f)
		return -ENOMEM;
	f->dev = dev;
	f->map = s2m->fled;
	mutex_init(&f->lock);
	INIT_DELAYED_WORK(&f->strobe_off, s2mpu06_fled_strobe_off_work);
	INIT_WORK(&f->cable_work, s2mpu06_fled_cable_work);

	f->flash_gpio = devm_gpiod_get(dev, "flash", GPIOD_OUT_LOW);
	if (IS_ERR(f->flash_gpio))
		return dev_err_probe(dev, PTR_ERR(f->flash_gpio), "cannot get the flash gpio\n");

	if (device_property_read_u32(dev, "led-max-microamp", &torch_max))
		torch_max = 160000;
	torch_max = clamp(torch_max, S2MPU06_TORCH_STEP_UA, S2MPU06_TORCH_MAX_UA);
	if (device_property_read_u32(dev, "flash-max-microamp", &flash_max))
		flash_max = 600000;
	flash_max = clamp(flash_max, S2MPU06_FLASH_STEP_UA, S2MPU06_FLASH_MAX_UA);
	if (device_property_read_u32(dev, "flash-max-timeout-us", &timeout_max))
		timeout_max = 1000000;
	timeout_max = max(timeout_max, S2MPU06_TIMEOUT_MIN_US);

	/* the cable state decides the LED input */
	f->edev = extcon_get_edev_by_phandle(dev->parent, 0);
	if (IS_ERR(f->edev)) {
		ret = PTR_ERR(f->edev);
		f->edev = NULL;
		if (ret == -EPROBE_DEFER)
			return ret;
	}

	/* everything off, torch timer as downstream (else the torch goes out) */
	gpiod_set_value_cansleep(f->flash_gpio, 0);
	ret = regmap_update_bits(f->map, S2MPU06_FLED_CTRL0, S2MPU06_FLED_CTRL0_TORCH_EN, 0);
	if (!ret)
		ret = regmap_update_bits(f->map, S2MPU06_FLED_CTRL4,
					 S2MPU06_FLED_CTRL4_TORCH_TMR |
					 S2MPU06_FLED_CTRL4_TIMEOUT,
					 S2MPU06_FLED_CTRL4_TORCH_TMR |
					 S2MPU06_FLED_CTRL4_TIMEOUT);
	if (!ret)
		ret = regmap_update_bits(f->map, S2MPU06_FLED_CTRL1,
					 S2MPU06_FLED_CTRL1_FLASH_ISET,
					 FIELD_PREP(S2MPU06_FLED_CTRL1_FLASH_ISET,
						    flash_max / S2MPU06_FLASH_STEP_UA - 1));
	if (ret)
		return dev_err_probe(dev, ret, "cannot set up the flash block\n");

	f->cable = f->edev && s2mpu06_fled_cable_present(f);
	s2mpu06_fled_set_input(f);

	ret = devm_add_action_or_reset(dev, s2mpu06_fled_shutdown_hw, f);
	if (ret)
		return ret;

	fled_cdev = &f->fled_cdev;
	fled_cdev->ops = &s2mpu06_fled_ops;

	s = &fled_cdev->brightness;
	s->min = S2MPU06_FLASH_STEP_UA;
	s->max = flash_max - flash_max % S2MPU06_FLASH_STEP_UA;
	s->step = S2MPU06_FLASH_STEP_UA;
	s->val = s->max;

	s = &fled_cdev->timeout;
	s->min = S2MPU06_TIMEOUT_MIN_US;
	s->max = timeout_max;
	s->step = S2MPU06_TIMEOUT_STEP_US;
	s->val = min(500000U, timeout_max);

	led_cdev = &fled_cdev->led_cdev;
	led_cdev->max_brightness = torch_max / S2MPU06_TORCH_STEP_UA;
	led_cdev->brightness_set_blocking = s2mpu06_fled_brightness_set;
	led_cdev->flags |= LED_CORE_SUSPENDRESUME | LED_DEV_CAP_FLASH;

	init_data.fwnode = dev_fwnode(dev);
	init_data.default_label = "flash";

	ret = devm_led_classdev_flash_register_ext(dev, fled_cdev, &init_data);
	if (ret)
		return dev_err_probe(dev, ret, "cannot register the flash LED\n");

	if (f->edev) {
		/* one block can sit on one chain only: watch every cable at once */
		f->cable_nb.notifier_call = s2mpu06_fled_cable_event;
		ret = devm_extcon_register_notifier_all(dev, f->edev, &f->cable_nb);
		if (ret)
			return dev_err_probe(dev, ret, "cannot watch the cable\n");
		schedule_work(&f->cable_work);
	}

	dev_info(dev, "torch up to %u uA, flash %u uA, cable %s\n",
		 torch_max - torch_max % S2MPU06_TORCH_STEP_UA,
		 fled_cdev->brightness.max, f->cable ? "present" : "absent");
	return 0;
}

static const struct of_device_id s2mpu06_fled_of_match[] = {
	{ .compatible = "samsung,s2mpu06-fled" },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mpu06_fled_of_match);

static struct platform_driver s2mpu06_fled_driver = {
	.driver = {
		.name = "s2mpu06-fled",
		.of_match_table = s2mpu06_fled_of_match,
	},
	.probe = s2mpu06_fled_probe,
};
module_platform_driver(s2mpu06_fled_driver);

MODULE_DESCRIPTION("Samsung S2MPU06 flash LED driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:s2mpu06-fled");
