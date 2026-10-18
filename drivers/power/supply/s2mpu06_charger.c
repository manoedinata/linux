// SPDX-License-Identifier: GPL-2.0
/*
 * Charger of the Samsung S2MPU06 PMIC
 *
 * The charger IC runs the charging cycle on its own; this driver reports its
 * state, exposes the current and voltage limits and keeps its watchdog fed.
 * Register layout from the downstream s2mpu06_charger driver:
 * Copyright (C) 2015 Samsung Electronics
 */

#include <linux/bits.h>
#include <linux/interrupt.h>
#include <linux/mfd/samsung/s2mpu06.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

/* PMIC block */
#define S2MPU06_PMIC_STATUS1		0x06
#define S2MPU06_PMIC_STATUS1_ACOK	BIT(2)

/* charger block */
#define S2MPU06_CHG_STATUS1		0x09
#define S2MPU06_CHG_STATUS1_CHGVIN	BIT(2)
#define S2MPU06_CHG_STATUS1_CHARGING	BIT(5)
#define S2MPU06_CHG_STATUS1_CC_CV	GENMASK(6, 5)
#define S2MPU06_CHG_STATUS2		0x0a
#define S2MPU06_CHG_STATUS2_DONE	BIT(5)
#define S2MPU06_CHG_STATUS2_VINOVP	BIT(6)
#define S2MPU06_CHG_CTRL1		0x0d	/* input current limit, 10 mA/step */
#define S2MPU06_CHG_CTRL2		0x0e	/* float voltage */
#define S2MPU06_CHG_CTRL10		0x16	/* fast charge current */
#define S2MPU06_CHG_CTRL10_MASK		GENMASK(6, 0)
#define S2MPU06_CHG_CTRL12		0x18	/* watchdog */
#define S2MPU06_CHG_CTRL12_WDT_EN	GENMASK(3, 2)
#define S2MPU06_CHG_CTRL12_WDT_KICK	GENMASK(1, 0)

#define S2MPU06_ICHG_MIN_UA		240000
#define S2MPU06_ICHG_STEP_UA		20000
#define S2MPU06_ICHG_BASE_SEL		0x08
#define S2MPU06_ICHG_MAX_SEL		0x60

#define S2MPU06_IIN_STEP_UA		10000
#define S2MPU06_IIN_MIN_SEL		0x0a
#define S2MPU06_IIN_MAX_SEL		0xb4

#define S2MPU06_VFLOAT_BASE_SEL		0x60
#define S2MPU06_VFLOAT_MAX_SEL		0xb5
#define S2MPU06_VFLOAT_MIN_UV		3600000
#define S2MPU06_VFLOAT_STEP_UV		10000

#define S2MPU06_WDT_KICK_PERIOD		msecs_to_jiffies(10000)

struct s2mpu06_charger {
	struct device *dev;
	struct s2mpu06 *s2m;
	struct power_supply *psy;
	struct delayed_work wdt_work;
};

static int s2mpu06_charger_online(struct s2mpu06_charger *chg, int *online)
{
	unsigned int val;
	int ret;

	ret = regmap_read(chg->s2m->pmic, S2MPU06_PMIC_STATUS1, &val);
	if (ret)
		return ret;

	*online = !!(val & S2MPU06_PMIC_STATUS1_ACOK);
	return 0;
}

static int s2mpu06_charger_status(struct s2mpu06_charger *chg, int *status)
{
	unsigned int s1, s2;
	int ret, online;

	ret = s2mpu06_charger_online(chg, &online);
	if (ret)
		return ret;
	ret = regmap_read(chg->s2m->chg, S2MPU06_CHG_STATUS1, &s1);
	if (ret)
		return ret;
	ret = regmap_read(chg->s2m->chg, S2MPU06_CHG_STATUS2, &s2);
	if (ret)
		return ret;

	if (!online)
		*status = POWER_SUPPLY_STATUS_DISCHARGING;
	else if (s2 & S2MPU06_CHG_STATUS2_DONE)
		*status = POWER_SUPPLY_STATUS_FULL;
	else if (s1 & S2MPU06_CHG_STATUS1_CHARGING)
		*status = POWER_SUPPLY_STATUS_CHARGING;
	else
		*status = POWER_SUPPLY_STATUS_NOT_CHARGING;

	return 0;
}

static int s2mpu06_charger_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct s2mpu06_charger *chg = power_supply_get_drvdata(psy);
	unsigned int reg;
	int ret, online;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = s2mpu06_charger_online(chg, &online);
		val->intval = online;
		return ret;
	case POWER_SUPPLY_PROP_STATUS:
		return s2mpu06_charger_status(chg, &val->intval);
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		ret = regmap_read(chg->s2m->chg, S2MPU06_CHG_STATUS1, &reg);
		if (ret)
			return ret;
		reg &= S2MPU06_CHG_STATUS1_CC_CV;
		if (!(reg & S2MPU06_CHG_STATUS1_CHARGING))
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
		else if (reg == S2MPU06_CHG_STATUS1_CC_CV)
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		else
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		return 0;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = regmap_read(chg->s2m->chg, S2MPU06_CHG_STATUS2, &reg);
		if (ret)
			return ret;
		val->intval = (reg & S2MPU06_CHG_STATUS2_VINOVP) ?
			POWER_SUPPLY_HEALTH_OVERVOLTAGE : POWER_SUPPLY_HEALTH_GOOD;
		return 0;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = regmap_read(chg->s2m->chg, S2MPU06_CHG_CTRL10, &reg);
		if (ret)
			return ret;
		reg = clamp(reg & S2MPU06_CHG_CTRL10_MASK, (unsigned int)S2MPU06_ICHG_BASE_SEL,
			    (unsigned int)S2MPU06_ICHG_MAX_SEL);
		val->intval = S2MPU06_ICHG_MIN_UA +
			      (reg - S2MPU06_ICHG_BASE_SEL) * S2MPU06_ICHG_STEP_UA;
		return 0;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = S2MPU06_ICHG_MIN_UA +
			      (S2MPU06_ICHG_MAX_SEL - S2MPU06_ICHG_BASE_SEL) * S2MPU06_ICHG_STEP_UA;
		return 0;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = regmap_read(chg->s2m->chg, S2MPU06_CHG_CTRL1, &reg);
		if (ret)
			return ret;
		val->intval = min_t(unsigned int, reg, S2MPU06_IIN_MAX_SEL) * S2MPU06_IIN_STEP_UA;
		return 0;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = regmap_read(chg->s2m->chg, S2MPU06_CHG_CTRL2, &reg);
		if (ret)
			return ret;
		reg = clamp(reg, (unsigned int)S2MPU06_VFLOAT_BASE_SEL,
			    (unsigned int)S2MPU06_VFLOAT_MAX_SEL);
		val->intval = S2MPU06_VFLOAT_MIN_UV +
			      (reg - S2MPU06_VFLOAT_BASE_SEL) * S2MPU06_VFLOAT_STEP_UV;
		return 0;
	default:
		return -EINVAL;
	}
}

static int s2mpu06_charger_set_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	struct s2mpu06_charger *chg = power_supply_get_drvdata(psy);
	unsigned int sel;

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		sel = S2MPU06_ICHG_BASE_SEL +
		      (clamp(val->intval, S2MPU06_ICHG_MIN_UA,
			     S2MPU06_ICHG_MIN_UA + (S2MPU06_ICHG_MAX_SEL - S2MPU06_ICHG_BASE_SEL) *
						   S2MPU06_ICHG_STEP_UA) -
		       S2MPU06_ICHG_MIN_UA) / S2MPU06_ICHG_STEP_UA;
		return regmap_update_bits(chg->s2m->chg, S2MPU06_CHG_CTRL10,
					  S2MPU06_CHG_CTRL10_MASK, sel);
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		sel = clamp(val->intval / S2MPU06_IIN_STEP_UA, S2MPU06_IIN_MIN_SEL,
			    S2MPU06_IIN_MAX_SEL);
		return regmap_write(chg->s2m->chg, S2MPU06_CHG_CTRL1, sel);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		sel = S2MPU06_VFLOAT_BASE_SEL +
		      (clamp(val->intval, S2MPU06_VFLOAT_MIN_UV,
			     S2MPU06_VFLOAT_MIN_UV + (S2MPU06_VFLOAT_MAX_SEL - S2MPU06_VFLOAT_BASE_SEL) *
						     S2MPU06_VFLOAT_STEP_UV) -
		       S2MPU06_VFLOAT_MIN_UV) / S2MPU06_VFLOAT_STEP_UV;
		return regmap_write(chg->s2m->chg, S2MPU06_CHG_CTRL2, sel);
	default:
		return -EINVAL;
	}
}

static int s2mpu06_charger_property_is_writeable(struct power_supply *psy,
						 enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		return 1;
	default:
		return 0;
	}
}

static const enum power_supply_property s2mpu06_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
};

static char *s2mpu06_charger_supplicants[] = { "s2mpu06-fuelgauge" };

static const struct power_supply_desc s2mpu06_charger_desc = {
	.name = "s2mpu06-charger",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = s2mpu06_charger_props,
	.num_properties = ARRAY_SIZE(s2mpu06_charger_props),
	.get_property = s2mpu06_charger_get_property,
	.set_property = s2mpu06_charger_set_property,
	.property_is_writeable = s2mpu06_charger_property_is_writeable,
};

/* the charger IC drops back to its defaults when its watchdog is not cleared */
static void s2mpu06_charger_wdt_work(struct work_struct *work)
{
	struct s2mpu06_charger *chg = container_of(work, struct s2mpu06_charger, wdt_work.work);

	regmap_update_bits(chg->s2m->chg, S2MPU06_CHG_CTRL12, S2MPU06_CHG_CTRL12_WDT_KICK, 0x1);
	regmap_update_bits(chg->s2m->chg, S2MPU06_CHG_CTRL12, S2MPU06_CHG_CTRL12_WDT_KICK, 0x0);

	schedule_delayed_work(&chg->wdt_work, S2MPU06_WDT_KICK_PERIOD);
}

static irqreturn_t s2mpu06_charger_acok_irq(int irq, void *data)
{
	struct s2mpu06_charger *chg = data;

	power_supply_changed(chg->psy);

	return IRQ_HANDLED;
}

static void s2mpu06_charger_cancel_wdt(void *data)
{
	cancel_delayed_work_sync(data);
}

static int s2mpu06_charger_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct power_supply_config cfg = { };
	struct s2mpu06_charger *chg;
	unsigned int ctrl12;
	int i, irq, ret;

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->dev = dev;
	chg->s2m = dev_get_drvdata(dev->parent);
	INIT_DELAYED_WORK(&chg->wdt_work, s2mpu06_charger_wdt_work);

	cfg.drv_data = chg;
	cfg.supplied_to = s2mpu06_charger_supplicants;
	cfg.num_supplicants = ARRAY_SIZE(s2mpu06_charger_supplicants);

	chg->psy = devm_power_supply_register(dev, &s2mpu06_charger_desc, &cfg);
	if (IS_ERR(chg->psy))
		return dev_err_probe(dev, PTR_ERR(chg->psy), "cannot register the charger\n");

	for (i = S2MPU06_IRQ_ACOKBF; i <= S2MPU06_IRQ_ACOKBR; i++) {
		irq = regmap_irq_get_virq(chg->s2m->irq_data, i);
		if (irq < 0)
			return irq;

		ret = devm_request_threaded_irq(dev, irq, NULL, s2mpu06_charger_acok_irq,
						IRQF_ONESHOT, "s2mpu06-acok", chg);
		if (ret)
			return dev_err_probe(dev, ret, "cannot request the AC OK irq\n");
	}

	/* only feed the watchdog if the bootloader or a previous kernel armed it */
	ret = regmap_read(chg->s2m->chg, S2MPU06_CHG_CTRL12, &ctrl12);
	if (ret)
		return dev_err_probe(dev, ret, "cannot read the charger\n");
	dev_info(dev, "charger CTRL12 %#x (watchdog %s)\n", ctrl12,
		 (ctrl12 & S2MPU06_CHG_CTRL12_WDT_EN) ? "armed" : "off");

	if (ctrl12 & S2MPU06_CHG_CTRL12_WDT_EN) {
		ret = devm_add_action_or_reset(dev, s2mpu06_charger_cancel_wdt, &chg->wdt_work);
		if (ret)
			return ret;
		schedule_delayed_work(&chg->wdt_work, 0);
	}

	return 0;
}

static const struct platform_device_id s2mpu06_charger_id[] = {
	{ "s2mpu06-charger" },
	{ }
};
MODULE_DEVICE_TABLE(platform, s2mpu06_charger_id);

static struct platform_driver s2mpu06_charger_driver = {
	.probe = s2mpu06_charger_probe,
	.driver = {
		.name = "s2mpu06-charger",
	},
	.id_table = s2mpu06_charger_id,
};
module_platform_driver(s2mpu06_charger_driver);

MODULE_DESCRIPTION("Samsung S2MPU06 charger driver");
MODULE_LICENSE("GPL");
