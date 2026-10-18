// SPDX-License-Identifier: GPL-2.0
/*
 * Regulators of the Samsung S2MPU06 PMIC (3 bucks, 20 LDOs) on a SPEEDY bus.
 *
 * Register layout from the downstream s2mpu06 regulator driver:
 * Copyright (c) 2013 Samsung Electronics Co., Ltd
 */

#include <linux/bits.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/soc/samsung/exynos-speedy.h>

/* SPEEDY block ids */
#define S2MPU06_BLK_TOP			0
#define S2MPU06_BLK_PMIC		1

#define S2MPU06_TOP_PMICID		0x00

#define S2MPU06_REG_B1CTRL		0x13
#define S2MPU06_REG_B1OUT2		0x15
#define S2MPU06_REG_B2CTRL1		0x17
#define S2MPU06_REG_B2CTRL2		0x18
#define S2MPU06_REG_B3CTRL1		0x19
#define S2MPU06_REG_B3CTRL2		0x1a
#define S2MPU06_REG_EXT_CTRL		0xff

/* bits 7:6 of every LDO/BUCK enable register select the power mode, 0 is off */
#define S2MPU06_ENABLE_MASK		GENMASK(7, 6)
#define S2MPU06_ENABLE_ON		(0x3 << 6)
#define S2MPU06_LDO_VSEL_MASK		0x3f
#define S2MPU06_BUCK_VSEL_MASK		0xff

/* LDO4 and LDO14..16 are switched through EXT_CTRL instead */
#define S2MPU06_EXT_LDO4		BIT(0)
#define S2MPU06_EXT_LDO14		BIT(1)
#define S2MPU06_EXT_LDO15		BIT(2)
#define S2MPU06_EXT_LDO16		BIT(3)

#define S2MPU06_LDO_ENABLE_TIME_US	128

static int s2mpu06_ldo_is_enabled(struct regulator_dev *rdev)
{
	unsigned int val;
	int ret;

	ret = regmap_read(rdev->regmap, rdev->desc->enable_reg, &val);
	if (ret)
		return ret;

	/* any power mode but "off" counts as enabled */
	return !!(val & rdev->desc->enable_mask);
}

static const struct regulator_ops s2mpu06_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.is_enabled		= s2mpu06_ldo_is_enabled,
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
};

/* the external-control switches are plain bits, so the stock helper does it */
static const struct regulator_ops s2mpu06_ext_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.is_enabled		= regulator_is_enabled_regmap,
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
};

#define S2MPU06_LDO(_name, _min, _step, _reg)				\
	{								\
		.name		= _name,				\
		.of_match	= of_match_ptr(_name),			\
		.regulators_node = of_match_ptr("regulators"),		\
		.type		= REGULATOR_VOLTAGE,			\
		.owner		= THIS_MODULE,				\
		.ops		= &s2mpu06_ops,				\
		.min_uV		= _min,					\
		.uV_step	= _step,				\
		.n_voltages	= S2MPU06_LDO_VSEL_MASK + 1,		\
		.vsel_reg	= _reg,					\
		.vsel_mask	= S2MPU06_LDO_VSEL_MASK,		\
		.enable_reg	= _reg,					\
		.enable_mask	= S2MPU06_ENABLE_MASK,			\
		.enable_val	= S2MPU06_ENABLE_ON,			\
		.enable_time	= S2MPU06_LDO_ENABLE_TIME_US,		\
	}

#define S2MPU06_LDO_EXT(_name, _min, _step, _reg, _bit)			\
	{								\
		.name		= _name,				\
		.of_match	= of_match_ptr(_name),			\
		.regulators_node = of_match_ptr("regulators"),		\
		.type		= REGULATOR_VOLTAGE,			\
		.owner		= THIS_MODULE,				\
		.ops		= &s2mpu06_ext_ops,			\
		.min_uV		= _min,					\
		.uV_step	= _step,				\
		.n_voltages	= S2MPU06_LDO_VSEL_MASK + 1,		\
		.vsel_reg	= _reg,					\
		.vsel_mask	= S2MPU06_LDO_VSEL_MASK,		\
		.enable_reg	= S2MPU06_REG_EXT_CTRL,			\
		.enable_mask	= _bit,					\
		.enable_val	= _bit,					\
		.enable_time	= S2MPU06_LDO_ENABLE_TIME_US,		\
	}

#define S2MPU06_BUCK(_name, _min, _step, _vsel, _en, _time)		\
	{								\
		.name		= _name,				\
		.of_match	= of_match_ptr(_name),			\
		.regulators_node = of_match_ptr("regulators"),		\
		.type		= REGULATOR_VOLTAGE,			\
		.owner		= THIS_MODULE,				\
		.ops		= &s2mpu06_ops,				\
		.min_uV		= _min,					\
		.uV_step	= _step,				\
		.n_voltages	= S2MPU06_BUCK_VSEL_MASK + 1,		\
		.vsel_reg	= _vsel,				\
		.vsel_mask	= S2MPU06_BUCK_VSEL_MASK,		\
		.enable_reg	= _en,					\
		.enable_mask	= S2MPU06_ENABLE_MASK,			\
		.enable_val	= S2MPU06_ENABLE_ON,			\
		.enable_time	= _time,				\
	}

static const struct regulator_desc s2mpu06_regulators[] = {
	S2MPU06_LDO("LDO1",  400000, 25000, 0x1e),
	S2MPU06_LDO("LDO2",  800000, 25000, 0x20),
	S2MPU06_LDO("LDO3",  800000, 12500, 0x21),
	S2MPU06_LDO_EXT("LDO4", 1800000, 25000, 0x22, S2MPU06_EXT_LDO4),
	S2MPU06_LDO("LDO5",  1800000, 25000, 0x23),
	S2MPU06_LDO("LDO6",  400000, 25000, 0x24),
	S2MPU06_LDO("LDO7",  800000, 12500, 0x26),
	S2MPU06_LDO("LDO8",  1800000, 25000, 0x27),
	S2MPU06_LDO("LDO13", 800000, 25000, 0x2c),
	S2MPU06_LDO_EXT("LDO14", 1800000, 25000, 0x2d, S2MPU06_EXT_LDO14),
	S2MPU06_LDO_EXT("LDO15", 800000, 25000, 0x2e, S2MPU06_EXT_LDO15),
	S2MPU06_LDO_EXT("LDO16", 1800000, 25000, 0x2f, S2MPU06_EXT_LDO16),
	S2MPU06_LDO("LDO17", 800000, 25000, 0x30),
	S2MPU06_LDO("LDO18", 1800000, 25000, 0x31),
	S2MPU06_LDO("LDO19", 1800000, 25000, 0x32),
	S2MPU06_LDO("LDO20", 800000, 25000, 0x33),
	S2MPU06_LDO("LDO21", 1800000, 25000, 0x34),
	S2MPU06_LDO("LDO22", 1800000, 25000, 0x35),
	S2MPU06_LDO("LDO23", 800000, 12500, 0x36),
	S2MPU06_LDO("LDO24", 1800000, 25000, 0x37),
	S2MPU06_BUCK("BUCK1", 400000, 6250, S2MPU06_REG_B1OUT2, S2MPU06_REG_B1CTRL, 95),
	S2MPU06_BUCK("BUCK2", 800000, 25000, S2MPU06_REG_B2CTRL2, S2MPU06_REG_B2CTRL1, 106),
	S2MPU06_BUCK("BUCK3", 800000, 25000, S2MPU06_REG_B3CTRL2, S2MPU06_REG_B3CTRL1, 150),
};

static int s2mpu06_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regulator_config config = { };
	struct device_node *regs_np;
	struct regmap *top, *pmic;
	unsigned int id;
	int i, ret;

	top = devm_exynos_speedy_regmap(dev, S2MPU06_BLK_TOP);
	if (IS_ERR(top))
		return dev_err_probe(dev, PTR_ERR(top), "cannot get TOP regmap\n");

	pmic = devm_exynos_speedy_regmap(dev, S2MPU06_BLK_PMIC);
	if (IS_ERR(pmic))
		return dev_err_probe(dev, PTR_ERR(pmic), "cannot get PMIC regmap\n");

	ret = regmap_read(top, S2MPU06_TOP_PMICID, &id);
	if (ret)
		return dev_err_probe(dev, ret, "cannot read the PMIC id\n");
	dev_info(dev, "S2MPU06 PMIC id %#x\n", id);

	regs_np = of_get_child_by_name(dev->of_node, "regulators");
	if (!regs_np)
		return dev_err_probe(dev, -EINVAL, "no regulators node\n");

	config.dev = dev;
	config.regmap = pmic;

	for (i = 0; i < ARRAY_SIZE(s2mpu06_regulators); i++) {
		struct device_node *np;
		struct regulator_dev *rdev;

		/* skip rails the board does not describe, so they stay as the bootloader left them */
		np = of_get_child_by_name(regs_np, s2mpu06_regulators[i].name);
		if (!np)
			continue;
		of_node_put(np);

		rdev = devm_regulator_register(dev, &s2mpu06_regulators[i], &config);
		if (IS_ERR(rdev)) {
			ret = dev_err_probe(dev, PTR_ERR(rdev), "cannot register %s\n",
					    s2mpu06_regulators[i].name);
			of_node_put(regs_np);
			return ret;
		}
	}

	of_node_put(regs_np);
	return 0;
}

static const struct of_device_id s2mpu06_of_match[] = {
	{ .compatible = "samsung,s2mpu06-pmic" },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mpu06_of_match);

static struct platform_driver s2mpu06_driver = {
	.probe = s2mpu06_probe,
	.driver = {
		.name = "s2mpu06-regulator",
		.of_match_table = s2mpu06_of_match,
	},
};
module_platform_driver(s2mpu06_driver);

MODULE_DESCRIPTION("Samsung S2MPU06 regulator driver");
MODULE_LICENSE("GPL");
