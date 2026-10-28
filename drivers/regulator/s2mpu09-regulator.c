// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung S2MPU09 PMIC LDO regulators, accessed through the APM firmware
 * (ACPM) on Exynos9610: the APM owns the SPEEDY bus the PMIC sits on, so the
 * AP sends register reads and writes as ACPM PMIC commands.
 *
 * Only the LDOs that mainline consumers need are described so far: LDO12-14,
 * the USB PHY supplies (0.85 V, 1.8 V, 3.0 V on the Galaxy A51). The regmap
 * only allows their control registers.
 *
 * Register layout from the downstream s2mpu09 driver: LDOx_CTRL holds the
 * voltage selector in [5:0] and the enable mode in [7:6] (3 = on in normal
 * mode).
 */

#include <linux/bits.h>
#include <linux/err.h>
#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#define S2MPU09_ACPM_TYPE_PMIC		0x01	/* PMIC register block */
#define S2MPU09_ACPM_SPEEDY_CHAN	0

#define S2MPU09_REG_L12CTRL		0x45
#define S2MPU09_REG_L13CTRL		0x46
#define S2MPU09_REG_L14CTRL		0x47

#define S2MPU09_LDO_VSEL_MASK		GENMASK(5, 0)
#define S2MPU09_LDO_N_VOLTAGES		(S2MPU09_LDO_VSEL_MASK + 1)
#define S2MPU09_ENABLE_MASK		GENMASK(7, 6)
#define S2MPU09_ENABLE_ON		GENMASK(7, 6)
#define S2MPU09_ENABLE_TIME_LDO		128

struct s2mpu09_acpm {
	struct acpm_handle *handle;
	unsigned int acpm_chan;
};

static int s2mpu09_acpm_reg_read(void *context, unsigned int reg,
				 unsigned int *val)
{
	struct s2mpu09_acpm *acpm = context;
	u8 v;
	int ret;

	ret = acpm->handle->ops->pmic.read_reg(acpm->handle, acpm->acpm_chan,
					       S2MPU09_ACPM_TYPE_PMIC, reg,
					       S2MPU09_ACPM_SPEEDY_CHAN, &v);
	if (!ret)
		*val = v;
	return ret;
}

static int s2mpu09_acpm_reg_write(void *context, unsigned int reg,
				  unsigned int val)
{
	struct s2mpu09_acpm *acpm = context;

	return acpm->handle->ops->pmic.write_reg(acpm->handle, acpm->acpm_chan,
						 S2MPU09_ACPM_TYPE_PMIC, reg,
						 S2MPU09_ACPM_SPEEDY_CHAN, val);
}

static bool s2mpu09_accessible_reg(struct device *dev, unsigned int reg)
{
	return reg >= S2MPU09_REG_L12CTRL && reg <= S2MPU09_REG_L14CTRL;
}

static const struct regmap_config s2mpu09_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = S2MPU09_REG_L14CTRL,
	.readable_reg = s2mpu09_accessible_reg,
	.writeable_reg = s2mpu09_accessible_reg,
	.reg_read = s2mpu09_acpm_reg_read,
	.reg_write = s2mpu09_acpm_reg_write,
	.cache_type = REGCACHE_NONE,
};

static const struct regulator_ops s2mpu09_ldo_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.is_enabled		= regulator_is_enabled_regmap,
};

#define S2MPU09_LDO(_num, _min, _step)					\
	{								\
		.name		= "LDO" #_num,				\
		.of_match	= of_match_ptr("LDO" #_num),		\
		.regulators_node = of_match_ptr("regulators"),		\
		.id		= _num,					\
		.ops		= &s2mpu09_ldo_ops,			\
		.type		= REGULATOR_VOLTAGE,			\
		.owner		= THIS_MODULE,				\
		.min_uV		= _min,					\
		.uV_step	= _step,				\
		.n_voltages	= S2MPU09_LDO_N_VOLTAGES,		\
		.vsel_reg	= S2MPU09_REG_L##_num##CTRL,		\
		.vsel_mask	= S2MPU09_LDO_VSEL_MASK,		\
		.enable_reg	= S2MPU09_REG_L##_num##CTRL,		\
		.enable_mask	= S2MPU09_ENABLE_MASK,			\
		.enable_val	= S2MPU09_ENABLE_ON,			\
		.enable_time	= S2MPU09_ENABLE_TIME_LDO,		\
	}

static const struct regulator_desc s2mpu09_regulators[] = {
	S2MPU09_LDO(12, 700000, 12500),
	S2MPU09_LDO(13, 700000, 25000),
	S2MPU09_LDO(14, 1800000, 25000),
};

static int s2mpu09_regulator_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regulator_config config = { };
	struct s2mpu09_acpm *acpm;
	struct regmap *regmap;
	int i;

	acpm = devm_kzalloc(dev, sizeof(*acpm), GFP_KERNEL);
	if (!acpm)
		return -ENOMEM;

	acpm->handle = devm_acpm_get_by_node(dev, dev->parent->of_node);
	if (IS_ERR(acpm->handle))
		return dev_err_probe(dev, PTR_ERR(acpm->handle),
				     "failed to get the ACPM handle\n");

	if (of_property_read_u32(dev->of_node, "samsung,acpm-channel",
				 &acpm->acpm_chan))
		return dev_err_probe(dev, -EINVAL,
				     "missing samsung,acpm-channel\n");

	regmap = devm_regmap_init(dev, NULL, acpm, &s2mpu09_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap),
				     "failed to init the ACPM regmap\n");

	config.dev = dev;
	config.regmap = regmap;

	for (i = 0; i < ARRAY_SIZE(s2mpu09_regulators); i++) {
		struct regulator_dev *rdev;
		unsigned int val;

		/* log what S-BOOT left, before the core applies constraints */
		if (!regmap_read(regmap, s2mpu09_regulators[i].vsel_reg, &val))
			dev_info(dev, "%s ctrl %#04x\n",
				 s2mpu09_regulators[i].name, val);

		rdev = devm_regulator_register(dev, &s2mpu09_regulators[i],
					       &config);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev),
					     "failed to register %s\n",
					     s2mpu09_regulators[i].name);
	}

	return 0;
}

static const struct of_device_id s2mpu09_regulator_of_match[] = {
	{ .compatible = "samsung,s2mpu09-acpm-regulator" },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mpu09_regulator_of_match);

static struct platform_driver s2mpu09_regulator_driver = {
	.driver = {
		.name = "s2mpu09-regulator",
		.of_match_table = s2mpu09_regulator_of_match,
	},
	.probe = s2mpu09_regulator_probe,
};
module_platform_driver(s2mpu09_regulator_driver);

MODULE_DESCRIPTION("Samsung S2MPU09 LDO regulators over ACPM");
MODULE_LICENSE("GPL");
