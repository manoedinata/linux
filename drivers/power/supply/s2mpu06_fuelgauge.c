// SPDX-License-Identifier: GPL-2.0
/*
 * Fuel gauge of the Samsung S2MPU06 PMIC
 *
 * The gauge measures and estimates on its own; this driver only reads its
 * result registers. Register layout from the downstream s2mpu06_fuelgauge
 * driver: Copyright (C) 2015 Samsung Electronics
 */

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/iio/consumer.h>
#include <linux/math64.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/mfd/samsung/s2mpu06.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#define S2MPU06_FG_RVBAT		0x04
#define S2MPU06_FG_ROCV			0x06
#define S2MPU06_FG_RCUR			0x08
#define S2MPU06_FG_RSOC			0x0c
#define S2MPU06_FG_RTEMP		0x0e

#define S2MPU06_FG_NEG			BIT(15)
#define S2MPU06_FG_RETRIES		10
#define S2MPU06_FG_TEMP_PERIOD		msecs_to_jiffies(30000)

struct s2mpu06_fuelgauge {
	struct device *dev;
	struct s2mpu06 *s2m;
	struct power_supply *psy;
	struct power_supply_battery_info *info;

	/* battery thermistor: raw ADC value -> 0.1 degC */
	struct iio_channel *adc;
	u32 *temp_adc;
	s32 *temp_deci;
	int temp_points;
	int temp;
	int temp_bucket;
	struct delayed_work temp_work;
};

/*
 * The 16-bit results are read a byte at a time, as the vendor driver does, and
 * the gauge may update them in between. A torn read shows as the high byte
 * changing across the low byte read, so retry until it does not.
 */
static int s2mpu06_fg_read16(struct s2mpu06_fuelgauge *fg, unsigned int reg, u16 *val)
{
	unsigned int lo, hi, hi2;
	int i, ret;

	for (i = 0; i < S2MPU06_FG_RETRIES; i++) {
		ret = regmap_read(fg->s2m->fg, reg + 1, &hi);
		if (ret)
			return ret;
		ret = regmap_read(fg->s2m->fg, reg, &lo);
		if (ret)
			return ret;
		ret = regmap_read(fg->s2m->fg, reg + 1, &hi2);
		if (ret)
			return ret;

		if (hi == hi2) {
			*val = (hi << 8) | lo;
			return 0;
		}
	}

	return -EAGAIN;
}

/* 13 fractional bits of volts */
static int s2mpu06_fg_read_uv(struct s2mpu06_fuelgauge *fg, unsigned int reg, int *uv)
{
	u16 raw;
	int ret;

	ret = s2mpu06_fg_read16(fg, reg, &raw);
	if (ret)
		return ret;

	*uv = ((u32)raw * 1000 >> 13) * 1000;
	return 0;
}

/* signed 4.12 fixed point of a percent/100 in the gauge; returns 1/100 %, may be < 0 */
static int s2mpu06_fg_read_soc(struct s2mpu06_fuelgauge *fg, int *soc)
{
	u16 a;
	int ret;

	ret = s2mpu06_fg_read16(fg, S2MPU06_FG_RSOC, &a);
	if (ret)
		return ret;

	if (a & S2MPU06_FG_NEG)
		*soc = -(int)(((u16)~a + 1) * 10000 >> 12);
	else
		*soc = (a & 0x7fff) * 10000 >> 12;

	return 0;
}

static int s2mpu06_fg_adc_to_temp(struct s2mpu06_fuelgauge *fg, int raw)
{
	int i, n = fg->temp_points;

	if (raw <= fg->temp_adc[0])
		return fg->temp_deci[0];
	if (raw >= fg->temp_adc[n - 1])
		return fg->temp_deci[n - 1];

	for (i = 1; i < n; i++) {
		if (raw > fg->temp_adc[i])
			continue;
		return fg->temp_deci[i - 1] +
		       (int)div_s64((s64)(fg->temp_deci[i] - fg->temp_deci[i - 1]) *
				    (raw - fg->temp_adc[i - 1]),
				    fg->temp_adc[i] - fg->temp_adc[i - 1]);
	}

	return fg->temp_deci[n - 1];
}

/* the gauge takes the temperature as one of four coarse environments, in degC */
static int s2mpu06_fg_temp_bucket(int deci)
{
	int c = deci / 10;

	if (c < -5)
		return 0;
	if (c < 15)
		return 10;
	if (c < 35)
		return 25;
	return 40;
}

static void s2mpu06_fg_temp_work(struct work_struct *work)
{
	struct s2mpu06_fuelgauge *fg = container_of(work, struct s2mpu06_fuelgauge, temp_work.work);
	int raw, bucket;

	if (iio_read_channel_raw(fg->adc, &raw) >= 0) {
		fg->temp = s2mpu06_fg_adc_to_temp(fg, raw);

		bucket = s2mpu06_fg_temp_bucket(fg->temp);
		if (bucket != fg->temp_bucket &&
		    !regmap_write(fg->s2m->fg, S2MPU06_FG_RTEMP, bucket))
			fg->temp_bucket = bucket;
	}

	schedule_delayed_work(&fg->temp_work, S2MPU06_FG_TEMP_PERIOD);
}

static int s2mpu06_fg_get_status(struct s2mpu06_fuelgauge *fg)
{
	struct power_supply *chg = power_supply_get_by_name("s2mpu06-charger");
	union power_supply_propval val;
	int ret;

	if (!chg)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	ret = power_supply_get_property(chg, POWER_SUPPLY_PROP_STATUS, &val);
	power_supply_put(chg);

	return ret ? POWER_SUPPLY_STATUS_UNKNOWN : val.intval;
}

static int s2mpu06_fg_get_property(struct power_supply *psy, enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct s2mpu06_fuelgauge *fg = power_supply_get_drvdata(psy);
	int ret, soc;
	u16 raw;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = s2mpu06_fg_get_status(fg);
		return 0;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		return 0;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		return 0;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		return s2mpu06_fg_read_uv(fg, S2MPU06_FG_RVBAT, &val->intval);
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		return s2mpu06_fg_read_uv(fg, S2MPU06_FG_ROCV, &val->intval);
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = s2mpu06_fg_read16(fg, S2MPU06_FG_RCUR, &raw);
		if (ret)
			return ret;
		/* 4096 counts per ampere; positive when charging */
		if (raw & S2MPU06_FG_NEG)
			val->intval = ((u16)~raw + 1) * 1000 >> 12;
		else
			val->intval = -(int)((raw & 0x7fff) * 1000 >> 12);
		val->intval *= 1000;
		return 0;
	case POWER_SUPPLY_PROP_TEMP:
		if (!fg->adc)
			return -ENODATA;
		val->intval = fg->temp;
		return 0;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = s2mpu06_fg_read_soc(fg, &soc);
		if (ret)
			return ret;
		val->intval = clamp(soc / 100, 0, 100);
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		if (!fg->info || fg->info->charge_full_design_uah < 0)
			return -ENODATA;
		val->intval = fg->info->charge_full_design_uah;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		if (!fg->info || fg->info->voltage_max_design_uv < 0)
			return -ENODATA;
		val->intval = fg->info->voltage_max_design_uv;
		return 0;
	default:
		return -EINVAL;
	}
}

static const enum power_supply_property s2mpu06_fg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
};

static const struct power_supply_desc s2mpu06_fg_desc = {
	.name = "s2mpu06-fuelgauge",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = s2mpu06_fg_props,
	.num_properties = ARRAY_SIZE(s2mpu06_fg_props),
	.get_property = s2mpu06_fg_get_property,
};

static void s2mpu06_fg_put_info(void *data)
{
	struct s2mpu06_fuelgauge *fg = data;

	if (fg->info)
		power_supply_put_battery_info(fg->psy, fg->info);
}

static void s2mpu06_fg_cancel_temp_work(void *data)
{
	cancel_delayed_work_sync(data);
}

/* optional: the NTC thermistor is read by the AP's ADC and converted with a table */
static int s2mpu06_fg_init_temp(struct s2mpu06_fuelgauge *fg)
{
	struct device *dev = fg->dev;
	int n, raw, ret;

	fg->adc = devm_iio_channel_get(dev, "adc-temp");
	if (IS_ERR(fg->adc)) {
		ret = PTR_ERR(fg->adc);
		fg->adc = NULL;
		return ret == -EPROBE_DEFER ? ret : 0;
	}

	n = of_property_count_u32_elems(dev->of_node, "samsung,temp-adc-table");
	if (n < 2 || n != of_property_count_u32_elems(dev->of_node, "samsung,temp-decidegc-table")) {
		dev_warn(dev, "no usable temperature table, temperature disabled\n");
		fg->adc = NULL;
		return 0;
	}

	fg->temp_adc = devm_kcalloc(dev, n, sizeof(*fg->temp_adc), GFP_KERNEL);
	fg->temp_deci = devm_kcalloc(dev, n, sizeof(*fg->temp_deci), GFP_KERNEL);
	if (!fg->temp_adc || !fg->temp_deci)
		return -ENOMEM;
	fg->temp_points = n;

	ret = of_property_read_u32_array(dev->of_node, "samsung,temp-adc-table", fg->temp_adc, n);
	if (!ret)
		ret = of_property_read_u32_array(dev->of_node, "samsung,temp-decidegc-table",
						 (u32 *)fg->temp_deci, n);
	if (ret)
		return ret;

	fg->temp_bucket = -1;
	if (iio_read_channel_raw(fg->adc, &raw) >= 0)
		fg->temp = s2mpu06_fg_adc_to_temp(fg, raw);

	INIT_DELAYED_WORK(&fg->temp_work, s2mpu06_fg_temp_work);
	ret = devm_add_action_or_reset(dev, s2mpu06_fg_cancel_temp_work, &fg->temp_work);
	if (ret)
		return ret;
	schedule_delayed_work(&fg->temp_work, 0);

	return 0;
}

static int s2mpu06_fg_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct power_supply_config cfg = { };
	struct s2mpu06_fuelgauge *fg;
	int ret;

	fg = devm_kzalloc(dev, sizeof(*fg), GFP_KERNEL);
	if (!fg)
		return -ENOMEM;

	fg->dev = dev;
	fg->s2m = dev_get_drvdata(dev->parent);

	/* the battery description hangs off the PMIC's node */
	device_set_of_node_from_dev(dev, dev->parent);
	cfg.drv_data = fg;
	cfg.fwnode = dev_fwnode(dev);

	ret = s2mpu06_fg_init_temp(fg);
	if (ret)
		return dev_err_probe(dev, ret, "cannot set up the temperature sensing\n");

	fg->psy = devm_power_supply_register(dev, &s2mpu06_fg_desc, &cfg);
	if (IS_ERR(fg->psy))
		return dev_err_probe(dev, PTR_ERR(fg->psy), "cannot register the fuel gauge\n");

	if (power_supply_get_battery_info(fg->psy, &fg->info))
		fg->info = NULL;
	else
		devm_add_action_or_reset(dev, s2mpu06_fg_put_info, fg);

	return 0;
}

static const struct platform_device_id s2mpu06_fg_id[] = {
	{ "s2mpu06-fuelgauge" },
	{ }
};
MODULE_DEVICE_TABLE(platform, s2mpu06_fg_id);

static struct platform_driver s2mpu06_fg_driver = {
	.probe = s2mpu06_fg_probe,
	.driver = {
		.name = "s2mpu06-fuelgauge",
	},
	.id_table = s2mpu06_fg_id,
};
module_platform_driver(s2mpu06_fg_driver);

MODULE_DESCRIPTION("Samsung S2MPU06 fuel gauge driver");
MODULE_LICENSE("GPL");
