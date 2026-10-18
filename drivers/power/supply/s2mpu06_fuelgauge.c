// SPDX-License-Identifier: GPL-2.0
/*
 * Fuel gauge of the Samsung S2MPU06 PMIC
 *
 * The gauge measures and estimates on its own. This driver reads its result
 * registers, scales and smooths the capacity like the vendor driver does,
 * feeds it the battery temperature, handles its low-battery alerts and
 * reloads its cell characteristics when the battery has aged.
 *
 * Register layout and algorithms from the downstream s2mpu06_fuelgauge
 * driver: Copyright (C) 2015 Samsung Electronics
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/iio/consumer.h>
#include <linux/math64.h>
#include <linux/mfd/samsung/s2mpu06.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#define S2MPU06_FG_RVBAT		0x04
#define S2MPU06_FG_ROCV			0x06
#define S2MPU06_FG_RCUR			0x08
#define S2MPU06_FG_RSOC			0x0c
#define S2MPU06_FG_RTEMP		0x0e
#define S2MPU06_FG_CELL_CHAR1		0x0f	/* 5 bytes */
#define S2MPU06_FG_CELL_CHAR1_LEN	5
#define S2MPU06_FG_IRQ_LVL		0x18
#define S2MPU06_FG_START		0x1a
#define S2MPU06_FG_START_RESET		0x33
#define S2MPU06_FG_POR			0x1b
#define S2MPU06_FG_POR_BIT		BIT(4)
#define S2MPU06_FG_VBAT_I2C_L		0x22
#define S2MPU06_FG_VBAT_I2C_H		0x23
#define S2MPU06_FG_CTRL			0x2a
#define S2MPU06_FG_CTRL_VBAT_I2C	BIT(3)
#define S2MPU06_FG_CTRL_DATA_VER	GENMASK(2, 0)
#define S2MPU06_FG_OFFSET_0		0x2f	/* 20-bit offset in 0x2f..0x31 */
#define S2MPU06_FG_CELL_CHAR0		0x34	/* 164 bytes */
#define S2MPU06_FG_CELL_CHAR0_LEN	164

#define S2MPU06_FG_NEG			BIT(15)
#define S2MPU06_FG_RETRIES		10
#define S2MPU06_FG_POLL_PERIOD		msecs_to_jiffies(10000)
#define S2MPU06_FG_CURRENT_SAMPLES	10

/* capacity is kept in 0.1 % units for the scaling */
#define S2MPU06_FG_CAPACITY_FULL	1000

/* the vendor's power-off rule */
#define S2MPU06_FG_VCELL_OK_MV		3400
#define S2MPU06_FG_VCELL_EMPTY_MV	3200
#define S2MPU06_FG_DISCHARGE_SAMPLES	5

struct s2mpu06_fuelgauge {
	struct device *dev;
	struct s2mpu06 *s2m;
	struct power_supply *psy;
	struct power_supply_battery_info *info;
	struct mutex lock;
	struct delayed_work work;
	struct notifier_block alert_nb;

	/* battery thermistor: raw ADC value -> 0.1 degC */
	struct iio_channel *adc;
	u32 *temp_adc;
	s32 *temp_deci;
	int temp_points;
	int temp;
	bool temp_valid;
	int temp_bucket;

	/* capacity scaling, 0.1 % units */
	u32 capacity_max_cfg;
	u32 capacity_margin;
	u32 capacity_max;
	int capacity;		/* percent reported */
	bool capacity_valid;
	int fuel_alert_soc;
	unsigned int alert_status;

	/* charger state */
	bool charging;
	int charger_status;

	/* averages and the power-off rule */
	int cur_hist[S2MPU06_FG_CURRENT_SAMPLES];
	int cur_cnt;
	int cur_idx;
	int pre_vcell_mv;
	int discharge_cnt;
	int level;

	/* ageing */
	int num_steps;
	u32 *age_cycle;
	u32 *age_float_uv;
	u32 *age_raw100_mv;
	u8 *cell_char0;
	u8 *cell_char1;
	int cycle_count;	/* -1 until userspace tells */
	int want_step;
	int data_ver;
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
static int s2mpu06_fg_read_mv(struct s2mpu06_fuelgauge *fg, unsigned int reg, int *mv)
{
	u16 raw;
	int ret;

	ret = s2mpu06_fg_read16(fg, reg, &raw);
	if (ret)
		return ret;

	*mv = (u32)raw * 1000 >> 13;
	return 0;
}

/* the gauge's own state of charge in 0.01 %, may be negative */
static int s2mpu06_fg_read_rawsoc(struct s2mpu06_fuelgauge *fg, int *soc)
{
	u16 a;
	int ret;

	ret = s2mpu06_fg_read16(fg, S2MPU06_FG_RSOC, &a);
	if (ret)
		return ret;

	if (a & S2MPU06_FG_NEG)
		*soc = -(int)(((u16)~a + 1) * 10000 >> 12);
	else
		*soc = min((a & 0x7fff) * 10000 >> 12, 10000);

	return 0;
}

/* in mA, positive when charging */
static int s2mpu06_fg_read_current_ma(struct s2mpu06_fuelgauge *fg, int *ma)
{
	u16 raw;
	int ret;

	ret = s2mpu06_fg_read16(fg, S2MPU06_FG_RCUR, &raw);
	if (ret)
		return ret;

	/* 4096 counts per ampere */
	if (raw & S2MPU06_FG_NEG)
		*ma = ((u16)~raw + 1) * 1000 >> 12;
	else
		*ma = -(int)((raw & 0x7fff) * 1000 >> 12);

	return 0;
}

/* ------------------------------------------------------------- temperature */

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

static void s2mpu06_fg_update_temp(struct s2mpu06_fuelgauge *fg)
{
	int raw, bucket;

	if (!fg->adc || iio_read_channel_raw(fg->adc, &raw) < 0)
		return;

	fg->temp = s2mpu06_fg_adc_to_temp(fg, raw);
	fg->temp_valid = true;

	bucket = s2mpu06_fg_temp_bucket(fg->temp);
	if (bucket != fg->temp_bucket && !regmap_write(fg->s2m->fg, S2MPU06_FG_RTEMP, bucket))
		fg->temp_bucket = bucket;
}

/* -------------------------------------------------------- capacity policy */

static struct power_supply *s2mpu06_fg_charger(void)
{
	return power_supply_get_by_name("s2mpu06-charger");
}

static int s2mpu06_fg_check_capacity_max(struct s2mpu06_fuelgauge *fg, u32 capacity_max)
{
	if (capacity_max < fg->capacity_max_cfg - fg->capacity_margin - 10)
		return fg->capacity_max_cfg - fg->capacity_margin;
	if (capacity_max > fg->capacity_max_cfg + fg->capacity_margin)
		return fg->capacity_max_cfg + fg->capacity_margin;

	return capacity_max;
}

/*
 * When the battery has just been charged full, take the gauge's raw reading
 * as the new 100 % so that the reported capacity reaches 100 % even though the
 * gauge stops short of it, or overshoots it.
 */
static void s2mpu06_fg_dynamic_scale(struct s2mpu06_fuelgauge *fg, int capacity)
{
	int raw;

	if (s2mpu06_fg_read_rawsoc(fg, &raw))
		return;
	raw /= 10;

	if (raw < (int)(fg->capacity_max_cfg - fg->capacity_margin))
		return;	/* implausibly low, keep the current scale */

	fg->capacity_max = min_t(u32, raw, fg->capacity_max_cfg + fg->capacity_margin);
	fg->capacity_max = s2mpu06_fg_check_capacity_max(fg, fg->capacity_max * 100 / (capacity + 1));
	fg->capacity = capacity;	/* so the stepping below does not walk to it slowly */
	dev_info(fg->dev, "capacity max is now %u\n", fg->capacity_max);
}

static void s2mpu06_fg_update_capacity(struct s2mpu06_fuelgauge *fg)
{
	int raw, scaled, cap;

	if (s2mpu06_fg_read_rawsoc(fg, &raw))
		return;
	raw = min(raw / 10, S2MPU06_FG_CAPACITY_FULL);

	scaled = raw * S2MPU06_FG_CAPACITY_FULL / fg->capacity_max;
	cap = clamp(scaled, 0, S2MPU06_FG_CAPACITY_FULL) / 10;

	if (!fg->capacity_valid) {
		fg->capacity_valid = true;
		fg->capacity = cap;
		return;
	}

	/* move one percent per update ... */
	if (fg->capacity < cap)
		cap = fg->capacity + 1;
	else if (fg->capacity > cap)
		cap = fg->capacity - 1;

	/* ... and never up while discharging, that is a misreading */
	if (!fg->charging && fg->capacity < cap)
		cap = fg->capacity;

	fg->capacity = cap;
}

/* the vendor's rule for powering off a battery that has really run empty */
static void s2mpu06_fg_update_level(struct s2mpu06_fuelgauge *fg, int vcell_mv)
{
	bool discharging = false;
	int health;

	if (vcell_mv > S2MPU06_FG_VCELL_OK_MV) {
		fg->discharge_cnt = 0;
	} else {
		union power_supply_propval val;
		struct power_supply *chg = s2mpu06_fg_charger();

		health = POWER_SUPPLY_HEALTH_GOOD;
		if (chg) {
			if (!power_supply_get_property(chg, POWER_SUPPLY_PROP_HEALTH, &val))
				health = val.intval;
			power_supply_put(chg);
		}

		if (health == POWER_SUPPLY_HEALTH_OVERHEAT || health == POWER_SUPPLY_HEALTH_COLD) {
			discharging = true;
		} else {
			if (!fg->pre_vcell_mv || fg->pre_vcell_mv > vcell_mv)
				fg->discharge_cnt++;
			else
				fg->discharge_cnt--;
			discharging = fg->discharge_cnt >= S2MPU06_FG_DISCHARGE_SAMPLES;
		}
	}
	fg->pre_vcell_mv = vcell_mv;

	if (fg->capacity <= 0 && (discharging || vcell_mv < S2MPU06_FG_VCELL_EMPTY_MV))
		fg->level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	else if (fg->alert_status & (S2MPU06_FG_IRQ_SOC_LOW | S2MPU06_FG_IRQ_VBAT_LOW))
		fg->level = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	else if (fg->capacity >= 100 || fg->charger_status == POWER_SUPPLY_STATUS_FULL)
		fg->level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	else
		fg->level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
}

/* ---------------------------------------------------------------- alerts */

static int s2mpu06_fg_program_alert(struct s2mpu06_fuelgauge *fg)
{
	int ret;

	/* low nibble: voltage threshold (3.55 V), high nibble: capacity in percent */
	ret = regmap_write(fg->s2m->fg, S2MPU06_FG_IRQ_LVL, fg->fuel_alert_soc << 4);
	if (ret)
		return ret;

	return regmap_write(fg->s2m->fg, S2MPU06_FG_IRQ_LVL + 1, 0);
}

static int s2mpu06_fg_alert_event(struct notifier_block *nb, unsigned long status, void *data)
{
	struct s2mpu06_fuelgauge *fg = container_of(nb, struct s2mpu06_fuelgauge, alert_nb);

	fg->alert_status = status & (S2MPU06_FG_IRQ_SOC_LOW | S2MPU06_FG_IRQ_VBAT_LOW);
	dev_info(fg->dev, "fuel alert %#lx\n", status);

	/* the poll turns it into the capacity level and tells userspace */
	mod_delayed_work(system_percpu_wq, &fg->work, 0);

	return NOTIFY_OK;
}

/* ----------------------------------------------------------------- ageing */

static int s2mpu06_fg_cell_matches(struct s2mpu06_fuelgauge *fg, int step)
{
	const u8 *tbl = fg->cell_char0 + step * S2MPU06_FG_CELL_CHAR0_LEN;
	unsigned int val;
	int i, ret;

	/* the vendor driver compares the first four bytes */
	for (i = 0; i < 4; i++) {
		ret = regmap_read(fg->s2m->fg, S2MPU06_FG_CELL_CHAR0 + i, &val);
		if (ret)
			return ret;
		if (val != tbl[i])
			return 0;
	}

	return 1;
}

/* load the cell characteristics of an ageing step and have the gauge restart from them */
static int s2mpu06_fg_reset(struct s2mpu06_fuelgauge *fg, int step)
{
	struct regmap *fgmap = fg->s2m->fg;
	unsigned int reg2f, reg30, reg31, v;
	u32 offset = 0x30000, vbat;
	int ret;

	ret = regmap_bulk_write(fgmap, S2MPU06_FG_CELL_CHAR0,
				fg->cell_char0 + step * S2MPU06_FG_CELL_CHAR0_LEN,
				S2MPU06_FG_CELL_CHAR0_LEN);
	if (!ret)
		ret = regmap_bulk_write(fgmap, S2MPU06_FG_CELL_CHAR1,
					fg->cell_char1 + step * S2MPU06_FG_CELL_CHAR1_LEN,
					S2MPU06_FG_CELL_CHAR1_LEN);
	if (ret)
		return ret;

	ret = regmap_read(fgmap, S2MPU06_FG_OFFSET_0, &reg2f);
	if (!ret)
		ret = regmap_read(fgmap, S2MPU06_FG_OFFSET_0 + 1, &reg30);
	if (!ret)
		ret = regmap_read(fgmap, S2MPU06_FG_OFFSET_0 + 2, &reg31);
	if (ret)
		return ret;

	/* the long-life algorithm restarts from its default offset */
	regmap_write(fgmap, S2MPU06_FG_OFFSET_0 + 2, (offset >> 16) & 0x0f);
	regmap_write(fgmap, S2MPU06_FG_OFFSET_0 + 1, (offset >> 8) & 0xff);
	regmap_write(fgmap, S2MPU06_FG_OFFSET_0, offset & 0xff);

	/*
	 * This runs after the second top-off, so the voltage the gauge is told
	 * to believe is the "100 %" voltage of the step plus 10 mV.
	 */
	vbat = ((fg->age_raw100_mv[step] + 10 - 3000) << 15) / 3000;
	regmap_write(fgmap, S2MPU06_FG_VBAT_I2C_H, (vbat >> 8) & 0xff);
	regmap_write(fgmap, S2MPU06_FG_VBAT_I2C_L, vbat & 0xff);
	regmap_update_bits(fgmap, S2MPU06_FG_CTRL, S2MPU06_FG_CTRL_VBAT_I2C, S2MPU06_FG_CTRL_VBAT_I2C);

	msleep(10);
	regmap_write(fgmap, S2MPU06_FG_START, S2MPU06_FG_START_RESET);
	msleep(500);

	regmap_update_bits(fgmap, S2MPU06_FG_CTRL, S2MPU06_FG_CTRL_VBAT_I2C, 0);
	regmap_write(fgmap, S2MPU06_FG_OFFSET_0, reg2f);
	regmap_write(fgmap, S2MPU06_FG_OFFSET_0 + 1, reg30);
	regmap_write(fgmap, S2MPU06_FG_OFFSET_0 + 2, reg31);

	/* the reset puts the data version back to its default, restore it */
	if (!regmap_read(fgmap, S2MPU06_FG_CTRL, &v))
		regmap_write(fgmap, S2MPU06_FG_CTRL, (v & ~S2MPU06_FG_CTRL_DATA_VER) |
			     (fg->data_ver & S2MPU06_FG_CTRL_DATA_VER));

	return 0;
}

/* at full charge: switch to the cell data of the ageing step if the gauge has not got it yet */
static void s2mpu06_fg_aging_check(struct s2mpu06_fuelgauge *fg)
{
	union power_supply_propval val;
	struct power_supply *chg;
	int ret;

	if (!fg->num_steps || fg->want_step < 0)
		return;

	ret = s2mpu06_fg_cell_matches(fg, fg->want_step);
	if (ret != 0)
		return;

	dev_info(fg->dev, "reloading the cell data for ageing step %d\n", fg->want_step);

	/* the gauge restarts: a power loss in between must restore its state next time */
	regmap_update_bits(fg->s2m->fg, S2MPU06_FG_POR, S2MPU06_FG_POR_BIT, S2MPU06_FG_POR_BIT);

	chg = s2mpu06_fg_charger();
	if (chg) {
		val.intval = POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE;
		power_supply_set_property(chg, POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR, &val);
		msleep(100);
	}

	ret = s2mpu06_fg_reset(fg, fg->want_step);

	if (chg) {
		val.intval = POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO;
		power_supply_set_property(chg, POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR, &val);
		power_supply_put(chg);
	}

	if (!ret)
		regmap_update_bits(fg->s2m->fg, S2MPU06_FG_POR, S2MPU06_FG_POR_BIT, 0);
}

/* pick the step for the cycle count; the float voltage of the step goes to the charger */
static void s2mpu06_fg_select_step(struct s2mpu06_fuelgauge *fg)
{
	union power_supply_propval val;
	struct power_supply *chg;
	int step;

	if (!fg->num_steps || fg->cycle_count < 0)
		return;

	/* the vendor driver leaves the ageing alone in the cold */
	if (fg->adc && fg->temp < 50)
		return;

	for (step = fg->num_steps - 1; step > 0; step--)
		if (fg->age_cycle[step] <= fg->cycle_count)
			break;

	if (step == fg->want_step)
		return;

	fg->want_step = step;
	dev_info(fg->dev, "ageing step %d for %d cycles, float voltage %u uV\n", step,
		 fg->cycle_count, fg->age_float_uv[step]);

	chg = s2mpu06_fg_charger();
	if (chg) {
		val.intval = fg->age_float_uv[step];
		power_supply_set_property(chg, POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE, &val);
		power_supply_put(chg);
	}
}

/* ------------------------------------------------------------------ poll */

static void s2mpu06_fg_work(struct work_struct *work)
{
	struct s2mpu06_fuelgauge *fg = container_of(work, struct s2mpu06_fuelgauge, work.work);
	union power_supply_propval val;
	struct power_supply *chg;
	int prev_cap, prev_level, status = POWER_SUPPLY_STATUS_UNKNOWN, ma, mv = 0;
	bool online = false;

	mutex_lock(&fg->lock);
	prev_cap = fg->capacity;
	prev_level = fg->level;

	s2mpu06_fg_update_temp(fg);

	chg = s2mpu06_fg_charger();
	if (chg) {
		if (!power_supply_get_property(chg, POWER_SUPPLY_PROP_ONLINE, &val))
			online = val.intval;
		if (!power_supply_get_property(chg, POWER_SUPPLY_PROP_STATUS, &val))
			status = val.intval;
		power_supply_put(chg);
	}
	fg->charging = online;

	if (status == POWER_SUPPLY_STATUS_FULL && fg->charger_status != POWER_SUPPLY_STATUS_FULL) {
		s2mpu06_fg_dynamic_scale(fg, 100);
		s2mpu06_fg_select_step(fg);
		s2mpu06_fg_aging_check(fg);
	}
	fg->charger_status = status;

	if (!s2mpu06_fg_read_current_ma(fg, &ma)) {
		fg->cur_hist[fg->cur_idx] = ma;
		fg->cur_idx = (fg->cur_idx + 1) % S2MPU06_FG_CURRENT_SAMPLES;
		fg->cur_cnt = min(fg->cur_cnt + 1, S2MPU06_FG_CURRENT_SAMPLES);
	}

	s2mpu06_fg_update_capacity(fg);
	if (!s2mpu06_fg_read_mv(fg, S2MPU06_FG_RVBAT, &mv)) {
		s2mpu06_fg_update_level(fg, mv);

		/* the alert flag is only meaningful until the level recovers */
		if (fg->alert_status && fg->capacity > fg->fuel_alert_soc && mv > 3600)
			fg->alert_status = 0;
	}

	mutex_unlock(&fg->lock);

	if (fg->capacity != prev_cap || fg->level != prev_level)
		power_supply_changed(fg->psy);

	schedule_delayed_work(&fg->work, S2MPU06_FG_POLL_PERIOD);
}

static void s2mpu06_fg_external_power_changed(struct power_supply *psy)
{
	struct s2mpu06_fuelgauge *fg = power_supply_get_drvdata(psy);

	mod_delayed_work(system_percpu_wq, &fg->work, 0);
}

/* ------------------------------------------------------------ properties */

static int s2mpu06_fg_get_status(void)
{
	struct power_supply *chg = s2mpu06_fg_charger();
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
	union power_supply_propval chg_val;
	struct power_supply *chg;
	int ret = 0, i, sum, mv, cnt;

	mutex_lock(&fg->lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = s2mpu06_fg_get_status();
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		/* too hot or too cold is the charger's call, the rest of the pack is fine */
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		chg = s2mpu06_fg_charger();
		if (chg) {
			if (!power_supply_get_property(chg, POWER_SUPPLY_PROP_HEALTH, &chg_val) &&
			    (chg_val.intval == POWER_SUPPLY_HEALTH_OVERHEAT ||
			     chg_val.intval == POWER_SUPPLY_HEALTH_COLD))
				val->intval = chg_val.intval;
			power_supply_put(chg);
		}
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = s2mpu06_fg_read_mv(fg, S2MPU06_FG_RVBAT, &mv);
		val->intval = mv * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		/* the vendor's smoothing: each of five samples counts half of the running value */
		for (i = 0, sum = 0; i < 5 && !ret; i++) {
			ret = s2mpu06_fg_read_mv(fg, S2MPU06_FG_RVBAT, &mv);
			sum = i ? mv / 2 + sum / 2 : mv;
		}
		val->intval = sum * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		ret = s2mpu06_fg_read_mv(fg, S2MPU06_FG_ROCV, &mv);
		val->intval = mv * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = s2mpu06_fg_read_current_ma(fg, &mv);
		val->intval = mv * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		/* the mean of the last polls, 10 s apart */
		if (!fg->cur_cnt) {
			ret = -ENODATA;
			break;
		}
		for (i = 0, sum = 0, cnt = fg->cur_cnt; i < cnt; i++)
			sum += fg->cur_hist[i];
		val->intval = sum / cnt * 1000;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (!fg->adc || !fg->temp_valid)
			ret = -ENODATA;
		else
			val->intval = fg->temp;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (!fg->capacity_valid)
			ret = -ENODATA;
		else
			val->intval = fg->capacity;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = fg->level;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN:
		val->intval = fg->fuel_alert_soc;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		if (fg->cycle_count < 0)
			ret = -ENODATA;
		else
			val->intval = fg->cycle_count;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		/* an estimate: the design capacity times the reported state of charge */
		if (!fg->info || fg->info->charge_full_design_uah < 0 || !fg->capacity_valid)
			ret = -ENODATA;
		else
			val->intval = div_u64((u64)fg->info->charge_full_design_uah * fg->capacity, 100);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		if (!fg->info || fg->info->charge_full_design_uah < 0)
			ret = -ENODATA;
		else
			val->intval = fg->info->charge_full_design_uah;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		if (!fg->info || fg->info->voltage_max_design_uv < 0)
			ret = -ENODATA;
		else
			val->intval = fg->info->voltage_max_design_uv;
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&fg->lock);

	return ret;
}

static int s2mpu06_fg_set_property(struct power_supply *psy, enum power_supply_property psp,
				   const union power_supply_propval *val)
{
	struct s2mpu06_fuelgauge *fg = power_supply_get_drvdata(psy);
	int ret = 0;

	mutex_lock(&fg->lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN:
		if (val->intval < 0 || val->intval > 15) {
			ret = -EINVAL;
			break;
		}
		fg->fuel_alert_soc = val->intval;
		ret = s2mpu06_fg_program_alert(fg);
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		if (val->intval < 0) {
			ret = -EINVAL;
			break;
		}
		/* the cycle count has to be kept by userspace and written at every boot */
		fg->cycle_count = val->intval;
		s2mpu06_fg_select_step(fg);
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&fg->lock);

	return ret;
}

static int s2mpu06_fg_property_is_writeable(struct power_supply *psy, enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN || psp == POWER_SUPPLY_PROP_CYCLE_COUNT;
}

static const enum power_supply_property s2mpu06_fg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
};

static const struct power_supply_desc s2mpu06_fg_desc = {
	.name = "s2mpu06-fuelgauge",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = s2mpu06_fg_props,
	.num_properties = ARRAY_SIZE(s2mpu06_fg_props),
	.get_property = s2mpu06_fg_get_property,
	.set_property = s2mpu06_fg_set_property,
	.property_is_writeable = s2mpu06_fg_property_is_writeable,
	.external_power_changed = s2mpu06_fg_external_power_changed,
};

/* ----------------------------------------------------------------- probe */

static void s2mpu06_fg_put_info(void *data)
{
	struct s2mpu06_fuelgauge *fg = data;

	if (fg->info)
		power_supply_put_battery_info(fg->psy, fg->info);
}

static void s2mpu06_fg_stop(void *data)
{
	struct s2mpu06_fuelgauge *fg = data;

	blocking_notifier_chain_unregister(&fg->s2m->fg_nh, &fg->alert_nb);
	cancel_delayed_work_sync(&fg->work);
}

/* optional: the NTC thermistor is read by the AP's ADC and converted with a table */
static int s2mpu06_fg_init_temp(struct s2mpu06_fuelgauge *fg)
{
	struct device *dev = fg->dev;
	int n, ret;

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
	/* have a temperature before anything asks for it */
	s2mpu06_fg_update_temp(fg);
	return 0;
}

/* optional: the cell data of the ageing steps */
static int s2mpu06_fg_init_aging(struct s2mpu06_fuelgauge *fg)
{
	struct device *dev = fg->dev;
	struct device_node *np = dev->of_node;
	int n, ret;

	n = of_property_count_u32_elems(np, "samsung,age-cycles");
	if (n <= 0)
		return 0;

	if (n != of_property_count_u32_elems(np, "samsung,age-float-microvolt") ||
	    n != of_property_count_u32_elems(np, "samsung,age-raw100-millivolt") ||
	    n * S2MPU06_FG_CELL_CHAR0_LEN != of_property_count_u8_elems(np, "samsung,cell-character0") ||
	    n * S2MPU06_FG_CELL_CHAR1_LEN != of_property_count_u8_elems(np, "samsung,cell-character1")) {
		dev_warn(dev, "inconsistent ageing tables, ageing disabled\n");
		return 0;
	}

	fg->age_cycle = devm_kcalloc(dev, n, sizeof(u32), GFP_KERNEL);
	fg->age_float_uv = devm_kcalloc(dev, n, sizeof(u32), GFP_KERNEL);
	fg->age_raw100_mv = devm_kcalloc(dev, n, sizeof(u32), GFP_KERNEL);
	fg->cell_char0 = devm_kcalloc(dev, n, S2MPU06_FG_CELL_CHAR0_LEN, GFP_KERNEL);
	fg->cell_char1 = devm_kcalloc(dev, n, S2MPU06_FG_CELL_CHAR1_LEN, GFP_KERNEL);
	if (!fg->age_cycle || !fg->age_float_uv || !fg->age_raw100_mv ||
	    !fg->cell_char0 || !fg->cell_char1)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, "samsung,age-cycles", fg->age_cycle, n);
	if (!ret)
		ret = of_property_read_u32_array(np, "samsung,age-float-microvolt", fg->age_float_uv, n);
	if (!ret)
		ret = of_property_read_u32_array(np, "samsung,age-raw100-millivolt",
						 fg->age_raw100_mv, n);
	if (!ret)
		ret = of_property_read_u8_array(np, "samsung,cell-character0", fg->cell_char0,
						n * S2MPU06_FG_CELL_CHAR0_LEN);
	if (!ret)
		ret = of_property_read_u8_array(np, "samsung,cell-character1", fg->cell_char1,
						n * S2MPU06_FG_CELL_CHAR1_LEN);
	if (ret)
		return ret;

	fg->num_steps = n;
	return 0;
}

static int s2mpu06_fg_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct power_supply_config cfg = { };
	struct s2mpu06_fuelgauge *fg;
	unsigned int ctrl;
	int ret, raw;

	fg = devm_kzalloc(dev, sizeof(*fg), GFP_KERNEL);
	if (!fg)
		return -ENOMEM;

	fg->dev = dev;
	fg->s2m = dev_get_drvdata(dev->parent);
	fg->cycle_count = -1;
	fg->want_step = -1;
	fg->capacity_max_cfg = S2MPU06_FG_CAPACITY_FULL;
	fg->capacity_margin = 0;
	fg->fuel_alert_soc = 1;
	ret = devm_mutex_init(dev, &fg->lock);
	if (ret)
		return ret;
	INIT_DELAYED_WORK(&fg->work, s2mpu06_fg_work);

	/* the battery description hangs off the PMIC's node */
	device_set_of_node_from_dev(dev, dev->parent);
	cfg.drv_data = fg;
	cfg.fwnode = dev_fwnode(dev);

	device_property_read_u32(dev, "samsung,capacity-max", &fg->capacity_max_cfg);
	device_property_read_u32(dev, "samsung,capacity-max-margin", &fg->capacity_margin);
	device_property_read_u32(dev, "samsung,fuel-alert-soc", &fg->fuel_alert_soc);
	fg->fuel_alert_soc = clamp(fg->fuel_alert_soc, 0, 15);
	fg->capacity_max = fg->capacity_max_cfg;

	ret = s2mpu06_fg_init_temp(fg);
	if (ret)
		return dev_err_probe(dev, ret, "cannot set up the temperature sensing\n");
	ret = s2mpu06_fg_init_aging(fg);
	if (ret)
		return dev_err_probe(dev, ret, "cannot read the ageing tables\n");

	/* the data version in the gauge has to survive a reset */
	if (!regmap_read(fg->s2m->fg, S2MPU06_FG_CTRL, &ctrl))
		fg->data_ver = ctrl & S2MPU06_FG_CTRL_DATA_VER;

	s2mpu06_fg_program_alert(fg);
	fg->alert_nb.notifier_call = s2mpu06_fg_alert_event;

	fg->psy = devm_power_supply_register(dev, &s2mpu06_fg_desc, &cfg);
	if (IS_ERR(fg->psy))
		return dev_err_probe(dev, PTR_ERR(fg->psy), "cannot register the fuel gauge\n");

	if (power_supply_get_battery_info(fg->psy, &fg->info))
		fg->info = NULL;
	else
		devm_add_action_or_reset(dev, s2mpu06_fg_put_info, fg);

	/* a gauge that reads above the configured 100 % starts with a stretched scale */
	if (!s2mpu06_fg_read_rawsoc(fg, &raw) && raw / 10 > (int)fg->capacity_max_cfg)
		s2mpu06_fg_dynamic_scale(fg, 100);

	ret = blocking_notifier_chain_register(&fg->s2m->fg_nh, &fg->alert_nb);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, s2mpu06_fg_stop, fg);
	if (ret)
		return ret;

	schedule_delayed_work(&fg->work, 0);

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
