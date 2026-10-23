// SPDX-License-Identifier: GPL-2.0
/*
 * Charger of the Samsung S2MPU06 PMIC
 *
 * The charger IC runs the constant current and constant voltage phases on its
 * own; ending the charge is left to this driver, like the vendor's battery
 * driver did it. It reports the charger's state, exposes the current and
 * voltage limits, keeps the watchdog fed, restricts charging when the battery
 * is too hot or too cold and decides when the battery is full, when to
 * recharge it and when charging has taken too long.
 *
 * Register layout and temperature profile from the downstream s2mpu06_charger
 * driver and sec_battery configuration:
 * Copyright (C) 2015 Samsung Electronics
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/extcon.h>
#include <linux/math.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/mfd/samsung/s2mpu06.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

/* PMIC block */
#define S2MPU06_PMIC_STATUS1		0x06
#define S2MPU06_PMIC_STATUS1_ACOK	BIT(2)

/* charger block */
#define S2MPU06_CHG_STATUS1		0x09
#define S2MPU06_CHG_STATUS1_CHARGING	BIT(5)
#define S2MPU06_CHG_STATUS1_CC_CV	GENMASK(6, 5)
#define S2MPU06_CHG_STATUS2		0x0a
#define S2MPU06_CHG_STATUS2_DONE	BIT(5)
#define S2MPU06_CHG_STATUS2_VINOVP	BIT(6)
#define S2MPU06_CHG_CTRL1		0x0d	/* input current limit, 10 mA/step */
#define S2MPU06_CHG_CTRL2		0x0e	/* float voltage */
#define S2MPU06_CHG_CTRL3		0x0f
#define S2MPU06_CHG_CTRL3_MODE		GENMASK(5, 4)
#define S2MPU06_CHG_CTRL3_TE		BIT(6)	/* the IC ends the charge on its own */
#define S2MPU06_CHG_MODE_OFF		0
#define S2MPU06_CHG_MODE_ON		1
#define S2MPU06_CHG_CTRL4		0x10
#define S2MPU06_CHG_CTRL4_TIMER		BIT(0)	/* the IC's own charging timer */
#define S2MPU06_CHG_CTRL9		0x15	/* top-off current */
#define S2MPU06_CHG_CTRL9_MASK		GENMASK(6, 0)
#define S2MPU06_CHG_CTRL10		0x16	/* fast charge current */
#define S2MPU06_CHG_CTRL10_MASK		GENMASK(6, 0)
#define S2MPU06_CHG_CTRL12		0x18	/* watchdog */
#define S2MPU06_CHG_CTRL12_WDT_EN	GENMASK(3, 2)
#define S2MPU06_CHG_CTRL12_WDT_KICK	GENMASK(1, 0)
#define S2MPU06_CHG_CTRL12_NO_RECHG	BIT(7)	/* no automatic recharge by the IC */

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

#define S2MPU06_TOPOFF_MIN_UA		100000
#define S2MPU06_TOPOFF_STEP_UA		10000
#define S2MPU06_TOPOFF_BASE_SEL		0x02
#define S2MPU06_TOPOFF_MAX_SEL		0x2a

#define S2MPU06_POLL_PERIOD		msecs_to_jiffies(10000)

/*
 * End of charge, from the J4's sec_battery configuration. The IC flags top-off
 * (STATUS2 DONE) when the charge current falls below the top-off current.
 * The first time it does, with the battery at least at 93 % and 4.25 V, the
 * battery is reported full and the top-off current is lowered; the second time
 * charging is switched off. Top-off only counts when it is flagged for 30 s:
 * on a weak input a load spike also drops the charge current below it.
 * Charging is switched on again when the battery's open circuit voltage has
 * sagged below 4.25 V; the loaded voltage drops that far under a heavy load
 * right away. Both thresholds follow the float voltage, which the
 * fuel gauge lowers as the battery ages (by 2 % per 20 mV, as in the vendor's
 * age table). In the warm and cool bands charging ends at the first top-off
 * and resumes below 4.0 V.
 *
 * A safety timer of 4 h 50 min (1 h 30 min for a recharge) at 1034 mA runs
 * slower in proportion to a smaller charge current and starts over when the
 * battery has been discharging for a while.
 */
#define S2MPU06_EOC_1ST_UA		300000
#define S2MPU06_EOC_2ND_UA		150000
#define S2MPU06_EOC_BAND_UA		150000
#define S2MPU06_EOC_FULL_SOC		93	/* at the 4.35 V float voltage */
#define S2MPU06_EOC_FLOAT_UV		4350000
#define S2MPU06_EOC_MARGIN_UV		100000	/* full and recharge below the float voltage */
#define S2MPU06_EOC_BAND_RECHG_UV	4000000
#define S2MPU06_EOC_RECHG_COUNT		2
#define S2MPU06_EOC_DONE_COUNT		3
#define S2MPU06_EOC_TIME_MS		(17400 * MSEC_PER_SEC)
#define S2MPU06_EOC_RECHG_TIME_MS	(5400 * MSEC_PER_SEC)
#define S2MPU06_EOC_STD_UA		1034000
#define S2MPU06_EOC_VBUS_UV		5000000
#define S2MPU06_EOC_DISCHG_COUNT	5

enum s2mpu06_eoc_phase {
	S2MPU06_EOC_1ST,	/* charging down to the first top-off current */
	S2MPU06_EOC_2ND,	/* reported full, topping off down to the second one */
	S2MPU06_EOC_FULL,	/* done: off until the battery sags */
	S2MPU06_EOC_EXPIRED,	/* the safety timer ran out: off until the cable is replugged */
};

/*
 * Temperature profile of the J4's battery in 0.1 degC, from the downstream
 * configuration: charging stops above 50 degC and below 0 degC, and in the
 * warm (above 41 degC) and cool (below 15 degC) bands the current and the
 * float voltage are reduced.
 */
struct s2mpu06_temp_band {
	int enter;	/* the band is entered at this temperature ... */
	int leave;	/* ... and left again past this one (hysteresis) */
	int ichg_ua;	/* current cap, 0 = none */
	int vfloat_uv;	/* float voltage cap, 0 = none */
};

static const struct s2mpu06_temp_band s2mpu06_hot_block  = { 500, 450, 0, 0 };
static const struct s2mpu06_temp_band s2mpu06_cold_block = { 0, 50, 0, 0 };
static const struct s2mpu06_temp_band s2mpu06_warm       = { 410, 390, 1120000, 4150000 };
static const struct s2mpu06_temp_band s2mpu06_cool       = { 150, 200, 740000, 4150000 };

enum s2mpu06_temp_state {
	S2MPU06_TEMP_NORMAL,
	S2MPU06_TEMP_COOL,
	S2MPU06_TEMP_WARM,
	S2MPU06_TEMP_COLD_BLOCK,
	S2MPU06_TEMP_HOT_BLOCK,
};

/*
 * Input and charge current per cable type, from the J4's cable table: a PC port
 * gives 500 mA, a wall charger (the vendor's "TA") 1 A in and 1.2 A to the cell.
 */
#define S2MPU06_USB_IIN_UA		500000
#define S2MPU06_USB_ICHG_UA		500000
#define S2MPU06_DCP_IIN_UA		1000000
#define S2MPU06_DCP_ICHG_UA		1200000
#define S2MPU06_CDP_IIN_UA		1500000
#define S2MPU06_CDP_ICHG_UA		1500000

struct s2mpu06_charger {
	struct device *dev;
	struct s2mpu06 *s2m;
	struct power_supply *psy;
	struct delayed_work work;
	struct mutex lock;	/* requested limits and the thermal state */

	struct extcon_dev *edev;
	struct notifier_block cable_nb;
	struct work_struct cable_work;

	/* what userspace (or the bootloader) asked for, in register units */
	unsigned int req_ichg_sel;
	unsigned int req_iin_sel;
	unsigned int req_vfloat_sel;

	enum s2mpu06_temp_state state;
	unsigned int mode_before_block;

	/* end of charge */
	enum s2mpu06_eoc_phase phase;
	bool was_online;
	bool recharging;	/* the battery stays reported full */
	bool full_in_band;	/* stopped at the reduced float voltage, not really full */
	s64 budget_ms;		/* safety timer left, at the standard current */
	ktime_t tick;
	unsigned int rechg_cnt;
	unsigned int done_cnt;
	unsigned int dischg_cnt;

	bool inhibit;	/* userspace asked for no charging */
	bool blocked;	/* the charger is switched off for any reason */
};

static unsigned int s2mpu06_ichg_to_sel(int ua)
{
	ua = clamp(ua, S2MPU06_ICHG_MIN_UA,
		   S2MPU06_ICHG_MIN_UA +
		   (S2MPU06_ICHG_MAX_SEL - S2MPU06_ICHG_BASE_SEL) * S2MPU06_ICHG_STEP_UA);

	return S2MPU06_ICHG_BASE_SEL + (ua - S2MPU06_ICHG_MIN_UA) / S2MPU06_ICHG_STEP_UA;
}

static int s2mpu06_sel_to_ichg(unsigned int sel)
{
	sel = clamp(sel, (unsigned int)S2MPU06_ICHG_BASE_SEL, (unsigned int)S2MPU06_ICHG_MAX_SEL);

	return S2MPU06_ICHG_MIN_UA + (sel - S2MPU06_ICHG_BASE_SEL) * S2MPU06_ICHG_STEP_UA;
}

static unsigned int s2mpu06_vfloat_to_sel(int uv)
{
	uv = clamp(uv, S2MPU06_VFLOAT_MIN_UV,
		   S2MPU06_VFLOAT_MIN_UV +
		   (S2MPU06_VFLOAT_MAX_SEL - S2MPU06_VFLOAT_BASE_SEL) * S2MPU06_VFLOAT_STEP_UV);

	return S2MPU06_VFLOAT_BASE_SEL + (uv - S2MPU06_VFLOAT_MIN_UV) / S2MPU06_VFLOAT_STEP_UV;
}

static int s2mpu06_sel_to_vfloat(unsigned int sel)
{
	sel = clamp(sel, (unsigned int)S2MPU06_VFLOAT_BASE_SEL, (unsigned int)S2MPU06_VFLOAT_MAX_SEL);

	return S2MPU06_VFLOAT_MIN_UV + (sel - S2MPU06_VFLOAT_BASE_SEL) * S2MPU06_VFLOAT_STEP_UV;
}

static unsigned int s2mpu06_topoff_to_sel(int ua)
{
	ua = clamp(ua, S2MPU06_TOPOFF_MIN_UA,
		   S2MPU06_TOPOFF_MIN_UA +
		   (S2MPU06_TOPOFF_MAX_SEL - S2MPU06_TOPOFF_BASE_SEL) * S2MPU06_TOPOFF_STEP_UA);

	return S2MPU06_TOPOFF_BASE_SEL + (ua - S2MPU06_TOPOFF_MIN_UA) / S2MPU06_TOPOFF_STEP_UA;
}

static bool s2mpu06_charger_in_band(struct s2mpu06_charger *chg)
{
	return chg->state == S2MPU06_TEMP_COOL || chg->state == S2MPU06_TEMP_WARM;
}

/* write the requested limits, lowered to what the temperature band allows */
static int s2mpu06_charger_apply_limits(struct s2mpu06_charger *chg)
{
	unsigned int ichg = chg->req_ichg_sel, vfloat = chg->req_vfloat_sel;
	const struct s2mpu06_temp_band *band = NULL;
	int ret;

	if (chg->state == S2MPU06_TEMP_COOL)
		band = &s2mpu06_cool;
	else if (chg->state == S2MPU06_TEMP_WARM)
		band = &s2mpu06_warm;

	if (band) {
		ichg = min(ichg, s2mpu06_ichg_to_sel(band->ichg_ua));
		vfloat = min(vfloat, s2mpu06_vfloat_to_sel(band->vfloat_uv));
	}

	ret = regmap_update_bits(chg->s2m->chg, S2MPU06_CHG_CTRL10, S2MPU06_CHG_CTRL10_MASK, ichg);
	if (ret)
		return ret;
	ret = regmap_write(chg->s2m->chg, S2MPU06_CHG_CTRL2, vfloat);
	if (ret)
		return ret;

	ret = regmap_update_bits(chg->s2m->chg, S2MPU06_CHG_CTRL9, S2MPU06_CHG_CTRL9_MASK,
				 s2mpu06_topoff_to_sel(band ? S2MPU06_EOC_BAND_UA :
						       chg->phase == S2MPU06_EOC_1ST ?
						       S2MPU06_EOC_1ST_UA : S2MPU06_EOC_2ND_UA));
	if (ret)
		return ret;

	return regmap_write(chg->s2m->chg, S2MPU06_CHG_CTRL1, chg->req_iin_sel);
}

/*
 * charging is off while the battery is too hot or too cold, when it is full,
 * when the safety timer ran out or when userspace inhibits it
 */
static int s2mpu06_charger_update_block(struct s2mpu06_charger *chg)
{
	bool block = chg->inhibit || chg->state >= S2MPU06_TEMP_COLD_BLOCK ||
		     chg->phase >= S2MPU06_EOC_FULL;
	unsigned int mode;
	int ret;

	if (block == chg->blocked)
		return 0;

	ret = regmap_read(chg->s2m->chg, S2MPU06_CHG_CTRL3, &mode);
	if (ret)
		return ret;
	mode = FIELD_GET(S2MPU06_CHG_CTRL3_MODE, mode);

	if (block) {
		/* remember what to switch back to */
		if (mode != S2MPU06_CHG_MODE_OFF)
			chg->mode_before_block = mode;
		mode = S2MPU06_CHG_MODE_OFF;
	} else {
		mode = chg->mode_before_block;
	}

	ret = regmap_update_bits(chg->s2m->chg, S2MPU06_CHG_CTRL3, S2MPU06_CHG_CTRL3_MODE,
				 FIELD_PREP(S2MPU06_CHG_CTRL3_MODE, mode));
	if (!ret)
		chg->blocked = block;

	return ret;
}

static enum s2mpu06_temp_state s2mpu06_charger_next_state(enum s2mpu06_temp_state cur, int t)
{
	/* the blocking bands win over the reduced ones; each has its own hysteresis */
	if (t >= s2mpu06_hot_block.enter ||
	    (cur == S2MPU06_TEMP_HOT_BLOCK && t > s2mpu06_hot_block.leave))
		return S2MPU06_TEMP_HOT_BLOCK;
	if (t <= s2mpu06_cold_block.enter ||
	    (cur == S2MPU06_TEMP_COLD_BLOCK && t < s2mpu06_cold_block.leave))
		return S2MPU06_TEMP_COLD_BLOCK;
	if (t >= s2mpu06_warm.enter ||
	    (cur == S2MPU06_TEMP_WARM && t >= s2mpu06_warm.leave))
		return S2MPU06_TEMP_WARM;
	if (t <= s2mpu06_cool.enter ||
	    (cur == S2MPU06_TEMP_COOL && t < s2mpu06_cool.leave))
		return S2MPU06_TEMP_COOL;

	return S2MPU06_TEMP_NORMAL;
}

/*
 * The battery's temperature, voltage and charge come from the fuel gauge;
 * without them nothing changes. Not to be called with the lock held: the fuel
 * gauge reads the charger with its own lock held.
 */
static int s2mpu06_charger_get_fg(enum power_supply_property psp, int *out)
{
	struct power_supply *fg = power_supply_get_by_name("s2mpu06-fuelgauge");
	union power_supply_propval val;
	int ret;

	if (!fg)
		return -ENODEV;

	ret = power_supply_get_property(fg, psp, &val);
	power_supply_put(fg);
	if (ret)
		return ret;

	*out = val.intval;
	return 0;
}

static void s2mpu06_charger_thermal(struct s2mpu06_charger *chg)
{
	enum s2mpu06_temp_state old, next;
	int temp, ret = 0;

	if (s2mpu06_charger_get_fg(POWER_SUPPLY_PROP_TEMP, &temp))
		return;

	mutex_lock(&chg->lock);
	old = chg->state;
	next = s2mpu06_charger_next_state(old, temp);
	if (next == old)
		goto out;

	dev_info(chg->dev, "battery at %d.%d degC, charging state %d -> %d\n",
		 temp / 10, abs(temp % 10), old, next);

	/* the caps of the new band go in before charging can resume */
	chg->state = next;
	ret = s2mpu06_charger_apply_limits(chg);

	if (!ret)
		ret = s2mpu06_charger_update_block(chg);

	/* try again at the next poll */
	if (ret)
		chg->state = old;
out:
	mutex_unlock(&chg->lock);
	if (!ret && next != old)
		power_supply_changed(chg->psy);
}

struct s2mpu06_battery {
	int soc;
	int vbat_uv;
	int ocv_uv;
	int ibat_avg_ua;	/* negative when discharging */
	bool valid;
};

static void s2mpu06_charger_read_battery(struct s2mpu06_battery *bat)
{
	bat->valid = !s2mpu06_charger_get_fg(POWER_SUPPLY_PROP_CAPACITY, &bat->soc) &&
		     !s2mpu06_charger_get_fg(POWER_SUPPLY_PROP_VOLTAGE_NOW, &bat->vbat_uv) &&
		     !s2mpu06_charger_get_fg(POWER_SUPPLY_PROP_VOLTAGE_OCV, &bat->ocv_uv) &&
		     !s2mpu06_charger_get_fg(POWER_SUPPLY_PROP_CURRENT_AVG, &bat->ibat_avg_ua);
}

static void s2mpu06_charger_eoc_reset(struct s2mpu06_charger *chg)
{
	chg->phase = S2MPU06_EOC_1ST;
	chg->recharging = false;
	chg->full_in_band = false;
	chg->budget_ms = S2MPU06_EOC_TIME_MS;
	chg->tick = ktime_get_boottime();
	chg->rechg_cnt = 0;
	chg->done_cnt = 0;
	chg->dischg_cnt = 0;
}

/* what the cell gets: the charge current, unless the input limit allows less */
static int s2mpu06_charger_cell_ua(struct s2mpu06_charger *chg)
{
	unsigned int ichg, iin, vfloat;
	u64 from_input;

	if (regmap_read(chg->s2m->chg, S2MPU06_CHG_CTRL10, &ichg) ||
	    regmap_read(chg->s2m->chg, S2MPU06_CHG_CTRL1, &iin) ||
	    regmap_read(chg->s2m->chg, S2MPU06_CHG_CTRL2, &vfloat))
		return S2MPU06_EOC_STD_UA;

	/* the vendor's estimate: 90 % of the input power at the float voltage */
	from_input = (u64)min_t(unsigned int, iin, S2MPU06_IIN_MAX_SEL) * S2MPU06_IIN_STEP_UA *
		     (S2MPU06_EOC_VBUS_UV / 1000) * 9;
	from_input = div_u64(from_input, s2mpu06_sel_to_vfloat(vfloat) / 1000 * 10);

	return min_t(u64, s2mpu06_sel_to_ichg(ichg & S2MPU06_CHG_CTRL10_MASK), from_input);
}

/* runs the safety timer down while charging; true when it ran out */
static bool s2mpu06_charger_eoc_timer(struct s2mpu06_charger *chg, bool charging)
{
	ktime_t now = ktime_get_boottime();
	s64 ms = ktime_ms_delta(now, chg->tick);

	chg->tick = now;
	if (!charging)
		return false;

	chg->budget_ms -= div_s64(ms * s2mpu06_charger_cell_ua(chg), S2MPU06_EOC_STD_UA);

	return chg->budget_ms <= 0;
}

/* one step of the end of charge logic, with the lock held; true when the state changed */
static bool s2mpu06_charger_eoc(struct s2mpu06_charger *chg, int online,
				const struct s2mpu06_battery *bat)
{
	enum s2mpu06_eoc_phase old = chg->phase;
	bool was_recharging = chg->recharging, charging, band;
	int vfloat, full_uv, full_soc, rechg_uv;
	unsigned int s2;

	if (!online) {
		if (chg->was_online)
			s2mpu06_charger_eoc_reset(chg);
		chg->was_online = false;
		goto out;
	}
	if (!chg->was_online) {
		s2mpu06_charger_eoc_reset(chg);
		chg->was_online = true;
	}

	band = s2mpu06_charger_in_band(chg);
	vfloat = s2mpu06_sel_to_vfloat(chg->req_vfloat_sel);
	full_uv = vfloat - S2MPU06_EOC_MARGIN_UV;
	full_soc = S2MPU06_EOC_FULL_SOC - (S2MPU06_EOC_FLOAT_UV - vfloat) / 10000;
	rechg_uv = band ? S2MPU06_EOC_BAND_RECHG_UV : full_uv;

	switch (chg->phase) {
	case S2MPU06_EOC_1ST:
	case S2MPU06_EOC_2ND:
		/* only the temperature or userspace can block charging in these phases */
		charging = !chg->blocked;
		if (s2mpu06_charger_eoc_timer(chg, charging)) {
			dev_warn(chg->dev, "charging took too long, off until the cable is replugged\n");
			chg->phase = S2MPU06_EOC_EXPIRED;
			chg->recharging = false;
			break;
		}
		if (!charging || !bat->valid) {
			chg->done_cnt = 0;
			break;
		}

		/* a battery that is drained while charging gets the full time again */
		if (bat->ibat_avg_ua >= 0) {
			chg->dischg_cnt = 0;
		} else if (++chg->dischg_cnt >= S2MPU06_EOC_DISCHG_COUNT) {
			chg->dischg_cnt = 0;
			chg->budget_ms = S2MPU06_EOC_TIME_MS;
		}

		if (regmap_read(chg->s2m->chg, S2MPU06_CHG_STATUS2, &s2) ||
		    !(s2 & S2MPU06_CHG_STATUS2_DONE)) {
			chg->done_cnt = 0;
			break;
		}
		if (++chg->done_cnt < S2MPU06_EOC_DONE_COUNT)
			break;
		chg->done_cnt = 0;

		if (band) {
			dev_info(chg->dev, "charged to the reduced float voltage, off\n");
			chg->phase = S2MPU06_EOC_FULL;
			chg->full_in_band = true;
			chg->recharging = false;
			chg->budget_ms = S2MPU06_EOC_TIME_MS;
		} else if (bat->soc < full_soc || bat->vbat_uv < full_uv) {
			break;
		} else if (chg->phase == S2MPU06_EOC_1ST) {
			dev_info(chg->dev, "battery full at %d mV, topping off\n", bat->vbat_uv / 1000);
			chg->phase = S2MPU06_EOC_2ND;
		} else {
			dev_info(chg->dev, "battery charged at %d mV, off\n", bat->vbat_uv / 1000);
			chg->phase = S2MPU06_EOC_FULL;
			chg->full_in_band = false;
		}
		break;
	case S2MPU06_EOC_FULL:
		chg->tick = ktime_get_boottime();
		if (!bat->valid || bat->ocv_uv > rechg_uv) {
			chg->rechg_cnt = 0;
			break;
		}
		if (++chg->rechg_cnt < S2MPU06_EOC_RECHG_COUNT)
			break;

		dev_info(chg->dev, "battery at %d mV (open circuit), recharging\n", bat->ocv_uv / 1000);
		chg->recharging = !chg->full_in_band;
		chg->full_in_band = false;
		chg->phase = S2MPU06_EOC_1ST;
		chg->budget_ms = S2MPU06_EOC_RECHG_TIME_MS;
		chg->rechg_cnt = 0;
		chg->dischg_cnt = 0;
		break;
	case S2MPU06_EOC_EXPIRED:
		break;
	}

out:
	/* the top-off current follows the phase; a failed write is retried at the next poll */
	if (chg->phase != old)
		s2mpu06_charger_apply_limits(chg);
	s2mpu06_charger_update_block(chg);

	return chg->phase != old || chg->recharging != was_recharging;
}

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

	/* top-off alone is not full: that is decided by the end of charge logic */
	if (!online)
		*status = POWER_SUPPLY_STATUS_DISCHARGING;
	else if (chg->phase == S2MPU06_EOC_EXPIRED ||
		 (chg->phase == S2MPU06_EOC_FULL && chg->full_in_band))
		*status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	else if (chg->phase >= S2MPU06_EOC_2ND || chg->recharging)
		*status = POWER_SUPPLY_STATUS_FULL;
	else if (chg->blocked)
		*status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	else if (s1 & S2MPU06_CHG_STATUS1_CHARGING || s2 & S2MPU06_CHG_STATUS2_DONE)
		*status = POWER_SUPPLY_STATUS_CHARGING;
	else
		*status = POWER_SUPPLY_STATUS_NOT_CHARGING;

	return 0;
}

static int s2mpu06_charger_health(struct s2mpu06_charger *chg, int *health)
{
	unsigned int reg;
	int ret;

	ret = regmap_read(chg->s2m->chg, S2MPU06_CHG_STATUS2, &reg);
	if (ret)
		return ret;

	if (reg & S2MPU06_CHG_STATUS2_VINOVP)
		*health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	else if (chg->state == S2MPU06_TEMP_HOT_BLOCK)
		*health = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (chg->state == S2MPU06_TEMP_COLD_BLOCK)
		*health = POWER_SUPPLY_HEALTH_COLD;
	else if (chg->phase == S2MPU06_EOC_EXPIRED)
		*health = POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE;
	else
		*health = POWER_SUPPLY_HEALTH_GOOD;

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
		return s2mpu06_charger_health(chg, &val->intval);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = regmap_read(chg->s2m->chg, S2MPU06_CHG_CTRL10, &reg);
		if (ret)
			return ret;
		val->intval = s2mpu06_sel_to_ichg(reg & S2MPU06_CHG_CTRL10_MASK);
		return 0;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = s2mpu06_sel_to_ichg(S2MPU06_ICHG_MAX_SEL);
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
		val->intval = s2mpu06_sel_to_vfloat(reg);
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		val->intval = chg->inhibit ? POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE :
					     POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO;
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
	int ret;

	mutex_lock(&chg->lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		chg->req_ichg_sel = s2mpu06_ichg_to_sel(val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		chg->req_iin_sel = clamp(val->intval / S2MPU06_IIN_STEP_UA,
					 S2MPU06_IIN_MIN_SEL, S2MPU06_IIN_MAX_SEL);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		chg->req_vfloat_sel = s2mpu06_vfloat_to_sel(val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		if (val->intval != POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO &&
		    val->intval != POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE) {
			mutex_unlock(&chg->lock);
			return -EINVAL;
		}
		chg->inhibit = val->intval == POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE;
		ret = s2mpu06_charger_update_block(chg);
		mutex_unlock(&chg->lock);
		if (!ret)
			power_supply_changed(psy);
		return ret;
	default:
		mutex_unlock(&chg->lock);
		return -EINVAL;
	}

	/* a thermal cap still applies to what was just asked for */
	ret = s2mpu06_charger_apply_limits(chg);
	mutex_unlock(&chg->lock);

	return ret;
}

static int s2mpu06_charger_property_is_writeable(struct power_supply *psy,
						 enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
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
	POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
};

static char *s2mpu06_charger_supplicants[] = { "s2mpu06-fuelgauge" };

static const struct power_supply_desc s2mpu06_charger_desc = {
	.name = "s2mpu06-charger",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.charge_behaviours = BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO) |
			     BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE),
	.properties = s2mpu06_charger_props,
	.num_properties = ARRAY_SIZE(s2mpu06_charger_props),
	.get_property = s2mpu06_charger_get_property,
	.set_property = s2mpu06_charger_set_property,
	.property_is_writeable = s2mpu06_charger_property_is_writeable,
};

/*
 * feeds the watchdog (the IC drops to its defaults otherwise), checks the
 * temperature and ends the charge
 */
static void s2mpu06_charger_work(struct work_struct *work)
{
	struct s2mpu06_charger *chg = container_of(work, struct s2mpu06_charger, work.work);
	struct s2mpu06_battery bat;
	unsigned int ctrl12;
	bool changed;
	int online;

	if (!regmap_read(chg->s2m->chg, S2MPU06_CHG_CTRL12, &ctrl12) &&
	    (ctrl12 & S2MPU06_CHG_CTRL12_WDT_EN)) {
		regmap_update_bits(chg->s2m->chg, S2MPU06_CHG_CTRL12,
				   S2MPU06_CHG_CTRL12_WDT_KICK, 0x1);
		regmap_update_bits(chg->s2m->chg, S2MPU06_CHG_CTRL12,
				   S2MPU06_CHG_CTRL12_WDT_KICK, 0x0);
	}

	s2mpu06_charger_thermal(chg);

	s2mpu06_charger_read_battery(&bat);
	if (!s2mpu06_charger_online(chg, &online)) {
		mutex_lock(&chg->lock);
		changed = s2mpu06_charger_eoc(chg, online, &bat);
		mutex_unlock(&chg->lock);
		if (changed)
			power_supply_changed(chg->psy);
	}

	schedule_delayed_work(&chg->work, S2MPU06_POLL_PERIOD);
}

static irqreturn_t s2mpu06_charger_acok_irq(int irq, void *data)
{
	struct s2mpu06_charger *chg = data;

	power_supply_changed(chg->psy);
	/* a cable change starts the end of charge logic over */
	mod_delayed_work(system_percpu_wq, &chg->work, 0);

	return IRQ_HANDLED;
}

/* the cable the MUIC reports decides how much current may be drawn and given to the cell */
static void s2mpu06_charger_cable_work(struct work_struct *work)
{
	struct s2mpu06_charger *chg = container_of(work, struct s2mpu06_charger, cable_work);
	int iin = S2MPU06_USB_IIN_UA, ichg = S2MPU06_USB_ICHG_UA;

	if (extcon_get_state(chg->edev, EXTCON_CHG_USB_CDP) > 0) {
		iin = S2MPU06_CDP_IIN_UA;
		ichg = S2MPU06_CDP_ICHG_UA;
	} else if (extcon_get_state(chg->edev, EXTCON_CHG_USB_DCP) > 0 ||
		   ((extcon_get_state(chg->edev, EXTCON_JIG) > 0 ||
		     extcon_get_state(chg->edev, EXTCON_DOCK) > 0) &&
		    extcon_get_state(chg->edev, EXTCON_CHG_USB_SDP) <= 0)) {
		/* dedicated chargers, the UART jigs and the dock supply like a wall charger */
		iin = S2MPU06_DCP_IIN_UA;
		ichg = S2MPU06_DCP_ICHG_UA;
	}

	mutex_lock(&chg->lock);
	chg->req_iin_sel = clamp(iin / S2MPU06_IIN_STEP_UA, S2MPU06_IIN_MIN_SEL, S2MPU06_IIN_MAX_SEL);
	chg->req_ichg_sel = s2mpu06_ichg_to_sel(ichg);
	if (!s2mpu06_charger_apply_limits(chg))
		dev_info(chg->dev, "cable: input %d mA, charge %d mA\n", iin / 1000, ichg / 1000);
	mutex_unlock(&chg->lock);

	power_supply_changed(chg->psy);
}

static int s2mpu06_charger_cable_event(struct notifier_block *nb, unsigned long state, void *data)
{
	struct s2mpu06_charger *chg = container_of(nb, struct s2mpu06_charger, cable_nb);

	schedule_work(&chg->cable_work);

	return NOTIFY_OK;
}

static const unsigned int s2mpu06_charger_cables[] = {
	EXTCON_CHG_USB_SDP, EXTCON_CHG_USB_DCP, EXTCON_CHG_USB_CDP, EXTCON_JIG, EXTCON_DOCK,
};

static void s2mpu06_charger_cancel_cable(void *data)
{
	cancel_work_sync(data);
}

static void s2mpu06_charger_cancel_work(void *data)
{
	cancel_delayed_work_sync(data);
}

static int s2mpu06_charger_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct power_supply_config cfg = { };
	struct s2mpu06_charger *chg;
	unsigned int ctrl12, mode;
	int i, irq, ret;

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->dev = dev;
	chg->s2m = dev_get_drvdata(dev->parent);
	chg->mode_before_block = S2MPU06_CHG_MODE_ON;
	INIT_DELAYED_WORK(&chg->work, s2mpu06_charger_work);
	ret = devm_mutex_init(dev, &chg->lock);
	if (ret)
		return ret;

	/* start from what the bootloader or the previous kernel left configured */
	ret = regmap_read(chg->s2m->chg, S2MPU06_CHG_CTRL10, &chg->req_ichg_sel);
	if (!ret)
		ret = regmap_read(chg->s2m->chg, S2MPU06_CHG_CTRL1, &chg->req_iin_sel);
	if (!ret)
		ret = regmap_read(chg->s2m->chg, S2MPU06_CHG_CTRL2, &chg->req_vfloat_sel);
	if (ret)
		return dev_err_probe(dev, ret, "cannot read the charger\n");
	chg->req_ichg_sel &= S2MPU06_CHG_CTRL10_MASK;
	s2mpu06_charger_eoc_reset(chg);

	/*
	 * Ending the charge is up to this driver, as with the vendor's: no
	 * termination, no automatic recharge and no charging timer in the IC.
	 */
	ret = regmap_update_bits(chg->s2m->chg, S2MPU06_CHG_CTRL3, S2MPU06_CHG_CTRL3_TE, 0);
	if (!ret)
		ret = regmap_update_bits(chg->s2m->chg, S2MPU06_CHG_CTRL4,
					 S2MPU06_CHG_CTRL4_TIMER, 0);
	if (!ret)
		ret = regmap_update_bits(chg->s2m->chg, S2MPU06_CHG_CTRL12,
					 S2MPU06_CHG_CTRL12_NO_RECHG, S2MPU06_CHG_CTRL12_NO_RECHG);
	if (!ret)
		ret = s2mpu06_charger_apply_limits(chg);
	if (ret)
		return dev_err_probe(dev, ret, "cannot set up the charger\n");

	/*
	 * The mode survives a reboot: a previous kernel may have left charging
	 * off when the battery was full or too hot. Nothing blocks it here yet;
	 * the poll switches it off again when that still holds.
	 */
	ret = regmap_read(chg->s2m->chg, S2MPU06_CHG_CTRL3, &mode);
	if (!ret && FIELD_GET(S2MPU06_CHG_CTRL3_MODE, mode) == S2MPU06_CHG_MODE_OFF) {
		dev_info(dev, "charging was left off, switching it on\n");
		ret = regmap_update_bits(chg->s2m->chg, S2MPU06_CHG_CTRL3, S2MPU06_CHG_CTRL3_MODE,
					 FIELD_PREP(S2MPU06_CHG_CTRL3_MODE, S2MPU06_CHG_MODE_ON));
	}
	if (ret)
		return dev_err_probe(dev, ret, "cannot switch charging on\n");

	cfg.drv_data = chg;
	cfg.supplied_to = s2mpu06_charger_supplicants;
	cfg.num_supplicants = ARRAY_SIZE(s2mpu06_charger_supplicants);

	chg->psy = devm_power_supply_register(dev, &s2mpu06_charger_desc, &cfg);
	if (IS_ERR(chg->psy))
		return dev_err_probe(dev, PTR_ERR(chg->psy), "cannot register the charger\n");

	/* optional: without a MUIC the limits stay as they were left */
	chg->edev = extcon_get_edev_by_phandle(dev->parent, 0);
	if (IS_ERR(chg->edev)) {
		ret = PTR_ERR(chg->edev);
		chg->edev = NULL;
		if (ret == -EPROBE_DEFER)
			return ret;
	}
	if (chg->edev) {
		INIT_WORK(&chg->cable_work, s2mpu06_charger_cable_work);
		ret = devm_add_action_or_reset(dev, s2mpu06_charger_cancel_cable, &chg->cable_work);
		if (ret)
			return ret;

		chg->cable_nb.notifier_call = s2mpu06_charger_cable_event;
		for (i = 0; i < ARRAY_SIZE(s2mpu06_charger_cables); i++) {
			ret = devm_extcon_register_notifier(dev, chg->edev,
							    s2mpu06_charger_cables[i],
							    &chg->cable_nb);
			if (ret)
				return dev_err_probe(dev, ret, "cannot watch the cable\n");
		}
		schedule_work(&chg->cable_work);
	}

	regmap_read(chg->s2m->chg, S2MPU06_CHG_CTRL12, &ctrl12);
	dev_info(dev, "charger CTRL12 %#x (watchdog %s)\n", ctrl12,
		 (ctrl12 & S2MPU06_CHG_CTRL12_WDT_EN) ? "armed" : "off");

	ret = devm_add_action_or_reset(dev, s2mpu06_charger_cancel_work, &chg->work);
	if (ret)
		return ret;

	/* after the work's cleanup action, so that the irqs are gone before the work is cancelled */
	for (i = S2MPU06_IRQ_ACOKBF; i <= S2MPU06_IRQ_ACOKBR; i++) {
		irq = regmap_irq_get_virq(chg->s2m->irq_data, i);
		if (irq < 0)
			return irq;

		ret = devm_request_threaded_irq(dev, irq, NULL, s2mpu06_charger_acok_irq,
						IRQF_ONESHOT, "s2mpu06-acok", chg);
		if (ret)
			return dev_err_probe(dev, ret, "cannot request the AC OK irq\n");
	}

	schedule_delayed_work(&chg->work, 0);

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
