// SPDX-License-Identifier: GPL-2.0
/*
 * Charger of the Samsung S2MPU06 PMIC
 *
 * The charger IC runs the charging cycle on its own. This driver reports its
 * state, exposes the current and voltage limits, keeps its watchdog fed and
 * restricts charging when the battery is too hot or too cold.
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
#define S2MPU06_CHG_MODE_OFF		0
#define S2MPU06_CHG_MODE_ON		1
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

#define S2MPU06_POLL_PERIOD		msecs_to_jiffies(10000)

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

	return regmap_write(chg->s2m->chg, S2MPU06_CHG_CTRL1, chg->req_iin_sel);
}

/* charging is off while the battery is too hot or too cold or when userspace inhibits it */
static int s2mpu06_charger_update_block(struct s2mpu06_charger *chg)
{
	bool block = chg->inhibit || chg->state >= S2MPU06_TEMP_COLD_BLOCK;
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

/* the battery temperature comes from the fuel gauge; without it nothing changes */
static int s2mpu06_charger_get_temp(int *temp)
{
	struct power_supply *fg = power_supply_get_by_name("s2mpu06-fuelgauge");
	union power_supply_propval val;
	int ret;

	if (!fg)
		return -ENODEV;

	ret = power_supply_get_property(fg, POWER_SUPPLY_PROP_TEMP, &val);
	power_supply_put(fg);
	if (ret)
		return ret;

	*temp = val.intval;
	return 0;
}

static void s2mpu06_charger_thermal(struct s2mpu06_charger *chg)
{
	enum s2mpu06_temp_state old, next;
	int temp, ret = 0;

	if (s2mpu06_charger_get_temp(&temp))
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

/* feeds the watchdog (the IC drops to its defaults otherwise) and checks the temperature */
static void s2mpu06_charger_work(struct work_struct *work)
{
	struct s2mpu06_charger *chg = container_of(work, struct s2mpu06_charger, work.work);
	unsigned int ctrl12;

	if (!regmap_read(chg->s2m->chg, S2MPU06_CHG_CTRL12, &ctrl12) &&
	    (ctrl12 & S2MPU06_CHG_CTRL12_WDT_EN)) {
		regmap_update_bits(chg->s2m->chg, S2MPU06_CHG_CTRL12,
				   S2MPU06_CHG_CTRL12_WDT_KICK, 0x1);
		regmap_update_bits(chg->s2m->chg, S2MPU06_CHG_CTRL12,
				   S2MPU06_CHG_CTRL12_WDT_KICK, 0x0);
	}

	s2mpu06_charger_thermal(chg);

	schedule_delayed_work(&chg->work, S2MPU06_POLL_PERIOD);
}

static irqreturn_t s2mpu06_charger_acok_irq(int irq, void *data)
{
	struct s2mpu06_charger *chg = data;

	power_supply_changed(chg->psy);

	return IRQ_HANDLED;
}

/* the cable the MUIC reports decides how much current may be drawn and given to the cell */
static void s2mpu06_charger_cable_work(struct work_struct *work)
{
	struct s2mpu06_charger *chg = container_of(work, struct s2mpu06_charger, cable_work);
	int iin = S2MPU06_USB_IIN_UA, ichg = S2MPU06_USB_ICHG_UA;

	if (extcon_get_state(chg->edev, EXTCON_CHG_USB_DCP) > 0) {
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
	unsigned int ctrl12;
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
		for (i = 0; i < 2; i++) {
			ret = devm_extcon_register_notifier(dev, chg->edev,
							    i ? EXTCON_CHG_USB_DCP : EXTCON_CHG_USB_SDP,
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
