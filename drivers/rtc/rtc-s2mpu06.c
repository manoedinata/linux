// SPDX-License-Identifier: GPL-2.0
/*
 * RTC of the Samsung S2MPU06 PMIC
 *
 * Register layout from the downstream rtc-s2mp driver:
 * Copyright (c) 2015 Samsung Electronics Co., Ltd
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/mfd/samsung/s2mpu06.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/rtc.h>

/* RTC block registers */
#define S2MPU06_RTC_CTRL		0x00
#define S2MPU06_RTC_UPDATE		0x02
#define S2MPU06_RTC_CAPSEL		0x03
#define S2MPU06_RTC_TIME		0x04	/* sec, min, hour, weekday, date, month, year */
#define S2MPU06_RTC_ALARM0		0x0b	/* same layout as the time */

#define S2MPU06_RTC_CTRL_24H		BIT(1)
#define S2MPU06_RTC_CAPSEL_VAL		0xf8

#define S2MPU06_RTC_UPDATE_RUDR		BIT(0)
#define S2MPU06_RTC_UPDATE_WUDR		BIT(1)
#define S2MPU06_RTC_UPDATE_FREEZE	BIT(2)
#define S2MPU06_RTC_UPDATE_AUDR		BIT(4)
#define S2MPU06_RTC_UPDATE_BITS		(S2MPU06_RTC_UPDATE_RUDR | S2MPU06_RTC_UPDATE_WUDR | \
					 S2MPU06_RTC_UPDATE_FREEZE | S2MPU06_RTC_UPDATE_AUDR)

#define S2MPU06_RTC_HOUR_PM		BIT(6)
#define S2MPU06_RTC_ALARM_EN		BIT(7)

/* STATUS2 in the PMIC block */
#define S2MPU06_PMIC_STATUS2		0x07
#define S2MPU06_STATUS2_RTCA0E		BIT(2)

enum {
	RTC_SEC,
	RTC_MIN,
	RTC_HOUR,
	RTC_WEEKDAY,
	RTC_DATE,
	RTC_MONTH,
	RTC_YEAR,
	RTC_NR_REGS,
};

struct s2mpu06_rtc {
	struct device *dev;
	struct s2mpu06 *s2m;
	struct rtc_device *rtc;
	struct mutex lock;
	unsigned int update_base;
	int irq;
	bool alarm_irq_on;
};

static void s2mpu06_rtc_data_to_tm(const u8 *data, struct rtc_time *tm)
{
	tm->tm_sec = data[RTC_SEC] & 0x7f;
	tm->tm_min = data[RTC_MIN] & 0x7f;
	tm->tm_hour = data[RTC_HOUR] & 0x1f;
	tm->tm_wday = __fls(data[RTC_WEEKDAY] & 0x7f);
	tm->tm_mday = data[RTC_DATE] & 0x1f;
	tm->tm_mon = (data[RTC_MONTH] & 0x0f) - 1;
	tm->tm_year = (data[RTC_YEAR] & 0x7f) + 100;
}

static int s2mpu06_rtc_tm_to_data(const struct rtc_time *tm, u8 *data)
{
	if (tm->tm_year < 100)
		return -EINVAL;

	data[RTC_SEC] = tm->tm_sec;
	data[RTC_MIN] = tm->tm_min;
	data[RTC_HOUR] = tm->tm_hour | (tm->tm_hour >= 12 ? S2MPU06_RTC_HOUR_PM : 0);
	data[RTC_WEEKDAY] = BIT(tm->tm_wday);
	data[RTC_DATE] = tm->tm_mday;
	data[RTC_MONTH] = tm->tm_mon + 1;
	data[RTC_YEAR] = tm->tm_year - 100;

	return 0;
}

/* latch (read) or commit (write time/alarm) the shadow registers */
static int s2mpu06_rtc_update(struct s2mpu06_rtc *info, unsigned int bit)
{
	int ret;

	ret = regmap_write(info->s2m->rtc, S2MPU06_RTC_UPDATE, info->update_base | bit);
	if (ret)
		return ret;

	usleep_range(1000, 1500);
	return 0;
}

static int s2mpu06_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct s2mpu06_rtc *info = dev_get_drvdata(dev);
	u8 data[RTC_NR_REGS];
	int ret;

	mutex_lock(&info->lock);
	ret = s2mpu06_rtc_update(info, S2MPU06_RTC_UPDATE_RUDR);
	if (!ret)
		ret = regmap_bulk_read(info->s2m->rtc, S2MPU06_RTC_TIME, data, RTC_NR_REGS);
	mutex_unlock(&info->lock);
	if (ret)
		return ret;

	s2mpu06_rtc_data_to_tm(data, tm);
	return 0;
}

static int s2mpu06_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct s2mpu06_rtc *info = dev_get_drvdata(dev);
	u8 data[RTC_NR_REGS];
	int ret;

	ret = s2mpu06_rtc_tm_to_data(tm, data);
	if (ret)
		return ret;

	mutex_lock(&info->lock);
	ret = regmap_bulk_write(info->s2m->rtc, S2MPU06_RTC_TIME, data, RTC_NR_REGS);
	if (!ret)
		ret = s2mpu06_rtc_update(info, S2MPU06_RTC_UPDATE_WUDR);
	mutex_unlock(&info->lock);

	return ret;
}

static int s2mpu06_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct s2mpu06_rtc *info = dev_get_drvdata(dev);
	u8 data[RTC_NR_REGS];
	unsigned int status;
	int ret;

	mutex_lock(&info->lock);
	ret = s2mpu06_rtc_update(info, S2MPU06_RTC_UPDATE_RUDR);
	if (!ret)
		ret = regmap_bulk_read(info->s2m->rtc, S2MPU06_RTC_ALARM0, data, RTC_NR_REGS);
	if (!ret)
		ret = regmap_read(info->s2m->pmic, S2MPU06_PMIC_STATUS2, &status);
	mutex_unlock(&info->lock);
	if (ret)
		return ret;

	s2mpu06_rtc_data_to_tm(data, &alrm->time);
	alrm->enabled = info->alarm_irq_on;
	alrm->pending = !!(status & S2MPU06_STATUS2_RTCA0E);

	return 0;
}

static int s2mpu06_rtc_set_alarm_irq(struct s2mpu06_rtc *info, bool enable)
{
	if (enable && !info->alarm_irq_on)
		enable_irq(info->irq);
	else if (!enable && info->alarm_irq_on)
		disable_irq(info->irq);

	info->alarm_irq_on = enable;
	return 0;
}

static int s2mpu06_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct s2mpu06_rtc *info = dev_get_drvdata(dev);
	u8 data[RTC_NR_REGS];
	int ret, i;

	ret = s2mpu06_rtc_tm_to_data(&alrm->time, data);
	if (ret)
		return ret;

	mutex_lock(&info->lock);

	/* disarm first, then load the new time and arm it */
	for (i = 0; i < RTC_NR_REGS; i++)
		data[i] &= ~S2MPU06_RTC_ALARM_EN;
	ret = regmap_bulk_write(info->s2m->rtc, S2MPU06_RTC_ALARM0, data, RTC_NR_REGS);
	if (!ret)
		ret = s2mpu06_rtc_update(info, S2MPU06_RTC_UPDATE_AUDR);
	if (ret)
		goto out;

	for (i = 0; i < RTC_NR_REGS; i++)
		data[i] |= S2MPU06_RTC_ALARM_EN;
	ret = regmap_bulk_write(info->s2m->rtc, S2MPU06_RTC_ALARM0, data, RTC_NR_REGS);
	if (!ret)
		ret = s2mpu06_rtc_update(info, S2MPU06_RTC_UPDATE_AUDR);
	if (!ret)
		ret = s2mpu06_rtc_set_alarm_irq(info, alrm->enabled);
out:
	mutex_unlock(&info->lock);
	return ret;
}

static int s2mpu06_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct s2mpu06_rtc *info = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&info->lock);
	ret = s2mpu06_rtc_set_alarm_irq(info, enabled);
	mutex_unlock(&info->lock);

	return ret;
}

static irqreturn_t s2mpu06_rtc_alarm_irq(int irq, void *data)
{
	struct s2mpu06_rtc *info = data;

	rtc_update_irq(info->rtc, 1, RTC_IRQF | RTC_AF);

	return IRQ_HANDLED;
}

static const struct rtc_class_ops s2mpu06_rtc_ops = {
	.read_time		= s2mpu06_rtc_read_time,
	.set_time		= s2mpu06_rtc_set_time,
	.read_alarm		= s2mpu06_rtc_read_alarm,
	.set_alarm		= s2mpu06_rtc_set_alarm,
	.alarm_irq_enable	= s2mpu06_rtc_alarm_irq_enable,
};

static int s2mpu06_rtc_init_regs(struct s2mpu06_rtc *info)
{
	struct regmap *rtc = info->s2m->rtc;
	unsigned int update, ctrl, capsel;
	int ret;

	ret = regmap_read(rtc, S2MPU06_RTC_UPDATE, &update);
	if (ret)
		return ret;

	/* every update strobe is written together with the bits that are not strobes */
	info->update_base = update & ~S2MPU06_RTC_UPDATE_BITS;
	ret = regmap_write(rtc, S2MPU06_RTC_UPDATE, info->update_base);
	if (ret)
		return ret;

	ret = regmap_read(rtc, S2MPU06_RTC_CTRL, &ctrl);
	if (ret)
		return ret;
	ret = regmap_read(rtc, S2MPU06_RTC_CAPSEL, &capsel);
	if (ret)
		return ret;

	/* all zero means the RTC lost its backup power, so configure it again */
	if ((ctrl & S2MPU06_RTC_CTRL_24H) && capsel == S2MPU06_RTC_CAPSEL_VAL)
		return 0;

	ret = regmap_write(rtc, S2MPU06_RTC_CTRL, S2MPU06_RTC_CTRL_24H);
	if (ret)
		return ret;
	ret = s2mpu06_rtc_update(info, S2MPU06_RTC_UPDATE_AUDR);
	if (ret)
		return ret;

	return regmap_write(rtc, S2MPU06_RTC_CAPSEL, S2MPU06_RTC_CAPSEL_VAL);
}

static int s2mpu06_rtc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s2mpu06_rtc *info;
	int ret;

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = dev;
	info->s2m = dev_get_drvdata(dev->parent);
	mutex_init(&info->lock);
	platform_set_drvdata(pdev, info);

	info->irq = regmap_irq_get_virq(info->s2m->irq_data, S2MPU06_IRQ_RTCA0);
	if (info->irq < 0)
		return info->irq;

	ret = s2mpu06_rtc_init_regs(info);
	if (ret)
		return dev_err_probe(dev, ret, "cannot initialise the RTC registers\n");

	info->rtc = devm_rtc_allocate_device(dev);
	if (IS_ERR(info->rtc))
		return PTR_ERR(info->rtc);

	info->rtc->ops = &s2mpu06_rtc_ops;
	info->rtc->range_min = RTC_TIMESTAMP_BEGIN_2000;
	info->rtc->range_max = RTC_TIMESTAMP_END_2099;

	irq_set_status_flags(info->irq, IRQ_NOAUTOEN);
	ret = devm_request_threaded_irq(dev, info->irq, NULL, s2mpu06_rtc_alarm_irq,
					IRQF_ONESHOT, "rtc-alarm0", info);
	if (ret)
		return dev_err_probe(dev, ret, "cannot request the alarm irq\n");

	device_init_wakeup(dev, true);

	return devm_rtc_register_device(info->rtc);
}

static const struct platform_device_id s2mpu06_rtc_id[] = {
	{ "s2mpu06-rtc" },
	{ }
};
MODULE_DEVICE_TABLE(platform, s2mpu06_rtc_id);

static struct platform_driver s2mpu06_rtc_driver = {
	.probe = s2mpu06_rtc_probe,
	.driver = {
		.name = "s2mpu06-rtc",
	},
	.id_table = s2mpu06_rtc_id,
};
module_platform_driver(s2mpu06_rtc_driver);

MODULE_DESCRIPTION("Samsung S2MPU06 RTC driver");
MODULE_LICENSE("GPL");
