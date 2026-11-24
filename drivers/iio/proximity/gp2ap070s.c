// SPDX-License-Identifier: GPL-2.0
/*
 * Sharp GP2AP070S proximity sensor
 *
 * Register layout and the Galaxy J4 settings come from Samsung's downstream
 * driver (drivers/sensors/gp2ap070s.c). The chip measures reflected IR from
 * its own LED (D0, 16 bit, minus an offset register) and keeps a near/far
 * state with a high/low threshold hysteresis itself (COMMAND2 bit 3, set =
 * near). The interrupt line changes with that state.
 *
 * Measured on a J4 with the downstream settings: far reads 0, a hand over
 * the sensor 260..960.
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#define GP2AP070S_COM1			0x80
#define GP2AP070S_COM1_WAKEUP		BIT(7)
#define GP2AP070S_COM1_PS		BIT(5)
#define GP2AP070S_COM2			0x81
#define GP2AP070S_COM2_PROX		BIT(3)
#define GP2AP070S_COM3			0x82
#define GP2AP070S_COM3_INT_PULSE	BIT(1)
#define GP2AP070S_COM4			0x83
#define GP2AP070S_PS1			0x85
#define GP2AP070S_PS2			0x86
#define GP2AP070S_PS3			0x87
#define GP2AP070S_PS_LT			0x88	/* 16 bit, LSB first */
#define GP2AP070S_PS_HT			0x8a
#define GP2AP070S_OS_D0			0x8c
#define GP2AP070S_D0			0x90
#define GP2AP070S_REV_CODE		0xa0

#define GP2AP070S_MAX_VALUE		0xffff

/* downstream J4 device tree values */
#define GP2AP070S_DEF_COM4		0x03	/* interval 33 */
#define GP2AP070S_DEF_PS1		0x20
#define GP2AP070S_DEF_PS2		0x24
#define GP2AP070S_DEF_PS3		0x38
#define GP2AP070S_DEF_LOW		60
#define GP2AP070S_DEF_HIGH		110
#define GP2AP070S_DEF_OFFSET		65

struct gp2ap070s {
	struct device *dev;
	struct regmap *map;
	struct mutex lock;	/* power state, near/far state */
	bool events_on;
	bool near;
	u32 near_level;
};

static const struct regmap_config gp2ap070s_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xe0,
};

static int gp2ap070s_write16(struct gp2ap070s *p, unsigned int reg, unsigned int val)
{
	u8 buf[2] = { val & 0xff, val >> 8 };

	return regmap_bulk_write(p->map, reg, buf, 2);
}

static int gp2ap070s_read16(struct gp2ap070s *p, unsigned int reg, int *val)
{
	u8 buf[2];
	int ret;

	ret = regmap_bulk_read(p->map, reg, buf, 2);
	if (ret)
		return ret;
	*val = buf[0] | buf[1] << 8;
	return 0;
}

/* caller holds the lock */
static int gp2ap070s_set_running(struct gp2ap070s *p, bool on)
{
	return regmap_write(p->map, GP2AP070S_COM1,
			    on ? GP2AP070S_COM1_WAKEUP | GP2AP070S_COM1_PS : 0);
}

static int gp2ap070s_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val, int *val2, long mask)
{
	struct gp2ap070s *p = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&p->lock);
		if (!p->events_on) {
			ret = gp2ap070s_set_running(p, true);
			if (ret)
				goto unlock;
			/* one conversion takes up to ~10.5 ms */
			msleep(20);
		}
		ret = gp2ap070s_read16(p, GP2AP070S_D0, val);
		if (!p->events_on)
			gp2ap070s_set_running(p, false);
unlock:
		mutex_unlock(&p->lock);
		return ret ? ret : IIO_VAL_INT;

	case IIO_CHAN_INFO_CALIBBIAS:
		ret = gp2ap070s_read16(p, GP2AP070S_OS_D0, val);
		return ret ? ret : IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int gp2ap070s_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val, int val2, long mask)
{
	struct gp2ap070s *p = iio_priv(indio_dev);

	if (mask != IIO_CHAN_INFO_CALIBBIAS)
		return -EINVAL;
	if (val < 0 || val > GP2AP070S_MAX_VALUE || val2)
		return -EINVAL;
	return gp2ap070s_write16(p, GP2AP070S_OS_D0, val);
}

static int gp2ap070s_read_event_value(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      enum iio_event_type type,
				      enum iio_event_direction dir,
				      enum iio_event_info info,
				      int *val, int *val2)
{
	struct gp2ap070s *p = iio_priv(indio_dev);
	unsigned int reg = dir == IIO_EV_DIR_RISING ? GP2AP070S_PS_HT : GP2AP070S_PS_LT;
	int ret;

	ret = gp2ap070s_read16(p, reg, val);
	return ret ? ret : IIO_VAL_INT;
}

static int gp2ap070s_write_event_value(struct iio_dev *indio_dev,
				       const struct iio_chan_spec *chan,
				       enum iio_event_type type,
				       enum iio_event_direction dir,
				       enum iio_event_info info,
				       int val, int val2)
{
	struct gp2ap070s *p = iio_priv(indio_dev);
	unsigned int reg = dir == IIO_EV_DIR_RISING ? GP2AP070S_PS_HT : GP2AP070S_PS_LT;

	if (val < 0 || val > GP2AP070S_MAX_VALUE || val2)
		return -EINVAL;
	return gp2ap070s_write16(p, reg, val);
}

static int gp2ap070s_read_event_config(struct iio_dev *indio_dev,
				       const struct iio_chan_spec *chan,
				       enum iio_event_type type,
				       enum iio_event_direction dir)
{
	struct gp2ap070s *p = iio_priv(indio_dev);

	return p->events_on;
}

static int gp2ap070s_write_event_config(struct iio_dev *indio_dev,
					const struct iio_chan_spec *chan,
					enum iio_event_type type,
					enum iio_event_direction dir,
					int state)
{
	struct gp2ap070s *p = iio_priv(indio_dev);
	unsigned int com2;
	int ret = 0;

	mutex_lock(&p->lock);
	if (!!state == p->events_on)
		goto unlock;

	ret = gp2ap070s_set_running(p, state);
	if (ret)
		goto unlock;
	p->events_on = state;
	if (state) {
		/* the chip starts out far; take whatever it sees first */
		msleep(20);
		if (!regmap_read(p->map, GP2AP070S_COM2, &com2))
			p->near = com2 & GP2AP070S_COM2_PROX;
	}
unlock:
	mutex_unlock(&p->lock);
	return ret;
}

static irqreturn_t gp2ap070s_irq(int irq, void *dev_id)
{
	struct iio_dev *indio_dev = dev_id;
	struct gp2ap070s *p = iio_priv(indio_dev);
	unsigned int com2;
	bool near;

	mutex_lock(&p->lock);
	if (!p->events_on || regmap_read(p->map, GP2AP070S_COM2, &com2))
		goto unlock;

	near = com2 & GP2AP070S_COM2_PROX;
	if (near != p->near) {
		enum iio_event_direction dir = near ? IIO_EV_DIR_RISING : IIO_EV_DIR_FALLING;

		p->near = near;
		iio_push_event(indio_dev,
			       IIO_UNMOD_EVENT_CODE(IIO_PROXIMITY, 0, IIO_EV_TYPE_THRESH, dir),
			       iio_get_time_ns(indio_dev));
	}
unlock:
	mutex_unlock(&p->lock);
	return IRQ_HANDLED;
}

static ssize_t gp2ap070s_read_near_level(struct iio_dev *indio_dev,
					 uintptr_t priv,
					 const struct iio_chan_spec *chan,
					 char *buf)
{
	struct gp2ap070s *p = iio_priv(indio_dev);

	return sysfs_emit(buf, "%u\n", p->near_level);
}

static const struct iio_chan_spec_ext_info gp2ap070s_ext_info[] = {
	{
		.name = "nearlevel",
		.shared = IIO_SEPARATE,
		.read = gp2ap070s_read_near_level,
	},
	{ }
};

static const struct iio_event_spec gp2ap070s_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
	},
};

static const struct iio_chan_spec gp2ap070s_channels[] = {
	{
		.type = IIO_PROXIMITY,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_CALIBBIAS),
		.event_spec = gp2ap070s_events,
		.num_event_specs = ARRAY_SIZE(gp2ap070s_events),
		.ext_info = gp2ap070s_ext_info,
	},
};

static const struct iio_info gp2ap070s_info = {
	.read_raw = gp2ap070s_read_raw,
	.write_raw = gp2ap070s_write_raw,
	.read_event_value = gp2ap070s_read_event_value,
	.write_event_value = gp2ap070s_write_event_value,
	.read_event_config = gp2ap070s_read_event_config,
	.write_event_config = gp2ap070s_write_event_config,
};

static void gp2ap070s_shutdown_chip(void *data)
{
	struct gp2ap070s *p = data;

	gp2ap070s_set_running(p, false);
}

static int gp2ap070s_init_chip(struct gp2ap070s *p)
{
	const struct reg_sequence init[] = {
		{ GP2AP070S_COM1, 0 },
		{ GP2AP070S_COM2, 0 },	/* clear all interrupts */
		{ GP2AP070S_COM3, GP2AP070S_COM3_INT_PULSE },
		{ GP2AP070S_COM4, GP2AP070S_DEF_COM4 },
		{ GP2AP070S_PS1, GP2AP070S_DEF_PS1 },
		{ GP2AP070S_PS2, GP2AP070S_DEF_PS2 },
		{ GP2AP070S_PS3, GP2AP070S_DEF_PS3 },
	};
	int ret;

	ret = regmap_multi_reg_write(p->map, init, ARRAY_SIZE(init));
	if (!ret)
		ret = gp2ap070s_write16(p, GP2AP070S_PS_LT, GP2AP070S_DEF_LOW);
	if (!ret)
		ret = gp2ap070s_write16(p, GP2AP070S_PS_HT, GP2AP070S_DEF_HIGH);
	if (!ret)
		ret = gp2ap070s_write16(p, GP2AP070S_OS_D0, GP2AP070S_DEF_OFFSET);
	return ret;
}

static int gp2ap070s_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct iio_dev *indio_dev;
	struct gp2ap070s *p;
	unsigned int rev;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*p));
	if (!indio_dev)
		return -ENOMEM;
	p = iio_priv(indio_dev);
	p->dev = dev;
	mutex_init(&p->lock);

	ret = devm_regulator_get_enable(dev, "vdd");
	if (ret)
		return dev_err_probe(dev, ret, "cannot enable vdd\n");
	usleep_range(10000, 11000);

	p->map = devm_regmap_init_i2c(client, &gp2ap070s_regmap_config);
	if (IS_ERR(p->map))
		return dev_err_probe(dev, PTR_ERR(p->map), "cannot set up regmap\n");

	ret = regmap_read(p->map, GP2AP070S_REV_CODE, &rev);
	if (ret)
		return dev_err_probe(dev, ret, "no answer from the chip\n");

	ret = gp2ap070s_init_chip(p);
	if (ret)
		return dev_err_probe(dev, ret, "cannot set up the chip\n");
	ret = devm_add_action_or_reset(dev, gp2ap070s_shutdown_chip, p);
	if (ret)
		return ret;

	p->near_level = GP2AP070S_DEF_HIGH;
	device_property_read_u32(dev, "proximity-near-level", &p->near_level);

	indio_dev->name = "gp2ap070s";
	indio_dev->info = &gp2ap070s_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = gp2ap070s_channels;
	indio_dev->num_channels = ARRAY_SIZE(gp2ap070s_channels);

	if (client->irq > 0) {
		ret = devm_request_threaded_irq(dev, client->irq, NULL, gp2ap070s_irq,
						IRQF_ONESHOT, "gp2ap070s", indio_dev);
		if (ret)
			return dev_err_probe(dev, ret, "cannot request the irq\n");
	}

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret)
		return ret;

	dev_info(dev, "GP2AP070S rev %#x\n", rev);
	return 0;
}

static const struct of_device_id gp2ap070s_of_match[] = {
	{ .compatible = "sharp,gp2ap070s" },
	{ }
};
MODULE_DEVICE_TABLE(of, gp2ap070s_of_match);

static const struct i2c_device_id gp2ap070s_id[] = {
	{ "gp2ap070s" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, gp2ap070s_id);

static struct i2c_driver gp2ap070s_driver = {
	.driver = {
		.name = "gp2ap070s",
		.of_match_table = gp2ap070s_of_match,
	},
	.probe = gp2ap070s_probe,
	.id_table = gp2ap070s_id,
};
module_i2c_driver(gp2ap070s_driver);

MODULE_DESCRIPTION("Sharp GP2AP070S proximity sensor driver");
MODULE_LICENSE("GPL");
