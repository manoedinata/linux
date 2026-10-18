// SPDX-License-Identifier: GPL-2.0
/*
 * MFD core of the Samsung S2MPU06 PMIC on a SPEEDY bus
 *
 * The PMIC keeps one level-low interrupt line for all its functions. The top
 * block's INTSRC says which function raised it; only the PMIC block's INT1..3
 * are handled here (as a regmap-irq chip), the charger, fuel gauge and codec
 * sources are only acknowledged by reading their status registers (which
 * clears them) until their drivers exist.
 */

#include <linux/interrupt.h>
#include <linux/mfd/core.h>
#include <linux/mfd/samsung/s2mpu06.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/soc/samsung/exynos-speedy.h>

#define S2MPU06_TOP_PMICID		0x00
#define S2MPU06_TOP_INTSRC		0x05

#define S2MPU06_INTSRC_PMIC		BIT(0)
#define S2MPU06_INTSRC_CODEC		BIT(1)
#define S2MPU06_INTSRC_CHG		BIT(2)
#define S2MPU06_INTSRC_FG		BIT(4)

#define S2MPU06_PMIC_INT1		0x00
#define S2MPU06_PMIC_INT1M		0x03
#define S2MPU06_NUM_IRQ_REGS		3

#define S2MPU06_CHG_INT1		0x00	/* INT1..3 and PMIC_INT, read to clear */
#define S2MPU06_CHG_NUM_INT		4
#define S2MPU06_FG_IRQ_INT		0x02
#define S2MPU06_CODEC_INT1		0x01	/* INT1..3, read to clear */

#define S2MPU06_IRQ(_irq, _reg, _bit)	\
	[_irq] = { .reg_offset = (_reg), .mask = BIT(_bit) }

static const struct regmap_irq s2mpu06_irqs[] = {
	S2MPU06_IRQ(S2MPU06_IRQ_PWRONF, 0, 0),
	S2MPU06_IRQ(S2MPU06_IRQ_PWRONR, 0, 1),
	S2MPU06_IRQ(S2MPU06_IRQ_JIGONBF, 0, 2),
	S2MPU06_IRQ(S2MPU06_IRQ_JIGONBR, 0, 3),
	S2MPU06_IRQ(S2MPU06_IRQ_ACOKBF, 0, 4),
	S2MPU06_IRQ(S2MPU06_IRQ_ACOKBR, 0, 5),
	S2MPU06_IRQ(S2MPU06_IRQ_PWRON1S, 0, 6),
	S2MPU06_IRQ(S2MPU06_IRQ_MRB, 0, 7),
	S2MPU06_IRQ(S2MPU06_IRQ_RTC60S, 1, 0),
	S2MPU06_IRQ(S2MPU06_IRQ_RTCA1, 1, 1),
	S2MPU06_IRQ(S2MPU06_IRQ_RTCA0, 1, 2),
	S2MPU06_IRQ(S2MPU06_IRQ_SMPL, 1, 3),
	S2MPU06_IRQ(S2MPU06_IRQ_RTC1S, 1, 4),
	S2MPU06_IRQ(S2MPU06_IRQ_WTSR, 1, 5),
	S2MPU06_IRQ(S2MPU06_IRQ_WRSTB, 1, 7),
	S2MPU06_IRQ(S2MPU06_IRQ_120C, 2, 0),
	S2MPU06_IRQ(S2MPU06_IRQ_140C, 2, 1),
	S2MPU06_IRQ(S2MPU06_IRQ_TSD, 2, 2),
};

/* runs with the IRQ line asserted, before the PMIC block's INT1..3 are read */
static int s2mpu06_irq_pre(void *data)
{
	struct s2mpu06 *s2m = data;
	u8 buf[S2MPU06_CHG_NUM_INT];
	unsigned int src;
	int ret;

	ret = regmap_read(s2m->top, S2MPU06_TOP_INTSRC, &src);
	if (ret)
		return ret;

	if ((src & S2MPU06_INTSRC_CHG) && s2m->chg)
		regmap_bulk_read(s2m->chg, S2MPU06_CHG_INT1, buf, S2MPU06_CHG_NUM_INT);
	if ((src & S2MPU06_INTSRC_FG) && s2m->fg)
		regmap_read(s2m->fg, S2MPU06_FG_IRQ_INT, &ret);
	if ((src & S2MPU06_INTSRC_CODEC) && s2m->codec)
		regmap_bulk_read(s2m->codec, S2MPU06_CODEC_INT1, buf, 3);

	return 0;
}

static const struct regmap_irq_chip s2mpu06_irq_chip = {
	.name = "s2mpu06",
	.status_base = S2MPU06_PMIC_INT1,
	.mask_base = S2MPU06_PMIC_INT1M,
	.num_regs = S2MPU06_NUM_IRQ_REGS,
	.irqs = s2mpu06_irqs,
	.num_irqs = ARRAY_SIZE(s2mpu06_irqs),
	.handle_pre_irq = s2mpu06_irq_pre,
};

static const struct resource s2mpu06_pwrkey_res[] = {
	DEFINE_RES_IRQ_NAMED(S2MPU06_IRQ_PWRONF, "pwronf"),
	DEFINE_RES_IRQ_NAMED(S2MPU06_IRQ_PWRONR, "pwronr"),
};

static const struct mfd_cell s2mpu06_cells[] = {
	{ .name = "s2mpu06-regulator" },
	{ .name = "s2mpu06-rtc" },
	{ .name = "s2mpu06-charger" },
	{ .name = "s2mpu06-fuelgauge" },
	{
		.name = "s2mpu06-pwrkey",
		.num_resources = ARRAY_SIZE(s2mpu06_pwrkey_res),
		.resources = s2mpu06_pwrkey_res,
	},
};

static int s2mpu06_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s2mpu06 *s2m;
	unsigned int id;
	int irq, ret;

	s2m = devm_kzalloc(dev, sizeof(*s2m), GFP_KERNEL);
	if (!s2m)
		return -ENOMEM;
	s2m->dev = dev;

	s2m->top = devm_exynos_speedy_regmap(dev, S2MPU06_BLK_TOP);
	if (IS_ERR(s2m->top))
		return dev_err_probe(dev, PTR_ERR(s2m->top), "cannot get TOP regmap\n");

	s2m->pmic = devm_exynos_speedy_regmap(dev, S2MPU06_BLK_PMIC);
	if (IS_ERR(s2m->pmic))
		return dev_err_probe(dev, PTR_ERR(s2m->pmic), "cannot get PMIC regmap\n");

	s2m->rtc = devm_exynos_speedy_regmap(dev, S2MPU06_BLK_RTC);
	if (IS_ERR(s2m->rtc))
		return dev_err_probe(dev, PTR_ERR(s2m->rtc), "cannot get RTC regmap\n");

	s2m->chg = devm_exynos_speedy_regmap(dev, S2MPU06_BLK_CHG);
	if (IS_ERR(s2m->chg))
		return dev_err_probe(dev, PTR_ERR(s2m->chg), "cannot get charger regmap\n");
	s2m->fg = devm_exynos_speedy_regmap(dev, S2MPU06_BLK_FG);
	if (IS_ERR(s2m->fg))
		return dev_err_probe(dev, PTR_ERR(s2m->fg), "cannot get fuel gauge regmap\n");

	/* only used to acknowledge interrupts, so failing to get it is harmless */
	s2m->codec = devm_exynos_speedy_regmap(dev, S2MPU06_BLK_CODEC);
	if (IS_ERR(s2m->codec))
		s2m->codec = NULL;

	ret = regmap_read(s2m->top, S2MPU06_TOP_PMICID, &id);
	if (ret)
		return dev_err_probe(dev, ret, "cannot read the PMIC id\n");
	dev_info(dev, "S2MPU06 PMIC id %#x\n", id);

	platform_set_drvdata(pdev, s2m);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	/* the chip's irq_pre callback needs the context, so build it per device */
	{
		struct regmap_irq_chip *chip;

		chip = devm_kmemdup(dev, &s2mpu06_irq_chip, sizeof(*chip), GFP_KERNEL);
		if (!chip)
			return -ENOMEM;
		chip->irq_drv_data = s2m;
		ret = devm_regmap_add_irq_chip(dev, s2m->pmic, irq,
					       IRQF_ONESHOT | IRQF_TRIGGER_LOW, 0,
					       chip, &s2m->irq_data);
	}
	if (ret)
		return dev_err_probe(dev, ret, "cannot add the irq chip\n");

	return devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO, s2mpu06_cells,
				    ARRAY_SIZE(s2mpu06_cells), NULL, 0,
				    regmap_irq_get_domain(s2m->irq_data));
}

static const struct of_device_id s2mpu06_of_match[] = {
	{ .compatible = "samsung,s2mpu06-pmic" },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mpu06_of_match);

static struct platform_driver s2mpu06_driver = {
	.probe = s2mpu06_probe,
	.driver = {
		.name = "s2mpu06",
		.of_match_table = s2mpu06_of_match,
	},
};
module_platform_driver(s2mpu06_driver);

MODULE_DESCRIPTION("Samsung S2MPU06 PMIC core driver");
MODULE_LICENSE("GPL");
