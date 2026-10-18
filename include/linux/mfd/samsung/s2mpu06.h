/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Samsung S2MPU06 PMIC on a SPEEDY bus
 */

#ifndef __LINUX_MFD_S2MPU06_H
#define __LINUX_MFD_S2MPU06_H

struct device;
struct regmap;

/* SPEEDY block ids of the S2MPU06's functions */
#define S2MPU06_BLK_TOP		0
#define S2MPU06_BLK_PMIC	1
#define S2MPU06_BLK_RTC		2
#define S2MPU06_BLK_CODEC	3
#define S2MPU06_BLK_CHG		4
#define S2MPU06_BLK_FG		5
#define S2MPU06_BLK_FLED	6

/* PMIC interrupts (INT1..INT3 of the PMIC block), as hwirq numbers */
enum s2mpu06_irq {
	S2MPU06_IRQ_PWRONF,
	S2MPU06_IRQ_PWRONR,
	S2MPU06_IRQ_JIGONBF,
	S2MPU06_IRQ_JIGONBR,
	S2MPU06_IRQ_ACOKBF,
	S2MPU06_IRQ_ACOKBR,
	S2MPU06_IRQ_PWRON1S,
	S2MPU06_IRQ_MRB,

	S2MPU06_IRQ_RTC60S,
	S2MPU06_IRQ_RTCA1,
	S2MPU06_IRQ_RTCA0,
	S2MPU06_IRQ_SMPL,
	S2MPU06_IRQ_RTC1S,
	S2MPU06_IRQ_WTSR,
	S2MPU06_IRQ_WRSTB,

	S2MPU06_IRQ_120C,
	S2MPU06_IRQ_140C,
	S2MPU06_IRQ_TSD,

	S2MPU06_IRQ_NR,
};

/* data the MFD core shares with its cells through the parent's drvdata */
struct s2mpu06 {
	struct device *dev;
	struct regmap *top;
	struct regmap *pmic;
	struct regmap *chg;
	struct regmap *fg;
	struct regmap *codec;
	struct regmap_irq_chip_data *irq_data;
};

#endif
