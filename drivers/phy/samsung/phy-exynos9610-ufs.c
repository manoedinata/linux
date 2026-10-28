// SPDX-License-Identifier: GPL-2.0-only
/*
 * UFS PHY driver data for Samsung Exynos9610 SoC
 *
 * Values from the downstream CAL (ufs-cal-9610.c). Its PMA byte addresses
 * are given here as APB word offsets; one TRSV lane is 0x140 bytes.
 */

#include <linux/delay.h>
#include <linux/io.h>

#include "phy-samsung-ufs.h"

#define EXYNOS9610_UFS_PHY_CTRL			0x724
#define EXYNOS9610_UFS_PHY_CTRL_MASK		0x1
#define EXYNOS9610_UFS_PHY_CTRL_EN		BIT(0)

#define EXYNOS9610_TRSV_CH_OFFSET		0x50

#define EXYNOS9610_TRSV_REG_CFG(o, v, d)	\
	PHY_TRSV_REG_CFG_OFFSET(o, v, d, EXYNOS9610_TRSV_CH_OFFSET)

/* TRSV registers polled after PMC and on HIBERN8 exit */
#define EXYNOS9610_TRSV_CDR_AFC			0x7f
#define EXYNOS9610_TRSV_CDR_AFC_PMC_DONE	BIT(6)
#define EXYNOS9610_TRSV_CDR_AFC_H8_DONE		BIT(0)
#define EXYNOS9610_TRSV_CDR_CTRL		0x3c

/* Calibration for phy initialization */
static const struct samsung_ufs_phy_cfg exynos9610_pre_init_cfg[] = {
	PHY_COMN_REG_CFG(0x023, 0x80, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x01d, 0x10, PWR_MODE_ANY),
	EXYNOS9610_TRSV_REG_CFG(0x044, 0xb5, PWR_MODE_ANY),
	EXYNOS9610_TRSV_REG_CFG(0x04d, 0x43, PWR_MODE_ANY),
	EXYNOS9610_TRSV_REG_CFG(0x05b, 0x20, PWR_MODE_ANY),
	EXYNOS9610_TRSV_REG_CFG(0x05e, 0xc0, PWR_MODE_ANY),
	EXYNOS9610_TRSV_REG_CFG(0x06c, 0x94, PWR_MODE_ANY),
	EXYNOS9610_TRSV_REG_CFG(0x038, 0x12, PWR_MODE_ANY),
	EXYNOS9610_TRSV_REG_CFG(0x059, 0x58, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x023, 0xc0, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x023, 0x00, PWR_MODE_ANY),
	END_UFS_PHY_CFG
};

/*
 * Calibration for HS mode series A/B (the same values for both). The CAL has
 * per-gear values for 0x128/0x12c/0x134, but samsung_ufs_phy_calibrate()
 * applies every entry regardless of its description, so only the HS-G3 ones
 * (the gear this host negotiates) are listed.
 */
static const struct samsung_ufs_phy_cfg exynos9610_pre_pwr_hs_cfg[] = {
	EXYNOS9610_TRSV_REG_CFG(0x032, 0xbc, PWR_MODE_HS_ANY),
	EXYNOS9610_TRSV_REG_CFG(0x03c, 0x7f, PWR_MODE_HS_ANY),
	EXYNOS9610_TRSV_REG_CFG(0x048, 0xc0, PWR_MODE_HS_ANY),
	EXYNOS9610_TRSV_REG_CFG(0x04a, 0x00, PWR_MODE_HS_G3_ANY),
	EXYNOS9610_TRSV_REG_CFG(0x04b, 0x10, PWR_MODE_HS_G3_ANY),
	EXYNOS9610_TRSV_REG_CFG(0x04d, 0x63, PWR_MODE_HS_G3_ANY),
	END_UFS_PHY_CFG
};

/* Nothing to write after PMC, but the table must exist for the CDR wait */
static const struct samsung_ufs_phy_cfg exynos9610_post_pwr_hs_cfg[] = {
	END_UFS_PHY_CFG
};

static const struct samsung_ufs_phy_cfg *exynos9610_ufs_phy_cfgs[CFG_TAG_MAX] = {
	[CFG_PRE_INIT]		= exynos9610_pre_init_cfg,
	[CFG_PRE_PWR_HS]	= exynos9610_pre_pwr_hs_cfg,
	[CFG_POST_PWR_HS]	= exynos9610_post_pwr_hs_cfg,
};

static const struct samsung_ufs_phy_cfg exynos9610_post_h8_enter_cfg[] = {
	EXYNOS9610_TRSV_REG_CFG(0x031, 0x99, PWR_MODE_ANY),
	EXYNOS9610_TRSV_REG_CFG(0x03a, 0x7f, PWR_MODE_ANY),
	EXYNOS9610_TRSV_REG_CFG(0x03c, 0x7f, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x001, 0x02, PWR_MODE_ANY),
	END_UFS_PHY_CFG
};

static const struct samsung_ufs_phy_cfg exynos9610_pre_h8_exit_cfg[] = {
	PHY_COMN_REG_CFG(0x001, 0x3f, PWR_MODE_ANY),
	EXYNOS9610_TRSV_REG_CFG(0x031, 0xd9, PWR_MODE_ANY),
	EXYNOS9610_TRSV_REG_CFG(0x03a, 0x77, PWR_MODE_ANY),
	EXYNOS9610_TRSV_REG_CFG(0x03c, 0xff, PWR_MODE_ANY),
	END_UFS_PHY_CFG
};

static const struct samsung_ufs_phy_cfg *exynos9610_ufs_phy_cfgs_hibern8[] = {
	[CFG_POST_HIBERN8_ENTER]	= exynos9610_post_h8_enter_cfg,
	[CFG_PRE_HIBERN8_EXIT]		= exynos9610_pre_h8_exit_cfg,
};

/*
 * The CAL's "CDR AFC wait": poll the AFC status bit and kick the CDR (0xf0
 * low then high) until it is set. After a power mode change the done bit is
 * bit 6, on HIBERN8 exit it is bit 0; only the former runs in the
 * CFG_POST_PWR_HS state.
 */
static int exynos9610_ufs_phy_wait_cdr_afc(struct phy *phy, u8 lane)
{
	struct samsung_ufs_phy *ufs_phy = get_samsung_ufs_phy(phy);
	void __iomem *trsv = ufs_phy->reg_pma +
			     PHY_APB_ADDR(lane * EXYNOS9610_TRSV_CH_OFFSET);
	u32 mask = ufs_phy->ufs_phy_state == CFG_POST_PWR_HS ?
		   EXYNOS9610_TRSV_CDR_AFC_PMC_DONE :
		   EXYNOS9610_TRSV_CDR_AFC_H8_DONE;
	int i;

	for (i = 0; i < 100; i++) {
		usleep_range(40, 50);
		if (readl(trsv + PHY_APB_ADDR(EXYNOS9610_TRSV_CDR_AFC)) & mask)
			return 0;
		udelay(1);
		writel(0x7f, trsv + PHY_APB_ADDR(EXYNOS9610_TRSV_CDR_CTRL));
		writel(0xff, trsv + PHY_APB_ADDR(EXYNOS9610_TRSV_CDR_CTRL));
	}

	dev_err(ufs_phy->dev, "lane %u: CDR AFC did not finish\n", lane);
	return -ETIMEDOUT;
}

/* The CAL waits 200 us after the PMA init sequence */
static int exynos9610_ufs_phy_wait_for_cal(struct phy *phy, u8 lane)
{
	udelay(200);
	return 0;
}

const struct samsung_ufs_phy_drvdata exynos9610_ufs_phy = {
	.cfgs = exynos9610_ufs_phy_cfgs,
	.cfgs_hibern8 = exynos9610_ufs_phy_cfgs_hibern8,
	.isol = {
		.offset = EXYNOS9610_UFS_PHY_CTRL,
		.mask = EXYNOS9610_UFS_PHY_CTRL_MASK,
		.en = EXYNOS9610_UFS_PHY_CTRL_EN,
	},
	.wait_for_cal = exynos9610_ufs_phy_wait_for_cal,
	.wait_for_cdr = exynos9610_ufs_phy_wait_cdr_afc,
};
