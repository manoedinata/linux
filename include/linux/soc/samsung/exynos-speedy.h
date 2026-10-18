/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Samsung Exynos SPEEDY (Serial Protocol in an EffEctive Digital waY)
 * one-wire PMIC bus
 */

#ifndef __LINUX_SOC_SAMSUNG_EXYNOS_SPEEDY_H
#define __LINUX_SOC_SAMSUNG_EXYNOS_SPEEDY_H

struct device;
struct regmap;

/**
 * devm_exynos_speedy_regmap - 8-bit register map of one SPEEDY slave block
 * @dev: child device sitting below the SPEEDY controller node
 * @blkid: 4-bit block id, forms bits 11:8 of the 12-bit SPEEDY address
 *
 * A SPEEDY slave exposes up to 16 blocks of 256 registers each.
 */
struct regmap *devm_exynos_speedy_regmap(struct device *dev, unsigned int blkid);

#endif
