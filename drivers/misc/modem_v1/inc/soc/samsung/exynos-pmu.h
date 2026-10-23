/* SPDX-License-Identifier: GPL-2.0 */
/* downstream exynos_pmu_read/write over the mainline PMU syscon */
#ifndef MODEM_IF_EXYNOS_PMU_H
#define MODEM_IF_EXYNOS_PMU_H

#include <linux/err.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

static inline struct regmap *modem_if_pmu(void)
{
	return syscon_regmap_lookup_by_compatible("samsung,exynos7570-pmu");
}

static inline int exynos_pmu_read(unsigned int offset, unsigned int *val)
{
	struct regmap *pmu = modem_if_pmu();

	return IS_ERR(pmu) ? PTR_ERR(pmu) : regmap_read(pmu, offset, val);
}

static inline int exynos_pmu_write(unsigned int offset, unsigned int val)
{
	struct regmap *pmu = modem_if_pmu();

	return IS_ERR(pmu) ? PTR_ERR(pmu) : regmap_write(pmu, offset, val);
}

static inline int exynos_pmu_update(unsigned int offset, unsigned int mask, unsigned int val)
{
	struct regmap *pmu = modem_if_pmu();

	return IS_ERR(pmu) ? PTR_ERR(pmu) : regmap_update_bits(pmu, offset, mask, val);
}

#endif
