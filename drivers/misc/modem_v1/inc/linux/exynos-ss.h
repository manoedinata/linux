/* SPDX-License-Identifier: GPL-2.0 */
/* no Exynos debug snapshot on mainline */
#ifndef MODEM_IF_EXYNOS_SS_H
#define MODEM_IF_EXYNOS_SS_H
static inline int exynos_ss_set_enable(const char *name, int en)
{
	return 0;
}
#endif
