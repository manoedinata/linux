/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Build glue for the downstream (3.18) Samsung SCSC "maxwell" core (the
 * WLBT WiFi/BT subsystem of the Exynos7570) on mainline: replacements for
 * kernel interfaces that are gone or no longer exported.
 */
#ifndef SCSC_COMPAT_H
#define SCSC_COMPAT_H

#include <linux/bitmap.h>
#include <linux/printk.h>
#include <linux/stdarg.h>

/* printk_emit() is gone; vprintk_emit() is still exported */
static inline __printf(5, 6)
int printk_emit(int facility, int level, const char *dict, size_t dictlen,
		const char *fmt, ...)
{
	va_list args;
	int r;

	va_start(args, fmt);
	r = vprintk_emit(facility, level, NULL, fmt, args);
	va_end(args);
	return r;
}

/* arm64 is little endian */
#define bitmap_copy_le(dst, src, nbits)	bitmap_copy(dst, src, nbits)

#endif
