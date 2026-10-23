/* SPDX-License-Identifier: GPL-2.0 */
/* The J4 takes no modem GPIOs from the DT; legacy of_gpio is gone from mainline */
#ifndef MODEM_IF_OF_GPIO_H
#define MODEM_IF_OF_GPIO_H

#include <linux/gpio.h>
#include <linux/of.h>

static inline int of_get_named_gpio(const struct device_node *np, const char *name, int index)
{
	return -ENOENT;
}

#endif
