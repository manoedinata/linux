/*
 * Copyright (C) 2010 Samsung Electronics.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __MODEM_VARIATION_H__
#define __MODEM_VARIATION_H__

#include "modem_prj.h"

#define DECLARE_MODEM_INIT(type)					\
	int type ## _init_modemctl_device(				\
				struct modem_ctl *mc,			\
				struct modem_data *pdata)

#define DECLARE_MODEM_INIT_DUMMY(type)					\
	DECLARE_MODEM_INIT(type) { return 0; }

#define DECLARE_LINK_INIT(type)						\
	struct link_device *type ## _create_link_device(		\
				struct platform_device *pdev)

#define DECLARE_LINK_INIT_DUMMY(type)					\
	DECLARE_LINK_INIT(type) { return NULL; }

#define MODEM_INIT_CALL(type)	type ## _init_modemctl_device

#define LINK_INIT_CALL(type)	type ## _create_link_device

/**
 * Add extern declaration of modem & link type (all of them, so that the
 * DUMMY definitions in modem_variation.c have a prototype too)
 * (CAUTION!!! Every DUMMY function must be declared in modem_variation.c)
 */

/* modem device support */
DECLARE_MODEM_INIT(dummy);
DECLARE_MODEM_INIT(xmm6260);

DECLARE_MODEM_INIT(xmm6262);

DECLARE_MODEM_INIT(xmm7260);

DECLARE_MODEM_INIT(cbp71);

DECLARE_MODEM_INIT(cbp72);

DECLARE_MODEM_INIT(cbp82);

DECLARE_MODEM_INIT(cmc220);

DECLARE_MODEM_INIT(cmc221);

DECLARE_MODEM_INIT(sh222ap);

DECLARE_MODEM_INIT(ss310ap);

DECLARE_MODEM_INIT(ss222);

DECLARE_MODEM_INIT(ss300);

DECLARE_MODEM_INIT(mdm6600);

DECLARE_MODEM_INIT(esc6270);

DECLARE_MODEM_INIT(qsc6085);

DECLARE_MODEM_INIT(sprd8803);

/* link device support */
DECLARE_LINK_INIT(undefined);
DECLARE_LINK_INIT(mipi);

DECLARE_LINK_INIT(usb);

DECLARE_LINK_INIT(hsic);

DECLARE_LINK_INIT(dpram);

DECLARE_LINK_INIT(pld);

DECLARE_LINK_INIT(c2c);

DECLARE_LINK_INIT(lli);

DECLARE_LINK_INIT(shmem);

DECLARE_LINK_INIT(spi);

typedef int (*modem_init_call)(struct modem_ctl *, struct modem_data *);
typedef struct link_device *(*link_init_call)(struct platform_device *);

int call_modem_init_func(struct modem_ctl *mc, struct modem_data *pdata);

struct link_device *call_link_init_func(struct platform_device *pdev,
					       enum modem_link link_type);

#endif

