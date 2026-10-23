/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Build glue for the downstream (3.18) Samsung modem interface on mainline:
 * the downstream Kconfig choices for the Galaxy J4 (SM-J400F, Exynos7570,
 * Shannon SS305AP over shared memory) and replacements for downstream-only
 * kernel interfaces.
 */
#ifndef MODEM_IF_COMPAT_H
#define MODEM_IF_COMPAT_H

/* downstream .config of the J4 */
#define CONFIG_LINK_DEVICE_SHMEM	1
#define CONFIG_UMTS_MODEM_SS310AP	1
#define CONFIG_SEC_MODEM_SS310AP	1
#define CONFIG_CP_SECURE_BOOT		1
#define CONFIG_SOC_EXYNOS7570		1
#define CONFIG_HW_REV_DETECT		1
#define CONFIG_MCU_IPC			1
#define CONFIG_SHM_IPC			1
/* downstream Makefile: -DCONFIG_OPTION_REGION=\"$(PROJECT_REGION)\"; the J4 build is
 * neither of the option regions in the pdata (kor_skt, usa_att) */
#define CONFIG_OPTION_REGION		"open"


#include <linux/arm-smccc.h>
#include <linux/pm_qos.h>
#include <linux/sched/clock.h>
#include <linux/timer.h>

/* renamed or removed since 3.18 */
#define from_timer(var, t, field)	timer_container_of(var, t, field)
#define del_timer(t)			timer_delete(t)
#define del_timer_sync(t)		timer_delete_sync(t)

/*
 * Samsung's pm_qos classes (CPU cluster / bus / device throughput minimums
 * while data flows) do not exist on mainline; the requests are dropped.
 */
#define PM_QOS_CLUSTER0_FREQ_MIN		0
#define PM_QOS_CLUSTER1_FREQ_MIN		0
#define PM_QOS_BUS_THROUGHPUT			0
#define PM_QOS_BUS_THROUGHPUT_MAX		0
#define PM_QOS_DEVICE_THROUGHPUT		0
#define PM_QOS_BUS_THROUGHPUT_MAX_DEFAULT_VALUE	0
#define pm_qos_add_request(req, class, val)	do { } while (0)
#define pm_qos_update_request(req, val)		do { } while (0)
#define pm_qos_remove_request(req)		do { } while (0)
#define pm_qos_request_active(req)		0

/* the legacy GPIO sysfs export is gone; the J4 exports no modem GPIOs */
#define gpio_export(gpio, dir)			(-ENOSYS)
#define gpio_export_link(dev, name, gpio)	(-ENOSYS)

/* downstream exynos_smc(): a plain smc #0, x0 is the result */
static inline int exynos_smc(unsigned long cmd, unsigned long arg1,
			     unsigned long arg2, unsigned long arg3)
{
	struct arm_smccc_res res;

	arm_smccc_smc(cmd, arg1, arg2, arg3, 0, 0, 0, 0, &res);
	return (int)res.a0;
}

#endif
