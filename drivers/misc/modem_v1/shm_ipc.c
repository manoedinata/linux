/*
 * Copyright (C) 2014 Samsung Electronics Co.Ltd
 * http://www.samsung.com
 *
 * Shared memory driver
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
*/

#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/shm_ipc.h>
#include <linux/of_reserved_mem.h>

struct shm_plat_data {
	unsigned long p_addr;
	unsigned t_size;
	unsigned ipc_off;
	unsigned ipc_size;

	unsigned long p_sysram_addr;
	unsigned t_sysram_size;
} pdata;

unsigned long shm_get_phys_base(void)
{
	return pdata.p_addr;
}

unsigned shm_get_phys_size(void)
{
	return pdata.t_size;
}

unsigned long shm_get_sysram_base(void)
{
	return pdata.p_sysram_addr;
}

unsigned shm_get_sysram_size(void)
{
	return pdata.t_sysram_size;
}

unsigned shm_get_boot_size(void)
{
	return pdata.ipc_off;
}

unsigned shm_get_ipc_rgn_offset(void)
{
	return pdata.ipc_off;
}

unsigned shm_get_ipc_rgn_size(void)
{
	return pdata.ipc_size;
}

int shm_get_security_param3(unsigned long mode, u32 main_size, unsigned long *param)
{
	int ret = 0;

	switch (mode) {
	case 0: /* CP_BOOT_MODE_NORMAL */
		*param = main_size;
		break;
	case 1: /* CP_BOOT_MODE_DUMP */
#ifdef CP_NONSECURE_BOOT
		*param = pdata.p_addr;
#else
		*param = pdata.p_addr + pdata.ipc_off;
#endif
		break;
	case 2: /* CP_BOOT_RE_INIT */
		*param = 0;
		break;
	default:
		pr_info("%s: Invalid sec_mode(%lu)\n", __func__, mode);
		ret = -EINVAL;
		break;
	}

	return ret;
}

int shm_get_security_param2(unsigned long mode, u32 bl_size, unsigned long *param)
{
	int ret = 0;

	switch (mode) {
	case 0: /* CP_BOOT_MODE_NORMAL */
	case 1: /* CP_BOOT_MODE_DUMP */
		*param = bl_size;
		break;
	case 2: /* CP_BOOT_RE_INIT */
		*param = 0;
		break;
	default:
		pr_info("%s: Invalid sec_mode(%lu)\n", __func__, mode);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*
 * The carveout is a no-map reserved region: it has no linear mapping and
 * possibly no struct pages, so it is mapped like device memory (write
 * combining, as downstream's vmap with pgprot_writecombine) rather than
 * through vmap() of its pages.
 */
void __iomem *shm_request_region(unsigned long sh_addr, unsigned size)
{
	if (!sh_addr)
		return NULL;

	return ioremap_wc(sh_addr, size);
}


void __iomem *shm_get_boot_region(void)
{
	return shm_request_region(pdata.p_addr, pdata.ipc_off);
}

void __iomem *shm_get_ipc_region(void)
{
	return shm_request_region(pdata.p_addr + pdata.ipc_off,
			pdata.t_size - pdata.ipc_off);
}

void shm_release_region(void *v_addr)
{
	iounmap((void __iomem *)v_addr);
}

static int shm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;

	dev_err(dev, "%s: shmem driver init\n", __func__);

	if (dev->of_node) {
		struct device_node *rmem_np;
		struct reserved_mem *rmem;

		/* as a module the RESERVEDMEM_OF_DECLARE hook never runs */
		rmem_np = of_parse_phandle(dev->of_node, "memory-region", 0);
		rmem = rmem_np ? of_reserved_mem_lookup(rmem_np) : NULL;
		of_node_put(rmem_np);
		if (!rmem) {
			dev_err(dev, "no memory-region\n");
			return -EINVAL;
		}
		pdata.p_addr = rmem->base;
		pdata.t_size = rmem->size;

		ret = of_property_read_u32(dev->of_node, "shmem,ipc_offset",
				&pdata.ipc_off);
		if (ret) {
			dev_err(dev, "failed to get property, ipc_offset\n");
			return -EINVAL;
		}

		ret = of_property_read_u32(dev->of_node, "shmem,ipc_size",
				&pdata.ipc_size);
		if (ret) {
			dev_err(dev, "failed to get property, ipc_size\n");
			return -EINVAL;
		}
	} else {
		/* To do: In case of non-DT */
	}

	dev_err(dev, "paddr=%lu, t_size=%u, ipc_off=%u, ipc_size=%u\n",
		pdata.p_addr, pdata.t_size, pdata.ipc_off, pdata.ipc_size);

	return 0;
}

static void shm_remove(struct platform_device *pdev)
{
}

static const struct of_device_id exynos_shm_dt_match[] = {
		{ .compatible = "samsung,exynos7580-shm_ipc", },
		{ .compatible = "samsung,exynos8890-shm_ipc", },
		{ .compatible = "samsung,exynos7870-shm_ipc", },
		{ .compatible = "samsung,exynos-shm_ipc", },
		{},
};
MODULE_DEVICE_TABLE(of, exynos_shm_dt_match);

struct platform_driver shmem_driver = {
	.probe		= shm_probe,
	.remove		= shm_remove,
	.driver		= {
		.name = "shm_ipc",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(exynos_shm_dt_match),
		.suppress_bind_attrs = true,
	},
};
/* registered by modem_main.c, the module has one init */
