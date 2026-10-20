// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2016 Samsung Electronics Co., Ltd.
 *
 * Common Clock Framework support for Exynos7570.
 *
 * Register layout, parents and bit positions follow the Samsung downstream
 * "pwrcal" tables for S5E7570. Muxes are modelled by their select field only;
 * the per-mux enable bit (bit 21) is left as the bootloader set it.
 *
 * The USB20 PHY core clock comes from the 20 MHz USB PLL (WPLL), which sits
 * behind the WLBT block. It is registered by hand below and enabled by the AP,
 * the way the downstream kernel does when WLBT firmware is not using it.
 */

#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <dt-bindings/clock/samsung,exynos7570-cmu.h>

#include "clk.h"
#include "clk-exynos-arm64.h"

/*
 * Register offsets for CMU_MIF (0x10460000)
 */
#define SHARED0_PLL_LOCK                                 0x0000
#define SHARED1_PLL_LOCK                                 0x0020
#define SHARED2_PLL_LOCK                                 0x0040
#define SHARED0_PLL_CON0                                 0x0100
#define SHARED1_PLL_CON0                                 0x0120
#define SHARED2_PLL_CON0                                 0x0140
#define CLK_CON_MUX_SHARED0_PLL                          0x0200
#define CLK_CON_MUX_SHARED1_PLL                          0x0204
#define CLK_CON_MUX_SHARED2_PLL                          0x0208
#define CLK_CON_MUX_CLK_MIF_BUSD                         0x0220
#define CLK_CON_MUX_CLKCMU_DISPAUD_BUS                   0x0234
#define CLK_CON_MUX_CLKCMU_DISPAUD_DECON_INT_VCLK        0x0238
#define CLK_CON_MUX_CLKCMU_FSYS_BUS                      0x0248
#define CLK_CON_MUX_CLKCMU_FSYS_MMC0                     0x024c
#define CLK_CON_MUX_CLKCMU_FSYS_MMC2                     0x0250
#define CLK_CON_MUX_CLKCMU_FSYS_USB20DRD_REFCLK          0x0254
#define CLK_CON_MUX_CLKCMU_PERI_BUS                      0x0258
#define CLK_CON_MUX_CLKCMU_PERI_UART_DEBUG               0x0260
#define CLK_CON_MUX_CLKCMU_PERI_UART_SENSOR              0x0264
#define CLK_CON_MUX_CLKCMU_PERI_SPI_REARFROM             0x026c
#define CLK_CON_MUX_CLKCMU_PERI_SPI_ESE                  0x0270
#define CLK_CON_MUX_CLKCMU_PERI_USI_0                    0x0274
#define CLK_CON_MUX_CLKCMU_PERI_USI_1                    0x0278
#define CLK_CON_DIV_CLK_MIF_BUSD                         0x0414
#define CLK_CON_DIV_CLK_MIF_APB                          0x0418
#define CLK_CON_DIV_CLKCMU_DISPAUD_BUS                   0x0430
#define CLK_CON_DIV_CLKCMU_DISPAUD_DECON_INT_VCLK        0x0434
#define CLK_CON_DIV_CLKCMU_FSYS_BUS                      0x0444
#define CLK_CON_DIV_CLKCMU_FSYS_MMC0                     0x0448
#define CLK_CON_DIV_CLKCMU_FSYS_MMC2                     0x044c
#define CLK_CON_DIV_CLKCMU_FSYS_USB20DRD_REFCLK          0x0450
#define CLK_CON_DIV_CLKCMU_PERI_BUS                      0x0454
#define CLK_CON_DIV_CLKCMU_PERI_UART_DEBUG               0x045c
#define CLK_CON_DIV_CLKCMU_PERI_UART_SENSOR              0x0460
#define CLK_CON_DIV_CLKCMU_PERI_SPI_REARFROM             0x0468
#define CLK_CON_DIV_CLKCMU_PERI_SPI_ESE                  0x046c
#define CLK_CON_DIV_CLKCMU_PERI_USI_0                    0x0470
#define CLK_CON_DIV_CLKCMU_PERI_USI_1                    0x0474
#define CLK_ENABLE_CLK_MIF_OSCCLK                        0x0800
#define CLK_ENABLE_CLK_MIF_BUSD                          0x0810
#define CLK_ENABLE_CLK_MIF_APB                           0x0814
#define CLK_ENABLE_CLKCMU_DISPAUD_BUS                    0x083c
#define CLK_ENABLE_CLKCMU_DISPAUD_DECON_INT_VCLK         0x0840
#define CLK_ENABLE_CLKCMU_FSYS_BUS                       0x0850
#define CLK_ENABLE_CLKCMU_FSYS_MMC0                      0x0854
#define CLK_ENABLE_CLKCMU_FSYS_MMC2                      0x0858
#define CLK_ENABLE_CLKCMU_FSYS_USB20DRD_REFCLK           0x085c
#define CLK_ENABLE_CLKCMU_PERI_BUS                       0x0860
#define CLK_ENABLE_CLKCMU_PERI_UART_DEBUG                0x0868
#define CLK_ENABLE_CLKCMU_PERI_UART_SENSOR               0x086c
#define CLK_ENABLE_CLKCMU_PERI_SPI_REARFROM              0x0874
#define CLK_ENABLE_CLKCMU_PERI_SPI_ESE                   0x0878
#define CLK_ENABLE_CLKCMU_PERI_USI_0                     0x087c
#define CLK_ENABLE_CLKCMU_PERI_USI_1                     0x0880
#define CLK_ENABLE_CLK_MIF_TCXO                          0x089c

static const unsigned long mif_clk_regs[] __initconst = {
	SHARED0_PLL_LOCK,
	SHARED1_PLL_LOCK,
	SHARED2_PLL_LOCK,
	SHARED0_PLL_CON0,
	SHARED1_PLL_CON0,
	SHARED2_PLL_CON0,
	CLK_CON_MUX_SHARED0_PLL,
	CLK_CON_MUX_SHARED1_PLL,
	CLK_CON_MUX_SHARED2_PLL,
	CLK_CON_MUX_CLK_MIF_BUSD,
	CLK_CON_MUX_CLKCMU_DISPAUD_BUS,
	CLK_CON_MUX_CLKCMU_DISPAUD_DECON_INT_VCLK,
	CLK_CON_MUX_CLKCMU_FSYS_BUS,
	CLK_CON_MUX_CLKCMU_FSYS_MMC0,
	CLK_CON_MUX_CLKCMU_FSYS_MMC2,
	CLK_CON_MUX_CLKCMU_FSYS_USB20DRD_REFCLK,
	CLK_CON_MUX_CLKCMU_PERI_BUS,
	CLK_CON_MUX_CLKCMU_PERI_UART_DEBUG,
	CLK_CON_MUX_CLKCMU_PERI_UART_SENSOR,
	CLK_CON_MUX_CLKCMU_PERI_SPI_REARFROM,
	CLK_CON_MUX_CLKCMU_PERI_SPI_ESE,
	CLK_CON_MUX_CLKCMU_PERI_USI_0,
	CLK_CON_MUX_CLKCMU_PERI_USI_1,
	CLK_CON_DIV_CLK_MIF_BUSD,
	CLK_CON_DIV_CLK_MIF_APB,
	CLK_CON_DIV_CLKCMU_DISPAUD_BUS,
	CLK_CON_DIV_CLKCMU_DISPAUD_DECON_INT_VCLK,
	CLK_CON_DIV_CLKCMU_FSYS_BUS,
	CLK_CON_DIV_CLKCMU_FSYS_MMC0,
	CLK_CON_DIV_CLKCMU_FSYS_MMC2,
	CLK_CON_DIV_CLKCMU_FSYS_USB20DRD_REFCLK,
	CLK_CON_DIV_CLKCMU_PERI_BUS,
	CLK_CON_DIV_CLKCMU_PERI_UART_DEBUG,
	CLK_CON_DIV_CLKCMU_PERI_UART_SENSOR,
	CLK_CON_DIV_CLKCMU_PERI_SPI_REARFROM,
	CLK_CON_DIV_CLKCMU_PERI_SPI_ESE,
	CLK_CON_DIV_CLKCMU_PERI_USI_0,
	CLK_CON_DIV_CLKCMU_PERI_USI_1,
	CLK_ENABLE_CLK_MIF_OSCCLK,
	CLK_ENABLE_CLK_MIF_BUSD,
	CLK_ENABLE_CLK_MIF_APB,
	CLK_ENABLE_CLKCMU_DISPAUD_BUS,
	CLK_ENABLE_CLKCMU_DISPAUD_DECON_INT_VCLK,
	CLK_ENABLE_CLKCMU_FSYS_BUS,
	CLK_ENABLE_CLKCMU_FSYS_MMC0,
	CLK_ENABLE_CLKCMU_FSYS_MMC2,
	CLK_ENABLE_CLKCMU_FSYS_USB20DRD_REFCLK,
	CLK_ENABLE_CLKCMU_PERI_BUS,
	CLK_ENABLE_CLKCMU_PERI_UART_DEBUG,
	CLK_ENABLE_CLKCMU_PERI_UART_SENSOR,
	CLK_ENABLE_CLKCMU_PERI_SPI_REARFROM,
	CLK_ENABLE_CLKCMU_PERI_SPI_ESE,
	CLK_ENABLE_CLKCMU_PERI_USI_0,
	CLK_ENABLE_CLKCMU_PERI_USI_1,
	CLK_ENABLE_CLK_MIF_TCXO,
};

static const struct samsung_fixed_rate_clock mif_fixed_clks[] __initconst = {
	FRATE(CLK_FRAT_MIF_TCXO, "frat_mif_tcxo", NULL, 0, 26000000),
};

static const struct samsung_fixed_factor_clock mif_fixed_factor_clks[] __initconst = {
	FFACTOR(0, "ffac_mif_shared0_pll_div2", "mout_mif_shared0_pll", 1, 2, 0),
	FFACTOR(0, "ffac_mif_shared1_pll_div2", "mout_mif_shared1_pll", 1, 2, 0),
	FFACTOR(0, "ffac_mif_shared2_pll_div2", "mout_mif_shared2_pll", 1, 2, 0),
};

static const struct samsung_pll_clock mif_pll_clks[] __initconst = {
	PLL(pll_1417x, CLK_FOUT_MIF_SHARED0_PLL, "fout_mif_shared0_pll", "oscclk", SHARED0_PLL_LOCK, SHARED0_PLL_CON0, NULL),
	PLL(pll_1417x, CLK_FOUT_MIF_SHARED1_PLL, "fout_mif_shared1_pll", "oscclk", SHARED1_PLL_LOCK, SHARED1_PLL_CON0, NULL),
	PLL(pll_1417x, CLK_FOUT_MIF_SHARED2_PLL, "fout_mif_shared2_pll", "oscclk", SHARED2_PLL_LOCK, SHARED2_PLL_CON0, NULL),
};

/* List of parent clocks for muxes in CMU_MIF */
PNAME(mout_mif_shared0_pll_p) = { "oscclk", "fout_mif_shared0_pll" };
PNAME(mout_mif_shared1_pll_p) = { "oscclk", "fout_mif_shared1_pll" };
PNAME(mout_mif_shared2_pll_p) = { "oscclk", "fout_mif_shared2_pll" };
PNAME(mout_mif_busd_p) = { "mout_mif_shared0_pll", "ffac_mif_shared1_pll_div2", "ffac_mif_shared2_pll_div2" };
PNAME(mout_mif_cmu_fsys_bus_p) = { "ffac_mif_shared0_pll_div2", "ffac_mif_shared1_pll_div2" };
PNAME(mout_mif_cmu_fsys_mmc0_p) = { "ffac_mif_shared0_pll_div2", "ffac_mif_shared2_pll_div2" };
PNAME(mout_mif_cmu_fsys_mmc2_p) = { "ffac_mif_shared0_pll_div2", "ffac_mif_shared2_pll_div2" };
PNAME(mout_mif_cmu_fsys_usb20drd_refclk_p) = { "ffac_mif_shared0_pll_div2", "ffac_mif_shared1_pll_div2" };
PNAME(mout_mif_cmu_peri_bus_p) = { "ffac_mif_shared0_pll_div2", "ffac_mif_shared1_pll_div2" };
PNAME(mout_mif_cmu_peri_uart_debug_p) = { "ffac_mif_shared0_pll_div2", "ffac_mif_shared1_pll_div2" };
PNAME(mout_mif_cmu_peri_uart_sensor_p) = { "ffac_mif_shared0_pll_div2", "ffac_mif_shared1_pll_div2" };
PNAME(mout_mif_cmu_peri_spi_rearfrom_p) = { "ffac_mif_shared0_pll_div2", "oscclk" };
PNAME(mout_mif_cmu_peri_spi_ese_p) = { "ffac_mif_shared0_pll_div2", "oscclk" };
PNAME(mout_mif_cmu_peri_usi_0_p) = { "ffac_mif_shared0_pll_div2", "ffac_mif_shared1_pll_div2", "oscclk" };
PNAME(mout_mif_cmu_peri_usi_1_p) = { "ffac_mif_shared0_pll_div2", "ffac_mif_shared1_pll_div2", "oscclk" };
PNAME(mout_mif_cmu_dispaud_bus_p) = { "ffac_mif_shared0_pll_div2", "ffac_mif_shared1_pll_div2" };
PNAME(mout_mif_cmu_dispaud_decon_int_vclk_p) = { "mout_mif_shared0_pll", "mout_mif_shared1_pll", "mout_mif_shared2_pll" };

static const struct samsung_mux_clock mif_mux_clks[] __initconst = {
	MUX(CLK_MOUT_MIF_SHARED0_PLL, "mout_mif_shared0_pll", mout_mif_shared0_pll_p, CLK_CON_MUX_SHARED0_PLL, 12, 1),
	MUX(CLK_MOUT_MIF_SHARED1_PLL, "mout_mif_shared1_pll", mout_mif_shared1_pll_p, CLK_CON_MUX_SHARED1_PLL, 12, 1),
	MUX(CLK_MOUT_MIF_SHARED2_PLL, "mout_mif_shared2_pll", mout_mif_shared2_pll_p, CLK_CON_MUX_SHARED2_PLL, 12, 1),
	MUX(CLK_MOUT_MIF_BUSD, "mout_mif_busd", mout_mif_busd_p, CLK_CON_MUX_CLK_MIF_BUSD, 12, 2),
	MUX(CLK_MOUT_MIF_CMU_FSYS_BUS, "mout_mif_cmu_fsys_bus", mout_mif_cmu_fsys_bus_p, CLK_CON_MUX_CLKCMU_FSYS_BUS, 12, 1),
	MUX(CLK_MOUT_MIF_CMU_FSYS_MMC0, "mout_mif_cmu_fsys_mmc0", mout_mif_cmu_fsys_mmc0_p, CLK_CON_MUX_CLKCMU_FSYS_MMC0, 12, 1),
	MUX(CLK_MOUT_MIF_CMU_FSYS_MMC2, "mout_mif_cmu_fsys_mmc2", mout_mif_cmu_fsys_mmc2_p, CLK_CON_MUX_CLKCMU_FSYS_MMC2, 12, 1),
	MUX(CLK_MOUT_MIF_CMU_FSYS_USB20DRD_REFCLK, "mout_mif_cmu_fsys_usb20drd_refclk", mout_mif_cmu_fsys_usb20drd_refclk_p, CLK_CON_MUX_CLKCMU_FSYS_USB20DRD_REFCLK, 12, 1),
	MUX(CLK_MOUT_MIF_CMU_PERI_BUS, "mout_mif_cmu_peri_bus", mout_mif_cmu_peri_bus_p, CLK_CON_MUX_CLKCMU_PERI_BUS, 12, 1),
	MUX(CLK_MOUT_MIF_CMU_PERI_UART_DEBUG, "mout_mif_cmu_peri_uart_debug", mout_mif_cmu_peri_uart_debug_p, CLK_CON_MUX_CLKCMU_PERI_UART_DEBUG, 12, 1),
	MUX(CLK_MOUT_MIF_CMU_PERI_UART_SENSOR, "mout_mif_cmu_peri_uart_sensor", mout_mif_cmu_peri_uart_sensor_p, CLK_CON_MUX_CLKCMU_PERI_UART_SENSOR, 12, 1),
	MUX(CLK_MOUT_MIF_CMU_PERI_SPI_REARFROM, "mout_mif_cmu_peri_spi_rearfrom", mout_mif_cmu_peri_spi_rearfrom_p, CLK_CON_MUX_CLKCMU_PERI_SPI_REARFROM, 12, 1),
	MUX(CLK_MOUT_MIF_CMU_PERI_SPI_ESE, "mout_mif_cmu_peri_spi_ese", mout_mif_cmu_peri_spi_ese_p, CLK_CON_MUX_CLKCMU_PERI_SPI_ESE, 12, 1),
	MUX(CLK_MOUT_MIF_CMU_PERI_USI_0, "mout_mif_cmu_peri_usi_0", mout_mif_cmu_peri_usi_0_p, CLK_CON_MUX_CLKCMU_PERI_USI_0, 12, 2),
	MUX(CLK_MOUT_MIF_CMU_PERI_USI_1, "mout_mif_cmu_peri_usi_1", mout_mif_cmu_peri_usi_1_p, CLK_CON_MUX_CLKCMU_PERI_USI_1, 12, 2),
	MUX(CLK_MOUT_MIF_CMU_DISPAUD_BUS, "mout_mif_cmu_dispaud_bus", mout_mif_cmu_dispaud_bus_p, CLK_CON_MUX_CLKCMU_DISPAUD_BUS, 12, 1),
	MUX(CLK_MOUT_MIF_CMU_DISPAUD_DECON_INT_VCLK, "mout_mif_cmu_dispaud_decon_int_vclk", mout_mif_cmu_dispaud_decon_int_vclk_p, CLK_CON_MUX_CLKCMU_DISPAUD_DECON_INT_VCLK, 12, 2),
};

static const struct samsung_div_clock mif_div_clks[] __initconst = {
	DIV(CLK_DOUT_MIF_BUSD, "dout_mif_busd", "mout_mif_busd", CLK_CON_DIV_CLK_MIF_BUSD, 0, 4),
	DIV(CLK_DOUT_MIF_APB, "dout_mif_apb", "dout_mif_busd", CLK_CON_DIV_CLK_MIF_APB, 0, 2),
	DIV(CLK_DOUT_MIF_CMU_FSYS_BUS, "dout_mif_cmu_fsys_bus", "mout_mif_cmu_fsys_bus", CLK_CON_DIV_CLKCMU_FSYS_BUS, 0, 4),
	DIV(CLK_DOUT_MIF_CMU_FSYS_MMC0, "dout_mif_cmu_fsys_mmc0", "mout_mif_cmu_fsys_mmc0", CLK_CON_DIV_CLKCMU_FSYS_MMC0, 0, 10),
	DIV(CLK_DOUT_MIF_CMU_FSYS_MMC2, "dout_mif_cmu_fsys_mmc2", "mout_mif_cmu_fsys_mmc2", CLK_CON_DIV_CLKCMU_FSYS_MMC2, 0, 10),
	DIV(CLK_DOUT_MIF_CMU_FSYS_USB20DRD_REFCLK, "dout_mif_cmu_fsys_usb20drd_refclk", "mout_mif_cmu_fsys_usb20drd_refclk", CLK_CON_DIV_CLKCMU_FSYS_USB20DRD_REFCLK, 0, 4),
	DIV(CLK_DOUT_MIF_CMU_PERI_BUS, "dout_mif_cmu_peri_bus", "mout_mif_cmu_peri_bus", CLK_CON_DIV_CLKCMU_PERI_BUS, 0, 4),
	DIV(CLK_DOUT_MIF_CMU_PERI_UART_DEBUG, "dout_mif_cmu_peri_uart_debug", "mout_mif_cmu_peri_uart_debug", CLK_CON_DIV_CLKCMU_PERI_UART_DEBUG, 0, 4),
	DIV(CLK_DOUT_MIF_CMU_PERI_UART_SENSOR, "dout_mif_cmu_peri_uart_sensor", "mout_mif_cmu_peri_uart_sensor", CLK_CON_DIV_CLKCMU_PERI_UART_SENSOR, 0, 4),
	DIV(CLK_DOUT_MIF_CMU_PERI_SPI_REARFROM, "dout_mif_cmu_peri_spi_rearfrom", "mout_mif_cmu_peri_spi_rearfrom", CLK_CON_DIV_CLKCMU_PERI_SPI_REARFROM, 0, 6),
	DIV(CLK_DOUT_MIF_CMU_PERI_SPI_ESE, "dout_mif_cmu_peri_spi_ese", "mout_mif_cmu_peri_spi_ese", CLK_CON_DIV_CLKCMU_PERI_SPI_ESE, 0, 6),
	DIV(CLK_DOUT_MIF_CMU_PERI_USI_0, "dout_mif_cmu_peri_usi_0", "mout_mif_cmu_peri_usi_0", CLK_CON_DIV_CLKCMU_PERI_USI_0, 0, 6),
	DIV(CLK_DOUT_MIF_CMU_PERI_USI_1, "dout_mif_cmu_peri_usi_1", "mout_mif_cmu_peri_usi_1", CLK_CON_DIV_CLKCMU_PERI_USI_1, 0, 6),
	DIV(CLK_DOUT_MIF_CMU_DISPAUD_BUS, "dout_mif_cmu_dispaud_bus", "mout_mif_cmu_dispaud_bus", CLK_CON_DIV_CLKCMU_DISPAUD_BUS, 0, 4),
	DIV(CLK_DOUT_MIF_CMU_DISPAUD_DECON_INT_VCLK, "dout_mif_cmu_dispaud_decon_int_vclk", "mout_mif_cmu_dispaud_decon_int_vclk", CLK_CON_DIV_CLKCMU_DISPAUD_DECON_INT_VCLK, 0, 4),
};

static const struct samsung_gate_clock mif_gate_clks[] __initconst = {
	GATE(CLK_GOUT_MIF_CMU_FSYS_BUS, "gout_mif_cmu_fsys_bus", "dout_mif_cmu_fsys_bus", CLK_ENABLE_CLKCMU_FSYS_BUS, 0, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_MIF_CMU_FSYS_MMC0, "gout_mif_cmu_fsys_mmc0", "dout_mif_cmu_fsys_mmc0", CLK_ENABLE_CLKCMU_FSYS_MMC0, 0, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_MIF_CMU_FSYS_MMC2, "gout_mif_cmu_fsys_mmc2", "dout_mif_cmu_fsys_mmc2", CLK_ENABLE_CLKCMU_FSYS_MMC2, 0, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_MIF_CMU_FSYS_USB20DRD_REFCLK, "gout_mif_cmu_fsys_usb20drd_refclk", "dout_mif_cmu_fsys_usb20drd_refclk", CLK_ENABLE_CLKCMU_FSYS_USB20DRD_REFCLK, 0, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_MIF_CMU_PERI_BUS, "gout_mif_cmu_peri_bus", "dout_mif_cmu_peri_bus", CLK_ENABLE_CLKCMU_PERI_BUS, 0, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_MIF_CMU_PERI_UART_DEBUG, "gout_mif_cmu_peri_uart_debug", "dout_mif_cmu_peri_uart_debug", CLK_ENABLE_CLKCMU_PERI_UART_DEBUG, 0, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_MIF_CMU_PERI_UART_SENSOR, "gout_mif_cmu_peri_uart_sensor", "dout_mif_cmu_peri_uart_sensor", CLK_ENABLE_CLKCMU_PERI_UART_SENSOR, 0, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_MIF_CMU_PERI_SPI_REARFROM, "gout_mif_cmu_peri_spi_rearfrom", "dout_mif_cmu_peri_spi_rearfrom", CLK_ENABLE_CLKCMU_PERI_SPI_REARFROM, 0, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_MIF_CMU_PERI_SPI_ESE, "gout_mif_cmu_peri_spi_ese", "dout_mif_cmu_peri_spi_ese", CLK_ENABLE_CLKCMU_PERI_SPI_ESE, 0, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_MIF_CMU_PERI_USI_0, "gout_mif_cmu_peri_usi_0", "dout_mif_cmu_peri_usi_0", CLK_ENABLE_CLKCMU_PERI_USI_0, 0, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_MIF_CMU_PERI_USI_1, "gout_mif_cmu_peri_usi_1", "dout_mif_cmu_peri_usi_1", CLK_ENABLE_CLKCMU_PERI_USI_1, 0, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_MIF_CMU_DISPAUD_BUS, "gout_mif_cmu_dispaud_bus", "dout_mif_cmu_dispaud_bus", CLK_ENABLE_CLKCMU_DISPAUD_BUS, 0, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_MIF_CMU_DISPAUD_DECON_INT_VCLK, "gout_mif_cmu_dispaud_decon_int_vclk", "dout_mif_cmu_dispaud_decon_int_vclk", CLK_ENABLE_CLKCMU_DISPAUD_DECON_INT_VCLK, 0, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_MIF_PDMA0_ACLK, "gout_mif_pdma0_aclk", "dout_mif_busd", CLK_ENABLE_CLK_MIF_BUSD, 5, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_MIF_SPEEDY_PCLK, "gout_mif_speedy_pclk", "dout_mif_apb", CLK_ENABLE_CLK_MIF_APB, 25, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_MIF_SPEEDY_CLK, "gout_mif_speedy_clk", "frat_mif_tcxo", CLK_ENABLE_CLK_MIF_TCXO, 0, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_MIF_ADC_PCLK_S0, "gout_mif_adc_pclk_s0", "dout_mif_apb", CLK_ENABLE_CLK_MIF_APB, 4, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_MIF_ADC_PCLK_S1, "gout_mif_adc_pclk_s1", "dout_mif_apb", CLK_ENABLE_CLK_MIF_APB, 5, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_MIF_ADC_OSC_SYS, "gout_mif_adc_osc_sys", "oscclk", CLK_ENABLE_CLK_MIF_OSCCLK, 2, 0, 0),
	GATE(CLK_GOUT_MIF_GPIO_PCLK, "gout_mif_gpio_pclk", "dout_mif_apb", CLK_ENABLE_CLK_MIF_APB, 2, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_MIF_SYSREG_PCLK, "gout_mif_sysreg_pclk", "dout_mif_apb", CLK_ENABLE_CLK_MIF_APB, 1, CLK_IS_CRITICAL, 0),
};

static const struct samsung_cmu_info mif_cmu_info __initconst = {
	.fixed_clks	= mif_fixed_clks,
	.nr_fixed_clks	= ARRAY_SIZE(mif_fixed_clks),
	.fixed_factor_clks	= mif_fixed_factor_clks,
	.nr_fixed_factor_clks	= ARRAY_SIZE(mif_fixed_factor_clks),
	.pll_clks	= mif_pll_clks,
	.nr_pll_clks	= ARRAY_SIZE(mif_pll_clks),
	.mux_clks	= mif_mux_clks,
	.nr_mux_clks	= ARRAY_SIZE(mif_mux_clks),
	.div_clks	= mif_div_clks,
	.nr_div_clks	= ARRAY_SIZE(mif_div_clks),
	.gate_clks	= mif_gate_clks,
	.nr_gate_clks	= ARRAY_SIZE(mif_gate_clks),
	.clk_regs		= mif_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(mif_clk_regs),
	.nr_clk_ids		= MIF_NR_CLK,
};

/*
 * Register offsets for CMU_FSYS (0x13730000)
 */
#define CLK_CON_MUX_CLKPHY_FSYS_USB20DRD_PHYCLOCK_USER   0x0230
#define CLK_ENABLE_CLK_FSYS_BUS                          0x0804
#define CLK_ENABLE_CLK_FSYS_MMC_EMBD                     0x0814
#define CLK_ENABLE_CLK_FSYS_MMC_CARD                     0x081c
#define CLK_ENABLE_CLK_FSYS_USB20DRD_REFCLK              0x0828
#define CLK_ENABLE_CLKPHY_FSYS_USB20DRD_PHYCLOCK         0x0830
#define CLK_ENABLE_CLK_FSYS_USB20PHY_CLKCORE             0x083c

static const unsigned long fsys_clk_regs[] __initconst = {
	CLK_CON_MUX_CLKPHY_FSYS_USB20DRD_PHYCLOCK_USER,
	CLK_ENABLE_CLK_FSYS_BUS,
	CLK_ENABLE_CLK_FSYS_MMC_EMBD,
	CLK_ENABLE_CLK_FSYS_MMC_CARD,
	CLK_ENABLE_CLK_FSYS_USB20DRD_REFCLK,
	CLK_ENABLE_CLKPHY_FSYS_USB20DRD_PHYCLOCK,
	CLK_ENABLE_CLK_FSYS_USB20PHY_CLKCORE,
};

static const struct samsung_fixed_rate_clock fsys_fixed_clks[] __initconst = {
	FRATE(CLK_FRAT_FSYS_USB20DRD_PHYCLOCK, "frat_fsys_usb20drd_phyclock", NULL, 0, 60000000),
};

/* List of parent clocks for muxes in CMU_FSYS */
PNAME(mout_fsys_usb20drd_phyclock_user_p) = { "oscclk", "frat_fsys_usb20drd_phyclock" };

static const struct samsung_mux_clock fsys_mux_clks[] __initconst = {
	MUX(CLK_MOUT_FSYS_USB20DRD_PHYCLOCK_USER, "mout_fsys_usb20drd_phyclock_user", mout_fsys_usb20drd_phyclock_user_p, CLK_CON_MUX_CLKPHY_FSYS_USB20DRD_PHYCLOCK_USER, 12, 1),
};

static const struct samsung_gate_clock fsys_gate_clks[] __initconst = {
	GATE(CLK_GOUT_FSYS_MMC_EMBD_ACLK, "gout_fsys_mmc_embd_aclk", "gout_mif_cmu_fsys_bus", CLK_ENABLE_CLK_FSYS_BUS, 8, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_FSYS_MMC_CARD_ACLK, "gout_fsys_mmc_card_aclk", "gout_mif_cmu_fsys_bus", CLK_ENABLE_CLK_FSYS_BUS, 10, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_FSYS_MMC_EMBD_SDCLKIN, "gout_fsys_mmc_embd_sdclkin", "gout_mif_cmu_fsys_mmc0", CLK_ENABLE_CLK_FSYS_MMC_EMBD, 0, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_FSYS_MMC_CARD_SDCLKIN, "gout_fsys_mmc_card_sdclkin", "gout_mif_cmu_fsys_mmc2", CLK_ENABLE_CLK_FSYS_MMC_CARD, 0, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_FSYS_USB20DRD_HCLK_CTRL, "gout_fsys_usb20drd_hclk_ctrl", "gout_mif_cmu_fsys_bus", CLK_ENABLE_CLK_FSYS_BUS, 21, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_FSYS_USB20DRD_ACLK_HSDRD, "gout_fsys_usb20drd_aclk_hsdrd", "gout_mif_cmu_fsys_bus", CLK_ENABLE_CLK_FSYS_BUS, 20, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_FSYS_USB20DRD_REF_CLK, "gout_fsys_usb20drd_ref_clk", "gout_mif_cmu_fsys_usb20drd_refclk", CLK_ENABLE_CLK_FSYS_USB20DRD_REFCLK, 0, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_FSYS_USB20DRD_PHYCLOCK, "gout_fsys_usb20drd_phyclock", "mout_fsys_usb20drd_phyclock_user", CLK_ENABLE_CLKPHY_FSYS_USB20DRD_PHYCLOCK, 0, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS_USB20DRD_CLKCORE, "gout_fsys_usb20drd_clkcore", "fout_fsys_wpll_usb", CLK_ENABLE_CLK_FSYS_USB20PHY_CLKCORE, 0, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_FSYS_GPIO_PCLK, "gout_fsys_gpio_pclk", "gout_mif_cmu_fsys_bus", CLK_ENABLE_CLK_FSYS_BUS, 13, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS_SYSREG_PCLK, "gout_fsys_sysreg_pclk", "gout_mif_cmu_fsys_bus", CLK_ENABLE_CLK_FSYS_BUS, 19, CLK_IS_CRITICAL, 0),
};

static const struct samsung_cmu_info fsys_cmu_info __initconst = {
	.fixed_clks	= fsys_fixed_clks,
	.nr_fixed_clks	= ARRAY_SIZE(fsys_fixed_clks),
	.mux_clks	= fsys_mux_clks,
	.nr_mux_clks	= ARRAY_SIZE(fsys_mux_clks),
	.gate_clks	= fsys_gate_clks,
	.nr_gate_clks	= ARRAY_SIZE(fsys_gate_clks),
	.clk_regs		= fsys_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(fsys_clk_regs),
	.nr_clk_ids		= FSYS_NR_CLK,
};

/*
 * Register offsets for CMU_PERI (0x101f0000)
 */
#define CLK_CON_DIV_CLK_PERI_USI_0_SPI                   0x0400
#define CLK_CON_DIV_CLK_PERI_USI_1_SPI                   0x0404
#define CLK_ENABLE_CLK_PERI_BUS0                         0x0810
#define CLK_ENABLE_CLK_PERI_BUS1                         0x0814
#define CLK_ENABLE_CLK_PERI_BUS_SECURE_CHIPID            0x081c
#define CLK_ENABLE_CLK_PERI_BUS_SECURE_RTC_ALIVE         0x0824
#define CLK_ENABLE_CLK_PERI_BUS_SECURE_RTC_TOP           0x0828
#define CLK_ENABLE_CLK_PERI_UART_DEBUG                   0x0834
#define CLK_ENABLE_CLK_PERI_UART_SENSOR                  0x0838
#define CLK_ENABLE_CLK_PERI_SPI_REARFROM                 0x0840
#define CLK_ENABLE_CLK_PERI_SPI_ESE                      0x0844
#define CLK_ENABLE_CLK_PERI_USI_0                        0x0850
#define CLK_ENABLE_CLK_PERI_USI_1                        0x0854
#define CLK_ENABLE_CLK_PERI_USI_0_SPI                    0x0858
#define CLK_ENABLE_CLK_PERI_USI_1_SPI                    0x085c

static const unsigned long peri_clk_regs[] __initconst = {
	CLK_CON_DIV_CLK_PERI_USI_0_SPI,
	CLK_CON_DIV_CLK_PERI_USI_1_SPI,
	CLK_ENABLE_CLK_PERI_BUS0,
	CLK_ENABLE_CLK_PERI_BUS1,
	CLK_ENABLE_CLK_PERI_BUS_SECURE_CHIPID,
	CLK_ENABLE_CLK_PERI_BUS_SECURE_RTC_ALIVE,
	CLK_ENABLE_CLK_PERI_BUS_SECURE_RTC_TOP,
	CLK_ENABLE_CLK_PERI_UART_DEBUG,
	CLK_ENABLE_CLK_PERI_UART_SENSOR,
	CLK_ENABLE_CLK_PERI_SPI_REARFROM,
	CLK_ENABLE_CLK_PERI_SPI_ESE,
	CLK_ENABLE_CLK_PERI_USI_0,
	CLK_ENABLE_CLK_PERI_USI_1,
	CLK_ENABLE_CLK_PERI_USI_0_SPI,
	CLK_ENABLE_CLK_PERI_USI_1_SPI,
};

static const struct samsung_div_clock peri_div_clks[] __initconst = {
	DIV(CLK_DOUT_PERI_USI0_SPI, "dout_peri_usi0_spi", "gout_mif_cmu_peri_usi_0", CLK_CON_DIV_CLK_PERI_USI_0_SPI, 0, 2),
	DIV(CLK_DOUT_PERI_USI1_SPI, "dout_peri_usi1_spi", "gout_mif_cmu_peri_usi_1", CLK_CON_DIV_CLK_PERI_USI_1_SPI, 0, 2),
};

static const struct samsung_gate_clock peri_gate_clks[] __initconst = {
	GATE(CLK_GOUT_PERI_UART_DEBUG_PCLK, "gout_peri_uart_debug_pclk", "gout_mif_cmu_peri_bus", CLK_ENABLE_CLK_PERI_BUS1, 12, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_UART_DEBUG_UCLK, "gout_peri_uart_debug_uclk", "gout_mif_cmu_peri_uart_debug", CLK_ENABLE_CLK_PERI_UART_DEBUG, 0, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_UART_SENSOR_PCLK, "gout_peri_uart_sensor_pclk", "gout_mif_cmu_peri_bus", CLK_ENABLE_CLK_PERI_BUS1, 10, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_UART_SENSOR_UCLK, "gout_peri_uart_sensor_uclk", "gout_mif_cmu_peri_uart_sensor", CLK_ENABLE_CLK_PERI_UART_SENSOR, 0, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_USI0_PCLK, "gout_peri_usi0_pclk", "gout_mif_cmu_peri_bus", CLK_ENABLE_CLK_PERI_BUS1, 15, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_USI0_SCLK_UART, "gout_peri_usi0_sclk_uart", "gout_mif_cmu_peri_usi_0", CLK_ENABLE_CLK_PERI_USI_0, 0, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_USI0_SCLK_SPI, "gout_peri_usi0_sclk_spi", "dout_peri_usi0_spi", CLK_ENABLE_CLK_PERI_USI_0_SPI, 0, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_USI1_PCLK, "gout_peri_usi1_pclk", "gout_mif_cmu_peri_bus", CLK_ENABLE_CLK_PERI_BUS1, 16, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_USI1_SCLK_UART, "gout_peri_usi1_sclk_uart", "gout_mif_cmu_peri_usi_1", CLK_ENABLE_CLK_PERI_USI_1, 0, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_USI1_SCLK_SPI, "gout_peri_usi1_sclk_spi", "dout_peri_usi1_spi", CLK_ENABLE_CLK_PERI_USI_1_SPI, 0, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_SPI_ESE_PCLK, "gout_peri_spi_ese_pclk", "gout_mif_cmu_peri_bus", CLK_ENABLE_CLK_PERI_BUS1, 6, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_SPI_ESE_CLK, "gout_peri_spi_ese_clk", "gout_mif_cmu_peri_spi_ese", CLK_ENABLE_CLK_PERI_SPI_ESE, 0, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_SPI_REARFROM_PCLK, "gout_peri_spi_rearfrom_pclk", "gout_mif_cmu_peri_bus", CLK_ENABLE_CLK_PERI_BUS1, 5, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_SPI_REARFROM_CLK, "gout_peri_spi_rearfrom_clk", "gout_mif_cmu_peri_spi_rearfrom", CLK_ENABLE_CLK_PERI_SPI_REARFROM, 0, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_MCT_PCLK, "gout_peri_mct_pclk", "gout_mif_cmu_peri_bus", CLK_ENABLE_CLK_PERI_BUS0, 26, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_WDT_CPUCL0_PCLK, "gout_peri_wdt_cpucl0_pclk", "gout_mif_cmu_peri_bus", CLK_ENABLE_CLK_PERI_BUS1, 13, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_I2C_SENSOR1_PCLK, "gout_peri_i2c_sensor1_pclk", "gout_mif_cmu_peri_bus", CLK_ENABLE_CLK_PERI_BUS0, 24, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_I2C_SENSOR2_PCLK, "gout_peri_i2c_sensor2_pclk", "gout_mif_cmu_peri_bus", CLK_ENABLE_CLK_PERI_BUS0, 25, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_I2C_TSP_PCLK, "gout_peri_i2c_tsp_pclk", "gout_mif_cmu_peri_bus", CLK_ENABLE_CLK_PERI_BUS0, 23, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_I2C_FUELGAUGE_PCLK, "gout_peri_i2c_fuelgauge_pclk", "gout_mif_cmu_peri_bus", CLK_ENABLE_CLK_PERI_BUS0, 21, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_I2C_NFC_PCLK, "gout_peri_i2c_nfc_pclk", "gout_mif_cmu_peri_bus", CLK_ENABLE_CLK_PERI_BUS0, 19, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_I2C_MUIC_PCLK, "gout_peri_i2c_muic_pclk", "gout_mif_cmu_peri_bus", CLK_ENABLE_CLK_PERI_BUS0, 18, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_HSI2C_FRONTCAM_PCLK, "gout_peri_hsi2c_frontcam_pclk", "gout_mif_cmu_peri_bus", CLK_ENABLE_CLK_PERI_BUS0, 16, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_HSI2C_MAINCAM_PCLK, "gout_peri_hsi2c_maincam_pclk", "gout_mif_cmu_peri_bus", CLK_ENABLE_CLK_PERI_BUS0, 15, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_HSI2C_FRONTSENSOR_PCLK, "gout_peri_hsi2c_frontsensor_pclk", "gout_mif_cmu_peri_bus", CLK_ENABLE_CLK_PERI_BUS0, 13, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_HSI2C_REARAF_PCLK, "gout_peri_hsi2c_rearaf_pclk", "gout_mif_cmu_peri_bus", CLK_ENABLE_CLK_PERI_BUS0, 12, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_HSI2C_REARSENSOR_PCLK, "gout_peri_hsi2c_rearsensor_pclk", "gout_mif_cmu_peri_bus", CLK_ENABLE_CLK_PERI_BUS0, 11, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_PWM_MOTOR_PCLK, "gout_peri_pwm_motor_pclk", "gout_mif_cmu_peri_bus", CLK_ENABLE_CLK_PERI_BUS0, 29, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_RTC_ALIVE_PCLK, "gout_peri_rtc_alive_pclk", "gout_mif_cmu_peri_bus", CLK_ENABLE_CLK_PERI_BUS_SECURE_RTC_ALIVE, 0, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_RTC_TOP_PCLK, "gout_peri_rtc_top_pclk", "gout_mif_cmu_peri_bus", CLK_ENABLE_CLK_PERI_BUS_SECURE_RTC_TOP, 0, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_CHIPID_PCLK, "gout_peri_chipid_pclk", "gout_mif_cmu_peri_bus", CLK_ENABLE_CLK_PERI_BUS_SECURE_CHIPID, 0, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_GPIO_TOUCH_PCLK, "gout_peri_gpio_touch_pclk", "gout_mif_cmu_peri_bus", CLK_ENABLE_CLK_PERI_BUS0, 10, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERI_GPIO_TOP_PCLK, "gout_peri_gpio_top_pclk", "gout_mif_cmu_peri_bus", CLK_ENABLE_CLK_PERI_BUS0, 9, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERI_GPIO_NFC_PCLK, "gout_peri_gpio_nfc_pclk", "gout_mif_cmu_peri_bus", CLK_ENABLE_CLK_PERI_BUS0, 8, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERI_GPIO_ESE_PCLK, "gout_peri_gpio_ese_pclk", "gout_mif_cmu_peri_bus", CLK_ENABLE_CLK_PERI_BUS0, 7, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERI_GPIO_ALIVE_PCLK, "gout_peri_gpio_alive_pclk", "gout_mif_cmu_peri_bus", CLK_ENABLE_CLK_PERI_BUS1, 0, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERI_SYSREG_PCLK, "gout_peri_sysreg_pclk", "gout_mif_cmu_peri_bus", CLK_ENABLE_CLK_PERI_BUS1, 9, CLK_IS_CRITICAL, 0),
};

static const struct samsung_cmu_info peri_cmu_info __initconst = {
	.div_clks	= peri_div_clks,
	.nr_div_clks	= ARRAY_SIZE(peri_div_clks),
	.gate_clks	= peri_gate_clks,
	.nr_gate_clks	= ARRAY_SIZE(peri_gate_clks),
	.clk_regs		= peri_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(peri_clk_regs),
	.nr_clk_ids		= PERI_NR_CLK,
};

/*
 * Register offsets for CMU_DISPAUD (0x148d0000)
 */
#define CLK_CON_MUX_CLKCMU_DISPAUD_BUS_USER              0x0210
#define CLK_CON_MUX_CLKCMU_DISPAUD_DECON_INT_VCLK_USER   0x0214
#define CLK_CON_MUX_CLKPHY_DISPAUD_MIPIPHY_TXBYTECLKHS_USER 0x0224
#define CLK_CON_MUX_CLKPHY_DISPAUD_MIPIPHY_RXCLKESC0_USER 0x0228
#define CLK_CON_DIV_CLK_DISPAUD_APB                      0x0400
#define CLK_CON_DIV_CLK_DISPAUD_DECON_INT_VCLK           0x0404
#define CLK_ENABLE_CLK_DISPAUD_BUS                       0x0810
#define CLK_ENABLE_CLK_DISPAUD_APB                       0x0814
#define CLK_ENABLE_CLK_DISPAUD_DECON_INT_VCLK            0x0820
#define CLK_ENABLE_CLKPHY_DISPAUD_MIPIPHY_TXBYTECLKHS    0x0824
#define CLK_ENABLE_CLKPHY_DISPAUD_MIPIPHY_RXCLKESC0      0x0828

static const unsigned long dispaud_clk_regs[] __initconst = {
	CLK_CON_MUX_CLKCMU_DISPAUD_BUS_USER,
	CLK_CON_MUX_CLKCMU_DISPAUD_DECON_INT_VCLK_USER,
	CLK_CON_MUX_CLKPHY_DISPAUD_MIPIPHY_TXBYTECLKHS_USER,
	CLK_CON_MUX_CLKPHY_DISPAUD_MIPIPHY_RXCLKESC0_USER,
	CLK_CON_DIV_CLK_DISPAUD_APB,
	CLK_CON_DIV_CLK_DISPAUD_DECON_INT_VCLK,
	CLK_ENABLE_CLK_DISPAUD_BUS,
	CLK_ENABLE_CLK_DISPAUD_APB,
	CLK_ENABLE_CLK_DISPAUD_DECON_INT_VCLK,
	CLK_ENABLE_CLKPHY_DISPAUD_MIPIPHY_TXBYTECLKHS,
	CLK_ENABLE_CLKPHY_DISPAUD_MIPIPHY_RXCLKESC0,
};

static const struct samsung_fixed_rate_clock dispaud_fixed_clks[] __initconst = {
	FRATE(CLK_FRAT_DISPAUD_MIPIPHY_TXBYTECLKHS, "frat_dispaud_mipiphy_txbyteclkhs", NULL, 0, 188000000),
	FRATE(CLK_FRAT_DISPAUD_MIPIPHY_RXCLKESC0, "frat_dispaud_mipiphy_rxclkesc0", NULL, 0, 188000000),
};

/* List of parent clocks for muxes in CMU_DISPAUD */
PNAME(mout_dispaud_bus_user_p) = { "oscclk", "gout_mif_cmu_dispaud_bus" };
PNAME(mout_dispaud_decon_vclk_user_p) = { "oscclk", "gout_mif_cmu_dispaud_decon_int_vclk" };
PNAME(mout_dispaud_mipiphy_txbyteclkhs_user_p) = { "oscclk", "frat_dispaud_mipiphy_txbyteclkhs" };
PNAME(mout_dispaud_mipiphy_rxclkesc0_user_p) = { "oscclk", "frat_dispaud_mipiphy_rxclkesc0" };

static const struct samsung_mux_clock dispaud_mux_clks[] __initconst = {
	MUX(CLK_MOUT_DISPAUD_BUS_USER, "mout_dispaud_bus_user", mout_dispaud_bus_user_p, CLK_CON_MUX_CLKCMU_DISPAUD_BUS_USER, 12, 1),
	MUX(CLK_MOUT_DISPAUD_DECON_VCLK_USER, "mout_dispaud_decon_vclk_user", mout_dispaud_decon_vclk_user_p, CLK_CON_MUX_CLKCMU_DISPAUD_DECON_INT_VCLK_USER, 12, 1),
	MUX(CLK_MOUT_DISPAUD_MIPIPHY_TXBYTECLKHS_USER, "mout_dispaud_mipiphy_txbyteclkhs_user", mout_dispaud_mipiphy_txbyteclkhs_user_p, CLK_CON_MUX_CLKPHY_DISPAUD_MIPIPHY_TXBYTECLKHS_USER, 12, 1),
	MUX(CLK_MOUT_DISPAUD_MIPIPHY_RXCLKESC0_USER, "mout_dispaud_mipiphy_rxclkesc0_user", mout_dispaud_mipiphy_rxclkesc0_user_p, CLK_CON_MUX_CLKPHY_DISPAUD_MIPIPHY_RXCLKESC0_USER, 12, 1),
};

static const struct samsung_div_clock dispaud_div_clks[] __initconst = {
	DIV(CLK_DOUT_DISPAUD_APB, "dout_dispaud_apb", "mout_dispaud_bus_user", CLK_CON_DIV_CLK_DISPAUD_APB, 0, 2),
	DIV(CLK_DOUT_DISPAUD_DECON_VCLK, "dout_dispaud_decon_vclk", "mout_dispaud_decon_vclk_user", CLK_CON_DIV_CLK_DISPAUD_DECON_INT_VCLK, 0, 3),
};

static const struct samsung_gate_clock dispaud_gate_clks[] __initconst = {
	GATE(CLK_GOUT_DISPAUD_BUS, "gout_dispaud_bus", "mout_dispaud_bus_user", CLK_ENABLE_CLK_DISPAUD_BUS, 0, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_DISPAUD_BUS_VPP, "gout_dispaud_bus_vpp", "mout_dispaud_bus_user", CLK_ENABLE_CLK_DISPAUD_BUS, 1, 0, 0),
	GATE(CLK_GOUT_DISPAUD_BUS_DISP, "gout_dispaud_bus_disp", "mout_dispaud_bus_user", CLK_ENABLE_CLK_DISPAUD_BUS, 2, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_DISPAUD_BUS_PPMU, "gout_dispaud_bus_ppmu", "mout_dispaud_bus_user", CLK_ENABLE_CLK_DISPAUD_BUS, 3, 0, 0),
	GATE(CLK_GOUT_DISPAUD_APB, "gout_dispaud_apb", "dout_dispaud_apb", CLK_ENABLE_CLK_DISPAUD_APB, 0, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_DISPAUD_APB_DISP, "gout_dispaud_apb_disp", "dout_dispaud_apb", CLK_ENABLE_CLK_DISPAUD_APB, 1, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_DISPAUD_DECON_VCLK, "gout_dispaud_decon_vclk", "dout_dispaud_decon_vclk", CLK_ENABLE_CLK_DISPAUD_DECON_INT_VCLK, 0, 0, 0),
	GATE(CLK_GOUT_DISPAUD_MIPIPHY_TXBYTECLKHS, "gout_dispaud_mipiphy_txbyteclkhs", "mout_dispaud_mipiphy_txbyteclkhs_user", CLK_ENABLE_CLKPHY_DISPAUD_MIPIPHY_TXBYTECLKHS, 0, 0, 0),
	GATE(CLK_GOUT_DISPAUD_MIPIPHY_RXCLKESC0, "gout_dispaud_mipiphy_rxclkesc0", "mout_dispaud_mipiphy_rxclkesc0_user", CLK_ENABLE_CLKPHY_DISPAUD_MIPIPHY_RXCLKESC0, 0, 0, 0),
};

static const struct samsung_cmu_info dispaud_cmu_info __initconst = {
	.fixed_clks	= dispaud_fixed_clks,
	.nr_fixed_clks	= ARRAY_SIZE(dispaud_fixed_clks),
	.mux_clks	= dispaud_mux_clks,
	.nr_mux_clks	= ARRAY_SIZE(dispaud_mux_clks),
	.div_clks	= dispaud_div_clks,
	.nr_div_clks	= ARRAY_SIZE(dispaud_div_clks),
	.gate_clks	= dispaud_gate_clks,
	.nr_gate_clks	= ARRAY_SIZE(dispaud_gate_clks),
	.clk_regs		= dispaud_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(dispaud_clk_regs),
	.nr_clk_ids		= DISPAUD_NR_CLK,
	.clk_name		= "bus",
};

/*
 * USB PLL (WPLL), the 20 MHz reference of the USB PHY. The PLL itself is in
 * the WLBT block; USBPLL_CON1 in CMU_FSYS tells who drives it. Without WLBT
 * firmware the AP owns it: select the 26 MHz reference, enable it, hand the
 * ownership to the AP and start the AFC (same steps and 20 us delay as the
 * downstream pwrcal / SCSC code, usbpll,owner = "y", usbpll,udelay = <20>).
 */
#define USBPLL_CON1_OFFSET	0x1004
#define USBPLL_WPLL_SEL		BIT(0)	/* 1: AP owns the PLL, 0: WLBT */
#define USBPLL_WPLL_EN		BIT(1)
#define USBPLL_WPLL_AFC_START	BIT(2)
#define USBPLL_WPLL_FREF_SEL	BIT(3)	/* 1: 52 MHz TCXO, 0: 26 MHz */
#define USBPLL_RATE		20000000

struct exynos7570_wpll {
	struct clk_hw hw;
	void __iomem *con1;
};

#define to_wpll(_hw) container_of(_hw, struct exynos7570_wpll, hw)

static int exynos7570_wpll_prepare(struct clk_hw *hw)
{
	struct exynos7570_wpll *wpll = to_wpll(hw);
	u32 val = readl(wpll->con1);

	if ((val & (USBPLL_WPLL_SEL | USBPLL_WPLL_EN)) ==
	    (USBPLL_WPLL_SEL | USBPLL_WPLL_EN))
		return 0;

	val &= ~USBPLL_WPLL_FREF_SEL;
	val |= USBPLL_WPLL_EN;
	writel(val, wpll->con1);
	udelay(20);

	val |= USBPLL_WPLL_SEL;
	writel(val, wpll->con1);

	val |= USBPLL_WPLL_AFC_START;
	writel(val, wpll->con1);
	usleep_range(60, 100);

	return 0;
}

static int exynos7570_wpll_is_prepared(struct clk_hw *hw)
{
	u32 val = readl(to_wpll(hw)->con1);

	return (val & (USBPLL_WPLL_SEL | USBPLL_WPLL_EN)) ==
	       (USBPLL_WPLL_SEL | USBPLL_WPLL_EN);
}

static unsigned long exynos7570_wpll_recalc_rate(struct clk_hw *hw,
						 unsigned long parent_rate)
{
	return USBPLL_RATE;
}

static const struct clk_ops exynos7570_wpll_ops = {
	.prepare	= exynos7570_wpll_prepare,
	.is_prepared	= exynos7570_wpll_is_prepared,
	.recalc_rate	= exynos7570_wpll_recalc_rate,
};

static void __init exynos7570_register_wpll(struct device *dev)
{
	static const char *const parents[] = { "oscclk" };
	struct clk_init_data init = {
		.name = "fout_fsys_wpll_usb",
		.ops = &exynos7570_wpll_ops,
		.parent_names = parents,
		.num_parents = 1,
	};
	struct exynos7570_wpll *wpll;
	void __iomem *base;

	base = of_iomap(dev->of_node, 0);
	if (!base) {
		dev_err(dev, "failed to map the CMU for the USB PLL\n");
		return;
	}

	wpll = kzalloc(sizeof(*wpll), GFP_KERNEL);
	if (!wpll) {
		iounmap(base);
		return;
	}

	wpll->con1 = base + USBPLL_CON1_OFFSET;
	wpll->hw.init = &init;

	if (clk_hw_register(dev, &wpll->hw)) {
		dev_err(dev, "failed to register the USB PLL\n");
		kfree(wpll);
		iounmap(base);
	}
}

static int __init exynos7570_cmu_probe(struct platform_device *pdev)
{
	const struct samsung_cmu_info *info;
	struct device *dev = &pdev->dev;

	info = of_device_get_match_data(dev);
	exynos_arm64_register_cmu(dev, dev->of_node, info);

	if (info == &fsys_cmu_info)
		exynos7570_register_wpll(dev);

	return 0;
}

static const struct of_device_id exynos7570_cmu_of_match[] = {
	{
		.compatible = "samsung,exynos7570-cmu-mif",
		.data = &mif_cmu_info,
	},
	{
		.compatible = "samsung,exynos7570-cmu-fsys",
		.data = &fsys_cmu_info,
	},
	{
		.compatible = "samsung,exynos7570-cmu-peri",
		.data = &peri_cmu_info,
	},
	{
		.compatible = "samsung,exynos7570-cmu-dispaud",
		.data = &dispaud_cmu_info,
	},
	{ },
};

static struct platform_driver exynos7570_cmu_driver __refdata = {
	.driver = {
		.name = "exynos7570-cmu",
		.of_match_table = exynos7570_cmu_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = exynos7570_cmu_probe,
};

static int __init exynos7570_cmu_init(void)
{
	return platform_driver_register(&exynos7570_cmu_driver);
}
core_initcall(exynos7570_cmu_init);
