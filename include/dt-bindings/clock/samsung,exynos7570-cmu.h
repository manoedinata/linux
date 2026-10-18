/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Device Tree binding constants for Exynos7570 clock controller.
 */

#ifndef _DT_BINDINGS_CLOCK_EXYNOS7570_H
#define _DT_BINDINGS_CLOCK_EXYNOS7570_H

/* CMU_MIF */
#define CLK_FOUT_MIF_SHARED0_PLL                         1
#define CLK_FOUT_MIF_SHARED1_PLL                         2
#define CLK_FOUT_MIF_SHARED2_PLL                         3
#define CLK_MOUT_MIF_SHARED0_PLL                         4
#define CLK_MOUT_MIF_SHARED1_PLL                         5
#define CLK_MOUT_MIF_SHARED2_PLL                         6
#define CLK_FRAT_MIF_TCXO                                7
#define CLK_MOUT_MIF_BUSD                                8
#define CLK_DOUT_MIF_BUSD                                9
#define CLK_DOUT_MIF_APB                                 10
#define CLK_MOUT_MIF_CMU_FSYS_BUS                        11
#define CLK_DOUT_MIF_CMU_FSYS_BUS                        12
#define CLK_GOUT_MIF_CMU_FSYS_BUS                        13
#define CLK_MOUT_MIF_CMU_FSYS_MMC0                       14
#define CLK_DOUT_MIF_CMU_FSYS_MMC0                       15
#define CLK_GOUT_MIF_CMU_FSYS_MMC0                       16
#define CLK_MOUT_MIF_CMU_FSYS_MMC2                       17
#define CLK_DOUT_MIF_CMU_FSYS_MMC2                       18
#define CLK_GOUT_MIF_CMU_FSYS_MMC2                       19
#define CLK_MOUT_MIF_CMU_FSYS_USB20DRD_REFCLK            20
#define CLK_DOUT_MIF_CMU_FSYS_USB20DRD_REFCLK            21
#define CLK_GOUT_MIF_CMU_FSYS_USB20DRD_REFCLK            22
#define CLK_MOUT_MIF_CMU_PERI_BUS                        23
#define CLK_DOUT_MIF_CMU_PERI_BUS                        24
#define CLK_GOUT_MIF_CMU_PERI_BUS                        25
#define CLK_MOUT_MIF_CMU_PERI_UART_DEBUG                 26
#define CLK_DOUT_MIF_CMU_PERI_UART_DEBUG                 27
#define CLK_GOUT_MIF_CMU_PERI_UART_DEBUG                 28
#define CLK_MOUT_MIF_CMU_PERI_UART_SENSOR                29
#define CLK_DOUT_MIF_CMU_PERI_UART_SENSOR                30
#define CLK_GOUT_MIF_CMU_PERI_UART_SENSOR                31
#define CLK_MOUT_MIF_CMU_PERI_SPI_REARFROM               32
#define CLK_DOUT_MIF_CMU_PERI_SPI_REARFROM               33
#define CLK_GOUT_MIF_CMU_PERI_SPI_REARFROM               34
#define CLK_MOUT_MIF_CMU_PERI_SPI_ESE                    35
#define CLK_DOUT_MIF_CMU_PERI_SPI_ESE                    36
#define CLK_GOUT_MIF_CMU_PERI_SPI_ESE                    37
#define CLK_MOUT_MIF_CMU_PERI_USI_0                      38
#define CLK_DOUT_MIF_CMU_PERI_USI_0                      39
#define CLK_GOUT_MIF_CMU_PERI_USI_0                      40
#define CLK_MOUT_MIF_CMU_PERI_USI_1                      41
#define CLK_DOUT_MIF_CMU_PERI_USI_1                      42
#define CLK_GOUT_MIF_CMU_PERI_USI_1                      43
#define CLK_GOUT_MIF_PDMA0_ACLK                          44
#define CLK_GOUT_MIF_SPEEDY_PCLK                         45
#define CLK_GOUT_MIF_SPEEDY_CLK                          46
#define CLK_GOUT_MIF_ADC_PCLK_S0                         47
#define CLK_GOUT_MIF_ADC_PCLK_S1                         48
#define CLK_GOUT_MIF_ADC_OSC_SYS                         49
#define CLK_GOUT_MIF_GPIO_PCLK                           50
#define CLK_GOUT_MIF_SYSREG_PCLK                         51
#define MIF_NR_CLK                                       52

/* CMU_FSYS */
#define CLK_FRAT_FSYS_USB20DRD_PHYCLOCK                  1
#define CLK_MOUT_FSYS_USB20DRD_PHYCLOCK_USER             2
#define CLK_GOUT_FSYS_MMC_EMBD_ACLK                      3
#define CLK_GOUT_FSYS_MMC_CARD_ACLK                      4
#define CLK_GOUT_FSYS_MMC_EMBD_SDCLKIN                   5
#define CLK_GOUT_FSYS_MMC_CARD_SDCLKIN                   6
#define CLK_GOUT_FSYS_USB20DRD_HCLK_CTRL                 7
#define CLK_GOUT_FSYS_USB20DRD_ACLK_HSDRD                8
#define CLK_GOUT_FSYS_USB20DRD_REF_CLK                   9
#define CLK_GOUT_FSYS_USB20DRD_PHYCLOCK                  10
#define CLK_GOUT_FSYS_USB20DRD_CLKCORE                   11
#define CLK_GOUT_FSYS_GPIO_PCLK                          12
#define CLK_GOUT_FSYS_SYSREG_PCLK                        13
#define FSYS_NR_CLK                                      14

/* CMU_PERI */
#define CLK_DOUT_PERI_USI0_SPI                           1
#define CLK_DOUT_PERI_USI1_SPI                           2
#define CLK_GOUT_PERI_UART_DEBUG_PCLK                    3
#define CLK_GOUT_PERI_UART_DEBUG_UCLK                    4
#define CLK_GOUT_PERI_UART_SENSOR_PCLK                   5
#define CLK_GOUT_PERI_UART_SENSOR_UCLK                   6
#define CLK_GOUT_PERI_USI0_PCLK                          7
#define CLK_GOUT_PERI_USI0_SCLK_UART                     8
#define CLK_GOUT_PERI_USI0_SCLK_SPI                      9
#define CLK_GOUT_PERI_USI1_PCLK                          10
#define CLK_GOUT_PERI_USI1_SCLK_UART                     11
#define CLK_GOUT_PERI_USI1_SCLK_SPI                      12
#define CLK_GOUT_PERI_SPI_ESE_PCLK                       13
#define CLK_GOUT_PERI_SPI_ESE_CLK                        14
#define CLK_GOUT_PERI_SPI_REARFROM_PCLK                  15
#define CLK_GOUT_PERI_SPI_REARFROM_CLK                   16
#define CLK_GOUT_PERI_MCT_PCLK                           17
#define CLK_GOUT_PERI_WDT_CPUCL0_PCLK                    18
#define CLK_GOUT_PERI_I2C_SENSOR1_PCLK                   19
#define CLK_GOUT_PERI_I2C_SENSOR2_PCLK                   20
#define CLK_GOUT_PERI_I2C_TSP_PCLK                       21
#define CLK_GOUT_PERI_I2C_FUELGAUGE_PCLK                 22
#define CLK_GOUT_PERI_I2C_NFC_PCLK                       23
#define CLK_GOUT_PERI_I2C_MUIC_PCLK                      24
#define CLK_GOUT_PERI_HSI2C_FRONTCAM_PCLK                25
#define CLK_GOUT_PERI_HSI2C_MAINCAM_PCLK                 26
#define CLK_GOUT_PERI_HSI2C_FRONTSENSOR_PCLK             27
#define CLK_GOUT_PERI_HSI2C_REARAF_PCLK                  28
#define CLK_GOUT_PERI_HSI2C_REARSENSOR_PCLK              29
#define CLK_GOUT_PERI_PWM_MOTOR_PCLK                     30
#define CLK_GOUT_PERI_RTC_ALIVE_PCLK                     31
#define CLK_GOUT_PERI_RTC_TOP_PCLK                       32
#define CLK_GOUT_PERI_CHIPID_PCLK                        33
#define CLK_GOUT_PERI_GPIO_TOUCH_PCLK                    34
#define CLK_GOUT_PERI_GPIO_TOP_PCLK                      35
#define CLK_GOUT_PERI_GPIO_NFC_PCLK                      36
#define CLK_GOUT_PERI_GPIO_ESE_PCLK                      37
#define CLK_GOUT_PERI_GPIO_ALIVE_PCLK                    38
#define CLK_GOUT_PERI_SYSREG_PCLK                        39
#define PERI_NR_CLK                                      40

#endif /* _DT_BINDINGS_CLOCK_EXYNOS7570_H */
