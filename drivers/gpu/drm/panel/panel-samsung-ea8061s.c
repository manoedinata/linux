// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung EA8061S (AMS549HZ37) 720x1280 AMOLED MIPI-DSI video-mode panel.
 *
 * Panel hardware interface ported from the Samsung downstream Exynos7570
 * Galaxy J4 (j4lte) panel driver:
 *   panels/ea8061s_j4lte_lcd_ctrl.c   (init / brightness / display on-off)
 *   panels/ea8061s_j4lte_param.h      (DCS sequences + tables)    [#included]
 *   panels/dynamic_aid.c              (gamma computation engine)  [#included]
 *   panels/dynamic_aid_ea8061s_j4lte.h (EA8061S AID tables)       [#included]
 * so every DDI-visible byte (sequences, tables, AID math) matches
 * downstream exactly.
 *
 * Simplifications vs. downstream:
 *  - temperature is assumed to be 25 degC (the downstream default), so the
 *    room-temperature ELVSS column is used and TSET stays 0x19.
 *  - no mdnie, no dynamic brightness smoothing, no DPUI/sysfs attributes.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#include <drm/drm_modes.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include "dynamic_aid.c"
#include "ea8061s_j4lte_param.h"

#define LEVEL_IS_HBM(brightness)	((brightness) == EXTEND_BRIGHTNESS)

#define GAMMA_CMD_CNT		((u16)ARRAY_SIZE(SEQ_GAMMA_CONDITION_SET))
#define AID_CMD_CNT		((u16)ARRAY_SIZE(SEQ_AID_SET))
#define ELVSS_CMD_CNT		((u16)ARRAY_SIZE(SEQ_ELVSS_SET))
#define ACL_CMD_CNT		((u16)ARRAY_SIZE(SEQ_ACL_OFF))
#define OPR_CMD_CNT		((u16)ARRAY_SIZE(SEQ_ACL_OPR_OFF))
#define HBM_CMD_CNT		((u16)ARRAY_SIZE(SEQ_HBM_OFF))
#define TSET_CMD_CNT		((u16)ARRAY_SIZE(SEQ_TSET))

struct ea8061s {
	struct device *dev;
	struct mipi_dsi_device *dsi;
	struct drm_panel panel;
	struct backlight_device *bd;
	struct mutex lock;

	struct regulator *vddio;	/* LCD_1P8 (LDO20) */
	struct gpio_desc *enable;	/* LCD_3P0 load switch (gpd1-0) */
	struct gpio_desc *reset;	/* panel reset (gpd2-0) */

	bool prepared;
	bool enabled;
	/* panel left running by the bootloader and not taken over yet */
	bool handover;

	int *mtp;
	struct dynamic_aid_param_t daid;
	u8 mtp_raw[LDI_LEN_MTP];

	u8 gamma_table[IBRIGHTNESS_MAX][GAMMA_CMD_CNT];
	u8 elvss_table[IBRIGHTNESS_MAX][TEMP_MAX][ELVSS_CMD_CNT];

	u8 id_info[LDI_LEN_ID];

	int temperature_index;		/* fixed TEMP_ABOVE_MINUS_00_DEGREE */
	int adaptive_control;		/* ACL_STATUS_* */
	int brightness, bl;
};

static const struct drm_display_mode ea8061s_default_mode = {
	.clock = (720 + 56 + 4 + 48) * (1280 + 9 + 2 + 5) * 60 / 1000,
	.hdisplay = 720,
	.hsync_start = 720 + 56,
	.hsync_end = 720 + 56 + 4,
	.htotal = 720 + 56 + 4 + 48,
	.vdisplay = 1280,
	.vsync_start = 1280 + 9,
	.vsync_end = 1280 + 9 + 2,
	.vtotal = 1280 + 9 + 2 + 5,
	.width_mm = 68,
	.height_mm = 121,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static inline struct ea8061s *panel_to_ea8061s(struct drm_panel *panel)
{
	return container_of(panel, struct ea8061s, panel);
}

static int ea8061s_write(struct ea8061s *ctx, const u8 *seq, size_t len)
{
	int ret;

	ret = mipi_dsi_dcs_write_buffer(ctx->dsi, seq, len);
	if (ret < 0 && ret != len)
		dev_warn(ctx->dev, "DSI write of %zu bytes failed: %d\n",
			 len, ret);

	return 0;	/* keep the init sequence going */
}

#define DSI_WRITE(s, n)		ea8061s_write(ctx, s, (n))

/* ------------------------------------------------------------------ *
 * brightness (low_level_set_brightness)
 * ------------------------------------------------------------------ */

static int set_acl(struct ea8061s *ctx)
{
	int opr_status, acl_status;

	opr_status = brightness_opr_table[!!ctx->adaptive_control][ctx->brightness];
	acl_status = !!opr_status;

	DSI_WRITE(OPR_TABLE[opr_status], OPR_CMD_CNT);
	DSI_WRITE(ACL_TABLE[acl_status], ACL_CMD_CNT);

	return 0;
}

static int low_level_set_brightness(struct ea8061s *ctx, u8 force)
{
	(void)force;

	/* Test Key Enable */
	DSI_WRITE(SEQ_TEST_KEY_ON_F0, ARRAY_SIZE(SEQ_TEST_KEY_ON_F0));
	DSI_WRITE(SEQ_TEST_KEY_ON_FC, ARRAY_SIZE(SEQ_TEST_KEY_ON_FC));

	/* gamma update */
	DSI_WRITE(ctx->gamma_table[ctx->bl], GAMMA_CMD_CNT);

	/* aor update */
	DSI_WRITE(AOR_TABLE[ctx->bl], AID_CMD_CNT);

	/* elvss update */
	DSI_WRITE(ctx->elvss_table[ctx->bl][ctx->temperature_index],
		  ELVSS_CMD_CNT);

	/* acl/opr update */
	set_acl(ctx);

	/* hbm update */
	DSI_WRITE(HBM_TABLE[LEVEL_IS_HBM(ctx->brightness)], HBM_CMD_CNT);

	DSI_WRITE(SEQ_GAMMA_UPDATE, ARRAY_SIZE(SEQ_GAMMA_UPDATE));

	/* tset: temperature is fixed to 25C (downstream default) */
	DSI_WRITE(SEQ_TSET, TSET_CMD_CNT);

	/* Test Key Disable */
	DSI_WRITE(SEQ_TEST_KEY_OFF_F0, ARRAY_SIZE(SEQ_TEST_KEY_OFF_F0));
	DSI_WRITE(SEQ_TEST_KEY_OFF_FC, ARRAY_SIZE(SEQ_TEST_KEY_OFF_FC));

	return 0;
}

static int ea8061s_set_brightness(struct ea8061s *ctx, u8 force)
{
	int ret = 0;

	mutex_lock(&ctx->lock);

	ctx->brightness = ctx->bd->props.brightness;
	ctx->bl = brightness_table[ctx->brightness];

	if (ctx->prepared)
		ret = low_level_set_brightness(ctx, force);

	mutex_unlock(&ctx->lock);

	return ret;
}

static int ea8061s_bl_update_status(struct backlight_device *bd)
{
	struct ea8061s *ctx = bl_get_data(bd);

	return ea8061s_set_brightness(ctx, 0);
}

static int ea8061s_bl_get_brightness(struct backlight_device *bd)
{
	return bd->props.brightness;
}

static const struct backlight_ops ea8061s_bl_ops = {
	.get_brightness = ea8061s_bl_get_brightness,
	.update_status = ea8061s_bl_update_status,
};

/* ------------------------------------------------------------------ *
 * panel reads + gamma table build
 * ------------------------------------------------------------------ */

static int ea8061s_read_info(struct ea8061s *ctx, u8 reg, u32 len, u8 *buf)
{
	int ret;

	/* without this the panel answers with a single byte */
	ret = mipi_dsi_set_maximum_return_packet_size(ctx->dsi, len);
	if (ret < 0)
		dev_warn(ctx->dev, "failed to set the return packet size: %d\n", ret);

	ret = mipi_dsi_dcs_read(ctx->dsi, reg, buf, len);
	if (ret < 0) {
		dev_err(ctx->dev, "failed to read DCS reg %#x (len %u): %d\n",
			reg, len, ret);
		return ret;
	}
	if ((u32)ret != len)
		dev_info(ctx->dev, "short read %d/%u of DCS reg %#x\n",
			 ret, len, reg);

	return 0;
}

/* V255(msb is separated) ~ VT -> VT ~ V255(msb is not separated) and signed bit */
static void reorder_reg2mtp(u8 *reg, int *mtp)
{
	int j, c, v;

	for (c = 0, j = 0; c < CI_MAX; c++, j++) {
		if (reg[j++] & 0x01)
			mtp[(IV_MAX-1)*CI_MAX+c] = reg[j] * (-1);
		else
			mtp[(IV_MAX-1)*CI_MAX+c] = reg[j];
	}

	for (v = IV_MAX - 2; v >= 0; v--) {
		for (c = 0; c < CI_MAX; c++, j++) {
			if (reg[j] & 0x80)
				mtp[CI_MAX*v+c] = (reg[j] & 0x7F) * (-1);
			else
				mtp[CI_MAX*v+c] = reg[j];
		}
	}
}

/* VT ~ V255 -> [V255(msb separated) ... VT]; reg[0] = gamma cmd addr (0xCA) */
static void reorder_gamma2reg(int *gamma, u8 *reg)
{
	int j, c, v;
	int *pgamma;

	v = IV_MAX - 1;
	pgamma = &gamma[v * CI_MAX];
	for (c = 0, j = 1; c < CI_MAX; c++, pgamma++) {
		if (*pgamma & 0x100)
			reg[j++] = 1;
		else
			reg[j++] = 0;

		reg[j++] = *pgamma & 0xff;
	}

	for (v = IV_MAX - 2; v > IV_VT; v--) {
		pgamma = &gamma[v * CI_MAX];
		for (c = 0; c < CI_MAX; c++, pgamma++)
			reg[j++] = *pgamma;
	}

	v = IV_VT;
	pgamma = &gamma[v * CI_MAX];
	reg[j++] = pgamma[CI_RED] << 4 | pgamma[CI_GREEN];
	reg[j++] = pgamma[CI_BLUE];
}

static void init_dynamic_aid(struct ea8061s *ctx)
{
	ctx->daid.vreg = VREG_OUT_X1000;
	ctx->daid.iv_tbl = index_voltage_table;
	ctx->daid.iv_max = IV_MAX;
	ctx->daid.mtp = ctx->mtp;
	ctx->daid.gamma_default = gamma_default;
	ctx->daid.formular = gamma_formula;
	ctx->daid.vt_voltage_value = vt_voltage_value;

	ctx->daid.ibr_tbl = index_brightness_table;
	ctx->daid.ibr_max = IBRIGHTNESS_MAX;

	ctx->daid.offset_color = (const struct rgb_t(*)[])offset_color;
	ctx->daid.iv_ref = index_voltage_reference;
	ctx->daid.m_gray = m_gray;
}

static void init_elvss_table(struct ea8061s *ctx, const u8 *data)
{
	int i, temp;

	for (i = 0; i < IBRIGHTNESS_MAX; i++) {
		for (temp = 0; temp < TEMP_MAX; temp++) {
			/* Duplicate with reading value from DDI */
			memcpy(&ctx->elvss_table[i][temp][1], data,
			       LDI_LEN_ELVSS);

			ctx->elvss_table[i][temp][0] = LDI_REG_ELVSS;
			ctx->elvss_table[i][temp][LDI_OFFSET_ELVSS_1] =
				elvss_mpscon_offset_data[i][LDI_OFFSET_ELVSS_1];
			ctx->elvss_table[i][temp][LDI_OFFSET_ELVSS_2] =
				elvss_mpscon_offset_data[i][LDI_OFFSET_ELVSS_2];
			if (elvss_otp_data[i].nit &&
			    elvss_otp_data[i].not_otp[temp] != -1)
				ctx->elvss_table[i][temp][LDI_OFFSET_ELVSS_3] =
					elvss_otp_data[i].not_otp[temp];
		}
	}
}

/* pre-allocate + calculate the gamma command table for every level */
static int init_gamma(struct ea8061s *ctx)
{
	int i;
	int ret = 0;
	int **gamma;

	gamma = kcalloc(IBRIGHTNESS_MAX, sizeof(int *), GFP_KERNEL);
	if (!gamma) {
		dev_err(ctx->dev, "failed to allocate gamma table\n");
		return -ENOMEM;
	}

	for (i = 0; i < IBRIGHTNESS_MAX; i++) {
		gamma[i] = kcalloc(IV_MAX * CI_MAX, sizeof(int), GFP_KERNEL);
		if (!gamma[i]) {
			dev_err(ctx->dev, "failed to allocate gamma\n");
			ret = -ENOMEM;
			goto err_alloc_gamma;
		}
	}

	for (i = 0; i < IBRIGHTNESS_MAX; i++)
		memcpy(ctx->gamma_table[i], SEQ_GAMMA_CONDITION_SET,
		       GAMMA_CMD_CNT);

	reorder_reg2mtp(ctx->mtp_raw, ctx->mtp);
	dynamic_aid(ctx->daid, gamma);

	for (i = 0; i < IBRIGHTNESS_MAX; i++)
		reorder_gamma2reg(gamma[i], ctx->gamma_table[i]);

	for (i = 0; i < IBRIGHTNESS_MAX; i++)
		kfree(gamma[i]);
	kfree(gamma);

	return 0;

err_alloc_gamma:
	while (i > 0)
		kfree(gamma[--i]);
	kfree(gamma);
	return ret;
}

static int ea8061s_read_init_info(struct ea8061s *ctx)
{
	u8 data[LDI_LEN_ELVSS] = { 0, };
	int ret = 0;

	ret = ea8061s_read_info(ctx, LDI_REG_ID, LDI_LEN_ID, ctx->id_info);
	if (!ret && (ctx->id_info[0] || ctx->id_info[1] || ctx->id_info[2]))
		dev_info(ctx->dev, "panel id: %02x %02x %02x\n",
			 ctx->id_info[0], ctx->id_info[1], ctx->id_info[2]);

	init_dynamic_aid(ctx);

	/* Test Key Enable */
	DSI_WRITE(SEQ_TEST_KEY_ON_F0, ARRAY_SIZE(SEQ_TEST_KEY_ON_F0));

	ea8061s_read_info(ctx, LDI_REG_MTP, LDI_LEN_MTP, ctx->mtp_raw);
	ea8061s_read_info(ctx, LDI_REG_ELVSS, LDI_LEN_ELVSS, data);
	init_elvss_table(ctx, data);

	/* Test Key Disable */
	DSI_WRITE(SEQ_TEST_KEY_OFF_F0, ARRAY_SIZE(SEQ_TEST_KEY_OFF_F0));

	return init_gamma(ctx);
}

/* ------------------------------------------------------------------ *
 * panel init / display on / exit (ea8061s_j4lte_lcd_ctrl.c)
 * ------------------------------------------------------------------ */

static int ea8061s_init(struct ea8061s *ctx)
{
	dev_info(ctx->dev, "%s\n", __func__);
	usleep_range(5000, 6000);

	/* Test Key Enable */
	DSI_WRITE(SEQ_TEST_KEY_ON_F0, ARRAY_SIZE(SEQ_TEST_KEY_ON_F0));
	DSI_WRITE(SEQ_TEST_KEY_ON_FC, ARRAY_SIZE(SEQ_TEST_KEY_ON_FC));

	/* common setting */
	DSI_WRITE(SEQ_HSYNC_GEN_ON, ARRAY_SIZE(SEQ_HSYNC_GEN_ON));
	DSI_WRITE(SEQ_SOURCE_SLEW, ARRAY_SIZE(SEQ_SOURCE_SLEW));
	DSI_WRITE(SEQ_AID_SET, ARRAY_SIZE(SEQ_AID_SET));
	DSI_WRITE(SEQ_GAMMA_UPDATE, ARRAY_SIZE(SEQ_GAMMA_UPDATE));
	DSI_WRITE(SEQ_S_WIRE, ARRAY_SIZE(SEQ_S_WIRE));

	DSI_WRITE(SEQ_SLEEP_OUT, ARRAY_SIZE(SEQ_SLEEP_OUT));
	usleep_range(10000, 11000);

	DSI_WRITE(SEQ_POWER, ARRAY_SIZE(SEQ_POWER));
	DSI_WRITE(SEQ_AID_SETTING_MAX, ARRAY_SIZE(SEQ_AID_SETTING_MAX));
	DSI_WRITE(SEQ_GAMMA_UPDATE, ARRAY_SIZE(SEQ_GAMMA_UPDATE));

	msleep(165); /* Wait 10ms + 150 ms */

	DSI_WRITE(SEQ_GAMMA_CONDITION_SET, GAMMA_CMD_CNT);
	DSI_WRITE(SEQ_AID_SET, AID_CMD_CNT);
	DSI_WRITE(SEQ_ELVSS_SET, ELVSS_CMD_CNT);
	DSI_WRITE(SEQ_ACL_OFF, ACL_CMD_CNT);
	DSI_WRITE(SEQ_GAMMA_UPDATE, ARRAY_SIZE(SEQ_GAMMA_UPDATE));
	DSI_WRITE(SEQ_TSET, TSET_CMD_CNT);

	DSI_WRITE(SEQ_TEST_KEY_OFF_F0, ARRAY_SIZE(SEQ_TEST_KEY_OFF_F0));
	DSI_WRITE(SEQ_TEST_KEY_OFF_FC, ARRAY_SIZE(SEQ_TEST_KEY_OFF_FC));

	/*
	 * Called from prepare() with ctx->lock held and ctx->prepared not set yet,
	 * so ea8061s_set_brightness() can neither be used (it takes the lock) nor
	 * would it send anything.
	 */
	ctx->brightness = ctx->bd->props.brightness;
	ctx->bl = brightness_table[ctx->brightness];

	return low_level_set_brightness(ctx, 1);
}

static int ea8061s_displayon(struct ea8061s *ctx)
{
	dev_info(ctx->dev, "%s\n", __func__);

	/* Display On (29h) */
	DSI_WRITE(SEQ_DISPLAY_ON, ARRAY_SIZE(SEQ_DISPLAY_ON));

	usleep_range(10000, 11000);

	DSI_WRITE(SEQ_TEST_KEY_ON_F0, ARRAY_SIZE(SEQ_TEST_KEY_ON_F0));
	DSI_WRITE(SEQ_GAMMA_UPDATE, ARRAY_SIZE(SEQ_GAMMA_UPDATE));
	DSI_WRITE(SEQ_TEST_KEY_OFF_F0, ARRAY_SIZE(SEQ_TEST_KEY_OFF_F0));

	return 0;
}

static int ea8061s_exit(struct ea8061s *ctx)
{
	dev_info(ctx->dev, "%s\n", __func__);

	/* Display Off (28h) */
	DSI_WRITE(SEQ_DISPLAY_OFF, ARRAY_SIZE(SEQ_DISPLAY_OFF));

	/* Wait 20ms */
	usleep_range(20000, 21000);

	/* Sleep In (10h) */
	DSI_WRITE(SEQ_SLEEP_IN, ARRAY_SIZE(SEQ_SLEEP_IN));

	/* Wait 100ms */
	msleep(100);

	return 0;
}

/* ------------------------------------------------------------------ *
 * power (decon_board "dsim_set_panel_power_enable/disable" +
 * "dsim_reset_panel", verbatim delays)
 * ------------------------------------------------------------------ */

static int ea8061s_power_on(struct ea8061s *ctx)
{
	int ret;

	ret = regulator_enable(ctx->vddio);
	if (ret < 0) {
		dev_err(ctx->dev, "failed to enable vddio: %d\n", ret);
		return ret;
	}

	/* LCD_3P0 load switch */
	gpiod_set_value_cansleep(ctx->enable, 1);
	usleep_range(5000, 5100);

	/* reset: start low, then hi 10 ms - lo 10 ms - hi 10 ms */
	gpiod_set_value_cansleep(ctx->reset, 0);
	usleep_range(1000, 1100);
	gpiod_set_value_cansleep(ctx->reset, 1);
	usleep_range(10000, 10100);
	gpiod_set_value_cansleep(ctx->reset, 0);
	usleep_range(10000, 10100);
	gpiod_set_value_cansleep(ctx->reset, 1);
	usleep_range(10000, 10100);

	return 0;
}

static void ea8061s_power_off(struct ea8061s *ctx)
{
	gpiod_set_value_cansleep(ctx->reset, 0);
	usleep_range(10000, 10100);
	gpiod_set_value_cansleep(ctx->enable, 0);
	regulator_disable(ctx->vddio);
	msleep(30);
}

/* ------------------------------------------------------------------ *
 * DRM panel ops
 * ------------------------------------------------------------------ */

static int ea8061s_prepare(struct drm_panel *panel)
{
	struct ea8061s *ctx = panel_to_ea8061s(panel);
	int ret;

	mutex_lock(&ctx->lock);

	if (ctx->prepared)
		goto done;

	if (ctx->handover) {
		/*
		 * The bootloader powered, reset and initialised the panel and
		 * its video stream is running. Redoing that now would let the
		 * panel lock its line phase to a stream already in flight, so
		 * only take the supply reference and read the calibration data.
		 */
		ret = regulator_enable(ctx->vddio);
		if (ret < 0)
			goto err_power;

		ret = ea8061s_read_init_info(ctx);
		if (ret < 0)
			dev_warn(ctx->dev, "panel info read failed (%d)\n", ret);

		ctx->brightness = ctx->bd->props.brightness;
		ctx->bl = brightness_table[ctx->brightness];
		low_level_set_brightness(ctx, 1);

		ctx->prepared = true;
		goto done;
	}

	ret = ea8061s_power_on(ctx);
	if (ret < 0)
		goto err_power;

	ret = ea8061s_read_init_info(ctx);
	if (ret < 0)
		dev_warn(ctx->dev,
			 "panel info read failed (%d); leaving defaults\n",
			 ret);

	ret = ea8061s_init(ctx);
	if (ret < 0)
		goto err_init;

	ctx->prepared = true;

done:
	mutex_unlock(&ctx->lock);
	return 0;

err_init:
err_power:
	ea8061s_power_off(ctx);
	mutex_unlock(&ctx->lock);
	return ret;
}

static int ea8061s_enable(struct drm_panel *panel)
{
	struct ea8061s *ctx = panel_to_ea8061s(panel);

	if (!ctx->prepared)
		return -EINVAL;

	if (!ctx->enabled && !ctx->handover)
		ea8061s_displayon(ctx);

	/* from now on the panel is ours, a later prepare() does a full init */
	ctx->handover = false;
	ctx->enabled = true;

	return 0;
}

static int ea8061s_disable(struct drm_panel *panel)
{
	struct ea8061s *ctx = panel_to_ea8061s(panel);

	if (ctx->enabled)
		DSI_WRITE(SEQ_DISPLAY_OFF, ARRAY_SIZE(SEQ_DISPLAY_OFF));

	ctx->enabled = false;

	return 0;
}

static int ea8061s_unprepare(struct drm_panel *panel)
{
	struct ea8061s *ctx = panel_to_ea8061s(panel);

	mutex_lock(&ctx->lock);

	if (!ctx->prepared)
		goto done;

	ea8061s_exit(ctx);
	ea8061s_power_off(ctx);

	ctx->prepared = false;

done:
	mutex_unlock(&ctx->lock);
	return 0;
}

static int ea8061s_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &ea8061s_default_mode);
	if (!mode) {
		dev_err(panel->dev, "failed to add mode\n");
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs ea8061s_panel_funcs = {
	.prepare = ea8061s_prepare,
	.enable = ea8061s_enable,
	.disable = ea8061s_disable,
	.unprepare = ea8061s_unprepare,
	.get_modes = ea8061s_get_modes,
};

static const struct of_device_id ea8061s_of_match[] = {
	{ .compatible = "samsung,ea8061s" },
	{ }
};
MODULE_DEVICE_TABLE(of, ea8061s_of_match);

static int ea8061s_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct ea8061s *ctx;
	struct backlight_properties props = { };
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	drm_panel_init(&ctx->panel, dev, &ea8061s_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	ctx->dev = dev;
	dev_set_drvdata(dev, ctx);
	ctx->dsi = dsi;

	mutex_init(&ctx->lock);

	ctx->temperature_index = TEMP_ABOVE_MINUS_00_DEGREE;
	ctx->adaptive_control = ACL_STATUS_ON;

	ctx->mtp = devm_kcalloc(dev, IV_MAX * CI_MAX, sizeof(int), GFP_KERNEL);
	if (!ctx->mtp)
		return -ENOMEM;

	ctx->vddio = devm_regulator_get(dev, "vddio");
	if (IS_ERR(ctx->vddio)) {
		ret = PTR_ERR(ctx->vddio);
		dev_err_probe(dev, ret, "failed to get vddio regulator\n");
		return ret;
	}

	ctx->handover = of_property_read_bool(dev->of_node, "samsung,keep-bootloader-state");

	ctx->enable = devm_gpiod_get(dev, "enable", ctx->handover ? GPIOD_ASIS : GPIOD_OUT_LOW);
	if (IS_ERR(ctx->enable)) {
		ret = PTR_ERR(ctx->enable);
		return dev_err_probe(dev, ret, "failed to get enable gpio\n");
	}

	ctx->reset = devm_gpiod_get(dev, "reset", ctx->handover ? GPIOD_ASIS : GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset)) {
		ret = PTR_ERR(ctx->reset);
		return dev_err_probe(dev, ret, "failed to get reset gpio\n");
	}

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_MODE_VIDEO_NO_HFP | MIPI_DSI_MODE_LPM;

	props.type = BACKLIGHT_RAW;
	props.max_brightness = EXTEND_BRIGHTNESS;
	props.brightness = UI_DEFAULT_BRIGHTNESS;

	ctx->bd = devm_backlight_device_register(dev, "ea8061s", dev, ctx,
						 &ea8061s_bl_ops, &props);
	if (IS_ERR(ctx->bd)) {
		ret = PTR_ERR(ctx->bd);
		return dev_err_probe(dev, ret,
				     "failed to register backlight\n");
	}

	ctx->brightness = UI_DEFAULT_BRIGHTNESS;
	ctx->bl = brightness_table[ctx->brightness];

	ctx->panel.prepare_prev_first = true;
	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "failed to attach to the DSI host\n");
	}

	return 0;
}

static void ea8061s_remove(struct mipi_dsi_device *dsi)
{
	struct ea8061s *ctx = dev_get_drvdata(&dsi->dev);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
}

static struct mipi_dsi_driver ea8061s_driver = {
	.driver = {
		.name = "panel-samsung-ea8061s",
		.of_match_table = ea8061s_of_match,
	},
	.probe = ea8061s_probe,
	.remove = ea8061s_remove,
};
module_mipi_dsi_driver(ea8061s_driver);

MODULE_AUTHOR("Ported from the Samsung Exynos7570 j4lte panel driver");
MODULE_DESCRIPTION("Samsung EA8061S AMOLED panel driver");
MODULE_LICENSE("GPL");
