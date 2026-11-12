// SPDX-License-Identifier: GPL-2.0-only
/*
 * ChipWealth CH13726A MIPI-DSI panel driver
 * Copyright (c) 2024, Teguh Sobirin <teguh@sobir.in>.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>

#include <video/mipi_display.h>

static const struct regulator_bulk_data ch13726a_supplies[] = {
	{ .supply = "vdd1v2", },
	{ .supply = "vddio", },
	{ .supply = "vdd", },
	{ .supply = "avdd", },
};

/* Boards that route VCI and AVDD to one switched rail have no separate avdd. */
static const struct regulator_bulk_data ch13726a_supplies_no_avdd[] = {
	{ .supply = "vdd1v2", },
	{ .supply = "vddio", },
	{ .supply = "vdd", },
};

struct ch13726a_panel {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct regulator_bulk_data *supplies;
	unsigned int num_supplies;
	struct gpio_desc *reset_gpio;
	struct ch13726a_desc *desc;
	enum drm_panel_orientation orientation;
	struct drm_connector *connector;
};

struct ch13726a_desc {
	unsigned int width_mm;
	unsigned int height_mm;
	unsigned int bpc;

	const struct drm_display_mode *modes;
	unsigned int num_modes;

	/* Defaults to all four supplies */
	const struct regulator_bulk_data *supplies;
	unsigned int num_supplies;

	/*
	 * Per-board init/teardown, taken from each board's vendor device tree.
	 * The command set is the same everywhere - vendor page F0/B9, then
	 * sleep-out and display-on - only the encoding and the delays differ.
	 * A zero delay is skipped, except where noted.
	 */
	bool vendor_page_dcs;		/* F0/B9 as DCS, not generic, writes */
	bool init_max_brightness;	/* vendor leads with 51 FF */
	bool b9_high_refresh;		/* B9 11 for modes of 110 Hz and up */
	unsigned int reset_pulse_ms;	/* 0 means 10 ms */
	unsigned int power_on_delay_ms;
	unsigned int exit_sleep_delay_ms;
	unsigned int display_on_delay_ms;
	unsigned int display_off_delay_ms;	/* 0 means 50 ms */
	unsigned int sleep_in_delay_ms;
	unsigned long extra_mode_flags;
};

static inline struct ch13726a_panel *to_ch13726a_panel(struct drm_panel *panel)
{
	return container_of(panel, struct ch13726a_panel, panel);
}

static void ch13726a_reset(struct ch13726a_panel *ctx)
{
	unsigned int ms = ctx->desc->reset_pulse_ms ?: 10;

	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	fsleep(ms * 1000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	fsleep(ms * 1000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	fsleep(ms * 1000);
}

static const struct drm_display_mode *ch13726a_current_mode(struct ch13726a_panel *ctx)
{
	struct drm_connector *connector = ctx->connector;
	const struct drm_crtc_state *crtc_state;
	unsigned int i;

	if (!connector || !connector->state || !connector->state->crtc)
		return &ctx->desc->modes[0];

	crtc_state = connector->state->crtc->state;
	for (i = 0; i < ctx->desc->num_modes; i++)
		if (drm_mode_match(&crtc_state->mode, &ctx->desc->modes[i],
				   DRM_MODE_MATCH_TIMINGS | DRM_MODE_MATCH_CLOCK))
			return &ctx->desc->modes[i];

	return &ctx->desc->modes[0];
}

static int ch13726a_on(struct ch13726a_panel *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	const struct ch13726a_desc *desc = ctx->desc;
	u8 b9 = 0x00;

	if (desc->b9_high_refresh &&
	    drm_mode_vrefresh(ch13726a_current_mode(ctx)) >= 110)
		b9 = 0x11;

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	if (desc->init_max_brightness)
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x51, 0xff);

	if (desc->vendor_page_dcs) {
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x50);
		mipi_dsi_dcs_write_var_seq_multi(&dsi_ctx, 0xb9, b9);
	} else {
		mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0, 0x50);
		mipi_dsi_generic_write_var_seq_multi(&dsi_ctx, 0xb9, b9);
	}

	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	if (desc->exit_sleep_delay_ms)
		mipi_dsi_msleep(&dsi_ctx, desc->exit_sleep_delay_ms);

	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	if (desc->display_on_delay_ms)
		mipi_dsi_msleep(&dsi_ctx, desc->display_on_delay_ms);

	return dsi_ctx.accum_err;
}

static int ch13726a_disable(struct drm_panel *panel)
{
	struct ch13726a_panel *ctx = to_ch13726a_panel(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, ctx->desc->display_off_delay_ms ?: 50);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	if (ctx->desc->sleep_in_delay_ms)
		mipi_dsi_msleep(&dsi_ctx, ctx->desc->sleep_in_delay_ms);

	return dsi_ctx.accum_err;
}

static int ch13726a_prepare(struct drm_panel *panel)
{
	struct ch13726a_panel *ctx = to_ch13726a_panel(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = regulator_bulk_enable(ctx->num_supplies, ctx->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators: %d\n", ret);
		return ret;
	}

	if (ctx->desc->power_on_delay_ms)
		msleep(ctx->desc->power_on_delay_ms);
	ch13726a_reset(ctx);

	ret = ch13726a_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		regulator_bulk_disable(ctx->num_supplies, ctx->supplies);
		return ret;
	}

	msleep(28);

	return 0;
}

static int ch13726a_unprepare(struct drm_panel *panel)
{
	struct ch13726a_panel *ctx = to_ch13726a_panel(panel);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_bulk_disable(ctx->num_supplies, ctx->supplies);

	return 0;
}

static const struct drm_display_mode thor_bottom_modes[] = {
	{
		/* 120Hz */
		.clock = (1080 + 28 + 4 + 36) * (1240 + 16 + 4 + 8) * 120 / 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + 28,
		.hsync_end = 1080 + 28 + 4,
		.htotal = 1080 + 28 + 4 + 36,
		.vdisplay = 1240,
		.vsync_start = 1240 + 16,
		.vsync_end = 1240 + 16 + 4,
		.vtotal = 1240 + 16 + 4 + 8,
	},
	{
		/* 60Hz */
		.clock = (1080 + 28 + 4 + 36) * (1240 + 16 + 4 + 8) * 60 / 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + 28,
		.hsync_end = 1080 + 28 + 4,
		.htotal = 1080 + 28 + 4 + 36,
		.vdisplay = 1240,
		.vsync_start = 1240 + 16,
		.vsync_end = 1240 + 16 + 4,
		.vtotal = 1240 + 16 + 4 + 8,
	}
};

static struct ch13726a_desc thor_bottom_desc = {
	.modes = thor_bottom_modes,
	.num_modes = ARRAY_SIZE(thor_bottom_modes),
	.width_mm = 65,
	.height_mm = 75,
	.bpc = 8,
};

static const struct drm_display_mode retroidpocket_rp5_modes[] = {
	{
		/* 60Hz */
		.clock = (1080 + 12 + 4 + 12) * (1920 + 12 + 4 + 12) * 60 / 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + 12,
		.hsync_end = 1080 + 12 + 4,
		.htotal = 1080 + 12 + 4 + 12,
		.vdisplay = 1920,
		.vsync_start = 1920 + 12,
		.vsync_end = 1920 + 12 + 4,
		.vtotal = 1920 + 12 + 4 + 12,
	},
};

static struct ch13726a_desc retroidpocket_rp5_desc = {
	.modes = retroidpocket_rp5_modes,
	.num_modes = ARRAY_SIZE(retroidpocket_rp5_modes),
	.width_mm = 68,
	.height_mm = 121,
	.bpc = 8,
	.exit_sleep_delay_ms = 120,
};

static const struct drm_display_mode retroidpocket_rpmini_modes[] = {
	{
		/* 60Hz */
		.clock = (960 + 28 + 4 + 36) * (1280 + 16 + 4 + 8) * 60 / 1000,
		.hdisplay = 960,
		.hsync_start = 960 + 28,
		.hsync_end = 960 + 28 + 4,
		.htotal = 960 + 28 + 4 + 36,
		.vdisplay = 1280,
		.vsync_start = 1280 + 16,
		.vsync_end = 1280 + 16 + 4,
		.vtotal = 1280 + 16 + 4 + 8,
	},
};

static struct ch13726a_desc retroidpocket_rpmini_desc = {
	.modes = retroidpocket_rpmini_modes,
	.num_modes = ARRAY_SIZE(retroidpocket_rpmini_modes),
	.width_mm = 65,
	.height_mm = 75,
	.bpc = 8,
	.exit_sleep_delay_ms = 120,
};

/* RP Mini V2 and Thor Lite: the 1080x1240 Thor panel, 60Hz only on SM8250 */
static struct ch13726a_desc retroidpocket_rpminiv2_desc = {
	.modes = &thor_bottom_modes[1],
	.num_modes = 1,
	.width_mm = 65,
	.height_mm = 75,
	.bpc = 8,
	.exit_sleep_delay_ms = 120,
};

/*
 * Retroid Pocket Classic, both boards' vendor timings and sequences.
 * The Rev Q board carries the Thor bottom module; the vendor runs it at
 * 60 Hz by default and switches B9 to 0x11 for 120 Hz.
 */
static const struct drm_display_mode rp_classic_modes[] = {
	{
		/* 60Hz */
		.clock = (1080 + 28 + 4 + 36) * (1240 + 16 + 4 + 8) * 60 / 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + 28,
		.hsync_end = 1080 + 28 + 4,
		.htotal = 1080 + 28 + 4 + 36,
		.vdisplay = 1240,
		.vsync_start = 1240 + 16,
		.vsync_end = 1240 + 16 + 4,
		.vtotal = 1240 + 16 + 4 + 8,
	},
	{
		/* 120Hz */
		.clock = (1080 + 28 + 4 + 36) * (1240 + 16 + 4 + 8) * 120 / 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + 28,
		.hsync_end = 1080 + 28 + 4,
		.htotal = 1080 + 28 + 4 + 36,
		.vdisplay = 1240,
		.vsync_start = 1240 + 16,
		.vsync_end = 1240 + 16 + 4,
		.vtotal = 1240 + 16 + 4 + 8,
	}
};

static struct ch13726a_desc rp_classic_desc = {
	.modes = rp_classic_modes,
	.num_modes = ARRAY_SIZE(rp_classic_modes),
	.width_mm = 65,
	.height_mm = 75,
	.bpc = 8,
	.b9_high_refresh = true,
	.reset_pulse_ms = 10,
	.power_on_delay_ms = 20,
	.exit_sleep_delay_ms = 120,
	.display_on_delay_ms = 20,
	.display_off_delay_ms = 50,
	.sleep_in_delay_ms = 120,
};

/*
 * Original Retroid Pocket Classic board. The vendor calls this panel "g1392";
 * that name is stale copy-paste in its own device tree (the node claims "lcd"
 * while declaring physical-type "oled"), and the module is NOT the CH13726A
 * one the refreshed board uses - different porches, traffic mode and delays.
 * It is driven here because it speaks the same undocumented vendor register
 * page (F0 50 / B9 00), which no other panel in the tree does. That is strong
 * evidence of the same DDIC family but is NOT a confirmed part number, so do
 * not send this compatible upstream without one.
 *
 * Mode and delays are the vendor kernel node verbatim: 1080x1240,
 * H fp12/pulse4/bp12, V fp12/pulse4/bp12, burst mode, DCS-encoded vendor
 * page, 51 FF brightness prefix, sleep-out +120 ms, display-on +150 ms,
 * display-off +16 ms, sleep-in +50 ms, 20 ms reset pulses.
 *
 * VCI and AVDD are one switched 3.3 V rail on this board, so there is no
 * separate avdd supply.
 *
 * TODO verify on hardware: width_mm/height_mm are borrowed from the CH13726A
 * module, and the vendor's ESD reg_read (0x0A, expect 0x9C) is not wired up.
 */
static const struct drm_display_mode rp_classic_g1392_modes[] = {
	{
		/* 60Hz */
		.clock = (1080 + 12 + 4 + 12) * (1240 + 12 + 4 + 12) * 60 / 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + 12,
		.hsync_end = 1080 + 12 + 4,
		.htotal = 1080 + 12 + 4 + 12,
		.vdisplay = 1240,
		.vsync_start = 1240 + 12,
		.vsync_end = 1240 + 12 + 4,
		.vtotal = 1240 + 12 + 4 + 12,
	}
};

static struct ch13726a_desc rp_classic_g1392_desc = {
	.modes = rp_classic_g1392_modes,
	.num_modes = ARRAY_SIZE(rp_classic_g1392_modes),
	.width_mm = 65,
	.height_mm = 75,
	.bpc = 8,
	.supplies = ch13726a_supplies_no_avdd,
	.num_supplies = ARRAY_SIZE(ch13726a_supplies_no_avdd),
	.vendor_page_dcs = true,
	.init_max_brightness = true,
	.reset_pulse_ms = 20,
	.power_on_delay_ms = 20,
	.exit_sleep_delay_ms = 120,
	.display_on_delay_ms = 150,
	.display_off_delay_ms = 16,
	.sleep_in_delay_ms = 50,
	.extra_mode_flags = MIPI_DSI_MODE_VIDEO_BURST,
};

static int ch13726a_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
{
	struct ch13726a_panel *ctx = to_ch13726a_panel(panel);

	ctx->connector = connector;

	for (uint8_t i = 0; i < ctx->desc->num_modes; i++) {
		const struct drm_display_mode *m = &ctx->desc->modes[i];
		struct drm_display_mode *mode;

		mode = drm_mode_duplicate(connector->dev, m);
		if (!mode) {
			dev_err(&ctx->dsi->dev, "failed to add mode %ux%u@%u\n",
				m->hdisplay, m->vdisplay, drm_mode_vrefresh(m));
			return -ENOMEM;
		}

		mode->type = DRM_MODE_TYPE_DRIVER;
		if (i == 0)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);
	}

	connector->display_info.width_mm = ctx->desc->width_mm;
	connector->display_info.height_mm = ctx->desc->height_mm;
	connector->display_info.bpc = ctx->desc->bpc;

	return ctx->desc->num_modes;
}

static enum drm_panel_orientation ch13726a_get_orientation(struct drm_panel *panel)
{
	struct ch13726a_panel *ctx = to_ch13726a_panel(panel);

	return ctx->orientation;
}

static const struct drm_panel_funcs ch13726a_panel_funcs = {
	.prepare = ch13726a_prepare,
	.unprepare = ch13726a_unprepare,
	.disable = ch13726a_disable,
	.get_modes = ch13726a_get_modes,
	.get_orientation = ch13726a_get_orientation,
};

static int ch13726a_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness(dsi, brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return 0;
}

static const struct backlight_ops ch13726a_bl_ops = {
	.update_status = ch13726a_bl_update_status,
};

static struct backlight_device *
ch13726a_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 255,
		.max_brightness = 255,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &ch13726a_bl_ops, &props);
}

static int ch13726a_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct ch13726a_panel *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, __typeof(*ctx), panel,
				   &ch13726a_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->desc = (struct ch13726a_desc *)of_device_get_match_data(dev);
	if (!ctx->desc)
		return -ENODEV;

	if (ctx->desc->supplies) {
		ctx->num_supplies = ctx->desc->num_supplies;
		ret = devm_regulator_bulk_get_const(dev, ctx->num_supplies,
						    ctx->desc->supplies,
						    &ctx->supplies);
	} else {
		ctx->num_supplies = ARRAY_SIZE(ch13726a_supplies);
		ret = devm_regulator_bulk_get_const(dev, ctx->num_supplies,
						    ch13726a_supplies,
						    &ctx->supplies);
	}
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ret = drm_of_get_panel_orientation(dev->of_node, &ctx->orientation);
	if (ret < 0) {
		dev_err(dev, "%pOF: failed to get orientation %d\n", dev->of_node, ret);
		return ret;
	}

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS |
			  ctx->desc->extra_mode_flags;

	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = ch13726a_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to attach to DSI host: %d\n", ret);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static void ch13726a_remove(struct mipi_dsi_device *dsi)
{
	struct ch13726a_panel *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id ch13726a_of_match[] = {
	{ .compatible = "ayntec,thor-panel-bottom", .data = &thor_bottom_desc },
	{ .compatible = "ayntec,thorlite-panel-bottom", .data = &retroidpocket_rpminiv2_desc },
	{ .compatible = "retroidpocket,rp5-panel", .data = &retroidpocket_rp5_desc },
	{ .compatible = "retroidpocket,rpmini-panel", .data = &retroidpocket_rpmini_desc },
	{ .compatible = "retroidpocket,rpminiv2-panel", .data = &retroidpocket_rpminiv2_desc },
	{ .compatible = "retroid,pocket-classic-panel", .data = &rp_classic_desc },
	{ .compatible = "retroid,pocket-classic-g1392-panel", .data = &rp_classic_g1392_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ch13726a_of_match);

static struct mipi_dsi_driver ch13726a_driver = {
	.probe = ch13726a_probe,
	.remove = ch13726a_remove,
	.driver = {
		.name = "panel-ch13726a-amoled",
		.of_match_table = ch13726a_of_match,
	},
};
module_mipi_dsi_driver(ch13726a_driver);

MODULE_DESCRIPTION("DRM driver for CH13726A DSI panels");
MODULE_LICENSE("GPL");
