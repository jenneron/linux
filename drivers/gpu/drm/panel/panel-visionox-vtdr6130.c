// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2023, Linaro Limited

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/of.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>

#include <video/mipi_display.h>

struct visionox_vtdr6130;

struct visionox_vtdr6130_desc {
	const struct regulator_bulk_data *supplies;
	unsigned int num_supplies;
	const struct drm_display_mode *modes;
	unsigned int num_modes;
	unsigned long mode_flags;
	const struct drm_dsc_config *dsc;
	unsigned int reset_delay_ms;
	int (*on)(struct visionox_vtdr6130 *ctx);
	const struct attribute_group *attr_group;
	u16 frame_len_120;
	u16 frame_len_60;
};

struct visionox_vtdr6130 {
	struct drm_panel panel;
	struct drm_connector *connector;
	const struct visionox_vtdr6130_desc *desc;
	struct drm_dsc_config dsc;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data *supplies;
	enum drm_panel_orientation orientation;
	bool enabled;
	u16 frame_len_120;
	u16 frame_len_60;
};

static const struct regulator_bulk_data visionox_vtdr6130_supplies[] = {
	{ .supply = "vddio" },
	{ .supply = "vci" },
	{ .supply = "vdd" },
};

static const struct regulator_bulk_data retroidpocket_rp6_supplies[] = {
	{ .supply = "vdd" },
	{ .supply = "vddio" },
	{ .supply = "vci" },
	{ .supply = "vdd18" },
	{ .supply = "vdd28" },
};

static inline struct visionox_vtdr6130 *to_visionox_vtdr6130(struct drm_panel *panel)
{
	return container_of(panel, struct visionox_vtdr6130, panel);
}

static void visionox_vtdr6130_reset(struct visionox_vtdr6130 *ctx)
{
	unsigned int delay = ctx->desc->reset_delay_ms * 1000;

	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(delay, delay + 1000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(delay, delay + 1000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(delay, delay + 1000);
}

static int visionox_vtdr6130_on(struct visionox_vtdr6130 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };
	struct drm_dsc_picture_parameter_set pps;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x03, 0x01);

	mipi_dsi_dcs_set_tear_on_multi(&dsi_ctx, MIPI_DSI_DCS_TEAR_MODE_VBLANK);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx,
				     MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx,
				     MIPI_DCS_SET_DISPLAY_BRIGHTNESS, 0x00,
				     0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x59, 0x09);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6c, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6d, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x70, 0x12, 0x00, 0x00, 0xab,
				     0x30, 0x80, 0x09, 0x60, 0x04, 0x38, 0x00,
				     0x28, 0x02, 0x1c, 0x02, 0x1c, 0x02, 0x00,
				     0x02, 0x0e, 0x00, 0x20, 0x03, 0xdd, 0x00,
				     0x07, 0x00, 0x0c, 0x02, 0x77, 0x02, 0x8b,
				     0x18, 0x00, 0x10, 0xf0, 0x07, 0x10, 0x20,
				     0x00, 0x06, 0x0f, 0x0f, 0x33, 0x0e, 0x1c,
				     0x2a, 0x38, 0x46, 0x54, 0x62, 0x69, 0x70,
				     0x77, 0x79, 0x7b, 0x7d, 0x7e, 0x02, 0x02,
				     0x22, 0x00, 0x2a, 0x40, 0x2a, 0xbe, 0x3a,
				     0xfc, 0x3a, 0xfa, 0x3a, 0xf8, 0x3b, 0x38,
				     0x3b, 0x78, 0x3b, 0xb6, 0x4b, 0xb6, 0x4b,
				     0xf4, 0x4b, 0xf4, 0x6c, 0x34, 0x84, 0x74,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xaa, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb1, 0x01, 0x38, 0x00, 0x14,
				     0x00, 0x1c, 0x00, 0x01, 0x66, 0x00, 0x14,
				     0x00, 0x14, 0x00, 0x01, 0x66, 0x00, 0x14,
				     0x05, 0xcc, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xaa, 0x13);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xce, 0x09, 0x11, 0x09, 0x11,
				     0x08, 0xc1, 0x07, 0xfa, 0x05, 0xa4, 0x00,
				     0x3c, 0x00, 0x34, 0x00, 0x24, 0x00, 0x0c,
				     0x00, 0x0c, 0x04, 0x00, 0x35);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xaa, 0x14);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb2, 0x03, 0x33);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb4, 0x00, 0x33, 0x00, 0x00,
				     0x00, 0x3e, 0x00, 0x00, 0x00, 0x3e, 0x00,
				     0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb5, 0x00, 0x09, 0x09, 0x09,
				     0x09, 0x09, 0x09, 0x06, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb9, 0x00, 0x00, 0x08, 0x09,
				     0x09, 0x09);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbc, 0x10, 0x00, 0x00, 0x06,
				     0x11, 0x09, 0x3b, 0x09, 0x47, 0x09, 0x47,
				     0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbe, 0x10, 0x10, 0x00, 0x08,
				     0x22, 0x09, 0x19, 0x09, 0x25, 0x09, 0x25,
				     0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x5a, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x65, 0x14);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfa, 0x08, 0x08, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x5a, 0x81);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x65, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf3, 0x0f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xaa, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x5a, 0x82);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf9, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x51, 0x83);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x65, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf8, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x5a, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x65, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf4, 0x9a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x5a, 0x00);

	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 20);

	drm_dsc_pps_payload_pack(&pps, dsi->dsc);
	mipi_dsi_picture_parameter_set_multi(&dsi_ctx, &pps);

	return dsi_ctx.accum_err;
}

static int visionox_vtdr6130_get_current_mode(struct visionox_vtdr6130 *ctx)
{
	struct drm_connector *connector = ctx->connector;
	struct drm_crtc_state *crtc_state;
	unsigned int i;

	if (!connector || !connector->state || !connector->state->crtc)
		return 0;

	crtc_state = connector->state->crtc->state;

	for (i = 0; i < ctx->desc->num_modes; i++) {
		if (drm_mode_match(&crtc_state->mode, &ctx->desc->modes[i],
				   DRM_MODE_MATCH_TIMINGS | DRM_MODE_MATCH_CLOCK))
			return i;
	}

	return 0;
}

static void retroidpocket_rp6_set_frame_len(struct mipi_dsi_multi_context *dsi_ctx,
					    struct visionox_vtdr6130 *ctx)
{
	u8 payload[] = {
		0xb1,
		0x02, 0x16, 0x00, 0x14,
		ctx->frame_len_120 >> 8, ctx->frame_len_120 & 0xff, 0x00,
		0x02, 0x16, 0x00, 0x14,
		ctx->frame_len_60 >> 8, ctx->frame_len_60 & 0xff, 0x00,
		0x02, 0x16, 0x00, 0x14, 0x00, 0x10, 0x00,
	};

	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xf0, 0xaa, 0x10);
	mipi_dsi_dcs_write_buffer_multi(dsi_ctx, payload, sizeof(payload));
}

static int retroidpocket_rp6_on(struct visionox_vtdr6130 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };
	struct drm_dsc_picture_parameter_set pps;
	int mode = visionox_vtdr6130_get_current_mode(ctx);
	int vrefresh = drm_mode_vrefresh(&ctx->desc->modes[mode]);

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x03, 0x01);
	mipi_dsi_dcs_set_tear_on_multi(&dsi_ctx, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx,
				     MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx,
				     MIPI_DCS_SET_DISPLAY_BRIGHTNESS, 0x07,
				     0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_ADDRESS_MODE, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x59, 0x09);
	if (vrefresh == 120)
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6c, 0x00);
	else
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6c, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6d, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x70, 0x11, 0x00, 0x00, 0x89,
				     0x30, 0x80, 0x07, 0x80, 0x04, 0x38, 0x00,
				     0x14, 0x02, 0x1c, 0x02, 0x1c, 0x02, 0x00,
				     0x02, 0x25, 0x00, 0x20, 0x01, 0xd5, 0x00,
				     0x07, 0x00, 0x0d, 0x05, 0x7a, 0x05, 0x16,
				     0x18, 0x00, 0x10, 0xf0, 0x03, 0x0c, 0x20,
				     0x00, 0x06, 0x0b, 0x0b, 0x33, 0x0e, 0x1c,
				     0x2a, 0x38, 0x46, 0x54, 0x62, 0x69, 0x70,
				     0x77, 0x79, 0x7b, 0x7d, 0x7e, 0x01, 0x02,
				     0x01, 0x00, 0x09, 0x40, 0x09, 0xbe, 0x19,
				     0xfc, 0x19, 0xfa, 0x19, 0xf8, 0x1a, 0x38,
				     0x1a, 0x78, 0x1a, 0xb6, 0x2a, 0xb6, 0x2a,
				     0xf4, 0x2a, 0xf4, 0x4b, 0x34, 0x63, 0x74,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	retroidpocket_rp6_set_frame_len(&dsi_ctx, ctx);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xaa, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd0, 0x80, 0x05, 0xa0, 0x14,
				     0x14, 0x00, 0x35, 0x2d, 0x17, 0x22, 0x00,
				     0x00, 0x2d, 0x17, 0x32, 0x00, 0x00, 0x00,
				     0x05, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xaa, 0x13);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc9, 0x07, 0xa4, 0x06, 0xe0,
				     0x06, 0xe0, 0x06, 0xe0, 0x05, 0xe4, 0x04,
				     0xcc, 0x03, 0xe8, 0x00, 0xf0, 0x00, 0x0c,
				     0x00, 0x0c, 0x17, 0x00, 0xc5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xce, 0x0f, 0x48, 0x0d, 0xc0,
				     0x0d, 0xc0, 0x0d, 0xc0, 0x0b, 0xbc, 0x09,
				     0x98, 0x07, 0xd0, 0x01, 0xe0, 0x00, 0x14,
				     0x00, 0x14, 0x17, 0x00, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xaa, 0x14);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb2, 0x03, 0x33);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb4, 0x0a, 0x72, 0x00, 0x00,
				     0x0a, 0x72, 0x00, 0x00, 0x0a, 0x72, 0x00,
				     0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb5, 0x00, 0x1e, 0x1e, 0x1e,
				     0x1e, 0x1e, 0x1e, 0x06, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb9, 0x00, 0x00, 0x08, 0x1e,
				     0x1e, 0x1e);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbc, 0x10, 0x00, 0x00, 0x08,
				     0x11, 0x1e, 0x7c, 0x1e, 0x7c, 0x1e, 0x7c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbe, 0x10, 0x10, 0x00, 0x0c,
				     0x22, 0x1e, 0x62, 0x1e, 0x62, 0x1e, 0x62);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x5a, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x65, 0x13);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfd, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x5a, 0x81);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x65, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf3, 0x0f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x5a, 0x82);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf9, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x5a, 0x83);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x65, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf8, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xaa, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x5a, 0x00);

	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 150);

	drm_dsc_pps_payload_pack(&pps, dsi->dsc);
	mipi_dsi_picture_parameter_set_multi(&dsi_ctx, &pps);

	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 50);

	return dsi_ctx.accum_err;
}

static int visionox_vtdr6130_enable(struct drm_panel *panel)
{
	struct visionox_vtdr6130 *ctx = to_visionox_vtdr6130(panel);

	ctx->enabled = true;

	return 0;
}

static int visionox_vtdr6130_disable(struct drm_panel *panel)
{
	struct visionox_vtdr6130 *ctx = to_visionox_vtdr6130(panel);
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };

	ctx->enabled = false;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 20);

	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	return dsi_ctx.accum_err;
}

static int visionox_vtdr6130_prepare(struct drm_panel *panel)
{
	struct visionox_vtdr6130 *ctx = to_visionox_vtdr6130(panel);
	int ret;

	ret = regulator_bulk_enable(ctx->desc->num_supplies, ctx->supplies);
	if (ret < 0)
		return ret;

	visionox_vtdr6130_reset(ctx);

	ret = ctx->desc->on(ctx);
	if (ret < 0) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		regulator_bulk_disable(ctx->desc->num_supplies, ctx->supplies);
		return ret;
	}

	return 0;
}

static int visionox_vtdr6130_unprepare(struct drm_panel *panel)
{
	struct visionox_vtdr6130 *ctx = to_visionox_vtdr6130(panel);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	regulator_bulk_disable(ctx->desc->num_supplies, ctx->supplies);

	return 0;
}

static const struct drm_display_mode visionox_vtdr6130_modes[] = {
	{
		.clock = (1080 + 20 + 2 + 20) * (2400 + 20 + 2 + 18) * 144 / 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + 20,
		.hsync_end = 1080 + 20 + 2,
		.htotal = 1080 + 20 + 2 + 20,
		.vdisplay = 2400,
		.vsync_start = 2400 + 20,
		.vsync_end = 2400 + 20 + 2,
		.vtotal = 2400 + 20 + 2 + 18,
		.width_mm = 71,
		.height_mm = 157,
	},
};

static const struct drm_display_mode retroidpocket_rp6_modes[] = {
	{
		.clock = (1080 + 16 + 2 + 22) * (1920 + 20 + 4 + 16) * 120 / 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + 16,
		.hsync_end = 1080 + 16 + 2,
		.htotal = 1080 + 16 + 2 + 22,
		.vdisplay = 1920,
		.vsync_start = 1920 + 20,
		.vsync_end = 1920 + 20 + 4,
		.vtotal = 1920 + 20 + 4 + 16,
		.width_mm = 68,
		.height_mm = 120,
	},
	{
		.clock = (1080 + 16 + 2 + 22) * (1920 + 16 + 4 + 1972) * 60 / 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + 16,
		.hsync_end = 1080 + 16 + 2,
		.htotal = 1080 + 16 + 2 + 22,
		.vdisplay = 1920,
		.vsync_start = 1920 + 16,
		.vsync_end = 1920 + 16 + 4,
		.vtotal = 1920 + 16 + 4 + 1972,
		.width_mm = 68,
		.height_mm = 120,
	},
};

static int visionox_vtdr6130_get_modes(struct drm_panel *panel,
				       struct drm_connector *connector)
{
	struct visionox_vtdr6130 *ctx = to_visionox_vtdr6130(panel);
	struct drm_display_mode *mode;
	unsigned int i;

	for (i = 0; i < ctx->desc->num_modes; i++) {
		mode = drm_mode_duplicate(connector->dev, &ctx->desc->modes[i]);
		if (!mode)
			return -ENOMEM;

		drm_mode_set_name(mode);

		mode->type = DRM_MODE_TYPE_DRIVER;
		if (i == 0)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_probed_add(connector, mode);
	}

	connector->display_info.width_mm = ctx->desc->modes[0].width_mm;
	connector->display_info.height_mm = ctx->desc->modes[0].height_mm;
	ctx->connector = connector;

	return ctx->desc->num_modes;
}

static enum drm_panel_orientation
visionox_vtdr6130_get_orientation(struct drm_panel *panel)
{
	struct visionox_vtdr6130 *ctx = to_visionox_vtdr6130(panel);

	return ctx->orientation;
}

static const struct drm_panel_funcs visionox_vtdr6130_panel_funcs = {
	.disable = visionox_vtdr6130_disable,
	.enable = visionox_vtdr6130_enable,
	.prepare = visionox_vtdr6130_prepare,
	.unprepare = visionox_vtdr6130_unprepare,
	.get_modes = visionox_vtdr6130_get_modes,
	.get_orientation = visionox_vtdr6130_get_orientation,
};

static int visionox_vtdr6130_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
	ret = mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);
	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return ret;
}

static int visionox_vtdr6130_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
	ret = mipi_dsi_dcs_get_display_brightness_large(dsi, &brightness);
	dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	if (ret < 0)
		return ret;

	return brightness;
}

static const struct backlight_ops visionox_vtdr6130_bl_ops = {
	.update_status = visionox_vtdr6130_bl_update_status,
	.get_brightness = visionox_vtdr6130_bl_get_brightness,
};

static struct backlight_device *
visionox_vtdr6130_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 4095,
		.max_brightness = 4095,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &visionox_vtdr6130_bl_ops, &props);
}

static ssize_t retroidpocket_rp6_frame_len_store(struct device *dev,
						 u16 *frame_len, u16 min,
						 u16 max, const char *buf,
						 size_t count)
{
	struct visionox_vtdr6130 *ctx = dev_get_drvdata(dev);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	u16 val;
	int ret;

	ret = kstrtou16(buf, 0, &val);
	if (ret)
		return ret;

	if (val < min || val > max)
		return -EINVAL;

	*frame_len = val;

	if (!ctx->enabled)
		return count;

	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
	retroidpocket_rp6_set_frame_len(&dsi_ctx, ctx);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xaa, 0x00);
	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	if (dsi_ctx.accum_err)
		return dsi_ctx.accum_err;

	return count;
}

static ssize_t frame_len_120_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct visionox_vtdr6130 *ctx = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", ctx->frame_len_120);
}

static ssize_t frame_len_120_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct visionox_vtdr6130 *ctx = dev_get_drvdata(dev);

	return retroidpocket_rp6_frame_len_store(dev, &ctx->frame_len_120,
						 0x10, 0x80, buf, count);
}
static DEVICE_ATTR_RW(frame_len_120);

static ssize_t frame_len_60_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct visionox_vtdr6130 *ctx = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", ctx->frame_len_60);
}

static ssize_t frame_len_60_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct visionox_vtdr6130 *ctx = dev_get_drvdata(dev);

	return retroidpocket_rp6_frame_len_store(dev, &ctx->frame_len_60,
						 0x76c, 0x834, buf, count);
}
static DEVICE_ATTR_RW(frame_len_60);

static struct attribute *retroidpocket_rp6_attrs[] = {
	&dev_attr_frame_len_120.attr,
	&dev_attr_frame_len_60.attr,
	NULL
};

static const struct attribute_group retroidpocket_rp6_attr_group = {
	.attrs = retroidpocket_rp6_attrs,
};

static const struct drm_dsc_config visionox_vtdr6130_dsc = {
	.dsc_version_major = 0x1,
	.dsc_version_minor = 0x2,
	.slice_height = 40,
	.slice_width = 540,
	.slice_count = 2,
	.bits_per_component = 8,
	.bits_per_pixel = 8 << 4,
	.block_pred_enable = true,
};

static const struct visionox_vtdr6130_desc visionox_vtdr6130_desc = {
	.supplies = visionox_vtdr6130_supplies,
	.num_supplies = ARRAY_SIZE(visionox_vtdr6130_supplies),
	.modes = visionox_vtdr6130_modes,
	.num_modes = ARRAY_SIZE(visionox_vtdr6130_modes),
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_NO_EOT_PACKET |
		      MIPI_DSI_CLOCK_NON_CONTINUOUS,
	.dsc = &visionox_vtdr6130_dsc,
	.reset_delay_ms = 10,
	.on = visionox_vtdr6130_on,
};

static const struct drm_dsc_config retroidpocket_rp6_dsc = {
	.dsc_version_major = 0x1,
	.dsc_version_minor = 0x1,
	.slice_height = 12,
	.slice_width = 540,
	.slice_count = 2,
	.bits_per_component = 8,
	.bits_per_pixel = 8 << 4,
	.block_pred_enable = true,
};

static const struct visionox_vtdr6130_desc retroidpocket_rp6_desc = {
	.supplies = retroidpocket_rp6_supplies,
	.num_supplies = ARRAY_SIZE(retroidpocket_rp6_supplies),
	.modes = retroidpocket_rp6_modes,
	.num_modes = ARRAY_SIZE(retroidpocket_rp6_modes),
	.mode_flags = MIPI_DSI_MODE_NO_EOT_PACKET |
		      MIPI_DSI_CLOCK_NON_CONTINUOUS,
	.dsc = &retroidpocket_rp6_dsc,
	.reset_delay_ms = 20,
	.on = retroidpocket_rp6_on,
	.attr_group = &retroidpocket_rp6_attr_group,
	.frame_len_120 = 0x1c,
	.frame_len_60 = 0x7d0,
};

static int visionox_vtdr6130_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct visionox_vtdr6130 *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct visionox_vtdr6130, panel,
				   &visionox_vtdr6130_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->desc = of_device_get_match_data(dev);
	if (!ctx->desc)
		return -ENODEV;

	ret = devm_regulator_bulk_get_const(&dsi->dev, ctx->desc->num_supplies,
					    ctx->desc->supplies,
					    &ctx->supplies);
	if (ret < 0)
		return ret;

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->frame_len_120 = ctx->desc->frame_len_120;
	ctx->frame_len_60 = ctx->desc->frame_len_60;

	if (ctx->desc->attr_group) {
		ret = devm_device_add_group(dev, ctx->desc->attr_group);
		if (ret)
			return ret;
	}

	ret = drm_of_get_panel_orientation(dev->of_node, &ctx->orientation);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to get orientation\n");

	ctx->dsc = *ctx->desc->dsc;

	dsi->dsc = &ctx->dsc;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = ctx->desc->mode_flags;
	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = visionox_vtdr6130_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	ret = devm_drm_panel_add(dev, &ctx->panel);
	if (ret)
		return ret;

	return devm_mipi_dsi_attach(dev, dsi);
}

static const struct of_device_id visionox_vtdr6130_of_match[] = {
	{ .compatible = "retroidpocket,rp6-panel", .data = &retroidpocket_rp6_desc },
	{ .compatible = "retroidpocket,rpduo-panel-top", .data = &retroidpocket_rp6_desc },
	{ .compatible = "visionox,vtdr6130", .data = &visionox_vtdr6130_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, visionox_vtdr6130_of_match);

static struct mipi_dsi_driver visionox_vtdr6130_driver = {
	.probe = visionox_vtdr6130_probe,
	.driver = {
		.name = "panel-visionox-vtdr6130",
		.of_match_table = visionox_vtdr6130_of_match,
	},
};
module_mipi_dsi_driver(visionox_vtdr6130_driver);

MODULE_AUTHOR("Neil Armstrong <neil.armstrong@linaro.org>");
MODULE_DESCRIPTION("Panel driver for Visionox VTDR6130 based AMOLED DSI panels");
MODULE_LICENSE("GPL");
