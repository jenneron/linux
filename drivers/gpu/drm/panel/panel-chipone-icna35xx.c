// SPDX-License-Identifier: GPL-2.0-only
/*
 * Chipone ICNA35XX Driver IC panels driver
 *
 * Copyright (c) 2025 Teguh Sobirin <teguh@sobir.in>
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

struct panel_info {
	struct drm_panel panel;
	struct drm_connector *connector;
	struct mipi_dsi_device *dsi;
	struct panel_desc *desc;
	enum drm_panel_orientation orientation;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data *supplies;
	bool enabled;

	int vrefresh;
	u16 frame_len_120;
	u16 frame_len_60;
};

struct panel_desc {
	unsigned int width_mm;
	unsigned int height_mm;

	unsigned int bpc;
	unsigned int lanes;
	unsigned long mode_flags;
	enum mipi_dsi_pixel_format format;
	unsigned int max_brightness;
	unsigned int min_brightness;

	const struct drm_display_mode *modes;
	unsigned int num_modes;
	int (*init_sequence)(struct panel_info *pinfo);

	unsigned int display_off_ms;
	unsigned int enter_sleep_ms;

	struct drm_dsc_config dsc;

	u16 frame_len_120;
	u16 frame_len_60;
};

static const struct regulator_bulk_data panel_supplies[] = {
	{ .supply = "vdd" },
	{ .supply = "vddio" },
	{ .supply = "vci" },
	{ .supply = "disp" },
	{ .supply = "blvdd" },
};

static inline struct panel_info *to_panel_info(struct drm_panel *panel)
{
	return container_of(panel, struct panel_info, panel);
}

static int icna35xx_get_current_vrefresh(struct panel_info *pinfo)
{
	struct drm_connector *connector = pinfo->connector;
	struct drm_crtc_state *crtc_state;
	unsigned int i;

	/* Use the default (first) mode if no info is available yet */
	if (connector && connector->state && connector->state->crtc) {
		crtc_state = connector->state->crtc->state;

		for (i = 0; i < pinfo->desc->num_modes; i++) {
			if (drm_mode_match(&crtc_state->mode,
					   &pinfo->desc->modes[i],
					   DRM_MODE_MATCH_TIMINGS |
					   DRM_MODE_MATCH_CLOCK))
				return drm_mode_vrefresh(&pinfo->desc->modes[i]);
		}
	}

	return drm_mode_vrefresh(&pinfo->desc->modes[0]);
}

static int icna3512_init_sequence(struct panel_info *pinfo)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = pinfo->dsi };
	struct drm_dsc_picture_parameter_set pps;
	int vrefresh = icna35xx_get_current_vrefresh(pinfo);

	pinfo->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x9C, 0xA5, 0xA5);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xFD, 0x5A, 0x5A);

	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x53, 0xE0);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x35, 0x00);

	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);

	mipi_dsi_msleep(&dsi_ctx, 120);

	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x9F, 0x0F);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xCE, 0x22);

	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x9F, 0x01);

	switch (vrefresh) {
	case 120:
		mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xB3,
			0x00, 0xE0, 0xA0, 0x10, 0xC8, 0x00, 0x02, 0x83,
			0x00, 0x10, 0x14, 0x00, 0x00, 0xC3, 0x00, 0x10,
			0x14, 0x00, 0x00, 0xE0, 0x10, 0x10, 0x9C, 0x00,
			0x00, 0xE0, 0xA0, 0x10, 0xC8, 0x22, 0x18, 0x18,
			0x18, 0x18, 0x18);
		mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x9F, 0x07);
		mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xB5,
			0x04, 0x0C, 0x08, 0x0C, 0x04, 0x00, 0xC4);
		mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xD9,
			0x88, 0x40, 0x40, 0x88, 0x40, 0x40, 0x00, 0xEB,
			0x11, 0xFF);
		mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xCE,
			0x01, 0x01, 0x01, 0x01, 0x04, 0x09, 0x2C);
		mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x48, 0x30);
		break;
	case 60:
		mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xB3,
			0x00, 0xE0, 0xA0, 0x10, 0xC8, 0x00);
		mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x9F, 0x07);
		mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xB2,
			0x04, 0x18, 0x08, 0x0C, 0x02, 0x00, 0xC4);
		mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xD3,
			0x88, 0x4A, 0x4A, 0x88, 0x4A, 0x4A, 0x00, 0xEB,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
		mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xCB,
			0x01, 0x01, 0x01, 0x01, 0x04, 0x2C);
		mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x48, 0x00);
		break;
	default:
		/* 165 hz */
		mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x48, 0x20);
		break;
	}

	drm_dsc_pps_payload_pack(&pps, &pinfo->desc->dsc);
	mipi_dsi_picture_parameter_set_multi(&dsi_ctx, &pps);

	mipi_dsi_msleep(&dsi_ctx, 20);

	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);

	return dsi_ctx.accum_err;
}

static void icna3520_set_frame_len(struct mipi_dsi_multi_context *dsi_ctx,
				   struct panel_info *pinfo)
{
	u8 payload_120[] = {
		0xB3,
		0x00, 0xD8, 0x00, 0x1C,
		pinfo->frame_len_120 >> 8, pinfo->frame_len_120 & 0xff,
	};
	u8 payload_60[] = {
		0xB3,
		0x00, 0xDB, 0x00, 0x1C, 0x00, 0x1C, 0x00,
		0x00, 0xDB, 0x00, 0x1C,
		pinfo->frame_len_60 >> 8, pinfo->frame_len_60 & 0xff, 0x00,
	};

	if (pinfo->vrefresh == 60) {
		mipi_dsi_generic_write_seq_multi(dsi_ctx, 0x48, 0x10);
		mipi_dsi_generic_write_seq_multi(dsi_ctx, 0x9F, 0x00);
		mipi_dsi_generic_write_multi(dsi_ctx, payload_60,
					     sizeof(payload_60));
	} else {
		mipi_dsi_generic_write_seq_multi(dsi_ctx, 0x48, 0x00);
		mipi_dsi_generic_write_seq_multi(dsi_ctx, 0x9F, 0x00);
		mipi_dsi_generic_write_multi(dsi_ctx, payload_120,
					     sizeof(payload_120));
	}
}

static int icna3520_init_sequence(struct panel_info *pinfo)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = pinfo->dsi };
	struct drm_dsc_picture_parameter_set pps;

	pinfo->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x9C, 0xA5, 0xA5);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xFD, 0x5A, 0x5A);

	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x53, 0xE0);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x35, 0x00);

	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);

	mipi_dsi_msleep(&dsi_ctx, 120);

	pinfo->vrefresh = icna35xx_get_current_vrefresh(pinfo);
	icna3520_set_frame_len(&dsi_ctx, pinfo);

	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x9F, 0x01);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xB2, 0x00);

	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x9F, 0x0D);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xB2, 0x27);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xB6, 0x03);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xBB, 0x01);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xB2, 0x24);

	drm_dsc_pps_payload_pack(&pps, &pinfo->desc->dsc);
	mipi_dsi_picture_parameter_set_multi(&dsi_ctx, &pps);

	mipi_dsi_msleep(&dsi_ctx, 20);

	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);

	return dsi_ctx.accum_err;
}

static int mangmi_pocket_max_init_sequence(struct panel_info *pinfo)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = pinfo->dsi };
	struct drm_dsc_picture_parameter_set pps;
	int vrefresh = icna35xx_get_current_vrefresh(pinfo);

	pinfo->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9c, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfd, 0x5a, 0x5a);
	if (vrefresh >= 90)
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x48, 0x30);
	else
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x48, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9f, 0x0f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xce, 0x22);
	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 255);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9f, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xdf, 0xc5, 0x86);

	switch (vrefresh) {
	case 144:
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb3, 0x00, 0xe0, 0xa0,
					     0x10, 0xc8, 0x00, 0x02, 0x83, 0x00,
					     0x10, 0x14, 0x00, 0x00, 0xc3, 0x00,
					     0x10, 0x14, 0x00, 0x00, 0xe0, 0x00,
					     0x10, 0x14, 0x00, 0x00, 0xe0, 0xa0,
					     0x10, 0xc8, 0x22, 0x18, 0x18, 0x18,
					     0x18, 0x18);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9f, 0x07);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb5, 0x04, 0x0a, 0x08,
					     0x0a, 0x04, 0x00, 0xc4);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd9, 0x66, 0xe4, 0xe4,
					     0x66, 0xe4, 0xe4, 0x00, 0xc4, 0x00,
					     0x00, 0x00, 0x00, 0x00, 0x00,
					     0x00);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xce, 0x01, 0x01, 0x01,
					     0x01, 0x04, 0x07, 0xa4);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x48, 0x30);
		break;
	case 120:
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb3, 0x00, 0xe0, 0xa0,
					     0x10, 0xc8, 0x00, 0x02, 0x83, 0x00,
					     0x10, 0x14, 0x00, 0x00, 0xc3, 0x00,
					     0x10, 0x14, 0x00, 0x00, 0xe0, 0x10,
					     0x10, 0x9c, 0x00, 0x00, 0xe0, 0xa0,
					     0x10, 0xc8, 0x22, 0x18, 0x18, 0x18,
					     0x18, 0x18);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9f, 0x07);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb5, 0x04, 0x0c, 0x08,
					     0x0c, 0x04, 0x00, 0xc4);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd9, 0x88, 0x40, 0x40,
					     0x88, 0x40, 0x40, 0x00, 0xeb, 0x00,
					     0x00, 0x00, 0x00, 0x00, 0x00,
					     0x00);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xce, 0x01, 0x01, 0x01,
					     0x01, 0x04, 0x09, 0x2c);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x48, 0x30);
		break;
	case 90:
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb3, 0x00, 0xe0, 0x40,
					     0x10, 0xa8, 0x00);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9f, 0x07);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb2, 0x04, 0x10, 0x08,
					     0x0c, 0x04, 0x00, 0xc4);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd3, 0x55, 0x80, 0x80,
					     0x55, 0x80, 0x80, 0x00, 0x9c, 0x00,
					     0x00, 0x00, 0x00, 0x00, 0x00,
					     0x00);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcb, 0x01, 0x01, 0x01,
					     0x01, 0x04, 0x06, 0x1c);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x48, 0x00);
		break;
	default: /* 60 Hz */
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb3, 0x00, 0xe0, 0xa0,
					     0x10, 0xc8, 0x00);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9f, 0x07);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb2, 0x04, 0x18, 0x08,
					     0x0c, 0x04, 0x00, 0xc4);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd3, 0x88, 0x4a, 0x4a,
					     0x88, 0x4a, 0x4a, 0x00, 0xeb, 0x00,
					     0x00, 0x00, 0x00, 0x00, 0x00,
					     0x00);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcb, 0x01, 0x01, 0x01,
					     0x01, 0x04, 0x09, 0x2c);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x48, 0x00);
		break;
	}

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9f, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbe, 0x04, 0x00, 0x10, 0x10,
				     0x00, 0x03, 0x02, 0x07, 0xa4, 0x01, 0x00,
				     0x00, 0x40, 0x20, 0x00, 0x00);
	mipi_dsi_dcs_set_display_brightness_multi(&dsi_ctx, 0x0000);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				     0xe0);
	mipi_dsi_dcs_set_tear_on_multi(&dsi_ctx, MIPI_DSI_DCS_TEAR_MODE_VBLANK);

	drm_dsc_pps_payload_pack(&pps, &pinfo->desc->dsc);
	mipi_dsi_picture_parameter_set_multi(&dsi_ctx, &pps);

	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 50);

	return dsi_ctx.accum_err;
}

static const struct drm_display_mode odin2portal_modes[] = {
	{
		/* 165Hz */
		.clock = (1080 + 98 + 1 + 23) * (1920 + 20 + 1 + 15) * 165 / 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + 98,
		.hsync_end = 1080 + 98 + 1,
		.htotal = 1080 + 98 + 1 + 23,
		.vdisplay = 1920,
		.vsync_start = 1920 + 20,
		.vsync_end = 1920 + 20 + 1,
		.vtotal = 1920 + 20 + 1 + 15,
	},
	{
		/* 120Hz */
		.clock = (1080 + 156 + 1 + 23) * (1920 + 412 + 1 + 15) * 120 / 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + 156,
		.hsync_end = 1080 + 156 + 1,
		.htotal = 1080 + 156 + 1 + 23,
		.vdisplay = 1920,
		.vsync_start = 1920 + 412,
		.vsync_end = 1920 + 412 + 1,
		.vtotal = 1920 + 412 + 1 + 15,
	},
	{
		/* 60Hz */
		.clock = (1080 + 156 + 1 + 23) * (1920 + 2760 + 1 + 15) * 60 / 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + 156,
		.hsync_end = 1080 + 156 + 1,
		.htotal = 1080 + 156 + 1 + 23,
		.vdisplay = 1920,
		.vsync_start = 1920 + 2760,
		.vsync_end = 1920 + 2760 + 1,
		.vtotal = 1920 + 2760 + 1 + 15,
	},
};

static const struct drm_display_mode thor_top_modes[] = {
	{
		/* 120Hz */
		.clock = (1080 + 24 + 1 + 24) * (1920 + 28 + 1 + 28) * 120 / 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + 24,
		.hsync_end = 1080 + 24 + 1,
		.htotal = 1080 + 24 + 1 + 24,
		.vdisplay = 1920,
		.vsync_start = 1920 + 28,
		.vsync_end = 1920 + 28 + 1,
		.vtotal = 1920 + 28 + 1 + 28,
	},
	{
		/* 60Hz */
		.clock = (1080 + 24 + 1 + 24) * (1920 + 2006 + 1 + 28) * 60 / 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + 24,
		.hsync_end = 1080 + 24 + 1,
		.htotal = 1080 + 24 + 1 + 24,
		.vdisplay = 1920,
		.vsync_start = 1920 + 2006,
		.vsync_end = 1920 + 2006 + 1,
		.vtotal = 1920 + 2006 + 1 + 28,
	},
};

static struct panel_desc odin2portal_desc = {
	.modes = odin2portal_modes,
	.num_modes = ARRAY_SIZE(odin2portal_modes),
	.width_mm = 160,
	.height_mm = 89,
	.bpc = 8,
	.lanes = 4,
	.format = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_NO_EOT_PACKET | MIPI_DSI_CLOCK_NON_CONTINUOUS |
			MIPI_DSI_MODE_LPM,
	.init_sequence = icna3512_init_sequence,
	.dsc = {
		.dsc_version_major = 0x1,
		.dsc_version_minor = 0x1,
		.slice_height = 20,
		.slice_width = 540,
		.slice_count = 2,
		.bits_per_component = 8,
		.bits_per_pixel = 8 << 4,
		.block_pred_enable = true,
	},
};

static struct panel_desc thor_top_desc = {
	.modes = thor_top_modes,
	.num_modes = ARRAY_SIZE(thor_top_modes),
	.width_mm = 136,
	.height_mm = 68,
	.bpc = 8,
	.lanes = 4,
	.format = MIPI_DSI_FMT_RGB888,
	.mode_flags =  MIPI_DSI_MODE_NO_EOT_PACKET | MIPI_DSI_CLOCK_NON_CONTINUOUS |
			MIPI_DSI_MODE_LPM,
	.init_sequence = icna3520_init_sequence,
	.frame_len_120 = 0x4C,
	.frame_len_60 = 0x7D6,
	.dsc = {
		.dsc_version_major = 0x1,
		.dsc_version_minor = 0x1,
		.slice_height = 12,
		.slice_width = 540,
		.slice_count = 2,
		.bits_per_component = 8,
		.bits_per_pixel = 8 << 4,
		.block_pred_enable = true,
	},
};

static const struct drm_display_mode mangmi_pocket_max_modes[] = {
	{
		/* 60 Hz */
		.clock = (1080 + 156 + 1 + 23) * (1920 + 20 + 1 + 15) * 60 /
			 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + 156,
		.hsync_end = 1080 + 156 + 1,
		.htotal = 1080 + 156 + 1 + 23,
		.vdisplay = 1920,
		.vsync_start = 1920 + 20,
		.vsync_end = 1920 + 20 + 1,
		.vtotal = 1920 + 20 + 1 + 15,
	},
	{
		/* 90 Hz */
		.clock = (1080 + 156 + 1 + 23) * (1920 + 20 + 1 + 15) * 90 /
			 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + 156,
		.hsync_end = 1080 + 156 + 1,
		.htotal = 1080 + 156 + 1 + 23,
		.vdisplay = 1920,
		.vsync_start = 1920 + 20,
		.vsync_end = 1920 + 20 + 1,
		.vtotal = 1920 + 20 + 1 + 15,
	},
	{
		/* 120 Hz */
		.clock = (1080 + 156 + 1 + 23) * (1920 + 20 + 1 + 15) * 120 /
			 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + 156,
		.hsync_end = 1080 + 156 + 1,
		.htotal = 1080 + 156 + 1 + 23,
		.vdisplay = 1920,
		.vsync_start = 1920 + 20,
		.vsync_end = 1920 + 20 + 1,
		.vtotal = 1920 + 20 + 1 + 15,
	},
	{
		/* 144 Hz */
		.clock = (1080 + 156 + 1 + 23) * (1920 + 20 + 1 + 15) * 144 /
			 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + 156,
		.hsync_end = 1080 + 156 + 1,
		.htotal = 1080 + 156 + 1 + 23,
		.vdisplay = 1920,
		.vsync_start = 1920 + 20,
		.vsync_end = 1920 + 20 + 1,
		.vtotal = 1920 + 20 + 1 + 15,
	},
};

static struct panel_desc mangmi_pocket_max_desc = {
	.modes = mangmi_pocket_max_modes,
	.num_modes = ARRAY_SIZE(mangmi_pocket_max_modes),
	.width_mm = 87,
	.height_mm = 155,
	.bpc = 8,
	.lanes = 4,
	.format = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM,
	.max_brightness = 3515,
	.min_brightness = 1,
	.init_sequence = mangmi_pocket_max_init_sequence,
	.display_off_ms = 120,
	.enter_sleep_ms = 10,
	.dsc = {
		.dsc_version_major = 0x1,
		.dsc_version_minor = 0x1,
		.slice_height = 20,
		.slice_width = 540,
		.slice_count = 2,
		.bits_per_component = 8,
		.bits_per_pixel = 8 << 4,
		.block_pred_enable = true,
	},
};

static void icna35xx_reset(struct panel_info *pinfo)
{
	gpiod_set_value_cansleep(pinfo->reset_gpio, 0);
	usleep_range(20000, 21000);
	gpiod_set_value_cansleep(pinfo->reset_gpio, 1);
	usleep_range(20000, 21000);
	gpiod_set_value_cansleep(pinfo->reset_gpio, 0);
	usleep_range(20000, 21000);
}

static int icna35xx_prepare(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(panel_supplies), pinfo->supplies);
	if (ret < 0) {
		dev_err(panel->dev, "failed to enable regulators: %d\n", ret);
		return ret;
	}

	icna35xx_reset(pinfo);

	ret = pinfo->desc->init_sequence(pinfo);
	if (ret < 0) {
		regulator_bulk_disable(ARRAY_SIZE(panel_supplies), pinfo->supplies);
		dev_err(panel->dev, "failed to initialize panel: %d\n", ret);
		return ret;
	}

	pinfo->enabled = true;

	return 0;
}

static int icna35xx_disable(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = pinfo->dsi };

	pinfo->enabled = false;
	pinfo->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, pinfo->desc->display_off_ms ?: 50);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, pinfo->desc->enter_sleep_ms ?: 120);

	return dsi_ctx.accum_err;
}

static int icna35xx_unprepare(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);

	gpiod_set_value_cansleep(pinfo->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(panel_supplies), pinfo->supplies);

	return 0;
}

static int icna35xx_get_modes(struct drm_panel *panel,
			       struct drm_connector *connector)
{
	struct panel_info *pinfo = to_panel_info(panel);
	unsigned int i;

	for (i = 0; i < pinfo->desc->num_modes; i++) {
		const struct drm_display_mode *m = &pinfo->desc->modes[i];
		struct drm_display_mode *mode;

		mode = drm_mode_duplicate(connector->dev, m);
		if (!mode)
			return -ENOMEM;

		mode->type = DRM_MODE_TYPE_DRIVER;
		if (i == 0)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);
	}

	connector->display_info.width_mm = pinfo->desc->width_mm;
	connector->display_info.height_mm = pinfo->desc->height_mm;
	connector->display_info.bpc = pinfo->desc->bpc;
	pinfo->connector = connector;

	return pinfo->desc->num_modes;
}

static enum drm_panel_orientation icna35xx_get_orientation(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);

	return pinfo->orientation;
}

static const struct drm_panel_funcs icna35xx_panel_funcs = {
	.disable = icna35xx_disable,
	.prepare = icna35xx_prepare,
	.unprepare = icna35xx_unprepare,
	.get_modes = icna35xx_get_modes,
	.get_orientation = icna35xx_get_orientation,
};

static int icna35xx_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct panel_info *pinfo = mipi_dsi_get_drvdata(dsi);
	unsigned int min = pinfo->desc->min_brightness;
	unsigned int max = bl->props.max_brightness;
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	/* Map the non-zero range onto [min, max] for panels that blank at low levels */
	if (min && brightness)
		brightness = min + (max - min) * brightness / max;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return ret;
}

static int icna35xx_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness_large(dsi, &brightness);

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return ret < 0 ? ret : brightness;
}

static const struct backlight_ops icna35xx_bl_ops = {
	.update_status = icna35xx_bl_update_status,
	.get_brightness = icna35xx_bl_get_brightness,
};

static struct backlight_device *icna35xx_create_backlight(struct mipi_dsi_device *dsi)
{
	struct panel_info *pinfo = mipi_dsi_get_drvdata(dsi);
	unsigned int max = pinfo->desc->max_brightness ?: 4096;
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = max,
		.max_brightness = max,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &icna35xx_bl_ops, &props);
}

static ssize_t icna3520_frame_len_store(struct device *dev, u16 *frame_len,
					int vrefresh, u16 min, u16 max,
					const char *buf, size_t count)
{
	struct panel_info *pinfo = dev_get_drvdata(dev);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = pinfo->dsi };
	u16 val;
	int ret;

	ret = kstrtou16(buf, 0, &val);
	if (ret)
		return ret;

	if (val < min || val > max)
		return -EINVAL;

	*frame_len = val;

	if (!pinfo->enabled || pinfo->vrefresh != vrefresh)
		return count;

	pinfo->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
	icna3520_set_frame_len(&dsi_ctx, pinfo);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x9F, 0x0D);
	pinfo->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	if (dsi_ctx.accum_err)
		return dsi_ctx.accum_err;

	return count;
}

static ssize_t frame_len_120_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct panel_info *pinfo = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", pinfo->frame_len_120);
}

static ssize_t frame_len_120_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct panel_info *pinfo = dev_get_drvdata(dev);

	return icna3520_frame_len_store(dev, &pinfo->frame_len_120, 120,
					0x10, 0x80, buf, count);
}
static DEVICE_ATTR_RW(frame_len_120);

static ssize_t frame_len_60_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct panel_info *pinfo = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", pinfo->frame_len_60);
}

static ssize_t frame_len_60_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct panel_info *pinfo = dev_get_drvdata(dev);

	return icna3520_frame_len_store(dev, &pinfo->frame_len_60, 60,
					0x770, 0x838, buf, count);
}
static DEVICE_ATTR_RW(frame_len_60);

static struct attribute *icna3520_attrs[] = {
	&dev_attr_frame_len_120.attr,
	&dev_attr_frame_len_60.attr,
	NULL
};

static const struct attribute_group icna3520_group = {
	.attrs = icna3520_attrs,
};

static int icna35xx_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct panel_info *pinfo;
	int ret;

	pinfo = devm_drm_panel_alloc(dev, __typeof(*pinfo), panel,
				     &icna35xx_panel_funcs,
				     DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(pinfo))
		return PTR_ERR(pinfo);

	ret = devm_regulator_bulk_get_const(dev, ARRAY_SIZE(panel_supplies),
					    panel_supplies, &pinfo->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	pinfo->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(pinfo->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(pinfo->reset_gpio), "failed to get reset gpio\n");

	pinfo->desc = (struct panel_desc *)of_device_get_match_data(dev);
	if (!pinfo->desc)
		return -ENODEV;

	pinfo->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, pinfo);

	if (pinfo->desc->frame_len_120) {
		pinfo->frame_len_120 = pinfo->desc->frame_len_120;
		pinfo->frame_len_60 = pinfo->desc->frame_len_60;

		ret = devm_device_add_group(dev, &icna3520_group);
		if (ret)
			return ret;
	}

	ret = drm_of_get_panel_orientation(dev->of_node, &pinfo->orientation);
	if (ret < 0) {
		dev_err(dev, "%pOF: failed to get orientation %d\n", dev->of_node, ret);
		return ret;
	}

	pinfo->panel.prepare_prev_first = true;

	pinfo->panel.backlight = icna35xx_create_backlight(dsi);
	if (IS_ERR(pinfo->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(pinfo->panel.backlight),
				     "Failed to create backlight\n");

	ret = devm_drm_panel_add(dev, &pinfo->panel);
	if (ret)
		return ret;

	pinfo->dsi->lanes = pinfo->desc->lanes;
	pinfo->dsi->format = pinfo->desc->format;
	pinfo->dsi->mode_flags = pinfo->desc->mode_flags;
	pinfo->dsi->dsc = &pinfo->desc->dsc;

	return devm_mipi_dsi_attach(dev, dsi);
}

static const struct of_device_id icna35xx_of_match[] = {
	{ .compatible = "ayaneo,pocketds-panel-top", .data = &odin2portal_desc },
	{ .compatible = "ayaneo,pocketevo-panel", .data = &odin2portal_desc },
	{ .compatible = "ayntec,odin2portal-panel", .data = &odin2portal_desc },
	{ .compatible = "ayntec,odin3-panel", .data = &thor_top_desc },
	{ .compatible = "ayntec,thor-panel-top", .data = &thor_top_desc },
	{ .compatible = "mangmi,pocket-max-panel", .data = &mangmi_pocket_max_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, icna35xx_of_match);

static struct mipi_dsi_driver icna35xx_driver = {
	.probe = icna35xx_probe,
	.driver = {
		.name = "panel-chipone-icna35xx",
		.of_match_table = icna35xx_of_match,
	},
};
module_mipi_dsi_driver(icna35xx_driver);

MODULE_AUTHOR("Teguh Sobirin <teguh@sobir.in>");
MODULE_DESCRIPTION("DRM driver for Chipone ICNA35XX based MIPI DSI panels");
MODULE_LICENSE("GPL");
