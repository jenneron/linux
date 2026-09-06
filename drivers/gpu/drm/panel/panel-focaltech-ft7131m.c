// SPDX-License-Identifier: GPL-2.0-only
/*
 * FocalTech FT7131M 1080x1920 DSI video panel (Anbernic RG55G1).
 * Init sequence from Qualcomm Panel_ft7131m_1080p_video.xml.
 *
 * Ported from https://github.com/lualiliu/RG55G1-linux with the downstream
 * ABL continuous-splash coupling removed; the SM4450 display stack already
 * quiesces the bootloader scanout, so the panel is always reset and fully
 * initialised here rather than inherited from the bootloader.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

#include "panel-focaltech-ft7131m-init.h"


struct ft7131m {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct regulator_bulk_data supplies[4];
	struct gpio_desc *reset;
	enum drm_panel_orientation orientation;
};

static inline struct ft7131m *panel_to_ft7131m(struct drm_panel *panel)
{
	return container_of(panel, struct ft7131m, panel);
}

static void ft7131m_send_cmds(struct mipi_dsi_device *dsi,
			      const struct ft7131m_cmd *cmds, size_t count)
{
	size_t i;

	for (i = 0; i < count; i++) {
		const struct ft7131m_cmd *cmd = &cmds[i];
		int ret;

		if (!cmd->type) {
			if (cmd->len)
				msleep(cmd->len);
			continue;
		}

		if (cmd->len <= 1) {
			ret = mipi_dsi_dcs_write(dsi, cmd->data[0], NULL, 0);
		} else {
			ret = mipi_dsi_dcs_write(dsi, cmd->data[0],
						 cmd->data + 1, cmd->len - 1);
		}
		if (ret < 0)
			dev_err(&dsi->dev, "init cmd %zu failed: %d\n", i, ret);
	}
}

static int ft7131m_prepare(struct drm_panel *panel)
{
	struct ft7131m *ctx = panel_to_ft7131m(panel);
	struct mipi_dsi_device *dsi = ctx->dsi;
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (ret)
		return ret;

	if (ctx->reset) {
		gpiod_set_value_cansleep(ctx->reset, 1);
		msleep(10);
		gpiod_set_value_cansleep(ctx->reset, 0);
		msleep(10);
	}

	ft7131m_send_cmds(dsi, ft7131m_init_cmds, FT7131M_INIT_CMD_COUNT);

	mipi_dsi_dcs_exit_sleep_mode(dsi);
	msleep(120);

	return 0;
}

static int ft7131m_enable(struct drm_panel *panel)
{
	struct ft7131m *ctx = panel_to_ft7131m(panel);

	mipi_dsi_dcs_set_display_on(ctx->dsi);
	/* Stock post-display-on settle from the vendor on-command sequence. */
	msleep(300);

	return 0;
}

static int ft7131m_disable(struct drm_panel *panel)
{
	struct ft7131m *ctx = panel_to_ft7131m(panel);
	struct mipi_dsi_device *dsi = ctx->dsi;

	ft7131m_send_cmds(dsi, ft7131m_off_cmds, FT7131M_OFF_CMD_COUNT);

	mipi_dsi_dcs_set_display_off(dsi);
	msleep(16);

	return 0;
}

static int ft7131m_unprepare(struct drm_panel *panel)
{
	struct ft7131m *ctx = panel_to_ft7131m(panel);

	mipi_dsi_dcs_enter_sleep_mode(ctx->dsi);
	msleep(50);

	if (ctx->reset)
		gpiod_set_value_cansleep(ctx->reset, 1);

	regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);

	return 0;
}

static const struct drm_display_mode ft7131m_mode = {
	/* 1080x1920 @ ~61Hz from Panel_ft7131m_1080p_video.xml */
	.clock = 145279,
	.hdisplay = 1080,
	.hsync_start = 1080 + 52,
	.hsync_end = 1080 + 52 + 16,
	.htotal = 1080 + 52 + 50 + 16,
	.vdisplay = 1920,
	.vsync_start = 1920 + 36,
	.vsync_end = 1920 + 36 + 8,
	.vtotal = 1920 + 36 + 24 + 8,
	.width_mm = 68,
	.height_mm = 121,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static int ft7131m_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &ft7131m_mode);
}

static enum drm_panel_orientation ft7131m_get_orientation(struct drm_panel *panel)
{
	struct ft7131m *ctx = panel_to_ft7131m(panel);

	return ctx->orientation;
}

static const struct drm_panel_funcs ft7131m_funcs = {
	.prepare = ft7131m_prepare,
	.enable = ft7131m_enable,
	.disable = ft7131m_disable,
	.unprepare = ft7131m_unprepare,
	.get_modes = ft7131m_get_modes,
	.get_orientation = ft7131m_get_orientation,
};

static int ft7131m_probe(struct mipi_dsi_device *dsi)
{
	struct ft7131m *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(&dsi->dev, struct ft7131m, panel,
				   &ft7131m_funcs, DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	mipi_dsi_set_drvdata(dsi, ctx);
	ctx->dsi = dsi;

	ctx->supplies[0].supply = "vdd";
	ctx->supplies[1].supply = "vddio";
	ctx->supplies[2].supply = "avdd";
	ctx->supplies[3].supply = "avee";

	ret = devm_regulator_bulk_get(&dsi->dev, ARRAY_SIZE(ctx->supplies),
				      ctx->supplies);
	if (ret)
		return dev_err_probe(&dsi->dev, ret,
				     "failed to get supplies\n");

	ctx->reset = devm_gpiod_get_optional(&dsi->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset))
		return dev_err_probe(&dsi->dev, PTR_ERR(ctx->reset),
				     "failed to get reset GPIO\n");

	ret = drm_of_get_panel_orientation(dsi->dev.of_node, &ctx->orientation);
	if (ret) {
		dev_err(&dsi->dev, "%pOF: failed to get orientation: %d\n",
			dsi->dev.of_node, ret);
		return ret;
	}

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret && ret != -ENODEV)
		return ret;

	/*
	 * DSI host must be powered (LP-11) before init cmds; otherwise
	 * msm transfer returns -EINVAL (!power_on).
	 */
	ctx->panel.prepare_prev_first = true;

	ret = devm_drm_panel_add(&dsi->dev, &ctx->panel);
	if (ret)
		return ret;

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	/*
	 * Panel_ft7131m_1080p_video.xml: DSITrafficMode=1 → non-burst
	 * sync event. VIDEO_BURST (mode 2) caused white screen + horizontal
	 * noise after ABL handoff despite a stable DSI link (no FIFO errs).
	 * Continuous clock matches ABL splash.
	 */
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_LPM;

	return devm_mipi_dsi_attach(&dsi->dev, dsi);
}

static void ft7131m_remove(struct mipi_dsi_device *dsi)
{
}

static const struct of_device_id ft7131m_of_match[] = {
	{ .compatible = "focaltech,ft7131m" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ft7131m_of_match);

static struct mipi_dsi_driver ft7131m_driver = {
	.probe = ft7131m_probe,
	.remove = ft7131m_remove,
	.driver = {
		.name = "panel-focaltech-ft7131m",
		.of_match_table = ft7131m_of_match,
	},
};

module_mipi_dsi_driver(ft7131m_driver);

MODULE_AUTHOR("Anbernic RG 55G1 port");
MODULE_DESCRIPTION("FocalTech FT7131M 1080p DSI panel");
MODULE_LICENSE("GPL");
