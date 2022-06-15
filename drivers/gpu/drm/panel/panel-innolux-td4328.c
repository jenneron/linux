// SPDX-License-Identifier: GPL-2.0-only
/*
 * Innolux panels with a Synaptics TD4328 controller, as fitted to the
 * AYN Odin. Generated with linux-mdss-dsi-panel-driver-generator from the
 * vendor device tree.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

struct innolux_td4328 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct regulator *vddio;
	struct regulator *vddpos;
	struct regulator *vddneg;
	struct gpio_desc *enable_gpio;
	struct gpio_desc *reset_gpio;
	enum drm_panel_orientation orientation;
};

static inline struct innolux_td4328 *to_innolux_td4328(struct drm_panel *panel)
{
	return container_of_const(panel, struct innolux_td4328, panel);
}

static void innolux_td4328_reset(struct innolux_td4328 *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(80);
}

static int innolux_td4328_on(struct innolux_td4328 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb0, 0x00);
	if (of_device_is_compatible(ctx->dsi->dev.of_node, "ayn,odin-m0-panel"))
		mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc2,
						 0x01, 0xf7, 0x80, 0x08, 0x68, 0x08,
						 0x0c, 0x10, 0x00, 0x08, 0x70, 0x00,
						 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
						 0x03, 0x83, 0x00, 0x00, 0x00);
	else if (of_device_is_compatible(ctx->dsi->dev.of_node, "ayn,odin-m2-panel"))
		mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc2,
						 0x01, 0xf7, 0x80, 0x04, 0x68, 0x08,
						 0x09, 0x10, 0x00, 0x08, 0x30, 0x00,
						 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
						 0x02, 0x80, 0x00, 0x00, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xd6, 0x01);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb0, 0x03);
	mipi_dsi_dcs_set_column_address_multi(&dsi_ctx, 0x0000, 0x0437);
	mipi_dsi_dcs_set_page_address_multi(&dsi_ctx, 0x0000, 0x077f);
	mipi_dsi_dcs_set_tear_on_multi(&dsi_ctx, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 150);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 50);

	return dsi_ctx.accum_err;
}

static int innolux_td4328_off(struct innolux_td4328 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_usleep_range(&dsi_ctx, 5000, 6000);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 83);

	return dsi_ctx.accum_err;
}

static int innolux_td4328_prepare(struct drm_panel *panel)
{
	struct innolux_td4328 *ctx = to_innolux_td4328(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = regulator_enable(ctx->vddio);
	if (ret)
		return ret;

	msleep(20);

	ret = regulator_enable(ctx->vddpos);
	if (ret)
		goto disable_vddio;

	ret = regulator_enable(ctx->vddneg);
	if (ret)
		goto disable_vddpos;

	msleep(20);

	gpiod_set_value_cansleep(ctx->enable_gpio, 1);
	msleep(20);

	innolux_td4328_reset(ctx);

	ret = innolux_td4328_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		gpiod_set_value_cansleep(ctx->enable_gpio, 0);
		goto disable_vddneg;
	}

	return 0;

disable_vddneg:
	regulator_disable(ctx->vddneg);
disable_vddpos:
	regulator_disable(ctx->vddpos);
disable_vddio:
	regulator_disable(ctx->vddio);

	return ret;
}

static int innolux_td4328_unprepare(struct drm_panel *panel)
{
	struct innolux_td4328 *ctx = to_innolux_td4328(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = innolux_td4328_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	gpiod_set_value_cansleep(ctx->enable_gpio, 0);
	regulator_disable(ctx->vddneg);
	regulator_disable(ctx->vddpos);
	regulator_disable(ctx->vddio);

	return 0;
}

static const struct drm_display_mode innolux_td4328_mode = {
	.clock = (1080 + 60 + 10 + 60) * (1920 + 20 + 8 + 20) * 60 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 60,
	.hsync_end = 1080 + 60 + 10,
	.htotal = 1080 + 60 + 10 + 60,
	.vdisplay = 1920,
	.vsync_start = 1920 + 20,
	.vsync_end = 1920 + 20 + 8,
	.vtotal = 1920 + 20 + 8 + 20,
	.width_mm = 75,
	.height_mm = 132,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int innolux_td4328_get_modes(struct drm_panel *panel,
				      struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &innolux_td4328_mode);
}

static enum drm_panel_orientation
innolux_td4328_get_orientation(struct drm_panel *panel)
{
	struct innolux_td4328 *ctx = to_innolux_td4328(panel);

	return ctx->orientation;
}

static const struct drm_panel_funcs innolux_td4328_panel_funcs = {
	.prepare = innolux_td4328_prepare,
	.unprepare = innolux_td4328_unprepare,
	.get_modes = innolux_td4328_get_modes,
	.get_orientation = innolux_td4328_get_orientation,
};

static int innolux_td4328_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct innolux_td4328 *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct innolux_td4328, panel,
				   &innolux_td4328_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ret = drm_of_get_panel_orientation(dev->of_node, &ctx->orientation);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get orientation\n");

	ctx->vddio = devm_regulator_get(dev, "vddio");
	if (IS_ERR(ctx->vddio))
		return dev_err_probe(dev, PTR_ERR(ctx->vddio),
				     "Failed to get vddio regulator\n");

	ctx->vddpos = devm_regulator_get(dev, "vddpos");
	if (IS_ERR(ctx->vddpos))
		return dev_err_probe(dev, PTR_ERR(ctx->vddpos),
				     "Failed to get vddpos regulator\n");

	ctx->vddneg = devm_regulator_get(dev, "vddneg");
	if (IS_ERR(ctx->vddneg))
		return dev_err_probe(dev, PTR_ERR(ctx->vddneg),
				     "Failed to get vddneg regulator\n");

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->enable_gpio),
				     "Failed to get enable-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_NO_EOT_PACKET |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ctx->panel.prepare_prev_first = true;

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
	}

	return 0;
}

static void innolux_td4328_remove(struct mipi_dsi_device *dsi)
{
	struct innolux_td4328 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id innolux_td4328_of_match[] = {
	{ .compatible = "ayn,odin-m0-panel" },
	{ .compatible = "ayn,odin-m2-panel" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, innolux_td4328_of_match);

static struct mipi_dsi_driver innolux_td4328_driver = {
	.probe = innolux_td4328_probe,
	.remove = innolux_td4328_remove,
	.driver = {
		.name = "panel-innolux-td4328",
		.of_match_table = innolux_td4328_of_match,
	},
};
module_mipi_dsi_driver(innolux_td4328_driver);

MODULE_AUTHOR("Anton Bambura <jenneron@protonmail.com>");
MODULE_DESCRIPTION("DRM driver for the Innolux TD4328 command mode DSI panels of the AYN Odin");
MODULE_LICENSE("GPL");
