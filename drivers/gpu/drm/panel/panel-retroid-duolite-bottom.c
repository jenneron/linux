// SPDX-License-Identifier: GPL-2.0-only
/*
 * Bottom panel of the Retroid Pocket Duo Lite. The WL818S bridge firmware
 * drives its MIPI side, so this only sequences the panel supplies.
 */

#include <linux/delay.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_connector.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

/* Enabled one at a time, in this order, to keep the inrush down on battery */
static const struct regulator_bulk_data duolite_bottom_supplies[] = {
	{ .supply = "vpos" },
	{ .supply = "vneg" },
	{ .supply = "vdd" },
	{ .supply = "vddio" },
};

struct duolite_bottom {
	struct drm_panel panel;
	struct regulator_bulk_data *supplies;
};

static inline struct duolite_bottom *to_duolite_bottom(struct drm_panel *panel)
{
	return container_of(panel, struct duolite_bottom, panel);
}

static int duolite_bottom_prepare(struct drm_panel *panel)
{
	struct duolite_bottom *ctx = to_duolite_bottom(panel);
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(duolite_bottom_supplies); i++) {
		ret = regulator_enable(ctx->supplies[i].consumer);
		if (ret)
			goto err_disable;
		usleep_range(5000, 6000);
	}

	msleep(20);

	return 0;

err_disable:
	while (--i >= 0)
		regulator_disable(ctx->supplies[i].consumer);

	return ret;
}

/* The bias goes down before the logic supplies, as on the vendor kernel */
static int duolite_bottom_unprepare(struct drm_panel *panel)
{
	struct duolite_bottom *ctx = to_duolite_bottom(panel);
	int i;

	for (i = 0; i < ARRAY_SIZE(duolite_bottom_supplies); i++)
		regulator_disable(ctx->supplies[i].consumer);

	return 0;
}

/* From the bridge EDID ("GH7005"), which reports no physical size */
static const struct drm_display_mode duolite_bottom_mode = {
	.clock = 85090,
	.hdisplay = 1280,
	.hsync_start = 1280 + 90,
	.hsync_end = 1280 + 90 + 40,
	.htotal = 1280 + 90 + 40 + 20,
	.vdisplay = 960,
	.vsync_start = 960 + 0,
	.vsync_end = 960 + 0 + 8,
	.vtotal = 960 + 0 + 8 + 24,
	.width_mm = 85,
	.height_mm = 64,
	.flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
};

static int duolite_bottom_get_modes(struct drm_panel *panel,
				    struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector,
						    &duolite_bottom_mode);
}

static const struct drm_panel_funcs duolite_bottom_panel_funcs = {
	.prepare = duolite_bottom_prepare,
	.unprepare = duolite_bottom_unprepare,
	.get_modes = duolite_bottom_get_modes,
};

static int duolite_bottom_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct duolite_bottom *ctx;
	int ret;

	/* DisplayPort keeps the connector named DP-1 for userspace */
	ctx = devm_drm_panel_alloc(dev, struct duolite_bottom, panel,
				   &duolite_bottom_panel_funcs,
				   DRM_MODE_CONNECTOR_DisplayPort);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ret = devm_regulator_bulk_get_const(dev,
					    ARRAY_SIZE(duolite_bottom_supplies),
					    duolite_bottom_supplies,
					    &ctx->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	platform_set_drvdata(pdev, ctx);
	drm_panel_add(&ctx->panel);

	return 0;
}

static void duolite_bottom_remove(struct platform_device *pdev)
{
	struct duolite_bottom *ctx = platform_get_drvdata(pdev);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id duolite_bottom_of_match[] = {
	{ .compatible = "retroidpocket,duolite-bottom-panel" },
	{ }
};
MODULE_DEVICE_TABLE(of, duolite_bottom_of_match);

static struct platform_driver duolite_bottom_driver = {
	.probe = duolite_bottom_probe,
	.remove = duolite_bottom_remove,
	.driver = {
		.name = "panel-retroid-duolite-bottom",
		.of_match_table = duolite_bottom_of_match,
	},
};
module_platform_driver(duolite_bottom_driver);

MODULE_DESCRIPTION("Retroid Pocket Duo Lite bottom panel driver");
MODULE_LICENSE("GPL");
