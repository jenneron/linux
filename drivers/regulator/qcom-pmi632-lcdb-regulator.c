// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm PMI632 LCDB (LCD bias) regulator driver
 *
 * The LCDB block generates the positive (LDO) and negative (NCP) bias
 * rails for LCD panels from a shared boost. Register layout follows the
 * downstream qpnp-lcdb-regulator driver.
 */

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#define LCDB_BASE			0xec00

#define LCDB_INT_RT_STATUS		0x10
#define LCDB_VREG_OK_RT_STS		BIT(0)

#define LCDB_ENABLE_CTL1		0x46
#define LCDB_MODULE_EN			BIT(7)

#define LCDB_LDO_OUTPUT_VOLTAGE		0x71
#define LCDB_NCP_OUTPUT_VOLTAGE		0x81
#define LCDB_OUTPUT_VOLTAGE_MASK	GENMASK(4, 0)

struct qcom_lcdb {
	struct device *dev;
	struct regmap *regmap;
	struct mutex lock;	/* serialises the shared module enable */
	unsigned int users;
};

struct qcom_lcdb_vreg {
	struct qcom_lcdb *lcdb;
	struct regulator_desc desc;
	bool enabled;
};

static int qcom_lcdb_enable(struct regulator_dev *rdev)
{
	struct qcom_lcdb_vreg *vreg = rdev_get_drvdata(rdev);
	struct qcom_lcdb *lcdb = vreg->lcdb;
	unsigned int val;
	int ret = 0;

	mutex_lock(&lcdb->lock);

	if (vreg->enabled)
		goto out;

	if (!lcdb->users) {
		ret = regmap_write(lcdb->regmap, LCDB_BASE + LCDB_ENABLE_CTL1,
				   LCDB_MODULE_EN);
		if (ret)
			goto out;

		ret = regmap_read_poll_timeout(lcdb->regmap,
					       LCDB_BASE + LCDB_INT_RT_STATUS,
					       val, val & LCDB_VREG_OK_RT_STS,
					       1000, 100000);
		if (ret) {
			dev_err(lcdb->dev, "timeout waiting for vreg_ok\n");
			regmap_write(lcdb->regmap,
				     LCDB_BASE + LCDB_ENABLE_CTL1, 0);
			goto out;
		}
	}

	lcdb->users++;
	vreg->enabled = true;
out:
	mutex_unlock(&lcdb->lock);
	return ret;
}

static int qcom_lcdb_disable(struct regulator_dev *rdev)
{
	struct qcom_lcdb_vreg *vreg = rdev_get_drvdata(rdev);
	struct qcom_lcdb *lcdb = vreg->lcdb;
	int ret = 0;

	mutex_lock(&lcdb->lock);

	if (!vreg->enabled)
		goto out;

	if (lcdb->users == 1) {
		ret = regmap_write(lcdb->regmap, LCDB_BASE + LCDB_ENABLE_CTL1,
				   0);
		if (ret)
			goto out;
	}

	lcdb->users--;
	vreg->enabled = false;
out:
	mutex_unlock(&lcdb->lock);
	return ret;
}

static int qcom_lcdb_is_enabled(struct regulator_dev *rdev)
{
	struct qcom_lcdb_vreg *vreg = rdev_get_drvdata(rdev);

	return vreg->enabled;
}

static const struct regulator_ops qcom_lcdb_ops = {
	.enable = qcom_lcdb_enable,
	.disable = qcom_lcdb_disable,
	.is_enabled = qcom_lcdb_is_enabled,
	.list_voltage = regulator_list_voltage_linear_range,
	.map_voltage = regulator_map_voltage_linear_range,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
};

static const struct linear_range qcom_lcdb_ranges[] = {
	REGULATOR_LINEAR_RANGE(4000000, 0, 9, 100000),
	REGULATOR_LINEAR_RANGE(4950000, 10, 31, 50000),
};

#define QCOM_LCDB_DESC(_name, _vsel_reg)				\
{									\
	.name = _name,							\
	.of_match = _name,						\
	.type = REGULATOR_VOLTAGE,					\
	.owner = THIS_MODULE,						\
	.ops = &qcom_lcdb_ops,						\
	.linear_ranges = qcom_lcdb_ranges,				\
	.n_linear_ranges = ARRAY_SIZE(qcom_lcdb_ranges),		\
	.n_voltages = 32,						\
	.vsel_reg = LCDB_BASE + (_vsel_reg),				\
	.vsel_mask = LCDB_OUTPUT_VOLTAGE_MASK,				\
}

static const struct regulator_desc qcom_lcdb_descs[] = {
	QCOM_LCDB_DESC("ldo", LCDB_LDO_OUTPUT_VOLTAGE),
	QCOM_LCDB_DESC("ncp", LCDB_NCP_OUTPUT_VOLTAGE),
};

static int qcom_lcdb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regulator_config cfg = { .dev = dev };
	struct qcom_lcdb *lcdb;
	unsigned int val;
	bool enabled;
	int i, ret;

	lcdb = devm_kzalloc(dev, sizeof(*lcdb), GFP_KERNEL);
	if (!lcdb)
		return -ENOMEM;

	lcdb->dev = dev;
	lcdb->regmap = dev_get_regmap(dev->parent, NULL);
	if (!lcdb->regmap)
		return dev_err_probe(dev, -ENODEV, "failed to get regmap\n");

	mutex_init(&lcdb->lock);

	ret = regmap_read(lcdb->regmap, LCDB_BASE + LCDB_ENABLE_CTL1, &val);
	if (ret)
		return ret;
	enabled = val & LCDB_MODULE_EN;

	cfg.regmap = lcdb->regmap;

	for (i = 0; i < ARRAY_SIZE(qcom_lcdb_descs); i++) {
		struct qcom_lcdb_vreg *vreg;
		struct regulator_dev *rdev;

		vreg = devm_kzalloc(dev, sizeof(*vreg), GFP_KERNEL);
		if (!vreg)
			return -ENOMEM;

		vreg->lcdb = lcdb;
		vreg->desc = qcom_lcdb_descs[i];
		vreg->enabled = enabled;
		if (enabled)
			lcdb->users++;

		cfg.driver_data = vreg;

		rdev = devm_regulator_register(dev, &vreg->desc, &cfg);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev),
					     "failed to register %s\n",
					     vreg->desc.name);
	}

	return 0;
}

static const struct of_device_id qcom_lcdb_match[] = {
	{ .compatible = "qcom,pmi632-lcdb" },
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_lcdb_match);

static struct platform_driver qcom_lcdb_driver = {
	.probe = qcom_lcdb_probe,
	.driver = {
		.name = "qcom-pmi632-lcdb-regulator",
		.of_match_table = qcom_lcdb_match,
	},
};
module_platform_driver(qcom_lcdb_driver);

MODULE_DESCRIPTION("Qualcomm PMI632 LCDB regulator driver");
MODULE_LICENSE("GPL");
