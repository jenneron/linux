// SPDX-License-Identifier: GPL-2.0-only
/*
 * Hynitron CST66xx touchscreen driver
 *
 * This is a deliberately small runtime-only driver.  It implements the touch
 * report protocol used by the AYANEO Pocket S Mini, but does not contain or
 * write controller firmware.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>

#define CST66XX_RUNTIME_ADDR	0x58
#define CST66XX_MAX_TOUCHES	5
#define CST66XX_MAX_RECORDS	15
#define CST66XX_RECORD_SIZE	5
#define CST66XX_HEADER_SIZE	9

struct cst66xx_ts {
	struct i2c_client *client;
	struct input_dev *input;
	struct gpio_desc *reset_gpio;
	struct regulator *vdd;
	struct touchscreen_properties prop;
};

static int cst66xx_transfer(struct cst66xx_ts *ts, const u8 *command,
			    size_t command_len, u8 *data, size_t data_len)
{
	struct i2c_msg messages[2];
	int count = 0;
	int ret;

	if (command_len) {
		messages[count].addr = CST66XX_RUNTIME_ADDR;
		messages[count].flags = 0;
		messages[count].len = command_len;
		messages[count].buf = (u8 *)command;
		count++;
	}

	if (data_len) {
		messages[count].addr = CST66XX_RUNTIME_ADDR;
		messages[count].flags = I2C_M_RD;
		messages[count].len = data_len;
		messages[count].buf = data;
		count++;
	}

	ret = i2c_transfer(ts->client->adapter, messages, count);
	if (ret == count)
		return 0;

	return ret < 0 ? ret : -EIO;
}

static int cst66xx_write(struct cst66xx_ts *ts, const u8 *data, size_t len)
{
	return cst66xx_transfer(ts, data, len, NULL, 0);
}

static void cst66xx_hw_reset(struct cst66xx_ts *ts)
{
	/* reset-gpios is active-low: logical 1 asserts the physical line. */
	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	usleep_range(2000, 3000);
	gpiod_set_value_cansleep(ts->reset_gpio, 0);
	msleep(50);
}

static u16 cst66xx_checksum(const u8 *data, size_t len)
{
	u16 checksum = 0x55;
	size_t i;

	for (i = 0; i < len; i++)
		checksum += data[i];

	return checksum;
}

static int cst66xx_read_report(struct cst66xx_ts *ts, u8 *buffer,
			       unsigned int *finger_count,
			       unsigned int *key_count)
{
	static const u8 read_command[] = { 0xd0, 0x07, 0x00, 0x00 };
	static const u8 finish_command[] = { 0xd0, 0x00, 0x02, 0xab };
	unsigned int record_count;
	u16 expected_checksum;
	int ret;
	int finish_ret;

	ret = cst66xx_transfer(ts, read_command, sizeof(read_command), buffer,
			       CST66XX_HEADER_SIZE);
	if (ret)
		return ret;

	*finger_count = buffer[3] & 0x0f;
	*key_count = buffer[3] >> 4;
	record_count = *finger_count + *key_count;

	if (*finger_count > CST66XX_MAX_TOUCHES ||
	    record_count > CST66XX_MAX_RECORDS) {
		ret = -EPROTO;
		goto finish;
	}

	if (record_count > 1) {
		ret = cst66xx_transfer(ts, NULL, 0, &buffer[CST66XX_HEADER_SIZE],
				       (record_count - 1) * CST66XX_RECORD_SIZE);
		if (ret)
			goto finish;
	}

	expected_checksum = get_unaligned_le16(buffer);
	if (cst66xx_checksum(&buffer[4], record_count * CST66XX_RECORD_SIZE) !=
	    expected_checksum)
		ret = -EBADMSG;

finish:
	/* Always release the report latch after a successfully read header. */
	finish_ret = cst66xx_write(ts, finish_command, sizeof(finish_command));
	return ret ? ret : finish_ret;
}

static irqreturn_t cst66xx_irq_thread(int irq, void *data)
{
	struct cst66xx_ts *ts = data;
	u8 buffer[4 + CST66XX_MAX_RECORDS * CST66XX_RECORD_SIZE] = { 0 };
	unsigned int finger_count;
	unsigned int key_count;
	unsigned int i;
	int ret;

	ret = cst66xx_read_report(ts, buffer, &finger_count, &key_count);
	if (ret) {
		dev_dbg_ratelimited(&ts->client->dev,
				    "failed to read touch report: %d\n", ret);
		return IRQ_HANDLED;
	}

	/* 0xff is the CST66xx coordinate-report type. */
	if (buffer[2] != 0xff)
		return IRQ_HANDLED;

	for (i = 0; i < finger_count; i++) {
		unsigned int offset = (key_count + i) * CST66XX_RECORD_SIZE;
		u16 x = buffer[offset + 4] |
			((buffer[offset + 7] & 0x0f) << 8);
		u16 y = buffer[offset + 5] |
			((buffer[offset + 7] & 0xf0) << 4);
		u8 pressure = buffer[offset + 6];
		u8 id = buffer[offset + 8] & 0x0f;
		bool active = buffer[offset + 8] >> 4;

		if (id >= CST66XX_MAX_TOUCHES)
			continue;

		input_mt_slot(ts->input, id);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, active);
		if (active) {
			touchscreen_report_pos(ts->input, &ts->prop, x, y, true);
			input_report_abs(ts->input, ABS_MT_PRESSURE, pressure);
			input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR,
					 max_t(u8, pressure >> 3, 1));
		}
	}

	input_mt_sync_frame(ts->input);
	input_sync(ts->input);

	return IRQ_HANDLED;
}

static void cst66xx_disable_regulator(void *data)
{
	regulator_disable(data);
}

static int cst66xx_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct cst66xx_ts *ts;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EOPNOTSUPP;
	if (client->irq <= 0)
		return dev_err_probe(dev, -EINVAL, "missing touchscreen IRQ\n");

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;
	i2c_set_clientdata(client, ts);

	ts->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(ts->vdd))
		return dev_err_probe(dev, PTR_ERR(ts->vdd),
				     "failed to get vdd regulator\n");

	ret = regulator_enable(ts->vdd);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable vdd regulator\n");

	ret = devm_add_action_or_reset(dev, cst66xx_disable_regulator, ts->vdd);
	if (ret)
		return ret;

	ts->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ts->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ts->reset_gpio),
				     "failed to get reset GPIO\n");

	usleep_range(5000, 6000);
	cst66xx_hw_reset(ts);

	ts->input = devm_input_allocate_device(dev);
	if (!ts->input)
		return -ENOMEM;

	ts->input->name = "Hynitron CST66xx Touchscreen";
	ts->input->id.bustype = BUS_I2C;
	ts->input->dev.parent = dev;
	input_set_drvdata(ts->input, ts);

	input_set_capability(ts->input, EV_KEY, BTN_TOUCH);
	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0, 959, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0, 1279, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR, 0, 31, 0, 0);
	touchscreen_parse_properties(ts->input, true, &ts->prop);

	ret = input_mt_init_slots(ts->input, CST66XX_MAX_TOUCHES,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return ret;

	ret = input_register_device(ts->input);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register input device\n");

	ret = devm_request_threaded_irq(dev, client->irq, NULL,
					cst66xx_irq_thread,
					IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
					client->name, ts);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	return 0;
}

static int cst66xx_suspend(struct device *dev)
{
	static const u8 deep_sleep[] = { 0xd0, 0x00, 0x22, 0xab };
	struct cst66xx_ts *ts = i2c_get_clientdata(to_i2c_client(dev));

	disable_irq(ts->client->irq);
	cst66xx_write(ts, deep_sleep, sizeof(deep_sleep));
	gpiod_set_value_cansleep(ts->reset_gpio, 1);

	return 0;
}

static int cst66xx_resume(struct device *dev)
{
	static const u8 disable_low_power[] = { 0xd0, 0x00, 0x04, 0x00 };
	static const u8 normal_mode[] = { 0xd0, 0x00, 0x00, 0x00 };
	static const u8 gesture_off[] = { 0xd0, 0x00, 0x0c, 0x00 };
	static const u8 scan_on[] = { 0xd0, 0x00, 0x01, 0x00 };
	struct cst66xx_ts *ts = i2c_get_clientdata(to_i2c_client(dev));
	int ret;

	cst66xx_hw_reset(ts);
	ret = cst66xx_write(ts, disable_low_power, sizeof(disable_low_power));
	if (!ret) {
		usleep_range(1000, 2000);
		ret = cst66xx_write(ts, disable_low_power,
				    sizeof(disable_low_power));
	}
	if (!ret)
		ret = cst66xx_write(ts, normal_mode, sizeof(normal_mode));
	if (!ret)
		ret = cst66xx_write(ts, gesture_off, sizeof(gesture_off));
	if (!ret)
		ret = cst66xx_write(ts, scan_on, sizeof(scan_on));

	enable_irq(ts->client->irq);
	return ret;
}

static DEFINE_SIMPLE_DEV_PM_OPS(cst66xx_pm_ops,
				cst66xx_suspend, cst66xx_resume);

static const struct of_device_id cst66xx_of_match[] = {
	{ .compatible = "hynitron,cst66xx" },
	{ .compatible = "hyn,66xx" },
	{ }
};
MODULE_DEVICE_TABLE(of, cst66xx_of_match);

static const struct i2c_device_id cst66xx_i2c_ids[] = {
	{ "cst66xx" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cst66xx_i2c_ids);

static struct i2c_driver cst66xx_driver = {
	.driver = {
		.name = "hynitron-cst66xx",
		.of_match_table = cst66xx_of_match,
		.pm = pm_sleep_ptr(&cst66xx_pm_ops),
	},
	.probe = cst66xx_probe,
	.id_table = cst66xx_i2c_ids,
};
module_i2c_driver(cst66xx_driver);

MODULE_AUTHOR("ROCKNIX contributors");
MODULE_DESCRIPTION("Hynitron CST66xx touchscreen driver");
MODULE_LICENSE("GPL");
