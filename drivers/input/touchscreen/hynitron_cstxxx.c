// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Driver for Hynitron cstxxx Touchscreen
 *
 *  Copyright (c) 2022 Chris Morgan <macromorgan@hotmail.com>
 *
 *  This code is based on hynitron_core.c authored by Hynitron.
 *  Note that no datasheet was available, so much of these registers
 *  are undocumented. This is essentially a cleaned-up version of the
 *  vendor driver with support removed for hardware I cannot test and
 *  device-specific functions replated with generic functions wherever
 *  possible.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>

#include <drm/drm_panel.h>

/* Per chip data */
struct hynitron_ts_chip_data {
	unsigned int max_touch_num;
	u32 ic_chkcode;
	int (*firmware_info)(struct i2c_client *client);
	int (*bootloader_enter)(struct i2c_client *client);
	int (*init_input)(struct i2c_client *client);
	void (*report_touch)(struct i2c_client *client);
	int (*suspend)(struct i2c_client *client);
	int (*resume)(struct i2c_client *client);
};

/* Data generic to all (supported and non-supported) controllers. */
struct hynitron_ts_data {
	const struct hynitron_ts_chip_data *chip;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct touchscreen_properties prop;
	struct gpio_desc *reset_gpio;
	struct regulator *vdd;
	struct drm_panel_follower panel_follower;
	struct mutex pm_lock;
	bool panel_off;
	bool suspended;
	bool powered;
};

/*
 * Since I have no datasheet, these values are guessed and/or assumed
 * based on observation and testing.
 */
#define CST3XX_FIRMWARE_INFO_START_CMD		0x01d1
#define CST3XX_FIRMWARE_INFO_END_CMD		0x09d1
#define CST3XX_FIRMWARE_CHK_CODE_REG		0xfcd1
#define CST3XX_FIRMWARE_VERSION_REG		0x08d2
#define CST3XX_FIRMWARE_VER_INVALID_VAL		0xa5a5a5a5

#define CST3XX_BOOTLDR_PROG_CMD			0xaa01a0
#define CST3XX_BOOTLDR_PROG_CHK_REG		0x02a0
#define CST3XX_BOOTLDR_CHK_VAL			0xac

#define CST3XX_TOUCH_DATA_PART_REG		0x00d0
#define CST3XX_TOUCH_DATA_FULL_REG		0x07d0
#define CST3XX_TOUCH_DATA_CHK_VAL		0xab
#define CST3XX_TOUCH_DATA_TOUCH_VAL		0x03
#define CST3XX_TOUCH_DATA_STOP_CMD		0xab00d0
#define CST3XX_TOUCH_COUNT_MASK			GENMASK(6, 0)

#define CST66XX_CMD_PREAMBLE			0xd0000400
#define CST66XX_CMD_NORMAL			0xd0000000
#define CST66XX_CMD_GESTURE_OFF			0xd0000c00
#define CST66XX_CMD_REPORT_ON			0xd0000100
#define CST66XX_CMD_DEEP_SLEEP			0xd00022ab
#define CST66XX_CMD_INFO			0xd0030000
#define CST66XX_CMD_TOUCH			0xd0070000
#define CST66XX_CMD_TOUCH_ACK			0xd00002ab
#define CST66XX_RESET_US			8000
#define CST66XX_BOOT_MS				50
#define CST66XX_INFO_LEN			50
#define CST66XX_TOUCH_HDR_LEN			4
#define CST66XX_TOUCH_REC_LEN			5
#define CST66XX_TOUCH_TYPE_POS			0xff
#define CST66XX_CHECKSUM_SEED			0x55
#define CST66XX_MAX_POINTS			5


/*
 * Hard coded reset delay value of 20ms not IC dependent in
 * vendor driver.
 */
static void hyn_reset_proc(struct i2c_client *client, int delay)
{
	struct hynitron_ts_data *ts_data = i2c_get_clientdata(client);

	gpiod_set_value_cansleep(ts_data->reset_gpio, 1);
	msleep(20);
	gpiod_set_value_cansleep(ts_data->reset_gpio, 0);
	if (delay)
		fsleep(delay * 1000);
}

static irqreturn_t hyn_interrupt_handler(int irq, void *dev_id)
{
	struct i2c_client *client = dev_id;
	struct hynitron_ts_data *ts_data = i2c_get_clientdata(client);

	ts_data->chip->report_touch(client);

	return IRQ_HANDLED;
}

/*
 * The vendor driver would retry twice before failing to read or write
 * to the i2c device.
 */

static int cst3xx_i2c_write(struct i2c_client *client,
			    unsigned char *buf, int len)
{
	int ret;
	int retries = 0;

	while (retries < 2) {
		ret = i2c_master_send(client, buf, len);
		if (ret == len)
			return 0;
		if (ret <= 0)
			retries++;
		else
			break;
	}

	return ret < 0 ? ret : -EIO;
}

static int cst3xx_i2c_read_register(struct i2c_client *client, u16 reg,
				    u8 *val, u16 len)
{
	__le16 buf = cpu_to_le16(reg);
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = (u8 *)&buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = val,
		}
	};
	int err;
	int ret;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret == ARRAY_SIZE(msgs))
		return 0;

	err = ret < 0 ? ret : -EIO;
	dev_err(&client->dev, "Error reading %d bytes from 0x%04x: %d (%d)\n",
		len, reg, err, ret);

	return err;
}

static int cst3xx_firmware_info(struct i2c_client *client)
{
	struct hynitron_ts_data *ts_data = i2c_get_clientdata(client);
	int err;
	u32 tmp;
	unsigned char buf[4];

	/*
	 * Tests suggest this command needed to read firmware regs.
	 */
	put_unaligned_le16(CST3XX_FIRMWARE_INFO_START_CMD, buf);
	err = cst3xx_i2c_write(client, buf, 2);
	if (err)
		return err;

	usleep_range(10000, 11000);

	/*
	 * Read register for check-code to determine if device detected
	 * correctly.
	 */
	err = cst3xx_i2c_read_register(client, CST3XX_FIRMWARE_CHK_CODE_REG,
				       buf, 4);
	if (err)
		return err;

	tmp = get_unaligned_le32(buf);
	if ((tmp & 0xffff0000) != ts_data->chip->ic_chkcode) {
		dev_err(&client->dev, "%s ic mismatch, chkcode is %u\n",
			__func__, tmp);
		return -ENODEV;
	}

	usleep_range(10000, 11000);

	/* Read firmware version and test if firmware missing. */
	err = cst3xx_i2c_read_register(client, CST3XX_FIRMWARE_VERSION_REG,
				       buf, 4);
	if (err)
		return err;

	tmp = get_unaligned_le32(buf);
	if (tmp == CST3XX_FIRMWARE_VER_INVALID_VAL) {
		dev_err(&client->dev, "Device firmware missing\n");
		return -ENODEV;
	}

	/*
	 * Tests suggest cmd required to exit reading firmware regs.
	 */
	put_unaligned_le16(CST3XX_FIRMWARE_INFO_END_CMD, buf);
	err = cst3xx_i2c_write(client, buf, 2);
	if (err)
		return err;

	usleep_range(5000, 6000);

	return 0;
}

static int cst3xx_bootloader_enter(struct i2c_client *client)
{
	int err;
	u8 retry;
	u32 tmp = 0;
	unsigned char buf[3];

	for (retry = 0; retry < 5; retry++) {
		hyn_reset_proc(client, (7 + retry));
		/* set cmd to enter program mode */
		put_unaligned_le24(CST3XX_BOOTLDR_PROG_CMD, buf);
		err = cst3xx_i2c_write(client, buf, 3);
		if (err)
			continue;

		usleep_range(2000, 2500);

		/* check whether in program mode */
		err = cst3xx_i2c_read_register(client,
					       CST3XX_BOOTLDR_PROG_CHK_REG,
					       buf, 1);
		if (err)
			continue;

		tmp = get_unaligned(buf);
		if (tmp == CST3XX_BOOTLDR_CHK_VAL)
			break;
	}

	if (tmp != CST3XX_BOOTLDR_CHK_VAL) {
		dev_err(&client->dev, "%s unable to enter bootloader mode\n",
			__func__);
		return -ENODEV;
	}

	hyn_reset_proc(client, 40);

	return 0;
}

static int cst66xx_cmd(struct i2c_client *client, u32 cmd)
{
	u8 buf[4];

	put_unaligned_be32(cmd, buf);

	return cst3xx_i2c_write(client, buf, sizeof(buf));
}

static int cst66xx_read(struct i2c_client *client, u32 cmd, u8 *val, u16 len)
{
	int ret;

	ret = cst66xx_cmd(client, cmd);
	if (ret)
		return ret;

	ret = i2c_master_recv(client, val, len);
	if (ret == len)
		return 0;

	return ret < 0 ? ret : -EIO;
}

static void cst66xx_preamble(struct i2c_client *client)
{
	cst66xx_cmd(client, CST66XX_CMD_PREAMBLE);
	usleep_range(1000, 1100);
	cst66xx_cmd(client, CST66XX_CMD_PREAMBLE);
}

static int cst66xx_set_normal(struct i2c_client *client)
{
	int err;

	cst66xx_preamble(client);

	err = cst66xx_cmd(client, CST66XX_CMD_NORMAL);
	if (!err)
		err = cst66xx_cmd(client, CST66XX_CMD_GESTURE_OFF);
	if (!err)
		err = cst66xx_cmd(client, CST66XX_CMD_REPORT_ON);

	return err;
}

static int cst66xx_firmware_info(struct i2c_client *client)
{
	struct hynitron_ts_data *ts_data = i2c_get_clientdata(client);
	u8 buf[CST66XX_INFO_LEN];
	int retry, err = -ENODEV;
	u32 chip;

	for (retry = 0; retry < 5; retry++) {
		err = cst66xx_set_normal(client);
		if (!err)
			err = cst66xx_read(client, CST66XX_CMD_INFO, buf, sizeof(buf));
		if (!err && buf[2] == 0xca && buf[3] == 0xca)
			break;

		err = -ENODEV;
		usleep_range(1000, 2000);
		cst66xx_cmd(client, CST66XX_CMD_PREAMBLE);
	}
	if (err) {
		dev_err(&client->dev, "failed to read cst66xx info\n");
		return err;
	}

	chip = get_unaligned_le32(&buf[0]);
	if ((chip & 0xffff0000) != ts_data->chip->ic_chkcode) {
		dev_err(&client->dev, "ic mismatch, chip type is 0x%08x\n", chip);
		return -ENODEV;
	}

	dev_info(&client->dev, "chip 0x%08x fw 0x%08x project 0x%08x %ux%u\n",
		 chip, get_unaligned_le32(&buf[32]), get_unaligned_le32(&buf[36]),
		 get_unaligned_le16(&buf[28]), get_unaligned_le16(&buf[30]));

	return cst66xx_set_normal(client);
}

static int cst66xx_suspend(struct i2c_client *client)
{
	cst66xx_preamble(client);
	usleep_range(1000, 1100);

	return cst66xx_cmd(client, CST66XX_CMD_DEEP_SLEEP);
}

static int cst66xx_resume(struct i2c_client *client)
{
	struct hynitron_ts_data *ts_data = i2c_get_clientdata(client);

	gpiod_set_value_cansleep(ts_data->reset_gpio, 1);
	fsleep(CST66XX_RESET_US);
	gpiod_set_value_cansleep(ts_data->reset_gpio, 0);
	msleep(CST66XX_BOOT_MS);

	return cst66xx_set_normal(client);
}

static void cst3xx_report_contact(struct hynitron_ts_data *ts_data,
				  u8 id, unsigned int x, unsigned int y, u8 w)
{
	input_mt_slot(ts_data->input_dev, id);
	input_mt_report_slot_state(ts_data->input_dev, MT_TOOL_FINGER, 1);
	touchscreen_report_pos(ts_data->input_dev, &ts_data->prop, x, y, true);
	input_report_abs(ts_data->input_dev, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(ts_data->input_dev, ABS_MT_WIDTH_MAJOR, w);
}

static int cst3xx_finish_touch_read(struct i2c_client *client)
{
	unsigned char buf[3];
	int err;

	put_unaligned_le24(CST3XX_TOUCH_DATA_STOP_CMD, buf);
	err = cst3xx_i2c_write(client, buf, 3);
	if (err) {
		dev_err(&client->dev,
			"send read touch info ending failed: %d\n", err);
		return err;
	}

	return 0;
}

/*
 * Handle events from IRQ. Note that for cst3xx it appears that IRQ
 * fires continuously while touched, otherwise once every 1500ms
 * when not touched (assume touchscreen waking up periodically).
 * Note buffer is sized for 5 fingers, if more needed buffer must
 * be increased. The buffer contains 5 bytes for each touch point,
 * a touch count byte, a check byte, and then a second check byte after
 * all other touch points.
 *
 * For example 1 touch would look like this:
 * touch1[5]:touch_count[1]:chk_byte[1]
 *
 * 3 touches would look like this:
 * touch1[5]:touch_count[1]:chk_byte[1]:touch2[5]:touch3[5]:chk_byte[1]
 */
static void cst3xx_touch_report(struct i2c_client *client)
{
	struct hynitron_ts_data *ts_data = i2c_get_clientdata(client);
	u8 buf[28];
	u8 finger_id, sw, w;
	unsigned int x, y;
	unsigned int touch_cnt, end_byte;
	unsigned int idx = 0;
	unsigned int i;
	int err;

	/* Read and validate the first bits of input data. */
	err = cst3xx_i2c_read_register(client, CST3XX_TOUCH_DATA_PART_REG,
				       buf, 28);
	if (err ||
	    buf[6] != CST3XX_TOUCH_DATA_CHK_VAL ||
	    buf[0] == CST3XX_TOUCH_DATA_CHK_VAL) {
		dev_err(&client->dev, "cst3xx touch read failure\n");
		return;
	}

	/* Report to the device we're done reading the touch data. */
	err = cst3xx_finish_touch_read(client);
	if (err)
		return;

	touch_cnt = buf[5] & CST3XX_TOUCH_COUNT_MASK;
	if (touch_cnt > ts_data->chip->max_touch_num) {
		dev_err(&client->dev, "cst3xx invalid touch count (%d vs %d max)\n",
			touch_cnt, ts_data->chip->max_touch_num);
		return;
	}

	/*
	 * Check the check bit of the last touch slot. The check bit is
	 * always present after touch point 1 for valid data, and then
	 * appears as the last byte after all other touch data.
	 */
	if (touch_cnt > 1) {
		end_byte = touch_cnt * 5 + 2;
		if (buf[end_byte] != CST3XX_TOUCH_DATA_CHK_VAL) {
			dev_err(&client->dev, "cst3xx touch read failure\n");
			return;
		}
	}

	/* Parse through the buffer to capture touch data. */
	for (i = 0; i < touch_cnt; i++) {
		x = ((buf[idx + 1] << 4) | ((buf[idx + 3] >> 4) & 0x0f));
		y = ((buf[idx + 2] << 4) | (buf[idx + 3] & 0x0f));
		w = (buf[idx + 4] >> 3);
		sw = (buf[idx] & 0x0f) >> 1;
		finger_id = (buf[idx] >> 4) & 0x0f;

		/* Sanity check we don't have more fingers than we expect */
		if (finger_id >= ts_data->chip->max_touch_num) {
			dev_err(&client->dev,
				"cst3xx invalid finger id %d\n", finger_id);
			return;
		}

		/* sw value of 0 means no touch, 0x03 means touch */
		if (sw == CST3XX_TOUCH_DATA_TOUCH_VAL)
			cst3xx_report_contact(ts_data, finger_id, x, y, w);

		idx += 5;

		/* Skip the 2 bytes between point 1 and point 2 */
		if (i == 0)
			idx += 2;
	}

	input_mt_sync_frame(ts_data->input_dev);
	input_sync(ts_data->input_dev);
}

static void cst66xx_touch_report(struct i2c_client *client)
{
	struct hynitron_ts_data *ts_data = i2c_get_clientdata(client);
	u8 buf[CST66XX_TOUCH_HDR_LEN + CST66XX_MAX_POINTS * CST66XX_TOUCH_REC_LEN];
	unsigned int fingers, keys, records, i;
	unsigned int x, y;
	int ret, len;
	u16 sum;
	u8 *rec;
	int err;

	err = cst66xx_read(client, CST66XX_CMD_TOUCH, buf,
			   CST66XX_TOUCH_HDR_LEN + CST66XX_TOUCH_REC_LEN);
	if (err)
		goto ack;

	fingers = buf[3] & 0x0f;
	keys = buf[3] >> 4;
	records = fingers + keys;
	if (records > CST66XX_MAX_POINTS) {
		err = -EINVAL;
		goto ack;
	}

	if (records > 1) {
		len = (records - 1) * CST66XX_TOUCH_REC_LEN;
		ret = i2c_master_recv(client,
				      &buf[CST66XX_TOUCH_HDR_LEN + CST66XX_TOUCH_REC_LEN],
				      len);
		if (ret != len) {
			err = ret < 0 ? ret : -EIO;
			goto ack;
		}
	}

	sum = CST66XX_CHECKSUM_SEED;
	for (i = 0; i < records * CST66XX_TOUCH_REC_LEN; i++)
		sum += buf[CST66XX_TOUCH_HDR_LEN + i];
	if (sum != get_unaligned_le16(buf))
		err = -EBADMSG;

ack:
	cst66xx_cmd(client, CST66XX_CMD_TOUCH_ACK);

	if (err) {
		dev_dbg(&client->dev, "cst66xx touch read failure: %d\n", err);
		return;
	}

	if (buf[2] != CST66XX_TOUCH_TYPE_POS)
		return;

	for (i = 0; i < fingers; i++) {
		rec = &buf[CST66XX_TOUCH_HDR_LEN + (keys + i) * CST66XX_TOUCH_REC_LEN];

		if ((rec[4] & 0x0f) >= ts_data->chip->max_touch_num || !(rec[4] >> 4))
			continue;

		x = rec[0] | (rec[3] & 0x0f) << 8;
		y = rec[1] | (rec[3] & 0xf0) << 4;
		cst3xx_report_contact(ts_data, rec[4] & 0x0f, x, y, rec[2]);
	}

	input_mt_sync_frame(ts_data->input_dev);
	input_sync(ts_data->input_dev);
}

static int cst3xx_input_dev_int(struct i2c_client *client)
{
	struct hynitron_ts_data *ts_data = i2c_get_clientdata(client);
	int err;

	ts_data->input_dev = devm_input_allocate_device(&client->dev);
	if (!ts_data->input_dev) {
		dev_err(&client->dev, "Failed to allocate input device\n");
		return -ENOMEM;
	}

	ts_data->input_dev->name = "Hynitron cst3xx Touchscreen";
	ts_data->input_dev->phys = "input/ts";
	ts_data->input_dev->id.bustype = BUS_I2C;

	input_set_drvdata(ts_data->input_dev, ts_data);

	input_set_capability(ts_data->input_dev, EV_ABS, ABS_MT_POSITION_X);
	input_set_capability(ts_data->input_dev, EV_ABS, ABS_MT_POSITION_Y);
	input_set_abs_params(ts_data->input_dev, ABS_MT_TOUCH_MAJOR,
			     0, 255, 0, 0);

	touchscreen_parse_properties(ts_data->input_dev, true, &ts_data->prop);

	if (!ts_data->prop.max_x || !ts_data->prop.max_y) {
		dev_err(&client->dev,
			"Invalid x/y (%d, %d), using defaults\n",
			ts_data->prop.max_x, ts_data->prop.max_y);
		ts_data->prop.max_x = 1152;
		ts_data->prop.max_y = 1920;
		input_abs_set_max(ts_data->input_dev,
				  ABS_MT_POSITION_X, ts_data->prop.max_x);
		input_abs_set_max(ts_data->input_dev,
				  ABS_MT_POSITION_Y, ts_data->prop.max_y);
	}

	err = input_mt_init_slots(ts_data->input_dev,
				  ts_data->chip->max_touch_num,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (err) {
		dev_err(&client->dev,
			"Failed to initialize input slots: %d\n", err);
		return err;
	}

	err = input_register_device(ts_data->input_dev);
	if (err) {
		dev_err(&client->dev,
			"Input device registration failed: %d\n", err);
		return err;
	}

	return 0;
}

static int hyn_power_on(struct hynitron_ts_data *ts_data)
{
	struct i2c_client *client = ts_data->client;
	int err;

	gpiod_set_value_cansleep(ts_data->reset_gpio, 1);
	err = regulator_enable(ts_data->vdd);
	if (err)
		return err;

	err = ts_data->chip->resume(client);
	if (err)
		dev_warn(&client->dev, "chip resume failed: %d\n", err);

	return 0;
}

static void hyn_update_power(struct hynitron_ts_data *ts_data)
{
	struct i2c_client *client = ts_data->client;
	bool on = !ts_data->panel_off && !ts_data->suspended;
	int err;

	if (on == ts_data->powered)
		return;

	if (on) {
		err = hyn_power_on(ts_data);
		if (err) {
			dev_err(&client->dev, "failed to enable vdd: %d\n", err);
			return;
		}
		enable_irq(client->irq);
	} else {
		disable_irq(client->irq);
		err = ts_data->chip->suspend(client);
		if (err)
			dev_warn(&client->dev, "chip suspend failed: %d\n", err);
		input_mt_sync_frame(ts_data->input_dev);
		input_sync(ts_data->input_dev);
		gpiod_set_value_cansleep(ts_data->reset_gpio, 1);
		regulator_disable(ts_data->vdd);
	}

	ts_data->powered = on;
}

static void hyn_set_power_flag(struct hynitron_ts_data *ts_data, bool *flag,
			       bool val)
{
	guard(mutex)(&ts_data->pm_lock);
	*flag = val;
	hyn_update_power(ts_data);
}

static void hyn_vdd_disable(void *data)
{
	struct hynitron_ts_data *ts_data = data;

	if (ts_data->powered)
		regulator_disable(ts_data->vdd);
}

static int hyn_panel_disabling(struct drm_panel_follower *follower)
{
	struct hynitron_ts_data *ts_data =
		container_of(follower, struct hynitron_ts_data, panel_follower);

	hyn_set_power_flag(ts_data, &ts_data->panel_off, true);

	return 0;
}

static int hyn_panel_enabled(struct drm_panel_follower *follower)
{
	struct hynitron_ts_data *ts_data =
		container_of(follower, struct hynitron_ts_data, panel_follower);

	hyn_set_power_flag(ts_data, &ts_data->panel_off, false);

	return 0;
}

static const struct drm_panel_follower_funcs hyn_panel_follower_funcs = {
	.panel_disabling = hyn_panel_disabling,
	.panel_enabled = hyn_panel_enabled,
};

static int hyn_probe(struct i2c_client *client)
{
	struct hynitron_ts_data *ts_data;
	int err;

	ts_data = devm_kzalloc(&client->dev, sizeof(*ts_data), GFP_KERNEL);
	if (!ts_data)
		return -ENOMEM;

	ts_data->client = client;
	i2c_set_clientdata(client, ts_data);

	ts_data->chip = device_get_match_data(&client->dev);
	if (!ts_data->chip)
		return -EINVAL;

	err = devm_mutex_init(&client->dev, &ts_data->pm_lock);
	if (err)
		return err;

	ts_data->reset_gpio = devm_gpiod_get(&client->dev, "reset",
					     ts_data->chip->resume ?
					     GPIOD_OUT_HIGH : GPIOD_OUT_LOW);
	err = PTR_ERR_OR_ZERO(ts_data->reset_gpio);
	if (err) {
		dev_err(&client->dev, "request reset gpio failed: %d\n", err);
		return err;
	}

	ts_data->vdd = devm_regulator_get(&client->dev, "vdd");
	if (IS_ERR(ts_data->vdd)) {
		err = PTR_ERR(ts_data->vdd);
		if (err != -EPROBE_DEFER)
			dev_err(&client->dev, "Failed to request vdd regulator: %d\n", err);
		return err;
	}

	if (ts_data->chip->resume)
		err = hyn_power_on(ts_data);
	else
		err = regulator_enable(ts_data->vdd);
	if (err) {
		dev_err(&client->dev, "enable regulator failed: %d\n", err);
		return err;
	}

	ts_data->powered = true;
	err = devm_add_action_or_reset(&client->dev, hyn_vdd_disable, ts_data);
	if (err)
		return err;

	if (!ts_data->chip->resume)
		hyn_reset_proc(client, 60);

	// err = ts_data->chip->bootloader_enter(client);
	// if (err < 0)
	// 	return err;

	err = ts_data->chip->init_input(client);
	if (err < 0)
		return err;

	err = ts_data->chip->firmware_info(client);
	if (err < 0)
		return err;

	err = devm_request_threaded_irq(&client->dev, client->irq,
					NULL, hyn_interrupt_handler,
					IRQF_ONESHOT,
					"Hynitron Touch Int", client);
	if (err) {
		dev_err(&client->dev, "failed to request IRQ: %d\n", err);
		return err;
	}

	if (ts_data->chip->suspend && drm_is_panel_follower(&client->dev)) {
		ts_data->panel_follower.funcs = &hyn_panel_follower_funcs;
		err = devm_drm_panel_add_follower(&client->dev,
						  &ts_data->panel_follower);
		if (err)
			return dev_err_probe(&client->dev, err,
					     "failed to follow panel\n");
	}

	return 0;
}

static int hyn_suspend(struct device *dev)
{
	struct hynitron_ts_data *ts_data = dev_get_drvdata(dev);

	if (ts_data->chip->suspend)
		hyn_set_power_flag(ts_data, &ts_data->suspended, true);

	return 0;
}

static int hyn_resume(struct device *dev)
{
	struct hynitron_ts_data *ts_data = dev_get_drvdata(dev);

	if (ts_data->chip->suspend)
		hyn_set_power_flag(ts_data, &ts_data->suspended, false);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(hyn_pm_ops, hyn_suspend, hyn_resume);

static const struct hynitron_ts_chip_data cst3xx_data = {
	.max_touch_num		= 5,
	.ic_chkcode		= 0xcaca0000,
	.firmware_info		= &cst3xx_firmware_info,
	.bootloader_enter	= &cst3xx_bootloader_enter,
	.init_input		= &cst3xx_input_dev_int,
	.report_touch		= &cst3xx_touch_report,
};

static const struct hynitron_ts_chip_data cst66xx_data = {
	.max_touch_num		= CST66XX_MAX_POINTS,
	.ic_chkcode		= 0xcaca0000,
	.firmware_info		= &cst66xx_firmware_info,
	.init_input		= &cst3xx_input_dev_int,
	.report_touch		= &cst66xx_touch_report,
	.suspend		= &cst66xx_suspend,
	.resume			= &cst66xx_resume,
};

static const struct i2c_device_id hyn_tpd_id[] = {
	{ .name = "hynitron_ts" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(i2c, hyn_tpd_id);

static const struct of_device_id hyn_dt_match[] = {
	{ .compatible = "hynitron,cst340", .data = &cst3xx_data },
	{ .compatible = "hynitron,cst3530", .data = &cst66xx_data },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, hyn_dt_match);

static struct i2c_driver hynitron_i2c_driver = {
	.driver = {
		.name = "Hynitron-TS",
		.of_match_table = hyn_dt_match,
		.pm = pm_sleep_ptr(&hyn_pm_ops),
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.id_table = hyn_tpd_id,
	.probe = hyn_probe,
};

module_i2c_driver(hynitron_i2c_driver);

MODULE_AUTHOR("Chris Morgan");
MODULE_DESCRIPTION("Hynitron Touchscreen Driver");
MODULE_LICENSE("GPL");
