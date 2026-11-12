// SPDX-License-Identifier: GPL-2.0-only
/*
 * RSInput Gamepad Driver
 *
 * Copyright (C) 2024 Teguh Sobirin <teguh@sobir.in>
 *
 */
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/serdev.h>
#include <linux/slab.h>
#include <uapi/linux/sched/types.h>

#define FRAME_HEAD_1              0xA5
#define FRAME_HEAD_2              0xD3
#define FRAME_HEAD_3              0x5A
#define FRAME_HEAD_4              0x3D

#define CMD_COMMOD                0x01
#define CMD_STATUS                0x02

#define DATA_COMMOD_VERSION       0x02
#define DATA_COMMOD_SET_PAR       0x05

#define FRAME_POS_SEQ             4
#define FRAME_POS_CMD             5
#define FRAME_POS_LEN_L           6
#define FRAME_POS_LEN_H           7
#define FRAME_POS_DATA_1          8
#define FRAME_POS_DATA_2          9
#define FRAME_POS_DATA_3          10
#define FRAME_POS_DATA_4          11
#define FRAME_POS_DATA_5          12
#define FRAME_POS_DATA_6          13
#define FRAME_POS_DATA_7          14
#define FRAME_POS_DATA_8          15
#define FRAME_POS_DATA_9          16
#define FRAME_POS_DATA_10         17
#define FRAME_POS_DATA_11         18
#define FRAME_POS_DATA_12         19
#define FRAME_POS_DATA_13         20
#define FRAME_POS_DATA_14         21

#define MCU_PKT_SIZE_MIN          9

#define MCU_VERSION_MAX_LEN       64
#define REPORT_RESUME_TIME        500

struct rsinput_driver {
    struct serdev_device *serdev;
    struct input_dev *input;
	struct regulator *vdd;
    struct gpio_desc *boot_gpio;
    struct gpio_desc *enable_gpio;
    struct gpio_desc *reset_gpio;
    uint8_t rx_buf[1024];
    uint8_t sequence_number;
    bool resuming;
    ktime_t resume_time;
    bool invert_x;
    bool invert_y;
    bool invert_rx;
    bool invert_ry;
    bool digital_triggers;
    u32 axis_range;
    u32 trigger_left_deadzone;
    u32 trigger_right_deadzone;
};

static bool rumble_enable = true;
module_param(rumble_enable, bool, 0660);
MODULE_PARM_DESC(rumble_enable, "Forward rumble effects to the PMIC haptics");

static const unsigned int keymap[] = {
    BTN_DPAD_UP, BTN_DPAD_DOWN, BTN_DPAD_LEFT, BTN_DPAD_RIGHT,
    BTN_NORTH,   BTN_WEST,	    BTN_EAST,	   BTN_SOUTH,
    BTN_TL,	     BTN_TR,	    BTN_SELECT,	   BTN_START,
    BTN_THUMBL,  BTN_THUMBR,    BTN_MODE,	   BTN_BACK
};

static uint8_t compute_checksum(const uint8_t *data, size_t len) {
    uint8_t checksum = 0;

    for (size_t i = FRAME_POS_SEQ; i < len - 1; i++) {
        checksum ^= data[i];
    }

    return checksum;
}

static int rsinput_send_command(struct rsinput_driver *drv, uint8_t cmd, const uint8_t *data, size_t len) {
    uint8_t frame[256];
    uint8_t checksum = 0;
    size_t frame_len = 0;

    frame[frame_len++] = FRAME_HEAD_1;
    frame[frame_len++] = FRAME_HEAD_2;
    frame[frame_len++] = FRAME_HEAD_3;
    frame[frame_len++] = FRAME_HEAD_4;

    frame[frame_len++] = drv->sequence_number;
    drv->sequence_number++;

    frame[frame_len++] = cmd;

    frame[frame_len++] = len & 0xFF;
    frame[frame_len++] = (len >> 8) & 0xFF;

    if (data && len) {
        memcpy(&frame[frame_len], data, len);
        frame_len += len;
    }

    checksum = compute_checksum(frame, frame_len + 1);
    frame[frame_len++] = checksum;

    return serdev_device_write_buf(drv->serdev, frame, frame_len);
}

static int rsinput_init_commands(struct rsinput_driver *drv) {
    int error;

    msleep(50);
    uint8_t version_request[] = {DATA_COMMOD_VERSION};
    error = rsinput_send_command(drv, CMD_COMMOD, version_request, sizeof(version_request));
    if (error < 0) {
        dev_err(&drv->serdev->dev, "Failed to request MCU version: %d\n", error);
        return error;
    }

    msleep(50);
    uint8_t mcu_params[] = {
        DATA_COMMOD_SET_PAR, 
        0x01,
        0x00, 0x00, 0x00, 0x28,
        0x00, 0x00, 0x00, 0x07
    };
    error = rsinput_send_command(drv, CMD_COMMOD, mcu_params, sizeof(mcu_params));
    if (error < 0) {
        dev_err(&drv->serdev->dev, "Failed to set MCU parameters: %d\n", error);
        return error;
    }

    return 0;
}

static void handle_cmd_commod(struct rsinput_driver *drv, const uint8_t *data, size_t payload_length) {
    switch (data[FRAME_POS_DATA_1]) {
        case DATA_COMMOD_VERSION:
            if (payload_length >= 1) {
                char mcu_version[MCU_VERSION_MAX_LEN] = {0};
                size_t version_length = payload_length;
                if (version_length > MCU_VERSION_MAX_LEN - 1) {
                    version_length = MCU_VERSION_MAX_LEN - 1;
                }
                memcpy(mcu_version, &data[FRAME_POS_DATA_1], version_length);
                mcu_version[version_length] = '\0';
                dev_info(&drv->serdev->dev, "MCU Version: %s\n", mcu_version);
            } else {
                dev_err(&drv->serdev->dev, "Invalid MCU version response length\n");
            }
            break;
        case DATA_COMMOD_SET_PAR:
            dev_dbg(&drv->serdev->dev, "MCU parameters set successfully\n");
            break;
        default:
            dev_dbg(&drv->serdev->dev, "Unhandled CMD_COMMOD sub-command: 0x%02x\n", data[FRAME_POS_DATA_1]);
            break;
    }
}

#define TRIGGER_RAW_FULL_RANGE    0x755
#define TRIGGER_RAW_MIN           354
#define TRIGGER_RAW_MAX           1845
#define AXIS_RANGE_DEFAULT        0x580

static int rsinput_trigger(u16 raw, u32 deadzone) {
    int val = clamp(TRIGGER_RAW_FULL_RANGE - (int)raw,
                    TRIGGER_RAW_MIN, TRIGGER_RAW_MAX) - TRIGGER_RAW_MIN;

    return val < (int)deadzone ? 0 : val;
}

static int rsinput_axis(struct rsinput_driver *drv, const uint8_t *p, bool invert) {
    int val = -(int16_t)(p[0] | (p[1] << 8));

    if (invert)
        val = -val;

    return clamp(val, -(int)drv->axis_range, (int)drv->axis_range);
}

static void handle_cmd_status(struct rsinput_driver *drv, const uint8_t *data, size_t payload_length) {
    if (drv->resuming) {
        if (ktime_ms_delta(ktime_get(), drv->resume_time) < REPORT_RESUME_TIME)
            return;
        drv->resuming = false;
    }

    if (payload_length >= 6) {
    static unsigned long prev_states;
    unsigned long keys = data[FRAME_POS_DATA_1] | (data[FRAME_POS_DATA_2] << 8);
    unsigned long current_states = keys, changes;
    int i;

    bitmap_xor(&changes, &current_states, &prev_states, ARRAY_SIZE(keymap));

    for_each_set_bit(i, &changes, ARRAY_SIZE(keymap)) {
        input_report_key(drv->input, keymap[i], (current_states & BIT(i)));
    }

    u16 raw_l = data[FRAME_POS_DATA_3] | (data[FRAME_POS_DATA_4] << 8);
    u16 raw_r = data[FRAME_POS_DATA_5] | (data[FRAME_POS_DATA_6] << 8);

    if (drv->digital_triggers) {
        input_report_key(drv->input, BTN_TL2, raw_l != 0);
        input_report_key(drv->input, BTN_TR2, raw_r != 0);
    } else {
        input_report_abs(drv->input, ABS_Z,
                         rsinput_trigger(raw_l, drv->trigger_left_deadzone));
        input_report_abs(drv->input, ABS_RZ,
                         rsinput_trigger(raw_r, drv->trigger_right_deadzone));
    }

    input_report_abs(drv->input, ABS_X, rsinput_axis(drv, &data[FRAME_POS_DATA_7], drv->invert_x));
    input_report_abs(drv->input, ABS_Y, rsinput_axis(drv, &data[FRAME_POS_DATA_9], drv->invert_y));
    input_report_abs(drv->input, ABS_RX, rsinput_axis(drv, &data[FRAME_POS_DATA_11], drv->invert_rx));
    input_report_abs(drv->input, ABS_RY, rsinput_axis(drv, &data[FRAME_POS_DATA_13], drv->invert_ry));

    input_sync(drv->input);
    prev_states = keys;

    } else {
        dev_warn(&drv->serdev->dev, "Invalid CMD_STATUS response length\n");
    }
}

static void rsinput_process_data(struct rsinput_driver *drv, const uint8_t *data, size_t len) {
    while (len >= MCU_PKT_SIZE_MIN) {
        if (data[0] != FRAME_HEAD_1) {
            data++;
            len--;
            continue;
        }

        uint16_t payload_length = data[FRAME_POS_LEN_L] | (data[FRAME_POS_LEN_H] << 8);
        size_t frame_length = MCU_PKT_SIZE_MIN + payload_length;

        if (len < frame_length) {
            return;
        }

        uint8_t received_checksum = data[frame_length - 1];
        uint8_t computed_checksum = compute_checksum(data, frame_length);
        if (computed_checksum != received_checksum) {
            data += frame_length;
            len -= frame_length;
            continue;
        }

        switch (data[FRAME_POS_CMD]) {
            case CMD_COMMOD:
                handle_cmd_commod(drv, data, payload_length);
                break;
            case CMD_STATUS:
                handle_cmd_status(drv, data, payload_length);
                break;
            default:
                dev_warn(&drv->serdev->dev, "Unhandled command: 0x%02X\n", data[FRAME_POS_CMD]);
                break;
        }

        data += frame_length;
        len -= frame_length;
    }

    if (len > 0) {
        dev_warn(&drv->serdev->dev, "Trailing bytes after processing: %zu\n", len);
    }
}

static size_t rsinput_rx(struct serdev_device *serdev, const u8 *data, size_t count) {
    struct rsinput_driver *drv = serdev_device_get_drvdata(serdev);

    if (!drv || !data || count == 0) {
        dev_dbg_ratelimited(&serdev->dev, "Invalid RX data\n");
        goto error;
    }

    if (count > sizeof(drv->rx_buf)) {
        dev_dbg_ratelimited(&serdev->dev, "RX buffer overflow\n");
        goto error;
    }

    if (count < MCU_PKT_SIZE_MIN) {
        dev_dbg_ratelimited(&serdev->dev, "Frame too short for checksum validation\n");
        goto error;
    }

    memcpy(drv->rx_buf, data, count);

    rsinput_process_data(drv, drv->rx_buf, count);

error:
    return count;
}

static const struct serdev_device_ops rsinput_rx_ops = {
    .receive_buf = rsinput_rx,
};

#if IS_REACHABLE(CONFIG_INPUT_QCOM_HV_HAPTICS)
extern int qcom_spmi_haptics_global_upload(struct ff_effect *effect);
extern int qcom_spmi_haptics_global_playback(int effect_id, int val);

static int rsinput_rumble_upload(struct input_dev *dev, struct ff_effect *effect,
                                 struct ff_effect *old) {
    if (!rumble_enable)
        return 0;

    return qcom_spmi_haptics_global_upload(effect);
}

static int rsinput_rumble_play(struct input_dev *dev, int effect_id, int val) {
    if (!rumble_enable)
        return 0;

    return qcom_spmi_haptics_global_playback(effect_id, val);
}

static int rsinput_setup_rumble(struct rsinput_driver *drv) {
    int error;

    input_set_capability(drv->input, EV_FF, FF_PERIODIC);

    error = input_ff_create(drv->input, 1);
    if (error)
        return dev_err_probe(&drv->serdev->dev, error,
                             "Unable to create force feedback device\n");

    drv->input->ff->upload = rsinput_rumble_upload;
    drv->input->ff->playback = rsinput_rumble_play;

    return 0;
}
#else
static int rsinput_setup_rumble(struct rsinput_driver *drv) {
    return 0;
}
#endif

static int rsinput_probe(struct serdev_device *serdev) {
    struct rsinput_driver *drv;
    u32 gamepad_bus = 0;
    u32 gamepad_vid = 0;
    u32 gamepad_pid = 0;
    u32 gamepad_rev = 0;
    int error;

    drv = devm_kzalloc(&serdev->dev, sizeof(*drv), GFP_KERNEL);
    if (!drv)
    return -ENOMEM;

    drv->invert_x = device_property_read_bool(&serdev->dev, "invert-x");
    drv->invert_y = device_property_read_bool(&serdev->dev, "invert-y");
    drv->invert_rx = device_property_read_bool(&serdev->dev, "invert-rx");
    drv->invert_ry = device_property_read_bool(&serdev->dev, "invert-ry");
    drv->digital_triggers = device_property_read_bool(&serdev->dev, "digital-triggers");

    drv->axis_range = AXIS_RANGE_DEFAULT;
    device_property_read_u32(&serdev->dev, "axis-range", &drv->axis_range);
    device_property_read_u32(&serdev->dev, "trigger-left-deadzone",
                             &drv->trigger_left_deadzone);
    device_property_read_u32(&serdev->dev, "trigger-right-deadzone",
                             &drv->trigger_right_deadzone);

    drv->vdd = devm_regulator_get(&serdev->dev, "vdd");
	if (IS_ERR(drv->vdd)) {
		error = PTR_ERR(drv->vdd);
		return error;
	}

    drv->boot_gpio =
        devm_gpiod_get_optional(&serdev->dev, "boot", GPIOD_OUT_HIGH);
    if (IS_ERR(drv->boot_gpio)) {
        error = PTR_ERR(drv->boot_gpio);
        dev_warn(&serdev->dev, "Unable to get boot gpio: %d\n", error);
    }

    drv->enable_gpio =
        devm_gpiod_get_optional(&serdev->dev, "enable", GPIOD_OUT_HIGH);
    if (IS_ERR(drv->enable_gpio)) {
        error = PTR_ERR(drv->enable_gpio);
        dev_warn(&serdev->dev, "Unable to get enable gpio: %d\n", error);
    }

    drv->reset_gpio =
        devm_gpiod_get_optional(&serdev->dev, "reset", GPIOD_OUT_HIGH);
    if (IS_ERR(drv->reset_gpio)) {
        error = PTR_ERR(drv->reset_gpio);
        dev_warn(&serdev->dev, "Unable to get reset gpio: %d\n", error);
    }

	error = regulator_enable(drv->vdd);
	if (error < 0) {
		return error;
	}

    if (drv->boot_gpio)
        gpiod_set_value_cansleep(drv->boot_gpio, 0);

    if (drv->reset_gpio)
        gpiod_set_value_cansleep(drv->reset_gpio, 0);

    msleep(20);

    if (drv->enable_gpio)
        gpiod_set_value_cansleep(drv->enable_gpio, 1);

    if (drv->reset_gpio)
        gpiod_set_value_cansleep(drv->reset_gpio, 1);

    msleep(50);

    error = serdev_device_open(serdev);
    if (error)
        return dev_err_probe(&serdev->dev, error, "Unable to open UART device\n");

    drv->serdev = serdev;
    drv->sequence_number = 0;
    
    serdev_device_set_drvdata(serdev, drv);

    error = serdev_device_set_baudrate(serdev, 115200);
    if (error < 0)
        return dev_err_probe(&serdev->dev, error, "Failed to set up host baud rate\n");

    serdev_device_set_flow_control(serdev, false);

    drv->input = devm_input_allocate_device(&serdev->dev);
    if (!drv->input)
        return -ENOMEM;

    drv->input->phys = "rsinput-gamepad/input0";
    
    error = device_property_read_string(&serdev->dev, "gamepad-name", &drv->input->name);
    if (error) {
        drv->input->name = "RSInput Gamepad";
    }

    device_property_read_u32(&serdev->dev, "gamepad-bus", &gamepad_bus);
    device_property_read_u32(&serdev->dev, "gamepad-vid", &gamepad_vid);
    device_property_read_u32(&serdev->dev, "gamepad-pid", &gamepad_pid);
    device_property_read_u32(&serdev->dev, "gamepad-rev", &gamepad_rev);

    drv->input->id.bustype = (u16)gamepad_bus;
    drv->input->id.vendor  = (u16)gamepad_vid;
    drv->input->id.product = (u16)gamepad_pid;
    drv->input->id.version = (u16)gamepad_rev;

    __set_bit(EV_KEY, drv->input->evbit);
    for (int i = 0; i < ARRAY_SIZE(keymap); i++)
        input_set_capability(drv->input, EV_KEY, keymap[i]);

    __set_bit(EV_ABS, drv->input->evbit);
    input_set_abs_params(drv->input, ABS_X, -drv->axis_range, drv->axis_range, 0, 0);
    input_set_abs_params(drv->input, ABS_Y, -drv->axis_range, drv->axis_range, 0, 0);
    input_set_abs_params(drv->input, ABS_RX, -drv->axis_range, drv->axis_range, 0, 0);
    input_set_abs_params(drv->input, ABS_RY, -drv->axis_range, drv->axis_range, 0, 0);

    if (drv->digital_triggers) {
        input_set_capability(drv->input, EV_KEY, BTN_TL2);
        input_set_capability(drv->input, EV_KEY, BTN_TR2);
    } else {
        input_set_abs_params(drv->input, ABS_Z, 0,
                             TRIGGER_RAW_MAX - TRIGGER_RAW_MIN, 0, 0);
        input_set_abs_params(drv->input, ABS_RZ, 0,
                             TRIGGER_RAW_MAX - TRIGGER_RAW_MIN, 0, 0);
    }

    error = rsinput_setup_rumble(drv);
    if (error)
        return error;

    error = input_register_device(drv->input);
    if (error)
        return error;

    serdev_device_set_client_ops(serdev, &rsinput_rx_ops);

    error = rsinput_init_commands(drv);
    if (error < 0) {
        serdev_device_close(serdev);
        return error;
    }

    return 0;
}

static int rsinput_suspend(struct device *dev)
{
    struct serdev_device *serdev = to_serdev_device(dev);
    struct rsinput_driver *drv = serdev_device_get_drvdata(serdev);

    serdev_device_close(serdev);

    if (drv->enable_gpio)
        gpiod_set_value_cansleep(drv->enable_gpio, 0);

    if (drv->reset_gpio)
        gpiod_set_value_cansleep(drv->reset_gpio, 0);
    
    regulator_disable(drv->vdd);

    return 0;
}

static int rsinput_resume(struct device *dev)
{
    struct serdev_device *serdev = to_serdev_device(dev);
    struct rsinput_driver *drv = serdev_device_get_drvdata(serdev);
    int error;

    error = regulator_enable(drv->vdd);
	if (error < 0) {
		return error;
	}

    if (drv->reset_gpio)
        gpiod_set_value_cansleep(drv->reset_gpio, 0);

    if (drv->enable_gpio)
        gpiod_set_value_cansleep(drv->enable_gpio, 1);

    if (drv->reset_gpio)
        gpiod_set_value_cansleep(drv->reset_gpio, 1);

    msleep(50);

    error = serdev_device_open(serdev);
    if (error)
        return dev_err_probe(dev, error, "Failed to reopen UART on resume\n");

    error = serdev_device_set_baudrate(serdev, 115200);
    if (error < 0)
        return dev_err_probe(dev, error, "Failed to restore baud rate on resume\n");

    serdev_device_set_flow_control(serdev, false);

    drv->resume_time = ktime_get();
    drv->resuming = true;
    drv->sequence_number = 0;

    error = rsinput_init_commands(drv);
    if (error < 0) {
        serdev_device_close(serdev);
        return error;
    }

    return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(rsinput_pm_ops, rsinput_suspend, rsinput_resume);

static void rsinput_remove(struct serdev_device *serdev) {
    struct rsinput_driver *drv = serdev_device_get_drvdata(serdev);

    serdev_device_close(serdev);
    input_unregister_device(drv->input);
    if (drv->enable_gpio)
        gpiod_set_value_cansleep(drv->enable_gpio, 0);

    if (drv->reset_gpio)
        gpiod_set_value_cansleep(drv->reset_gpio, 0);
    
    regulator_disable(drv->vdd);
}

static const struct of_device_id rsinput_of_match[] = {
    { .compatible = "ayntec,rsinput" },
    { .compatible = "retroidpocket,rsinput" },
    { .compatible = "gamepad,rsinput" },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rsinput_of_match);

static struct serdev_device_driver rsinput_driver = {
    .probe = rsinput_probe,
    .remove = rsinput_remove,
    .driver = {
        .name = "rsinput",
        .of_match_table = rsinput_of_match,
        .pm = pm_sleep_ptr(&rsinput_pm_ops),
    },
};

module_serdev_device_driver(rsinput_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("RSInput Gamepad Driver");
MODULE_AUTHOR("Teguh Sobirin <teguh@sobir.in>");
