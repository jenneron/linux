/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _SYNAPTICS_DSX_H_
#define _SYNAPTICS_DSX_H_

#include <linux/gpio/consumer.h>

#define PLATFORM_DRIVER_NAME "synaptics_dsx"
#define I2C_DRIVER_NAME "synaptics_dsx_i2c"

/*
 * struct synaptics_dsx_board_data - DSX board data
 * @x_flip: x flip flag
 * @y_flip: y flip flag
 * @swap_axes: swap axes flag
 * @irq_gpiod: attention interrupt GPIO descriptor
 * @power_gpiod: power switch GPIO descriptor
 * @reset_gpiod: reset GPIO descriptor
 * @max_y_for_2d: maximum y value for 2D area
 * @irq_flags: IRQ flags
 * @i2c_addr: I2C slave address
 * @ub_i2c_addr: microbootloader mode I2C slave address
 * @power_delay_ms: delay time to wait after powering up device
 * @reset_delay_ms: delay time to wait after resetting device
 * @reset_active_ms: reset active time
 * @pwr_reg_name: pointer to name of regulator for power control
 * @bus_reg_name: pointer to name of regulator for bus pullup control
 */
struct synaptics_dsx_board_data {
	bool x_flip;
	bool y_flip;
	bool swap_axes;
	struct gpio_desc *irq_gpiod;
	struct gpio_desc *power_gpiod;
	struct gpio_desc *reset_gpiod;
	int max_y_for_2d;
	unsigned long irq_flags;
	unsigned short i2c_addr;
	unsigned short ub_i2c_addr;
	unsigned int power_delay_ms;
	unsigned int reset_delay_ms;
	unsigned int reset_active_ms;
	const char *pwr_reg_name;
	const char *bus_reg_name;
};

#endif
