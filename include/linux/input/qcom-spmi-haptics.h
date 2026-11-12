/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_INPUT_QCOM_SPMI_HAPTICS_H
#define _LINUX_INPUT_QCOM_SPMI_HAPTICS_H

#include <linux/errno.h>
#include <linux/kconfig.h>

/*
 * Let another input device (such as a gamepad) play a rumble effect on
 * the PMIC SPMI haptics.
 */
#if IS_REACHABLE(CONFIG_INPUT_QCOM_SPMI_HAPTICS)
int qcom_spmi_haptics_rumble(unsigned int strong_magnitude,
			     unsigned int weak_magnitude);
#else
static inline int qcom_spmi_haptics_rumble(unsigned int strong_magnitude,
					   unsigned int weak_magnitude)
{
	return -ENODEV;
}
#endif

#endif
