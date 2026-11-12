/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_INPUT_QCOM_HV_HAPTICS_H
#define _LINUX_INPUT_QCOM_HV_HAPTICS_H

#include <linux/types.h>

struct ff_effect;

/*
 * Let another input device (such as a gamepad) forward its force
 * feedback effects to the PMIC high voltage haptics.
 */
int qcom_spmi_haptics_global_upload(struct ff_effect *effect);
int qcom_spmi_haptics_global_playback(int effect_id, int val);
int qcom_spmi_haptics_global_set_gain(u16 gain);

/* Whether the HAP530 VI sense feedback is enabled. */
bool qcom_haptics_vi_sense_is_enabled(void);

#endif
