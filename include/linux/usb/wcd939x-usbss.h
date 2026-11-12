/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __LINUX_USB_WCD939X_USBSS_H
#define __LINUX_USB_WCD939X_USBSS_H

#include <linux/kconfig.h>
#include <linux/types.h>

struct device;

#if IS_REACHABLE(CONFIG_TYPEC_MUX_WCD939X_USBSS)
int wcd939x_usbss_audio_config(struct device *dev, bool enable);
#else
static inline int wcd939x_usbss_audio_config(struct device *dev, bool enable)
{
	return 0;
}
#endif

#endif /* __LINUX_USB_WCD939X_USBSS_H */
