/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * rtw88 LED subsystem.
 *
 * Uses FreeBSD's led(9) framework: at attach we register a /dev/led/
 * node that userspace can drive with the standard led syntax
 * (`s{on|off}`, `f<period>`, patterns).  A background hook from the
 * TX/RX paths pulses the LED on activity.
 *
 * The register that drives the LED is chip-family-specific.  8821C
 * uses REG_LED_CFG bit BIT_LED0_SV; other chips override via the
 * gpio-mask field in chip_info.
 */

#ifndef _RTW88_LED_H_
#define _RTW88_LED_H_

#include "rtw88.h"

int	rtw88_led_attach(struct rtw88_dev *rtwdev);
void	rtw88_led_detach(struct rtw88_dev *rtwdev);

/* Set the LED steady-on or steady-off from driver code. */
void	rtw88_led_set(struct rtw88_dev *rtwdev, bool on);

#endif /* _RTW88_LED_H_ */
