/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * rtw88 LED subsystem, FreeBSD led(9) glue.
 *
 * Registers /dev/led/<device_name> with a callback that writes the
 * chip's LED-control register.  Userspace pattern strings work as
 * documented in led(4).
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <dev/led/led.h>

#include "rtw88.h"
#include "rtw88_led.h"
#include "rtw88_regs.h"

static void
rtw88_led_callback(void *arg, int onoff)
{
	struct rtw88_dev *d = arg;
	uint32_t v;

	if (!d->led.attached)
		return;

	RTW88_DEV_LOCK(d);
	if (rtw_read32(d, REG_LED_CFG, &v) != 0) {
		RTW88_DEV_UNLOCK(d);
		return;
	}
	if (onoff)
		v &= ~BIT_LED0_SV;	/* active-low on 8821C */
	else
		v |= BIT_LED0_SV;
	(void)rtw_write32(d, REG_LED_CFG, v);
	RTW88_DEV_UNLOCK(d);

	d->led.on = (onoff != 0);
}

int
rtw88_led_attach(struct rtw88_dev *d)
{
	char name[32];

	if (!d->chip->led_supported)
		return (0);

	snprintf(name, sizeof(name), "%s",
	    device_get_nameunit(d->dev));
	d->led.cdev = led_create(rtw88_led_callback, d, name);
	if (d->led.cdev == NULL) {
		rtw_warn(d, "led_create(%s) failed\n", name);
		return (ENOMEM);
	}
	d->led.attached = true;
	rtw88_led_set(d, false);
	return (0);
}

void
rtw88_led_detach(struct rtw88_dev *d)
{
	if (!d->led.attached)
		return;
	if (d->led.cdev != NULL)
		led_destroy(d->led.cdev);
	d->led.cdev = NULL;
	d->led.attached = false;
}

void
rtw88_led_set(struct rtw88_dev *d, bool on)
{
	rtw88_led_callback(d, on ? 1 : 0);
}
