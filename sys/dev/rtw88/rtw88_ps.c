/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * rtw88 power-save subsystem.  Implements the shallow LPS layer using
 * the chip's SET_PWR_MODE H2C protocol: ACTIVE <-> LPS + awake_interval
 * + smart_ps.  The H2C command format is a hardware protocol shared
 * with Linux upstream rtw88; the code below is an independent FreeBSD
 * implementation, not a translation of Linux `ps.c`.
 *
 * Deep-PS (SR / Selective Suspend) is deferred: requires the chip's
 * power-sequence table and pause-tx machinery.  Interface leaves a
 * hook (`rtw88_lps_deep_supported`) so it can be added later.
 */

#include <sys/param.h>
#include <sys/systm.h>

#include "rtw88.h"
#include "rtw88_ps.h"
#include "rtw88_debug.h"
#include "rtw88_regs.h"

/* H2C_CMD_SET_PWR_MODE comes from rtw88_regs.h. */

/*
 * ACTIVE-mode word encoding (matches captured Linux frame
 * "20 00 00 00 c0 00 00 00"):
 *   byte 0 = 0x20 (cmd)
 *   byte 1 = 0x00 (mode = 0 = ACTIVE)
 *   ex bytes = 0x0c00 = pwr_state 0x0c, port 0
 */
static int
rtw88_h2c_pwr_mode_active(struct rtw88_dev *d, uint8_t macid)
{
	uint32_t msg = (uint32_t)H2C_CMD_SET_PWR_MODE |
	    ((uint32_t)macid << 16);
	uint32_t ex = (0u << 5) | (0x0cu << 8);

	return (rtw_h2c(d, H2C_CMD_SET_PWR_MODE, msg, ex));
}

/*
 * LPS-mode word encoding (matches captured Linux frame
 * "20 01 21 01 00 0c 00 00"):
 *   byte 0 = 0x20 (cmd)
 *   byte 1 = 0x01 (mode = 1 = LPS)
 *   byte 2 = 0x21 (rlbm=1, smart_ps=2)
 *   byte 3 = 0x01 (awake_interval=1 beacon)
 *   ex bytes = 0x0c00 = pwr_state 0x0c, port 0
 */
static int
rtw88_h2c_pwr_mode_lps(struct rtw88_dev *d, uint8_t macid,
    uint8_t awake_interval, uint8_t smart_ps)
{
	uint32_t msg = (uint32_t)H2C_CMD_SET_PWR_MODE |
	    (1u << 8) |
	    (1u << 16) |
	    ((uint32_t)(smart_ps & 0x3) << 20) |
	    ((uint32_t)awake_interval << 24);
	uint32_t ex = ((uint32_t)macid << 0) | (0x0cu << 8);

	return (rtw_h2c(d, H2C_CMD_SET_PWR_MODE, msg, ex));
}

/* ------------------------------------------------------------------ */
/* Attach/detach.                                                      */
/* ------------------------------------------------------------------ */
void
rtw88_ps_attach(struct rtw88_dev *d, uint8_t macid)
{
	d->lps.mode = RTW88_LPS_MODE_ACTIVE;
	d->lps.in_lps = false;
	d->lps.wow = false;
	d->lps.awake_interval = 1;
	d->lps.smart_ps = 2;
	d->lps.macid = macid;
}

void
rtw88_ps_detach(struct rtw88_dev *d)
{
	if (d->lps.in_lps)
		(void)rtw88_lps_leave(d);
}

/* ------------------------------------------------------------------ */
/* Enter / leave.                                                      */
/* ------------------------------------------------------------------ */
int
rtw88_lps_enter(struct rtw88_dev *d)
{
	int rc;

	if (d->lps.in_lps)
		return (0);

	rc = rtw88_h2c_pwr_mode_lps(d, d->lps.macid,
	    d->lps.awake_interval, d->lps.smart_ps);
	if (rc == 0) {
		d->lps.mode = RTW88_LPS_MODE_LEGACY;
		d->lps.in_lps = true;
		rtw88_debug_inc(d, RTW88_DBG_CNT_PS_ENTER);
	}
	return (rc);
}

int
rtw88_lps_leave(struct rtw88_dev *d)
{
	int rc;

	if (!d->lps.in_lps) {
		/*
		 * Even when we think we're already ACTIVE, re-fire the
		 * ACTIVE H2C: firmware's state machine sometimes drifts
		 * (captured under sustained TX) and re-arming ACTIVE is
		 * cheap.  Match Linux's rtw_leave_lps_check idempotency.
		 */
		return (rtw88_h2c_pwr_mode_active(d, d->lps.macid));
	}

	rc = rtw88_h2c_pwr_mode_active(d, d->lps.macid);
	if (rc == 0) {
		d->lps.mode = RTW88_LPS_MODE_ACTIVE;
		d->lps.in_lps = false;
		rtw88_debug_inc(d, RTW88_DBG_CNT_PS_LEAVE);
	}
	return (rc);
}

int
rtw88_ps_notify(struct rtw88_dev *d, bool wants_ps)
{
	if (wants_ps)
		return (rtw88_lps_enter(d));
	return (rtw88_lps_leave(d));
}
