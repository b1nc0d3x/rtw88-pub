/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * rtw88 power-save subsystem.
 *
 * Shallow LPS: firmware SET_PWR_MODE H2C toggle between ACTIVE and
 * legacy PS-Poll (LPS mode = 1) with a configurable awake-interval.
 * Deep-PS / SR (Selective Suspend) is deferred pending the chip-side
 * power-sequence table port.
 *
 * Wired to net80211 via `rtw88_ps_notify()`: front-ends call this from
 * their `iv_newstate` and `IEEE80211_F_PMGTON` handlers.
 */

#ifndef _RTW88_PS_H_
#define _RTW88_PS_H_

#include "rtw88.h"

/*
 * One-time attach.  Zeroes state, records macid, wires up chip
 * defaults (awake_interval, smart_ps).
 */
void	rtw88_ps_attach(struct rtw88_dev *rtwdev, uint8_t macid);

/*
 * Chip-detach cleanup: force ACTIVE if we're currently in LPS.
 */
void	rtw88_ps_detach(struct rtw88_dev *rtwdev);

/*
 * Enter shallow LPS.  Fires SET_PWR_MODE with mode=LPS + configured
 * awake_interval + smart_ps.  Idempotent.
 */
int	rtw88_lps_enter(struct rtw88_dev *rtwdev);

/*
 * Leave LPS.  Fires SET_PWR_MODE with mode=ACTIVE.  Idempotent.
 */
int	rtw88_lps_leave(struct rtw88_dev *rtwdev);

/*
 * Convenience: consult net80211 vap flags + drive enter/leave.
 * Callers pass the ic->ic_flags snapshot.
 */
int	rtw88_ps_notify(struct rtw88_dev *rtwdev, bool wants_ps);

#endif /* _RTW88_PS_H_ */
