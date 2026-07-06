/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * rtw88 BT/WiFi coexistence subsystem.
 *
 * Even WiFi-only dongles (e.g. RTL8821CU with no BT chip on-board) need
 * to program the PTA arbiter and antenna switch: the chip's WL-side
 * TX/RX timing latches off these bits regardless of whether BT
 * firmware is running.
 *
 * This is the FreeBSD port of Linux `drivers/net/wireless/realtek/rtw88/
 * coex.c` -- currently only the init + link_up refresh paths are
 * ported; the full BT-side state machine (bt_link_info + tdma tables
 * for BT active/idle) is deferred.  Chip-specific hook points live in
 * `rtwdev->chip->coex_cfg_init` and `coex_switch_rf_wonly`.
 */

#ifndef _RTW88_COEX_H_
#define _RTW88_COEX_H_

#include "rtw88.h"

/*
 * Coex "TDMA case" IDs known to the shipping chips.  Case 0 = wifi-only
 * (no BT slots) -- the only one currently used by the port.
 */
#define	RTW88_COEX_TDMA_WIFI_ONLY	0

/*
 * Bring the coex engine up at chip power-on (post-fw-download).
 *  - programs PTA arbiter defaults
 *  - drives GNT_WL/GNT_BT to the wifi-only state
 *  - switches the antenna path to WLG via BBSW
 *  - fires the initial BT_WIFI_CONTROL + COEX_TDMA_TYPE H2C pair
 * Sets `rtwdev->coex.enabled = true`.
 */
int	rtw88_coex_init(struct rtw88_dev *rtwdev);

/*
 * LINK_UP refresh -- re-fires the BT_WIFI_CONTROL + COEX_TDMA_TYPE
 * pair.  Linux captures show this pair repeated at assoc + scan-start;
 * without it, the chip's PTA arbiter drifts and unicast RX stops.
 */
int	rtw88_coex_link_up(struct rtw88_dev *rtwdev);

/*
 * LINK_DOWN reset -- optional, currently a no-op.  Reserved for the
 * BT-side state machine port.
 */
void	rtw88_coex_link_down(struct rtw88_dev *rtwdev);

/*
 * TX-prepare refresh -- rerun by the per-frame prepare-tx hook so
 * GNT_WL is asserted before the modulator fires.  Cheap; safe on the
 * hot path.  Caller holds rtwdev->mtx.
 */
void	rtw88_coex_prepare_tx(struct rtw88_dev *rtwdev);

/*
 * Raw H2C helpers -- exported for the transitional monolith call sites.
 */
int	rtw88_coex_h2c_bt_wifi_control(struct rtw88_dev *rtwdev,
	    uint8_t b1, uint8_t b2);
int	rtw88_coex_h2c_tdma_type(struct rtw88_dev *rtwdev, uint8_t tdma_case);

#endif /* _RTW88_COEX_H_ */
