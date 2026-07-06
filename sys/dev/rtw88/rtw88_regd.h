/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * rtw88 regulatory-domain subsystem.
 *
 * Linux's rtw88 defers regulatory to cfg80211's regd tables + a
 * mac80211 notifier that programs the chip's per-band regd code
 * before RF transmit power is enabled.
 *
 * FreeBSD net80211 has its own regulatory-domain path (`net80211/
 * ieee80211_regdomain.[ch]`) with a per-country SKU lookup, and
 * ifconfig can set the domain via `ifconfig wlanN regdomain FCC`.
 * This port:
 *   - keeps a per-country table of (regd_2g, regd_5g) values matching
 *     Linux's rtw_regd_table (extract), so any country that
 *     wpa_supplicant configures via `country=US` resolves to the same
 *     chip-side codes Linux would program;
 *   - exposes `rtw88_regd_apply()` for the front-end to call when the
 *     user changes country / at initial attach with EFUSE default.
 *
 * The chip-side register writes to program the regd code live in
 * rtw88_regd.c and use the chip-info `max_power_index` clamp.
 */

#ifndef _RTW88_REGD_H_
#define _RTW88_REGD_H_

#include "rtw88.h"

/* Regd codes (Linux rtw88 regd.h -- reduced set). */
enum rtw88_regd_code {
	RTW88_REGD_FCC		= 0,
	RTW88_REGD_ETSI		= 1,
	RTW88_REGD_MKK		= 2,
	RTW88_REGD_IC		= 3,
	RTW88_REGD_KCC		= 4,
	RTW88_REGD_ACMA		= 5,
	RTW88_REGD_CHILE	= 6,
	RTW88_REGD_UKRAINE	= 7,
	RTW88_REGD_MEXICO	= 8,
	RTW88_REGD_CN		= 9,
	RTW88_REGD_QATAR	= 10,
	RTW88_REGD_UK		= 11,
	RTW88_REGD_WW		= 15,	/* worldwide */
};

/*
 * Initialise regulatory state -- called at attach.  Populates
 * `rtwdev->regd` with the WW default; front-end can override via
 * `rtw88_regd_apply` once the country code is known.
 */
void	rtw88_regd_init(struct rtw88_dev *rtwdev);

/*
 * Apply an ISO-3166 2-letter country code.  Looks up the per-band
 * regd values from the internal table, updates `rtwdev->regd`, and
 * (if the chip's PA is active) reprograms the chip's power-index
 * table.
 */
int	rtw88_regd_apply(struct rtw88_dev *rtwdev, const char iso2[3]);

/*
 * Query current regd -- read-only accessor.
 */
const struct rtw88_regd_state *
	rtw88_regd_get(struct rtw88_dev *rtwdev);

#endif /* _RTW88_REGD_H_ */
