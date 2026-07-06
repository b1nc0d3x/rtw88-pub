/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * rtw88 regulatory-domain subsystem.
 *
 * FreeBSD net80211's regulatory-domain path (`net80211/
 * ieee80211_regdomain.[ch]`) supplies the per-country SKU lookup; the
 * user changes it via `ifconfig wlanN regdomain FCC`.  This subsystem:
 *   - keeps a per-country table of (regd_2g, regd_5g) values so any
 *     country that wpa_supplicant configures via `country=US`
 *     resolves to the correct chip-side codes (regd codes are
 *     chip-defined constants; the country -> code mapping is shared
 *     with Linux upstream because both drivers program the same
 *     hardware);
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
