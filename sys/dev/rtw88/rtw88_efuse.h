/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * rtw88 EFUSE subsystem: read the chip's OTP EFUSE map and expose the
 * per-chip parameters (MAC, xtal cap, RFE option, package type,
 * BT-coex presence) via `rtwdev->efuse`.
 *
 * Chip-specific offsets come from `rtwdev->chip->efuse_*`; the walker
 * itself is chip-agnostic (matches Linux `rtw_dump_logical_efuse_map`).
 */

#ifndef _RTW88_EFUSE_H_
#define _RTW88_EFUSE_H_

#include "rtw88.h"

/*
 * Populate `rtwdev->efuse` from chip OTP.
 * Returns 0 on success or an errno.  On success, subsequent readers
 * consult the cached map at `rtwdev->efuse`.
 */
int	rtw88_efuse_read(struct rtw88_dev *rtwdev);

/*
 * Standalone walker: expand a physical EFUSE map into the logical layout.
 * Exposed for unit-testing.  `phy` and `log` must both be at least
 * `rtwdev->chip->efuse_size` bytes.
 */
int	rtw88_efuse_logical_walk(struct rtw88_dev *rtwdev,
	    const uint8_t *phy, uint8_t *log);

#endif /* _RTW88_EFUSE_H_ */
