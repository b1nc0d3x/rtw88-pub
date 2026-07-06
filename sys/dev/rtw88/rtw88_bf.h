/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * rtw88 beamforming subsystem.
 *
 * Explicit beamforming (11n / 11ac).  Currently a thin scaffold: the
 * chip-side register writes are wired for the SU (single-user) case
 * on chips where `chip->bf_su_supported` is set, but nothing is
 * actively invoked from the driver's assoc path yet because 8821CU
 * only supports SU as bfee (client role) and the ASUS test AP does
 * not negotiate it.
 *
 * MU-MIMO (bf_mu_supported) requires the MU header IE + STAINFO fields
 * that aren't ported to net80211 yet -- deferred.
 */

#ifndef _RTW88_BF_H_
#define _RTW88_BF_H_

#include "rtw88.h"

/*
 * One-time attach.  Consults chip caps; wires initial disabled state.
 */
void	rtw88_bf_attach(struct rtw88_dev *rtwdev);

/*
 * Assoc-time: informed of the peer AP's beamforming capability IEs.
 *   `htcap` = HT beamforming capability field (802.11n TxBF cap)
 *   `vhtcap` = VHT beamforming capability field (802.11ac)
 * The subsystem decides SU vs MU vs off and programs the chip.
 */
void	rtw88_bf_assoc(struct rtw88_dev *rtwdev, const uint8_t peer_mac[6],
	    uint32_t htcap, uint32_t vhtcap, uint16_t aid);

/*
 * Disassoc-time: teardown BF state.  Safe to call unconditionally.
 */
void	rtw88_bf_disassoc(struct rtw88_dev *rtwdev);

#endif /* _RTW88_BF_H_ */
