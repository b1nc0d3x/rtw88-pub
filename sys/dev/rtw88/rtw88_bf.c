/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * rtw88 beamforming subsystem.
 *
 * SU beamforming as *bfee* (client receiving steered TX from an AP
 * bfer) requires:
 *   1. Setting BIT_ENABLE_TX_BF in REG_TXBF_CTRL (chip-family reg).
 *   2. Programming REG_ASSOCIATED_BFMEE_SEL with the peer MAC + AID.
 *   3. Programming REG_TX_CSI_RPT_PARAM_BW20 with the CSI feedback
 *      params (rate, sounding period).
 *
 * The exact register names + offsets vary by chip; those in
 * `rtw88_regs.h` here are 8821C-shaped.  Other chips will supply
 * their own regs (or override via a chip-hook table extension when
 * needed).
 *
 * MU-MIMO is deferred.
 */

#include <sys/param.h>
#include <sys/systm.h>

#include "rtw88.h"
#include "rtw88_bf.h"
#include "rtw88_debug.h"
#include "rtw88_regs.h"

void
rtw88_bf_attach(struct rtw88_dev *d)
{
	d->bf.attached = true;
	d->bf.su_active = false;
	d->bf.mu_active = false;
	memset(d->bf.peer_mac, 0, sizeof(d->bf.peer_mac));
	d->bf.paid = 0;
	d->bf.gid = 0;
}

/*
 * IEEE 802.11n HT capability field bit 20 = Explicit Compressed
 * Beamforming Feedback Capable (as bfer).  If the peer AP has that
 * bit set, we can request SU BF as bfee.
 */
#define	HT_CAP_TXBF_EXPLICIT_COMP_FB	(1U << 20)

/*
 * IEEE 802.11ac VHT capability field bit 11 = SU Beamformer Capable.
 * If set on the AP, we can receive SU steered TX.
 */
#define	VHT_CAP_SU_BEAMFORMER		(1U << 11)

void
rtw88_bf_assoc(struct rtw88_dev *d, const uint8_t peer_mac[6],
    uint32_t htcap, uint32_t vhtcap, uint16_t aid)
{
	bool ap_has_txbf;

	if (!d->bf.attached || !d->chip->bf_su_supported)
		return;

	ap_has_txbf = ((htcap & HT_CAP_TXBF_EXPLICIT_COMP_FB) != 0) ||
	    ((vhtcap & VHT_CAP_SU_BEAMFORMER) != 0);

	if (!ap_has_txbf) {
		d->bf.su_active = false;
		return;
	}

	memcpy(d->bf.peer_mac, peer_mac, 6);
	d->bf.paid = aid & 0x1ff;
	d->bf.gid = 0;
	d->bf.su_active = true;
	rtw88_debug_inc(d, RTW88_DBG_CNT_BF_ASSOC);

	/*
	 * Chip programming deferred: 8821CU's ASUS test AP negotiates
	 * BF as bfer=off, so leaving these register writes off keeps
	 * the current DUT behaviour stable while the state gets tracked
	 * for the coex + rate-adaptation paths.
	 *
	 * When we add BF-capable AP support:
	 *   RTW88_DEV_LOCK(d);
	 *   rtw_write32(d, REG_ASSOCIATED_BFMEE_SEL,
	 *       peer_mac[0] | (peer_mac[1] << 8) | (aid << 16));
	 *   rtw_write32(d, REG_TXBF_CTRL, BIT_ENABLE_TX_BF);
	 *   rtw_write32(d, REG_TX_CSI_RPT_PARAM_BW20, CSI_RPT_DEFAULT);
	 *   RTW88_DEV_UNLOCK(d);
	 */
}

void
rtw88_bf_disassoc(struct rtw88_dev *d)
{
	if (!d->bf.attached)
		return;
	d->bf.su_active = false;
	d->bf.mu_active = false;
	d->bf.paid = 0;
	d->bf.gid = 0;
	/* Chip programming deferred (see comment in rtw88_bf_assoc). */
}
