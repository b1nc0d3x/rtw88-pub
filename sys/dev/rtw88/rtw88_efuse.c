/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * rtw88 EFUSE subsystem.  Ported from the monolithic if_rtw88_usb.c
 * `rtw88_efuse_*` block; converted to the bus/chip-abstract API so
 * future PCI / SDIO chip families reuse the same walker.
 *
 * Physical-byte protocol (8821C boot ROM, reused across the family):
 *   1. compose ctrl = (addr << 8) into REG_EFUSE_CTRL, clearing
 *      BIT_EF_FLAG to kick off a fresh read
 *   2. poll BIT_EF_FLAG to come back set (chip latched data)
 *   3. read the low byte = data
 *
 * The physical map is encoded as block-headers + word_en bitmaps;
 * rtw88_efuse_logical_walk expands that into a per-chip-sized logical
 * map, from which we extract MAC / xtal_k / rfe_option using the
 * chip-info offsets.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/malloc.h>

#include "rtw88.h"
#include "rtw88_efuse.h"
#include "rtw88_regs.h"

static MALLOC_DEFINE(M_RTW88_EFUSE, "rtw88_efuse", "rtw88 EFUSE scratch");

static int
rtw88_efuse_read_phys_byte(struct rtw88_dev *d, uint16_t addr, uint8_t *out)
{
	uint32_t val;
	int error, try;

	RTW88_DEV_ASSERT_LOCKED(d);

	if ((error = rtw_read32(d, REG_EFUSE_CTRL, &val)) != 0)
		return (error);
	val &= ~(BIT_MASK_EF_DATA | BITS_EF_ADDR);
	val |= ((uint32_t)(addr & BIT_MASK_EF_ADDR)) << BIT_SHIFT_EF_ADDR;
	val &= ~BIT_EF_FLAG;
	if ((error = rtw_write32(d, REG_EFUSE_CTRL, val)) != 0)
		return (error);

	for (try = 0; try < 1000; try++) {
		if ((error = rtw_read32(d, REG_EFUSE_CTRL, &val)) != 0)
			return (error);
		if ((val & BIT_EF_FLAG) != 0)
			break;
		DELAY(20);
	}
	if (try == 1000)
		return (ETIMEDOUT);

	*out = (uint8_t)(val & BIT_MASK_EF_DATA);
	return (0);
}

static int
rtw88_efuse_read_phys_map(struct rtw88_dev *d, uint8_t *phy)
{
	uint32_t ldo;
	uint8_t ldo_pwr;
	uint16_t addr;
	int error;

	RTW88_DEV_ASSERT_LOCKED(d);

	/* switch_efuse_bank: BIT_MASK_EFUSE_BANK_SEL = 0 (WIFI bank). */
	if ((error = rtw_read32(d, REG_LDO_EFUSE_CTRL, &ldo)) != 0)
		return (error);
	ldo &= ~BIT_MASK_EFUSE_BANK_SEL;
	if ((error = rtw_write32(d, REG_LDO_EFUSE_CTRL, ldo)) != 0)
		return (error);

	/* cfg_ldo25(false): clear bit 7 of REG_LDO_EFUSE_CTRL+3. */
	if ((error = rtw_read8(d, REG_LDO_EFUSE_CTRL + 3, &ldo_pwr)) != 0)
		return (error);
	ldo_pwr &= ~(1U << 7);
	if ((error = rtw_write8(d, REG_LDO_EFUSE_CTRL + 3, ldo_pwr)) != 0)
		return (error);

	for (addr = 0; addr < d->chip->efuse_size; addr++) {
		error = rtw88_efuse_read_phys_byte(d, addr, &phy[addr]);
		if (error != 0) {
			rtw_err(d, "efuse phys read off=%u: %d\n", addr, error);
			return (error);
		}
	}
	return (0);
}

/*
 * Walk a physical EFUSE map into the logical layout.  Header format:
 *
 *  1-byte:  bits 7:4 = block[3:0], bits 3:0 = word_en
 *  2-byte:  hdr1 bits 7:5 = block[2:0], bits 4:0 = 0xf
 *           hdr2 bits 7:4 = block[6:3], bits 3:0 = word_en
 *  end:     hdr1 == 0xff, OR (hdr1 & 0x1f) == 0xf && hdr2 == 0xff
 *
 * Each word_en bit 0..3 governs one 2-byte word; bit clear -> write,
 * set -> skip.  logical_idx = (block_idx << 3) + (word_idx << 1).
 */
int
rtw88_efuse_logical_walk(struct rtw88_dev *d, const uint8_t *phy, uint8_t *log)
{
	const uint16_t size = d->chip->efuse_size;
	const uint16_t protect = d->chip->efuse_ptct_size;
	uint16_t phy_idx = 0;
	uint8_t hdr1, hdr2, blk_idx, word_en;
	int i, log_idx;

	memset(log, 0xff, size);

	while (phy_idx < size - protect) {
		hdr1 = phy[phy_idx];
		hdr2 = phy[phy_idx + 1];
		if (hdr1 == 0xff)
			break;
		if (((hdr1 & 0x1f) == 0xf) && hdr2 == 0xff)
			break;

		if ((hdr1 & 0x1f) == 0xf) {
			blk_idx = ((hdr2 & 0xf0) >> 1) | ((hdr1 >> 5) & 0x07);
			word_en = hdr2 & 0xf;
			phy_idx += 2;
		} else {
			blk_idx = (hdr1 & 0xf0) >> 4;
			word_en = hdr1 & 0xf;
			phy_idx += 1;
		}

		for (i = 0; i < 4; i++) {
			if ((word_en & (1U << i)) != 0)
				continue;
			log_idx = ((int)blk_idx << 3) + (i << 1);
			if (phy_idx + 1 >= size - protect)
				return (EINVAL);
			if (log_idx + 1 >= size)
				return (EINVAL);
			log[log_idx] = phy[phy_idx];
			log[log_idx + 1] = phy[phy_idx + 1];
			phy_idx += 2;
		}
	}
	return (0);
}

int
rtw88_efuse_read(struct rtw88_dev *d)
{
	const struct rtw88_chip_info *chip = d->chip;
	uint8_t *phy, *log;
	int error, i;

	phy = malloc(chip->efuse_size, M_RTW88_EFUSE, M_WAITOK | M_ZERO);
	log = malloc(chip->efuse_size, M_RTW88_EFUSE, M_WAITOK | M_ZERO);

	RTW88_DEV_LOCK(d);
	error = rtw88_efuse_read_phys_map(d, phy);
	RTW88_DEV_UNLOCK(d);
	if (error != 0)
		goto out;

	error = rtw88_efuse_logical_walk(d, phy, log);
	if (error != 0) {
		rtw_err(d, "efuse logical walk failed: %d\n", error);
		goto out;
	}

	/*
	 * Cache the logical map + extract per-chip parameters via
	 * chip-info offsets.
	 */
	memcpy(d->efuse.raw, log, chip->efuse_size);
	memcpy(d->efuse.mac_addr, &log[chip->efuse_mac_offset], 6);
	d->efuse.xtal_cap = log[chip->efuse_xtal_k_offset] & 0x3F;
	d->efuse.rfe_option = log[chip->efuse_rfe_offset] & 0x1F;
	d->efuse.pkg_type = (log[chip->efuse_rfe_offset] & (1U << 5)) ? 1 : 0;
	d->efuse.btcoex = ((1U << d->efuse.rfe_option) &
	    chip->coex_rfe_btg_mask) != 0;
	d->efuse.valid = true;

	/*
	 * Program the chip's own-MAC filter register so unicast frames
	 * are actually accepted.  Kept here (rather than in the vif-add
	 * hook) because we only ever run one Port 0 STA vap.
	 */
	RTW88_DEV_LOCK(d);
	for (i = 0; i < 6; i++)
		(void)rtw_write8(d, REG_MACID0 + i, d->efuse.mac_addr[i]);
	RTW88_DEV_UNLOCK(d);

	rtw_info(d,
	    "efuse MAC: %02x:%02x:%02x:%02x:%02x:%02x  xtal_k=0x%02x"
	    "  rfe=0x%02x pkg=%u btcoex=%d\n",
	    d->efuse.mac_addr[0], d->efuse.mac_addr[1], d->efuse.mac_addr[2],
	    d->efuse.mac_addr[3], d->efuse.mac_addr[4], d->efuse.mac_addr[5],
	    d->efuse.xtal_cap, d->efuse.rfe_option, d->efuse.pkg_type,
	    d->efuse.btcoex);
out:
	free(log, M_RTW88_EFUSE);
	free(phy, M_RTW88_EFUSE);
	return (error);
}
