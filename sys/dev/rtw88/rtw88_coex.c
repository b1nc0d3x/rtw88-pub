/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * rtw88 BT/WiFi coexistence subsystem.  Port of Linux `coex.c` -- init
 * path + link-up refresh + LTE coex GNT indirect writes.  Full BT-side
 * state machine (bt_link_info + tdma tables for BT active/idle) is
 * deferred until we run this driver alongside an actual BT stack.
 *
 * All register access goes through `rtw_read/write*(rtwdev, ...)` so
 * the same code compiles for USB, PCI, SDIO front-ends and any
 * chip-family (8821C today, 8822B/C/8814A future).  Chip-family knobs
 * live in `rtwdev->chip` (coex_* fields + optional hooks).
 */

#include <sys/param.h>
#include <sys/systm.h>

#include "rtw88.h"
#include "rtw88_coex.h"
#include "rtw88_debug.h"
#include "rtw88_regs.h"

/* H2C sub-command IDs come from rtw88_regs.h. */

/* ------------------------------------------------------------------ */
/* LTE coex indirect register access.                                  */
/*                                                                     */
/* Port of reference util.c ltecoex_read_reg / ltecoex_reg_write.      */
/* All callers hold rtwdev->mtx.                                       */
/* ------------------------------------------------------------------ */
static int
rtw88_ltecoex_ready(struct rtw88_dev *d)
{
	uint32_t v;
	int i;

	for (i = 0; i < 100; i++) {
		if (rtw_read32(d, REG_LTECOEX_CTRL, &v) != 0)
			return (EIO);
		if ((v & BIT_LTECOEX_READY) != 0)
			return (0);
		DELAY(10);
	}
	return (ETIMEDOUT);
}

static int
rtw88_ltecoex_read_indirect(struct rtw88_dev *d, uint16_t offset,
    uint32_t *val)
{
	int error;

	if ((error = rtw88_ltecoex_ready(d)) != 0)
		return (error);
	if ((error = rtw_write32(d, REG_LTECOEX_CTRL,
	    LTECOEX_READ_OP | offset)) != 0)
		return (error);
	return (rtw_read32(d, REG_LTECOEX_RDATA, val));
}

static int
rtw88_ltecoex_write_indirect_raw(struct rtw88_dev *d, uint16_t offset,
    uint32_t val)
{
	int error;

	if ((error = rtw88_ltecoex_ready(d)) != 0)
		return (error);
	if ((error = rtw_write32(d, REG_LTECOEX_WDATA, val)) != 0)
		return (error);
	return (rtw_write32(d, REG_LTECOEX_CTRL,
	    LTECOEX_WRITE_OP | offset));
}

static int
rtw88_ltecoex_write_indirect(struct rtw88_dev *d, uint16_t offset,
    uint32_t mask, uint32_t val)
{
	uint32_t cur;
	int error, shift;

	if ((error = rtw88_ltecoex_read_indirect(d, offset, &cur)) != 0)
		return (error);
	shift = ffs(mask) - 1;
	cur = (cur & ~mask) | ((val << shift) & mask);
	return (rtw88_ltecoex_write_indirect_raw(d, offset, cur));
}

/* ------------------------------------------------------------------ */
/* GNT_WL / GNT_BT ratchet -- wifi-only "COEX_SET_ANT_WONLY" state.    */
/* ------------------------------------------------------------------ */
static void
rtw88_coex_set_gnt_wonly(struct rtw88_dev *d)
{

	RTW88_DEV_ASSERT_LOCKED(d);

	(void)rtw88_ltecoex_write_indirect(d, LTE_COEX_CTRL_OFFSET,
	    LTE_GNT_BT_HI_MASK, COEX_GNT_SET_SW_LOW);
	(void)rtw88_ltecoex_write_indirect(d, LTE_COEX_CTRL_OFFSET,
	    LTE_GNT_BT_LO_MASK, COEX_GNT_SET_SW_LOW);
	(void)rtw88_ltecoex_write_indirect(d, LTE_COEX_CTRL_OFFSET,
	    LTE_GNT_WL_HI_MASK, COEX_GNT_SET_SW_HIGH);
	(void)rtw88_ltecoex_write_indirect(d, LTE_COEX_CTRL_OFFSET,
	    LTE_GNT_WL_LO_MASK, COEX_GNT_SET_SW_HIGH);
}

/* ------------------------------------------------------------------ */
/* 2G BBSW antenna switch -- WLG path selected via REG_RFE_CTRL8.     */
/* ------------------------------------------------------------------ */
static void
rtw88_coex_set_ant_2g_bbsw(struct rtw88_dev *d)
{
	uint32_t v;

	RTW88_DEV_ASSERT_LOCKED(d);

	if (rtw_read32(d, REG_LED_CFG, &v) != 0)
		return;
	v &= ~BIT_DPDT_SEL_EN;
	v |= BIT_DPDT_WL_SEL;
	(void)rtw_write32(d, REG_LED_CFG, v);

	if (rtw_read32(d, REG_RFE_CTRL8, &v) != 0)
		return;
	v = (v & ~RFE_CTRL8_RFE_SEL89_MASK) | RFE_CTRL8_RFE_SEL89_DPDT;
	v = (v & ~RFE_CTRL8_R_RFE_SEL_15_MASK) |
	    RFE_CTRL8_R_RFE_SEL_15_BBSW_WLG;
	(void)rtw_write32(d, REG_RFE_CTRL8, v);
}

/* ------------------------------------------------------------------ */
/* Chip-family default coex_cfg_init (8821C-shaped).                   */
/*                                                                     */
/* Chips whose `chip->coex_cfg_init` is NULL fall back to this path.  */
/* The 8821C-family hook is currently identical; when 8822B/C land,   */
/* those chips can override via the chip_info table.                  */
/* ------------------------------------------------------------------ */
static void
rtw88_coex_cfg_init_default(struct rtw88_dev *d)
{
	uint32_t v32;
	uint16_t v16;
	uint8_t v8;

	RTW88_DEV_ASSERT_LOCKED(d);

	/* Enable beacon-function bit in BCN_CTRL. */
	if (rtw_read8(d, REG_BCN_CTRL, &v8) == 0) {
		v8 |= BIT_EN_BCN_FUNCTION;
		(void)rtw_write8(d, REG_BCN_CTRL, v8);
	}

	/* BT_TDMA_TIME sample rate = 0x05. */
	if (rtw_read8(d, REG_BT_TDMA_TIME, &v8) == 0) {
		v8 = (v8 & ~BIT_MASK_SAMPLE_RATE) | 0x05;
		(void)rtw_write8(d, REG_BT_TDMA_TIME, v8);
	}

	(void)rtw_write8(d, REG_BT_STAT_CTRL, BT_CNT_ENABLE);

	if (rtw_read32(d, REG_GPIO_MUXCFG, &v32) == 0) {
		v32 |= BIT_BT_PTA_EN | BIT_PO_BT_PTA_PINS;
		(void)rtw_write32(d, REG_GPIO_MUXCFG, v32);
	}

	if (rtw_read8(d, REG_QUEUE_CTRL, &v8) == 0) {
		v8 = (v8 | BIT_PTA_WL_TX_EN) & ~BIT_PTA_EDCCA_EN;
		(void)rtw_write8(d, REG_QUEUE_CTRL, v8);
	}

	if (rtw_read16(d, REG_BT_COEX_V2, &v16) == 0) {
		v16 |= BIT_GNT_BT_POLARITY;
		(void)rtw_write16(d, REG_BT_COEX_V2, v16);
	}

	if (rtw_read8(d, REG_BT_COEX_TABLE_H + 3, &v8) == 0) {
		v8 |= BCN_PRI_EN;
		(void)rtw_write8(d, REG_BT_COEX_TABLE_H + 3, v8);
	}

	/*
	 * Coex priority table case-0 (wifi-only).  Without these the PTA
	 * arbiter defaults to a state that can silently gate WiFi TX on
	 * chips with no BT firmware running.
	 */
	(void)rtw_write32(d, REG_BT_COEX_TABLE0, COEX_TABLE_WL_ONLY);
	(void)rtw_write32(d, REG_BT_COEX_TABLE1, COEX_TABLE_WL_ONLY);
	(void)rtw_write32(d, REG_BT_COEX_BRK_TABLE, COEX_BRK_TABLE_DEF);
}

/* ------------------------------------------------------------------ */
/* H2C helpers.                                                        */
/* ------------------------------------------------------------------ */
int
rtw88_coex_h2c_bt_wifi_control(struct rtw88_dev *d, uint8_t b1, uint8_t b2)
{
	uint32_t msg = (uint32_t)H2C_CMD_BT_WIFI_CONTROL |
	    ((uint32_t)b1 << 8) | ((uint32_t)b2 << 16);
	int rc = rtw_h2c(d, H2C_CMD_BT_WIFI_CONTROL, msg, 0);

	if (rc == 0) {
		d->coex.last_bt_wifi_ctrl_payload = msg;
		rtw88_debug_inc(d, RTW88_DBG_CNT_H2C_TX);
	}
	return (rc);
}

int
rtw88_coex_h2c_tdma_type(struct rtw88_dev *d, uint8_t tdma_case)
{
	uint32_t msg = (uint32_t)H2C_CMD_COEX_TDMA_TYPE |
	    ((uint32_t)tdma_case << 8);
	int rc = rtw_h2c(d, H2C_CMD_COEX_TDMA_TYPE, msg, 0);

	if (rc == 0) {
		d->coex.current_tdma_case = tdma_case;
		rtw88_debug_inc(d, RTW88_DBG_CNT_H2C_TX);
	}
	return (rc);
}

/* ------------------------------------------------------------------ */
/* Public API.                                                         */
/* ------------------------------------------------------------------ */
int
rtw88_coex_init(struct rtw88_dev *d)
{
	const struct rtw88_chip_info *chip = d->chip;
	uint8_t b1, b2;

	RTW88_DEV_LOCK(d);

	if (chip->coex_cfg_init != NULL)
		chip->coex_cfg_init(d);
	else
		rtw88_coex_cfg_init_default(d);

	rtw88_coex_set_gnt_wonly(d);
	rtw88_coex_set_ant_2g_bbsw(d);

	RTW88_DEV_UNLOCK(d);

	/* Fire the initial BT_WIFI_CONTROL + COEX_TDMA_TYPE pair. */
	b1 = (chip->coex_bt_wifi_ctrl_payload >> 8) & 0xff;
	b2 = (chip->coex_bt_wifi_ctrl_payload >> 16) & 0xff;
	(void)rtw88_coex_h2c_bt_wifi_control(d, b1, b2);
	(void)rtw88_coex_h2c_tdma_type(d, chip->coex_default_tdma);

	d->coex.enabled = true;
	d->coex.bt_disabled = true;	/* only wifi-only supported today */
	return (0);
}

int
rtw88_coex_link_up(struct rtw88_dev *d)
{
	const struct rtw88_chip_info *chip = d->chip;
	uint8_t b1 = (chip->coex_bt_wifi_ctrl_payload >> 8) & 0xff;
	uint8_t b2 = (chip->coex_bt_wifi_ctrl_payload >> 16) & 0xff;

	d->coex.link_up_count++;
	rtw88_debug_inc(d, RTW88_DBG_CNT_COEX_LINK_UP);
	(void)rtw88_coex_h2c_bt_wifi_control(d, b1, b2);
	(void)rtw88_coex_h2c_tdma_type(d, chip->coex_default_tdma);
	return (0);
}

void
rtw88_coex_link_down(struct rtw88_dev *d)
{
	/*
	 * Currently a no-op.  Reserved for BT-side coex teardown when
	 * the BT state machine is ported.
	 */
	(void)d;
}

void
rtw88_coex_prepare_tx(struct rtw88_dev *d)
{
	RTW88_DEV_ASSERT_LOCKED(d);
	rtw88_coex_set_gnt_wonly(d);
	rtw88_coex_set_ant_2g_bbsw(d);
}
