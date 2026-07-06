/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * rtw88 register layout for the 8821C family.  Offsets and bit names
 * are lifted verbatim from reference mainline
 * drivers/net/wireless/realtek/rtw88/reg.h so the porting diff stays
 * trivial.  Only the registers used by phase 2c (firmware download)
 * are pulled across; later phases add the rest as they grow scope.
 */

#ifndef _RTW88_REGS_H_
#define _RTW88_REGS_H_

#define	REG_SYS_FUNC_EN		0x0002
#define	  BIT_FEN_BB_GLB_RST	(1U << 1)
#define	  BIT_FEN_BB_RSTB	(1U << 0)
#define	  BIT_FEN_CPUEN		(1U << 10)	/* applied to REG+1 byte */

#define	REG_SYS_CLK_CTRL	0x0008
#define	  BIT_CPU_CLK_EN	(1U << 14)	/* applied to REG+1 byte */

#define	REG_RSV_CTRL		0x001C
#define	  BIT_WLMCU_IOIF		(1U << 8)	/* on REG+1 byte: bit 0 */

#define	REG_RF_CTRL		0x001F
#define	  BIT_RF_SDM_RSTB	(1U << 2)
#define	  BIT_RF_RSTB		(1U << 1)
#define	  BIT_RF_EN		(1U << 0)

/* phy_set_param: BB power-on + crystal cap registers. */
#define	REG_AFE_XTAL_CTRL	0x0024
#define	  BIT_MASK_XTAL_CAP	0x7E000000
#define	REG_AFE_PLL_CTRL	0x0028
#define	  BIT_MASK_PLL_CAP	0x0000007E
#define	REG_RXPSEL		0x0808
#define	  BIT_RX_PSEL_RST	((1U << 28) | (1U << 29))
#define	REG_CCK0_FAREPORT	0x0A2C

/* set_channel: BB + RF per-channel tuning regs. */
#define	REG_SYS_CTRL_BB		0x0000	/* shadowed at 8821C addr 0x0000 */
#define	  BIT_FEN_EN		(1U << 26)
#define	REG_CCK_CHECK		0x0454
#define	REG_RXCCAMSK		0x0814
#define	REG_CLKTRK		0x0860
#define	REG_ADCCLK		0x08AC
#define	REG_ADC160		0x08C4
#define	REG_RXSB		0x0A00
#define	REG_CCA_FLTR		0x0A20
#define	REG_TXSF2		0x0A24
#define	REG_TXSF6		0x0A28
#define	REG_ENTXCCK		0x0A80
#define	REG_ENRXCCA		0x0A84
#define	REG_TXFILTER		0x0AAC
#define	REG_TXSCALE_A		0x0C1C
#define	REG_TXDFIR		0x0C20
#define	REG_ACBB0		0x0948
#define	REG_ACBBRXFIR		0x094C
#define	REG_CHFIR		0x08F0
#define	REG_RFECTL_A		0x0CB8
#define	  B_BTG_SWITCH		(1U << 16)
#define	  B_CTRL_SWITCH		(1U << 18)
#define	  B_WL_SWITCH		((1U << 20) | (1U << 22))
#define	  B_WLG_SWITCH		(1U << 21)
#define	  B_WLA_SWITCH		(1U << 23)
#define	REG_DMEM_CTRL_BB	0x1080	/* alias for REG_CPU_DMEM_CON */
#define	  BIT_WL_RST		(1U << 16)

/* BTG / WLG band-switch payloads (8821C-specific). */
#define	BTG_LNA			0xFC84	/* into REG_ENTXCCK[15:0] */
#define	WLG_LNA			0x7532
#define	BTG_CCA			0x0E	/* into REG_ENRXCCA[23:16] */
#define	WLG_CCA			0x12

/* RF register addresses (consumed by the SIPI bus). */
#define	RF_REG_18		0x18
#define	RF_XTALX2		0xB8
#define	RF_LUTDBG		0xDF
#define	RF_LUTWA		0x33
#define	RF_LUTWD0		0x3F
#define	RF_LUTWE2		0xEE
#define	  RF18_BAND_MASK	((1U << 16) | (1U << 9) | (1U << 8))
#define	  RF18_BAND_2G		0
#define	  RF18_BAND_5G		((1U << 16) | (1U << 8))
#define	  RF18_CHANNEL_MASK	0xFF
#define	  RF18_RFSI_MASK	((1U << 18) | (1U << 17))
#define	  RF18_RFSI_GE		(1U << 17)
#define	  RF18_RFSI_GT		(1U << 18)
#define	  RF18_BW_MASK		((1U << 11) | (1U << 10))
#define	  RF18_BW_20M		((1U << 11) | (1U << 10))
#define	  RF18_BW_40M		(1U << 11)
#define	  RF18_BW_80M		(1U << 10)
#define	  RFREG_MASK		0xFFFFF

/* 8821C RF path-A direct read base.  RF reg N read = base + (N << 2). */
#define	RTW88_RF_PATH_A_BASE	0x2800

/*
 * Host -> firmware command mailboxes.  reference fw.c uses four 8-byte
 * boxes (HMEBOX[0..3] + HMEBOX[0..3]_EX); the chip raises bits in
 * REG_HMETFR while the box is busy.  Pure EP0 register writes -- this
 * is the only H2C path that doesn't require bulk-OUT TX to be alive.
 */
#define	REG_HMETFR		0x01CC
#define	  BIT_INT_BOX0		(1U << 0)
#define	  BIT_INT_BOX1		(1U << 1)
#define	  BIT_INT_BOX2		(1U << 2)
#define	  BIT_INT_BOX3		(1U << 3)
#define	REG_HMEBOX0		0x01D0
#define	REG_HMEBOX1		0x01D4
#define	REG_HMEBOX2		0x01D8
#define	REG_HMEBOX3		0x01DC
#define	REG_HMEBOX0_EX		0x01F0
#define	REG_HMEBOX1_EX		0x01F4
#define	REG_HMEBOX2_EX		0x01F8
#define	REG_HMEBOX3_EX		0x01FC

/*
 * TSF (Timing Synchronization Function) — chip-side free-running 1us
 * counter, 64-bit total.  Used as the firmware-alive heartbeat by the
 * watchdog in fwka_task: if TSFTR_L doesn't advance between consecutive
 * ticks, the chip's beacon-receive + scheduler are wedged.  Per-port;
 * port 0 (STA) is 0x0560 -- matches Linux rtw88 main.h.
 */
#define	REG_TSFTR		0x0560
#define	REG_TSFTR_H		0x0564

/*
 * 8-byte H2C command IDs we currently emit (mailbox path).  Names
 * match reference fw.h.  Anything that goes through the bulk-OUT H2C
 * packet path is gated on TX working and not used yet.
 */
#define	H2C_CMD_WL_CH_INFO		0x66
#define	H2C_CMD_COEX_TDMA_TYPE		0x60
#define	H2C_CMD_BT_WIFI_CONTROL		0x69
#define	H2C_CMD_MEDIA_STATUS_RPT	0x01
#define	H2C_CMD_SET_PWR_MODE		0x20
#define	H2C_CMD_RA_INFO			0x40
#define	H2C_CMD_RSSI_MONITOR		0x42
#define	H2C_CMD_WL_PHY_INFO		0x58

/*
 * RA_INFO rate-set selectors for 2.4 GHz STA on 8821C (1T1R, 20 MHz).
 * Mirror Linux main.h enum rtw_rate_id.  The chip's rate-adaptation
 * engine picks rates from the SET corresponding to the rate_id we
 * seed via RA_INFO H2C + W1 RATE_ID in every TX desc.  Pick by the
 * AP's negotiated wireless set (per Linux get_rate_id, main.c:1036):
 *
 *   AP advertises HT (ni_flags & IEEE80211_NODE_HT): BGN_20M_1SS
 *   AP is CCK+OFDM only (hostapd hw_mode=g w/o ieee80211n=1): BG
 *
 * Without HT-aware selection, the chip RAs into MCS 0..7 even
 * against a non-HT AP, the AP can't demodulate, and no ACK ever
 * fires — chip retries forever and the AP DEAUTHs us.  Confirmed
 * via rtwn0 monitor capture 2026-06-26
 * (project_rtw88_air_capture_HT_mismatch_2026_06_26.md).
 */
#define	RTW88_RATEID_BGN_20M_1SS	3	/* Linux main.h RTW_RATEID enum */
#define	RTW88_RATEID_GN_N1SS		5	/* 5 GHz: OFDM + HT MCS0-7 */
#define	RTW88_RATEID_BG			6	/* CCK + OFDM only */
#define	RTW88_RATEID_G			7	/* OFDM only (5 GHz, no HT) */
/*
 * RA masks — match Linux rtw88_8821cu observed 2026-07-01 in QEMU capture
 * (qemu-trace-x86/parity_capture_2026_07_01/DIVERGENCES.md).
 *
 * Linux constrains the initial RA mask to a sparse subset of legacy
 * rates + all HT MCS.  This forces the chip's rate-adaptation engine
 * to prefer HT MCS while keeping a couple of CCK/OFDM fallbacks for
 * beacon-decode compatibility.  A wider mask (0x000FFFFF = all rates)
 * lets the RA jump to any legacy rate the AP can't demodulate for
 * post-assoc data frames, which correlates with the M2-not-received
 * pattern seen on production HE APs.
 */
#define	RTW88_DEFAULT_RA_MASK_2G_BGN_1SS	0x000FF015U
				/* CCK 1M + 5.5M + OFDM 6M + HT MCS0-7 */
#define	RTW88_TIGHTEN_RA_MASK_2G_BGN_1SS	0x000F0000U
				/* HT MCS4-7 only — fired ~27ms after
				 * initial mask as a NO_UPDATE tighten */
#define	RTW88_DEFAULT_RA_MASK_5G_GN_1SS		0x000FF010U
				/* OFDM 6M + HT MCS0-7 (no CCK on 5 GHz) */
#define	RTW88_TIGHTEN_RA_MASK_5G_GN_1SS		0x000F0000U
				/* HT MCS4-7 only */
#define	RTW88_DEFAULT_RA_MASK_2G_BG		0x00000015U
				/* CCK 1M + 5.5M + OFDM 6M (no HT link) */
#define	RTW88_DEFAULT_RA_MASK_5G_G		0x00000010U
				/* OFDM 6M (no HT link, no CCK) */

/*
 * EDCA / WMM per-AC parameter registers.  Layout per Linux reg.h:
 *   bits 26:16 = TXOP_LMT (11-bit unit = 32us)
 *   bits 15:12 = CWMAX (ecw, 4 bits, value = log2(cw+1))
 *   bits 11:8  = CWMIN (ecw, 4 bits)
 *   bits  7:0  = AIFS (us)
 */
#define	REG_EDCA_VO_PARAM	0x0500
#define	REG_EDCA_VI_PARAM	0x0504
#define	REG_EDCA_BE_PARAM	0x0508
#define	REG_EDCA_BK_PARAM	0x050C

/*
 * Digital input-gain (DIG) register + false-alarm counters.  8821C
 * single-path-A.  rtw88_dig_tick() is our port of reference's periodic
 * rtw_phy_dig() loop -- the watchdog samples REG_FA_OFDM (plus CCK
 * when enabled), trends the count, and nudges REG_DIG_PATH_A within
 * coverage bounds.  Names diverge from upstream intentionally.
 */
#define	REG_DIG_PATH_A		0x0C50
#define	  DIG_PATH_A_MASK	0x7F
#define	REG_FAS			0x09A4		/* FA reset @ bit 17 */
#define	REG_FA_CCK		0x0A5C		/* 16-bit CCK FA count */
#define	REG_RXDESC		0x0A2C		/* reset toggle @ bit 15 */
#define	REG_CNTRST		0x0B58		/* global reset @ bit 0 */
#define	REG_FA_OFDM		0x0F48		/* 16-bit OFDM FA count */

/*
 * DIG thresholds and IGI bounds.  Values verbatim from reference phy.c
 * for the 8821C family (chip->dig_min = 0x1c).  PERF = associated;
 * CVRG = scanning / idle.  step[] is the IGI increment per FA bucket.
 */
#define	DIG_PERF_FA_TH_LOW		250
#define	DIG_PERF_FA_TH_HIGH		500
#define	DIG_PERF_FA_TH_EXTRA_HIGH	750
#define	DIG_PERF_MAX			0x5A
#define	DIG_PERF_MID			0x40
#define	DIG_CVRG_FA_TH_LOW		2000
#define	DIG_CVRG_FA_TH_HIGH		4000
#define	DIG_CVRG_FA_TH_EXTRA_HIGH	5000
#define	DIG_CVRG_MAX			0x2A
#define	DIG_CVRG_MID			0x26
#define	DIG_CVRG_MIN			0x1C
#define	DIG_RSSI_GAIN_OFFSET		15
#define	DIG_MIN_8821C			0x1C
#define	DIG_TICK_INTERVAL_MS		2000

/*
 * REG_SYS_FUNC_EN bit 6 is shared with the legacy "PCIe analog enable"
 * pin -- 8821C's phy_set_param toggles it on USB too because it gates
 * the on-die BB block.  Naming follows reference.
 */
#define	  BIT_FEN_PCIEA		(1U << 6)

#define	REG_EFUSE_CTRL		0x0030
#define	  BIT_EF_FLAG		(1U << 31)
#define	  BIT_SHIFT_EF_ADDR	8
#define	  BIT_MASK_EF_ADDR	0x3FF
#define	  BIT_MASK_EF_DATA	0xFF
#define	  BITS_EF_ADDR		(BIT_MASK_EF_ADDR << BIT_SHIFT_EF_ADDR)

#define	REG_LDO_EFUSE_CTRL	0x0034
#define	  BIT_MASK_EFUSE_BANK_SEL ((1U << 8) | (1U << 9))

#define	REG_GPIO_MUXCFG		0x0040
#define	  BIT_FSPI_EN		(1U << 19)
#define	  BIT_WLRFE_4_5_EN	(1U << 2)	/* on REG+3 byte */
#define	  BIT_BT_PTA_EN		(1U << 5)
#define	  BIT_PO_BT_PTA_PINS	(1U << 9)

#define	REG_AFE_CTRL1		0x0024
#define	  BIT_MAC_CLK_SEL	((1U << 20) | (1U << 21))
#define	REG_DATA_SC		0x0483
#define	REG_USTIME_TSF		0x055C
#define	REG_USTIME_EDCA		0x0638
#define	  MAC_CLK_SPEED		0x50	/* 80 = 0x50 */
#define	  BIT_RFMOD		((1U << 7) | (1U << 8))
#define	  BIT_CHECK_CCK_EN	(1U << 7)

#define	REG_LED_CFG		0x004C
/*
 * LED0/LED1 are wired to REG_LED_CFG bits.  Datasheet 8821C: LED0_SV
 * at bit 27, LED1_SV at bit 26 (active-low).  Note bit 26 is shared
 * with LNAON_SEL_EN on this chip; keep the LED path RMW-safe and
 * don't touch the coex-owned DPDT bits.
 */
#define	  BIT_LED0_SV		(1U << 27)
#define	  BIT_LNAON_SEL_EN	(1U << 26)
#define	  BIT_PAPE_SEL_EN	(1U << 25)
#define	  BIT_DPDT_WL_SEL	(1U << 24)
#define	  BIT_DPDT_SEL_EN	(1U << 23)

#define	REG_PAD_CTRL1		0x0064
#define	  BIT_PAPE_WLBT_SEL	(1U << 29)
#define	  BIT_LNAON_WLBT_SEL	(1U << 28)

/*
 * BT-coex antenna-switch path control for 8821c.  reference rtw88
 * rtw8821c_coex_cfg_ant_switch() pokes these to physically route
 * the antenna to WiFi 2G when assoc starts; without it BT-coex
 * leaves the antenna shared / BT-priority and the AP never sees
 * our AUTH_REQ at full power.  Definitions lifted verbatim from
 * reference drivers/net/wireless/realtek/rtw88/{reg.h,rtw8821c.h}.
 */
#define	REG_CTRL_TYPE			0x0067
#define	  BIT_CTRL_TYPE1		(1U << 5)
#define	  BIT_CTRL_TYPE2		(1U << 4)
#define	REG_RFE_CTRL8			0x0CB4
#define	  RFE_CTRL8_RFE_SEL89_MASK	0x000000FFU
#define	  RFE_CTRL8_RFE_SEL89_PTA	0x00000066U	/* PTA_CTRL_PIN */
#define	  RFE_CTRL8_RFE_SEL89_DPDT	0x00000077U	/* DPDT_CTRL_PIN */
#define	  RFE_CTRL8_R_RFE_SEL_15_MASK	0xF0000000U
#define	  RFE_CTRL8_R_RFE_SEL_15_PTA_2G	0x20000000U
/*
 * BBSW (BB-software) DPDT antenna-switch path for SWITCH_TO_WLG with no
 * polarity inverse.  Linux 8821C COEX_SWITCH_CTRL_BY_BBSW + WLG branch
 * computes regval = 0x2, written to bits 28-31 via BIT_MASK_R_RFE_SEL_15.
 */
#define	  RFE_CTRL8_R_RFE_SEL_15_BBSW_WLG	0x20000000U

/*
 * Score-board mailbox to BT firmware.  reference rtw_coex_write_scbd()
 * writes 0x8003 here (BT_INT_EN | ACTIVE | ONOFF) at assoc-start so
 * BT firmware knows WiFi is connecting and yields TDMA accordingly.
 */
#define	REG_WIFI_BT_INFO		0x00AA
#define	  WIFI_BT_INFO_SCBD_ACTIVE	0x0001U
#define	  WIFI_BT_INFO_SCBD_ONOFF	0x0002U
#define	  WIFI_BT_INFO_BT_INT_EN	0x8000U

/*
 * PTA / BT-coex chip-init registers reference rtw88
 * rtw8821c_coex_cfg_init() pokes at chip-power-on time.  These run
 * even on WiFi-only RTL8821CU dongles (no BT firmware) because the
 * chip's PTA arbitrator latches WL-side TX/RX timing from these bits.
 */
#define	REG_QUEUE_CTRL		0x04C6		/* byte */
#define	  BIT_PTA_WL_TX_EN	(1U << 4)
#define	  BIT_PTA_EDCCA_EN	(1U << 5)
#define	REG_BT_COEX_TABLE0	0x06C0	/* BT priority table */
#define	REG_BT_COEX_TABLE1	0x06C4	/* WL priority table */
#define	REG_BT_COEX_BRK_TABLE	0x06C8
#define	  COEX_TABLE_WL_ONLY	0x55555555U
#define	  COEX_BRK_TABLE_DEF	0xF0FFFFFFU
#define	REG_BT_COEX_TABLE_H	0x06CC
#define	  BIT_BCN_QUEUE		(1U << 7)	/* on REG+3 byte */
#define	  BCN_PRI_EN		BIT_BCN_QUEUE
#define	REG_BT_COEX_V2		0x0762		/* 16-bit */
#define	  BIT_GNT_BT_POLARITY	(1U << 12)
#define	REG_BT_STAT_CTRL	0x0778
#define	  BT_CNT_ENABLE		0x01
#define	REG_BT_TDMA_TIME	0x0790
#define	  BIT_MASK_SAMPLE_RATE	0x1FU

/*
 * LTE-coex indirect-access window.  The chip exposes an on-die LTE
 * arbiter as a 256-byte register file accessed through three EP0
 * registers (REG_LTECOEX_CTRL/_WDATA/_RDATA).  ltecoex offset 0x38
 * holds the GNT_BT/GNT_WL HW/SW selector and value bits read by the
 * RF/PA gating path -- without explicit GNT_WL=SW_HIGH the chip's
 * modulator/PA stays in a sub-threshold state and emits malformed RF
 * (P28 PHY-error signature).  Linux walks this at every AUTH prelude
 * (see linux-auth-trace-2026_06_22.txt lines 5366-5399).
 */
#define	REG_LTECOEX_CTRL	0x1700	/* status/ctrl + offset/dir */
#define	  BIT_LTECOEX_READY	(1U << 29)
#define	  LTECOEX_READ_OP	0x800F0000U
#define	  LTECOEX_WRITE_OP	0xC00F0000U
#define	REG_LTECOEX_WDATA	0x1704
#define	REG_LTECOEX_RDATA	0x1708
#define	LTE_COEX_CTRL_OFFSET	0x0038	/* indirect offset for GNT_* bits */
#define	  LTE_GNT_BT_HI_MASK	0xC000U	/* bits 14:15  -- GNT_BT high  */
#define	  LTE_GNT_BT_LO_MASK	0x0C00U	/* bits 10:11  -- GNT_BT low   */
#define	  LTE_GNT_WL_HI_MASK	0x3000U	/* bits 12:13  -- GNT_WL high  */
#define	  LTE_GNT_WL_LO_MASK	0x0300U	/* bits  8:9   -- GNT_WL low   */
#define	  COEX_GNT_SET_HW_PTA	0x0
#define	  COEX_GNT_SET_SW_LOW	0x1
#define	  COEX_GNT_SET_SW_HIGH	0x3

#define	REG_WLRF1		0x00EC
#define	  BIT_WLRF1_BBRF_EN	((1U << 24) | (1U << 25) | (1U << 26))

#define	REG_SYS_STATUS1		0x00F4		/* +1 byte bit0 = PFM stuck */

#define	REG_CR_EXT		0x1100		/* +3 byte = 0x1103 */

#define	REG_MCUFW_CTRL		0x0080
#define	  BIT_MCUFWDL_EN	(1U << 0)
#define	  BIT_IMEM_DW_OK	(1U << 3)
#define	  BIT_IMEM_CHKSUM_OK	(1U << 4)
#define	  BIT_DMEM_DW_OK	(1U << 5)
#define	  BIT_DMEM_CHKSUM_OK	(1U << 6)
#define	  BIT_CPU_CLK_SEL	((1U << 12) | (1U << 13))
#define	  BIT_FW_DW_RDY		(1U << 14)
#define	  BIT_FW_INIT_RDY	(1U << 15)
#define	  BIT_BOOT_FSPI_EN	(1U << 20)
#define	  BIT_CHECK_SUM_OK	(BIT_IMEM_CHKSUM_OK | BIT_DMEM_CHKSUM_OK)
#define	  FW_READY		(BIT_FW_INIT_RDY | BIT_FW_DW_RDY |	\
				 BIT_IMEM_DW_OK | BIT_DMEM_DW_OK |	\
				 BIT_CHECK_SUM_OK)
#define	  FW_READY_MASK		(0xFFFFu & ~BIT_CPU_CLK_SEL)

#define	REG_CR			0x0100
#define	  BIT_HCI_TXDMA_EN	(1U << 0)
#define	  BIT_HCI_RXDMA_EN	(1U << 1)
#define	  BIT_TXDMA_EN		(1U << 2)
#define	  BIT_RXDMA_EN		(1U << 3)
#define	  BIT_PROTOCOL_EN	(1U << 4)
#define	  BIT_SCHEDULE_EN	(1U << 5)
#define	  BIT_MACTXEN		(1U << 6)
#define	  BIT_MACRXEN		(1U << 7)
#define	  BIT_ENSWBCN		(1U << 8)	/* applied to REG+1 byte */
/*
 * MAC_TRX_ENABLE -- canonical "all MAC TRX engines up" value loaded
 * into REG_CR after queue mapping.  Mirrors reference mac.c MAC_TRX_ENABLE.
 */
#define	  MAC_TRX_ENABLE	(BIT_HCI_TXDMA_EN | BIT_HCI_RXDMA_EN |	\
				 BIT_TXDMA_EN | BIT_RXDMA_EN |		\
				 BIT_PROTOCOL_EN | BIT_SCHEDULE_EN |	\
				 BIT_MACTXEN | BIT_MACRXEN)

#define	REG_TRXFF_BNDY		0x0114		/* +1 byte fix-up for 3081 */
#define	REG_RXFF_BNDY		0x011C

/*
 * Hardware security CAM (Linux sec.h).  Each CAM entry is 8 32-bit
 * dwords addressed via REG_CAMCMD; bit 16 = WRITE_ENABLE, bit 30 =
 * CLEAR, bit 31 = POLLING (chip clears when transaction finishes).
 * REG_CAMWRITE supplies the dword payload.  RTW_SEC_CONFIG (already
 * defined further down for the existing engine-enable path) gates
 * the engine + use-default-key behaviour.
 */
#define	REG_CAMCMD		0x0670
#define	REG_CAMWRITE		0x0674
#define	REG_CAMREAD		0x0678
#define	  BIT_CAM_WRITE_ENABLE	(1U << 16)
#define	  BIT_CAM_CLEAR		(1U << 30)
#define	  BIT_CAM_POLLING	(1U << 31)
#define	  RTW88_CAM_ENTRY_SHIFT	3	/* 8 dwords per entry */
#define	  RTW88_CAM_DEFAULT_KEYS 4	/* slots 0..3 = group keys */
#define	  RTW88_CAM_PAIRWISE_SLOT 4	/* first usable PTK slot */
#define	  RTW88_CAM_TYPE_NONE	0
#define	  RTW88_CAM_TYPE_WEP40	1
#define	  RTW88_CAM_TYPE_TKIP	2
#define	  RTW88_CAM_TYPE_AES	4
#define	  RTW88_CAM_TYPE_WEP104	5

#define	REG_FIFOPAGE_CTRL_2	0x0204
#define	  BIT_MASK_BCN_HEAD_1_V1 0x0FFF
#define	  BIT_BCN_VALID_V1	(1U << 15)

#define	REG_AUTO_LLT_V1		0x0208
#define	  BIT_AUTO_INIT_LLT_V1	(1U << 0)
#define	  BIT_MASK_BLK_DESC_NUM	0xF0	/* GENMASK(7,4) */

#define	REG_TXDMA_STATUS	0x0210
#define	  BTI_PAGE_OVF		(1U << 2)

#define	REG_TXDMA_OFFSET_CHK	0x020C
#define	  BIT_DROP_DATA_EN	(1U << 9)

#define	REG_RQPN_CTRL_2		0x022C
#define	  BIT_LD_RQPN		(1U << 31)

#define	REG_FIFOPAGE_INFO_1	0x0230
#define	REG_FIFOPAGE_INFO_2	0x0234
#define	REG_FIFOPAGE_INFO_3	0x0238
#define	REG_FIFOPAGE_INFO_4	0x023C
#define	REG_FIFOPAGE_INFO_5	0x0240

#define	REG_H2C_HEAD		0x0244
#define	REG_H2C_TAIL		0x0248
#define	REG_H2C_READ_ADDR	0x024C
#define	REG_H2C_INFO		0x0254

#define	REG_RXDMA_MODE		0x0290
#define	  BIT_DMA_MODE		(1U << 1)
#define	  BIT_DMA_BURST_CNT	(0x0C)	/* GENMASK(3,2) */
#define	  BIT_DMA_BURST_SIZE	(0x30)	/* GENMASK(5,4) */
#define	  BIT_DMA_BURST_SIZE_512 1	/* << 4 */

#define	REG_FWHW_TXQ_CTRL	0x0420
#define	  BIT_EN_WR_FREE_TAIL	(1U << 20)
#define	REG_BCNQ_BDNY_V1	0x0424
#define	REG_BCNQ1_BDNY_V1	0x0456

#define	REG_CPU_DMEM_CON	0x1080
#define	  BIT_WL_PLATFORM_RST	(1U << 16)	/* on 32-bit access */
#define	  BIT_DDMA_EN		(1U << 8)	/* on 32-bit access */

/*
 * Protocol / EDCA / beacon registers used by mac_init.
 * Offsets match reference mainline reg.h verbatim.
 */
#define	REG_INIRTS_RATE_SEL	0x0480
#define	REG_AMPDU_MAX_TIME_V1	0x0455
#define	REG_TX_HANG_CTRL	0x045E
#define	  BIT_EN_EOF_V1		(1U << 2)
#define	REG_PRECNT_CTRL		0x04E5
#define	  BIT_EN_PRECNT		(1U << 11)
#define	REG_PROT_MODE_CTRL	0x04C8
#define	REG_BAR_MODE_CTRL	0x04CC
#define	REG_FAST_EDCA_VOVI_SETTING 0x1448
#define	REG_FAST_EDCA_BEBK_SETTING 0x144C
#define	REG_TIMER0_SRC_SEL	0x05B4
#define	  BIT_TSFT_SEL_TIMER0	((1U << 4) | (1U << 5) | (1U << 6))
#define	REG_TXPAUSE		0x0522
#define	REG_SLOT		0x051B
#define	REG_PIFS		0x0512
#define	REG_SIFS		0x0514
#define	REG_EDCA_VO_PARAM	0x0500
#define	REG_EDCA_VI_PARAM	0x0504
#define	REG_RD_NAV_NXT		0x0544
#define	REG_RXTSF_OFFSET_CCK	0x055E
#define	REG_TBTT_PROHIBIT	0x0540
#define	REG_DRVERLYINT		0x0558
#define	REG_BCNDMATIM		0x0559
#define	REG_TX_PTCL_CTRL	0x0520	/* +1 byte */
#define	  BIT_SIFS_BK_EN	(1U << 12)

#define	REG_BCN_CTRL		0x0550
#define	  BIT_EN_BCN_FUNCTION	(1U << 3)
#define	  BIT_DIS_TSF_UDT	(1U << 4)

#define	REG_RCR			0x0608
#define	  BIT_APP_PHYSTS	(1U << 28)
#define	  BIT_APP_ICV		(1U << 29)
#define	  BIT_APP_MIC		(1U << 30)
#define	  BIT_APP_FCS		(1U << 31)
#define	  BIT_RCR_CBSSID_BCN	(1U << 7)
#define	  BIT_RCR_APWRMGT	(1U << 5)
#define	  BIT_RCR_AB		(1U << 3)
#define	  BIT_RCR_AM		(1U << 2)
#define	  BIT_RCR_APM		(1U << 1)
#define	  BIT_RCR_AAP		(1U << 0)
/* Pre-AUTH "joining BSS" RCR value Linux writes immediately before
 * AUTH_REQ TX (per linux-assoc.pcap REG 0x608 trace).  Differs from
 * our attach-time base WLAN_RCR_CFG = 0xE400220E in: BIT(7)
 * CBSSID_BCN set, BIT(14) set, BIT(20) set, BIT(28) APP_PHYSTS set,
 * BIT(9)/BIT(13) cleared.  8821C TX scheduler PA-gates
 * unicast-to-BSSID frames (incl. AUTH_REQ) when CBSSID_BCN is clear.
 */
#define	WLAN_RCR_AUTH_READY	0xF410408EU
/* Port 0 (default STA port) self-MAC + peer-BSSID, per reference rtw_vif_port[0]. */
#define	REG_MACID0		0x0610
#define	REG_BSSID0		0x0618
#define	REG_AID			0x06A8

/* REG_CR (0x0100) bits 16-17 = net_type for port 0; mirrors Linux rtw_net_type. */
#define	RTW88_NET_NO_LINK	0
#define	RTW88_NET_AD_HOC	1
#define	RTW88_NET_MGD_LINKED	2
#define	RTW88_NET_AP_MODE	3
#define	REG_RX_DRVINFO_SZ	0x060F
#define	REG_RX_PKT_LIMIT	0x060C
#define	REG_TCR			0x0604
#define	REG_RXFLTMAP0		0x06A0
#define	REG_RXFLTMAP2		0x06A4
#define	REG_ACKTO_CCK		0x0639
#define	REG_ACKTO		0x0640	/* OFDM ACK timeout */
/*
 * REG_RRSR (Response Rate Set Register) — bitmask of rates the chip's
 * MAC will use when generating responses (ACKs, CTSs).  Each bit
 * indexes into the DESC_RATE enum.  Linux's RRSR_INIT_2G = 0x15F enables
 * CCK 1/2/5.5/11 + OFDM 6/12/24.  Default (0) means no rates enabled,
 * which makes the chip's MAC silently fail to ACK incoming frames
 * because no valid response rate is configured.  Without ACKs the AP
 * thinks we missed every frame and retransmits hard.
 */
#define	REG_RRSR		0x0440
#define	  RRSR_INIT_2G		0x15FU	/* CCK 1/2/5.5/11 + OFDM 6/12/24 */
#define	REG_WMAC_TRXPTCL_CTL	0x0668
/*
 * P32: was 0x0010300eU until fixed against linux-rw-full-trace.txt.
 * The original constant was the byte-reversed transcription of the
 * actual register value reference rtw88 writes during set_channel_mac
 * (mac.c:42).  Both Linux and our chip are little-endian so the on-wire
 * order is the same; the bug was a hex-dump misreading by whoever
 * transcribed this from a usbmon capture.
 */
#define	  WMAC_TRXPTCL_CTL_INIT	0x0e301000U	/* reference 8821C BW20 value */
#define	REG_WMAC_TRXPTCL_CTL_H	0x066C
#define	RTW_SEC_CONFIG		0x0680
#define	  RTW_SEC_TX_UNI_USE_DK	(1U << 0)
#define	  RTW_SEC_RX_UNI_USE_DK	(1U << 1)
#define	  RTW_SEC_TX_DEC_EN	(1U << 2)
#define	  RTW_SEC_RX_DEC_EN	(1U << 3)
#define	  RTW_SEC_TX_BC_USE_DK	(1U << 6)
#define	  RTW_SEC_RX_BC_USE_DK	(1U << 7)
#define	REG_SND_PTCL_CTRL	0x0718
#define	  BIT_DIS_CHK_VHTSIGB_CRC (1U << 6)
#define	REG_WMAC_OPTION_FUNCTION 0x07D0

#define	REG_TXDMA_PQ_MAP	0x010C
#define	  RTW_DMA_MAPPING_HIGH	0
#define	  RTW_DMA_MAPPING_NORMAL 1

#define	REG_H2CQ_CSR		0x1330
#define	  BIT_H2CQ_FULL		(1U << 0)

#define	REG_DDMA_CH0SA		0x1200
#define	REG_DDMA_CH0DA		0x1204
#define	REG_DDMA_CH0CTRL	0x1208
#define	  BIT_DDMACH0_OWN	(1U << 31)
#define	  BIT_DDMACH0_CHKSUM_EN	(1U << 29)
#define	  BIT_DDMACH0_CHKSUM_STS (1U << 27)
#define	  BIT_DDMACH0_DDMA_MODE	(1U << 26)
#define	  BIT_DDMACH0_RESET_CHKSUM_STS (1U << 25)
#define	  BIT_DDMACH0_CHKSUM_CONT (1U << 24)
#define	  BIT_MASK_DDMACH0_DLEN	0x3FFFF

#define	REG_FW_DBG7		0x00D4
#define	  FW_KEY_MASK		0xFFFFFFFF
#define	  ILLEGAL_KEY_GROUP	0xFAAAAA00

/*
 * IDDMA OCP base for the TX buffer that the chip exposes to its
 * internal DMA controller.  reference constant OCPBASE_TXBUF_88XX.
 */
#define	OCPBASE_TXBUF_88XX	0x18780000

/*
 * TX-descriptor layout for 8821C is 48 bytes.  Only the fields the
 * firmware-download path actually populates are decoded here.
 */
#define	RTW88_TX_PKT_DESC_SZ	48

/* W0: bits 15:0 = TXPKTSIZE, bits 23:16 = OFFSET, bit 26 = LS */
#define	RTW88_TXD_W0_TXPKTSIZE_S	0
#define	RTW88_TXD_W0_OFFSET_S		16
#define	RTW88_TXD_W0_LS			(1U << 26)
/* W0: bit 24 = BMC (broadcast/multicast) */
#define	RTW88_TXD_W0_BMC		(1U << 24)
/* W0: bit 31 = DIS_QSEL_SEQ (disable HW QSEL seq # for this frame) */
#define	RTW88_TXD_W0_DIS_QSEL_SEQ	(1U << 31)
/* W1: bits 12:8 = QSEL */
#define	RTW88_TXD_W1_QSEL_S		8
#define	  TX_DESC_QSEL_BEACON		0x10
#define	  TX_DESC_QSEL_HIGH		0x11
#define	  TX_DESC_QSEL_MGMT		0x12
#define	  TX_DESC_QSEL_H2C		0x13
/* W1: bits 20:16 = RATE_ID (driver rate-table index for the chip) */
#define	RTW88_TXD_W1_RATE_ID_S		16
#define	  RTW_RATEID_B_20M		0x08	/* 802.11b rates only */
/* W1: bits 23:22 = SEC_TYPE -- chip-side encryption hint (Linux tx.h:33).
 * 0 = no encryption (sw or unencrypted), 1 = WEP/TKIP, 3 = AES/CCMP.
 */
#define	RTW88_TXD_W1_SEC_TYPE_S		22
#define	  RTW88_TXD_SEC_TYPE_NONE	0
#define	  RTW88_TXD_SEC_TYPE_WEPTKIP	1
#define	  RTW88_TXD_SEC_TYPE_AES	3
/* W1 extras (Linux tx.h:30-35) */
#define	RTW88_TXD_W1_MACID_S		0	/* 8 bits — per-STA chip CAM idx */
#define	RTW88_TXD_W1_PKT_OFFSET_S	24	/* 5 bits */
#define	RTW88_TXD_W1_MORE_DATA		(1U << 29)
/* W2 (Linux tx.h:36-39) */
#define	RTW88_TXD_W2_AGG_EN		(1U << 12)
#define	RTW88_TXD_W2_SPE_RPT		(1U << 19)
#define	RTW88_TXD_W2_AMPDU_DEN_S	20	/* 3 bits */
#define	RTW88_TXD_W2_BT_NULL		(1U << 23)
/* W3: bit 8 = USE_RATE (use the rate in W4 instead of auto) */
#define	RTW88_TXD_W3_USE_RATE		(1U << 8)
/* W3: bit 10 = DISDATAFB (disable data-rate fallback for this frame) */
#define	RTW88_TXD_W3_DISDATAFB		(1U << 10)
/* W3 extras (Linux tx.h:40-45) */
#define	RTW88_TXD_W3_HW_SSN_SEL_S	6	/* 2 bits — vif port id */
#define	RTW88_TXD_W3_USE_RTS		(1U << 12)
#define	RTW88_TXD_W3_NAVUSEHDR		(1U << 15)
#define	RTW88_TXD_W3_MAX_AGG_NUM_S	17	/* 5 bits */
/* W4: bits 6:0 = DATARATE (descriptor rate index: 0 = CCK 1 Mbps) */
#define	RTW88_TXD_W4_DATARATE_M		0x0000007FU
#define	RTW88_TXD_RATE_CCK_1M		0
#define	RTW88_TXD_RATE_OFDM_6M		4	/* DESC_RATE6M per Linux main.h */
#define	RTW88_TXD_W4_DATARATE_FB_LIMIT_S 8	/* 5 bits */
#define	RTW88_TXD_W4_RTSRATE_S		24	/* 5 bits */
/* W5 (Linux tx.h:49-53) */
#define	RTW88_TXD_W5_DATA_SHORT		(1U << 4)	/* short GI */
#define	RTW88_TXD_W5_DATA_BW_S		5	/* 2 bits: 0=20 1=40 2=80 */
#define	RTW88_TXD_W5_DATA_LDPC		(1U << 7)
#define	RTW88_TXD_W5_DATA_STBC_S	8	/* 2 bits */
#define	RTW88_TXD_W5_DATA_RTS_SHORT	(1U << 12)
/* W6 (Linux tx.h:54) */
#define	RTW88_TXD_W6_SW_DEFINE_M	0x00000FFFU	/* 12-bit serial# */
/* W8 (Linux tx.h:57) */
#define	RTW88_TXD_W8_EN_HWSEQ		(1U << 15)
/* W9 (Linux tx.h:58-60) */
#define	RTW88_TXD_W9_SW_SEQ_S		12	/* 12 bits mirroring frame seq */
#define	RTW88_TXD_W9_TIM_OFFSET_M	0x0000007FU
#define	RTW88_TXD_W9_TIM_EN		(1U << 7)

#endif /* _RTW88_REGS_H_ */
