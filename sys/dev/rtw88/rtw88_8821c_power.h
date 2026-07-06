/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * 8821C MAC power-on sequence tables.  Layout + values lifted
 * verbatim from Linux mainline drivers/net/wireless/realtek/rtw88/
 * rtw8821c.c so the porting diff stays trivial.  Only the USB-path
 * entries fire at runtime (the parser filters on intf_mask); the
 * PCI/SDIO rows are kept for diff-fidelity with the upstream table.
 */

#ifndef _RTW88_8821C_POWER_H_
#define _RTW88_8821C_POWER_H_

#include <sys/types.h>
#include <sys/_null.h>

#define	RTW_PWR_POLLING_CNT	20000

#define	RTW_PWR_CMD_READ	0x00
#define	RTW_PWR_CMD_WRITE	0x01
#define	RTW_PWR_CMD_POLLING	0x02
#define	RTW_PWR_CMD_DELAY	0x03
#define	RTW_PWR_CMD_END		0x04

#define	RTW_PWR_ADDR_MAC	0x00
#define	RTW_PWR_ADDR_USB	0x01
#define	RTW_PWR_ADDR_PCIE	0x02
#define	RTW_PWR_ADDR_SDIO	0x03

#define	RTW_PWR_INTF_SDIO_MSK	(1U << 0)
#define	RTW_PWR_INTF_USB_MSK	(1U << 1)
#define	RTW_PWR_INTF_PCI_MSK	(1U << 2)
#define	RTW_PWR_INTF_ALL_MSK	(0xFU)

#define	RTW_PWR_CUT_ALL_MSK	0xFF

#define	RTW_PWR_DELAY_US	0
#define	RTW_PWR_DELAY_MS	1

/*
 * Linux uses bit-fields for base:4/cmd:4; in plain C we widen both to
 * uint8_t.  The parser masks the relevant nibble so the on-disk values
 * are identical.  Layout intentionally mirrors struct rtw_pwr_seq_cmd.
 */
struct rtw88_pwr_seq_cmd {
	uint16_t	offset;
	uint8_t		cut_mask;
	uint8_t		intf_mask;
	uint8_t		base;
	uint8_t		cmd;
	uint8_t		mask;
	uint8_t		value;
};

static const struct rtw88_pwr_seq_cmd trans_carddis_to_cardemu_8821c[] = {
	{0x0086, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_SDIO, RTW_PWR_CMD_WRITE,   (1U << 0), 0},
	{0x0086, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_SDIO, RTW_PWR_CMD_POLLING, (1U << 1), (1U << 1)},
	{0x004A, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_USB_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,   (1U << 0), 0},
	{0x0005, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,
	 ((1U << 3) | (1U << 4) | (1U << 7)), 0},
	{0x0300, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_PCI_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,   0xFF, 0},
	{0x0301, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_PCI_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,   0xFF, 0},
	{0xFFFF, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_ALL_MSK,
	 0,                  RTW_PWR_CMD_END,     0, 0},
};

static const struct rtw88_pwr_seq_cmd trans_cardemu_to_act_8821c[] = {
	{0x0020, RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_USB_MSK | RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,   (1U << 0), (1U << 0)},
	{0x0001, RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_USB_MSK | RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_DELAY,   1, RTW_PWR_DELAY_MS},
	{0x0000, RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_USB_MSK | RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,   (1U << 5), 0},
	{0x0005, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,
	 ((1U << 4) | (1U << 3) | (1U << 2)), 0},
	{0x0075, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_PCI_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,   (1U << 0), (1U << 0)},
	{0x0006, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_POLLING, (1U << 1), (1U << 1)},
	{0x0075, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_PCI_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,   (1U << 0), 0},
	{0x0006, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,   (1U << 0), (1U << 0)},
	{0x0005, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,   (1U << 7), 0},
	{0x0005, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,
	 ((1U << 4) | (1U << 3)), 0},
	{0x10C3, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_USB_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,   (1U << 0), (1U << 0)},
	{0x0005, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,   (1U << 0), (1U << 0)},
	{0x0005, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_POLLING, (1U << 0), 0},
	{0x0020, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,   (1U << 3), (1U << 3)},
	{0x0074, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_PCI_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,   (1U << 5), (1U << 5)},
	{0x0022, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_PCI_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,   (1U << 1), 0},
	{0x0062, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_PCI_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,
	 ((1U << 7) | (1U << 6) | (1U << 5)),
	 ((1U << 7) | (1U << 6) | (1U << 5))},
	{0x0061, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_PCI_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,
	 ((1U << 7) | (1U << 6) | (1U << 5)), 0},
	{0x007C, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,   (1U << 1), 0},
	{0xFFFF, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_ALL_MSK,
	 0,                  RTW_PWR_CMD_END,     0, 0},
};

static const struct rtw88_pwr_seq_cmd trans_act_to_cardemu_8821c[] = {
	{0x0093, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,   (1U << 3), 0},
	{0x001F, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,   0xFF, 0},
	{0x0049, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,   (1U << 1), 0},
	{0x0006, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,   (1U << 0), (1U << 0)},
	{0x0002, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,   (1U << 1), 0},
	{0x10C3, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_USB_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,   (1U << 0), 0},
	{0x0005, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,   (1U << 1), (1U << 1)},
	{0x0005, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_POLLING, (1U << 1), 0},
	{0x0020, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,   (1U << 3), 0},
	{0x0000, RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_USB_MSK | RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,   (1U << 5), (1U << 5)},
	{0xFFFF, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_ALL_MSK,
	 0,                  RTW_PWR_CMD_END,     0, 0},
};

static const struct rtw88_pwr_seq_cmd trans_cardemu_to_carddis_8821c[] = {
	{0x0007, RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_USB_MSK | RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,   0xFF, 0x20},
	{0x0067, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,   (1U << 5), 0},
	{0x004A, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_USB_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,   (1U << 0), 0},
	{0x0081, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,
	 ((1U << 7) | (1U << 6)), 0},
	{0x0005, RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_USB_MSK | RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,
	 ((1U << 3) | (1U << 4)), (1U << 3)},
	{0x0090, RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_USB_MSK | RTW_PWR_INTF_PCI_MSK,
	 RTW_PWR_ADDR_MAC,  RTW_PWR_CMD_WRITE,   (1U << 1), 0},
	{0xFFFF, RTW_PWR_CUT_ALL_MSK, RTW_PWR_INTF_ALL_MSK,
	 0,                  RTW_PWR_CMD_END,     0, 0},
};

static const struct rtw88_pwr_seq_cmd * const card_enable_flow_8821c[] = {
	trans_carddis_to_cardemu_8821c,
	trans_cardemu_to_act_8821c,
	NULL
};

static const struct rtw88_pwr_seq_cmd * const card_disable_flow_8821c[] = {
	trans_act_to_cardemu_8821c,
	trans_cardemu_to_carddis_8821c,
	NULL
};

#endif /* _RTW88_8821C_POWER_H_ */
