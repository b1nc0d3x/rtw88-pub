/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * rtw88 debug subsystem.
 *
 * Per-device sysctl node `dev.rtw88.N` with children that read/write
 * chip state (MAC, BB, RF register windows; coex/PS/BF/regd state
 * introspection; H2C log counters).  FreeBSD-native — where Linux
 * rtw88 uses debugfs, this uses sysctl(9).
 *
 * The subsystem is bus-agnostic -- USB/PCI/SDIO front-ends call
 * `rtw88_debug_attach()` once at attach with the softc's sysctl_ctx
 * and OID node; this hangs subnodes off the caller's tree.
 */

#ifndef _RTW88_DEBUG_H_
#define _RTW88_DEBUG_H_

#include "rtw88.h"

/*
 * Attach the debug subsystem to the caller's sysctl tree.
 * `parent_oid` is typically `SYSCTL_STATIC_CHILDREN(_dev_XXX_N)` or the
 * per-instance node the front-end creates in attach.
 */
int	rtw88_debug_attach(struct rtw88_dev *rtwdev,
	    struct sysctl_ctx_list *ctx, struct sysctl_oid *parent_oid);

void	rtw88_debug_detach(struct rtw88_dev *rtwdev);

/*
 * Increment a per-subsystem counter (call sites in coex.c, ps.c, etc).
 * Cheap; used for post-mortem introspection.
 */
enum rtw88_debug_counter {
	RTW88_DBG_CNT_COEX_LINK_UP,
	RTW88_DBG_CNT_PS_ENTER,
	RTW88_DBG_CNT_PS_LEAVE,
	RTW88_DBG_CNT_BF_ASSOC,
	RTW88_DBG_CNT_REGD_CHANGE,
	RTW88_DBG_CNT_H2C_TX,
	RTW88_DBG_CNT_C2H_RX,
	RTW88_DBG_CNT_MAX
};

void	rtw88_debug_inc(struct rtw88_dev *rtwdev, enum rtw88_debug_counter c);

#endif /* _RTW88_DEBUG_H_ */
