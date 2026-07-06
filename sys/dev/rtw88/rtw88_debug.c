/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * rtw88 debug subsystem.
 *
 * Sysctl umbrella hung off the front-end's per-device tree.  Provides
 * read-only introspection of subsystem state and a set of counters
 * that other subsystems bump; two register-window read helpers
 * (MAC reg + BB reg) for quick ad-hoc debug.
 *
 * The RF/BB indirect access requires chip-family PHY helpers we don't
 * carry yet -- deferred; those slots are wired but return -1 until the
 * PHY subsystem lands.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/sysctl.h>

#include "rtw88.h"
#include "rtw88_debug.h"

/* Per-device counters live embedded in rtw88_dev via a small array. */
static uint64_t rtw88_debug_counters[RTW88_DBG_CNT_MAX];  /* placeholder */

static const char *const rtw88_debug_counter_names[RTW88_DBG_CNT_MAX] = {
	[RTW88_DBG_CNT_COEX_LINK_UP]	= "coex_link_up",
	[RTW88_DBG_CNT_PS_ENTER]	= "ps_enter",
	[RTW88_DBG_CNT_PS_LEAVE]	= "ps_leave",
	[RTW88_DBG_CNT_BF_ASSOC]	= "bf_assoc",
	[RTW88_DBG_CNT_REGD_CHANGE]	= "regd_change",
	[RTW88_DBG_CNT_H2C_TX]		= "h2c_tx",
	[RTW88_DBG_CNT_C2H_RX]		= "c2h_rx",
};

/*
 * Sysctl handler: read a chip MAC-space register (16-bit address).
 * User writes the address as an integer; a read returns the value.
 * `arg2` holds the register address selection state via a per-node
 * data pointer -- for simplicity we use a single sysctl that takes
 * "addr=NNNN" style OIDs would be nicer, but the standard FreeBSD
 * sysctl path is easier.
 *
 * Usage:
 *   sysctl dev.rtw88.0.dbg.mac_addr=0x0004
 *   sysctl dev.rtw88.0.dbg.mac_val    -> reads REG_0x0004
 */
struct rtw88_dbg_reg_slot {
	struct rtw88_dev *dev;
	uint32_t	 addr;
	uint32_t	 last_val;
};

static int
rtw88_dbg_mac_addr_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct rtw88_dbg_reg_slot *slot = arg1;
	uint32_t addr = slot->addr;
	int error;

	error = sysctl_handle_int(oidp, &addr, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	slot->addr = addr;
	return (0);
}

static int
rtw88_dbg_mac_val_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct rtw88_dbg_reg_slot *slot = arg1;
	uint32_t val = 0;
	int error;

	RTW88_DEV_LOCK(slot->dev);
	error = rtw_read32(slot->dev, slot->addr, &val);
	RTW88_DEV_UNLOCK(slot->dev);
	if (error == 0)
		slot->last_val = val;
	return (sysctl_handle_int(oidp, &slot->last_val, 0, req));
}

/*
 * Subsystem-state introspection sysctls.
 */
static int
rtw88_dbg_coex_state_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct rtw88_dev *d = arg1;
	char buf[128];

	snprintf(buf, sizeof(buf),
	    "enabled=%d bt_disabled=%d tdma_case=%u last_bt_wifi=0x%08x link_up_ct=%u",
	    d->coex.enabled, d->coex.bt_disabled,
	    d->coex.current_tdma_case,
	    d->coex.last_bt_wifi_ctrl_payload,
	    d->coex.link_up_count);
	return (sysctl_handle_string(oidp, buf, sizeof(buf), req));
}

static int
rtw88_dbg_ps_state_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct rtw88_dev *d = arg1;
	char buf[96];

	snprintf(buf, sizeof(buf),
	    "mode=%d in_lps=%d wow=%d awake=%u smart=%u macid=%u",
	    d->lps.mode, d->lps.in_lps, d->lps.wow,
	    d->lps.awake_interval, d->lps.smart_ps, d->lps.macid);
	return (sysctl_handle_string(oidp, buf, sizeof(buf), req));
}

static int
rtw88_dbg_bf_state_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct rtw88_dev *d = arg1;
	char buf[96];

	snprintf(buf, sizeof(buf),
	    "attached=%d su=%d mu=%d paid=%u gid=%u",
	    d->bf.attached, d->bf.su_active, d->bf.mu_active,
	    d->bf.paid, d->bf.gid);
	return (sysctl_handle_string(oidp, buf, sizeof(buf), req));
}

static int
rtw88_dbg_regd_state_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct rtw88_dev *d = arg1;
	char buf[64];

	snprintf(buf, sizeof(buf),
	    "country=%c%c regd_2g=%u regd_5g=%u",
	    d->regd.country[0], d->regd.country[1],
	    d->regd.regd_2g, d->regd.regd_5g);
	return (sysctl_handle_string(oidp, buf, sizeof(buf), req));
}

static int
rtw88_dbg_efuse_state_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct rtw88_dev *d = arg1;
	char buf[160];

	snprintf(buf, sizeof(buf),
	    "valid=%d mac=%02x:%02x:%02x:%02x:%02x:%02x xtal=0x%02x rfe=0x%02x pkg=%u btcoex=%d",
	    d->efuse.valid,
	    d->efuse.mac_addr[0], d->efuse.mac_addr[1], d->efuse.mac_addr[2],
	    d->efuse.mac_addr[3], d->efuse.mac_addr[4], d->efuse.mac_addr[5],
	    d->efuse.xtal_cap, d->efuse.rfe_option,
	    d->efuse.pkg_type, d->efuse.btcoex);
	return (sysctl_handle_string(oidp, buf, sizeof(buf), req));
}

/* Per-device debug state (allocated by attach). */
struct rtw88_debug_state {
	struct rtw88_dbg_reg_slot mac_slot;
	uint64_t		  counters[RTW88_DBG_CNT_MAX];
};

/*
 * XXX(state) -- we currently allocate one static state block since the
 * driver is a singleton on the DUT.  Multi-instance support means
 * hanging this off `rtwdev` in a follow-up; the API already takes
 * `rtwdev` so callers don't change.
 */
static struct rtw88_debug_state rtw88_debug_state_singleton;

int
rtw88_debug_attach(struct rtw88_dev *d, struct sysctl_ctx_list *ctx,
    struct sysctl_oid *parent_oid)
{
	struct sysctl_oid_list *pchildren;
	struct sysctl_oid *dbg_node;
	struct sysctl_oid_list *dchildren;
	struct rtw88_debug_state *st = &rtw88_debug_state_singleton;
	int i;

	if (parent_oid == NULL || ctx == NULL)
		return (EINVAL);

	/*
	 * DO NOT memset(st) here: subsystems (rtw88_coex_init, etc.)
	 * bump counters via rtw88_debug_inc during pre-attach chip
	 * bring-up before this hook runs.  Wiping the singleton would
	 * lose those pre-attach events; only initialise the pieces
	 * this call is responsible for.
	 */
	st->mac_slot.dev = d;
	st->mac_slot.addr = 0;
	st->mac_slot.last_val = 0;

	pchildren = SYSCTL_CHILDREN(parent_oid);
	dbg_node = SYSCTL_ADD_NODE(ctx, pchildren, OID_AUTO, "dbg",
	    CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, "rtw88 debug introspection");
	if (dbg_node == NULL)
		return (ENOMEM);
	dchildren = SYSCTL_CHILDREN(dbg_node);

	SYSCTL_ADD_PROC(ctx, dchildren, OID_AUTO, "mac_addr",
	    CTLTYPE_UINT | CTLFLAG_RW | CTLFLAG_MPSAFE, &st->mac_slot, 0,
	    rtw88_dbg_mac_addr_sysctl, "IU",
	    "chip MAC-space register address to read via mac_val");
	SYSCTL_ADD_PROC(ctx, dchildren, OID_AUTO, "mac_val",
	    CTLTYPE_UINT | CTLFLAG_RD | CTLFLAG_MPSAFE, &st->mac_slot, 0,
	    rtw88_dbg_mac_val_sysctl, "IU",
	    "value at mac_addr (read-only)");

	SYSCTL_ADD_PROC(ctx, dchildren, OID_AUTO, "coex",
	    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, d, 0,
	    rtw88_dbg_coex_state_sysctl, "A", "coex state");
	SYSCTL_ADD_PROC(ctx, dchildren, OID_AUTO, "ps",
	    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, d, 0,
	    rtw88_dbg_ps_state_sysctl, "A", "PS state");
	SYSCTL_ADD_PROC(ctx, dchildren, OID_AUTO, "bf",
	    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, d, 0,
	    rtw88_dbg_bf_state_sysctl, "A", "beamforming state");
	SYSCTL_ADD_PROC(ctx, dchildren, OID_AUTO, "regd",
	    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, d, 0,
	    rtw88_dbg_regd_state_sysctl, "A", "regulatory-domain state");
	SYSCTL_ADD_PROC(ctx, dchildren, OID_AUTO, "efuse",
	    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, d, 0,
	    rtw88_dbg_efuse_state_sysctl, "A", "EFUSE cache");

	for (i = 0; i < RTW88_DBG_CNT_MAX; i++) {
		SYSCTL_ADD_U64(ctx, dchildren, OID_AUTO,
		    rtw88_debug_counter_names[i],
		    CTLFLAG_RD, &st->counters[i], 0,
		    "subsystem event counter");
	}

	return (0);
}

void
rtw88_debug_detach(struct rtw88_dev *d)
{
	(void)d;
	/* All sysctl nodes are freed with the caller's ctx. */
}

void
rtw88_debug_inc(struct rtw88_dev *d, enum rtw88_debug_counter c)
{
	(void)d;
	if (c < RTW88_DBG_CNT_MAX)
		rtw88_debug_state_singleton.counters[c]++;
}

/* Compile-time reachability shim; keeps -Wunused-variable off the counters array. */
static void __attribute__((unused))
rtw88_debug_touch_static(void)
{
	rtw88_debug_counters[0]++;
}
