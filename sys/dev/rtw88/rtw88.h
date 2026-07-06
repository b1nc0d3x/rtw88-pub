/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * rtw88 core header: bus/chip-abstract handle used by every subsystem
 * (coex, ps, bf, led, regd, efuse, debug).  Each subsystem takes a
 * `struct rtw88_dev *` and calls through `hci_ops` for register access
 * and `chip` for per-silicon parameters.  A bus-specific attach (USB,
 * PCI, SDIO) supplies both tables + the `priv` back-pointer.
 *
 * Model: Linux drivers/net/wireless/realtek/rtw88/main.h `rtw_dev`.
 * Simplified for the FreeBSD port -- no ieee80211_hw / dev_alloc etc;
 * the caller (per-bus front-end) owns the net80211 side.
 */

#ifndef _RTW88_H_
#define _RTW88_H_

#include <sys/param.h>
#include <sys/types.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/sysctl.h>

struct rtw88_dev;
struct ieee80211com;

/* ------------------------------------------------------------------ */
/* Chip identifiers (Linux rtw_chip_id).  Prefixed to avoid collision */
/* with the USB front-end's probe-table index enum which uses the     */
/* short RTW88_CHIP_* names.                                          */
/* ------------------------------------------------------------------ */
enum rtw88_chip_ident {
	RTW88_CHIP_ID_8822B = 0,
	RTW88_CHIP_ID_8822C,
	RTW88_CHIP_ID_8723D,
	RTW88_CHIP_ID_8821C,
	RTW88_CHIP_ID_8703B,
	RTW88_CHIP_ID_8812A,
	RTW88_CHIP_ID_8814A,
	RTW88_CHIP_ID_8821A,
};

enum rtw88_hci_type {
	RTW88_HCI_TYPE_PCIE = 0,
	RTW88_HCI_TYPE_USB,
	RTW88_HCI_TYPE_SDIO,
};

/* ------------------------------------------------------------------ */
/* HCI (bus) abstraction: register + H2C.                             */
/*                                                                     */
/* Register addresses are u32 to accommodate the PCI MMIO window;      */
/* USB back-ends truncate to u16 internally.  All accessors return     */
/* an errno (0 on success).                                            */
/* ------------------------------------------------------------------ */
struct rtw88_hci_ops {
	int	(*read8)(struct rtw88_dev *, uint32_t addr, uint8_t *val);
	int	(*read16)(struct rtw88_dev *, uint32_t addr, uint16_t *val);
	int	(*read32)(struct rtw88_dev *, uint32_t addr, uint32_t *val);
	int	(*write8)(struct rtw88_dev *, uint32_t addr, uint8_t val);
	int	(*write16)(struct rtw88_dev *, uint32_t addr, uint16_t val);
	int	(*write32)(struct rtw88_dev *, uint32_t addr, uint32_t val);
	int	(*read_region)(struct rtw88_dev *, uint32_t addr,
		    void *buf, uint32_t len);
	int	(*write_region)(struct rtw88_dev *, uint32_t addr,
		    const void *buf, uint32_t len);
	/*
	 * h2c: dispatch a firmware command via the chip's H2C mailbox.
	 * `cmd` is the sub-command (byte 0 of the H2C packet); `msg` and
	 * `msg_ext` are the two 32-bit payload words the mailbox expects.
	 */
	int	(*h2c)(struct rtw88_dev *, uint8_t cmd,
		    uint32_t msg, uint32_t msg_ext);
};

/* ------------------------------------------------------------------ */
/* Chip capability table.                                              */
/*                                                                     */
/* One instance per silicon (8821C, 8822B, 8814A, ...).  Consumed by   */
/* every subsystem to keep chip-family logic out of subsystem code.    */
/* ------------------------------------------------------------------ */
struct rtw88_chip_info {
	enum rtw88_chip_ident chip_id;
	const char	*name;

	/* EFUSE layout -- consumed by rtw88_efuse.c */
	uint16_t	efuse_size;		/* physical map size */
	uint16_t	efuse_ptct_size;	/* protection region */
	uint16_t	efuse_mac_offset;	/* logical off of MAC[0] */
	uint16_t	efuse_xtal_k_offset;	/* logical off of xtal_k */
	uint16_t	efuse_rfe_offset;	/* logical off of rfe_option */

	/* Coex defaults -- consumed by rtw88_coex.c */
	uint8_t		coex_default_tdma;	/* tdma_case at init */
	uint32_t	coex_bt_wifi_ctrl_payload; /* H2C 0x69 payload */
	uint32_t	coex_tdma_type_payload;	/* H2C 0x60 payload */
	uint32_t	coex_rfe_btg_mask;	/* rfe_option BTG codes bitmap */

	/* LPS -- consumed by rtw88_ps.c */
	uint32_t	lps_deep_mode_supported; /* bitmap of PS_MODE_* */

	/* Beamforming -- consumed by rtw88_bf.c */
	bool		bf_su_supported;
	bool		bf_mu_supported;

	/* LED -- consumed by rtw88_led.c */
	bool		led_supported;
	uint32_t	led_gpio_mask;		/* which chip GPIOs drive LEDs */

	/* Regulatory -- consumed by rtw88_regd.c */
	uint8_t		max_power_index;	/* clamp for TX power */

	/* Chip-specific coex hooks (optional). */
	void	(*coex_cfg_init)(struct rtw88_dev *);
	void	(*coex_switch_rf_wonly)(struct rtw88_dev *);
};

/* ------------------------------------------------------------------ */
/* Per-subsystem state blocks embedded in rtw88_dev.                   */
/* ------------------------------------------------------------------ */

struct rtw88_efuse {
	bool		valid;			/* logical map populated */
	uint8_t		mac_addr[6];
	uint8_t		xtal_cap;		/* 6-bit */
	uint8_t		rfe_option;		/* 5-bit */
	uint8_t		pkg_type;		/* rfe bit 5 */
	bool		btcoex;			/* derived from rfe_option */
	uint8_t		raw[512];		/* cached logical map */
};

struct rtw88_coex_state {
	bool		enabled;		/* coex engine wired up */
	bool		bt_disabled;		/* wifi-only mode */
	uint8_t		current_tdma_case;
	uint32_t	last_bt_wifi_ctrl_payload;
	uint32_t	link_up_count;		/* diagnostic */
};

/* LPS mode -- Linux rtw88 ps.h. */
enum rtw88_lps_mode {
	RTW88_LPS_MODE_ACTIVE = 0,	/* no PS */
	RTW88_LPS_MODE_LEGACY = 1,	/* legacy PS-Poll */
	RTW88_LPS_MODE_UAPSD  = 2,	/* U-APSD */
	RTW88_LPS_MODE_WMMPS  = 3,	/* WMM-PS */
};

struct rtw88_lps_state {
	enum rtw88_lps_mode mode;
	bool		in_lps;			/* chip currently in PS */
	bool		wow;			/* WoWLAN gate */
	uint8_t		awake_interval;		/* beacons between wakes */
	uint8_t		smart_ps;		/* 0..2 */
	uint8_t		macid;
};

struct rtw88_bf_state {
	bool		attached;
	bool		su_active;
	bool		mu_active;
	uint8_t		peer_mac[6];
	uint16_t	paid;			/* partial AID */
	uint16_t	gid;			/* group ID */
};

struct rtw88_led_state {
	bool		attached;
	struct cdev	*cdev;			/* /dev/led/rtw88.NN */
	bool		on;
};

struct rtw88_regd_state {
	char		country[3];		/* NUL-terminated ISO2 */
	uint8_t		regd_2g;		/* per-chip regd code */
	uint8_t		regd_5g;
};

/* ------------------------------------------------------------------ */
/* rtw88_dev: bus/chip-abstract driver handle.                         */
/* ------------------------------------------------------------------ */
struct rtw88_dev {
	void		*priv;			/* bus-specific back-ptr */
	device_t	 dev;			/* newbus device */
	struct ieee80211com *ic;		/* net80211 handle */
	struct mtx	*mtx;			/* pointer to bus-owned mtx */
	enum rtw88_hci_type hci_type;
	const struct rtw88_hci_ops *hci_ops;
	const struct rtw88_chip_info *chip;

	struct rtw88_efuse	efuse;
	struct rtw88_coex_state	coex;
	struct rtw88_lps_state	lps;
	struct rtw88_bf_state	bf;
	struct rtw88_led_state	led;
	struct rtw88_regd_state	regd;

	struct sysctl_ctx_list	sysctl_ctx;
	struct sysctl_oid	*sysctl_tree;
};

/* ------------------------------------------------------------------ */
/* HCI wrapper macros -- terse call-site.                              */
/* ------------------------------------------------------------------ */
static inline int
rtw_read8(struct rtw88_dev *d, uint32_t a, uint8_t *v)
{
	return (d->hci_ops->read8(d, a, v));
}

static inline int
rtw_read16(struct rtw88_dev *d, uint32_t a, uint16_t *v)
{
	return (d->hci_ops->read16(d, a, v));
}

static inline int
rtw_read32(struct rtw88_dev *d, uint32_t a, uint32_t *v)
{
	return (d->hci_ops->read32(d, a, v));
}

static inline int
rtw_write8(struct rtw88_dev *d, uint32_t a, uint8_t v)
{
	return (d->hci_ops->write8(d, a, v));
}

static inline int
rtw_write16(struct rtw88_dev *d, uint32_t a, uint16_t v)
{
	return (d->hci_ops->write16(d, a, v));
}

static inline int
rtw_write32(struct rtw88_dev *d, uint32_t a, uint32_t v)
{
	return (d->hci_ops->write32(d, a, v));
}

static inline int
rtw_read_region(struct rtw88_dev *d, uint32_t a, void *b, uint32_t n)
{
	return (d->hci_ops->read_region(d, a, b, n));
}

static inline int
rtw_write_region(struct rtw88_dev *d, uint32_t a, const void *b, uint32_t n)
{
	return (d->hci_ops->write_region(d, a, b, n));
}

static inline int
rtw_h2c(struct rtw88_dev *d, uint8_t cmd, uint32_t msg, uint32_t msg_ext)
{
	return (d->hci_ops->h2c(d, cmd, msg, msg_ext));
}

/* Read-modify-write with mask + shift on a 32-bit register. */
static inline int
rtw_rmw32(struct rtw88_dev *d, uint32_t addr, uint32_t mask, uint32_t val)
{
	uint32_t v;
	int error;

	if ((error = rtw_read32(d, addr, &v)) != 0)
		return (error);
	v = (v & ~mask) | (val & mask);
	return (rtw_write32(d, addr, v));
}

/* ------------------------------------------------------------------ */
/* Diagnostic printf helpers.                                          */
/* ------------------------------------------------------------------ */
#define rtw_info(d, fmt, ...) \
	device_printf((d)->dev, fmt, ##__VA_ARGS__)
#define rtw_warn(d, fmt, ...) \
	device_printf((d)->dev, "WARN: " fmt, ##__VA_ARGS__)
#define rtw_err(d, fmt, ...) \
	device_printf((d)->dev, "ERR: " fmt, ##__VA_ARGS__)

#ifdef RTW88_DEBUG
#define rtw_dbg(d, fmt, ...) \
	device_printf((d)->dev, "DBG: " fmt, ##__VA_ARGS__)
#else
#define rtw_dbg(d, fmt, ...) do { (void)(d); } while (0)
#endif

/* Lock assertion helpers -- callers pin the mtx to the bus softc. */
#define RTW88_DEV_LOCK(d)		mtx_lock((d)->mtx)
#define RTW88_DEV_UNLOCK(d)		mtx_unlock((d)->mtx)
#define RTW88_DEV_ASSERT_LOCKED(d)	mtx_assert((d)->mtx, MA_OWNED)

#endif /* _RTW88_H_ */
