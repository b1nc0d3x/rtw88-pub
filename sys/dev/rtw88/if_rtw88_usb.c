/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * Realtek RTL8821CU / RTL8811CU USB 802.11ac STA driver.
 *
 * Bring-up: probe via the Realtek vendor request (bRequest = 0x05,
 * wValue = register addr, wIndex = 0; same as sys/dev/rtwn/usb/),
 * read SYS_CFG1/2 to confirm 8821C silicon, power on, download the
 * 3081 WCPU firmware, init MAC + PHY, attach net80211.  STA mode only;
 * single VAP; HT20 1T1R 2.4 + 5 GHz.
 *
 * Default crypto is SW CCMP (chip security engine off, net80211
 * encrypts in software).  hw.rtw88_usb.hw_ccmp=1 switches to HW CCMP
 * via the chip's CAM (experimental, requires DUT validation).
 *
 * Default aggregation is off.  hw.rtw88_usb.ampdu=1 advertises
 * IEEE80211_HTC_AMPDU and tags TX desc AGG_EN per BA session
 * (experimental).
 *
 * Operator counters under dev.rtw88_usb.N.stats.*; FW liveness
 * watchdog samples TSFTR in the fwka task.
 */

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/condvar.h>
#include <sys/conf.h>
#include <sys/endian.h>
#include <sys/firmware.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/systm.h>
#include <sys/taskqueue.h>

#include <sys/socket.h>
#include <sys/mbuf.h>
#include <sys/sysctl.h>

#include <net/if.h>
#include <net/if_media.h>
#include <net/ethernet.h>

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_regdomain.h>
#include <net80211/ieee80211_radiotap.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>

#include "rtw88_regs.h"
#include "rtw88_8821c_power.h"
#include "rtw88_8821c_mac_tbl.h"
#include "rtw88_8821c_agc_btg_type2.h"
#include "if_rtw88_usb_cdev.h"

/* Bus/chip-abstract core + subsystem APIs. */
#include "rtw88.h"
#include "rtw88_efuse.h"
#include "rtw88_coex.h"
#include "rtw88_ps.h"
#include "rtw88_bf.h"
#include "rtw88_led.h"
#include "rtw88_regd.h"
#include "rtw88_debug.h"

/*
 * Vendor / product numbers.  The canonical definitions live in
 * sys/dev/usb/usbdevs (USB_VENDOR_REALTEK = 0x0bda,
 * USB_PRODUCT_REALTEK_RTW8821CU = 0xc811), but pulling usbdevs.h into
 * an out-of-tree kmod build requires a regenerated obj-tree copy.
 * Raw IDs are used here; switching to the symbolic names is a
 * one-line change when this lands under /usr/src.
 *
 * Vendor IDs:
 *   0x0bda Realtek
 *   0x2001 D-Link
 *   0x7392 Edimax
 *   0x2c4e Mercusys
 *
 * Product table lifted verbatim from Linux upstream
 * drivers/net/wireless/realtek/rtw88/rtw8821cu.c
 * rtw_8821cu_id_table (15 entries) — every device that Linux's rtw88
 * binds to the rtw8821c_hw_spec.  Same silicon; same driver behaviour.
 * If a probe hit is actually a different chip variant the attach-time
 * SYS_CFG2 check will reject loudly before any chip-specific writes
 * fire.
 */
#define	RTW88_VENDOR_REALTEK	0x0bda
#define	RTW88_VENDOR_DLINK	0x2001
#define	RTW88_VENDOR_EDIMAX	0x7392
#define	RTW88_VENDOR_MERCUSYS	0x2c4e

/*
 * Per-table-entry chip spec dispatcher.
 *
 * STRUCT_USB_HOST_ID's third macro slot (USB_DRIVER_INFO) carries an
 * unsigned-long index that the framework copies into uaa->driver_info
 * on a successful match.  We use it as an index into rtw88_chip_specs[]
 * so attach knows which silicon family it's looking at without a
 * separate VID/PID re-decode.
 *
 * Today only 8821C is wired (all 15 probe-table entries point to it),
 * but the dispatcher means adding RTL8822B / RTL8814A / etc.  later
 * is a one-line addition per chip + a probe-table flag flip.
 */
enum {
	RTW88_CHIP_8821C = 0,
	RTW88_N_CHIP_SPECS
};

/*
 * Per-chip capability flags.  Chip-specific code paths key off these so
 * a future RTL8822B/C/8814A port is "add an enum index + spec entry"
 * not "grep through the driver for hardcoded 8821C assumptions".  Wired
 * gates:
 *   RTW88_CAP_HT20  -> ic_htcaps advertisement (HT IEs in assoc-req)
 *   RTW88_CAP_HT40  -> CHWIDTH40 / SHORTGI40 caps + BW40 BB/RF tune
 *   RTW88_CAP_VHT80 -> BW80 BB/RF tune (VHT ic-caps wiring pending)
 *   RTW88_CAP_2GHZ  -> getradiocaps 2.4 GHz channel emission + tune guard
 *   RTW88_CAP_5GHZ  -> getradiocaps 5 GHz channel emission + tune guard
 * Remaining bits are forward-declared scaffold; they document the
 * silicon's capabilities-of-record for future port work.
 */
#define	RTW88_CAP_HT20		(1U << 0)	/* 802.11n HT 20 MHz */
#define	RTW88_CAP_HT40		(1U << 1)	/* HT 40 MHz */
#define	RTW88_CAP_VHT80		(1U << 2)	/* 802.11ac VHT 80 MHz */
#define	RTW88_CAP_2GHZ		(1U << 3)	/* 2.4 GHz band */
#define	RTW88_CAP_5GHZ		(1U << 4)	/* 5 GHz band */
#define	RTW88_CAP_LPS		(1U << 5)	/* Low Power Save, deferred */
#define	RTW88_CAP_BTCOEX	(1U << 6)	/* BT coex engine, deferred */
#define	RTW88_CAP_BEAMFORMING	(1U << 7)	/* sender-side MU-MIMO */
#define	RTW88_CAP_AMSDU		(1U << 8)	/* A-MSDU bundling */

/*
 * Channel bandwidth enum.  Values match Linux's enum rtw_channel_width
 * (main.h) so on-air constants like RF18_BW_* and primary_ch_idx
 * encoding line up without translation.
 */
#define	RTW88_CHANNEL_WIDTH_20	0
#define	RTW88_CHANNEL_WIDTH_40	1
#define	RTW88_CHANNEL_WIDTH_80	2

/*
 * Primary subchannel position in the wider channel (Linux enum rtw_sc_idx
 * from main.h).  HT40+ (primary chan, secondary above) means the primary
 * 20 MHz is the LOWER half of the 40 MHz block; HT40- is the inverse.
 */
#define	RTW88_SC_DONT_CARE	0
#define	RTW88_SC_20_UPPER	1	/* primary = upper 20 (HT40-) */
#define	RTW88_SC_20_LOWER	2	/* primary = lower 20 (HT40+) */
#define	RTW88_SC_20_UPMOST	3
#define	RTW88_SC_20_LOWEST	4

struct rtw88_chip_spec {
	uint16_t	chip_id;	/* RTW88_FW_SIGNATURE_* */
	uint16_t	caps;		/* RTW88_CAP_* OR mask */
	uint8_t		n_tx_chains;
	uint8_t		n_rx_chains;
	const char     *name;
	const char     *fw_name;
};

static const struct rtw88_chip_spec rtw88_chip_specs[RTW88_N_CHIP_SPECS] = {
	[RTW88_CHIP_8821C] = {
		.chip_id	= 0x8821,
		/*
		 * Hardware actually supports more (VHT80, LPS, BT coex, BF)
		 * -- the caps mask reflects what's PORTED so chip-specific
		 * code can gate on the actual driver-side support, not the
		 * silicon datasheet.
		 */
		/*
		 * RTW88_CAP_HT40 + RTW88_CAP_VHT80 deliberately omitted —
		 * the BB+MAC+RF substrate is in place but DUT testing
		 * 2026-06-29 against TESTAP_OPEN ht/40+ showed the
		 * mid-assoc HT20->HT40 chip reprogram kills the RX path
		 * (newstate RUN -> INIT immediately).  Substrate stays
		 * (per-bw functions are still in the module), advertisement
		 * gates re-enable from here once the upgrade path is fixed.
		 */
		.caps		= RTW88_CAP_HT20 |
				  RTW88_CAP_2GHZ | RTW88_CAP_5GHZ |
				  RTW88_CAP_AMSDU | RTW88_CAP_LPS,
		.n_tx_chains	= 1,
		.n_rx_chains	= 1,
		.name		= "RTL8821CU/8811CU 11ac",
		.fw_name	= "rtw88-rtl8821cufw",
	},
};

/*
 * Realtek vendor control-pipe request.  Identical to the rtwn family.
 */
#define	RTW88_USB_REQ_REGS	0x05

/*
 * 8821C / 8822B / 8822C register power-domain quirk.  The chip splits
 * its register space into three sections:
 *   on     : 0x000-0x0FF, 0x1000-0x10FF -- always powered
 *   off    : 0x100-0xFFF, 0x1100-0xFDFF -- can be power-gated
 *   local  : 0xFE00+                    -- USB-specific
 * After any access to the "on" section, reference pings 1 byte at 0x4E0
 * (an "off" section register) with the same data to wake / keep the
 * off section alive.  Without this seal the next "off"-section access
 * (e.g. REG_DDMA_CH0CTRL at 0x1208) hangs on EP0.
 */
#define	RTW88_REG_SEC_PING_ADDR	0x04E0

/*
 * EP0 control-message timeouts.  reference uses 1000 ms for reads and
 * 500 ms for writes; we mirror those because the first off-section
 * access right after enabling MCUFWDL_EN takes hundreds of ms to
 * complete on USB.
 */
#define	RTW88_EP0_READ_TIMEOUT	1000
#define	RTW88_EP0_WRITE_TIMEOUT	500

/*
 * Chip ID register set.  Names + offsets lifted verbatim from
 * drivers/net/wireless/realtek/rtw88/reg.h.
 */
#define	REG_SYS_CFG1		0x00F0
#define	REG_SYS_CFG2		0x00FC

#define	SYS_CFG1_CHIP_VER_SHIFT	12
#define	SYS_CFG1_CHIP_VER_MASK	0xF
#define	SYS_CFG1_VENDOR_SHIFT	16
#define	SYS_CFG1_VENDOR_MASK	0xF
#define	SYS_CFG1_BIT_RTL_ID	(1U << 23)
#define	SYS_CFG1_BIT_LDO	(1U << 24)
#define	SYS_CFG1_BIT_RF_TYPE	(1U << 27)

/*
 * Firmware image name registered by sys/modules/rtw88-fw/
 * rtw88-rtl8821cufw.  Layout below mirrors reference mainline
 * rtw88/fw.h struct rtw_fw_hdr (FW_HDR_SIZE = 64) for the 3081 WCPU
 * format used by 8821C.  Fields are little-endian on the wire; this
 * driver decodes via le16/le32 readers.
 */
#define	RTW88_FW_HDR_SIZE	64

/*
 * Driver version string, surfaced via dev.rtw88_usb.N.version.  Bumped
 * with each tagged release so support requests can specify the exact
 * kmod build.  Also embedded in the kmod's MODULE_VERSION number so
 * `kldstat -v` and `pciconf -lv` -like tooling can match.
 */
#define	RTW88_DRIVER_VERSION	"1.1.2"

/*
 * Module-level tunables.  Each instance latches the value at attach time
 * so dynamically toggling at runtime won't half-flip an already-attached
 * device; the tunable can be set via loader.conf or `sysctl hw.rtw88_usb.*`
 * before kldload.
 *
 *   hw_ccmp  0 = (default) advertise AES_CCM as a SW-only cipher and leave
 *               the chip's HW security engine off (REG_SEC_CONFIG=0).
 *               Validated working data plane (see project memory
 *               rtw88_data_plane_WIN_2026_06_26).
 *           1 = advertise AES_CCM in ic_cryptocaps so net80211 expects HW
 *               encrypt/decrypt; program the HW security engine
 *               (REG_SEC_CONFIG=TX_DEC_EN|RX_DEC_EN|USE_DK); install PTK
 *               in the chip's CAM at iv_key_set time.  EXPERIMENTAL --
 *               requires DUT validation against the original wall bugs
 *               (broadcast dir 0x2 stamp + encrypted unicast silently
 *               dropped).
 */
static SYSCTL_NODE(_hw, OID_AUTO, rtw88_usb,
    CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, "rtw88 USB driver");
static int rtw88_hw_ccmp = 0;
SYSCTL_INT(_hw_rtw88_usb, OID_AUTO, hw_ccmp, CTLFLAG_RDTUN, &rtw88_hw_ccmp,
    0, "0=SW CCMP (default, validated); 1=HW CCMP via chip CAM "
    "(experimental, requires DUT validation)");

/*
 *   ampdu    0 = (default) advertise no AMPDU; net80211 won't open BA
 *               sessions; chip TXes single MPDUs.  Throughput capped at
 *               legacy rates for the link.
 *           1 = advertise IEEE80211_HTC_AMPDU; net80211 negotiates BA
 *               sessions via its default SW path; driver sets the chip's
 *               TX desc AGG_EN bit on QoS data frames to a peer with an
 *               active BA on that TID; RX path tags incoming QoS data
 *               with M_AMPDU so net80211's SW reorder buffer engages.
 *               EXPERIMENTAL.
 */
static int rtw88_ampdu = 0;
SYSCTL_INT(_hw_rtw88_usb, OID_AUTO, ampdu, CTLFLAG_RDTUN, &rtw88_ampdu,
    0, "0=no AMPDU (default); 1=advertise HTC_AMPDU and enable TX "
    "aggregation when net80211 brings up BA sessions (experimental)");

/*
 * Per-frame AMPDU TX desc parameters.  Tunable so an operator can dial
 * down for misbehaving APs without rebuilding.  All read at every data
 * TX submission, so no attach-time latch.
 *
 *   ampdu_max_num   1..31  default 31 = max sub-MPDUs in a burst
 *                          (matches Linux rtw88's default cap for 8821C)
 *   ampdu_density   0..7   default 4 = 8us min spacing
 *                          (matches our HTCAP_MPDUDENSITY advertisement)
 */
static int rtw88_ampdu_max_num = 31;
SYSCTL_INT(_hw_rtw88_usb, OID_AUTO, ampdu_max_num, CTLFLAG_RWTUN,
    &rtw88_ampdu_max_num, 0,
    "AMPDU max sub-MPDUs per burst (1..31, default 31)");
static int rtw88_ampdu_density = 4;
SYSCTL_INT(_hw_rtw88_usb, OID_AUTO, ampdu_density, CTLFLAG_RWTUN,
    &rtw88_ampdu_density, 0,
    "AMPDU sub-MPDU density code (0..7, default 4 = 8us)");

/*
 * TX queue depths, latched at attach.  Defaults are tuned for the
 * typical home / office throughput (~100 Mbps); high-throughput LAN
 * scenarios may want larger sc_tx_data_q.  Both are tunables so an
 * operator can size without rebuilding the kmod.
 */
static int rtw88_tx_data_qdepth = 256;
SYSCTL_INT(_hw_rtw88_usb, OID_AUTO, tx_data_qdepth, CTLFLAG_RDTUN,
    &rtw88_tx_data_qdepth, 0,
    "data TX queue depth (default 256; latched at attach)");
static int rtw88_tx_mgmt_qdepth = 64;
SYSCTL_INT(_hw_rtw88_usb, OID_AUTO, tx_mgmt_qdepth, CTLFLAG_RDTUN,
    &rtw88_tx_mgmt_qdepth, 0,
    "mgmt TX queue depth (default 64; latched at attach)");

/*
 * RPC bridge cdev (/dev/rtw88_usbN).  Lets a userland tool drive raw
 * EP0 register reads/writes via ioctl -- only useful for chip RE work
 * (it was the bring-up shim that lets Linux/QEMU drive the chip via
 * RPC).  Default 0 = don't create the cdev so production deployments
 * don't expose the chip-write path as an extra attack surface (the
 * cdev is root-only, but better not to have it at all if unused).
 */
static int rtw88_cdev_enable = 0;
SYSCTL_INT(_hw_rtw88_usb, OID_AUTO, cdev_enable, CTLFLAG_RDTUN,
    &rtw88_cdev_enable, 0,
    "0=no /dev/rtw88_usbN cdev (default); 1=create the RPC-bridge cdev");

/*
 * Bulk transfer slots.  rtw88 USB silicon presents 1 bulk-IN (RX) +
 * 4 bulk-OUT pipes; the OUTs map to the priority queues HI/NORMAL/LO
 * plus the H2C command channel.  usb_config templates leave the
 * endpoint address as UE_ADDR_ANY and let usbdi(9) match by direction
 * + type when the descriptor is walked.
 */
enum {
	RTW88_BULK_RX = 0,
	RTW88_BULK_TX_HI,
	RTW88_BULK_TX_NORMAL,
	RTW88_BULK_TX_LO,
	RTW88_BULK_TX_H2C,
	RTW88_N_TRANSFER
};

#define	RTW88_RX_BUFSZ		32768
#define	RTW88_TX_BUFSZ		16384
#define	RTW88_TX_TIMEOUT	5000	/* ms */

/*
 * Radiotap headers.  Layout matches rtwn(4) for compatibility with
 * the same userland tooling; both fields TX-side and RX-side use the
 * canonical IEEE80211_RADIOTAP_* field order.
 */
struct rtw88_rx_radiotap_header {
	struct ieee80211_radiotap_header wr_ihdr;
	uint64_t	wr_tsft;
	uint8_t		wr_flags;
	uint8_t		wr_rate;
	uint16_t	wr_chan_freq;
	uint16_t	wr_chan_flags;
	int8_t		wr_dbm_antsignal;
	int8_t		wr_dbm_antnoise;
} __packed __aligned(8);

#define	RTW88_RX_RADIOTAP_PRESENT				\
	((1 << IEEE80211_RADIOTAP_TSFT) |			\
	 (1 << IEEE80211_RADIOTAP_FLAGS) |			\
	 (1 << IEEE80211_RADIOTAP_RATE) |			\
	 (1 << IEEE80211_RADIOTAP_CHANNEL) |			\
	 (1 << IEEE80211_RADIOTAP_DBM_ANTSIGNAL) |		\
	 (1 << IEEE80211_RADIOTAP_DBM_ANTNOISE))

struct rtw88_tx_radiotap_header {
	struct ieee80211_radiotap_header wt_ihdr;
	uint8_t		wt_flags;
	uint8_t		wt_pad;
	uint16_t	wt_chan_freq;
	uint16_t	wt_chan_flags;
} __packed;

#define	RTW88_TX_RADIOTAP_PRESENT				\
	((1 << IEEE80211_RADIOTAP_FLAGS) |			\
	 (1 << IEEE80211_RADIOTAP_CHANNEL))

/*
 * 8821C RX packet descriptor layout (24 bytes, little-endian).
 * Bit fields named per reference mainline rx.h RTW_RX_DESC_W0_*.
 */
#define	RTW88_RX_DESC_SZ	24
#define	RTW88_RX_DESC_W0_PKT_LEN_M	0x00003FFF
#define	RTW88_RX_DESC_W0_CRC_ERR	(1U << 14)
#define	RTW88_RX_DESC_W0_ICV_ERR	(1U << 15)
#define	RTW88_RX_DESC_W0_DRVINFO_SZ_M	0x000F0000	/* in 8-byte units */
#define	RTW88_RX_DESC_W0_DRVINFO_SZ_S	16
#define	RTW88_RX_DESC_W0_ENC_TYPE_M	0x00700000
#define	RTW88_RX_DESC_W0_ENC_TYPE_S	20
#define	RTW88_RX_DESC_W0_SHIFT_M	0x03000000
#define	RTW88_RX_DESC_W0_SHIFT_S	24
#define	RTW88_RX_DESC_W0_PHYST		(1U << 26)
#define	RTW88_RX_DESC_W0_SWDEC		(1U << 27)
#define	RTW88_RX_DESC_W2_C2H		(1U << 28)
#define	RTW88_RX_DESC_W3_RX_RATE_M	0x0000007F
#define	RTW88_RX_DESC_W4_BW_M		0x00000030
#define	RTW88_RX_DESC_W4_BW_S		4
/* W5: full 32-bit TSF low (set by chip at frame RX time) */

/*
 * DESC_RATE* descriptor rate codes (8821C, mirrors Linux main.h).
 * 0x00..0x03 = CCK 1/2/5.5/11; 0x04..0x0B = OFDM 6..54;
 * 0x0C..0x23 = HT MCS0..MCS23; 0x2C+ = VHT MCS.
 */
#define	RTW88_DESC_RATE_CCK_MAX		0x03
#define	RTW88_DESC_RATE_OFDM_MIN	0x04
#define	RTW88_DESC_RATE_OFDM_MAX	0x0B
#define	RTW88_DESC_RATE_MCS_MIN		0x0C
#define	RTW88_DESC_RATE_MCS_MAX		0x23
#define	RTW88_DESC_RATE_VHT_MIN		0x2C

/* RX desc encryption type field (Linux enum rtw_rx_desc_enc). */
#define	RTW88_RX_DESC_ENC_NONE		0
#define	RTW88_RX_DESC_ENC_AES		4

/*
 * CCMP cipher header (IV) and trailer (MIC) sizes.  When HW CCMP RX is
 * enabled the chip decrypts the body but leaves the 8-byte IV (after the
 * MAC header) and the 8-byte MIC (at the tail) in the frame.  The driver
 * strips both and clears FC1_PROTECTED before handing up to net80211.
 */
#define	RTW88_CCMP_HDR_LEN		8
#define	RTW88_CCMP_MIC_LEN		8

/*
 * 8821C phy_status (drv_info) layout: 4 le32 words = 16 bytes when
 * drv_info_sz=2 (in 8-byte units).  Byte 0 carries the page ID in
 * its low nibble: page 0 = CCK, page 1 = OFDM/HT/VHT.
 */
#define	RTW88_PHY_STATUS_SIZE		16
#define	RTW88_PHY_STAT_PAGE(p)		((p)[0] & 0x0f)

/* Per-frame stats lifted out of the RX descriptor + drv_info. */
struct rtw88_rx_stat {
	uint16_t pkt_len;	/* over-the-air payload, FCS included */
	uint8_t  drv_info_sz;	/* bytes (Linux unit *8) */
	uint8_t  shift;
	uint8_t  rate;		/* DESC_RATE* */
	uint8_t  bw;		/* 0=20 1=40 2=80 (W4_BW raw) */
	uint8_t  enc_type;
	uint8_t  crc_err   : 1,
		 icv_err   : 1,
		 swdec     : 1,
		 phy_status: 1,
		 is_c2h    : 1,
		 decrypted : 1;
	int8_t   signal_dbm;	/* per-frame signal power, dBm */
	int8_t   rx_power;	/* path-A power, dBm */
	uint32_t tsf_low;
};

struct rtw88_usb_softc {
	struct ieee80211com	 sc_ic;
	device_t		 sc_dev;
	struct usb_device	*sc_udev;
	struct mtx		 sc_mtx;
	uint8_t			 sc_nendpoints;

	/*
	 * Bus/chip-abstract handle.  Consumed by subsystems (coex, ps, bf,
	 * led, regd, efuse, debug) via `struct rtw88_dev *`.  Filled in at
	 * attach: priv = this softc; hci_ops = &rtw88_usb_hci_ops; chip =
	 * &rtw88_8821c_chip_info (or per-chip variant when RTL8822B/C /
	 * 8814A ports land).
	 */
	struct rtw88_dev	 sc_rtwdev;

	/*
	 * Endpoints discovered at attach by walking the USB interface
	 * descriptor.  Replaces the hardcoded EP 0x05/0x06/0x08 in the
	 * usb_config[] template -- chip variants (RTL8811CU vs RTL8821CU
	 * vs OEM rebadges) may expose the same chip family at different
	 * physical USB endpoint addresses.
	 *
	 * sc_ep_bulk_out is sorted by EP address ascending; the lowest
	 * address is the "high" FIFO that takes mgmt + H2C in the chip's
	 * RQPN mapping for 8821C (Linux rtw88: out_ep[HIGH] = first).
	 */
	uint8_t			 sc_ep_bulk_in;		/* RX address */
	uint8_t			 sc_ep_bulk_out[4];	/* TX OUT addresses */
	uint8_t			 sc_n_bulk_out;
	bool			 sc_fw_running;
	bool			 sc_ic_attached;
	bool			 sc_running;
	bool			 sc_hw_ccmp;	/* latched from hw.rtw88_usb.hw_ccmp */
	bool			 sc_ampdu;	/* latched from hw.rtw88_usb.ampdu */
	/*
	 * Chip silicon spec resolved at attach from the matched probe
	 * table entry's driver_info field.  Today only rtw88_chip_specs
	 * has 8821C, but accessing chip metadata via this pointer instead
	 * of hardcoded constants makes the future RTL8822B/C / RTL8814A
	 * port a one-additional-spec change.
	 */
	const struct rtw88_chip_spec *sc_chip;
	bool			 sc_have_mac;
	uint8_t			 sc_mac[6];
	uint8_t			 sc_crystal_cap;	/* EFUSE xtal_k, 6 bits */
	uint8_t			 sc_rfe_option;		/* EFUSE rfe_option & 0x1f */
	uint8_t			 sc_pkg_type;		/* 0 or 1, from rfe_option BIT(5) */
	uint8_t			 sc_cut_version;	/* SYS_CFG1[15:12] */
	bool			 sc_rfe_btg;		/* derived from rfe_option */
	uint32_t		 sc_ch_param[3];	/* saved post-phy_set_param */
	uint8_t			 sc_cur_channel;
	uint8_t			 sc_cur_bw;	/* RTW88_CHANNEL_WIDTH_* */
	uint8_t			 sc_cur_primary_ch_idx;	/* HT40/80 primary */

	/*
	 * Periodic digital-input-gain (DIG) state.  sc_dig_co fires every
	 * DIG_TICK_INTERVAL_MS in softclock context and enqueues sc_dig_task
	 * onto sc_tq; the kthread runs rtw88_dig_task() which is where the
	 * USB register I/O happens.  Doing the I/O from a kthread (rather
	 * than the callout directly) is mandatory: rtw88_dig_tick() takes
	 * sc_mtx and walks helpers that drop it via usbd_do_request_flags()
	 * / usb_pause_mtx() during USB transfers, which softclock cannot
	 * tolerate.  Tracks a 4-deep ring of (IGI, false-alarm-count)
	 * samples plus the up/down trend bitmap upstream calls igi_bitmap;
	 * sc_dig_scan_save holds the IGI we sample at scan_start so
	 * scan_end can restore it.
	 */
	struct callout		 sc_dig_co;
	struct task		 sc_dig_task;
	struct task		 sc_prepare_tx_task;
	struct task		 sc_post_state_task;
	struct taskqueue	*sc_tq;

	/*
	 * FW keepalive: per Linux usbmon trace 2026-06-25, mac80211 fires
	 * SET_PWR_MODE H2C ~1/sec during sustained traffic. Without these,
	 * the chip's FW power-management state machine times out and
	 * unicast TX/RX degrades after a few seconds (Bug B).
	 * Periodic callout (sc_fwka_co) re-arms SET_PWR_MODE ACTIVE every
	 * second from sc_tq to keep FW state warm. Started from
	 * post_state_task RUN handler, stopped on LINK_DOWN.
	 */
	struct callout		 sc_fwka_co;
	struct task		 sc_fwka_task;
	bool			 sc_fwka_active;

	/*
	 * Second RA_INFO (Divergence #2).  Linux fires a follow-up
	 * H2C_CMD_RA_INFO ~2 s after LINK_UP with no_update=1 and the
	 * tighten mask 0x000F0000 (HT MCS 4-7 only).  Serves as a rate-
	 * lifecycle transition once the initial exchange settles.  We
	 * piggyback on the fwka 1 Hz tick: reset at LINK_UP, fire once
	 * when assoc_ms >= RTW88_RA_TIGHTEN_MS.
	 */
	bool			 sc_ra_tighten_fired;
	uint32_t		 sc_ra_tighten_mask;

	/*
	 * TX-report serial number counter.  Feeds W6.SW_DEFINE bits [7:2]
	 * when W2.SPE_RPT is set (currently only for EAPOL).  Linux ports
	 * this from atomic_inc_return(&tx_report->sn); we use uint8_t + wrap.
	 */
	uint8_t			 sc_tx_report_sn;

	/*
	 * EWMA of st.signal_dbm over RX frames addressed to us from the
	 * BSS we're associated to.  Stored as unsigned byte per Linux
	 * convention (= signal_dbm + 100, range 0..100).  Currently only
	 * exposed for diagnostics; RSSI_MONITOR / WL_PHY_INFO H2Cs that
	 * would consume it were removed after 2026-06-25 tests showed
	 * periodic-RSSI-feedback regressed link stability (FW makes a
	 * rate/PM decision based on our RSSI report that AP can't match).
	 */
	uint8_t			 sc_avg_rssi;
	bool			 sc_avg_rssi_valid;

	/*
	 * FW watchdog: tracks RX completion count across fwka ticks.  An
	 * associated AP sends beacons every TBTT (~100 ms) so a live chip
	 * has continuous RX activity; three consecutive identical reads
	 * (= ~3s with zero RX) -> the bulk-IN pipe or chip RX MAC is wedged.
	 *
	 * Earlier versions sampled REG_TSFTR (0x0560), but that's the
	 * chip's TSF synchronisation counter which only advances after
	 * beacon sync engages -- post-association but pre-sync produced
	 * false positives.  RX completions are a stronger liveness signal:
	 * if the AP is sending and we receive nothing, the radio is dead.
	 */
	uint64_t		 sc_last_rx_packets;
	uint8_t			 sc_fw_stall_ticks;
	bool			 sc_fw_stall_logged;

	/*
	 * Per-link RA rate-set selector.  Seeded from the AP's HT
	 * advertisement in post_state_task LINK_UP:
	 *   IEEE80211_NODE_HT set on iv_bss      -> BGN_20M_1SS (HT MCS)
	 *   not HT (hostapd hw_mode=g only etc.) -> BG (CCK + OFDM only)
	 * Consumed by every data-frame TX desc (W1 RATE_ID) and the
	 * RA_INFO H2C.  Without this the chip RAs into MCS rates that
	 * non-HT APs can't demodulate, so they never ACK and the chip
	 * retries forever -> AP DEAUTH within ~10 s of any traffic.
	 * Default BG keeps us safe on non-HT APs until the LINK_UP
	 * handler updates it.
	 */
	uint8_t			 sc_rate_id;

	/*
	 * H2C mailbox round-robin counter (0..3).  Per-instance so a
	 * multi-dongle setup doesn't cross-contaminate box selection.
	 */
	uint8_t			 sc_h2c_box_next;

	/*
	 * Bisect gate for the post-LINK_UP H2C burst.  Bitfield; see
	 * post_state_task for the per-bit meaning.  Userspace sets via
	 * dev.rtw88_usb.0.linkup_skip to isolate which H2C blocks the
	 * supplicant's M4 EAPOL TX.  Default 0 — run the full burst.
	 */
	uint16_t		 sc_linkup_skip;

	/*
	 * Tier 3c hypothesis test: re-run rtw88_phy_calibration (IQK) at
	 * LINK_UP, before the post-state H2C burst fires.  Linux does IQK
	 * once at mgd_prepare_tx (before AUTH-REQ, our SCAN->AUTH parity)
	 * and does NOT re-IQK at LINK_UP -- neither driver does this in
	 * the reference flow.  The hypothesis is that IQK results drift
	 * across the AUTH+ASSOC exchange enough that our M2 tx goes out
	 * with stale calibration and lands at the AP as noise.
	 *
	 * Default 0 (off).  Set dev.rtw88_usb.0.linkup_iqk=1 to enable.
	 * WARNING: enabling stalls the LINK_UP path by up to 6 s (300x20ms
	 * RF_DTXLOK poll); AP may retry M1 several times during the stall.
	 */
	uint8_t			 sc_linkup_iqk;

	/*
	 * reg_monitor.py --handshake-diff on 2026-07-02 Linux capture
	 * revealed Linux writes REG_FIFOPAGE_CTRL_2 = (pg_addr |
	 * BIT_BCN_VALID_V1) BETWEEN M1 rx and M2 tx.  Our rtw88_upload_
	 * rsvd_pages sets the same bit at LINK_UP burst (before M1
	 * arrives); the chip may auto-clear it while processing the rsvd
	 * page, so by M2 tx time the bit is stale.  Enabling this re-fires
	 * the write from rtw88_eapol_log's M1 rx branch.
	 *
	 * Default 0 (off).  Set dev.rtw88_usb.0.m1_bcn_refire=1 to enable.
	 */
	uint8_t			 sc_m1_bcn_refire;
	uint32_t		 sc_stat_m1_bcn_refire;	/* telemetry */

	/*
	 * DUT test 2026-07-02: our M2 wire bytes diverge from Linux at
	 * FC[0] (0x08 non-QoS Data vs 0x88 QoS-Data) and the QoS Ctrl
	 * field is absent.  net80211's ieee80211_output.c:1548 EXPLICITLY
	 * strips QoS-Data encoding from EAPOL frames as a legacy-AP
	 * workaround (comment at line 1532 acknowledges this may break
	 * modern APs that require QoS once WMM is negotiated).  Both APs
	 * we tested (WIFI, TESTAP_WPA2) advertise WMM + silently drop our
	 * non-QoS M2.
	 *
	 * Enabling this sysctl makes rtw88_transmit / rtw88_raw_xmit
	 * detect the specific "non-QoS Data + EAPOL SNAP" pattern in TX
	 * mbufs and rewrite to QoS-Data + TID=7 (VO) in place before
	 * crypto encap runs.  Matches Linux rtw88's OTA representation.
	 *
	 * Default 0 (off).  Set dev.rtw88_usb.0.eapol_qos_upgrade=1.
	 */
	uint8_t			 sc_eapol_qos_upgrade;
	uint32_t		 sc_stat_eapol_qos_upgrade;

	/*
	 * Diagnostic: dump the FULL M2/M4 tx mbuf as hex to dmesg so we
	 * can diff Key Data (RSN IE, PMKID, ANonce echo, MIC) against
	 * Linux M2 reference in linux_m_hex/M2.hex.  Default 0.
	 */
	uint8_t			 sc_eapol_fullhex;

	/*
	 * Deferred post-newstate work.  iv_newstate runs under net80211's
	 * ic_comlock (a non-sleepable mtx); USB I/O issued inline would
	 * sleep with comlock held and trip propagate_priority on the next
	 * concurrent setappie / ioctl (witness panic observed 2026-06-23).
	 * Inline newstate captures the work into these fields under
	 * RTW88_LOCK, then enqueues sc_post_state_task on sc_tq; the task
	 * does the actual USB writes outside comlock.  "Latest wins" is
	 * fine here -- net80211 doesn't ack on the wire; chip-side seeds
	 * just need to land before TX of association data starts flowing.
	 */
	uint8_t			 sc_post_state_ops;	/* bitmask */
#define	RTW88_POST_BSSID	0x01
#define	RTW88_POST_LINK_UP	0x02
#define	RTW88_POST_LINK_DOWN	0x04
	uint8_t			 sc_post_state_bssid[IEEE80211_ADDR_LEN];
	uint16_t		 sc_post_state_aid;	/* assoc ID, 11-bit */
	/*
	 * Per-link properties captured at newstate (where ic_comlock holds
	 * iv_bss live).  post_state_task must NOT touch vap->iv_bss because
	 * a racing LINK_DOWN can free it under our feet.
	 */
	bool			 sc_post_state_ht;	/* iv_bss HT-negotiated */
	bool			 sc_post_state_sgi;	/* SHORTGI20 joint cap */
	bool			 sc_post_state_5ghz;	/* iv_bss on 5 GHz */

	/*
	 * Deferred HW-CAM key install/clear queue.  net80211 fires
	 * iv_key_set / iv_key_delete with the ic_node mutex held
	 * (ieee80211_node_setuckey, ieee80211_node_delucastkey); the
	 * USB EP0 write that programs the CAM sleeps via
	 * usbd_do_request_flags, which trips witness with
	 * "sleeping thread holds rtw88_usb0_node" panic.  Capture the
	 * op here under RTW88_LOCK (a regular mtx, but USB drops it
	 * transparently around the cv_wait), enqueue sc_cam_task on
	 * sc_tq, and do the EP0 traffic from the task's kthread where
	 * no caller-held node lock exists.  Queue is small but bounded
	 * (worst case = 4-way handshake = PTK + up to 3 group keys
	 * installed back-to-back, plus per-key clear on rekey).
	 */
#define	RTW88_CAM_OP_INSTALL	1
#define	RTW88_CAM_OP_CLEAR	2
#define	RTW88_CAM_QUEUE_DEPTH	16
	struct rtw88_cam_op {
		uint8_t	op;
		uint8_t	slot;
		uint8_t	type;
		uint8_t	keyidx;
		bool	group;
		uint8_t	peer[IEEE80211_ADDR_LEN];
		uint8_t	key[16];
		uint8_t	keylen;
	}			 sc_cam_q[RTW88_CAM_QUEUE_DEPTH];
	uint8_t			 sc_cam_q_n;
	struct task		 sc_cam_task;

	/*
	 * Dedicated taskqueue for sc_cam_task.  Splits the CAM-install
	 * USB EP0 traffic off the shared sc_tq, where it would otherwise
	 * queue behind sc_fwka_task / sc_dig_task / sc_post_state_task
	 * (all of which can sit in usbd_do_request_flags for 5-15 ms).
	 * Without this, iv_key_set enqueue -> CAM-write latency grows
	 * past supplicant's M4 deadline; AP's M3 retransmit timer wins
	 * the race and restarts the 4-way handshake.  Adding a dedicated
	 * thread drops the worst case from ~15 ms to ~1 ms.
	 */
	struct taskqueue	*sc_cam_tq;

	/*
	 * PTK-install gating for outbound encrypted unicast.
	 *
	 * iv_key_set returns to net80211 with sc_cam_task only enqueued
	 * (USB writes happen ~5-15 ms later in the taskqueue kthread).
	 * supplicant then immediately TXs M4 -- a Protected unicast data
	 * frame to the AP -- which the chip's CCMP engine tries to look
	 * up in CAM slot 4.  Empty slot -> chip silently drops, AP keeps
	 * retransmitting M3, handshake stalls.  Observed 2026-06-24
	 * after the QoS-promote fix (c2f732a) opened M2 progression.
	 *
	 * Gate on sc_ptk_install_pending: set true in iv_key_set when a
	 * non-group install is enqueued, cleared by sc_cam_task as soon
	 * as the install completes.  rtw88_tx_enqueue_locked routes
	 * Protected unicast data frames to sc_tx_pending_q while the
	 * flag is set; sc_cam_task drains the pending queue into the
	 * normal data queue right after the install lands.
	 */
	bool			 sc_ptk_install_pending;
	struct mbufq		 sc_tx_pending_q;

	/*
	 * WPA2 4-way M4 pre-install encap-defer marker.  wpa_supplicant on
	 * FreeBSD sends M4 BEFORE installing the PTK; at M4 tx time
	 * ni->ni_ucastkey is still IEEE80211_CIPHER_NONE and
	 * ieee80211_crypto_encap returns NULL.
	 *
	 * Fix: mark the mbuf here and let it flow through tx_enqueue_locked's
	 * Protected+ptk_install_pending gate into sc_tx_pending_q.
	 * sc_cam_task's post-install drain runs ieee80211_crypto_encap on
	 * this specific mbuf before routing to the VO queue -- by which
	 * point ni->ni_ucastkey holds the PTK.  No callout, no separate
	 * drain task, no rcvif-cast surprises.  Single-slot: at most one
	 * pending-encap M4 per 4-way; subsequent M4s during the window fall
	 * back to plaintext.
	 */
	struct mbuf		*sc_m4_needs_encap;

	/*
	 * EAPOL timing instrumentation.  Set to `ticks` on SCAN -> AUTH
	 * entry in rtw88_newstate; every later EAPOL TX/RX, post_state_task
	 * milestone, and vap_key_set call prints (ticks - sc_assoc_ticks)
	 * in ms so we can audit the 4-way handshake latency budget against
	 * wpa_supplicant's ~1 s M2/M3 retry timer (origin of the startup
	 * RUN -> INIT bounces).
	 */
	int			 sc_assoc_ticks;
	bool			 sc_dig_active;
	uint8_t			 sc_dig_scan_save;
	uint8_t			 sc_igi_log[4];		/* idx 0 = most recent */
	uint16_t		 sc_fa_log[4];
	uint8_t			 sc_igi_trend_bits;
	uint16_t		 sc_pre_min_rssi;	/* tracked for future use */
	uint16_t		 sc_min_rssi;
	const struct firmware	*sc_tbl_fw;
	const uint8_t		*sc_tbl_blob;
	size_t			 sc_tbl_size;
	uint8_t			*sc_rx_stage;	/* RTW88_RX_BUFSZ scratch */
	uint64_t		 sc_rx_packets;	/* RX callbacks since attach */
	uint64_t		 sc_rx_frames;	/* mbufs handed to net80211 */

	/*
	 * Radiotap (monitor mode / promiscuous capture) headers.  Live
	 * regardless of monitor VAP presence so a pcap user on wlan0 in
	 * any mode sees per-frame PHY metadata.
	 */
	struct rtw88_rx_radiotap_header sc_rxtap;
	struct rtw88_tx_radiotap_header sc_txtap;

	/*
	 * Production stats counters, exposed under dev.rtw88_usb.0.stats.*
	 * so an operator can debug a wedged link without enabling tracing.
	 * Updated under RTW88_LOCK in driver-internal paths and atomically
	 * (relaxed) from USB callbacks; 64-bit on the LP64 archs we target.
	 */
	uint64_t		 sc_stat_tx_packets;	/* mbufs successfully sent */
	uint64_t		 sc_stat_tx_drops;	/* queue overflow, oversize */
	uint64_t		 sc_stat_tx_errors;	/* USB xfer errored */
	uint64_t		 sc_stat_rx_errors;	/* CRC / ICV err in RX desc */
	uint64_t		 sc_stat_rx_drops;	/* mbuf alloc fail, etc. */
	uint64_t		 sc_stat_usb_errors;	/* EP0 vendor-request fail */
	uint64_t		 sc_stat_cam_fails;	/* iv_key_set queue overflow */
	uint64_t		 sc_stat_fw_stalls;	/* watchdog: FW frozen */
	uint64_t		 sc_stat_c2h;		/* C2H events received */
	uint64_t		 sc_stat_c2h_unknown;	/* C2H with unrecognised cmd */

	/*
	 * Per-queue high-watermark.  Tracks the maximum backlog ever seen
	 * since the last `stats.reset` (or driver load).  Lets an operator
	 * see whether TX has ever pressed against its qdepth tunable, even
	 * if the live qlen sysctl shows it idle now.
	 */
	uint32_t		 sc_stat_tx_data_qlen_max;
	uint32_t		 sc_stat_tx_mgmt_qlen_max;
	uint32_t		 sc_stat_tx_pending_qlen_max;

	/*
	 * Per-rate-class RX histogram.  Each RX data/mgmt frame increments
	 * one bin based on the chip RX descriptor's rate field bucketed
	 * into CCK / OFDM / HT-MCS.  Lets an operator confirm at a glance
	 * whether the AP is sending at the speeds it advertised.
	 */
	uint64_t		 sc_stat_rx_cck;	/* rate 0..3 */
	uint64_t		 sc_stat_rx_ofdm;	/* rate 4..11 */
	uint64_t		 sc_stat_rx_mcs;	/* rate 12..35 */
	/*
	 * CCX TX report counters.  Bumped from C2H CCX_TX_RPT (top cmd 0x03,
	 * V0 layout) or HALMAC CCX_RPT (sub-cmd 0x0f, V1 layout) when the
	 * chip surfaces a per-TX delivery report.  Only fires for frames
	 * the driver requested a report on; we don't currently set the
	 * "request TX report" bit in TX desc W6 SW_DEFINE, but if the
	 * firmware emits unsolicited reports on certain frame types
	 * (KEEP_ALIVE NULL frames in particular) the counters surface them.
	 */
	uint64_t		 sc_stat_tx_acked;	/* CCX status == 0 */
	uint64_t		 sc_stat_tx_xretries;	/* status != 0 */
	const struct firmware	*sc_fw;
	struct usb_xfer		*sc_xfer[RTW88_N_TRANSFER];

	/* peek/poke address shared between sysctls -- userspace
	 * sets peek_addr, then reads peek_value to fetch the chip's
	 * 4-byte value at that EP0 vendor-request address.  Lets us
	 * dump arbitrary chip registers from shell without re-flashing
	 * the driver.  Companion poke_value writes the 4-byte value.
	 */
	uint16_t		 sc_peek_addr;

	/* SIPI (RF path A) peek address -- userspace writes the 8-bit
	 * RF register index (0x00..0x3F typically), then reads rf_value
	 * which masks the full RFREG width.  Lets us snapshot the
	 * chip's RF state for Linux-vs-FreeBSD diff at AUTH-time.
	 */
	uint8_t			 sc_rf_addr;

	/*
	 * Per-frame RX trace countdown.  Userspace sets sc_rx_trace
	 * to N to log the next N RX frames (w0, w3, fc0, da, sa); each
	 * rtw88_rx_one() decrements until zero.  Used to compare the
	 * chip-side RX desc layout against the Linux kprobe groundtruth
	 * captured in qemu-trace-x86/linux-rx-with-auth.trace.
	 */
	uint32_t		 sc_rx_trace;

	/*
	 * Per-C2H trace countdown.  Sysctl-set to N; each rx_c2h_dispatch()
	 * dumps id/seq/payload[0..7] until it hits zero.  Auto-armed to
	 * RTW88_C2H_TRACE_BOOT at attach so bring-up is captured without
	 * an operator racing the sysctl.
	 */
	uint32_t		 sc_c2h_trace;
#define	RTW88_C2H_TRACE_BOOT	16

	/*
	 * Per-frame mgmt TX trace countdown.  Userspace sets sc_tx_trace
	 * to N to log the next N mgmt TX submissions (full 48-byte TX
	 * desc + 802.11 hdr bytes) right before they're handed to
	 * usbd_transfer_submit.  Used to confirm what AUTH_REQ TX bytes
	 * actually leave the host for the chip.
	 */
	uint32_t		 sc_tx_trace;

	/*
	 * Per-EP0-write timing trace countdown.  Userspace sets
	 * sc_write_trace to N to log the next N rtw88_usb_write_region()
	 * completions with microsecond elapsed-time + gap-from-previous.
	 * Used to compare FreeBSD usb_proc completion latency against
	 * Linux's interrupt-context handlers (linux-rw-full-trace.txt).
	 */
	uint32_t		 sc_write_trace;
	int64_t			 sc_write_last_ns;

	/*
	 * MMIO trace toggle.  When nonzero, every EP0 register read/write
	 * emits a line in the same format Linux's bpftrace probes produce
	 * (W%d a=0x%x v=0x%x for writes, R%d a=0x%x for reads), where %d is
	 * the byte width (1/2/4).  Used to diff our AUTH-window register
	 * sequence against linux-auth-trace-2026_06_22.txt and find the
	 * first divergence.  Flip on right before triggering wpa_supplicant
	 * and off after deauth so the firmware-download phase doesn't drown
	 * the trace.
	 */
	uint32_t		 sc_trace_mmio;

	/*
	 * Serializes EP0 control transfers across usbd_do_request_flags's
	 * sc_mtx drop.  Background:  usbd releases &sc_mtx while it sleeps
	 * waiting for the USB completion, which lets a SECOND thread also
	 * holding sc_mtx call rtw88_usb_{read,write}_region(), enter
	 * usbd_do_request_flags(), and race the first thread's request at
	 * EP0.  Net80211's ic_set_channel (SIPI RF writes) and
	 * ic_update_{promisc,mcast} (RCR writes) commonly run concurrently
	 * during state transitions, so the chip ends up programmed with
	 * a non-deterministic write order.  Verified by P29 write_trace
	 * showing two distinct tids interleaved at addr=0x0c90 / 0x061a-d
	 * with elapsed_us=1005/1003 (~2x typical) and negative
	 * gap_from_prev_us, signature of in-flight overlap.
	 *
	 * The flag plus msleep loop forces strict EP0 ordering without
	 * preventing non-USB code from running during the USB wait.
	 */
	bool			 sc_io_busy;

	/*
	 * Software 802.11 sequence counter for mgmt TX frames that
	 * arrive with seq=0 (the wpa_supplicant raw_xmit path doesn't
	 * stamp one).  reference rtw88 lets mac80211 fill seq before
	 * raw_xmit; FreeBSD's net80211 path here can hand us mbufs
	 * with the seq_ctrl bytes still zero, which makes the AP
	 * treat every AUTH_REQ retry as a duplicate of the first.
	 */
	uint16_t		 sc_tx_seq;

	/*
	 * Userland cdev for the TCP-RPC bridge.  Exposes raw EP0
	 * vendor-request register read/write to /dev/rtw88_usb0.
	 * Companion userspace daemon (tools/rtw88_rpcd.c) tunnels
	 * those ioctls over TCP so a Linux/QEMU guest can drive the
	 * chip while it stays bound to this FreeBSD driver -- gives
	 * us byte-for-byte observability of Linux's chip-side access
	 * sequence during a full assoc.
	 */
	struct cdev		*sc_cdev;

	/*
	 * Single-slot TX sync state.  rtw88_bulk_tx_sync() loads the
	 * (data, len) into sc_tx_buf and arms sc_tx_pending; the matching
	 * pipe's callback drives the xfer to completion and signals
	 * sc_tx_cv.  Sized to RTW88_TX_BUFSZ which covers every
	 * firmware-download fragment (max 4 KiB payload + 48-byte desc).
	 */
	uint8_t			*sc_tx_buf;
	uint32_t		 sc_tx_len;
	bool			 sc_tx_pending;
	bool			 sc_tx_done;
	usb_error_t		 sc_tx_err;
	struct cv		 sc_tx_cv;

	/*
	 * Async TX queues.  Split by qsel (chip's per-AC descriptor bits)
	 * onto endpoints matching Linux 8821C rqpn_table[3] (num_out_pipes=3):
	 *
	 *   MGMT  (qsel 12+) -> BULK_TX_HI     (EP 0x05, out_ep[0], HIGH)
	 *   VI/VO (qsel 4-7) -> BULK_TX_NORMAL (EP 0x06, out_ep[1], NORMAL)
	 *   BE/BK (qsel 0-3) -> BULK_TX_LO     (EP 0x08, out_ep[2], LOW)
	 *
	 * Session 3 (2026-07-01) usbmon capture on working Linux run:
	 * frame 30037 = EAPOL M2 at EP 0x06 (qsel=7 VO), frame 30245 = DHCP
	 * DISCOVER at EP 0x08 (qsel=0 BE).  Confirms Linux qsel_to_ep from
	 * usb.c: dma_map_vo=NORMAL, dma_map_be=LOW.  v1.1.2 initial split
	 * had this INVERTED (BE->NORMAL, VO->LOW) which is why M2 never
	 * reached the AP -- chip TX scheduler expects VO on the NORMAL pipe.
	 *
	 * Each xfer callback drains its own queue from USB_ST_SETUP and
	 * holds the in-flight mbuf in sc_tx_*_inflight for cleanup on
	 * TRANSFERRED/error, then chains the next submit.
	 */
	struct mbufq		 sc_tx_mgmt_q;
	struct mbuf		*sc_tx_mgmt_inflight;
	struct mbufq		 sc_tx_data_q;		/* BE/BK -> EP 0x08 */
	struct mbuf		*sc_tx_data_inflight;
	struct mbufq		 sc_tx_data_vo_q;	/* VI/VO -> EP 0x06 */
	struct mbuf		*sc_tx_data_vo_inflight;
};

struct rtw88_fw_hdr_summary {
	uint16_t signature;
	uint16_t version;
	uint8_t  subversion;
	uint8_t  subindex;
	uint16_t year;
	uint8_t  month;
	uint8_t  day;
	uint8_t  hour;
	uint8_t  min;
	uint8_t  mem_usage;
	uint16_t h2c_fmt_ver;
	uint32_t dmem_addr;
	uint32_t dmem_size;
	uint32_t imem_addr;
	uint32_t imem_size;
	uint32_t emem_addr;
	uint32_t emem_size;
};

struct rtw88_vap {
	struct ieee80211vap	vap;
	int			(*newstate)(struct ieee80211vap *,
				    enum ieee80211_state, int);
};
#define	RTW88_VAP(vap)		((struct rtw88_vap *)(vap))

#define	RTW88_LOCK(sc)		mtx_lock(&(sc)->sc_mtx)
#define	RTW88_UNLOCK(sc)	mtx_unlock(&(sc)->sc_mtx)
#define	RTW88_ASSERT_LOCKED(sc)	mtx_assert(&(sc)->sc_mtx, MA_OWNED)

static const STRUCT_USB_HOST_ID rtw88_usb_devs[] = {
	/* Realtek 0x0bda — all 11 PIDs from Linux rtw_8821cu_id_table */
	{ USB_VPI(RTW88_VENDOR_REALTEK, 0x2006, RTW88_CHIP_8821C) },
	{ USB_VPI(RTW88_VENDOR_REALTEK, 0x8731, RTW88_CHIP_8821C) },
	{ USB_VPI(RTW88_VENDOR_REALTEK, 0x8811, RTW88_CHIP_8821C) },
	{ USB_VPI(RTW88_VENDOR_REALTEK, 0xb820, RTW88_CHIP_8821C) },
	{ USB_VPI(RTW88_VENDOR_REALTEK, 0xb82b, RTW88_CHIP_8821C) },
	{ USB_VPI(RTW88_VENDOR_REALTEK, 0xc80c, RTW88_CHIP_8821C) },
	{ USB_VPI(RTW88_VENDOR_REALTEK, 0xc811, RTW88_CHIP_8821C) },
	{ USB_VPI(RTW88_VENDOR_REALTEK, 0xc820, RTW88_CHIP_8821C) },
	{ USB_VPI(RTW88_VENDOR_REALTEK, 0xc821, RTW88_CHIP_8821C) },
	{ USB_VPI(RTW88_VENDOR_REALTEK, 0xc82a, RTW88_CHIP_8821C) },
	{ USB_VPI(RTW88_VENDOR_REALTEK, 0xc82b, RTW88_CHIP_8821C) },
	/* OEM rebadges with their own VID */
	{ USB_VPI(RTW88_VENDOR_DLINK,    0x331d, RTW88_CHIP_8821C) }, /* D-Link */
	{ USB_VPI(RTW88_VENDOR_EDIMAX,   0xc811, RTW88_CHIP_8821C) }, /* Edimax */
	{ USB_VPI(RTW88_VENDOR_EDIMAX,   0xd811, RTW88_CHIP_8821C) }, /* Edimax */
	{ USB_VPI(RTW88_VENDOR_MERCUSYS, 0x0105, RTW88_CHIP_8821C) }, /* Mercusys */
};

static device_probe_t	rtw88_usb_match;
static device_attach_t	rtw88_usb_attach;
static device_detach_t	rtw88_usb_detach;
static d_ioctl_t	rtw88_cdev_ioctl;
static struct cdevsw	rtw88_cdevsw;

static usb_callback_t	rtw88_bulk_rx_cb;
static usb_callback_t	rtw88_bulk_tx_cb;
static usb_callback_t	rtw88_bulk_tx_data_be_cb;
static usb_callback_t	rtw88_bulk_tx_data_vo_cb;

static int		rtw88_usb_read_region(struct rtw88_usb_softc *,
			    uint16_t, uint8_t *, uint16_t);
static int		rtw88_usb_read1(struct rtw88_usb_softc *, uint16_t,
			    uint8_t *);
static int		rtw88_usb_read4(struct rtw88_usb_softc *, uint16_t,
			    uint32_t *);
static int		rtw88_usb_write_region(struct rtw88_usb_softc *,
			    uint16_t, const uint8_t *, uint16_t);
static int		rtw88_usb_write1(struct rtw88_usb_softc *, uint16_t,
			    uint8_t);
static int		rtw88_usb_write2(struct rtw88_usb_softc *, uint16_t,
			    uint16_t);
static int		rtw88_usb_write4(struct rtw88_usb_softc *, uint16_t,
			    uint32_t);
static int		rtw88_usb_setbits1(struct rtw88_usb_softc *, uint16_t,
			    uint8_t);
static int		rtw88_usb_clrbits1(struct rtw88_usb_softc *, uint16_t,
			    uint8_t);
static int		rtw88_check_hw_ready(struct rtw88_usb_softc *,
			    uint16_t, uint32_t, uint32_t);
static void		rtw88_fw_hdr_decode(const uint8_t *,
			    struct rtw88_fw_hdr_summary *);
static int		rtw88_fw_acquire(struct rtw88_usb_softc *);
static void		rtw88_fw_release(struct rtw88_usb_softc *);
static int		rtw88_discover_endpoints(struct rtw88_usb_softc *);
static int		rtw88_xfers_setup(struct rtw88_usb_softc *);
static void		rtw88_xfers_teardown(struct rtw88_usb_softc *);
static int		rtw88_bulk_tx_sync(struct rtw88_usb_softc *, uint8_t,
			    const uint8_t *, uint32_t);
static int		rtw88_download_firmware(struct rtw88_usb_softc *);
static int		rtw88_mac_power_on(struct rtw88_usb_softc *);
/* rtw88_coex_cfg_init / rtw88_efuse_read_mac / rtw88_h2c_bt_wifi_control /
 * rtw88_h2c_coex_tdma_type moved to their respective subsystem TUs. */
static int		rtw88_set_channel_mac(struct rtw88_usb_softc *,
			    uint8_t, uint8_t, uint8_t);
static int		rtw88_mac_init(struct rtw88_usb_softc *);
static int		rtw88_h2c_pwr_mode_active(struct rtw88_usb_softc *);
static void		rtw88_fwka_task(void *, int);
static void		rtw88_fwka_start(struct rtw88_usb_softc *);
static void		rtw88_fwka_stop(struct rtw88_usb_softc *);
static int		rtw88_h2c_media_status_report(struct rtw88_usb_softc *,
			    uint8_t, bool);
static int		rtw88_h2c_ra_info(struct rtw88_usb_softc *,
			    uint8_t, uint8_t, uint8_t, uint8_t,
			    bool, bool, uint8_t, bool, uint32_t);
static int		rtw88_h2c_keep_alive(struct rtw88_usb_softc *, bool);
static int		rtw88_h2c_default_port(struct rtw88_usb_softc *,
			    uint8_t, uint8_t);
static int		rtw88_h2c_rsvd_page(struct rtw88_usb_softc *);
static int		rtw88_upload_rsvd_pages(struct rtw88_usb_softc *,
			    const uint8_t *);
static int		rtw88_cam_write_entry(struct rtw88_usb_softc *,
			    uint8_t, uint8_t, uint8_t, bool,
			    const uint8_t *, const uint8_t *, size_t);
static int		rtw88_cam_clear_entry(struct rtw88_usb_softc *,
			    uint8_t);
static int		rtw88_vap_key_set(struct ieee80211vap *,
			    const struct ieee80211_key *);
static int		rtw88_vap_key_delete(struct ieee80211vap *,
			    const struct ieee80211_key *);
static int		rtw88_conf_tx_all_ac_locked(struct rtw88_usb_softc *,
			    struct ieee80211com *);
static int		rtw88_phy_set_param(struct rtw88_usb_softc *);
static int		rtw88_phy_set_channel(struct rtw88_usb_softc *,
			    uint8_t, uint8_t, uint8_t);
static int		rtw88_phy_write_rf_a(struct rtw88_usb_softc *,
			    uint32_t, uint32_t);
static int		rtw88_write_rf_a_mask(struct rtw88_usb_softc *,
			    uint8_t, uint32_t, uint32_t);
static int		rtw88_net80211_attach(struct rtw88_usb_softc *);
static struct ieee80211vap *rtw88_vap_create(struct ieee80211com *,
			    const char [IFNAMSIZ], int, enum ieee80211_opmode,
			    int, const uint8_t [IEEE80211_ADDR_LEN],
			    const uint8_t [IEEE80211_ADDR_LEN]);
static void		rtw88_vap_delete(struct ieee80211vap *);
static int		rtw88_newstate(struct ieee80211vap *,
			    enum ieee80211_state, int);
static void		rtw88_getradiocaps(struct ieee80211com *, int, int *,
			    struct ieee80211_channel []);
static void		rtw88_set_channel(struct ieee80211com *);
static void		rtw88_scan_start(struct ieee80211com *);
static void		rtw88_scan_end(struct ieee80211com *);
static void		rtw88_newassoc(struct ieee80211_node *, int);
static void		rtw88_parent(struct ieee80211com *);
static int		rtw88_transmit(struct ieee80211com *, struct mbuf *);
static int		rtw88_raw_xmit(struct ieee80211_node *, struct mbuf *,
			    const struct ieee80211_bpf_params *);
static int		rtw88_wme_update(struct ieee80211com *);
static void		rtw88_update_promisc(struct ieee80211com *);
static void		rtw88_update_mcast(struct ieee80211com *);
static int		rtw88_assoc_ms(const struct rtw88_usb_softc *);
static void		rtw88_eapol_log(struct rtw88_usb_softc *,
			    const struct mbuf *, const char *);
static bool		rtw88_eapol_key_info(const struct mbuf *, uint16_t *);
static void		rtw88_cam_task(void *, int);

static const struct usb_config rtw88_config[RTW88_N_TRANSFER] = {
	[RTW88_BULK_RX] = {
		.type		= UE_BULK,
		.endpoint	= UE_ADDR_ANY,
		.direction	= UE_DIR_IN,
		.bufsize	= RTW88_RX_BUFSZ,
		.flags		= {
			.pipe_bof	= 1,
			.short_xfer_ok	= 1,
		},
		.callback	= rtw88_bulk_rx_cb,
	},
	/*
	 * Explicit endpoint addresses for OUT bulk slots, per Linux
	 * rtw88 reference (memory: project_rtw88_eapol_rate_fix_2026_06_25).
	 * RTL8821CU exposes 3 OUT bulk EPs which Linux maps via
	 * rqpn_table_8821c[num_out_pipes=3]:
	 *
	 *   EP 0x05 = HIGH FIFO   (firmware + mgmt + H2C)
	 *   EP 0x06 = NORMAL FIFO (BE/BK data: DHCP, IP, ICMP)
	 *   EP 0x08 = LOW FIFO    (VI/VO data: EAPOL M2/M4 + video/voice)
	 *
	 * Prior driver routed ALL data through EP 0x08 (LOW); DUT testing
	 * 2026-06-30 against a production HE AP showed that with BE traffic
	 * (ARP/DHCP) queued on EP 0x08, VO EAPOL M2 frames on the same
	 * endpoint never reached the AP even though the driver saw them
	 * successfully emitted.  Linux upstream on the same dongle uses
	 * the split above and completes 4-way + DHCP.
	 */
	[RTW88_BULK_TX_HI] = {
		.type		= UE_BULK,
		.endpoint	= 0x05,
		.direction	= UE_DIR_OUT,
		.bufsize	= RTW88_TX_BUFSZ,
		.flags		= {
			.pipe_bof	= 1,
			.force_short_xfer = 1,
		},
		.callback	= rtw88_bulk_tx_cb,
		.timeout	= RTW88_TX_TIMEOUT,
	},
	/*
	 * NORMAL FIFO (EP 0x06): VI/VO data.  EAPOL M2/M4 (mac80211-tagged
	 * as VO priority) and any QoS data with TID 4-7 goes here.  Matches
	 * Linux 8821C rqpn_table[3].dma_map_{vo,vi}=NORMAL -> out_ep[1].
	 * Drained by rtw88_bulk_tx_data_vo_cb from sc_tx_data_vo_q.
	 */
	[RTW88_BULK_TX_NORMAL] = {
		.type		= UE_BULK,
		.endpoint	= 0x06,
		.direction	= UE_DIR_OUT,
		.bufsize	= RTW88_TX_BUFSZ,
		.flags		= {
			.pipe_bof	= 1,
			.force_short_xfer = 1,
		},
		.callback	= rtw88_bulk_tx_data_vo_cb,
		.timeout	= RTW88_TX_TIMEOUT,
	},
	/*
	 * LOW FIFO (EP 0x08): BE/BK data.  All non-VO/non-mgmt data frames
	 * (DHCP, ICMP, TCP, IPv6 NDP) go here.  Matches Linux 8821C
	 * rqpn_table[3].dma_map_{be,bk}=LOW -> out_ep[2].  Drained by
	 * rtw88_bulk_tx_data_be_cb from sc_tx_data_q.
	 */
	[RTW88_BULK_TX_LO] = {
		.type		= UE_BULK,
		.endpoint	= 0x08,
		.direction	= UE_DIR_OUT,
		.bufsize	= RTW88_TX_BUFSZ,
		.flags		= {
			.pipe_bof	= 1,
			.force_short_xfer = 1,
		},
		.callback	= rtw88_bulk_tx_data_be_cb,
		.timeout	= RTW88_TX_TIMEOUT,
	},
	/*
	 * H2C commands share the HIGH FIFO with mgmt + firmware (Linux:
	 * out_ep[HIGH] = 0x05).  FreeBSD's USB stack allows multiple xfer
	 * slots to bind the same endpoint.
	 */
	[RTW88_BULK_TX_H2C] = {
		.type		= UE_BULK,
		.endpoint	= 0x05,
		.direction	= UE_DIR_OUT,
		.bufsize	= RTW88_TX_BUFSZ,
		.flags		= {
			.pipe_bof	= 1,
			.force_short_xfer = 1,
		},
		.callback	= rtw88_bulk_tx_cb,
		.timeout	= RTW88_TX_TIMEOUT,
	},
};

static int
rtw88_usb_match(device_t self)
{
	struct usb_attach_arg *uaa = device_get_ivars(self);

	if (uaa->usb_mode != USB_MODE_HOST)
		return (ENXIO);
	if (uaa->info.bConfigIndex != 0)
		return (ENXIO);
	if (uaa->info.bIfaceIndex != 0)
		return (ENXIO);

	return (usbd_lookup_id_by_uaa(rtw88_usb_devs,
	    sizeof(rtw88_usb_devs), uaa));
}

/*
 * True if addr falls in the chip's "always-on" register section.
 * Accesses to this range require a follow-up 1-byte seal write to
 * keep the "off" section power-gated awake (see comment above on
 * RTW88_REG_SEC_PING_ADDR).
 */
static inline bool
rtw88_addr_in_on_section(uint16_t addr)
{

	return (addr <= 0x00FF ||
	    (addr >= 0x1000 && addr <= 0x10FF));
}

/*
 * Post-access seal: write 1 byte (the LSB of the just-accessed data)
 * to 0x4E0.  Mirrors reference rtw_usb_reg_sec().  No-op if the just-
 * accessed address is not in the always-on section.  Failures here
 * are not propagated -- the seal is best-effort.
 */
/*
 * P30 EP0 serialization helpers.  Acquire/release sc_io_busy under sc_mtx.
 * The caller MUST hold sc_mtx and continue to hold it across the paired
 * exit; sc_mtx may be DROPPED by usbd between enter and exit.
 *
 * Recursive entry (e.g. rtw88_usb_reg_sec called from inside read/write
 * _region after the main usbd request) is NOT supported; reg_sec is the
 * tail of a serialized parent and rides the parent's claim.
 */
static void
rtw88_usb_io_enter(struct rtw88_usb_softc *sc)
{

	RTW88_ASSERT_LOCKED(sc);
	while (sc->sc_io_busy)
		msleep(&sc->sc_io_busy, &sc->sc_mtx, 0, "rtwio", 0);
	sc->sc_io_busy = true;
}

static void
rtw88_usb_io_exit(struct rtw88_usb_softc *sc)
{

	RTW88_ASSERT_LOCKED(sc);
	sc->sc_io_busy = false;
	wakeup_one(&sc->sc_io_busy);
}

static void
rtw88_usb_reg_sec(struct rtw88_usb_softc *sc, uint16_t addr, uint8_t data)
{
	struct usb_device_request req;

	if (!rtw88_addr_in_on_section(addr))
		return;

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = RTW88_USB_REQ_REGS;
	USETW(req.wValue, RTW88_REG_SEC_PING_ADDR);
	USETW(req.wIndex, 0);
	USETW(req.wLength, 1);
	(void)usbd_do_request_flags(sc->sc_udev, &sc->sc_mtx, &req, &data, 0,
	    NULL, RTW88_EP0_WRITE_TIMEOUT);
}

static int
rtw88_usb_read_region(struct rtw88_usb_softc *sc, uint16_t addr,
    uint8_t *buf, uint16_t len)
{
	struct usb_device_request req;
	usb_error_t err;

	RTW88_ASSERT_LOCKED(sc);
	rtw88_usb_io_enter(sc);

	if (sc->sc_trace_mmio && (len == 1 || len == 2 || len == 4))
		device_printf(sc->sc_dev, "R%u a=0x%x\n", len, addr);

	req.bmRequestType = UT_READ_VENDOR_DEVICE;
	req.bRequest = RTW88_USB_REQ_REGS;
	USETW(req.wValue, addr);
	USETW(req.wIndex, 0);
	USETW(req.wLength, len);

	err = usbd_do_request_flags(sc->sc_udev, &sc->sc_mtx, &req, buf, 0,
	    NULL, RTW88_EP0_READ_TIMEOUT);
	if (err != USB_ERR_NORMAL_COMPLETION) {
		static struct timeval rd_last;
		static const struct timeval rd_iv = { 1, 0 };
		sc->sc_stat_usb_errors++;
		if (ratecheck(&rd_last, &rd_iv))
			device_printf(sc->sc_dev,
			    "EP0 read 0x%04x len %u: %s "
			    "(further errors throttled)\n",
			    addr, len, usbd_errstr(err));
		rtw88_usb_io_exit(sc);
		return (EIO);
	}
	rtw88_usb_reg_sec(sc, addr, buf[0]);
	rtw88_usb_io_exit(sc);
	return (0);
}

static int
rtw88_usb_read4(struct rtw88_usb_softc *sc, uint16_t addr, uint32_t *val)
{
	uint8_t buf[4];
	int error;

	error = rtw88_usb_read_region(sc, addr, buf, sizeof(buf));
	if (error != 0)
		return (error);
	*val = le32dec(buf);
	return (0);
}

static int
rtw88_usb_read1(struct rtw88_usb_softc *sc, uint16_t addr, uint8_t *val)
{

	return (rtw88_usb_read_region(sc, addr, val, 1));
}

static int
rtw88_usb_read2(struct rtw88_usb_softc *sc, uint16_t addr, uint16_t *val)
{
	uint8_t buf[2];
	int error;

	error = rtw88_usb_read_region(sc, addr, buf, sizeof(buf));
	if (error != 0)
		return (error);
	*val = le16dec(buf);
	return (0);
}

static int
rtw88_usb_write_region(struct rtw88_usb_softc *sc, uint16_t addr,
    const uint8_t *buf, uint16_t len)
{
	struct usb_device_request req;
	struct bintime bt_enter, bt_exit;
	usb_error_t err;
	int64_t enter_ns = 0, exit_ns;
	bool trace;

	RTW88_ASSERT_LOCKED(sc);
	rtw88_usb_io_enter(sc);

	trace = (sc->sc_write_trace != 0);
	if (trace) {
		binuptime(&bt_enter);
		enter_ns = (int64_t)bt_enter.sec * 1000000000LL +
		    ((int64_t)(bt_enter.frac >> 32) * 1000000000LL) /
		    ((int64_t)1 << 32);
	}

	if (sc->sc_trace_mmio && (len == 1 || len == 2 || len == 4)) {
		uint32_t v = 0;
		int i;
		for (i = 0; i < (int)len; i++)
			v |= ((uint32_t)buf[i]) << (i * 8);
		device_printf(sc->sc_dev, "W%u a=0x%x v=0x%x\n", len, addr, v);
	}

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = RTW88_USB_REQ_REGS;
	USETW(req.wValue, addr);
	USETW(req.wIndex, 0);
	USETW(req.wLength, len);

	err = usbd_do_request_flags(sc->sc_udev, &sc->sc_mtx, &req,
	    __DECONST(void *, buf), 0, NULL, RTW88_EP0_WRITE_TIMEOUT);
	if (err != USB_ERR_NORMAL_COMPLETION) {
		static struct timeval wr_last;
		static const struct timeval wr_iv = { 1, 0 };
		sc->sc_stat_usb_errors++;
		if (ratecheck(&wr_last, &wr_iv))
			device_printf(sc->sc_dev,
			    "EP0 write 0x%04x len %u: %s "
			    "(further errors throttled)\n",
			    addr, len, usbd_errstr(err));
		rtw88_usb_io_exit(sc);
		return (EIO);
	}

	if (trace) {
		binuptime(&bt_exit);
		exit_ns = (int64_t)bt_exit.sec * 1000000000LL +
		    ((int64_t)(bt_exit.frac >> 32) * 1000000000LL) /
		    ((int64_t)1 << 32);
		device_printf(sc->sc_dev,
		    "write_trace tid=%d addr=0x%04x len=%u elapsed_us=%lld "
		    "gap_from_prev_us=%lld\n",
		    curthread->td_tid, addr, len,
		    (long long)(exit_ns - enter_ns) / 1000LL,
		    sc->sc_write_last_ns == 0 ? 0LL :
		    (long long)(enter_ns - sc->sc_write_last_ns) / 1000LL);
		sc->sc_write_last_ns = exit_ns;
		sc->sc_write_trace--;
	}

	rtw88_usb_reg_sec(sc, addr, buf[0]);
	rtw88_usb_io_exit(sc);
	return (0);
}

static int
rtw88_usb_write1(struct rtw88_usb_softc *sc, uint16_t addr, uint8_t val)
{

	return (rtw88_usb_write_region(sc, addr, &val, 1));
}

static int
rtw88_usb_write2(struct rtw88_usb_softc *sc, uint16_t addr, uint16_t val)
{
	uint8_t buf[2];

	le16enc(buf, val);
	return (rtw88_usb_write_region(sc, addr, buf, 2));
}

static int
rtw88_usb_write4(struct rtw88_usb_softc *sc, uint16_t addr, uint32_t val)
{
	uint8_t buf[4];

	le32enc(buf, val);
	return (rtw88_usb_write_region(sc, addr, buf, 4));
}

static int
rtw88_usb_setbits1(struct rtw88_usb_softc *sc, uint16_t addr, uint8_t bits)
{
	uint8_t v;
	int error;

	error = rtw88_usb_read1(sc, addr, &v);
	if (error != 0)
		return (error);
	return (rtw88_usb_write1(sc, addr, v | bits));
}

static int
rtw88_usb_clrbits1(struct rtw88_usb_softc *sc, uint16_t addr, uint8_t bits)
{
	uint8_t v;
	int error;

	error = rtw88_usb_read1(sc, addr, &v);
	if (error != 0)
		return (error);
	return (rtw88_usb_write1(sc, addr, v & ~bits));
}

/*
 * Poll up to 50 ms for (read32(reg) & mask) == target.  Mirrors
 * reference mainline check_hw_ready().
 */
static int
rtw88_check_hw_ready(struct rtw88_usb_softc *sc, uint16_t reg, uint32_t mask,
    uint32_t target)
{
	uint32_t val;
	int try;

	for (try = 0; try < 50; try++) {
		if (rtw88_usb_read4(sc, reg, &val) == 0 &&
		    (val & mask) == target)
			return (0);
		usb_pause_mtx(&sc->sc_mtx, hz / 100);
	}
	return (ETIMEDOUT);
}

/*
 * Decode the 64-byte firmware header into native-endian fields.
 * Offsets mirror reference mainline drivers/net/wireless/realtek/rtw88/
 * fw.h struct rtw_fw_hdr.
 */
static void
rtw88_fw_hdr_decode(const uint8_t *p, struct rtw88_fw_hdr_summary *out)
{
	out->signature   = le16dec(p + 0x00);
	out->version     = le16dec(p + 0x04);
	out->subversion  = p[0x06];
	out->subindex    = p[0x07];
	out->month       = p[0x10];
	out->day         = p[0x11];
	out->hour        = p[0x12];
	out->min         = p[0x13];
	out->year        = le16dec(p + 0x14);
	out->mem_usage   = p[0x18];
	out->h2c_fmt_ver = le16dec(p + 0x1C);
	out->dmem_addr   = le32dec(p + 0x20);
	out->dmem_size   = le32dec(p + 0x24);
	out->imem_size   = le32dec(p + 0x30);
	out->emem_size   = le32dec(p + 0x34);
	out->emem_addr   = le32dec(p + 0x38);
	out->imem_addr   = le32dec(p + 0x3C);
}

static int
rtw88_fw_acquire(struct rtw88_usb_softc *sc)
{
	const struct firmware *fw;
	struct rtw88_fw_hdr_summary h;

	fw = firmware_get(sc->sc_chip->fw_name);
	if (fw == NULL) {
		device_printf(sc->sc_dev,
		    "firmware \"%s\" not registered (kldload %s.ko first)\n",
		    sc->sc_chip->fw_name, sc->sc_chip->fw_name);
		return (ENOENT);
	}
	if (fw->datasize < RTW88_FW_HDR_SIZE) {
		device_printf(sc->sc_dev,
		    "firmware blob too small: %zu bytes\n", fw->datasize);
		firmware_put(fw, FIRMWARE_UNLOAD);
		return (EINVAL);
	}

	rtw88_fw_hdr_decode(fw->data, &h);
	if (h.signature != sc->sc_chip->chip_id) {
		device_printf(sc->sc_dev,
		    "firmware signature 0x%04x != expected 0x%04x\n",
		    h.signature, sc->sc_chip->chip_id);
		firmware_put(fw, FIRMWARE_UNLOAD);
		return (EINVAL);
	}

	device_printf(sc->sc_dev,
	    "firmware: sig=0x%04x ver=%u.%u.%u built %04u-%02u-%02u %02u:%02u"
	    " (%zu bytes)\n",
	    h.signature, h.version, h.subversion, h.subindex,
	    h.year, h.month, h.day, h.hour, h.min, fw->datasize);
	device_printf(sc->sc_dev,
	    "  dmem addr=0x%08x size=%u  imem addr=0x%08x size=%u"
	    "  emem addr=0x%08x size=%u  mem_usage=0x%02x h2c_fmt=%u\n",
	    h.dmem_addr, h.dmem_size, h.imem_addr, h.imem_size,
	    h.emem_addr, h.emem_size, h.mem_usage, h.h2c_fmt_ver);

	sc->sc_fw = fw;
	return (0);
}

static void
rtw88_fw_release(struct rtw88_usb_softc *sc)
{

	if (sc->sc_fw != NULL) {
		firmware_put(sc->sc_fw, FIRMWARE_UNLOAD);
		sc->sc_fw = NULL;
	}
}

/*
 * 8821C CCK LNA gain tables (Linux rtw8821c.c).  Indexed by rfe_option:
 * table_0 when rfe_option == 0, table_1 otherwise.
 */
static const int8_t rtw88_lna_gain_table_0[8] = {
	22, 8, -6, -22, -31, -40, -46, -52
};
static const int8_t rtw88_lna_gain_table_1[16] = {
	10, 6, 2, -2, -6, -10, -14, -17,
	-20, -24, -28, -31, -34, -37, -40, -44
};

static int8_t
rtw88_get_cck_rx_pwr_8821c(struct rtw88_usb_softc *sc, uint8_t lna_idx,
    uint8_t vga_idx)
{
	const int8_t *tab;
	uint8_t size;

	if (sc->sc_rfe_option == 0) {
		tab = rtw88_lna_gain_table_0;
		size = (uint8_t)nitems(rtw88_lna_gain_table_0);
	} else {
		tab = rtw88_lna_gain_table_1;
		size = (uint8_t)nitems(rtw88_lna_gain_table_1);
	}
	if (lna_idx >= size)
		return (-120);
	return ((int8_t)(tab[lna_idx] - 2 * (int8_t)vga_idx));
}

/*
 * Parse the 16-byte 8821C phy_status (drv_info) and fill the rx_power,
 * signal_dbm and bw fields of the stat struct.  Mirrors Linux
 * rtw8821c.c query_phy_status_page0/page1.
 */
static void
rtw88_query_phy_status_8821c(struct rtw88_usb_softc *sc, const uint8_t *ps,
    struct rtw88_rx_stat *st)
{
	uint32_t w0, w1, w3;
	uint8_t page, lna_idx, vga_idx, rxsc;
	int8_t pwdb_a;

	page = RTW88_PHY_STAT_PAGE(ps);
	w0 = le32dec(ps + 0);
	w1 = le32dec(ps + 4);
	w3 = le32dec(ps + 12);

	if (page == 0) {
		/* CCK: vga = w3[12:8]; lna = w3[15:13] | w3[23]<<3 */
		vga_idx = (uint8_t)((w3 >> 8) & 0x1FU);
		lna_idx = (uint8_t)(((w3 >> 13) & 0x07U) |
		    (((w3 >> 23) & 0x01U) << 3));
		st->rx_power = rtw88_get_cck_rx_pwr_8821c(sc, lna_idx, vga_idx);
		st->signal_dbm = st->rx_power;
		st->bw = 0;	/* CCK is always 20 MHz */
		return;
	}
	if (page != 1)
		return;

	/* OFDM/HT/VHT: PWDB_A = w0[15:8] - 110 */
	pwdb_a = (int8_t)((w0 >> 8) & 0xFFU);
	st->rx_power = (int8_t)(pwdb_a - 110);
	st->signal_dbm = (st->rx_power < -120) ? -120 : st->rx_power;

	if (st->rate > RTW88_DESC_RATE_CCK_MAX &&
	    st->rate < RTW88_DESC_RATE_MCS_MIN)
		rxsc = (uint8_t)((w1 >> 8) & 0x0FU);	/* L_RXSC */
	else
		rxsc = (uint8_t)((w1 >> 12) & 0x0FU);	/* HT_RXSC */

	if (rxsc >= 1 && rxsc <= 8)
		st->bw = 0;
	else if (rxsc >= 9 && rxsc <= 12)
		st->bw = 1;
	else if (rxsc >= 13)
		st->bw = 2;
	else
		st->bw = (uint8_t)((w3 >> 28) & 0x03U);	/* RF_MODE */
}

/*
 * Tag the mbuf with a properly-populated ieee80211_rx_stats before
 * handing it to net80211.  The bzero is critical: ieee80211_input_all
 * upstream declares its rx_stats on the stack and only sets r_flags/
 * c_nf/c_rssi, leaving c_pktflags as stack garbage; when the garbage
 * happens to look like DECRYPTED|FAIL_MMIC, crypto_demic fires
 * RTM_IEEE80211_MICHAEL and wpa_supplicant TKIP-countermeasures-locks
 * for 60 s.  We build our own zero-initialised rxs and route through
 * ieee80211_input_mimo_all to side-step that bug.
 *
 * Mirrors Linux rtw_rx_fill_rx_status from rx.c.
 */
/*
 * Hardware CCMP decrypt post-processing.  Called when the chip's RX engine
 * decrypted a Protected frame in HW (st.decrypted == 1).  The chip leaves
 * the 8-byte CCMP header (IV) sitting between the MAC header and the
 * plaintext payload, and leaves the 8-byte MIC tail in place.  net80211's
 * SW decap would try to decrypt again and fail, so we:
 *   1. memmove() the MAC header forward by 8 bytes to consume the IV.
 *   2. m_adj(-8) to drop the MIC tail.
 *   3. Clear FC1_PROTECTED so net80211 skips ieee80211_crypto_decap.
 *
 * Dead code under the SW CCMP default (REG_SEC_CONFIG=0 means the chip
 * never decrypts, st.decrypted is never 1).  Becomes live when the
 * hw.rtw88_usb.hw_ccmp tunable enables HW CCMP at attach.
 */
static void
rtw88_rx_strip_hw_ccmp(struct mbuf *m)
{
	struct ieee80211_frame *wh;
	uint8_t *p;
	int hdrlen;

	if (m->m_pkthdr.len < (int)(sizeof(struct ieee80211_frame) +
	    RTW88_CCMP_HDR_LEN + RTW88_CCMP_MIC_LEN))
		return;
	if (m->m_len < (int)sizeof(struct ieee80211_frame))
		return;
	wh = mtod(m, struct ieee80211_frame *);
	if ((wh->i_fc[1] & IEEE80211_FC1_PROTECTED) == 0)
		return;
	hdrlen = ieee80211_anyhdrsize(wh);
	if (m->m_pkthdr.len < hdrlen + RTW88_CCMP_HDR_LEN + RTW88_CCMP_MIC_LEN)
		return;
	if (m->m_len < hdrlen + RTW88_CCMP_HDR_LEN)
		return;

	/*
	 * Slide MAC header forward by CCMP_HDR_LEN to overwrite the IV, then
	 * adjust the mbuf to drop the now-duplicate leading 8 bytes.
	 */
	p = mtod(m, uint8_t *);
	memmove(p + RTW88_CCMP_HDR_LEN, p, hdrlen);
	m_adj(m, RTW88_CCMP_HDR_LEN);
	m_adj(m, -RTW88_CCMP_MIC_LEN);

	wh = mtod(m, struct ieee80211_frame *);
	wh->i_fc[1] &= ~IEEE80211_FC1_PROTECTED;
}

static void
rtw88_rx_annotate(struct rtw88_usb_softc *sc, struct mbuf *m,
    const struct rtw88_rx_stat *st)
{
	struct ieee80211_rx_stats rxs;
	uint32_t band;
	uint16_t freq;
	uint8_t ch;
	int8_t signal_dbm;

	bzero(&rxs, sizeof(rxs));
	ch = sc->sc_cur_channel;
	if (ch >= 36) {
		band = IEEE80211_CHAN_5GHZ;
		freq = 5000 + (uint16_t)ch * 5;
	} else if (ch >= 1 && ch <= 14) {
		band = IEEE80211_CHAN_2GHZ;
		freq = (ch == 14) ? 2484 : (uint16_t)(2407 + ch * 5);
	} else {
		band = 0;
		freq = 0;
	}

	rxs.r_flags = IEEE80211_R_NF | IEEE80211_R_RSSI |
	    IEEE80211_R_TSF32 | IEEE80211_R_TSF_START;
	if (freq != 0)
		rxs.r_flags |= IEEE80211_R_FREQ | IEEE80211_R_IEEE |
		    IEEE80211_R_BAND;

	rxs.c_rx_tsf = (uint64_t)st->tsf_low;
	rxs.c_nf = -95;
	signal_dbm = st->phy_status ? st->signal_dbm : -75;
	rxs.c_rssi = (int8_t)(signal_dbm + 95);

	/*
	 * Update EWMA of RSSI (diagnostics only; periodic RSSI_MONITOR H2C
	 * removed after 2026-06-25 stability regression).  Convention per
	 * Linux: avg_rssi = signal_dbm + 100 (so 50 = -50 dBm, range 0..100).
	 * Only update on phy_status-valid frames (the -75 dBm fallback
	 * above is a placeholder, not a real measurement).  EWMA alpha=1/8.
	 */
	if (st->phy_status) {
		int rssi_u = signal_dbm + 100;
		if (rssi_u < 0) rssi_u = 0;
		if (rssi_u > 100) rssi_u = 100;
		if (!sc->sc_avg_rssi_valid) {
			sc->sc_avg_rssi = (uint8_t)rssi_u;
			sc->sc_avg_rssi_valid = true;
		} else {
			sc->sc_avg_rssi = (uint8_t)
			    (((uint32_t)sc->sc_avg_rssi * 7 +
			      (uint32_t)rssi_u) >> 3);
		}
	}

	/* PHY type from descriptor rate (mirrors Linux rx encoding). */
	if (st->rate <= RTW88_DESC_RATE_CCK_MAX)
		rxs.c_pktflags |= IEEE80211_RX_F_CCK;
	else if (st->rate <= RTW88_DESC_RATE_OFDM_MAX)
		rxs.c_pktflags |= IEEE80211_RX_F_OFDM;
	else if (st->rate <= RTW88_DESC_RATE_MCS_MAX)
		rxs.c_pktflags |= IEEE80211_RX_F_HT;
	else if (st->rate >= RTW88_DESC_RATE_VHT_MIN)
		rxs.c_pktflags |= IEEE80211_RX_F_VHT;

	rxs.c_rate = st->rate;

	switch (st->bw) {
	case 1: rxs.c_width = IEEE80211_RX_FW_40MHZ; break;
	case 2: rxs.c_width = IEEE80211_RX_FW_80MHZ; break;
	default: rxs.c_width = IEEE80211_RX_FW_20MHZ; break;
	}

	rxs.c_freq = freq;
	rxs.c_ieee = ch;
	rxs.c_band = band;

	(void)ieee80211_add_rx_params(m, &rxs);
}

/*
 * C2H (chip->host) command IDs from Linux fw.h enum rtw_c2h_cmd_id.
 * Only the codes we explicitly count are named here; unknowns surface
 * via sc_stat_c2h_unknown.
 */
#define	RTW88_C2H_CCX_TX_RPT		0x03
#define	RTW88_C2H_BT_INFO		0x09
#define	RTW88_C2H_BT_MP_INFO		0x0b
#define	RTW88_C2H_RA_RPT		0x0c
#define	RTW88_C2H_HW_FEATURE_REPORT	0x19
#define	RTW88_C2H_WLAN_INFO		0x27
#define	RTW88_C2H_WLAN_RFON		0x32	/* Linux rtw_c2h_cmd_id */
/*
 * RTL8821CU firmware 24.11.0 emits id=0x15 with 1-byte payload = 0x02 as
 * a periodic heartbeat; not in Linux's rtw_c2h_cmd_id.  Observed via
 * dev.rtw88_usb.0.c2h_trace bring-up dump 2026-07-01.  Consume-and-drop.
 */
#define	RTW88_C2H_RTL8821CU_TICK	0x15
#define	RTW88_C2H_BCN_FILTER_NOTIFY	0x36
#define	RTW88_C2H_ADAPTIVITY		0x37
#define	RTW88_C2H_SCAN_RESULT		0x38
#define	RTW88_C2H_HALMAC		0xff

/* Sub-cmd IDs hidden behind RTW88_C2H_HALMAC (Linux enum rtw_c2h_cmd_id_ext). */
#define	RTW88_C2H_HALMAC_SCAN_STATUS_RPT	0x03
#define	RTW88_C2H_HALMAC_CCX_RPT		0x0f
#define	RTW88_C2H_HALMAC_CHAN_SWITCH		0x22

/*
 * Byte-for-byte mirror of Linux drivers/net/wireless/realtek/rtw88/fw.h
 * `struct rtw_c2h_cmd`.  All C2H packets (RTL8821CU and siblings) share
 * this framing regardless of the top-level cmd id:
 *
 *   id       -- rtw_c2h_cmd_id, e.g. RTW88_C2H_CCX_TX_RPT, RTW88_C2H_HALMAC
 *   seq      -- running counter, wraps mod 256 across a driver lifetime
 *   payload  -- variable-length body.  For id==HALMAC the first byte of
 *               payload is rtw_c2h_cmd_id_ext (the "sub-cmd"), i.e.
 *               payload[0] == c2h->payload[0].  For other ids the
 *               payload layout is per-command (see rtw_c2h_ra_rpt etc).
 *
 * Access via helpers so the offsets are wrong in ONE place if the layout
 * ever changes upstream.
 */
struct rtw88_c2h_cmd {
	uint8_t		id;
	uint8_t		seq;
	uint8_t		payload[];
} __packed;

#define	RTW88_C2H_MIN_LEN	__offsetof(struct rtw88_c2h_cmd, payload)

/*
 * C2H packet body starts immediately after the chip RX descriptor +
 * drv_info + shift padding (the same `buf + pkt_offset` the data path
 * uses).  Linux `struct rtw_c2h_cmd` (rtw88/fw.h) layout is
 * { u8 id; u8 seq; u8 payload[]; } — so body[0]=cmd_id, body[1]=seq
 * (running counter, NOT sub-cmd), body[2..]=payload.  The HALMAC (0xff)
 * sub-enum lives at payload[0] = body[2].  Linux drops most of these
 * onto chip-internal worker threads; we count + drop.  Real consumers
 * (RA rate-feedback, scan completion, BT coex feedback) belong with
 * the feature ports.
 *
 * Unknown IDs are logged once each (sc_c2h_unknown_seen bitmap by
 * top-byte; halmac sub-IDs get their own bitmap) so an operator can
 * see which codes their chip actually emits without a custom build.
 */
static void
rtw88_c2h_log_unknown(struct rtw88_usb_softc *sc, uint8_t cmd,
    uint8_t halmac_sub)
{
	static uint64_t seen_top[4];	/* 256 bits */
	static uint64_t seen_halmac[4];	/* 256 bits */
	uint64_t *bm;
	uint8_t bit;

	if (halmac_sub == 0xff) {
		bm = &seen_top[cmd >> 6];
		bit = cmd & 0x3f;
	} else {
		bm = &seen_halmac[halmac_sub >> 6];
		bit = halmac_sub & 0x3f;
	}
	if ((*bm & (1ULL << bit)) == 0) {
		*bm |= (1ULL << bit);
		if (halmac_sub == 0xff)
			device_printf(sc->sc_dev,
			    "c2h: first unknown cmd=0x%02x\n", cmd);
		else
			device_printf(sc->sc_dev,
			    "c2h: first unknown HALMAC sub=0x%02x\n",
			    halmac_sub);
	}
}

static void
rtw88_rx_c2h_dispatch(struct rtw88_usb_softc *sc, const uint8_t *body,
    uint16_t len)
{
	const struct rtw88_c2h_cmd *c2h;
	uint16_t payload_len;

	sc->sc_stat_c2h++;
	if (len < RTW88_C2H_MIN_LEN)
		return;
	c2h = (const struct rtw88_c2h_cmd *)body;
	payload_len = len - RTW88_C2H_MIN_LEN;

	/*
	 * First N C2Hs after driver load are dumped raw so the operator can
	 * confirm the id/seq/payload layout empirically -- some Realtek
	 * chips differ from the stock Linux struct.  Bounded to sc_c2h_trace
	 * so it can't loop-storm the console.
	 */
	if (sc->sc_c2h_trace > 0) {
		sc->sc_c2h_trace--;
		device_printf(sc->sc_dev,
		    "c2h trc: id=0x%02x seq=%u len=%u payload:"
		    " %02x %02x %02x %02x %02x %02x %02x %02x\n",
		    c2h->id, c2h->seq, payload_len,
		    payload_len > 0 ? c2h->payload[0] : 0,
		    payload_len > 1 ? c2h->payload[1] : 0,
		    payload_len > 2 ? c2h->payload[2] : 0,
		    payload_len > 3 ? c2h->payload[3] : 0,
		    payload_len > 4 ? c2h->payload[4] : 0,
		    payload_len > 5 ? c2h->payload[5] : 0,
		    payload_len > 6 ? c2h->payload[6] : 0,
		    payload_len > 7 ? c2h->payload[7] : 0);
	}

	switch (c2h->id) {
	case RTW88_C2H_CCX_TX_RPT:
		/*
		 * Linux GET_CCX_REPORT_STATUS_V0 = payload[0] & 0xc0.
		 * Status 0 = ACKed, non-zero = retries exhausted.
		 */
		if (payload_len >= 1) {
			if ((c2h->payload[0] & 0xc0) == 0)
				sc->sc_stat_tx_acked++;
			else
				sc->sc_stat_tx_xretries++;
		}
		break;
	case RTW88_C2H_BT_INFO:
	case RTW88_C2H_BT_MP_INFO:
	case RTW88_C2H_RA_RPT:
	case RTW88_C2H_HW_FEATURE_REPORT:
	case RTW88_C2H_WLAN_INFO:
	case RTW88_C2H_WLAN_RFON:
	case RTW88_C2H_BCN_FILTER_NOTIFY:
	case RTW88_C2H_ADAPTIVITY:
	case RTW88_C2H_SCAN_RESULT:
	case RTW88_C2H_RTL8821CU_TICK:
		break;
	case RTW88_C2H_HALMAC:
		/*
		 * halmac is a sub-dispatcher; sub_cmd_id lives at
		 * payload[0].  Linux rtw_fw_c2h_cmd_handle_ext does
		 * `sub_cmd_id = c2h->payload[0]`.
		 */
		if (payload_len < 1) {
			sc->sc_stat_c2h_unknown++;
			break;
		}
		switch (c2h->payload[0]) {
		case RTW88_C2H_HALMAC_SCAN_STATUS_RPT:
		case RTW88_C2H_HALMAC_CHAN_SWITCH:
			break;
		case RTW88_C2H_HALMAC_CCX_RPT:
			/* V1 STATUS = payload[9] & 0xc0. */
			if (payload_len >= 10) {
				if ((c2h->payload[9] & 0xc0) == 0)
					sc->sc_stat_tx_acked++;
				else
					sc->sc_stat_tx_xretries++;
			}
			break;
		default:
			sc->sc_stat_c2h_unknown++;
			rtw88_c2h_log_unknown(sc, RTW88_C2H_HALMAC,
			    c2h->payload[0]);
			break;
		}
		break;
	default:
		sc->sc_stat_c2h_unknown++;
		rtw88_c2h_log_unknown(sc, c2h->id, 0xff);
		break;
	}
}

/*
 * Demultiplex one bulk-IN packet that the USB framework has copied
 * into a temporary buffer.  Returns the offset within the staging
 * buffer where the *next* packet starts (8-byte aligned), or 0 if
 * the input is malformed and the caller should stop iterating.
 *
 * One bulk transfer may carry multiple back-to-back chip packets:
 *
 *   [desc24][drv_info][shift][802.11 frame][pad to 8]
 *   [desc24][drv_info][shift][802.11 frame][pad to 8] ...
 *
 * C2H (firmware-to-host) packets are surfaced via rtw88_rx_c2h_dispatch
 * before being consumed by ieee80211_input -- they're chip-bound events
 * (CCX TX reports, BT info, RA reports, scan results, halmac), not
 * 802.11 frames, and have a chip-specific header layout.
 */
static size_t
rtw88_rx_one(struct rtw88_usb_softc *sc, const uint8_t *buf, size_t avail)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct mbuf *m;
	struct rtw88_rx_stat st;
	uint32_t w0, w2, w3, w4, w5;
	size_t pkt_offset, total, aligned;

	if (avail < RTW88_RX_DESC_SZ)
		return (0);

	bzero(&st, sizeof(st));
	w0 = le32dec(buf + 0);
	w2 = le32dec(buf + 8);
	w3 = le32dec(buf + 12);
	w4 = le32dec(buf + 16);
	w5 = le32dec(buf + 20);

	st.pkt_len     = (uint16_t)(w0 & RTW88_RX_DESC_W0_PKT_LEN_M);
	st.crc_err     = (w0 & RTW88_RX_DESC_W0_CRC_ERR) ? 1 : 0;
	st.icv_err     = (w0 & RTW88_RX_DESC_W0_ICV_ERR) ? 1 : 0;
	st.drv_info_sz = (uint8_t)(((w0 & RTW88_RX_DESC_W0_DRVINFO_SZ_M)
	    >> RTW88_RX_DESC_W0_DRVINFO_SZ_S) * 8);
	st.enc_type    = (uint8_t)((w0 & RTW88_RX_DESC_W0_ENC_TYPE_M)
	    >> RTW88_RX_DESC_W0_ENC_TYPE_S);
	st.shift       = (uint8_t)((w0 & RTW88_RX_DESC_W0_SHIFT_M)
	    >> RTW88_RX_DESC_W0_SHIFT_S);
	st.phy_status  = (w0 & RTW88_RX_DESC_W0_PHYST) ? 1 : 0;
	st.swdec       = (w0 & RTW88_RX_DESC_W0_SWDEC) ? 1 : 0;
	st.decrypted   = (!st.swdec && st.enc_type != RTW88_RX_DESC_ENC_NONE)
	    ? 1 : 0;
	st.is_c2h      = (w2 & RTW88_RX_DESC_W2_C2H) ? 1 : 0;
	st.rate        = (uint8_t)(w3 & RTW88_RX_DESC_W3_RX_RATE_M);
	st.bw          = (uint8_t)((w4 & RTW88_RX_DESC_W4_BW_M)
	    >> RTW88_RX_DESC_W4_BW_S);
	st.tsf_low     = w5;
	st.signal_dbm  = -75;
	st.rx_power    = -75;

	pkt_offset = (size_t)RTW88_RX_DESC_SZ + st.drv_info_sz + st.shift;
	total = pkt_offset + st.pkt_len;
	if (total > avail)
		return (0);

	if (st.shift != 0) {
		static uint32_t shift_seen_mask;
		uint32_t bit = 1U << (st.shift & 0x1F);
		if ((shift_seen_mask & bit) == 0) {
			shift_seen_mask |= bit;
			device_printf(sc->sc_dev,
			    "RX_SHIFT_NONZERO: shift=%u drv_info_sz=%u "
			    "pkt_len=%u fc0=%02x (descriptor parse bug "
			    "candidate: pkt_offset uses [desc+drvinfo+shift] "
			    "vs Linux [desc+shift+drvinfo])\n",
			    st.shift, st.drv_info_sz, st.pkt_len,
			    *(buf + pkt_offset));
		}
	}
	if (sc->sc_rx_trace != 0 && st.pkt_len >= 16) {
		const uint8_t *h = buf + pkt_offset;
		device_printf(sc->sc_dev,
		    "rxtrc w0=%08x w3=%08x c2h=%d len=%u fc0=%02x "
		    "shift=%u info=%u "
		    "da=%02x:%02x:%02x:%02x:%02x:%02x "
		    "sa=%02x:%02x:%02x:%02x:%02x:%02x\n",
		    w0, w3, st.is_c2h, st.pkt_len, h[0],
		    st.shift, st.drv_info_sz,
		    h[4], h[5], h[6], h[7], h[8], h[9],
		    h[10], h[11], h[12], h[13], h[14], h[15]);
		sc->sc_rx_trace--;
	}

	aligned = (total + 7) & ~(size_t)7;

	if (st.is_c2h) {
		rtw88_rx_c2h_dispatch(sc, buf + pkt_offset, st.pkt_len);
		return (aligned);
	}
	if (st.pkt_len == 0)
		return (aligned);

	/*
	 * REG_RCR has BIT_APPFCS set, so hardware appends a 4-byte FCS to
	 * every RX frame.  Drop CRC errors outright and strip the trailing
	 * FCS; net80211's beacon parser would otherwise treat it as a
	 * malformed IE (BPARSE_BADIELEN).
	 */
	if (st.crc_err) {
		sc->sc_stat_rx_errors++;
		counter_u64_add(sc->sc_ic.ic_ierrors, 1);
		return (aligned);
	}
	if (st.icv_err) {
		sc->sc_stat_rx_errors++;
		counter_u64_add(sc->sc_ic.ic_ierrors, 1);
	}
	if (st.pkt_len <= IEEE80211_CRC_LEN)
		return (aligned);
	st.pkt_len -= IEEE80211_CRC_LEN;

	/*
	 * If the descriptor advertises a phy_status (drv_info) block, parse
	 * it for real signal_dbm + bw.  The block lives right after the
	 * 24-byte descriptor and before the optional shift padding.
	 */
	if (st.phy_status && st.drv_info_sz >= RTW88_PHY_STATUS_SIZE) {
		rtw88_query_phy_status_8821c(sc,
		    buf + RTW88_RX_DESC_SZ, &st);
	}

	if (st.pkt_len > MCLBYTES) {
		sc->sc_stat_rx_drops++;
		return (aligned);
	}
	m = m_get2(st.pkt_len, M_NOWAIT, MT_DATA, M_PKTHDR);
	if (m == NULL) {
		sc->sc_stat_rx_drops++;
		return (aligned);
	}

	memcpy(mtod(m, void *), buf + pkt_offset, st.pkt_len);
	m->m_pkthdr.len = m->m_len = st.pkt_len;
	m->m_pkthdr.rcvif = NULL;
	sc->sc_rx_frames++;

	/* Per-rate-class RX histogram */
	if (st.rate <= RTW88_DESC_RATE_CCK_MAX)
		sc->sc_stat_rx_cck++;
	else if (st.rate <= RTW88_DESC_RATE_OFDM_MAX)
		sc->sc_stat_rx_ofdm++;
	else if (st.rate <= RTW88_DESC_RATE_MCS_MAX)
		sc->sc_stat_rx_mcs++;

	if (st.decrypted && st.enc_type == RTW88_RX_DESC_ENC_AES)
		rtw88_rx_strip_hw_ccmp(m);

	/*
	 * Tag QoS-data frames from an HT peer as A-MPDU sub-frames so
	 * net80211's SW reorder buffer can resequence them.  net80211 looks
	 * up the source node via the MAC header; we just have to set the
	 * mbuf flag.  Dead under sc_ampdu==false (no BA session ever forms).
	 */
	if (sc->sc_ampdu && st.pkt_len >= 24) {
		const uint8_t *h = mtod(m, const uint8_t *);
		uint8_t fc0 = h[0];
		if ((fc0 & 0x8C) == 0x88)
			m->m_flags |= M_AMPDU;
	}

	rtw88_rx_annotate(sc, m, &st);
	rtw88_eapol_log(sc, m, "rx");

	/*
	 * Populate the per-RX radiotap header from the chip RX descriptor.
	 * ieee80211_input_mimo_all calls bpf_mtap below if radiotap is
	 * active on any interested listener.
	 */
	if (ieee80211_radiotap_active(ic)) {
		/*
		 * Translate chip desc-rate (Linux RTW88_DESC_RATE_*) into
		 * radiotap's 500 Kb/s units.  For HT/VHT frames the rate
		 * field isn't applicable — leave 0 and let chan_flags +
		 * c_pktflags IEEE80211_RX_F_HT/VHT carry the bandwidth/MCS.
		 */
		static const uint8_t rate500k[RTW88_DESC_RATE_OFDM_MAX + 1] = {
			[0x00] = 2,	/* CCK  1   Mb/s */
			[0x01] = 4,	/* CCK  2   Mb/s */
			[0x02] = 11,	/* CCK  5.5 Mb/s */
			[0x03] = 22,	/* CCK 11   Mb/s */
			[0x04] = 12,	/* OFDM 6   Mb/s */
			[0x05] = 18,	/* OFDM 9   Mb/s */
			[0x06] = 24,	/* OFDM 12  Mb/s */
			[0x07] = 36,	/* OFDM 18  Mb/s */
			[0x08] = 48,	/* OFDM 24  Mb/s */
			[0x09] = 72,	/* OFDM 36  Mb/s */
			[0x0A] = 96,	/* OFDM 48  Mb/s */
			[0x0B] = 108,	/* OFDM 54  Mb/s */
		};
		sc->sc_rxtap.wr_tsft = htole64((uint64_t)st.tsf_low);
		sc->sc_rxtap.wr_flags = 0;
		sc->sc_rxtap.wr_rate = (st.rate <= RTW88_DESC_RATE_OFDM_MAX) ?
		    rate500k[st.rate] : 0;
		sc->sc_rxtap.wr_chan_freq =
		    htole16(ic->ic_curchan ?
		    ic->ic_curchan->ic_freq : 0);
		sc->sc_rxtap.wr_chan_flags =
		    htole16(ic->ic_curchan ?
		    ic->ic_curchan->ic_flags : 0);
		sc->sc_rxtap.wr_dbm_antsignal = st.signal_dbm;
		sc->sc_rxtap.wr_dbm_antnoise = -96;
	}

	RTW88_UNLOCK(sc);
	(void)ieee80211_input_mimo_all(ic, m);
	RTW88_LOCK(sc);

	return (aligned);
}

/*
 * Bulk-IN RX callback.  On every completion copy the buffer out of
 * the USB DMA cache into a stack staging area, then iterate
 * rtw88_rx_one() until the buffer is consumed.  Re-arm the xfer
 * unconditionally so the chip's RX FIFO drains.
 */
static void
rtw88_bulk_rx_cb(struct usb_xfer *xfer, usb_error_t error)
{
	struct rtw88_usb_softc *sc = usbd_xfer_softc(xfer);
	struct usb_page_cache *pc;
	size_t off, consumed;
	int actlen;

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
		usbd_xfer_status(xfer, &actlen, NULL, NULL, NULL);
		sc->sc_rx_packets++;
		if (actlen < 0 || (size_t)actlen > RTW88_RX_BUFSZ ||
		    sc->sc_rx_stage == NULL)
			break;
		pc = usbd_xfer_get_frame(xfer, 0);
		usbd_copy_out(pc, 0, sc->sc_rx_stage, actlen);

		for (off = 0; off < (size_t)actlen; off += consumed) {
			consumed = rtw88_rx_one(sc, sc->sc_rx_stage + off,
			    (size_t)actlen - off);
			if (consumed == 0)
				break;
		}
		/* FALLTHROUGH */
	case USB_ST_SETUP:
		usbd_xfer_set_frame_len(xfer, 0, usbd_xfer_max_len(xfer));
		usbd_transfer_submit(xfer);
		break;
	default:
		if (error != USB_ERR_CANCELLED) {
			static struct timeval rx_last;
			static const struct timeval rx_iv = { 1, 0 };
			sc->sc_stat_rx_errors++;
			if (ratecheck(&rx_last, &rx_iv))
				device_printf(sc->sc_dev,
				    "rx error: %s (further throttled)\n",
				    usbd_errstr(error));
			usbd_xfer_set_stall(xfer);
			usbd_xfer_set_frame_len(xfer, 0,
			    usbd_xfer_max_len(xfer));
			usbd_transfer_submit(xfer);
		}
		break;
	}
}

/*
 * TX callback drives the single-slot sync TX state machine.  When
 * sc_tx_pending is set the callback loads the staged buffer into the
 * USB frame and submits; on completion (or error) it signals
 * sc_tx_cv so rtw88_bulk_tx_sync() wakes up.
 */
/*
 * Should this outbound frame be marked aggregable in the TX desc?
 * Returns true (and fills *tid_out) for QoS data frames addressed to an
 * HT peer with an active BA session on this TID.  The decision drives
 * RTW88_TXD_W2_AGG_EN + RTW88_TXD_W3_MAX_AGG_NUM in the data desc fill.
 * Dead under sc_ampdu==false (HTC_AMPDU not advertised, no BA ever runs).
 */
static bool
rtw88_tx_use_ampdu(struct rtw88_usb_softc *sc, struct mbuf *m, uint8_t *tid_out)
{
	struct ieee80211_node *ni;
	const uint8_t *h;
	uint8_t fc0, tid;

	if (!sc->sc_ampdu)
		return (false);
	if (m->m_pkthdr.len < 26)
		return (false);
	h = mtod(m, const uint8_t *);
	fc0 = h[0];
	/* QoS data: type=data (bits 3:2 = 10), subtype carries QoS bit (bit7) */
	if ((fc0 & 0x8C) != 0x88)
		return (false);
	ni = (struct ieee80211_node *)m->m_pkthdr.rcvif;
	if (ni == NULL || (ni->ni_flags & IEEE80211_NODE_HT) == 0)
		return (false);
	tid = h[24] & 0x0F;
	if (tid >= WME_NUM_TID)
		return (false);
	if ((ni->ni_tx_ampdu[tid].txa_flags & IEEE80211_AGGR_RUNNING) == 0)
		return (false);
	*tid_out = tid;
	return (true);
}

/*
 * Try to fill the xfer from the async mgmt queue.  Returns true and
 * submits the xfer if a frame was dequeued; returns false if the
 * queue is empty (caller does nothing).  Builds the 48-byte TX
 * descriptor in sc_tx_buf, copies the 802.11 frame in after it,
 * and stages the mbuf in sc_tx_mgmt_inflight so the TRANSFERRED /
 * error paths know how to free it.
 */
static bool
rtw88_bulk_tx_drain_q(struct rtw88_usb_softc *sc, struct usb_xfer *xfer,
    struct mbufq *q, struct mbuf **inflightp)
{
	struct mbuf *m;
	struct usb_page_cache *pc;
	uint8_t *desc = sc->sc_tx_buf;
	uint32_t w0, w1, w3, w4;
	uint16_t chksum, framelen, total;
	bool bmc;
	uint16_t i;

	RTW88_ASSERT_LOCKED(sc);

	m = mbufq_dequeue(q);
	if (m == NULL)
		return (false);
	framelen = (uint16_t)m->m_pkthdr.len;
	if (framelen == 0 ||
	    framelen + RTW88_TX_PKT_DESC_SZ > RTW88_TX_BUFSZ) {
		m_freem(m);
		return (false);
	}

	/*
	 * BMC bit in TX descriptor = receiver address (A1) is broadcast
	 * or multicast.  Linux uses is_broadcast_ether_addr(hdr->addr1) ||
	 * is_multicast_ether_addr(hdr->addr1) (tx.c:23).  The prior check
	 * read fc0 bit 0 (protocol version, always 0) and never set BMC,
	 * which is wrong for IBSS / probe / unencrypted-broadcast paths
	 * where A1 == DA == ff:ff:ff:ff:ff:ff.
	 */
	bmc = framelen >= 10 &&
	    (mtod(m, const uint8_t *)[4] & 0x01) != 0;
	/*
	 * Split mgmt vs data on FC type bits (FC0[3:2]): 0x00=mgmt,
	 * 0x08=data, 0x04=ctl.  Data frames need TID-based qsel + RA
	 * engine rate selection; the AUTH_REQ-bit-perfect mgmt path
	 * below would tag DHCP/ping as MGMT (qsel=18) and force CCK 1M
	 * which the chip's TX scheduler routes to the wrong FIFO and
	 * the AP drops as a malformed mgmt frame.
	 */
	uint8_t fc0 = mtod(m, const uint8_t *)[0];
	bool is_data = (fc0 & 0x0C) == 0x08;
	bool is_eapol_early = false;
	{
		uint16_t ki;
		is_eapol_early = rtw88_eapol_key_info(m, &ki);
	}
	memset(desc, 0, RTW88_TX_PKT_DESC_SZ);

	if (is_data) {
		/*
		 * Data-frame TX descriptor — port of Linux rtw_tx_fill_tx_desc
		 * (drivers/net/wireless/realtek/rtw88/tx.c:35).  Linux writes
		 * EVERY W0..W9 even when most fields are 0 for our 1T1R 20 MHz
		 * BGN STA case; the previous minimal layout (W0+W1 only) left
		 * the chip's TX scheduler reading uninitialised-via-memset
		 * fields for SW_SEQ / EN_HWSEQ / BW / SHORT_GI / SW_DEFINE
		 * / etc., and chip would stamp wrong dir bits OR silently
		 * drop frames on the bulk-OUT path.  Per 2026-06-27 AP-side
		 * wlandebug isolating the data-plane wall to chip-side
		 * mis-framing (project_rtw88_dir2_chip_bug_2026_06_27.md).
		 *
		 * Per-STA values (bw, rate_id, sgi, ldpc, stbc) hardcoded
		 * for the 1T1R 20 MHz STA case the driver targets today.
		 * When VHT/HT40 lands, thread these from a real sta_info.
		 *
		 * net80211 SW-encrypts AES_CCM for us (we dropped AES_CCM from
		 * ic_cryptocaps in attach), so the body arriving is already
		 * CCMP ciphertext + IV + MIC; SEC_TYPE stays 0 in W1 so the
		 * chip's CCMP engine doesn't re-encrypt.
		 */
		uint16_t seq_ctrl = (framelen >= 24) ?
		    le16dec(mtod(m, const uint8_t *) + 22) : 0;
		uint32_t seq = (uint32_t)(seq_ctrl >> 4) & 0xFFFU;

		/* W0: TXPKTSIZE | OFFSET | LS [| BMC] */
		w0 = ((uint32_t)framelen & 0xFFFFU) |
		    ((uint32_t)RTW88_TX_PKT_DESC_SZ << RTW88_TXD_W0_OFFSET_S) |
		    RTW88_TXD_W0_LS;
		if (bmc)
			w0 |= RTW88_TXD_W0_BMC;
		le32enc(desc + 0, w0);

		/*
		 * W1: MAC_ID | QSEL | RATE_ID [| SEC_TYPE]
		 *
		 * QSEL routes the frame through the chip's per-AC TX scheduler.
		 * Linux's rtw_tx_pkt_info_update sets qsel = skb->priority
		 * (the TID, 0..7).  Without proper qsel routing, EAPOL frames
		 * (which mac80211 sets to priority=7 = VO) get stuck behind BE
		 * traffic on busy channels and modern (HE-capable) APs time
		 * out the handshake.  For QoS data we pull the TID from the
		 * QoS control field; for non-QoS data we use BE (qsel=2) as
		 * the default; EAPOL detection short-circuits to VO (qsel=7).
		 */
		uint32_t qsel = 2;  /* BE default */
		if (is_eapol_early) {
			qsel = 7;   /* VO -- prioritize EAPOL */
		} else if ((fc0 & 0x80) != 0 && framelen >= 26) {
			/* QoS data: low 4 bits of QoS ctrl byte 24 is TID */
			qsel = mtod(m, const uint8_t *)[24] & 0x07;
		}
		w1 = ((uint32_t)0 << RTW88_TXD_W1_MACID_S) |
		    (qsel << RTW88_TXD_W1_QSEL_S) |
		    ((uint32_t)sc->sc_rate_id <<
		     RTW88_TXD_W1_RATE_ID_S);
		le32enc(desc + 4, w1);

		/*
		 * W2: AGG_EN + AMPDU_DEN when net80211 has a running BA session
		 * for this peer/TID; otherwise zero (chip TXes single MPDU).
		 * Density 4 = 8us, matches our HTCAP_MPDUDENSITY advertisement.
		 *
		 * SPE_RPT (bit 19) + SW_DEFINE (W6 bits 11:0) enable per-frame
		 * TX-status reporting.  Linux sets these for EAPOL (M2/M4 need
		 * TX-status feedback for the supplicant retransmit logic).
		 * Session 4 usbmon comparison: Linux M2 (frame 30037) had
		 * W2=0x00080000 W6=0x0000000c; ours had both zero, which the
		 * chip apparently treats as a "silent" data frame that skips
		 * the ACK+report path.  M2 wall candidate #2.
		 */
		{
			uint8_t agg_tid = 0;
			uint32_t w2 = 0;
			if (rtw88_tx_use_ampdu(sc, m, &agg_tid)) {
				uint32_t den = (uint32_t)rtw88_ampdu_density;
				w2 |= RTW88_TXD_W2_AGG_EN;
				w2 |= (den & 0x7U) << RTW88_TXD_W2_AMPDU_DEN_S;
				(void)agg_tid;
			}
			if (is_eapol_early)
				w2 |= RTW88_TXD_W2_SPE_RPT;
			le32enc(desc + 8, w2);
		}

		/*
		 * W3/W4: rate selection.  Linux routes data-subtype EAPOL
		 * (M2/M4 from mac80211's tx_control_port path) through the
		 * regular data path with rate_id from the per-STA info and
		 * use_rate=false — i.e. lets the chip's RA engine pick.
		 * We previously forced USE_RATE+DISDATAFB+CCK_1M for EAPOL,
		 * which DUT-tested 2026-06-30 against WIFI showed:
		 *   - chip ACKed M2 emission to net80211 (sc_tx_data_inflight
		 *     freed normally) but AP never decoded M2; 4-way looped
		 *     on M1 retransmits until session timeout.
		 *   - Linux on the same dongle + AP TX'd M2 at MCS 5 and
		 *     completed the 4-way + DHCP cleanly.
		 * Drop the EAPOL force-CCK-1M and let chip RA pick, matching
		 * Linux's data-path behavior.
		 */
		{
			uint8_t agg_tid = 0;
			w3 = 0;
			if (rtw88_tx_use_ampdu(sc, m, &agg_tid)) {
				uint32_t n = (uint32_t)rtw88_ampdu_max_num;
				w3 |= (n & 0x1FU) <<
				    RTW88_TXD_W3_MAX_AGG_NUM_S;
			}
			/* W4: DATARATE consulted only when W3 USE_RATE is set,
			 * harmless otherwise; matches Linux's default. */
			w4 = (uint32_t)RTW88_TXD_RATE_OFDM_6M;
		}
		le32enc(desc + 12, w3);
		le32enc(desc + 16, w4);

		/* W5: DATA_SHORT=0 DATA_BW=20MHz DATA_LDPC=0 DATA_STBC=0 */
		le32enc(desc + 20, 0);

		/*
		 * W6: SW_DEFINE (bits 11:0) = TX-report serial # when SPE_RPT
		 * is set.  Linux sets bits [7:2] = monotonic counter and keeps
		 * [1:0] zero (per rtw_tx_report_enable in tx.c:175).
		 * sc_tx_report_sn is a per-softc counter.
		 */
		if (is_eapol_early) {
			uint32_t sn = ((uint32_t)(++sc->sc_tx_report_sn) << 2) &
			    0xFCU;
			le32enc(desc + 24, sn);
		} else {
			le32enc(desc + 24, 0);
		}

		/* W7: TXDESC_CHECKSUM zero placeholder; computed below
		 * over W0..W7 after the rest of the desc is finalised. */
		le32enc(desc + 28, 0);

		/* W8: EN_HWSEQ=0 — Linux's mgmt path sets this true, data
		 * path keeps it false.  Data frames carry the seq stamped
		 * by net80211 in the 802.11 hdr; chip must not overwrite. */
		le32enc(desc + 32, 0);

		/* W9: SW_SEQ mirrors the 802.11 SEQ subfield of the frame
		 * header into the descriptor.  Without it the chip's TX
		 * scheduler uses 0 for everything and the AAD seq the AP
		 * computes for CCMP disagrees with what we encrypted under. */
		le32enc(desc + 36, seq << RTW88_TXD_W9_SW_SEQ_S);
	} else {
		/*
		 * Mgmt path.  Match the reference driver's AUTH_REQ TX desc
		 * bit-for-bit (captured at qemu-trace-x86/linux-assoc.pcap
		 * ts 1781907636, w0=0x8430001e w1=0x00081200 w3=0x00000500):
		 *   W0: DIS_QSEL_SEQ (mgmt uses its own seq)
		 *   W1: QSEL_MGMT + RATE_ID = B_20M
		 *   W2[19] = 0x00080000  agg-disable / spe report
		 *   W3: USE_RATE + DISDATAFB (CCK 1M fixed, no fallback)
		 *   W6[3-5] = 0x00000038  rts retry-limit
		 *   W8[15]  = 0x00008000  per-rate offset / sw-define
		 * Without these the AP silently drops the AUTH_REQ.
		 */
		w0 = ((uint32_t)framelen & 0xFFFFU) |
		    ((uint32_t)RTW88_TX_PKT_DESC_SZ << RTW88_TXD_W0_OFFSET_S) |
		    RTW88_TXD_W0_LS |
		    RTW88_TXD_W0_DIS_QSEL_SEQ;
		if (bmc)
			w0 |= RTW88_TXD_W0_BMC;
		le32enc(desc + 0, w0);
		w1 = ((uint32_t)TX_DESC_QSEL_MGMT) << RTW88_TXD_W1_QSEL_S |
		    ((uint32_t)RTW_RATEID_B_20M << RTW88_TXD_W1_RATE_ID_S);
		le32enc(desc + 4, w1);
		le32enc(desc + 8, 0x00080000);
		w3 = RTW88_TXD_W3_USE_RATE | RTW88_TXD_W3_DISDATAFB;
		le32enc(desc + 12, w3);
		w4 = RTW88_TXD_RATE_CCK_1M & RTW88_TXD_W4_DATARATE_M;
		le32enc(desc + 16, w4);
		le32enc(desc + 24, 0x00000038);
		le32enc(desc + 32, 0x00008000);
	}

	m_copydata(m, 0, framelen, desc + RTW88_TX_PKT_DESC_SZ);
	total = RTW88_TX_PKT_DESC_SZ + framelen;

	/*
	 * Promote plain data frames (fc subtype 0) to QoS data
	 * (fc subtype 8) on the wire.  The AP (multi-SSID ASUS family)
	 * advertises HT/QoS in beacon and sends M1/M3 as QoS data;
	 * captures show it MAC-ACKs our plain-data M2 at SIFS but
	 * silently drops at EAPOL.  Forcing QoS framing on outbound
	 * data tests the hypothesis that the AP's EAPOL state machine
	 * requires QoS framing once it has selected QoS for the BSS.
	 *
	 * The shift: existing layout is [hdr=24][body=N]; QoS layout
	 * is [hdr=24][qos=2][body=N].  memmove body up by 2, write
	 * 0x0000 (TID 0 / no AMSDU / normal ACK / EOSP=0) at offset
	 * 24, set fc subtype bit (0x80), grow framelen by 2.
	 *
	 * Only triggers for data frames; mgmt-path keeps fc 24-byte
	 * hdr.  Skipped if already QoS or framelen too small.
	 */
	/*
	 * QoS promotion: plain Data (subtype 0) -> QoS Data (subtype 8)
	 * adds a 2-byte QoS Control field at offset 24.  The TID in QoS
	 * Control MUST match the QSEL in W1:
	 *   - regular data (QSEL=0/TID0/BE): QoS Control = 0x0000
	 *   - EAPOL    (QSEL=7/TID7/VO):     QoS Control = 0x0007  <-- new
	 *
	 * Discovered 2026-06-26 by byte-diffing our M2 against Linux's:
	 * Linux's M2 at offset 0x48 = QoS Control 0x0007 (TID=7), and
	 * its FC byte 0 = 0x88 (QoS Data).  We were emitting plain Data
	 * (FC=0x08) with QSEL=7 -- chip TX scheduler treats EP/QSEL/TID
	 * mismatch as malformed -> chip silently drops at egress.
	 */
	bool is_eapol = is_eapol_early;

	if (is_data && framelen >= 24) {
		uint8_t *fhdr = desc + RTW88_TX_PKT_DESC_SZ;
		uint8_t cur_fc0 = fhdr[0];

		if ((cur_fc0 & 0xF0) == 0x00 &&
		    framelen + 2 + RTW88_TX_PKT_DESC_SZ <= RTW88_TX_BUFSZ) {
			memmove(fhdr + 26, fhdr + 24, framelen - 24);
			/*
			 * QoS Control TID must match QSEL in W1.  Linux
			 * emits EAPOL as QoS Data with TID=7 (VO) and QSEL=7
			 * in the desc (Session 3 usbmon frame 30037:
			 * offset 0x48 = 0x0007).  Regular data stays TID=0.
			 */
			fhdr[24] = is_eapol ? 0x07 : 0x00;
			fhdr[25] = 0x00;
			fhdr[0] = cur_fc0 | 0x80;
			framelen += 2;
			total += 2;
			/*
			 * Re-encode w0's framelen field (bits 15:0) since
			 * the prior encoding used the pre-promotion length.
			 */
			w0 = (w0 & ~(uint32_t)0xFFFFU) |
			    ((uint32_t)framelen & 0xFFFFU);
			le32enc(desc + 0, w0);
		}
	}

	/*
	 * net80211's mgmt-output path sometimes hands us mbufs with
	 * the 802.11 seq_ctrl bytes still zero (notably AUTH frames
	 * built via wpa_supplicant raw_xmit).  AP drops successive
	 * AUTH_REQs with the same seq# as duplicates.  Stamp a
	 * monotonic SW seq when zero so each retry looks fresh.
	 */
	if (framelen >= 24) {
		uint8_t *fhdr = desc + RTW88_TX_PKT_DESC_SZ;
		if (le16dec(fhdr + 22) == 0) {
			sc->sc_tx_seq = (sc->sc_tx_seq + 1) & 0x0FFF;
			le16enc(fhdr + 22,
			    (uint16_t)(sc->sc_tx_seq << 4));
		}
	}

	chksum = 0;
	for (i = 0; i < 16; i++)
		chksum ^= le16dec(desc + i * 2);
	le16enc(desc + 28, chksum);

	/*
	 * Populate TX radiotap header for any active bpf listener.  Chan
	 * freq/flags come from ic_curchan; flags stays 0 (no extra
	 * IEEE80211_RADIOTAP_F_* bits to surface).
	 */
	if (ieee80211_radiotap_active(&sc->sc_ic)) {
		struct ieee80211_channel *cc = sc->sc_ic.ic_curchan;
		sc->sc_txtap.wt_flags = 0;
		sc->sc_txtap.wt_chan_freq = htole16(cc ? cc->ic_freq : 0);
		sc->sc_txtap.wt_chan_flags = htole16(cc ? cc->ic_flags : 0);
	}

	*inflightp = m;

	if (sc->sc_tx_trace != 0) {
		const uint8_t *d = desc;
		const uint8_t *f = desc + RTW88_TX_PKT_DESC_SZ;
		device_printf(sc->sc_dev,
		    "txtrc w0=%08x w1=%08x w3=%08x w4=%08x flen=%u total=%u "
		    "desc=%02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x "
		    "%02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x "
		    "%02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x "
		    "%02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x "
		    "fc=%02x%02x dur=%02x%02x "
		    "a1=%02x:%02x:%02x:%02x:%02x:%02x "
		    "a2=%02x:%02x:%02x:%02x:%02x:%02x "
		    "a3=%02x:%02x:%02x:%02x:%02x:%02x "
		    "seq=%02x%02x body=%02x%02x%02x%02x%02x%02x\n",
		    w0, w1, w3, w4, framelen, total,
		    d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7],
		    d[8], d[9], d[10], d[11], d[12], d[13], d[14], d[15],
		    d[16], d[17], d[18], d[19], d[20], d[21], d[22], d[23],
		    d[24], d[25], d[26], d[27], d[28], d[29], d[30], d[31],
		    d[32], d[33], d[34], d[35], d[36], d[37], d[38], d[39],
		    d[40], d[41], d[42], d[43], d[44], d[45], d[46], d[47],
		    f[0], f[1], f[2], f[3],
		    f[4], f[5], f[6], f[7], f[8], f[9],
		    f[10], f[11], f[12], f[13], f[14], f[15],
		    f[16], f[17], f[18], f[19], f[20], f[21],
		    f[22], f[23],
		    framelen > 24 ? f[24] : 0,
		    framelen > 25 ? f[25] : 0,
		    framelen > 26 ? f[26] : 0,
		    framelen > 27 ? f[27] : 0,
		    framelen > 28 ? f[28] : 0,
		    framelen > 29 ? f[29] : 0);
		sc->sc_tx_trace--;
	}

	pc = usbd_xfer_get_frame(xfer, 0);
	usbd_copy_in(pc, 0, desc, total);
	usbd_xfer_set_frame_len(xfer, 0, total);
	/*
	 * Mirror Linux's URB_ZERO_PACKET flag set on every bulk-OUT
	 * URB (contrib/dev/rtw88/usb.c:384).  FreeBSD's force_short_xfer
	 * only fires a zero-length packet when the transfer size is
	 * already a multiple of MaxPacketSize -- it's a no-op for our
	 * 78-byte AUTH_REQ.  Linux's URB_ZERO_PACKET unconditionally
	 * appends a ZLP regardless of length.  The chip's bulk-OUT
	 * parser appears to need that ZLP as a transfer terminator;
	 * without it AUTH_REQ submission completes USB-side but the
	 * chip never radiates the frame.  Matches our OTA-silent
	 * symptom across 21 prior falsifications.  Set via FreeBSD's
	 * send_zlp flag which is the URB_ZERO_PACKET equivalent.
	 */
	usbd_xfer_set_zlp(xfer);
	usbd_transfer_submit(xfer);
	return (true);
}

static bool
rtw88_bulk_tx_drain_mgmt(struct rtw88_usb_softc *sc, struct usb_xfer *xfer)
{
	return (rtw88_bulk_tx_drain_q(sc, xfer, &sc->sc_tx_mgmt_q,
	    &sc->sc_tx_mgmt_inflight));
}

static bool
rtw88_bulk_tx_drain_data(struct rtw88_usb_softc *sc, struct usb_xfer *xfer)
{
	return (rtw88_bulk_tx_drain_q(sc, xfer, &sc->sc_tx_data_q,
	    &sc->sc_tx_data_inflight));
}

static int
rtw88_bulk_tx_drain_data_vo(struct rtw88_usb_softc *sc, struct usb_xfer *xfer)
{
	return (rtw88_bulk_tx_drain_q(sc, xfer, &sc->sc_tx_data_vo_q,
	    &sc->sc_tx_data_vo_inflight));
}

/*
 * BULK_TX_LO (EP 0x08) callback.  Drains sc_tx_data_q (BE/BK data)
 * matching Linux 8821C rqpn_table[3].dma_map_{be,bk}=LOW -> out_ep[2].
 */
static void
rtw88_bulk_tx_data_be_cb(struct usb_xfer *xfer, usb_error_t error)
{
	struct rtw88_usb_softc *sc = usbd_xfer_softc(xfer);

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
		if (sc->sc_tx_data_inflight != NULL) {
			m_freem(sc->sc_tx_data_inflight);
			sc->sc_tx_data_inflight = NULL;
			sc->sc_stat_tx_packets++;
		}
		/* FALLTHROUGH */
	case USB_ST_SETUP:
		(void)rtw88_bulk_tx_drain_data(sc, xfer);
		break;
	default:
		if (error != USB_ERR_CANCELLED) {
			static struct timeval be_last;
			static const struct timeval be_iv = { 1, 0 };
			sc->sc_stat_tx_errors++;
			counter_u64_add(sc->sc_ic.ic_oerrors, 1);
			if (ratecheck(&be_last, &be_iv))
				device_printf(sc->sc_dev,
				    "tx_data_be error: %s (throttled)\n",
				    usbd_errstr(error));
			if (sc->sc_tx_data_inflight != NULL) {
				m_freem(sc->sc_tx_data_inflight);
				sc->sc_tx_data_inflight = NULL;
			}
			usbd_xfer_set_stall(xfer);
		}
		break;
	}
}

/*
 * BULK_TX_NORMAL (EP 0x06) callback.  Drains sc_tx_data_vo_q (VI/VO data,
 * including EAPOL M2/M4 which mac80211 tags as VO priority).  Matches
 * Linux 8821C rqpn_table[3].dma_map_{vo,vi}=NORMAL -> out_ep[1].
 */
static void
rtw88_bulk_tx_data_vo_cb(struct usb_xfer *xfer, usb_error_t error)
{
	struct rtw88_usb_softc *sc = usbd_xfer_softc(xfer);

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
		if (sc->sc_tx_data_vo_inflight != NULL) {
			m_freem(sc->sc_tx_data_vo_inflight);
			sc->sc_tx_data_vo_inflight = NULL;
			sc->sc_stat_tx_packets++;
		}
		/* FALLTHROUGH -- chain next submission. */
	case USB_ST_SETUP:
		(void)rtw88_bulk_tx_drain_data_vo(sc, xfer);
		break;
	default:
		if (error != USB_ERR_CANCELLED) {
			static struct timeval vo_last;
			static const struct timeval vo_iv = { 1, 0 };
			sc->sc_stat_tx_errors++;
			counter_u64_add(sc->sc_ic.ic_oerrors, 1);
			if (ratecheck(&vo_last, &vo_iv))
				device_printf(sc->sc_dev,
				    "tx_data_vo error: %s (throttled)\n",
				    usbd_errstr(error));
			if (sc->sc_tx_data_vo_inflight != NULL) {
				m_freem(sc->sc_tx_data_vo_inflight);
				sc->sc_tx_data_vo_inflight = NULL;
			}
			usbd_xfer_set_stall(xfer);
		}
		break;
	}
}

static void
rtw88_bulk_tx_cb(struct usb_xfer *xfer, usb_error_t error)
{
	struct rtw88_usb_softc *sc = usbd_xfer_softc(xfer);
	struct usb_page_cache *pc;

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
		if (sc->sc_tx_pending) {
			sc->sc_tx_err = USB_ERR_NORMAL_COMPLETION;
			sc->sc_tx_pending = false;
			sc->sc_tx_done = true;
			cv_signal(&sc->sc_tx_cv);
		}
		if (sc->sc_tx_mgmt_inflight != NULL) {
			m_freem(sc->sc_tx_mgmt_inflight);
			sc->sc_tx_mgmt_inflight = NULL;
			sc->sc_stat_tx_packets++;
		}
		/* FALLTHROUGH -- chain next submission. */
	case USB_ST_SETUP:
		if (sc->sc_tx_pending) {
			pc = usbd_xfer_get_frame(xfer, 0);
			usbd_copy_in(pc, 0, sc->sc_tx_buf, sc->sc_tx_len);
			usbd_xfer_set_frame_len(xfer, 0, sc->sc_tx_len);
			/* Mirror Linux URB_ZERO_PACKET on bulk-OUT sync TX
			 * path (firmware download + H2C bulk).  See the
			 * MGMT TX site for the full rationale. */
			usbd_xfer_set_zlp(xfer);
			usbd_transfer_submit(xfer);
		} else {
			(void)rtw88_bulk_tx_drain_mgmt(sc, xfer);
		}
		break;
	default:
		if (error != USB_ERR_CANCELLED) {
			static struct timeval tx_last;
			static const struct timeval tx_iv = { 1, 0 };
			sc->sc_stat_tx_errors++;
			counter_u64_add(sc->sc_ic.ic_oerrors, 1);
			if (ratecheck(&tx_last, &tx_iv))
				device_printf(sc->sc_dev,
				    "tx error: %s "
				    "(further errors throttled)\n",
				    usbd_errstr(error));
			sc->sc_tx_err = error;
			if (sc->sc_tx_pending) {
				sc->sc_tx_pending = false;
				sc->sc_tx_done = true;
				cv_signal(&sc->sc_tx_cv);
			}
			if (sc->sc_tx_mgmt_inflight != NULL) {
				m_freem(sc->sc_tx_mgmt_inflight);
				sc->sc_tx_mgmt_inflight = NULL;
			}
			usbd_xfer_set_stall(xfer);
		}
		break;
	}
}

/*
 * Submit (data, len) on bulk-OUT pipe pipe_idx and block until the
 * USB callback marks it complete.  Caller must hold sc_mtx; the cv
 * drops + reacquires it across the wait.
 */
static int
rtw88_bulk_tx_sync(struct rtw88_usb_softc *sc, uint8_t pipe_idx,
    const uint8_t *data, uint32_t len)
{
	int error = 0;

	RTW88_ASSERT_LOCKED(sc);

	if (len == 0 || len > RTW88_TX_BUFSZ)
		return (EINVAL);
	if (pipe_idx < RTW88_BULK_TX_HI || pipe_idx > RTW88_BULK_TX_H2C)
		return (EINVAL);

	memcpy(sc->sc_tx_buf, data, len);
	sc->sc_tx_len = len;
	sc->sc_tx_pending = true;
	sc->sc_tx_done = false;
	sc->sc_tx_err = USB_ERR_NORMAL_COMPLETION;

	usbd_transfer_start(sc->sc_xfer[pipe_idx]);

	while (!sc->sc_tx_done) {
		error = cv_timedwait_sig(&sc->sc_tx_cv, &sc->sc_mtx,
		    hz * 6);	/* 6 s — longer than the framework's
				   own 5 s xfer timeout */
		if (error == EWOULDBLOCK || error == EINTR)
			break;
		error = 0;
	}
	if (!sc->sc_tx_done) {
		usbd_transfer_stop(sc->sc_xfer[pipe_idx]);
		sc->sc_tx_pending = false;
		error = ETIMEDOUT;
	} else if (sc->sc_tx_err != USB_ERR_NORMAL_COMPLETION) {
		error = EIO;
	}
	return (error);
}

/*
 * Configure chip-side state needed before any bulk-OUT FW packet
 * will be accepted: enable TX DMA on the host interface, route HIQ
 * traffic to high-priority, prime the FIFO page info / RQPN, and
 * quiesce the beacon engine.  Mirrors reference mac.c
 * download_firmware_reg_backup() minus the save side -- we don't
 * restore these because P2c is invoked once at attach against a
 * fresh chip.
 */
static int
rtw88_dlfw_prep_chip(struct rtw88_usb_softc *sc)
{
	uint8_t bcn_ctrl;
	uint32_t rqpn;
	int error;

	if ((error = rtw88_usb_write1(sc, REG_TXDMA_PQ_MAP + 1,
	    RTW_DMA_MAPPING_HIGH << 6)) != 0)
		return (error);

	if ((error = rtw88_usb_write1(sc, REG_CR,
	    BIT_HCI_TXDMA_EN | BIT_TXDMA_EN)) != 0)
		return (error);

	if ((error = rtw88_usb_write4(sc, REG_H2CQ_CSR, BIT_H2CQ_FULL)) != 0)
		return (error);

	if ((error = rtw88_usb_write2(sc, REG_FIFOPAGE_INFO_1, 0x0200)) != 0)
		return (error);

	if ((error = rtw88_usb_read4(sc, REG_RQPN_CTRL_2, &rqpn)) != 0)
		return (error);
	if ((error = rtw88_usb_write4(sc, REG_RQPN_CTRL_2,
	    rqpn | BIT_LD_RQPN)) != 0)
		return (error);

	if ((error = rtw88_usb_read1(sc, REG_BCN_CTRL, &bcn_ctrl)) != 0)
		return (error);
	bcn_ctrl = (bcn_ctrl & ~BIT_EN_BCN_FUNCTION) | BIT_DIS_TSF_UDT;
	return (rtw88_usb_write1(sc, REG_BCN_CTRL, bcn_ctrl));
}

/*
 * Build a 48-byte 8821C TX descriptor immediately followed by the
 * firmware fragment.  Before submission, arm the chip's rsvd-page
 * staging by writing pg_addr | BIT_BCN_VALID_V1 to REG_FIFOPAGE_CTRL_2
 * and setting BIT_ENSWBCN in REG_CR+1.  After submission, poll
 * REG_FIFOPAGE_CTRL_2 for BIT_BCN_VALID_V1 to confirm the chip
 * accepted the buffer.
 */
static int
rtw88_send_firmware_pkt(struct rtw88_usb_softc *sc, uint16_t pg_addr,
    uint32_t size, const uint8_t *data)
{
	uint8_t *desc;
	uint8_t bcn_ctrl;
	uint32_t total;
	int error;

	if (size == 0 || size > RTW88_TX_BUFSZ - RTW88_TX_PKT_DESC_SZ)
		return (EINVAL);

	if ((error = rtw88_usb_read1(sc, REG_BCN_CTRL, &bcn_ctrl)) != 0)
		return (error);

	if ((error = rtw88_usb_write2(sc, REG_FIFOPAGE_CTRL_2,
	    (uint16_t)((pg_addr & BIT_MASK_BCN_HEAD_1_V1) |
		BIT_BCN_VALID_V1))) != 0)
		return (error);
	if ((error = rtw88_usb_setbits1(sc, REG_CR + 1,
	    BIT_ENSWBCN >> 8)) != 0)
		return (error);
	/*
	 * Per-chunk BCN_CTRL guard: re-clear EN_BCN_FUNCTION and re-set
	 * DIS_TSF_UDT before each rsvd-page write.  reference fw.c
	 * rtw_fw_write_data_rsvd_page() does this every call because
	 * the prior call's "restore" leg writes the original BCN_CTRL
	 * back.  We don't restore, but we still need this on the first
	 * call to overcome whatever the boot ROM left in REG_BCN_CTRL.
	 */
	bcn_ctrl = (bcn_ctrl & ~BIT_EN_BCN_FUNCTION) | BIT_DIS_TSF_UDT;
	if ((error = rtw88_usb_write1(sc, REG_BCN_CTRL, bcn_ctrl)) != 0)
		return (error);

	desc = sc->sc_tx_buf;
	memset(desc, 0, RTW88_TX_PKT_DESC_SZ);
	le32enc(desc + 0,
	    ((size & 0xFFFFu) << RTW88_TXD_W0_TXPKTSIZE_S) |
	    ((uint32_t)RTW88_TX_PKT_DESC_SZ << RTW88_TXD_W0_OFFSET_S) |
	    RTW88_TXD_W0_LS);
	le32enc(desc + 4,
	    ((uint32_t)TX_DESC_QSEL_BEACON) << RTW88_TXD_W1_QSEL_S);
	memcpy(desc + RTW88_TX_PKT_DESC_SZ, data, size);
	total = RTW88_TX_PKT_DESC_SZ + size;

	/*
	 * 8821C TX descriptor checksum: 16-bit XOR over the first 32 bytes
	 * (16 le16 words) treating the checksum field itself as zero.
	 * reference: rtw8821c_fill_txdesc_checksum -> fill_txdesc_checksum_common
	 * with words=16.  Without this the chip silently drops the packet
	 * and bulk OUT times out.
	 */
	{
		uint16_t chksum = 0;
		uint32_t i;

		for (i = 0; i < 16; i++)
			chksum ^= le16dec(desc + i * 2);
		le16enc(desc + 28, chksum);
	}

	if ((error = rtw88_bulk_tx_sync(sc, RTW88_BULK_TX_HI, desc, total))
	    != 0)
		return (error);

	return (rtw88_check_hw_ready(sc, REG_FIFOPAGE_CTRL_2,
	    BIT_BCN_VALID_V1, BIT_BCN_VALID_V1));
}

/*
 * Trigger DDMA channel 0 — chip-internal DMA from TX FIFO to
 * IMEM/DMEM/EMEM.  src/dst are OCP-style 32-bit addresses on the
 * chip; ctrl mixes BIT_DDMACH0_OWN (kick) with optional
 * BIT_DDMACH0_CHKSUM_*.
 */
static int
rtw88_iddma_chunk(struct rtw88_usb_softc *sc, uint32_t src, uint32_t dst,
    uint32_t len, bool first)
{
	uint32_t ctrl;
	int error;

	error = rtw88_check_hw_ready(sc, REG_DDMA_CH0CTRL,
	    BIT_DDMACH0_OWN, 0);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "iddma busy at start (src=0x%08x dst=0x%08x)\n", src, dst);
		return (error);
	}

	ctrl = BIT_DDMACH0_CHKSUM_EN | BIT_DDMACH0_OWN |
	    (len & BIT_MASK_DDMACH0_DLEN);
	if (!first)
		ctrl |= BIT_DDMACH0_CHKSUM_CONT;

	if ((error = rtw88_usb_write4(sc, REG_DDMA_CH0SA, src)) == 0 &&
	    (error = rtw88_usb_write4(sc, REG_DDMA_CH0DA, dst)) == 0)
		error = rtw88_usb_write4(sc, REG_DDMA_CH0CTRL, ctrl);
	if (error != 0)
		return (error);

	return (rtw88_check_hw_ready(sc, REG_DDMA_CH0CTRL,
	    BIT_DDMACH0_OWN, 0));
}

/*
 * After a segment is fully DMA'd, ack the per-segment OK bits in
 * REG_MCUFW_CTRL so the FW READY mask satisfies at end of flow.
 * reference mac.c check_fw_checksum() does this.  Returns false if
 * REG_DDMA_CH0CTRL still has CHKSUM_STS set (DDMA flagged a checksum
 * mismatch on this segment).
 */
static bool
rtw88_segment_checksum_ack(struct rtw88_usb_softc *sc, uint32_t dst_addr)
{
	uint8_t fw_ctrl;
	uint32_t ddma_ctrl;
	bool ok;

	if (rtw88_usb_read1(sc, REG_MCUFW_CTRL, &fw_ctrl) != 0)
		return (false);
	if (rtw88_usb_read4(sc, REG_DDMA_CH0CTRL, &ddma_ctrl) != 0)
		return (false);

	ok = (ddma_ctrl & BIT_DDMACH0_CHKSUM_STS) == 0;
	/* DMEM addr starts at 0x00200000 on 8821C; below that = IMEM/EMEM. */
	if (dst_addr < 0x00200000) {
		fw_ctrl |= BIT_IMEM_DW_OK;
		if (ok)
			fw_ctrl |= BIT_IMEM_CHKSUM_OK;
		else
			fw_ctrl &= ~BIT_IMEM_CHKSUM_OK;
	} else {
		fw_ctrl |= BIT_DMEM_DW_OK;
		if (ok)
			fw_ctrl |= BIT_DMEM_CHKSUM_OK;
		else
			fw_ctrl &= ~BIT_DMEM_CHKSUM_OK;
	}
	(void)rtw88_usb_write1(sc, REG_MCUFW_CTRL, fw_ctrl);
	return (ok);
}

/*
 * Push (data, size) bytes into chip RAM at dst.  Splits into ≤4 KiB
 * chunks; each chunk is sent through TX FIFO then DMA'd into place.
 * The first chunk arms the checksum, subsequent chunks chain.
 */
static int
rtw88_download_firmware_segment(struct rtw88_usb_softc *sc, uint32_t dst,
    const uint8_t *data, uint32_t size)
{
	const uint32_t max_chunk = 0x1000;
	uint32_t off = 0, remain = size, chunk;
	uint32_t val32;
	bool first = true;
	int error;

	/* Arm the checksum-status reset bit. */
	error = rtw88_usb_read4(sc, REG_DDMA_CH0CTRL, &val32);
	if (error != 0)
		return (error);
	error = rtw88_usb_write4(sc, REG_DDMA_CH0CTRL,
	    val32 | BIT_DDMACH0_RESET_CHKSUM_STS);
	if (error != 0)
		return (error);

	while (remain > 0) {
		chunk = (remain > max_chunk) ? max_chunk : remain;
		/*
		 * Page address and OCP TXBUF source stay at 0 for every
		 * chunk -- reference mac.c download_firmware_to_mem() reuses
		 * the same FIFO slot, advancing only the segment offset
		 * in chip RAM (dst).  Page 0 means the rsvd-page mechanism
		 * stages our chunk at the head of the FIFO, then DDMA
		 * copies it out, freeing the slot for the next chunk.
		 */
		error = rtw88_send_firmware_pkt(sc, 0, chunk, data + off);
		if (error != 0) {
			device_printf(sc->sc_dev,
			    "send_firmware_pkt off=%u chunk=%u: %d\n",
			    off, chunk, error);
			return (error);
		}
		error = rtw88_iddma_chunk(sc,
		    OCPBASE_TXBUF_88XX + RTW88_TX_PKT_DESC_SZ,
		    dst + off, chunk, first);
		if (error != 0) {
			device_printf(sc->sc_dev,
			    "iddma_chunk dst=0x%08x off=%u: %d\n",
			    dst + off, off, error);
			return (error);
		}
		first = false;
		off += chunk;
		remain -= chunk;
	}

	if (!rtw88_segment_checksum_ack(sc, dst)) {
		device_printf(sc->sc_dev,
		    "segment dst=0x%08x: DDMA flagged checksum mismatch\n",
		    dst);
		return (EIO);
	}
	return (0);
}

/*
 * Toggle the WCPU 3081 CPU enable bit.  When `enable` is false the
 * CPU is held in reset (firmware download mode); true releases it.
 */
static int
rtw88_wcpu_enable(struct rtw88_usb_softc *sc, bool enable)
{
	int error;

	if (enable) {
		error = rtw88_usb_setbits1(sc, REG_RSV_CTRL + 1,
		    BIT_WLMCU_IOIF >> 8);
		if (error == 0)
			error = rtw88_usb_setbits1(sc, REG_SYS_FUNC_EN + 1,
			    BIT_FEN_CPUEN >> 8);
	} else {
		error = rtw88_usb_clrbits1(sc, REG_SYS_FUNC_EN + 1,
		    BIT_FEN_CPUEN >> 8);
		if (error == 0)
			error = rtw88_usb_clrbits1(sc, REG_RSV_CTRL + 1,
			    BIT_WLMCU_IOIF >> 8);
	}
	return (error);
}

/*
 * Reset the WL platform around firmware download — toggles the
 * platform-reset and CPU-clock-enable bits in two registers.
 */
static int
rtw88_reset_platform(struct rtw88_usb_softc *sc)
{
	int error;

	if ((error = rtw88_usb_clrbits1(sc, REG_CPU_DMEM_CON + 2,
	    BIT_WL_PLATFORM_RST >> 16)) != 0)
		return (error);
	if ((error = rtw88_usb_clrbits1(sc, REG_SYS_CLK_CTRL + 1,
	    BIT_CPU_CLK_EN >> 8)) != 0)
		return (error);
	if ((error = rtw88_usb_setbits1(sc, REG_CPU_DMEM_CON + 2,
	    BIT_WL_PLATFORM_RST >> 16)) != 0)
		return (error);
	return (rtw88_usb_setbits1(sc, REG_SYS_CLK_CTRL + 1,
	    BIT_CPU_CLK_EN >> 8));
}

/*
 * Power-sequence parser.  Walks a NULL-terminated array of command
 * tables, applying the rows that match our HCI interface (USB).
 * Implements the three opcodes the 8821C tables actually use --
 * WRITE (read-modify-write byte), POLLING (busy-wait byte mask),
 * DELAY (us or ms).  READ is decoded only to satisfy the table
 * layout; no 8821C row uses it.  Mirrors reference mac.c
 * rtw_pwr_seq_parser() with USB-only intf filter and the cut-mask
 * pinned to "any cut" (we don't yet read REG_SYS_CFG1[15:12]).
 */
static int
rtw88_pwr_seq_parser(struct rtw88_usb_softc *sc,
    const struct rtw88_pwr_seq_cmd * const *seq)
{
	const struct rtw88_pwr_seq_cmd *row;
	uint8_t value;
	uint32_t idx;
	int error, try;

	for (idx = 0; seq[idx] != NULL; idx++) {
		for (row = seq[idx]; row->cmd != RTW_PWR_CMD_END; row++) {
			if ((row->intf_mask & RTW_PWR_INTF_USB_MSK) == 0)
				continue;
			if (row->base != RTW_PWR_ADDR_MAC)
				continue;

			switch (row->cmd) {
			case RTW_PWR_CMD_WRITE:
				error = rtw88_usb_read1(sc, row->offset,
				    &value);
				if (error != 0)
					return (error);
				value &= ~row->mask;
				value |= (row->value & row->mask);
				error = rtw88_usb_write1(sc, row->offset,
				    value);
				if (error != 0)
					return (error);
				break;
			case RTW_PWR_CMD_POLLING:
				for (try = 0; try < RTW_PWR_POLLING_CNT;
				     try++) {
					error = rtw88_usb_read1(sc,
					    row->offset, &value);
					if (error != 0)
						return (error);
					if ((value & row->mask) ==
					    (row->value & row->mask))
						break;
					DELAY(50);
				}
				if (try == RTW_PWR_POLLING_CNT) {
					device_printf(sc->sc_dev,
					    "pwr_seq poll fail off=0x%04x"
					    " mask=0x%02x val=0x%02x"
					    " got=0x%02x\n",
					    row->offset, row->mask,
					    row->value, value);
					return (EBUSY);
				}
				break;
			case RTW_PWR_CMD_DELAY:
				if (row->value == RTW_PWR_DELAY_US)
					DELAY(row->offset);
				else
					usb_pause_mtx(&sc->sc_mtx,
					    USB_MS_TO_TICKS(row->offset));
				break;
			default:
				return (EINVAL);
			}
		}
	}
	return (0);
}

/*
 * Pre-power-on system config -- USB path of reference mac.c
 * rtw_mac_pre_system_cfg() for WCPU 3081 silicon.  Drops the IO
 * reservation byte, swings the PA/LNA pin mux to WL/BT-shared mode,
 * disables the BB / RF / WLRF blocks (firmware will re-enable later
 * during init_mac()).
 */
static int
rtw88_mac_pre_system_cfg(struct rtw88_usb_softc *sc)
{
	uint32_t val32;
	uint8_t val8;
	int error;

	if ((error = rtw88_usb_write1(sc, REG_RSV_CTRL, 0)) != 0)
		return (error);

	if ((error = rtw88_usb_read4(sc, REG_PAD_CTRL1, &val32)) != 0)
		return (error);
	val32 |= BIT_PAPE_WLBT_SEL | BIT_LNAON_WLBT_SEL;
	if ((error = rtw88_usb_write4(sc, REG_PAD_CTRL1, val32)) != 0)
		return (error);

	if ((error = rtw88_usb_read4(sc, REG_LED_CFG, &val32)) != 0)
		return (error);
	val32 &= ~(BIT_PAPE_SEL_EN | BIT_LNAON_SEL_EN);
	if ((error = rtw88_usb_write4(sc, REG_LED_CFG, val32)) != 0)
		return (error);

	if ((error = rtw88_usb_read4(sc, REG_GPIO_MUXCFG, &val32)) != 0)
		return (error);
	val32 |= BIT_WLRFE_4_5_EN;
	if ((error = rtw88_usb_write4(sc, REG_GPIO_MUXCFG, val32)) != 0)
		return (error);

	if ((error = rtw88_usb_read1(sc, REG_SYS_FUNC_EN, &val8)) != 0)
		return (error);
	val8 &= ~(BIT_FEN_BB_RSTB | BIT_FEN_BB_GLB_RST);
	if ((error = rtw88_usb_write1(sc, REG_SYS_FUNC_EN, val8)) != 0)
		return (error);

	if ((error = rtw88_usb_read1(sc, REG_RF_CTRL, &val8)) != 0)
		return (error);
	val8 &= ~(BIT_RF_SDM_RSTB | BIT_RF_RSTB | BIT_RF_EN);
	if ((error = rtw88_usb_write1(sc, REG_RF_CTRL, val8)) != 0)
		return (error);

	if ((error = rtw88_usb_read4(sc, REG_WLRF1, &val32)) != 0)
		return (error);
	val32 &= ~BIT_WLRF1_BBRF_EN;
	return (rtw88_usb_write4(sc, REG_WLRF1, val32));
}

/*
 * Post-power-on register tweaks -- USB path of reference mac.c
 * __rtw_mac_init_system_cfg() for the WCPU 3081 family.  Asserts
 * PLATFORM_RST and DDMA_EN in REG_CPU_DMEM_CON, lifts SYS_FUNC_EN
 * to the 8821C chip-specific mask (0xD8), forces CR_EXT[31:24] low
 * nibble to 0x0C, and clears any flash-boot bits left by the boot
 * ROM so our DDMA-based firmware download wins.
 */
static int
rtw88_mac_init_system_cfg(struct rtw88_usb_softc *sc)
{
	uint32_t val32;
	uint8_t val8;
	int error;

	if ((error = rtw88_usb_read4(sc, REG_CPU_DMEM_CON, &val32)) != 0)
		return (error);
	val32 |= BIT_WL_PLATFORM_RST | BIT_DDMA_EN;
	if ((error = rtw88_usb_write4(sc, REG_CPU_DMEM_CON, val32)) != 0)
		return (error);

	if ((error = rtw88_usb_setbits1(sc, REG_SYS_FUNC_EN + 1,
	    0xD8)) != 0)
		return (error);

	if ((error = rtw88_usb_read1(sc, REG_CR_EXT + 3, &val8)) != 0)
		return (error);
	val8 = (val8 & 0xF0) | 0x0C;
	if ((error = rtw88_usb_write1(sc, REG_CR_EXT + 3, val8)) != 0)
		return (error);

	if ((error = rtw88_usb_read4(sc, REG_MCUFW_CTRL, &val32)) != 0)
		return (error);
	if ((val32 & BIT_BOOT_FSPI_EN) != 0) {
		val32 &= ~BIT_BOOT_FSPI_EN;
		if ((error = rtw88_usb_write4(sc, REG_MCUFW_CTRL,
		    val32)) != 0)
			return (error);
		if ((error = rtw88_usb_read4(sc, REG_GPIO_MUXCFG,
		    &val32)) != 0)
			return (error);
		val32 &= ~BIT_FSPI_EN;
		if ((error = rtw88_usb_write4(sc, REG_GPIO_MUXCFG,
		    val32)) != 0)
			return (error);
	}
	return (0);
}

/*
 * Detect whether the MAC is in the ACT (powered-on) state.  Mirrors
 * reference mac.c rtw_mac_power_switch().  Returns true if powered.
 *   REG_CR == 0xea           -> boot ROM idle, MAC OFF
 *   SYS_STATUS1+1 bit 0 set  -> USB PFM stuck, MAC effectively OFF
 *   anything else            -> MAC is ACT
 */
static bool
rtw88_mac_currently_on(struct rtw88_usb_softc *sc)
{
	uint8_t cr, sysstat1_1;

	if (rtw88_usb_read1(sc, REG_CR, &cr) != 0)
		return (false);
	if (cr == 0xEA)
		return (false);
	if (rtw88_usb_read1(sc, REG_SYS_STATUS1 + 1, &sysstat1_1) != 0)
		return (false);
	if ((sysstat1_1 & 0x01) != 0)
		return (false);
	return (true);
}

/*
 * Run the card_disable_flow_8821c sequence to return the chip to the
 * boot-ROM "card-disabled" state.  Used when re-attaching to a chip
 * that's already past cardemu (e.g. firmware still loaded from a
 * previous kldload).
 */
static int
rtw88_mac_power_off(struct rtw88_usb_softc *sc)
{
	int error;

	error = rtw88_pwr_seq_parser(sc, card_disable_flow_8821c);
	if (error != 0)
		device_printf(sc->sc_dev,
		    "card_disable_flow_8821c failed: %d\n", error);
	return (error);
}

/*
 * Apply the carddis->cardemu->act sequence then post-power-on system
 * config.  Caller owns the EALREADY retry decision.
 */
static int
rtw88_mac_power_up_once(struct rtw88_usb_softc *sc)
{
	uint8_t val8;
	int error;

	if ((error = rtw88_mac_pre_system_cfg(sc)) != 0) {
		device_printf(sc->sc_dev, "pre_system_cfg failed: %d\n",
		    error);
		return (error);
	}

	if ((error = rtw88_pwr_seq_parser(sc, card_enable_flow_8821c)) != 0) {
		device_printf(sc->sc_dev,
		    "card_enable_flow_8821c failed: %d\n", error);
		return (error);
	}

	/*
	 * USB + 8821C: clear the PFM-stuck flag that the boot ROM may
	 * have left set in REG_SYS_STATUS1+1.  reference gates this on
	 * chip id == 8821C / 8822B / 8822C; we only ship 8821C so the
	 * write is unconditional.
	 */
	if ((error = rtw88_usb_read1(sc, REG_SYS_STATUS1 + 1, &val8)) != 0)
		return (error);
	val8 &= ~0x01;
	if ((error = rtw88_usb_write1(sc, REG_SYS_STATUS1 + 1, val8)) != 0)
		return (error);

	return (rtw88_mac_init_system_cfg(sc));
}

/*
 * MAC power-on -- mirror of reference mac.c rtw_mac_power_on() for the
 * USB path with the EALREADY retry.  If the chip is already in ACT
 * (typically because the prior kldload left firmware running) we
 * have to take it down through cardemu -> carddis first so the
 * card_enable_flow tables can start from a known state.  Without
 * this re-kldload wedges EP0 the moment pwr_seq tries to clear
 * REG_RSV_CTRL etc.
 */
static int
rtw88_mac_power_on(struct rtw88_usb_softc *sc)
{
	int error;

	if (rtw88_mac_currently_on(sc)) {
		device_printf(sc->sc_dev,
		    "mac_power_on: chip already ACT (stale FW); cycling\n");
		if ((error = rtw88_mac_power_off(sc)) != 0)
			return (error);
	}

	if ((error = rtw88_mac_power_up_once(sc)) != 0)
		return (error);

	/* Coex bring-up now runs through the subsystem TU (rtw88_coex.c).
	 * That function itself takes rtwdev->mtx internally, so callers
	 * from lock-held contexts must drop it first.  mac_power_on runs
	 * unlocked; safe.  */
	(void)rtw88_coex_init(&sc->sc_rtwdev);

	device_printf(sc->sc_dev, "mac_power_on: ACT state reached\n");
	return (0);
}

/*
 * Full 8821C firmware-download orchestrator.  Returns 0 if the FW
 * READY bits in REG_MCUFW_CTRL come up within the timeout window.
 */
static int
rtw88_download_firmware(struct rtw88_usb_softc *sc)
{
	struct rtw88_fw_hdr_summary h;
	const uint8_t *data, *segdata;
	uint32_t size, val32, dmem_size, imem_size, emem_size;
	int error;

	if (sc->sc_fw == NULL) {
		device_printf(sc->sc_dev, "no firmware acquired\n");
		return (ENOENT);
	}
	data = sc->sc_fw->data;
	size = sc->sc_fw->datasize;
	rtw88_fw_hdr_decode(data, &h);

	dmem_size = h.dmem_size + 8;	/* +FW_HDR_CHKSUM_SIZE */
	imem_size = h.imem_size + 8;
	emem_size = (h.mem_usage & 0x10u) ? (h.emem_size + 8) : 0;

	if (RTW88_FW_HDR_SIZE + dmem_size + imem_size + emem_size != size) {
		device_printf(sc->sc_dev,
		    "fw size mismatch: hdr+dmem+imem+emem=%u file=%u\n",
		    (unsigned)(RTW88_FW_HDR_SIZE + dmem_size + imem_size +
			emem_size), size);
		return (EINVAL);
	}

	device_printf(sc->sc_dev, "download_firmware: starting\n");

	RTW88_LOCK(sc);

	/*
	 * Bring the MAC out of disabled state -- mandatory gate before
	 * any bulk-OUT TX FIFO write will be accepted.  Without this the
	 * chip ignores the first DMEM packet and the bulk pipe times
	 * out at ~6 s.
	 */
	if ((error = rtw88_mac_power_on(sc)) != 0)
		goto out;

	/* Disable WCPU. */
	if ((error = rtw88_wcpu_enable(sc, false)) != 0)
		goto out;
	/* TX DMA + FIFO + beacon prep so the chip accepts bulk packets. */
	if ((error = rtw88_dlfw_prep_chip(sc)) != 0)
		goto out;
	/* Reset platform. */
	if ((error = rtw88_reset_platform(sc)) != 0)
		goto out;

	/* Arm download-enable bit in REG_MCUFW_CTRL (16-bit access). */
	if ((error = rtw88_usb_read4(sc, REG_MCUFW_CTRL, &val32)) != 0)
		goto out;
	error = rtw88_usb_write2(sc, REG_MCUFW_CTRL,
	    (uint16_t)((val32 & 0x3800) | BIT_MCUFWDL_EN));
	if (error != 0)
		goto out;


	/* DMEM */
	segdata = data + RTW88_FW_HDR_SIZE;
	error = rtw88_download_firmware_segment(sc,
	    h.dmem_addr & ~(1U << 31), segdata, dmem_size);
	if (error != 0) {
		device_printf(sc->sc_dev, "DMEM segment failed: %d\n", error);
		goto out;
	}

	/* IMEM */
	segdata += dmem_size;
	error = rtw88_download_firmware_segment(sc,
	    h.imem_addr & ~(1U << 31), segdata, imem_size);
	if (error != 0) {
		device_printf(sc->sc_dev, "IMEM segment failed: %d\n", error);
		goto out;
	}

	/* EMEM (optional) */
	if (emem_size > 0) {
		segdata += imem_size;
		error = rtw88_download_firmware_segment(sc,
		    h.emem_addr & ~(1U << 31), segdata, emem_size);
		if (error != 0) {
			device_printf(sc->sc_dev,
			    "EMEM segment failed: %d\n", error);
			goto out;
		}
	}

	/* End-flow: write TXDMA status + close download window. */
	(void)rtw88_usb_write4(sc, REG_TXDMA_STATUS, BTI_PAGE_OVF);

	if ((error = rtw88_usb_read4(sc, REG_MCUFW_CTRL, &val32)) != 0)
		goto out;
	if ((val32 & BIT_CHECK_SUM_OK) != BIT_CHECK_SUM_OK) {
		device_printf(sc->sc_dev,
		    "checksum fail: REG_MCUFW_CTRL=0x%08x\n", val32);
		error = EIO;
		goto out;
	}
	error = rtw88_usb_write2(sc, REG_MCUFW_CTRL,
	    (uint16_t)(((val32 | BIT_FW_DW_RDY) & ~BIT_MCUFWDL_EN) & 0xFFFFu));
	if (error != 0)
		goto out;

	/* Re-enable WCPU. */
	if ((error = rtw88_wcpu_enable(sc, true)) != 0)
		goto out;

	/* Validate READY bits. */
	error = rtw88_check_hw_ready(sc, REG_MCUFW_CTRL, FW_READY_MASK,
	    FW_READY);
	if (error != 0) {
		(void)rtw88_usb_read4(sc, REG_MCUFW_CTRL, &val32);
		device_printf(sc->sc_dev,
		    "fw not ready after download: REG_MCUFW_CTRL=0x%08x\n",
		    val32);
		goto out;
	}

	device_printf(sc->sc_dev, "download_firmware: READY\n");
out:
	RTW88_UNLOCK(sc);
	return (error);
}

/*
 * net80211 vap_create.  Single-STA driver; second clone or non-STA
 * opmode is rejected (per-softc state is not VAP-replicated).
 */
/*
 * 5 GHz channel-number tables.  These mirror the rtwn driver
 * (sys/dev/rtwn/rtl8812a/r12a.h) and match the channels reference's
 * rtw88 advertises in its sband for the 8821C family: UNII-1 + 2A
 * (with DFS) + 2C + 3.
 */
static const uint8_t rtw88_chan_5ghz_0[] =
    { 36, 40, 44, 48, 52, 56, 60, 64 };
static const uint8_t rtw88_chan_5ghz_1[] =
    { 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144 };
static const uint8_t rtw88_chan_5ghz_2[] =
    { 149, 153, 157, 161, 165, 169, 173, 177 };

static void
rtw88_getradiocaps(struct ieee80211com *ic, int maxchans, int *nchans,
    struct ieee80211_channel chans[])
{
	struct rtw88_usb_softc *sc = ic->ic_softc;
	uint8_t bands[IEEE80211_MODE_BYTES];

	memset(bands, 0, sizeof(bands));
	if (sc->sc_chip->caps & RTW88_CAP_2GHZ) {
		int cbw = 0;
		setbit(bands, IEEE80211_MODE_11B);
		setbit(bands, IEEE80211_MODE_11G);
		setbit(bands, IEEE80211_MODE_11NG);
		if (sc->sc_chip->caps & RTW88_CAP_HT40)
			cbw |= NET80211_CBW_FLAG_HT40;
		ieee80211_add_channels_default_2ghz(chans, maxchans, nchans,
		    bands, cbw);
	}

	if (sc->sc_chip->caps & RTW88_CAP_5GHZ) {
		int cbw = 0;
		setbit(bands, IEEE80211_MODE_11A);
		setbit(bands, IEEE80211_MODE_11NA);
		if (sc->sc_chip->caps & RTW88_CAP_HT40)
			cbw |= NET80211_CBW_FLAG_HT40;
		if (sc->sc_chip->caps & RTW88_CAP_VHT80) {
			setbit(bands, IEEE80211_MODE_VHT_5GHZ);
			cbw |= NET80211_CBW_FLAG_VHT80;
		}
		ieee80211_add_channel_list_5ghz(chans, maxchans, nchans,
		    rtw88_chan_5ghz_0, nitems(rtw88_chan_5ghz_0), bands, cbw);
		ieee80211_add_channel_list_5ghz(chans, maxchans, nchans,
		    rtw88_chan_5ghz_1, nitems(rtw88_chan_5ghz_1), bands, cbw);
		ieee80211_add_channel_list_5ghz(chans, maxchans, nchans,
		    rtw88_chan_5ghz_2, nitems(rtw88_chan_5ghz_2), bands, cbw);
	}
}

/*
 * Net80211 calls ic_newassoc when a peer (AP for STA mode) is added
 * to the node table after association.  We have no per-peer state to
 * push to the chip yet (no RA table, no per-MACID HW key install --
 * software crypto handles it), so this is a no-op.  Without the hook
 * set net80211 will SIGSEGV-equivalent kernel panic.
 */
static void
rtw88_newassoc(struct ieee80211_node *ni __unused, int isnew __unused)
{
}

static struct ieee80211vap *
rtw88_vap_create(struct ieee80211com *ic, const char name[IFNAMSIZ], int unit,
    enum ieee80211_opmode opmode, int flags,
    const uint8_t bssid[IEEE80211_ADDR_LEN],
    const uint8_t mac[IEEE80211_ADDR_LEN])
{
	struct rtw88_vap *rvp;
	struct ieee80211vap *vap;

	/*
	 * Single-VAP driver: TX/RX queues, sc_vap pointer, and CAM PTK
	 * slot are all per-softc, not per-vap.  Reject any additional
	 * clone attempt cleanly.  STA and MONITOR are accepted; other
	 * modes (HOSTAP, IBSS, MESH, etc.) need infrastructure we don't
	 * have.
	 */
	if (!TAILQ_EMPTY(&ic->ic_vaps))
		return (NULL);
	if (opmode != IEEE80211_M_STA && opmode != IEEE80211_M_MONITOR)
		return (NULL);

	rvp = malloc(sizeof(*rvp), M_80211_VAP, M_WAITOK | M_ZERO);
	vap = &rvp->vap;

	if (ieee80211_vap_setup(ic, vap, name, unit, opmode,
	    flags | IEEE80211_CLONE_NOBEACONS, bssid) != 0) {
		free(rvp, M_80211_VAP);
		return (NULL);
	}

	rvp->newstate = vap->iv_newstate;
	vap->iv_newstate = rtw88_newstate;
	/*
	 * iv_key_set / iv_key_delete enqueue HW CAM install/clear ops on
	 * sc_cam_task.  Under the default SW CCMP path (hw_ccmp=0) the chip
	 * security engine is off, so the CAM writes land harmlessly while
	 * net80211 SW-encrypts the body.  Under hw_ccmp=1 the chip's TX
	 * engine consults CAM by macid and encrypts the body; firmware-
	 * emitted NULL frames (keep-alive) are CCMP-encrypted by the chip.
	 */
	vap->iv_key_set = rtw88_vap_key_set;
	vap->iv_key_delete = rtw88_vap_key_delete;

	ieee80211_vap_attach(vap, ieee80211_media_change,
	    ieee80211_media_status, mac);
	ic->ic_opmode = opmode;
	return (vap);
}

static void
rtw88_vap_delete(struct ieee80211vap *vap)
{
	struct rtw88_vap *rvp = RTW88_VAP(vap);

	ieee80211_vap_detach(vap);
	free(rvp, M_80211_VAP);
}

/*
 * Direct-MMIO RF read on path A.  reference rtw_phy_read_rf() with
 * 8821C's rf_base_addr[A] = 0x2800: each RF register r is shadowed
 * at MMIO offset 0x2800 + (r << 2) and read 32-bit, masked.
 */
static int
rtw88_phy_read_rf_a(struct rtw88_usb_softc *sc, uint8_t addr, uint32_t mask,
    uint32_t *val)
{
	uint32_t raw;
	int error;

	RTW88_ASSERT_LOCKED(sc);

	error = rtw88_usb_read4(sc,
	    (uint16_t)(RTW88_RF_PATH_A_BASE + ((uint32_t)addr << 2)), &raw);
	if (error != 0)
		return (error);
	*val = raw & mask;
	return (0);
}

/*
 * Send one H2C packet via the BULK_TX_H2C pipe.  Packet is always
 * H2C_PKT_SIZE (32) bytes -- header (8) + body (<= 24); caller
 * populates the body, we fill the header and total_len.  The TX
 * descriptor is 48 bytes with QSEL_H2C; checksum is the same 16-bit
 * XOR over the first 32 desc bytes used elsewhere.
 *
 * reference rtw_fw_send_h2c_packet -> rtw_usb_write_data_h2c ->
 * rtw_usb_write_data.
 */
#define	RTW88_H2C_PKT_SIZE		32
#define	RTW88_H2C_PKT_HDR_SIZE		8
#define	RTW88_H2C_PKT_CATEGORY		0x01
#define	RTW88_H2C_PKT_CMD_ID		0xFF
#define	RTW88_H2C_SUB_IQK		0x0E
#define	RTW88_H2C_SUB_GENERAL_INFO	0x0D
#define	RTW88_H2C_SUB_PHYDM_INFO	0x11
#define	RTW88_FW_RF_1T1R		4	/* matches Linux FW_RF_1T1R */

static int
rtw88_h2c_packet(struct rtw88_usb_softc *sc, uint8_t sub_id,
    const uint8_t *body, uint16_t body_len)
{
	uint8_t buf[RTW88_TX_PKT_DESC_SZ + RTW88_H2C_PKT_SIZE];
	uint8_t *desc = buf;
	uint8_t *pkt = buf + RTW88_TX_PKT_DESC_SZ;
	uint32_t w0, w1, hdr0, hdr1;
	uint16_t chksum;
	uint16_t total_len;
	int i;

	RTW88_ASSERT_LOCKED(sc);

	if (body_len > RTW88_H2C_PKT_SIZE - RTW88_H2C_PKT_HDR_SIZE)
		return (EINVAL);
	total_len = RTW88_H2C_PKT_HDR_SIZE + body_len;

	memset(buf, 0, sizeof(buf));

	/*
	 * H2C packet header (8 bytes), per reference rtw_h2c_pkt_set_header +
	 * SET_PKT_H2C_TOTAL_LEN (fw.h SET_PKT_H2C_* macros):
	 *   word[0]: category[6:0] | cmd_id[15:8] | sub_cmd_id[31:16]
	 *   word[1]: total_len[15:0]
	 * reference rtw_fw_do_iqk passes total_size = H2C_PKT_HDR_SIZE + 1.
	 */
	hdr0 = (uint32_t)(RTW88_H2C_PKT_CATEGORY & 0x7FU) |
	    ((uint32_t)RTW88_H2C_PKT_CMD_ID << 8) |
	    ((uint32_t)sub_id << 16);
	hdr1 = (uint32_t)total_len;
	le32enc(pkt + 0, hdr0);
	le32enc(pkt + 4, hdr1);
	if (body != NULL && body_len > 0)
		memcpy(pkt + RTW88_H2C_PKT_HDR_SIZE, body, body_len);

	/*
	 * H2C TX descriptor.  reference rtw_usb_write_data_h2c sets ONLY
	 * tx_pkt_size + qsel in pkt_info; offset, ls, bmc all stay 0.
	 * Setting OFFSET=desc_sz makes the firmware skip past the H2C
	 * packet into the trailing zero fill and ignore the command.
	 */
	w0 = ((uint32_t)RTW88_H2C_PKT_SIZE & 0xFFFFU);
	le32enc(desc + 0, w0);
	w1 = ((uint32_t)TX_DESC_QSEL_H2C) << RTW88_TXD_W1_QSEL_S;
	le32enc(desc + 4, w1);

	/* 16-bit XOR checksum over first 32 desc bytes. */
	chksum = 0;
	for (i = 0; i < 16; i++)
		chksum ^= le16dec(desc + i * 2);
	le16enc(desc + 28, chksum);

	return (rtw88_bulk_tx_sync(sc, RTW88_BULK_TX_H2C, buf, sizeof(buf)));
}

/*
 * Kick a firmware-driven IQ calibration.  H2C body is one byte:
 * bit 0 = clear, bit 1 = segment_iqk.  reference rtw_fw_do_iqk; before
 * association we always pass clear=0, segment=0.
 */
static int
rtw88_fw_do_iqk(struct rtw88_usb_softc *sc, bool segment)
{
	uint8_t body = segment ? 0x02 : 0x00;

	return (rtw88_h2c_packet(sc, RTW88_H2C_SUB_IQK, &body, 1));
}

/*
 * GENERAL_INFO H2C: tells firmware the reserved-page TX boundary.
 * Linux fires this from rtw_core_start (after HCI starts) via
 * rtw_fw_send_general_info; for 8821CU (non-8051 WCPU) the body is
 * 4 bytes carrying FW_TX_BOUNDARY at bits 23:16 of word[2].
 *
 *   FW_TX_BOUNDARY = fifo->rsvd_fw_txbuf_addr - fifo->rsvd_boundary
 *                  = RTW88_RSVD_FW_TXBUF_ADDR (508) - RTW88_RSVD_BOUNDARY (460)
 *                  = 48 = 0x30
 *
 * Matches Linux pcap byte-for-byte: body bytes = 00 00 30 00.
 */
static int
rtw88_h2c_general_info(struct rtw88_usb_softc *sc)
{
	uint8_t body[4];
	uint32_t w;

	/*
	 * FW_TX_BOUNDARY = rsvd_fw_txbuf_addr - rsvd_boundary.  For our
	 * fixed layout (TXFF=512, RSVD=52, FW_TXBUF=4, CSI_BUF=0) this
	 * works out to 508 - 460 = 0x30, which is exactly the value
	 * Linux writes for the same firmware on this chip.  Hardcoded
	 * because the RTW88_RSVD_* constants are defined later in this
	 * file and only-recompute if the page layout ever changes.
	 */
	w = (uint32_t)0x30 << 16;
	le32enc(body, w);
	return (rtw88_h2c_packet(sc, RTW88_H2C_SUB_GENERAL_INFO, body,
	    sizeof(body)));
}

/*
 * PHYDM_INFO H2C: tells firmware's PhyDM module RF/efuse parameters.
 * Linux fires this immediately after GENERAL_INFO from rtw_core_start.
 * Body is 8 bytes packing word[2] (PHYDM params) + word[3] (zero).
 *
 *   word[2] bits  7:0  = REF_TYPE   (efuse->rfe_option)
 *   word[2] bits 15:8  = RF_TYPE    (FW_RF_1T1R = 4 for 8821C)
 *   word[2] bits 23:16 = CUT_VER    (hal->cut_version)
 *   word[2] bits 27:24 = RX_ANT_STATUS (hal->antenna_tx -- swap is in Linux)
 *   word[2] bits 31:28 = TX_ANT_STATUS (hal->antenna_rx)
 *
 * Linux pcap for this chip: body bytes = 02 04 04 11 00 00 00 00.
 * Path A is the only RF path on 8821C 1T1R so RX/TX_ANT both = 1.
 */
static int
rtw88_h2c_phydm_info(struct rtw88_usb_softc *sc)
{
	uint8_t body[8];
	uint32_t w2;

	w2 = ((uint32_t)sc->sc_rfe_option & 0xFFU)
	    | ((uint32_t)RTW88_FW_RF_1T1R << 8)
	    | ((uint32_t)sc->sc_cut_version << 16)
	    | ((uint32_t)1 << 24)	/* RX_ANT_STATUS = path A */
	    | ((uint32_t)1 << 28);	/* TX_ANT_STATUS = path A */
	le32enc(body + 0, w2);
	le32enc(body + 4, 0);
	return (rtw88_h2c_packet(sc, RTW88_H2C_SUB_PHYDM_INFO, body,
	    sizeof(body)));
}

/*
 * Equivalent of reference rtw8821c_phy_calibration -> rtw8821c_do_iqk.
 * Submit the IQK H2C and poll RF_DTXLOK (RF reg 0x08) for the 0xabcde
 * "done" sentinel.  reference caps the poll at 300 x 20 ms = 6 s; we keep
 * the same budget.  REG_IQKFAILMSK (0x1bf0) carries per-path failure
 * bits in the low byte after the FW writes the sentinel.
 */
#define	RTW88_RF_DTXLOK			0x08
#define	RTW88_RFREG_MASK		0xFFFFFU
#define	RTW88_REG_IQKFAILMSK		0x1BF0

static int
rtw88_phy_calibration(struct rtw88_usb_softc *sc)
{
	uint32_t rf_val, fail_mask;
	int error, i;
	bool done = false;

	RTW88_ASSERT_LOCKED(sc);

	error = rtw88_fw_do_iqk(sc, false);
	if (error != 0) {
		device_printf(sc->sc_dev, "iqk h2c submit failed: %d\n",
		    error);
		return (error);
	}

	for (i = 0; i < 300; i++) {
		if (rtw88_phy_read_rf_a(sc, RTW88_RF_DTXLOK, RTW88_RFREG_MASK,
		    &rf_val) == 0 && rf_val == 0xABCDE) {
			done = true;
			break;
		}
		usb_pause_mtx(&sc->sc_mtx, USB_MS_TO_TICKS(20));
	}

	if (rtw88_usb_read4(sc, RTW88_REG_IQKFAILMSK, &fail_mask) != 0)
		fail_mask = ~0U;
	device_printf(sc->sc_dev,
	    "iqk %s iters=%d fail_mask=0x%02x\n",
	    done ? "done" : "TIMEOUT", i, fail_mask & 0xff);

	/*
	 * IQK can write the 0xabcde DONE sentinel while still flagging
	 * per-path failures in IQKFAILMSK low byte (one bit per
	 * TX/RX/LOK path, see Realtek bf docs).  Treating "done" alone
	 * as success masks miscalibrated paths and lets auth proceed
	 * on a partially-uncalibrated radio -- the classic "correct bits,
	 * wrong analog, AP silent" failure mode this sequence is trying
	 * to prevent.  Escalate so failures show up in dmesg without
	 * waiting for auth to silently fail.
	 */
	if (done && (fail_mask & 0xff) != 0) {
		device_printf(sc->sc_dev,
		    "iqk reports DONE but fail_mask=0x%02x -- "
		    "one or more paths miscalibrated, "
		    "radio analog may be off\n",
		    fail_mask & 0xff);
	}

	/*
	 * Clear the DONE sentinel via the SIPI bus, matching reference
	 * rtw8821c_do_iqk's rtw_write_rf(RF_PATH_A, RF_DTXLOK, RFREG_MASK,
	 * 0x0).  The previous implementation poked BB address 0x2820,
	 * which does not reach RF reg 0x08 -- leaving the firmware-set
	 * 0xabcde stuck in place and making every subsequent poll exit
	 * immediately with "iters=0", so IQK never re-ran.
	 */
	(void)rtw88_phy_write_rf_a(sc, RTW88_RF_DTXLOK, 0);

	return (done ? 0 : ETIMEDOUT);
}

/*
 * Equivalent of reference rtw_chip_prepare_tx -> rtw8821c_phy_calibration.
 * Run before any state transition that originates a frame (we hook
 * SCAN -> AUTH in rtw88_newstate) so the radio is calibrated for the
 * channel net80211 just steered us onto.
 */

/*
 * LTE-coex indirect access + GNT_WL/GNT_BT ratchet + 2G BBSW antenna
 * switch + rtw88_coex_cfg_init moved to sys/dev/rtw88/rtw88_coex.c.
 * Call rtw88_coex_init / rtw88_coex_link_up / rtw88_coex_prepare_tx
 * against `&sc->sc_rtwdev` from the surrounding driver code.
 */

static void
rtw88_chip_prepare_tx(struct rtw88_usb_softc *sc)
{

	RTW88_ASSERT_LOCKED(sc);
	/*
	 * Coex GNT + BBSW ant-switch delegated to rtw88_coex.c's
	 * prepare_tx.  IQK stays inline here -- it's chip-family-specific
	 * and hasn't been ported to a subsystem yet.
	 */
	rtw88_coex_prepare_tx(&sc->sc_rtwdev);
	(void)rtw88_phy_calibration(sc);
}

/*
 * Equivalent of reference rtw_ops_mgd_prepare_tx().  Wakes the chip from
 * LPS deep (we never enter LPS yet, so a no-op), notifies the BT
 * coex engine (no coex implementation yet, also a no-op), and runs
 * the chip-side prepare hook above.  Both LPS and coex ports are
 * tracked in the missing-funcs catalog.
 *
 * Runs from sc_tq, NOT inline from iv_newstate.  IQK can take up to
 * 6 s (300 x 20 ms RF_DTXLOK poll) and would otherwise sleep with
 * net80211's COM lock held by ieee80211_newstate_cb -> WITNESS panics
 * the first softclock thread that touches the COM lock.
 */
static void
rtw88_prepare_tx_task(void *arg, int pending __unused)
{
	struct rtw88_usb_softc *sc = arg;

	RTW88_LOCK(sc);
	rtw88_chip_prepare_tx(sc);
	RTW88_UNLOCK(sc);
}

/*
 * Runs from sc_tq, NOT inline from iv_newstate.  iv_newstate is called
 * by net80211 with ic_comlock (a non-sleepable mtx) held; any USB I/O
 * issued there sleeps with comlock held, and the next concurrent ioctl
 * (e.g. setappie pushing wpa_supplicant's RSN IE) trips propagate_priority
 * -> "sleeping thread holds rtw88_usb0_com_lock" (panic 2026-06-23).
 *
 * iv_newstate captures the work into sc_post_state_ops + sc_post_state_bssid
 * under RTW88_LOCK and enqueues us; we replay it here outside comlock.
 *
 * Operations:
 *   POST_BSSID    SCAN -> AUTH: write REG_BSSID0..5 with captured AP MAC
 *   POST_LINK_UP  AUTH/ASSOC -> RUN: MEDIA_STATUS_RPT(connect=1) +
 *                 RA_INFO + EDCA per-AC (parity with Linux mac80211's
 *                 BSS_CHANGED_ASSOC handler, which is also async)
 *   POST_LINK_DOWN ASSOC/RUN -> INIT/SCAN: MEDIA_STATUS_RPT(connect=0)
 *
 * "Latest wins" is fine: net80211 doesn't expect ack-on-wire from iv_newstate
 * and chip-side seeds just need to land before the first associated-data TX.
 */
static void
rtw88_post_state_task(void *arg, int pending __unused)
{
	struct rtw88_usb_softc *sc = arg;
	struct ieee80211com *ic = &sc->sc_ic;
	uint8_t ops, bssid[IEEE80211_ADDR_LEN];
	uint32_t ra_mask = RTW88_DEFAULT_RA_MASK_2G_BGN_1SS;

	RTW88_LOCK(sc);
	ops = sc->sc_post_state_ops;
	memcpy(bssid, sc->sc_post_state_bssid, IEEE80211_ADDR_LEN);
	sc->sc_post_state_ops = 0;

	if (ops & RTW88_POST_BSSID) {
		int i;
		for (i = 0; i < IEEE80211_ADDR_LEN; i++)
			(void)rtw88_usb_write1(sc, REG_BSSID0 + i, bssid[i]);
		/*
		 * Re-write REG_MACID with our STA MAC post-ASSOC.  Linux's
		 * rtw88 writes 0x0610..0x0615 right before EAPOL fires
		 * (qemu-trace-x86/linux-bringup-2026_06_26.usbdump frames
		 * 15899-15909 at t=6.27s, just before M2).  We previously
		 * wrote MACID only at chip init; the chip's ASSOC state
		 * machine may reset/move the MACID slot and require a
		 * refresh before TX of authenticated data.  Without this,
		 * our M2 may be TXed with a stale or zeroed SA field that
		 * the AP rejects silently at MIC validation.
		 */
		for (i = 0; i < IEEE80211_ADDR_LEN; i++)
			(void)rtw88_usb_write1(sc, REG_MACID0 + i,
			    sc->sc_mac[i]);
	}

	/* If both UP and DOWN got queued, the later transition wins. */
	if ((ops & RTW88_POST_LINK_UP) && !(ops & RTW88_POST_LINK_DOWN)) {
		int t_enter = rtw88_assoc_ms(sc);
		uint16_t aid = sc->sc_post_state_aid;
		uint32_t cr, cr_new;

		device_printf(sc->sc_dev,
		    "[T+%d ms] post_state_task LINK_UP enter\n", t_enter);

		/*
		 * Tier 3c: opt-in IQK refresh at LINK_UP.  Not Linux parity
		 * (neither driver does this in reference flow); a hypothesis
		 * test for the M3 wall.  See sc_linkup_iqk field comment.
		 */
		if (sc->sc_linkup_iqk) {
			device_printf(sc->sc_dev,
			    "[T+%d ms] LINK_UP IQK refresh start (linkup_iqk=1)\n",
			    t_enter);
			(void)rtw88_phy_calibration(sc);
			device_printf(sc->sc_dev,
			    "[T+%d ms] LINK_UP IQK refresh done\n",
			    rtw88_assoc_ms(sc));
		}

		/*
		 * Per-port "I'm associated to an AP with this AID" state.
		 * Mirrors Linux rtw_vif_port_config (main.c:933) under
		 * PORT_SET_NET_TYPE + PORT_SET_AID for port 0 (STA mode):
		 *   REG_CR  (0x0100) bits 16-17 = RTW_NET_MGD_LINKED (2)
		 *   REG_AID (0x06a8) bits  0-10 = aid & 0x7ff
		 * Required for the chip's MACID-CAM search to resolve under
		 * RTW_SEC_*_USE_DK -- without these, USE_DK on pairwise gives
		 * the engine no peer-side anchor and unicast RX drops.
		 */
		if (rtw88_usb_read4(sc, REG_CR, &cr) == 0) {
			cr_new = (cr & ~(uint32_t)0x30000) |
			    ((uint32_t)RTW88_NET_MGD_LINKED << 16);
			(void)rtw88_usb_write4(sc, REG_CR, cr_new);
		}
		(void)rtw88_usb_write2(sc, REG_AID, aid);
		/*
		 * Pick the chip's rate-adaptation rate set from the AP's
		 * HT advertisement.  iv_bss carries IEEE80211_NODE_HT only
		 * if HT was negotiated end-to-end (AP beacon had an HT IE
		 * AND assoc-resp confirmed it).  A non-HT AP (hostapd
		 * hw_mode=g without ieee80211n=1) leaves the flag clear;
		 * we then constrain the chip's RA engine to CCK + OFDM via
		 * RATEID_BG so it never picks an MCS rate the AP can't
		 * demodulate.  See project_rtw88_air_capture_HT_mismatch.
		 */
		bool ra_sgi = sc->sc_post_state_sgi;
		bool ht_link = sc->sc_post_state_ht;
		bool is_5g = sc->sc_post_state_5ghz;
		/*
		 * Pick rate_id + ra_mask per band.  2.4 GHz includes CCK
		 * (b/g/n); 5 GHz drops CCK (g/n only).  Without the band
		 * split, a 5 GHz association advertises CCK rates the chip
		 * cannot TX.  Properties captured at newstate -- iv_bss is
		 * not safe to read here (LINK_DOWN can free it concurrently).
		 */
		if (is_5g) {
			sc->sc_rate_id = ht_link ?
			    RTW88_RATEID_GN_N1SS : RTW88_RATEID_G;
			ra_mask = ht_link ?
			    RTW88_DEFAULT_RA_MASK_5G_GN_1SS :
			    RTW88_DEFAULT_RA_MASK_5G_G;
			sc->sc_ra_tighten_mask = ht_link ?
			    RTW88_TIGHTEN_RA_MASK_5G_GN_1SS : 0;
		} else {
			sc->sc_rate_id = ht_link ?
			    RTW88_RATEID_BGN_20M_1SS : RTW88_RATEID_BG;
			ra_mask = ht_link ?
			    RTW88_DEFAULT_RA_MASK_2G_BGN_1SS :
			    RTW88_DEFAULT_RA_MASK_2G_BG;
			sc->sc_ra_tighten_mask = ht_link ?
			    RTW88_TIGHTEN_RA_MASK_2G_BGN_1SS : 0;
		}
		device_printf(sc->sc_dev,
		    "[T+%d ms] post_state RUN: aid=%u net_type="
		    "MGD_LINKED band=%s ht_link=%d sgi=%d "
		    "rate_id=%u ra_mask=0x%05x\n",
		    rtw88_assoc_ms(sc), aid,
		    is_5g ? "5GHz" : "2.4GHz",
		    (int)ht_link, (int)ra_sgi,
		    sc->sc_rate_id, ra_mask);
		/*
		 * H2C order mirrors Linux's mac80211 BSS_CHANGED_ASSOC
		 * (captured 2026-06-23 from a working VM oracle running
		 * the same chip+AP): RA_INFO, MEDIA_STATUS_RPT, KEEP_ALIVE,
		 * DEFAULT_PORT.  Each is needed for a separate chip-side
		 * state:
		 *   RA_INFO         -- rate adaptation seed
		 *   MEDIA_STATUS    -- TX scheduler "macid is linked"
		 *   KEEP_ALIVE      -- fw originates NULL pkts so AP doesn't
		 *                      fire DEAUTH reason 4 (inactivity)
		 *   DEFAULT_PORT    -- bind portid/macid as the default vif
		 *                      for fw-side timers + tx scheduler
		 * Without KEEP_ALIVE + DEFAULT_PORT the AP times us out
		 * within ~10 s of every ASSOC.
		 */
		/*
		 * Per-H2C bisect gate.  Bit 0 disables RA_INFO, bit 1
		 * disables MEDIA_STATUS, bit 2 PWR_MODE, bit 3 fwka,
		 * bit 4 RSVD_PAGE upload+H2C, bit 5 KEEP_ALIVE, bit 6
		 * DEFAULT_PORT, bit 7 EDCA, bit 8 RSSI_INFO.  Default 0
		 * (run all).  Set via dev.rtw88_usb.0.linkup_skip to
		 * isolate which H2C breaks the supplicant's M4 EAPOL TX —
		 * 2026-06-26 hostapd-dd diagnostic showed AP receives M2
		 * cleanly but not M4 (the SECOND data-subtype-0 frame after
		 * the LINK_UP burst fires).
		 */
		/*
		 * BT coex re-init at LINK_UP.  Linux fires 0x69 + 0x60 pair
		 * three times: once at power-on, again at scan-start, and a
		 * third pair at LINK_UP just before RA_INFO (see
		 * parity_capture_2026_07_01/h2c_from_dmesg.txt timestamps
		 * 147/155/165s).  We fire once at mac_init; add the LINK_UP
		 * refresh so the chip's shared-antenna arbiter re-latches
		 * for the associated STA's TX slot.  Payload matches Linux
		 * byte-for-byte (0x00010c69, 0x00000060).
		 */
		(void)rtw88_coex_link_up(&sc->sc_rtwdev);
		/*
		 * Linux fires an additional pre-M2 sequence here (REG 0x0204
		 * 0x81cc + REG 0x0101 gain toggle x4, REG_CR raw, REG_AID
		 * raw).  Tested 2026-07-02 on top of the TXAGC port and did
		 * NOT restore M3 arrival.  Preserved in git history at
		 * commit 793c274d if revisited as part of a broader chip-
		 * state port.  See qemu-trace-x86/parity_capture_2026_07_01
		 * frames 29901-30017 for the reference sequence.
		 */
		if ((sc->sc_linkup_skip & 0x01) == 0) {
			/*
			 * Fire ONE RA_INFO at LINK_UP with init_ra_lvl=1,
			 * no_update=0, and the sparse mask (0x000ff015 for
			 * 2G BGN).  Linux fires a SECOND RA_INFO with
			 * no_update=1 + tighten mask (0x000f0000) but only
			 * ~2 seconds later — long after the 4-way handshake
			 * completes.  That second call is a lifecycle
			 * event (post-first-RSSI-feedback), not part of
			 * assoc.  Firing it immediately at LINK_UP would
			 * cross the RA state mid-handshake.
			 * See qemu-trace-x86/parity_capture_2026_07_01
			 * timestamps: RA_INFO 1 at t+0ms, RA_INFO 2 at
			 * t+2066ms.
			 */
			(void)rtw88_h2c_ra_info(sc, 0,
			    sc->sc_rate_id,
			    /* bw_mode 20 MHz */ 0,
			    /* init_ra_lvl */    1,
			    /* sgi_en */         ra_sgi,
			    /* ldpc_en */        false,
			    /* vht_en */         0,
			    /* no_update */      false,
			    ra_mask);
		}
		if ((sc->sc_linkup_skip & 0x02) == 0)
			(void)rtw88_h2c_media_status_report(sc, 0, true);
		if ((sc->sc_linkup_skip & 0x04) == 0)
			(void)rtw88_h2c_pwr_mode_active(sc);
		if ((sc->sc_linkup_skip & 0x08) == 0)
			rtw88_fwka_start(sc);
		if ((sc->sc_linkup_skip & 0x10) == 0) {
			(void)rtw88_upload_rsvd_pages(sc, bssid);
			(void)rtw88_h2c_rsvd_page(sc);
		}
		/*
		 * KEEP_ALIVE (H2C 0x03) intentionally NOT sent.  Session 3
		 * usbmon capture 2026-07-01: Linux never emits H2C_KEEP_ALIVE
		 * during a normal assoc + DHCP + ping + disassoc cycle.  The
		 * fw NULL-packet emitter belongs to LPS/WoWLAN paths we don't
		 * enter yet.
		 */
		if ((sc->sc_linkup_skip & 0x40) == 0)
			(void)rtw88_h2c_default_port(sc, 0, 0);
		if ((sc->sc_linkup_skip & 0x80) == 0)
			(void)rtw88_conf_tx_all_ac_locked(sc, ic);
		device_printf(sc->sc_dev,
		    "[T+%d ms] post_state RUN: linked macid=0 "
		    "ra_mask=0x%05x (took %d ms)\n",
		    rtw88_assoc_ms(sc), ra_mask,
		    rtw88_assoc_ms(sc) - t_enter);
	} else if (ops & RTW88_POST_LINK_DOWN) {
		/*
		 * Clear the chip-CAM PTK and GTK slots on link teardown.
		 * net80211's iv_key_delete fires for SOME paths but not all
		 * -- locally-initiated MLME deauth (wpa_supplicant calling
		 * Request to deauthenticate during a re-AUTH cycle that
		 * follows AP DEAUTH) leaves the keys behind, so the next
		 * 4-way handshake's plaintext-EAPOL M2 goes out encrypted
		 * with the *stale* PTK, the AP can't decrypt, and the
		 * handshake fails with "pre-shared key may be incorrect".
		 * Wipe all five CAM slots we touch (0..3 group keys + 4
		 * pairwise) so the next handshake starts clean.
		 */
		int i;
		for (i = 0; i < RTW88_CAM_DEFAULT_KEYS; i++)
			(void)rtw88_cam_clear_entry(sc, (uint8_t)i);
		(void)rtw88_cam_clear_entry(sc, RTW88_CAM_PAIRWISE_SLOT);
		/* KEEP_ALIVE off at teardown: never enabled at LINK_UP. */
		(void)rtw88_h2c_media_status_report(sc, 0, false);
		rtw88_fwka_stop(sc);

		/*
		 * Roam-thrash hygiene: clear any pending PTK-install gate and
		 * drain held Protected unicast frames.  Without this, a
		 * supplicant deauth that fires between iv_key_set and the
		 * CAM write strands sc_ptk_install_pending=true into the next
		 * association -- M4 of the next handshake then gets parked
		 * forever in sc_tx_pending_q and the AP times us out.  Also
		 * drop any cam_q ops queued under the old BSSID so they don't
		 * land against the new one.
		 */
		sc->sc_ptk_install_pending = false;
		/*
		 * mbufq_drain frees every m in sc_tx_pending_q; if the
		 * sc_m4_needs_encap marker referenced one, it's freed as
		 * part of the drain -- just null the marker.
		 */
		mbufq_drain(&sc->sc_tx_pending_q);
		sc->sc_m4_needs_encap = NULL;
		sc->sc_cam_q_n = 0;

		/*
		 * Reset the link-RSSI EWMA so the next link starts fresh
		 * instead of blending into the stale dead-AP average.
		 */
		sc->sc_avg_rssi = 0;
		sc->sc_avg_rssi_valid = false;

		/*
		 * Drop any data frames still queued for the now-dead BSS.
		 * Without this they bleed into the next association: the
		 * USB callback pulls them, tags QSEL/RA from current state
		 * which is mid-AUTH, and the chip TXes plaintext-or-stale-
		 * key frames toward an AP that has no key for us.  Leave
		 * sc_tx_mgmt_q alone — a deauth/disassoc may still be in
		 * flight and should reach the AP before we tear down.
		 */
		mbufq_drain(&sc->sc_tx_data_q);
		mbufq_drain(&sc->sc_tx_data_vo_q);
	}
	RTW88_UNLOCK(sc);
}

static int
rtw88_newstate(struct ieee80211vap *vap, enum ieee80211_state nstate, int arg)
{
	struct rtw88_vap *rvp = RTW88_VAP(vap);
	struct ieee80211com *ic = vap->iv_ic;
	struct rtw88_usb_softc *sc = ic->ic_softc;
	enum ieee80211_state ostate = vap->iv_state;
	uint8_t pending = 0;

	device_printf(sc->sc_dev, "newstate: %s -> %s\n",
	    ieee80211_state_name[ostate], ieee80211_state_name[nstate]);

	/*
	 * iv_newstate is called by net80211 with ic_comlock held.  We may
	 * NOT issue USB I/O from here -- usbd_do_request_flags would sleep
	 * with comlock held and any concurrent ioctl would trip witness.
	 * Capture the post-state work into sc_post_state_* and let
	 * rtw88_post_state_task issue the actual writes on sc_tq.
	 */
	if (nstate == IEEE80211_S_AUTH && ostate == IEEE80211_S_SCAN) {
		struct ieee80211_node *ni = vap->iv_bss;

		/*
		 * Arm the EAPOL latency clock here -- every later
		 * rtw88_eapol_log() / post_state_task / vap_key_set
		 * print is relative to this tick value.
		 */
		sc->sc_assoc_ticks = ticks;
		taskqueue_enqueue(sc->sc_tq, &sc->sc_prepare_tx_task);
		if (ni != NULL) {
			RTW88_LOCK(sc);
			memcpy(sc->sc_post_state_bssid, ni->ni_bssid,
			    IEEE80211_ADDR_LEN);
			sc->sc_post_state_ops |= RTW88_POST_BSSID;
			RTW88_UNLOCK(sc);
			pending = 1;
		}
	}

	/*
	 * Post-ASSOC chip-side substrate.  Mirrors the chain Linux's
	 * mac80211 fires from rtw_sta_add + rtw_ops_bss_info_changed
	 * (BSS_CHANGED_ASSOC) -- also async on the mac80211 workqueue:
	 *   - MEDIA_STATUS_RPT(connect=1) -- TX scheduler "MACID 0 linked"
	 *   - RA_INFO -- seed rate-adaptation supported rates
	 *   - EDCA per-AC -- proper QoS backoff
	 *
	 * rsvd_page download (RSVD_PAGE H2C + bulk-OUT page data) NOT
	 * ported: only needed for LPS PS_POLL/NULL/QOS_NULL frames and
	 * we don't enter LPS yet.
	 */
	if (nstate == IEEE80211_S_RUN &&
	    (ostate == IEEE80211_S_ASSOC || ostate == IEEE80211_S_AUTH)) {
		struct ieee80211_node *ni = vap->iv_bss;
		uint16_t aid = (ni != NULL) ?
		    (uint16_t)(IEEE80211_AID(ni->ni_associd) & 0x7ff) : 0;
		bool ht_link = ni != NULL &&
		    (ni->ni_flags & IEEE80211_NODE_HT) != 0;
		bool sgi = ht_link &&
		    (ni->ni_htcap & IEEE80211_HTCAP_SHORTGI20) != 0;
		bool is_5g = ni != NULL && ni->ni_chan != NULL &&
		    IEEE80211_IS_CHAN_5GHZ(ni->ni_chan);

		RTW88_LOCK(sc);
		sc->sc_post_state_aid = aid;
		sc->sc_post_state_ht = ht_link;
		sc->sc_post_state_sgi = sgi;
		sc->sc_post_state_5ghz = is_5g;
		sc->sc_post_state_ops |= RTW88_POST_LINK_UP;
		sc->sc_post_state_ops &= ~RTW88_POST_LINK_DOWN;
		RTW88_UNLOCK(sc);
		/*
		 * Track BF state.  We pass 0 for both htcap and vhtcap so
		 * the subsystem records the peer but keeps SU inactive
		 * against non-BF APs.  A follow-up can extract the AP's
		 * actual cap fields from ni->ni_ies once the ies-parser
		 * plumbing is in place.
		 */
		if (ni != NULL)
			rtw88_bf_assoc(&sc->sc_rtwdev, ni->ni_bssid, 0, 0, aid);
		pending = 1;
	}

	/* On link loss / disconnect, tell chip the MACID is disconnected. */
	if ((ostate == IEEE80211_S_RUN || ostate == IEEE80211_S_ASSOC) &&
	    (nstate == IEEE80211_S_INIT || nstate == IEEE80211_S_SCAN)) {
		RTW88_LOCK(sc);
		sc->sc_post_state_ops |= RTW88_POST_LINK_DOWN;
		sc->sc_post_state_ops &= ~RTW88_POST_LINK_UP;
		RTW88_UNLOCK(sc);
		/*
		 * BF + coex link_down bookkeeping.  Both are pure state
		 * updates today; coex_link_down is a no-op body reserved
		 * for the BT state machine port, and bf_disassoc just
		 * clears the peer + su_active flag.
		 */
		rtw88_bf_disassoc(&sc->sc_rtwdev);
		rtw88_coex_link_down(&sc->sc_rtwdev);
		pending = 1;
	}

	if (pending)
		taskqueue_enqueue(sc->sc_tq, &sc->sc_post_state_task);

	return (rvp->newstate(vap, nstate, arg));
}

static void
rtw88_set_channel(struct ieee80211com *ic)
{
	struct rtw88_usb_softc *sc = ic->ic_softc;
	struct ieee80211_channel *c = ic->ic_curchan;
	uint8_t chan, bw, primary_ch_idx;
	int error;

	if (c == NULL)
		return;
	chan = IEEE80211_CHAN2IEEE(c);
	/*
	 * Net80211 sets IEEE80211_CHAN_HT40U / HT40D on per-channel entries
	 * to encode HT40 primary position; HT40U = primary is lower 20,
	 * HT40D = primary is upper 20.  VHT80 is similarly flagged via
	 * IEEE80211_CHAN_VHT80.  Translate into Linux's primary_ch_idx
	 * convention (1 = lower primary, 0 = upper primary / 20 MHz).
	 */
	bw = RTW88_CHANNEL_WIDTH_20;
	primary_ch_idx = RTW88_SC_DONT_CARE;
	if (IEEE80211_IS_CHAN_VHT80(c)) {
		bw = RTW88_CHANNEL_WIDTH_80;
		/*
		 * For VHT80 we'd need to know the center freq vs primary
		 * to pick UPPER/LOWER/UPMOST/LOWEST; net80211 doesn't
		 * surface that cleanly per-chan, so use UPPER as a
		 * conservative default until a per-channel calculator
		 * lands.  Single-VAP STA mode will get auto-tuned.
		 */
		primary_ch_idx = RTW88_SC_20_UPPER;
	} else if (IEEE80211_IS_CHAN_HT40U(c)) {
		/* HT40+: primary low, secondary above -> primary is LOWER. */
		bw = RTW88_CHANNEL_WIDTH_40;
		primary_ch_idx = RTW88_SC_20_LOWER;
	} else if (IEEE80211_IS_CHAN_HT40D(c)) {
		/* HT40-: primary high, secondary below -> primary is UPPER. */
		bw = RTW88_CHANNEL_WIDTH_40;
		primary_ch_idx = RTW88_SC_20_UPPER;
	}
	if (chan == sc->sc_cur_channel && bw == sc->sc_cur_bw &&
	    primary_ch_idx == sc->sc_cur_primary_ch_idx)
		return;

	error = rtw88_phy_set_channel(sc, chan, bw, primary_ch_idx);
	if (error != 0)
		device_printf(sc->sc_dev,
		    "set_channel: chan=%u bw=%u pri=%u failed: %d\n",
		    chan, bw, primary_ch_idx, error);
}

/*
 * Write a 7-bit IGI (initial gain index) into REG_DIG_PATH_A.  Mirrors
 * the single-path branch of reference rtw_phy_dig_write() -- 8821C has no
 * CCK-side DIG register and only one RF path.
 */
static int
rtw88_dig_write(struct rtw88_usb_softc *sc, uint8_t igi)
{
	uint32_t val;
	int error;

	RTW88_ASSERT_LOCKED(sc);

	if ((error = rtw88_usb_read4(sc, REG_DIG_PATH_A, &val)) != 0)
		return (error);
	val = (val & ~DIG_PATH_A_MASK) | (igi & DIG_PATH_A_MASK);
	return (rtw88_usb_write4(sc, REG_DIG_PATH_A, val));
}

/*
 * Sample the OFDM (and, when CCK is enabled on this channel, CCK)
 * false-alarm counters and then clear them so the next tick gets a
 * fresh delta.  *fa_out is total alarms since the last reset.  This
 * is the 8821C-specific half of reference's rtw_phy_stat_false_alarm()
 * trimmed to just the totals we need for DIG.
 */
static int
rtw88_dig_sample_fa(struct rtw88_usb_softc *sc, uint16_t *fa_out)
{
	uint32_t rxpsel, fa_cck, fa_ofdm;
	uint32_t total;
	int error;

	RTW88_ASSERT_LOCKED(sc);

	if ((error = rtw88_usb_read4(sc, REG_RXPSEL, &rxpsel)) != 0)
		return (error);
	if ((error = rtw88_usb_read4(sc, REG_FA_OFDM, &fa_ofdm)) != 0)
		return (error);
	total = fa_ofdm & 0xFFFF;
	if (rxpsel & (1U << 28)) {
		if ((error = rtw88_usb_read4(sc, REG_FA_CCK, &fa_cck)) != 0)
			return (error);
		total += fa_cck & 0xFFFF;
	}
	*fa_out = (total > UINT16_MAX) ? UINT16_MAX : (uint16_t)total;

	/*
	 * Reset the on-chip counters: pulse REG_FAS bit 17, toggle
	 * REG_RXDESC bit 15, and pulse REG_CNTRST bit 0.  Same order
	 * reference uses at the tail of rtw8821c_false_alarm_statistics().
	 */
	if ((error = rtw88_usb_read4(sc, REG_FAS, &fa_ofdm)) != 0)
		return (error);
	(void)rtw88_usb_write4(sc, REG_FAS, fa_ofdm | (1U << 17));
	(void)rtw88_usb_write4(sc, REG_FAS, fa_ofdm & ~(1U << 17));
	if (rtw88_usb_read4(sc, REG_RXDESC, &fa_ofdm) == 0) {
		(void)rtw88_usb_write4(sc, REG_RXDESC, fa_ofdm & ~(1U << 15));
		(void)rtw88_usb_write4(sc, REG_RXDESC, fa_ofdm | (1U << 15));
	}
	if (rtw88_usb_read4(sc, REG_CNTRST, &fa_ofdm) == 0) {
		(void)rtw88_usb_write4(sc, REG_CNTRST, fa_ofdm | 1U);
		(void)rtw88_usb_write4(sc, REG_CNTRST, fa_ofdm & ~1U);
	}
	return (0);
}

/*
 * Shift a (IGI, FA-count) sample into the 4-deep ring and update the
 * up/down trend bitmap (bit 0 = most recent move was up).  Renamed
 * from reference rtw_phy_dig_recorder().
 */
static void
rtw88_dig_record(struct rtw88_usb_softc *sc, uint8_t igi, uint16_t fa)
{
	uint8_t bits;
	bool up;

	bits = (uint8_t)((sc->sc_igi_trend_bits << 1) & 0xFE);
	up = igi > sc->sc_igi_log[0];
	if (up)
		bits |= 1;
	sc->sc_igi_log[3] = sc->sc_igi_log[2];
	sc->sc_igi_log[2] = sc->sc_igi_log[1];
	sc->sc_igi_log[1] = sc->sc_igi_log[0];
	sc->sc_igi_log[0] = igi;
	sc->sc_fa_log[3] = sc->sc_fa_log[2];
	sc->sc_fa_log[2] = sc->sc_fa_log[1];
	sc->sc_fa_log[1] = sc->sc_fa_log[0];
	sc->sc_fa_log[0] = fa;
	sc->sc_igi_trend_bits = bits;
}

/*
 * One pass of the DIG decision loop.  Equivalent to reference's
 * rtw_phy_dig(), trimmed to coverage-mode (we don't model the
 * associated/linked branch until net80211 sta tracking is wired in).
 */
static void
rtw88_dig_tick(struct rtw88_usb_softc *sc)
{
	uint16_t fa_cnt, fa_th_extra, fa_th_high, fa_th_low;
	uint8_t pre_igi, cur_igi;
	uint8_t lower, upper;
	int delta;

	RTW88_ASSERT_LOCKED(sc);

	if (rtw88_dig_sample_fa(sc, &fa_cnt) != 0)
		return;

	pre_igi = sc->sc_igi_log[0];
	cur_igi = pre_igi;

	/*
	 * Coverage thresholds: bigger FA delta -> bigger step up.  reference
	 * subtracts 2 from cur_igi unconditionally afterward, so a quiet
	 * channel slowly walks the IGI back down toward the floor.
	 */
	fa_th_extra = DIG_CVRG_FA_TH_EXTRA_HIGH;
	fa_th_high = DIG_CVRG_FA_TH_HIGH;
	fa_th_low = DIG_CVRG_FA_TH_LOW;
	if (fa_cnt > fa_th_extra)
		delta = 4;	/* step[0] (-2 applied below) */
	else if (fa_cnt > fa_th_high)
		delta = 3;
	else if (fa_cnt > fa_th_low)
		delta = 2;
	else
		delta = 0;
	delta -= 2;
	if (delta > 0)
		cur_igi = (uint8_t)MIN(cur_igi + delta, 0xFF);
	else if (-delta > cur_igi)
		cur_igi = 0;
	else
		cur_igi = (uint8_t)(cur_igi + delta);

	/* Coverage bounds: [DIG_CVRG_MIN, DIG_CVRG_MAX]. */
	lower = DIG_CVRG_MIN;
	upper = DIG_CVRG_MAX;
	if (cur_igi < lower)
		cur_igi = lower;
	else if (cur_igi > upper)
		cur_igi = upper;

	rtw88_dig_record(sc, cur_igi, fa_cnt);

	if (cur_igi != pre_igi)
		(void)rtw88_dig_write(sc, cur_igi);
}

/*
 * Callout entry point.  Runs in softclock with no lock held; just
 * kicks the taskqueue so the actual register work happens in our
 * kthread where USB transfers can block.
 */
static void
rtw88_dig_callout(void *arg)
{
	struct rtw88_usb_softc *sc = arg;

	taskqueue_enqueue(sc->sc_tq, &sc->sc_dig_task);
}

/*
 * Taskqueue body: one DIG decision pass, then re-arm the callout
 * unless detach has cleared sc_dig_active.  The active check is taken
 * twice -- once on entry, once before re-arming -- so detach's
 * "clear active; drain callout; drain task" sequence cannot race a
 * resurrection.
 */
static void
rtw88_dig_task(void *arg, int pending __unused)
{
	struct rtw88_usb_softc *sc = arg;

	RTW88_LOCK(sc);
	if (!sc->sc_dig_active) {
		RTW88_UNLOCK(sc);
		return;
	}
	rtw88_dig_tick(sc);
	if (sc->sc_dig_active)
		callout_reset(&sc->sc_dig_co,
		    DIG_TICK_INTERVAL_MS * hz / 1000,
		    rtw88_dig_callout, sc);
	RTW88_UNLOCK(sc);
}

/*
 * Seed igi_log[0] with whatever the BB tables left in REG_DIG_PATH_A
 * and arm the periodic callout.  Caller must hold sc_sx; callout
 * is initialised once at attach time but only kicked off here.
 */
static void
rtw88_dig_start(struct rtw88_usb_softc *sc)
{
	uint32_t val;

	RTW88_ASSERT_LOCKED(sc);

	if (sc->sc_dig_active)
		return;
	if (rtw88_usb_read4(sc, REG_DIG_PATH_A, &val) != 0)
		return;
	sc->sc_igi_log[0] = (uint8_t)(val & DIG_PATH_A_MASK);
	sc->sc_igi_log[1] = sc->sc_igi_log[0];
	sc->sc_igi_log[2] = sc->sc_igi_log[0];
	sc->sc_igi_log[3] = sc->sc_igi_log[0];
	sc->sc_fa_log[0] = sc->sc_fa_log[1] = 0;
	sc->sc_fa_log[2] = sc->sc_fa_log[3] = 0;
	sc->sc_igi_trend_bits = 0;
	sc->sc_dig_active = true;
	callout_reset(&sc->sc_dig_co,
	    DIG_TICK_INTERVAL_MS * hz / 1000,
	    rtw88_dig_callout, sc);
}

/*
 * FW keepalive callout: re-arm SET_PWR_MODE ACTIVE every 1 second to
 * keep the firmware's power-management state machine engaged. Linux
 * mac80211 fires PWR_MODE H2C ~1/sec during sustained traffic; without
 * it our chip's FW times out and unicast TX/RX degrades (Bug B).
 * Mirrors rtw88_dig_callout pattern: callout enqueues task on sc_tq,
 * task does USB I/O (which can sleep) and re-arms callout.
 */
#define	RTW88_FWKA_INTERVAL_MS	1000

/*
 * Delay from LINK_UP to the 2nd (tighten) RA_INFO H2C.  Linux fires at
 * ~2066 ms; 2000 is close enough since our fwka tick is 1 Hz.
 */
#define	RTW88_RA_TIGHTEN_MS	2000

static void
rtw88_fwka_callout(void *arg)
{
	struct rtw88_usb_softc *sc = arg;

	taskqueue_enqueue(sc->sc_tq, &sc->sc_fwka_task);
}

static void
rtw88_fwka_task(void *arg, int pending __unused)
{
	struct rtw88_usb_softc *sc = arg;

	RTW88_LOCK(sc);
	if (!sc->sc_fwka_active) {
		RTW88_UNLOCK(sc);
		return;
	}
	/*
	 * PWR_MODE ACTIVE every tick (1 Hz).  Without this the chip's FW
	 * PM state machine times out and unicast TX/RX degrades.  Linux
	 * also fires RSSI_MONITOR + WL_PHY_INFO at ~0.5 Hz here, but
	 * 2026-06-25 testing showed periodic RSSI feedback regressed
	 * link stability on our port (FW picks a rate based on our RSSI
	 * report that the AP can't match -> AP misses -> retries -> deauth).
	 * Those H2Cs are intentionally NOT fired.
	 */
	(void)rtw88_h2c_pwr_mode_active(sc);

	/*
	 * Second RA_INFO (Linux parity, Divergence #2).  Fires once, ~2 s
	 * after LINK_UP, with no_update=1 + tighten mask (HT MCS 4-7
	 * only).  Locks the chip's RA engine to the higher HT rates after
	 * the initial exchange has settled and rate has already stabilized.
	 */
	if (!sc->sc_ra_tighten_fired &&
	    sc->sc_ra_tighten_mask != 0 &&
	    rtw88_assoc_ms(sc) >= RTW88_RA_TIGHTEN_MS) {
		(void)rtw88_h2c_ra_info(sc, 0,
		    sc->sc_rate_id,
		    /* bw_mode 20 MHz */ 0,
		    /* init_ra_lvl */    0,
		    /* sgi_en */         false,
		    /* ldpc_en */        false,
		    /* vht_en */         0,
		    /* no_update */      true,
		    sc->sc_ra_tighten_mask);
		sc->sc_ra_tighten_fired = true;
		device_printf(sc->sc_dev,
		    "[T+%d ms] RA_INFO tighten fired (no_update=1 "
		    "mask=0x%05x)\n",
		    rtw88_assoc_ms(sc), sc->sc_ra_tighten_mask);
	}

	/*
	 * FW watchdog: zero RX completions across 3 consecutive ticks
	 * (~3s) on an associated link means the chip's bulk-IN or RX MAC
	 * is dead -- the AP sends beacons every ~100 ms so there's always
	 * something to receive.  Log once + bump sc_stat_fw_stalls so
	 * monitoring can react (kldunload + kldload to recover).
	 */
	{
		uint64_t rxn = sc->sc_rx_packets;
		if (rxn == sc->sc_last_rx_packets) {
			if (sc->sc_fw_stall_ticks < 255)
				sc->sc_fw_stall_ticks++;
			if (sc->sc_fw_stall_ticks >= 3 &&
			    !sc->sc_fw_stall_logged) {
				sc->sc_stat_fw_stalls++;
				sc->sc_fw_stall_logged = true;
				device_printf(sc->sc_dev,
				    "FW watchdog: no RX completions for "
				    ">=3 ticks at rx_packets=%ju -- chip "
				    "wedged (try kldunload + kldload)\n",
				    (uintmax_t)rxn);
			}
		} else {
			sc->sc_fw_stall_ticks = 0;
			sc->sc_fw_stall_logged = false;
			sc->sc_last_rx_packets = rxn;
		}
	}

	if (sc->sc_fwka_active)
		callout_reset(&sc->sc_fwka_co,
		    RTW88_FWKA_INTERVAL_MS * hz / 1000,
		    rtw88_fwka_callout, sc);
	RTW88_UNLOCK(sc);
}

static void
rtw88_fwka_start(struct rtw88_usb_softc *sc)
{
	RTW88_ASSERT_LOCKED(sc);
	if (sc->sc_fwka_active)
		return;
	sc->sc_fwka_active = true;
	/* Reset watchdog state so a fresh association starts clean. */
	sc->sc_last_rx_packets = sc->sc_rx_packets;
	sc->sc_fw_stall_ticks = 0;
	sc->sc_fw_stall_logged = false;
	/* Arm the delayed 2nd RA_INFO (Linux-parity, Divergence #2). */
	sc->sc_ra_tighten_fired = false;
	callout_reset(&sc->sc_fwka_co,
	    RTW88_FWKA_INTERVAL_MS * hz / 1000,
	    rtw88_fwka_callout, sc);
}

static void
rtw88_fwka_stop(struct rtw88_usb_softc *sc)
{
	RTW88_ASSERT_LOCKED(sc);
	sc->sc_fwka_active = false;
	callout_stop(&sc->sc_fwka_co);
}

/*
 * Ramp DIG to DIG_CVRG_MIN at scan_start and restore the
 * pre-scan value at scan_end.  Mirrors reference
 * rtw_phy_dig_set_max_coverage() + rtw_phy_dig_reset(), except we
 * also stash the live register value rather than the periodic-DM
 * history slot 0, since our DM loop only runs every 2 s and may not
 * have written recently.
 */
static void
rtw88_scan_start(struct ieee80211com *ic)
{
	struct rtw88_usb_softc *sc = ic->ic_softc;
	uint32_t val;

	RTW88_LOCK(sc);
	if (rtw88_usb_read4(sc, REG_DIG_PATH_A, &val) == 0) {
		sc->sc_dig_scan_save = (uint8_t)(val & DIG_PATH_A_MASK);
		(void)rtw88_dig_write(sc, DIG_CVRG_MIN);
	}
	RTW88_UNLOCK(sc);
}

static void
rtw88_scan_end(struct ieee80211com *ic)
{
	struct rtw88_usb_softc *sc = ic->ic_softc;

	RTW88_LOCK(sc);
	if (sc->sc_dig_scan_save != 0)
		(void)rtw88_dig_write(sc, sc->sc_dig_scan_save);
	RTW88_UNLOCK(sc);
}

static void
rtw88_parent(struct ieee80211com *ic)
{
	struct rtw88_usb_softc *sc = ic->ic_softc;
	int startall = (ic->ic_nrunning > 0 && !sc->sc_running);

	device_printf(sc->sc_dev,
	    "parent: nrunning=%d running=%d rx_pkts=%ju rx_frames=%ju\n",
	    ic->ic_nrunning, (int)sc->sc_running,
	    (uintmax_t)sc->sc_rx_packets, (uintmax_t)sc->sc_rx_frames);
	if (ic->ic_nrunning > 0) {
		sc->sc_running = true;
		if (startall)
			ieee80211_start_all(ic);
	} else {
		sc->sc_running = false;
	}
}

/*
 * Enqueue an mbuf onto the async mgmt TX queue and kick the bulk-OUT
 * xfer.  Caller does NOT free the mbuf on success -- ownership
 * transfers to the queue, and the USB callback frees on completion.
 */
static int
rtw88_tx_enqueue_locked(struct rtw88_usb_softc *sc, struct mbuf *m)
{
	bool is_data;
	int error;

	RTW88_ASSERT_LOCKED(sc);

	if (m->m_pkthdr.len <= 0 || m->m_pkthdr.len > UINT16_MAX)
		return (EINVAL);
	if (m->m_pkthdr.len + RTW88_TX_PKT_DESC_SZ > RTW88_TX_BUFSZ)
		return (EMSGSIZE);

	/*
	 * Split by 802.11 type (FC0[3:2]): mgmt (0x00) -> BULK_TX_HI;
	 * data (0x08) -> BE/BK path or VI/VO path (see below).
	 *
	 * Linux 8821C 3-bulkout rqpn splits data further by AC:
	 *   BE/BK (TID 0-3) -> NORMAL endpoint (EP 0x06)
	 *   VI/VO (TID 4-7) -> LOW    endpoint (EP 0x08)
	 * We follow suit; prior driver routed everything to LOW and DUT
	 * testing 2026-06-30 showed VO EAPOL got stuck behind BE traffic
	 * on the same endpoint.
	 *
	 * For QoS data, the low nibble of QoS ctrl byte 24 is the TID.
	 * Non-QoS data defaults to BE.  EAPOL (detected below via LLC/
	 * SNAP 0x888E) forces VO to match mac80211's tx_control_port
	 * routing convention.
	 */
	is_data = ((mtod(m, const uint8_t *)[0]) & 0x0C) == 0x08;

	/*
	 * Hold Protected unicast data frames in sc_tx_pending_q while a
	 * PTK install is queued but not yet executed by sc_cam_task.
	 * Without this gate, the supplicant's M4 (issued ~0-8 ms after
	 * iv_key_set returns) races the deferred CAM write and is
	 * silently dropped by the chip.  sc_cam_task drains
	 * sc_tx_pending_q into sc_tx_data_q the instant the install
	 * completes.
	 *
	 * Group keys (GTK) install in a different CAM slot keyed by
	 * broadcast addr, so we don't gate on them -- only the PTK
	 * install (sc_ptk_install_pending) blocks unicast TX.
	 */
	if (is_data && sc->sc_ptk_install_pending && m->m_pkthdr.len >= 24) {
		const uint8_t *fhdr = mtod(m, const uint8_t *);
		bool protected = (fhdr[1] & 0x40) != 0;
		bool to_bcast = (fhdr[4] & 0x01) != 0;	/* addr1[0] mcast bit */
		bool is_m4_marker = (m == sc->sc_m4_needs_encap);

		if (is_m4_marker || protected)
			device_printf(sc->sc_dev,
			    "[T+%d ms] tx_enqueue: gate m=%p prot=%d bcast=%d "
			    "m4_marker=%d fc=%02x%02x len=%d\n",
			    rtw88_assoc_ms(sc), m, protected, to_bcast,
			    is_m4_marker, fhdr[0], fhdr[1], m->m_pkthdr.len);
		if (protected && !to_bcast) {
			error = mbufq_enqueue(&sc->sc_tx_pending_q, m);
			if (error != 0) {
				sc->sc_stat_tx_drops++;
				return (error);
			}
			{
				uint32_t l = mbufq_len(&sc->sc_tx_pending_q);
				if (l > sc->sc_stat_tx_pending_qlen_max)
					sc->sc_stat_tx_pending_qlen_max = l;
			}
			if (is_m4_marker)
				device_printf(sc->sc_dev,
				    "[T+%d ms] tx_enqueue: M4 parked in "
				    "pending_q (qlen=%u)\n",
				    rtw88_assoc_ms(sc),
				    mbufq_len(&sc->sc_tx_pending_q));
			return (0);
		}
	} else if (m == sc->sc_m4_needs_encap) {
		device_printf(sc->sc_dev,
		    "[T+%d ms] tx_enqueue: M4 marker BYPASSED gate "
		    "(is_data=%d ptk_pending=%d len=%d) — flowing through\n",
		    rtw88_assoc_ms(sc), is_data, sc->sc_ptk_install_pending,
		    m->m_pkthdr.len);
	}

	if (is_data) {
		/*
		 * Pick TID / AC for the endpoint split.  QoS ctrl byte 24
		 * has TID in low nibble.  EAPOL is force-routed to VO to
		 * match mac80211's tx_control_port priority tagging.
		 */
		const uint8_t *fh = mtod(m, const uint8_t *);
		uint8_t fc0 = fh[0];
		uint8_t tid = 0;	/* BE */
		bool is_vo = false;
		bool is_eapol = false;
		{
			uint16_t ki_dummy;
			is_eapol = rtw88_eapol_key_info(m, &ki_dummy);
		}
		if ((fc0 & 0x80) != 0 && m->m_pkthdr.len >= 26)
			tid = fh[24] & 0x07;
		if (tid >= 4)
			is_vo = true;

		/*
		 * Route by AC per Linux rqpn_table[3]:
		 *   VI/VO (TID 4-7) + EAPOL -> sc_tx_data_vo_q -> EP 0x06
		 *   BE/BK (TID 0-3)         -> sc_tx_data_q    -> EP 0x08
		 * Session 3 usbmon capture 2026-07-01 confirmed Linux emits
		 * M2/M4 EAPOL on EP 0x06 (qsel=7 VO) and DHCP/ARP/ICMP on
		 * EP 0x08 (qsel=0 BE).  v1.1.2 initial split had this
		 * inverted, blocking M2.
		 */
		if (is_eapol || is_vo) {
			error = mbufq_enqueue(&sc->sc_tx_data_vo_q, m);
			if (error != 0) {
				sc->sc_stat_tx_drops++;
				counter_u64_add(sc->sc_ic.ic_oerrors, 1);
				return (error);
			}
			usbd_transfer_start(sc->sc_xfer[RTW88_BULK_TX_NORMAL]);
		} else {
			error = mbufq_enqueue(&sc->sc_tx_data_q, m);
			if (error != 0) {
				sc->sc_stat_tx_drops++;
				counter_u64_add(sc->sc_ic.ic_oerrors, 1);
				return (error);
			}
			{
				uint32_t l = mbufq_len(&sc->sc_tx_data_q);
				if (l > sc->sc_stat_tx_data_qlen_max)
					sc->sc_stat_tx_data_qlen_max = l;
			}
			usbd_transfer_start(sc->sc_xfer[RTW88_BULK_TX_LO]);
		}
	} else {
		error = mbufq_enqueue(&sc->sc_tx_mgmt_q, m);
		if (error != 0) {
			sc->sc_stat_tx_drops++;
			counter_u64_add(sc->sc_ic.ic_oerrors, 1);
			return (error);
		}
		{
			uint32_t l = mbufq_len(&sc->sc_tx_mgmt_q);
			if (l > sc->sc_stat_tx_mgmt_qlen_max)
				sc->sc_stat_tx_mgmt_qlen_max = l;
		}
		usbd_transfer_start(sc->sc_xfer[RTW88_BULK_TX_HI]);
	}
	return (0);
}

/*
 * Return ms since SCAN -> AUTH entry (sc_assoc_ticks), as a signed
 * value so logs read sanely if a frame arrives before assoc state was
 * armed (negative ms -> "pre-assoc, ignore").
 */
static int
rtw88_assoc_ms(const struct rtw88_usb_softc *sc)
{
	int delta = ticks - sc->sc_assoc_ticks;
	return ((int)(((int64_t)delta * 1000) / hz));
}

/*
 * Detect an EAPOL-Key frame inside a TX/RX mbuf and return its
 * Key Information word (host order) via *out.  Returns true iff the
 * mbuf is a data-subtype 802.11 frame whose LLC SNAP body carries
 * the 0x888E ethertype and the EAPOL packet-type field == 0x03
 * (EAPOL-Key).
 *
 * EAPOL-Key bytes after the LLC SNAP (8 bytes):
 *   [0]   protocol version
 *   [1]   packet type (3 = EAPOL-Key)
 *   [2-3] body length
 *   [4]   descriptor type (2 = RSN)
 *   [5-6] key info (big-endian)
 *
 * Key info patterns we expect during WPA2 PSK 4-way:
 *   M1: 0x008A   M2: 0x010A   M3: 0x13CA   M4: 0x030A
 *
 * Walks the mbuf via m_copydata so chained mbufs (which net80211 hands
 * us on the outbound EAPOL path -- the 802.11 hdr is one segment, the
 * encap'd body is another) are handled correctly.  An earlier
 * mtod()-only version silently bailed on TX-side M2/M4 because the
 * first segment is only 24 bytes.
 */
static bool
rtw88_eapol_key_info(const struct mbuf *m, uint16_t *out)
{
	uint8_t hdr[2];
	uint8_t snap_eapol[2 + 8 + 8];	/* slop for hdrsize peek */
	uint8_t snap[8];
	uint8_t eapol[8];
	uint16_t hdrlen;

	if (m->m_pkthdr.len < 24 + 8 + 7)
		return (false);
	m_copydata(m, 0, 2, hdr);
	/* must be data type (00b at bits 3:2). */
	if ((hdr[0] & IEEE80211_FC0_TYPE_MASK) != IEEE80211_FC0_TYPE_DATA)
		return (false);
	/*
	 * ieee80211_hdrsize only needs the FC bytes for the basic
	 * 24/26/30-byte sizing; peek the first 2 bytes into a flat
	 * scratch so the call works whether or not the mbuf is chained.
	 */
	m_copydata(m, 0, sizeof(snap_eapol), snap_eapol);
	hdrlen = ieee80211_hdrsize(snap_eapol);
	if (m->m_pkthdr.len < (int)(hdrlen + 8 + 7))
		return (false);
	m_copydata(m, hdrlen, 8, snap);
	/* SNAP for EAPOL: AA AA 03 00 00 00 88 8E. */
	if (snap[0] != 0xAA || snap[1] != 0xAA || snap[2] != 0x03)
		return (false);
	if (snap[6] != 0x88 || snap[7] != 0x8E)
		return (false);
	m_copydata(m, hdrlen + 8, sizeof(eapol), eapol);
	/* packet type 3 = EAPOL-Key. */
	if (eapol[1] != 0x03)
		return (false);
	*out = ((uint16_t)eapol[5] << 8) |
	       (uint16_t)eapol[6];
	return (true);
}

static void
rtw88_eapol_log(struct rtw88_usb_softc *sc, const struct mbuf *m,
    const char *dir)
{
	uint16_t key_info;
	const char *which;

	if (!rtw88_eapol_key_info(m, &key_info))
		return;
	/*
	 * Identify M1/M2/M3/M4 by the canonical key_info bits.  Bit 3 =
	 * pairwise; bit 8 = MIC present; bit 7 = secure; bit 6 = install.
	 * Anything outside the 4-way bestiary prints as raw hex.
	 */
	switch (key_info & 0x13FF) {
	case 0x008A: which = "M1"; break;
	case 0x010A: which = "M2"; break;
	case 0x13CA: which = "M3"; break;
	case 0x030A: which = "M4"; break;
	default:     which = "??"; break;
	}
	device_printf(sc->sc_dev,
	    "[T+%d ms] %s EAPOL-Key %s key_info=0x%04x\n",
	    rtw88_assoc_ms(sc), dir, which, key_info);
	/*
	 * On rx of M1, re-fire REG_FIFOPAGE_CTRL_2 = pg_addr | BCN_VALID_V1
	 * to match Linux's pre-M2 write (see reg_monitor.py --handshake-diff
	 * output 2026-07-02).  Opt-in via dev.rtw88_usb.0.m1_bcn_refire=1.
	 * usb_callback context holds sc_mtx already so the write is safe.
	 */
	if (sc->sc_m1_bcn_refire && dir[0] == 'r' &&
	    which[0] == 'M' && which[1] == '1') {
		/*
		 * pg_addr = RTW88_RSVD_DRV_ADDR = 0x1CC (defined below at
		 * line ~5570 — TXFF_PG_NUM(512) - RSVD_PG_NUM(52) = 460 =
		 * 0x1CC).  With BIT_BCN_VALID_V1 (0x8000) OR'd in: 0x81CC.
		 * Matches Linux's exact byte pattern from reg_monitor.py
		 * --handshake-diff output.  Literal avoids reshuffling the
		 * RSVD_* #define block above this function.
		 */
		uint16_t val = 0x81CC;
		(void)rtw88_usb_write2(sc, REG_FIFOPAGE_CTRL_2, val);
		sc->sc_stat_m1_bcn_refire++;
		device_printf(sc->sc_dev,
		    "[T+%d ms] M1 rx: re-fired FIFOPAGE_CTRL_2=0x%04x "
		    "(BCN_VALID_V1 refresh #%u)\n",
		    rtw88_assoc_ms(sc), val, sc->sc_stat_m1_bcn_refire);
	}
	/* Diagnostic dump of first 48 bytes for the M2/M4 mystery. */
	if (dir[0] == 't' && (which[0] == 'M') && m->m_pkthdr.len >= 48) {
		const uint8_t *b = mtod(m, const uint8_t *);
		device_printf(sc->sc_dev,
		    "  hdr: %02x%02x %02x%02x %02x%02x%02x%02x%02x%02x "
		    "%02x%02x%02x%02x%02x%02x %02x%02x%02x%02x%02x%02x "
		    "%02x%02x qos=%02x%02x llc=%02x%02x%02x%02x%02x%02x "
		    "eth=%02x%02x len=%d\n",
		    b[0], b[1], b[2], b[3],
		    b[4], b[5], b[6], b[7], b[8], b[9],
		    b[10], b[11], b[12], b[13], b[14], b[15],
		    b[16], b[17], b[18], b[19], b[20], b[21],
		    b[22], b[23], b[24], b[25],
		    b[26], b[27], b[28], b[29], b[30], b[31],
		    b[32], b[33], m->m_pkthdr.len);
	}

	/*
	 * Full-frame hex dump for M2/M4 tx.  Enables byte-for-byte diff of
	 * the EAPOL Key Data field (RSN IE, PMKID inclusion, ANonce echo,
	 * MIC) against Linux M2 in linux_m_hex/M2.hex.  Rate-limited to the
	 * first tx of each M step per session so a retry burst doesn't
	 * flood dmesg.
	 */
	if (sc->sc_eapol_fullhex && dir[0] == 't' && which[0] == 'M') {
		static char hexbuf[900];
		int i, off, need = m->m_pkthdr.len;
		uint8_t tmp[512];
		if (need > (int)sizeof(tmp))
			need = sizeof(tmp);
		m_copydata(m, 0, need, tmp);
		off = 0;
		for (i = 0; i < need && off < (int)sizeof(hexbuf) - 3; i++)
			off += snprintf(hexbuf + off, sizeof(hexbuf) - off,
			    "%02x", tmp[i]);
		device_printf(sc->sc_dev,
		    "  %s full-hex (%d bytes): %s\n", which, need, hexbuf);
	}
}

/*
 * Upgrade a plain-Data EAPOL frame to QoS-Data with TID=7 (VO) in-place.
 *
 * net80211's ieee80211_output.c:1548 explicitly strips QoS-Data encoding
 * from EAPOL frames as a legacy-AP workaround.  Modern APs (both APs we
 * tested 2026-07-02) advertise WMM + reject non-QoS Data once WMM is
 * negotiated.  Linux's rtw88 emits M2/M4 as QoS-Data TID=7 (VO); ours as
 * plain Data.  This rewrite matches Linux OTA representation.
 *
 * Layout before:
 *   [FC(2) Dur(2) Addr1(6) Addr2(6) Addr3(6) SeqCtrl(2)] [LLC(8) 88 8e] [body]
 * Layout after:
 *   [FC(2) Dur(2) Addr1(6) Addr2(6) Addr3(6) SeqCtrl(2)] [QoS(2)] [LLC(8) 88 8e] [body]
 *
 * Caller MUST invoke before rtw88_crypto_encap_if_needed so CCMP AAD
 * (M4 case) includes the QoS TID.  M2 is plaintext so ordering doesn't
 * affect it, but the same call site works for both.
 *
 * Returns the (possibly reallocated) mbuf, or NULL on failure (caller
 * must handle NULL as m_freem-equivalent).  Gated by sc_eapol_qos_upgrade.
 */
static struct mbuf *
rtw88_maybe_upgrade_eapol_qos(struct rtw88_usb_softc *sc, struct mbuf *m)
{
	uint16_t ki_dummy;
	uint8_t *p;
	int total;

	if (!sc->sc_eapol_qos_upgrade)
		return (m);
	if (m == NULL || m->m_pkthdr.len < 24 + 8 + 4)
		return (m);
	/*
	 * rtw88_eapol_key_info matches the exact "non-QoS Data + LLC SNAP
	 * with EAPOL ethertype + EAPOL-Key packet type" pattern.  Fails on
	 * already-QoS frames because ieee80211_hdrsize returns 26 for
	 * FC0=0x88, causing the LLC SNAP check at hdrlen to miss.  So a
	 * positive hit here is definitive proof this is a non-QoS EAPOL-
	 * Key frame that needs the upgrade.
	 */
	if (!rtw88_eapol_key_info(m, &ki_dummy))
		return (m);

	total = m->m_pkthdr.len;
	m = m_pullup(m, total);
	if (m == NULL)
		return (NULL);
	if (M_TRAILINGSPACE(m) < 2) {
		/*
		 * No tail room -- reallocate a fresh single-mbuf packet.  Rare
		 * on TX-path mbufs from wpa_supplicant (they're allocated with
		 * headroom + trailer for CCMP MIC).
		 */
		struct mbuf *n = m_get2(total + 2, M_NOWAIT, MT_DATA, M_PKTHDR);
		if (n == NULL) {
			m_freem(m);
			return (NULL);
		}
		m_move_pkthdr(n, m);
		m_copydata(m, 0, total, mtod(n, uint8_t *));
		n->m_len = total;
		n->m_pkthdr.len = total;
		m_freem(m);
		m = n;
	}

	/* Shift bytes 24..end right by 2, then insert QoS Ctrl. */
	p = mtod(m, uint8_t *);
	memmove(p + 26, p + 24, total - 24);
	p[24] = 7;	/* TID=7 (VO), no-ack=0, A-MSDU=0 */
	p[25] = 0;
	p[0] |= IEEE80211_FC0_SUBTYPE_QOS_DATA;	/* 0x08 -> 0x88 */
	/*
	 * Zero SeqCtrl (bytes 22-23).  Linux WORKING M2 has SeqCtrl=0x0000
	 * for the first TID=7 QoS-Data frame; ours had 0x0020 (seq_no=2)
	 * from net80211's global counter.  AP's per-TID dup-detection may
	 * be dropping our M2 for being out of expected sequence.  Verified
	 * by byte-diff of linux_working_m_hex/M2.hex vs dut_m_hex/M2.hex
	 * 2026-07-02.
	 */
	p[22] = 0;
	p[23] = 0;
	m->m_len += 2;
	m->m_pkthdr.len += 2;

	sc->sc_stat_eapol_qos_upgrade++;
	device_printf(sc->sc_dev,
	    "[T+%d ms] EAPOL QoS-Data upgrade #%u (FC[0] 0x08->0x88, "
	    "TID=7, len %d->%d)\n",
	    rtw88_assoc_ms(sc), sc->sc_stat_eapol_qos_upgrade,
	    total, total + 2);
	return (m);
}

/*
 * If the frame's 802.11 header has the Protected bit set, run SW crypto
 * encap so the body is real CCMP/TKIP/WEP ciphertext before we hand it
 * to the chip.  Without this net80211 leaves a gap in the mbuf for the
 * crypto header (per ieee80211_output.c:1380) but never fills it; we
 * then TX a frame with Protected=1 in the 802.11 header and plaintext
 * LLC SNAP in the body, which every AP silently drops.
 *
 * Special case: WPA2 4-way M4 (EAPOL-Key with key_info MIC+SECURE bits
 * both set) must go out ENCRYPTED per RFC 4a802.11-2016 § 12.7.2, but
 * FreeBSD net80211 leaves supplicant-originated raw EAPOL with
 * FC1.Protected=0 by design (Linux mac80211 sets it based on the CAM
 * entry).  Session 4 DUT capture 2026-07-01 confirmed AP silently
 * drops our plaintext M4 -> retries M3 -> deauth after ~10s.  Force
 * Protected=1 on outbound M4 so ieee80211_crypto_encap picks up the
 * newly-installed PTK.  M2 is unaffected (its key_info has MIC only,
 * no SECURE) and correctly goes out plaintext.
 *
 * Returns 0 on success (mbuf still valid, possibly with shifted m_data),
 * non-zero error on failure (caller must free mbuf).
 */
static int
rtw88_crypto_encap_if_needed(struct rtw88_usb_softc *sc,
    struct ieee80211_node *ni, struct mbuf *m)
{
	struct ieee80211_frame *wh;
	uint16_t key_info;

	if (ni == NULL || m->m_len < sizeof(*wh))
		return (0);
	wh = mtod(m, struct ieee80211_frame *);
	if ((wh->i_fc[1] & IEEE80211_FC1_PROTECTED) == 0) {
		/*
		 * EAPOL M4 detection: key_info bit 8 (MIC) + bit 9 (SECURE)
		 * both set, KEY_ACK bit 7 clear.  M1=0x008a, M2=0x010a,
		 * M4=0x030a.  Try to encrypt M4 with the freshly-installed
		 * PTK so the AP can decrypt it per RFC/IEEE 802.11-2016
		 * § 12.7.2 (the plaintext-M4 path is rejected by strict APs).
		 *
		 * If ieee80211_crypto_encap fails (typically because
		 * ni->ni_ucastkey.wk_cipher is still the "none" cipher at
		 * this instant — net80211's key install runs concurrently
		 * with supplicant's raw M4 xmit), fall back to plaintext
		 * emission.  That's what we did before this fix, so at
		 * worst we're no regression; occasionally the AP accepts
		 * a plaintext M4.  A follow-up commit should add a proper
		 * M4-holding queue keyed on PTK CAM-install completion.
		 */
		if (rtw88_eapol_key_info(m, &key_info) &&
		    (key_info & 0x0300) == 0x0300 &&
		    (key_info & 0x0080) == 0) {
			struct mbuf *m_pre = m;
			int len_pre = m->m_pkthdr.len;
			wh->i_fc[1] |= IEEE80211_FC1_PROTECTED;
			if (ieee80211_crypto_encap(ni, m) != NULL) {
				device_printf(sc->sc_dev,
				    "[T+%d ms] M4 SW CCMP encap succeeded "
				    "at first-try (key_info=0x%04x)\n",
				    rtw88_assoc_ms(sc), key_info);
				return (0);
			}
			device_printf(sc->sc_dev,
			    "[T+%d ms] M4 encap NULL: m_pre=%p len_pre=%d "
			    "ni=%p wk_cipher=%p\n",
			    rtw88_assoc_ms(sc), m_pre, len_pre, ni,
			    ni->ni_ucastkey.wk_cipher);
			/*
			 * Encap failed -- ni->ni_ucastkey still CIPHER_NONE
			 * (wpa_supplicant on -D bsd sends M4 before SET_KEY).
			 * Leave Protected=1 and route through the existing
			 * sc_tx_pending_q gate: tx_enqueue_locked sees
			 * Protected+ptk_install_pending and parks m there.
			 * Force ptk_install_pending in case iv_key_set
			 * hasn't fired yet.  Marker sc_m4_needs_encap tells
			 * sc_cam_task's post-install drain to re-run
			 * ieee80211_crypto_encap on this specific mbuf before
			 * routing to VO.  Ni ref rides on m->m_pkthdr.rcvif
			 * per net80211 raw_output convention (ieee80211_output.c
			 * line 572), released when we eventually m_freem
			 * post-tx-complete.
			 */
			RTW88_LOCK(sc);
			if (sc->sc_m4_needs_encap != NULL) {
				/* One-slot: fall back to plaintext for extras. */
				RTW88_UNLOCK(sc);
				wh = mtod(m, struct ieee80211_frame *);
				wh->i_fc[1] &= ~IEEE80211_FC1_PROTECTED;
				device_printf(sc->sc_dev,
				    "[T+%d ms] M4 encap-slot busy; plaintext\n",
				    rtw88_assoc_ms(sc));
				return (0);
			}
			sc->sc_m4_needs_encap = m;
			sc->sc_ptk_install_pending = true;
			RTW88_UNLOCK(sc);
			device_printf(sc->sc_dev,
			    "[T+%d ms] M4 encap deferred to CAM-install drain "
			    "(key_info=0x%04x)\n",
			    rtw88_assoc_ms(sc), key_info);
			/* Flow through: tx_enqueue_locked parks in pending_q. */
			return (0);
		}
		return (0);
	}
	if (ieee80211_crypto_encap(ni, m) == NULL) {
		static struct timeval ce_last;
		static const struct timeval ce_iv = { 1, 0 };
		if (ratecheck(&ce_last, &ce_iv))
			device_printf(sc->sc_dev,
			    "ieee80211_crypto_encap returned NULL "
			    "(further throttled)\n");
		return (ENOBUFS);
	}
	return (0);
}

static int
rtw88_transmit(struct ieee80211com *ic, struct mbuf *m)
{
	struct rtw88_usb_softc *sc = ic->ic_softc;
	struct ieee80211_node *ni;
	int error;

	/*
	 * Per net80211 ic_transmit contract the node reference rides on
	 * m_pkthdr.rcvif.  We use it only as a node pointer for SW crypto
	 * encap; the ref itself stays attached to the mbuf and is released
	 * by m_freem -> ieee80211_freebuf_free_node eventually.
	 */
	ni = (struct ieee80211_node *)m->m_pkthdr.rcvif;

	rtw88_eapol_log(sc, m, "tx");
	m = rtw88_maybe_upgrade_eapol_qos(sc, m);
	if (m == NULL)
		return (ENOMEM);
	error = rtw88_crypto_encap_if_needed(sc, ni, m);
	if (error != 0) {
		m_freem(m);
		return (error);
	}

	RTW88_LOCK(sc);
	error = rtw88_tx_enqueue_locked(sc, m);
	RTW88_UNLOCK(sc);
	if (error != 0)
		m_freem(m);
	return (error);
}

/*
 * WME / EDCA parameter push.  Net80211 invokes wme_update during
 * association so the AP's QoS settings reach the chip.  Walks
 * ic->ic_wme.wme_chanParams.cap_wmeParams[] and writes each AC
 * to REG_EDCA_*_PARAM.  Equivalent of Linux's per-AC __rtw_conf_tx
 * fired via the rtw_ops_conf_tx mac80211 callback.
 */
static int
rtw88_wme_update(struct ieee80211com *ic)
{
	struct rtw88_usb_softc *sc = ic->ic_softc;
	int error;

	RTW88_LOCK(sc);
	error = rtw88_conf_tx_all_ac_locked(sc, ic);
	RTW88_UNLOCK(sc);
	if (error != 0)
		device_printf(sc->sc_dev,
		    "wme_update: EDCA write failed: %d\n", error);
	/*
	 * net80211 ignores the return value of wme_update (per
	 * vap_update_wme); always return 0 so net80211's bookkeeping
	 * stays consistent even if a vendor-request hiccupped.
	 */
	return (0);
}

/*
 * Toggle the chip's RX filter to "accept everything" for monitor mode
 * + pcap capture.  net80211 calls ic_update_promisc with ic_promisc
 * incremented when monitor mode goes up or bpf adds a listener.  Sets:
 *   BIT_RCR_AAP   accept all packets (including not-to-us)
 *   BIT_APP_FCS   keep the 4-byte FCS in the body
 *   BIT_APP_ICV   keep the 8-byte ICV (CCMP MIC) in the body
 *   BIT_APP_PHYSTS keep PHY status drv_info (we already use it for RX)
 * Cleared when ic_promisc drops to 0.
 */
static void
rtw88_update_promisc(struct ieee80211com *ic)
{
	struct rtw88_usb_softc *sc = ic->ic_softc;
	uint32_t rcr;
	/*
	 * Only BIT_RCR_AAP (accept-all-packets) gets toggled here.
	 * BIT_APP_FCS / BIT_APP_ICV / BIT_APP_PHYSTS are part of the
	 * chip's baseline RCR (WLAN_RCR_CFG = 0xE400220E) — Linux
	 * never clears them and the chip's RX engine drops bytes off
	 * the frame tail if they're off, which silently broke STA-
	 * mode unicast RX after every promisc=0 transition (e.g.
	 * monitor mode + radiotap path teardown).  Toggling AAP is
	 * sufficient to capture neighbour STA / arbitrary-SSID
	 * frames; the FCS/ICV body bytes are needed by net80211 even
	 * for non-promisc RX.
	 */
	uint32_t mask = BIT_RCR_AAP;

	RTW88_LOCK(sc);
	if (rtw88_usb_read4(sc, REG_RCR, &rcr) != 0) {
		RTW88_UNLOCK(sc);
		return;
	}
	if (ic->ic_promisc > 0)
		rcr |= mask;
	else
		rcr &= ~mask;
	(void)rtw88_usb_write4(sc, REG_RCR, rcr);
	RTW88_UNLOCK(sc);
}

/*
 * Multicast-list filter is a no-op for the 8821C silicon under STA mode:
 * BIT_RCR_AM (accept multicast) is on by default, BIT_RCR_AB (accept
 * broadcast) is on by default, and there is no per-group hash table to
 * program.  Hook is wired so net80211 doesn't NULL-deref.
 */
static void
rtw88_update_mcast(struct ieee80211com *ic __unused)
{
}

static int
rtw88_raw_xmit(struct ieee80211_node *ni, struct mbuf *m,
    const struct ieee80211_bpf_params *params __unused)
{
	struct rtw88_usb_softc *sc = ni->ni_ic->ic_softc;
	int error;

	rtw88_eapol_log(sc, m, "tx");
	m = rtw88_maybe_upgrade_eapol_qos(sc, m);
	if (m == NULL)
		return (ENOMEM);
	error = rtw88_crypto_encap_if_needed(sc, ni, m);
	if (error != 0) {
		m_freem(m);
		return (error);
	}

	RTW88_LOCK(sc);
	error = rtw88_tx_enqueue_locked(sc, m);
	RTW88_UNLOCK(sc);
	if (error != 0)
		m_freem(m);
	return (error);
}

/*
 * EFUSE block (physical read + logical walker + MAC extract) moved to
 * sys/dev/rtw88/rtw88_efuse.c; chip-specific offsets come from
 * rtw88_8821c_chip_info.  Call rtw88_efuse_read(&sc->sc_rtwdev) from
 * attach and read the results from sc->sc_rtwdev.efuse.
 */

/*
 * MAC initialisation -- port of reference mainline mac.c rtw_mac_init()
 * for 8821C / USB / 3081 WCPU.  Six phases, executed inside the chip
 * after the firmware download succeeds:
 *
 *   1. txdma_queue_mapping -- program REG_TXDMA_PQ_MAP from the
 *      8821C rqpn table for 3-bulk-OUT (rqpn_table_8821c[3] =
 *      {NORMAL, NORMAL, LOW, LOW, HIGH, HIGH}); bring MAC_TRX engines
 *      up via REG_CR=MAC_TRX_ENABLE; assert RXDMA arbitration BW
 *      pin in TXDMA_PQ_MAP for USB.
 *
 *   2. priority_queue_cfg -- write FIFOPAGE_INFO_1..5 with the
 *      per-priority page counts derived from page_table_8821c[3]
 *      = {16,16,16,0,1}, load the rsvd boundary (page 460 with the
 *      8821C reserved-page budget of 52 pages out of 512), kick
 *      AUTO_LLT, then clear REG_CR+3.
 *
 *   3. init_h2c -- park the H2C ring base/tail/read at the rsvd
 *      h2cq page (page 500 << 7 = 0xFA00) and enable the H2C path
 *      via REG_H2C_INFO and REG_TXDMA_OFFSET_CHK+1 bit 7.  Required
 *      for ANY H2C cmd to ever land.
 *
 *   4. rtw8821c_mac_init -- ~40 register writes covering protocol
 *      (AMPDU, RTS, EDCA), beacon control, WMAC filtering (RXFLTMAP,
 *      RCR, TCR), SND_PTCL VHT-CRC.  Mirrors reference rtw8821c.c
 *      rtw8821c_mac_init().
 *
 *   5. drv_info_cfg -- set PHY status size to 4 bytes, lift the
 *      low nibble of TRXFF_BNDY+1 to 0xF (fix for 3081 rxdesc len),
 *      gate APP_PHYSTS into RCR, clear WMAC_OPTION_FUNCTION+4
 *      bits 8 & 9.
 *
 *   6. usb_init_burst -- HIGH-speed USB: 512-byte burst + BURST_CNT
 *      + DMA_MODE in REG_RXDMA_MODE; assert DROP_DATA_EN in
 *      REG_TXDMA_OFFSET_CHK (the host side will drop a TX descriptor
 *      whose offset check fails instead of stalling forever).
 *
 * Caller must hold sc_mtx.  All register access happens over EP0
 * vendor requests via the rtw88_usb_read/write helpers.
 */

/* 8821C reserved-page bookkeeping (sizes in 128-byte pages). */
#define	RTW88_TX_PAGE_SIZE_SHIFT	7
#define	RTW88_TXFF_SIZE			65536
#define	RTW88_TXFF_PG_NUM						\
	(RTW88_TXFF_SIZE >> RTW88_TX_PAGE_SIZE_SHIFT)		/* 512 */
#define	RTW88_RSVD_DRV_PG_NUM		8
#define	RTW88_RSVD_H2C_EXTRAINFO_NUM	24
#define	RTW88_RSVD_H2C_STATICINFO_NUM	8
#define	RTW88_RSVD_H2CQ_NUM		8
#define	RTW88_RSVD_CPU_INSTR_NUM	0
#define	RTW88_RSVD_FW_TXBUF_NUM		4
#define	RTW88_CSI_BUF_PG_NUM		0
#define	RTW88_RSVD_PG_NUM						\
	(RTW88_RSVD_DRV_PG_NUM + RTW88_RSVD_H2C_EXTRAINFO_NUM +	\
	 RTW88_RSVD_H2C_STATICINFO_NUM + RTW88_RSVD_H2CQ_NUM +	\
	 RTW88_RSVD_CPU_INSTR_NUM + RTW88_RSVD_FW_TXBUF_NUM +	\
	 RTW88_CSI_BUF_PG_NUM)						/* 52 */
#define	RTW88_ACQ_PG_NUM	(RTW88_TXFF_PG_NUM - RTW88_RSVD_PG_NUM) /* 460 */
#define	RTW88_RSVD_BOUNDARY	RTW88_ACQ_PG_NUM			/* 460 */

/*
 * Reserved-page addresses (walk from txff_pg_num down).  Only
 * h2cq_addr is used right now (init_h2c base); others are present
 * for future H2C / FW-cmd plumbing.
 */
#define	RTW88_RSVD_CSIBUF_ADDR		(RTW88_TXFF_PG_NUM - RTW88_CSI_BUF_PG_NUM)
#define	RTW88_RSVD_FW_TXBUF_ADDR	(RTW88_RSVD_CSIBUF_ADDR - RTW88_RSVD_FW_TXBUF_NUM)
#define	RTW88_RSVD_CPU_INSTR_ADDR	(RTW88_RSVD_FW_TXBUF_ADDR - RTW88_RSVD_CPU_INSTR_NUM)
#define	RTW88_RSVD_H2CQ_ADDR		(RTW88_RSVD_CPU_INSTR_ADDR - RTW88_RSVD_H2CQ_NUM)
#define	RTW88_RSVD_H2C_STA_ADDR		(RTW88_RSVD_H2CQ_ADDR - RTW88_RSVD_H2C_STATICINFO_NUM)
#define	RTW88_RSVD_H2C_INFO_ADDR	(RTW88_RSVD_H2C_STA_ADDR - RTW88_RSVD_H2C_EXTRAINFO_NUM)
#define	RTW88_RSVD_DRV_ADDR		(RTW88_RSVD_H2C_INFO_ADDR - RTW88_RSVD_DRV_PG_NUM)

/* page_table_8821c[3] -- 3-bulkout USB row. */
/*
 * 8821C / 4-bulkout USB page table (Linux rtw8821c.c page_table_8821c[4]:
 * {hq=16, nq=16, lq=16, exq=14, gapq=1}).  EXQ was 0 in the 3-bulkout
 * row -- but with MG mapped to EXTRA in the 4-bulkout PQ_MAP, EXQ=0
 * means the MGMT FIFO has zero pages allocated and chip silently drops
 * AUTH/ASSOC.  Setting EXQ=14 matches Linux.
 */
/*
 * page_table_8821c[3] (3-EP USB config) = {16, 16, 16, 0, 1}.
 * Fields: hq_num, nq_num, lq_num, eq_num, gap.  With MG now routed to
 * HIGH (not EXTRA) per the PQ_MAP fix above, the EXTRA queue must have
 * 0 pages — chip won't try to enqueue into an unused FIFO.  Frees up
 * 14 pages back into PUBQ for shared use.
 */
#define	RTW88_PG_HQ_NUM		16
#define	RTW88_PG_NQ_NUM		16
#define	RTW88_PG_LQ_NUM		16
#define	RTW88_PG_EXQ_NUM	0
#define	RTW88_PG_GAPQ_NUM	1
#define	RTW88_PG_PUBQ_NUM						\
	(RTW88_ACQ_PG_NUM - RTW88_PG_HQ_NUM - RTW88_PG_NQ_NUM -	\
	 RTW88_PG_LQ_NUM - RTW88_PG_EXQ_NUM - RTW88_PG_GAPQ_NUM)	/* 411 */

#define	RTW88_RXFF_SIZE		16384
#define	RTW88_C2H_PKT_BUF	256
#define	RTW88_USB_TX_AGG_DESC_NUM 3

/*
 * 8821C "WMAC magic numbers".  Lifted verbatim from reference mainline
 * rtw8821c.h so the diff stays minimal.  Each constant is the raw
 * value the chip MUST see at its corresponding register or the
 * protocol/EDCA/beacon engines mis-program.
 */
#define	WLAN_SLOT_TIME			0x09
#define	WLAN_PIFS_TIME			0x19
#define	WLAN_SIFS_CCK_CONT_TX		0x0A
#define	WLAN_SIFS_OFDM_CONT_TX		0x0E
#define	WLAN_SIFS_CCK_TRX		0x10
#define	WLAN_SIFS_OFDM_TRX		0x10
#define	WLAN_VO_TXOP_LIMIT		0x186
#define	WLAN_VI_TXOP_LIMIT		0x3BC
#define	WLAN_RDG_NAV			0x05
#define	WLAN_TXOP_NAV			0x1B
#define	WLAN_CCK_RX_TSF			0x30
#define	WLAN_OFDM_RX_TSF		0x30
#define	WLAN_TBTT_PROHIBIT		0x04
#define	WLAN_TBTT_HOLD_TIME		0x064
#define	WLAN_DRV_EARLY_INT		0x04
#define	WLAN_BCN_DMA_TIME		0x02
#define	WLAN_RX_FILTER0			0x0FFFFFFF
#define	WLAN_RX_FILTER2			0xFFFF
#define	WLAN_RCR_CFG			0xE400220E
/* WLAN_RCR_AUTH_READY and BIT_RCR_CBSSID_BCN moved to rtw88_regs.h
 * so the SCAN->AUTH newstate hook (which appears earlier in the
 * source) can reference them. */
#define	WLAN_RXPKT_MAX_SZ_512		0x18	/* 12288 >> 9 */
#define	WLAN_AMPDU_MAX_TIME		0x70
#define	WLAN_RTS_LEN_TH			0xFF
#define	WLAN_RTS_TX_TIME_TH		0x08
#define	WLAN_MAX_AGG_PKT_LIMIT		0x20
#define	WLAN_RTS_MAX_AGG_PKT_LIMIT	0x20
#define	FAST_EDCA_VO_TH			0x06
#define	FAST_EDCA_VI_TH			0x06
#define	FAST_EDCA_BE_TH			0x06
#define	FAST_EDCA_BK_TH			0x06
#define	WLAN_BAR_RETRY_LIMIT		0x01
#define	WLAN_RA_TRY_RATE_AGG_LIMIT	0x08
#define	WLAN_TX_FUNC_CFG1		0x30
#define	WLAN_TX_FUNC_CFG2		0x30
#define	WLAN_MAC_OPT_NORM_FUNC1		0x98
#define	WLAN_MAC_OPT_FUNC2		0xB0810041
#define	WLAN_PRE_TXCNT_TIME_TH		0x1E4

#define	WLAN_SIFS_CFG							\
	(WLAN_SIFS_CCK_CONT_TX |					\
	 (WLAN_SIFS_OFDM_CONT_TX << 8) |				\
	 (WLAN_SIFS_CCK_TRX << 16) |					\
	 (WLAN_SIFS_OFDM_TRX << 24))
#define	WLAN_TBTT_TIME							\
	(WLAN_TBTT_PROHIBIT | (WLAN_TBTT_HOLD_TIME << 8))
#define	WLAN_NAV_CFG	(WLAN_RDG_NAV | (WLAN_TXOP_NAV << 16))
#define	WLAN_RX_TSF_CFG	(WLAN_CCK_RX_TSF | (WLAN_OFDM_RX_TSF << 8))

/*
 * txdma_pq_map encoding for the 8821C / 3-bulk-OUT USB row (our case).
 * Per USB descriptor walk: 8821CU exposes 3 OUT bulk EPs (0x05/0x06/
 * 0x08), so Linux indexes rqpn_table_8821c[3]:
 *   {vo=NORMAL, vi=NORMAL, be=LOW, bk=LOW, mg=HIGH, hi=HIGH}
 *   = {NORMAL=2, NORMAL=2, LOW=1, LOW=1, HIGH=3, HIGH=3}
 *
 * Enum (Linux main.h:1014): EXTRA=0, LOW=1, NORMAL=2, HIGH=3.
 * Field shifts in REG_TXDMA_PQ_MAP: VO@4, VI@6, BE@8, BK@10, MG@12, HI@14.
 *
 * Was previously rqpn_table_8821c[4] (MG=EXTRA=0).  But our chip is
 * 3-EP, not 4-EP, so the 4-EP row tells the chip to route QSEL=7 (VO)
 * to NORMAL FIFO but tells it MG goes to EXTRA FIFO -- and EXTRA FIFO
 * has zero pages allocated (since there's no EXTRA endpoint).  Chip's
 * TX scheduler then silently drops or mis-routes EAPOL.  Discovered
 * 2026-06-26 after the endpoint-routing fix in ad4bdd6 still left us
 * 0/5 vs Linux 5/5.
 */
#define	RTW88_TXDMA_PQ_MAP_8821CU					\
	((2U << 4) | (2U << 6) | (1U << 8) | (1U << 10) |		\
	 (3U << 12) | (3U << 14))
#define	BIT_RXDMA_ARBBW_EN	(1U << 0)

static int
rtw88_txdma_queue_mapping(struct rtw88_usb_softc *sc)
{
	int error;

	RTW88_ASSERT_LOCKED(sc);

	if ((error = rtw88_usb_write2(sc, REG_TXDMA_PQ_MAP,
	    RTW88_TXDMA_PQ_MAP_8821CU)) != 0)
		return (error);

	if ((error = rtw88_usb_write1(sc, REG_CR, 0)) != 0)
		return (error);
	/*
	 * MAC_TRX_ENABLE has BIT_MACRXEN at bit 8 -- a 16-bit value.
	 * reference's rtw_write8(REG_CR, MAC_TRX_ENABLE) silently truncates
	 * the bit-8 MACRXEN bit; mirror that here with an 8-bit write
	 * of the low byte (BIT_MACRXEN lands in REG_CR+1 via priority
	 * queue cfg later if needed).
	 */
	if ((error = rtw88_usb_write1(sc, REG_CR,
	    (uint8_t)(MAC_TRX_ENABLE & 0xFF))) != 0)
		return (error);
	if ((error = rtw88_usb_write4(sc, REG_H2CQ_CSR, BIT_H2CQ_FULL)) != 0)
		return (error);
	/* USB: assert RXDMA arbitration BW pin in TXDMA_PQ_MAP byte 0. */
	return (rtw88_usb_setbits1(sc, REG_TXDMA_PQ_MAP, BIT_RXDMA_ARBBW_EN));
}

static int
rtw88_priority_queue_cfg(struct rtw88_usb_softc *sc)
{
	uint32_t val;
	int error, try;

	RTW88_ASSERT_LOCKED(sc);

	if ((error = rtw88_usb_write2(sc, REG_FIFOPAGE_INFO_1,
	    RTW88_PG_HQ_NUM)) != 0)
		return (error);
	if ((error = rtw88_usb_write2(sc, REG_FIFOPAGE_INFO_2,
	    RTW88_PG_LQ_NUM)) != 0)
		return (error);
	if ((error = rtw88_usb_write2(sc, REG_FIFOPAGE_INFO_3,
	    RTW88_PG_NQ_NUM)) != 0)
		return (error);
	if ((error = rtw88_usb_write2(sc, REG_FIFOPAGE_INFO_4,
	    RTW88_PG_EXQ_NUM)) != 0)
		return (error);
	if ((error = rtw88_usb_write2(sc, REG_FIFOPAGE_INFO_5,
	    RTW88_PG_PUBQ_NUM)) != 0)
		return (error);
	if ((error = rtw88_usb_read4(sc, REG_RQPN_CTRL_2, &val)) != 0)
		return (error);
	if ((error = rtw88_usb_write4(sc, REG_RQPN_CTRL_2,
	    val | BIT_LD_RQPN)) != 0)
		return (error);

	if ((error = rtw88_usb_write2(sc, REG_FIFOPAGE_CTRL_2,
	    RTW88_RSVD_BOUNDARY)) != 0)
		return (error);
	if ((error = rtw88_usb_setbits1(sc, REG_FWHW_TXQ_CTRL + 2,
	    BIT_EN_WR_FREE_TAIL >> 16)) != 0)
		return (error);

	if ((error = rtw88_usb_write2(sc, REG_BCNQ_BDNY_V1,
	    RTW88_RSVD_BOUNDARY)) != 0)
		return (error);
	if ((error = rtw88_usb_write2(sc, REG_FIFOPAGE_CTRL_2 + 2,
	    RTW88_RSVD_BOUNDARY)) != 0)
		return (error);
	if ((error = rtw88_usb_write2(sc, REG_BCNQ1_BDNY_V1,
	    RTW88_RSVD_BOUNDARY)) != 0)
		return (error);
	if ((error = rtw88_usb_write4(sc, REG_RXFF_BNDY,
	    RTW88_RXFF_SIZE - RTW88_C2H_PKT_BUF - 1)) != 0)
		return (error);

	/* USB-only: program TX-aggregation block-descriptor count. */
	if ((error = rtw88_usb_read4(sc, REG_AUTO_LLT_V1, &val)) != 0)
		return (error);
	val = (val & ~BIT_MASK_BLK_DESC_NUM) |
	    ((RTW88_USB_TX_AGG_DESC_NUM << 4) & BIT_MASK_BLK_DESC_NUM);
	if ((error = rtw88_usb_write4(sc, REG_AUTO_LLT_V1, val)) != 0)
		return (error);
	if ((error = rtw88_usb_write1(sc, REG_AUTO_LLT_V1 + 3,
	    RTW88_USB_TX_AGG_DESC_NUM)) != 0)
		return (error);
	if ((error = rtw88_usb_setbits1(sc, REG_TXDMA_OFFSET_CHK + 1,
	    (1U << 1))) != 0)
		return (error);

	if ((error = rtw88_usb_setbits1(sc, REG_AUTO_LLT_V1,
	    BIT_AUTO_INIT_LLT_V1)) != 0)
		return (error);

	/* Poll for chip to clear the AUTO_INIT bit (= LLT load complete). */
	for (try = 0; try < 50; try++) {
		if ((error = rtw88_usb_read4(sc, REG_AUTO_LLT_V1, &val)) != 0)
			return (error);
		if ((val & BIT_AUTO_INIT_LLT_V1) == 0)
			break;
		usb_pause_mtx(&sc->sc_mtx, USB_MS_TO_TICKS(1));
	}
	if (try == 50) {
		device_printf(sc->sc_dev,
		    "priority_queue_cfg: LLT init never cleared\n");
		return (EBUSY);
	}

	return (rtw88_usb_write1(sc, REG_CR + 3, 0));
}

static int
rtw88_init_h2c(struct rtw88_usb_softc *sc)
{
	const uint32_t h2cq_addr =
	    RTW88_RSVD_H2CQ_ADDR << RTW88_TX_PAGE_SIZE_SHIFT;
	const uint32_t h2cq_size =
	    RTW88_RSVD_H2CQ_NUM << RTW88_TX_PAGE_SIZE_SHIFT;
	uint32_t val32;
	uint8_t val8;
	int error;

	RTW88_ASSERT_LOCKED(sc);

	if ((error = rtw88_usb_read4(sc, REG_H2C_HEAD, &val32)) != 0)
		return (error);
	val32 = (val32 & 0xFFFC0000) | h2cq_addr;
	if ((error = rtw88_usb_write4(sc, REG_H2C_HEAD, val32)) != 0)
		return (error);

	if ((error = rtw88_usb_read4(sc, REG_H2C_READ_ADDR, &val32)) != 0)
		return (error);
	val32 = (val32 & 0xFFFC0000) | h2cq_addr;
	if ((error = rtw88_usb_write4(sc, REG_H2C_READ_ADDR, val32)) != 0)
		return (error);

	if ((error = rtw88_usb_read4(sc, REG_H2C_TAIL, &val32)) != 0)
		return (error);
	val32 = (val32 & 0xFFFC0000) | (h2cq_addr + h2cq_size);
	if ((error = rtw88_usb_write4(sc, REG_H2C_TAIL, val32)) != 0)
		return (error);

	if ((error = rtw88_usb_read1(sc, REG_H2C_INFO, &val8)) != 0)
		return (error);
	val8 = (val8 & 0xFC) | 0x01;
	if ((error = rtw88_usb_write1(sc, REG_H2C_INFO, val8)) != 0)
		return (error);

	if ((error = rtw88_usb_read1(sc, REG_H2C_INFO, &val8)) != 0)
		return (error);
	val8 = (val8 & 0xFB) | 0x04;
	if ((error = rtw88_usb_write1(sc, REG_H2C_INFO, val8)) != 0)
		return (error);

	if ((error = rtw88_usb_read1(sc, REG_TXDMA_OFFSET_CHK + 1,
	    &val8)) != 0)
		return (error);
	val8 = (val8 & 0x7F) | 0x80;
	return (rtw88_usb_write1(sc, REG_TXDMA_OFFSET_CHK + 1, val8));
}

static int
rtw88_8821c_mac_init(struct rtw88_usb_softc *sc)
{
	const uint16_t pre_txcnt = WLAN_PRE_TXCNT_TIME_TH | BIT_EN_PRECNT;
	const uint32_t prot_mode = WLAN_RTS_LEN_TH |
	    (WLAN_RTS_TX_TIME_TH << 8) |
	    (WLAN_MAX_AGG_PKT_LIMIT << 16) |
	    (WLAN_RTS_MAX_AGG_PKT_LIMIT << 24);
	const uint16_t bar_mode = WLAN_BAR_RETRY_LIMIT |
	    (WLAN_RA_TRY_RATE_AGG_LIMIT << 8);
	int error;

	RTW88_ASSERT_LOCKED(sc);

	/* Protocol */
	if ((error = rtw88_usb_write1(sc, REG_AMPDU_MAX_TIME_V1,
	    WLAN_AMPDU_MAX_TIME)) != 0)
		return (error);
	if ((error = rtw88_usb_setbits1(sc, REG_TX_HANG_CTRL,
	    BIT_EN_EOF_V1)) != 0)
		return (error);
	if ((error = rtw88_usb_write1(sc, REG_PRECNT_CTRL,
	    (uint8_t)(pre_txcnt & 0xFF))) != 0)
		return (error);
	if ((error = rtw88_usb_write1(sc, REG_PRECNT_CTRL + 1,
	    (uint8_t)(pre_txcnt >> 8))) != 0)
		return (error);
	if ((error = rtw88_usb_write4(sc, REG_PROT_MODE_CTRL,
	    prot_mode)) != 0)
		return (error);
	if ((error = rtw88_usb_write2(sc, REG_BAR_MODE_CTRL + 2,
	    bar_mode)) != 0)
		return (error);
	if ((error = rtw88_usb_write1(sc, REG_FAST_EDCA_VOVI_SETTING,
	    FAST_EDCA_VO_TH)) != 0)
		return (error);
	if ((error = rtw88_usb_write1(sc, REG_FAST_EDCA_VOVI_SETTING + 2,
	    FAST_EDCA_VI_TH)) != 0)
		return (error);
	if ((error = rtw88_usb_write1(sc, REG_FAST_EDCA_BEBK_SETTING,
	    FAST_EDCA_BE_TH)) != 0)
		return (error);
	if ((error = rtw88_usb_write1(sc, REG_FAST_EDCA_BEBK_SETTING + 2,
	    FAST_EDCA_BK_TH)) != 0)
		return (error);
	if ((error = rtw88_usb_setbits1(sc, REG_INIRTS_RATE_SEL,
	    (1U << 5))) != 0)
		return (error);

	/* EDCA / timers */
	if ((error = rtw88_usb_clrbits1(sc, REG_TIMER0_SRC_SEL,
	    BIT_TSFT_SEL_TIMER0)) != 0)
		return (error);
	if ((error = rtw88_usb_write2(sc, REG_TXPAUSE, 0)) != 0)
		return (error);
	if ((error = rtw88_usb_write1(sc, REG_SLOT, WLAN_SLOT_TIME)) != 0)
		return (error);
	if ((error = rtw88_usb_write1(sc, REG_PIFS, WLAN_PIFS_TIME)) != 0)
		return (error);
	if ((error = rtw88_usb_write4(sc, REG_SIFS, WLAN_SIFS_CFG)) != 0)
		return (error);
	if ((error = rtw88_usb_write2(sc, REG_EDCA_VO_PARAM + 2,
	    WLAN_VO_TXOP_LIMIT)) != 0)
		return (error);
	if ((error = rtw88_usb_write2(sc, REG_EDCA_VI_PARAM + 2,
	    WLAN_VI_TXOP_LIMIT)) != 0)
		return (error);
	if ((error = rtw88_usb_write4(sc, REG_RD_NAV_NXT,
	    WLAN_NAV_CFG)) != 0)
		return (error);
	if ((error = rtw88_usb_write2(sc, REG_RXTSF_OFFSET_CCK,
	    WLAN_RX_TSF_CFG)) != 0)
		return (error);

	/* Beacon */
	if ((error = rtw88_usb_setbits1(sc, REG_BCN_CTRL,
	    BIT_EN_BCN_FUNCTION)) != 0)
		return (error);
	if ((error = rtw88_usb_write4(sc, REG_TBTT_PROHIBIT,
	    WLAN_TBTT_TIME)) != 0)
		return (error);
	if ((error = rtw88_usb_write1(sc, REG_DRVERLYINT,
	    WLAN_DRV_EARLY_INT)) != 0)
		return (error);
	if ((error = rtw88_usb_write1(sc, REG_BCNDMATIM,
	    WLAN_BCN_DMA_TIME)) != 0)
		return (error);
	if ((error = rtw88_usb_clrbits1(sc, REG_TX_PTCL_CTRL + 1,
	    BIT_SIFS_BK_EN >> 8)) != 0)
		return (error);

	/* WMAC */
	if ((error = rtw88_usb_write4(sc, REG_RXFLTMAP0,
	    WLAN_RX_FILTER0)) != 0)
		return (error);
	if ((error = rtw88_usb_write2(sc, REG_RXFLTMAP2,
	    WLAN_RX_FILTER2)) != 0)
		return (error);
	if ((error = rtw88_usb_write4(sc, REG_RCR, WLAN_RCR_CFG)) != 0)
		return (error);
	if ((error = rtw88_usb_write1(sc, REG_RX_PKT_LIMIT,
	    WLAN_RXPKT_MAX_SZ_512)) != 0)
		return (error);
	if ((error = rtw88_usb_write1(sc, REG_TCR + 2,
	    WLAN_TX_FUNC_CFG2)) != 0)
		return (error);
	if ((error = rtw88_usb_write1(sc, REG_TCR + 1,
	    WLAN_TX_FUNC_CFG1)) != 0)
		return (error);
	if ((error = rtw88_usb_write1(sc, REG_ACKTO_CCK, 0x40)) != 0)
		return (error);
	/*
	 * REG_ACKTO (0x0640) -- OFDM ACK timeout. Linux ALWAYS writes 0x40
	 * here right after REG_ACKTO_CCK (per usbmon trace of rtw88_8821cu
	 * 6.8.0-124 against same AP, 2026-06-25). Without it, the chip's
	 * OFDM ACK timeout is at HW default (likely too short to schedule
	 * the ACK TX in the SIFS deadline) -- chip RXes the unicast data
	 * frame but its HW ACK never reaches the AP. AP retransmits, marks
	 * us inactive after ~10s, sends DEAUTH reason=4 INACTIVITY.
	 *
	 * Discovered via Linux/FreeBSD usbmon diff on same RTL8821CU + same
	 * home AP -- Linux completes the 4-way handshake and stays
	 * associated, we drop to SCANNING within ~6s of COMPLETED.
	 */
	if ((error = rtw88_usb_write1(sc, REG_ACKTO, 0x40)) != 0)
		return (error);
	/*
	 * REG_RRSR (Response Rate Set, 0x0440) controls the rates the chip's
	 * MAC uses for response frames (ACK/CTS) it generates internally to
	 * incoming unicast.  Linux writes RRSR_INIT_2G = 0x15F at chip init
	 * (rtw_phy_setup, main.c:1286 -> phy.c:664).  Without it the chip's
	 * MAC has no rates enabled for ACK generation — it RXes the frame,
	 * delivers to net80211, but silently fails to emit the SIFS-scheduled
	 * ACK.  AP doesn't see an ACK, retransmits at maximum aggression
	 * (~2ms intervals), eventually deauths.  Discovered 2026-06-26 via
	 * hostapd-dd showing M3 sent repeatedly while we kept RXing it
	 * without ever advancing the WPA state machine on AP side.
	 */
	if ((error = rtw88_usb_write2(sc, REG_RRSR, RRSR_INIT_2G)) != 0)
		return (error);
	if ((error = rtw88_usb_setbits1(sc, REG_WMAC_TRXPTCL_CTL_H,
	    (1U << 1))) != 0)
		return (error);
	if ((error = rtw88_usb_setbits1(sc, REG_SND_PTCL_CTRL,
	    BIT_DIS_CHK_VHTSIGB_CRC)) != 0)
		return (error);
	if ((error = rtw88_usb_write4(sc, REG_WMAC_OPTION_FUNCTION + 8,
	    WLAN_MAC_OPT_FUNC2)) != 0)
		return (error);
	return (rtw88_usb_write1(sc, REG_WMAC_OPTION_FUNCTION + 4,
	    WLAN_MAC_OPT_NORM_FUNC1));
}

static int
rtw88_drv_info_cfg(struct rtw88_usb_softc *sc)
{
	uint8_t val8;
	uint32_t val32;
	int error;

	RTW88_ASSERT_LOCKED(sc);

	if ((error = rtw88_usb_write1(sc, REG_RX_DRVINFO_SZ, 4)) != 0)
		return (error);
	/* 3081 fixup: TRXFF_BNDY+1 low nibble = 0xF (RX-desc len issue). */
	if ((error = rtw88_usb_read1(sc, REG_TRXFF_BNDY + 1, &val8)) != 0)
		return (error);
	val8 = (val8 & 0xF0) | 0xF;
	if ((error = rtw88_usb_write1(sc, REG_TRXFF_BNDY + 1, val8)) != 0)
		return (error);
	if ((error = rtw88_usb_read4(sc, REG_RCR, &val32)) != 0)
		return (error);
	if ((error = rtw88_usb_write4(sc, REG_RCR,
	    val32 | BIT_APP_PHYSTS)) != 0)
		return (error);
	if ((error = rtw88_usb_read4(sc, REG_WMAC_OPTION_FUNCTION + 4,
	    &val32)) != 0)
		return (error);
	val32 &= ~((1U << 8) | (1U << 9));
	return (rtw88_usb_write4(sc, REG_WMAC_OPTION_FUNCTION + 4, val32));
}

static int
rtw88_usb_init_burst(struct rtw88_usb_softc *sc)
{
	const uint8_t rxdma =
	    (uint8_t)(BIT_DMA_BURST_CNT | BIT_DMA_MODE |
	    (BIT_DMA_BURST_SIZE_512 << 4));
	int error;

	RTW88_ASSERT_LOCKED(sc);

	if ((error = rtw88_usb_write1(sc, REG_RXDMA_MODE, rxdma)) != 0)
		return (error);
	return (rtw88_usb_setbits1(sc, REG_TXDMA_OFFSET_CHK + 1,
	    BIT_DROP_DATA_EN >> 8));
}

static int
rtw88_mac_init(struct rtw88_usb_softc *sc)
{
	int error = 0;

	RTW88_LOCK(sc);
	if ((error = rtw88_txdma_queue_mapping(sc)) != 0) {
		device_printf(sc->sc_dev,
		    "txdma_queue_mapping failed: %d\n", error);
		goto out;
	}
	if ((error = rtw88_priority_queue_cfg(sc)) != 0) {
		device_printf(sc->sc_dev,
		    "priority_queue_cfg failed: %d\n", error);
		goto out;
	}
	if ((error = rtw88_init_h2c(sc)) != 0) {
		device_printf(sc->sc_dev, "init_h2c failed: %d\n", error);
		goto out;
	}
	if ((error = rtw88_8821c_mac_init(sc)) != 0) {
		device_printf(sc->sc_dev, "8821c_mac_init failed: %d\n",
		    error);
		goto out;
	}
	if ((error = rtw88_drv_info_cfg(sc)) != 0) {
		device_printf(sc->sc_dev, "drv_info_cfg failed: %d\n",
		    error);
		goto out;
	}
	if ((error = rtw88_usb_init_burst(sc)) != 0) {
		device_printf(sc->sc_dev, "usb_init_burst failed: %d\n",
		    error);
		goto out;
	}

	/*
	 * Match reference mac.c / sec.c: REG_CR full value is
	 *   MAC_TRX_ENABLE | BIT_MAC_SEC_EN (bit 9) | BIT_32K_CAL_TMR_EN (bit 10)
	 * = 0x06FF.  Without BIT_MAC_SEC_EN the chip's MAC security pipeline
	 * is disabled and the address-filter path that feeds unicast frames
	 * up to the host stays blocked -- AUTH_RESP / ASSOC_RESP from the
	 * AP are silently dropped even though BIT_APM is set in RCR.  reference
	 * sets these via rtw_sec_enable_sec_engine() during rtw_core_start.
	 */
	(void)rtw88_usb_write1(sc, REG_CR + 1, 0x06);
	/*
	 * STA port BCN_CTRL: BIT_EN_BCN_FUNCTION = 0x08, per reference
	 * rtw_ops_add_interface (IFTYPE_STATION case).  Without it the
	 * chip's beacon receive engine stays off and certain RX behaviors
	 * are degraded.
	 */
	(void)rtw88_usb_write1(sc, REG_BCN_CTRL, BIT_EN_BCN_FUNCTION);
	/*
	 * REG_WMAC_TRXPTCL_CTL initial value for 20 MHz BW (RFMOD bits
	 * 7-8 = 0).  reference reads-modify-writes this every set_channel
	 * via rtw_set_channel_mac_bw; the chip's reset default of 0 for
	 * the rest of the bits gates RX MAC unicast delivery.  Captured
	 * via usbmon as the persistent value reference maintains.
	 *
	 * 2026-06-24: tested removing this write; chip read 0x0E301000
	 * either way (reset default OR stale state persists). Not the
	 * unicast-data RX gate. Original write retained.
	 */
	(void)rtw88_usb_write4(sc, REG_WMAC_TRXPTCL_CTL,
	    WMAC_TRXPTCL_CTL_INIT);
	/*
	 * RTW_SEC_CONFIG: leave OFF (zero) under the SW CCMP path.
	 *
	 * Linux sets TX_DEC_EN | RX_DEC_EN | TX/RX UNI/BC USE_DK to enable
	 * the chip's CCMP engine + default-key CAM search.  With USE_DK ON
	 * the chip consults CAM by macid for EVERY Protected TX (fc[1]
	 * Protected=1).  We don't install CAM entries under SW CCMP
	 * (iv_key_set returns 1 unconditionally, sc_cam_task is dead-code)
	 * so the chip's TX lookup hits a stale/empty slot and the chip's
	 * TX scheduler reverts to a default that mangles fc[1] (stamps
	 * FromDS=1 on broadcasts, silently drops encrypted unicast).
	 *
	 * Per AP-side wlandebug 2026-06-27: chip stamps dir=0x2 on broadcast,
	 * AP discards "hostap_input:599 incorrect dir 0x2"; encrypted unicast
	 * leaves zero footprint at AP recv path.  Both regressed at the
	 * AES_CCM/SW-CCMP cutover.
	 *
	 * REG_CR BIT_MAC_SEC_EN remains set (the RX-side address-filter
	 * pipeline still needs it to deliver unicast to host); only the
	 * 0x0680 engine config bits are cleared.
	 */
	if (sc->sc_hw_ccmp) {
		/*
		 * HW CCMP path: turn the chip's security engine ON.  TX/RX_DEC_EN
		 * enables CCMP encrypt/decrypt; the four UNI/BC USE_DK bits tell
		 * the chip to consult the CAM (default-key search) for both
		 * pairwise and group frames.  PTK lands in CAM slot 4 at
		 * iv_key_set; GTK lands in slots 0..3 by key index.
		 */
		(void)rtw88_usb_write2(sc, RTW_SEC_CONFIG,
		    RTW_SEC_TX_DEC_EN | RTW_SEC_RX_DEC_EN |
		    RTW_SEC_TX_UNI_USE_DK | RTW_SEC_RX_UNI_USE_DK |
		    RTW_SEC_TX_BC_USE_DK | RTW_SEC_RX_BC_USE_DK);
	} else {
		(void)rtw88_usb_write2(sc, RTW_SEC_CONFIG, 0);
	}
	/*
	 * 0x06DC, 0x14C0, 0x167C, 0x042C are touched by rtw_bf_phy_init
	 * (some RMW, some field-mask) at the tail of rtw8821c_phy_set_param.
	 * Issuing wholesale writes here would clobber FW-loaded defaults
	 * the chip pre-populated.  See rtw88_bf_phy_init().
	 */
	/* AID=0 and BSSID=0 at init time (the reference does this in
	 * rtw_vif_port_config at add_interface).  We re-write BSSID with
	 * the AP MAC at SCAN->AUTH later. */
	{
		int i;
		(void)rtw88_usb_write4(sc, REG_AID, 0);
		for (i = 0; i < 6; i++)
			(void)rtw88_usb_write1(sc, REG_BSSID0 + i, 0);
	}
	/*
	 * GENERAL_INFO + PHYDM_INFO H2C pair the reference fires from
	 * rtw_core_start (main.c:1428-1429) right after HCI starts.
	 * Linux pcap shows these fire pre-AUTH twice (scan + immediate
	 * pre-AUTH) and seed firmware's PhyDM module with our RF/efuse
	 * params so the BB block can properly drive AGC + TX power
	 * tables.  Our prior init never sent these; broader H2C extract
	 * (qemu-trace-x86/extract_h2c_full.py) revealed the gap.
	 */
	(void)rtw88_h2c_general_info(sc);
	(void)rtw88_h2c_phydm_info(sc);
	/*
	 * Fire the BT-coex H2Cs the reference driver issues at init.  The
	 * RTL8821CU is a combo WiFi+BT silicon; without telling its on-chip
	 * BT engine "WiFi has the antenna, TDMA case = 0 (WiFi-only)", the
	 * shared 2.4 GHz front end keeps swapping ownership and unicast
	 * frames addressed to us never reach the RX bulk pipe.  Found via
	 * H2C-cmd <-> register-write correlation over the reference's
	 * usbmon trace at AUTH time (only cmds 0x60 + 0x69 are uniquely
	 * paired with the RX-path register writes that bring up unicast).
	 */
	(void)rtw88_coex_h2c_bt_wifi_control(&sc->sc_rtwdev, 0x0C, 0x01);
	(void)rtw88_coex_h2c_tdma_type(&sc->sc_rtwdev, 0);
	/*
	 * Trigger thermal/power-tracking measurement.  Equivalent of the
	 * first tick of Linux's rtw8821c_pwr_track:
	 *   rtw_write_rf(RF_PATH_A, RF_T_METER=0x42, GENMASK(17,16), 0x03);
	 * Linux fires this periodically from rtw_phy_dynamic_mechanism;
	 * the bpftrace SIPI diff (qemu-trace-x86/run4/...) shows it goes
	 * out as the only SIPI write Linux makes during AUTH that we don't.
	 * One-shot here is the first fire; if it unblocks AUTH, port the
	 * full periodic adjust loop next.
	 */
	(void)rtw88_write_rf_a_mask(sc, 0x42, 0x30000U, 0x03);
	device_printf(sc->sc_dev, "mac_init: complete\n");
out:
	RTW88_UNLOCK(sc);
	return (error);
}

/*
 * Read RF-path-A register via direct MMIO at base 0x2800.  reference
 * phy.c rtw_phy_read_rf() uses base_addr[rf_path] + (addr << 2);
 * the SIPI read path (rtw_phy_read_rf_sipi) is for chips that
 * don't expose rf_base_addr.  8821C does, so we use the simple
 * one (32-bit read with mask, no SIPI handshake).
 */
static int
rtw88_read_rf_a(struct rtw88_usb_softc *sc, uint8_t addr, uint32_t mask,
    uint32_t *out)
{
	uint16_t direct;
	uint32_t val;
	int error;

	RTW88_ASSERT_LOCKED(sc);

	direct = (uint16_t)(RTW88_RF_PATH_A_BASE + ((uint32_t)addr << 2));
	if ((error = rtw88_usb_read4(sc, direct, &val)) != 0)
		return (error);
	if (mask == RFREG_MASK)
		*out = val & RFREG_MASK;
	else
		*out = val & mask;
	return (0);
}

/* Masked RF-path-A write: read-modify-write through the SIPI bus. */
static int
rtw88_write_rf_a_mask(struct rtw88_usb_softc *sc, uint8_t addr,
    uint32_t mask, uint32_t data)
{
	uint32_t old, shift, merged;
	int error;

	RTW88_ASSERT_LOCKED(sc);

	if (mask == RFREG_MASK)
		return (rtw88_phy_write_rf_a(sc, addr, data));

	if ((error = rtw88_read_rf_a(sc, addr, RFREG_MASK, &old)) != 0)
		return (error);
	shift = __builtin_ctz(mask);
	merged = (old & ~mask) | ((data << shift) & mask);
	return (rtw88_phy_write_rf_a(sc, addr, merged));
}

/*
 * Set the BB chip's RFE band-switch to the 2.4 GHz WLAN-G (no
 * Bluetooth coexistence) or BTG (Bluetooth-shared) routing.
 * 8821CU's hal->rfe_btg derives from EFUSE rfe_option per the
 * reference switch in rtw8821c_read_efuse(); our chip with rfe=0x02
 * lands in BTG.
 */
static int
rtw88_switch_rf_set_2g(struct rtw88_usb_softc *sc, bool btg)
{
	uint32_t reg;
	int error;

	RTW88_ASSERT_LOCKED(sc);

	if ((error = rtw88_usb_setbits1(sc, REG_DMEM_CTRL_BB + 2,
	    BIT_WL_RST >> 16)) != 0)
		return (error);
	/* REG_SYS_CTRL is the 0x0000 alias of REG_SYS_FUNC_EN/REG_SYS_CTRL;
	 * BIT_FEN_EN is bit 26 -> bit 2 of byte at offset 0x03.
	 */
	if ((error = rtw88_usb_setbits1(sc, REG_SYS_CTRL_BB + 3,
	    BIT_FEN_EN >> 24)) != 0)
		return (error);

	if ((error = rtw88_usb_read4(sc, REG_RFECTL_A, &reg)) != 0)
		return (error);
	if (btg) {
		reg |= B_BTG_SWITCH;
		reg &= ~(B_CTRL_SWITCH | B_WL_SWITCH | B_WLG_SWITCH |
		    B_WLA_SWITCH);
	} else {
		reg |= B_WL_SWITCH | B_WLG_SWITCH;
		reg &= ~(B_BTG_SWITCH | B_CTRL_SWITCH | B_WLA_SWITCH);
	}

	/* Reload CCA + LNA payloads per band. */
	{
		uint32_t cca, lna;

		if ((error = rtw88_usb_read4(sc, REG_ENRXCCA, &cca)) != 0)
			return (error);
		cca = (cca & ~0x00FF0000) |
		    ((uint32_t)(btg ? BTG_CCA : WLG_CCA) << 16);
		if ((error = rtw88_usb_write4(sc, REG_ENRXCCA, cca)) != 0)
			return (error);
		if ((error = rtw88_usb_read4(sc, REG_ENTXCCK, &lna)) != 0)
			return (error);
		lna = (lna & ~0x0000FFFF) | (btg ? BTG_LNA : WLG_LNA);
		if ((error = rtw88_usb_write4(sc, REG_ENTXCCK, lna)) != 0)
			return (error);
	}

	return (rtw88_usb_write4(sc, REG_RFECTL_A, reg));
}

/*
 * Port of reference mac.c rtw_set_channel_mac(), BW20 branch.  Fires
 * the MAC-side per-channel housekeeping our prior set_channel_20m
 * skipped entirely: DATA_SC primary-channel hint, WMAC_TRXPTCL_CTL
 * RFMOD clear, AFE clock-tree select, microsecond-timer reload, and
 * CCK-enable per-band.  The bpftrace capture of Linux's set_channel
 * (qemu-trace-x86/run4/linux-bpftrace-auth.out) shows these 5 writes
 * land between set_channel_bb and the BB stamp that makes the TXAGC
 * range (0x1d00-0x1d34) accept stores; without them, our TX-power
 * table writes silently drop on the floor.
 */
static int
rtw88_set_channel_mac(struct rtw88_usb_softc *sc, uint8_t channel,
    uint8_t bw, uint8_t primary_ch_idx)
{
	uint32_t val32;
	uint8_t txsc20, txsc40, b;
	int error;

	RTW88_ASSERT_LOCKED(sc);

	/*
	 * REG_DATA_SC: primary 20 and 40 subchannel encoding (Linux mac.c
	 * rtw_set_channel_mac).  txsc20 = primary_ch_idx directly.  For
	 * BW80 the 40 MHz primary is derived from where the primary 20
	 * sits within the wider channel (UPPER/UPMOST -> SC_40_UPPER,
	 * else SC_40_LOWER).
	 */
	txsc20 = primary_ch_idx;
	txsc40 = 0;
	if (bw == RTW88_CHANNEL_WIDTH_80) {
		if (primary_ch_idx == RTW88_SC_20_UPPER ||
		    primary_ch_idx == RTW88_SC_20_UPMOST)
			txsc40 = 1;	/* RTW_SC_40_UPPER */
		else
			txsc40 = 2;	/* RTW_SC_40_LOWER */
	}
	if ((error = rtw88_usb_write1(sc, REG_DATA_SC,
	    (uint8_t)((txsc20 & 0xF) | ((txsc40 & 0xF) << 4)))) != 0)
		return (error);

	/* REG_WMAC_TRXPTCL_CTL bits 7:8 = RFMOD: 00 = 20M, 01 = 40M, 10 = 80M. */
	if ((error = rtw88_usb_read4(sc, REG_WMAC_TRXPTCL_CTL, &val32)) != 0)
		return (error);
	val32 &= ~BIT_RFMOD;
	if (bw == RTW88_CHANNEL_WIDTH_40)
		val32 |= (1U << 7);
	else if (bw == RTW88_CHANNEL_WIDTH_80)
		val32 |= (1U << 8);
	if ((error = rtw88_usb_write4(sc, REG_WMAC_TRXPTCL_CTL, val32)) != 0)
		return (error);

	if ((error = rtw88_usb_read4(sc, REG_AFE_CTRL1, &val32)) != 0)
		return (error);
	val32 &= ~BIT_MAC_CLK_SEL;
	if ((error = rtw88_usb_write4(sc, REG_AFE_CTRL1, val32)) != 0)
		return (error);

	if ((error = rtw88_usb_write1(sc, REG_USTIME_TSF, MAC_CLK_SPEED)) != 0)
		return (error);
	if ((error = rtw88_usb_write1(sc, REG_USTIME_EDCA, MAC_CLK_SPEED))
	    != 0)
		return (error);

	if ((error = rtw88_usb_read1(sc, REG_CCK_CHECK, &b)) != 0)
		return (error);
	b &= ~BIT_CHECK_CCK_EN;
	if (channel > 14)
		b |= BIT_CHECK_CCK_EN;
	return (rtw88_usb_write1(sc, REG_CCK_CHECK, b));
}

/*
 * Per-channel RF retune for 2.4 GHz, 20 MHz BW.
 * Mirrors reference rtw8821c_set_channel_rf() with the 5 GHz / 40 / 80
 * MHz branches stripped (no callers in our scan-only world).
 */
static int
rtw88_set_channel_rf_2g(struct rtw88_usb_softc *sc, uint8_t channel, uint8_t bw)
{
	uint32_t rf_reg18;
	int error;

	RTW88_ASSERT_LOCKED(sc);

	if ((error = rtw88_read_rf_a(sc, RF_REG_18, RFREG_MASK,
	    &rf_reg18)) != 0)
		return (error);

	rf_reg18 &= ~(RF18_BAND_MASK | RF18_CHANNEL_MASK | RF18_RFSI_MASK |
	    RF18_BW_MASK);
	rf_reg18 |= RF18_BAND_2G;
	rf_reg18 |= channel & RF18_CHANNEL_MASK;
	switch (bw) {
	case RTW88_CHANNEL_WIDTH_40: rf_reg18 |= RF18_BW_40M; break;
	case RTW88_CHANNEL_WIDTH_80: rf_reg18 |= RF18_BW_80M; break;
	default:                     rf_reg18 |= RF18_BW_20M; break;
	}

	if ((error = rtw88_switch_rf_set_2g(sc, sc->sc_rfe_btg)) != 0)
		return (error);
	if ((error = rtw88_write_rf_a_mask(sc, RF_LUTDBG, (1U << 6), 1)) != 0)
		return (error);
	if ((error = rtw88_write_rf_a_mask(sc, 0x64, 0xF, 0xF)) != 0)
		return (error);

	if ((error = rtw88_phy_write_rf_a(sc, RF_REG_18, rf_reg18)) != 0)
		return (error);

	/* Toggle RF_XTALX2 bit 19 low then high (channel-recal pulse). */
	if ((error = rtw88_write_rf_a_mask(sc, RF_XTALX2, (1U << 19), 0)) != 0)
		return (error);
	return (rtw88_write_rf_a_mask(sc, RF_XTALX2, (1U << 19), 1));
}

/*
 * RX digital filter, per-BW.  Mirrors reference rtw8821c_set_channel_rxdfir():
 *   ACBB0[29:28] = 0x2 (always)
 *   ACBBRXFIR[29:28] = 0x1 on BW80, else 0x2
 *   TXDFIR[31] = 1 on BW20, else 0
 *   CHFIR[31]  = 1 on BW80, else 0
 */
static int
rtw88_set_channel_rxdfir(struct rtw88_usb_softc *sc, uint8_t bw)
{
	uint32_t val;
	uint32_t acbbrxfir_v;
	int error;

	RTW88_ASSERT_LOCKED(sc);

	if ((error = rtw88_usb_read4(sc, REG_ACBB0, &val)) != 0)
		return (error);
	val = (val & ~(3U << 28)) | (2U << 28);
	if ((error = rtw88_usb_write4(sc, REG_ACBB0, val)) != 0)
		return (error);

	acbbrxfir_v = (bw == RTW88_CHANNEL_WIDTH_80) ? 1U : 2U;
	if ((error = rtw88_usb_read4(sc, REG_ACBBRXFIR, &val)) != 0)
		return (error);
	val = (val & ~(3U << 28)) | (acbbrxfir_v << 28);
	if ((error = rtw88_usb_write4(sc, REG_ACBBRXFIR, val)) != 0)
		return (error);

	if ((error = rtw88_usb_read4(sc, REG_TXDFIR, &val)) != 0)
		return (error);
	if (bw == RTW88_CHANNEL_WIDTH_20)
		val |= (1U << 31);
	else
		val &= ~(1U << 31);
	if ((error = rtw88_usb_write4(sc, REG_TXDFIR, val)) != 0)
		return (error);

	if ((error = rtw88_usb_read4(sc, REG_CHFIR, &val)) != 0)
		return (error);
	if (bw == RTW88_CHANNEL_WIDTH_80)
		val |= (1U << 31);
	else
		val &= ~(1U << 31);
	return (rtw88_usb_write4(sc, REG_CHFIR, val));
}

/*
 * BB per-channel config for 2.4 GHz, 20 MHz, default (non-SRRC)
 * region.  Mirrors the channel <= 14 + set_bw 20 path of reference
 * rtw8821c_set_channel_bb().
 */
/*
 * BB bandwidth-select tail.  Mirrors the set_bw switch at the bottom
 * of reference rtw8821c_set_channel_bb(): REG_RXSB primary-subchan
 * bit for HT40, REG_ADCCLK with primary_ch_idx + BW width, ADC160 enable.
 * Called by both rtw88_set_channel_bb_2g and _5g.
 */
static int
rtw88_set_channel_bb_bw(struct rtw88_usb_softc *sc, uint8_t bw,
    uint8_t primary_ch_idx)
{
	uint32_t val;
	int error;

	RTW88_ASSERT_LOCKED(sc);

	switch (bw) {
	case RTW88_CHANNEL_WIDTH_40:
		if ((error = rtw88_usb_read4(sc, REG_RXSB, &val)) != 0)
			return (error);
		if (primary_ch_idx == 1)
			val |= (1U << 4);
		else
			val &= ~(1U << 4);
		if ((error = rtw88_usb_write4(sc, REG_RXSB, val)) != 0)
			return (error);

		if ((error = rtw88_usb_read4(sc, REG_ADCCLK, &val)) != 0)
			return (error);
		val = (val & 0xFF3FF300) |
		    (0x20020000U | (((uint32_t)primary_ch_idx & 0xF) << 2) |
		    RTW88_CHANNEL_WIDTH_40);
		if ((error = rtw88_usb_write4(sc, REG_ADCCLK, val)) != 0)
			return (error);
		break;
	case RTW88_CHANNEL_WIDTH_80:
		if ((error = rtw88_usb_read4(sc, REG_ADCCLK, &val)) != 0)
			return (error);
		val = (val & 0xFCFFCF00) |
		    (0x40040000U | (((uint32_t)primary_ch_idx & 0xF) << 2) |
		    RTW88_CHANNEL_WIDTH_80);
		if ((error = rtw88_usb_write4(sc, REG_ADCCLK, val)) != 0)
			return (error);
		break;
	case RTW88_CHANNEL_WIDTH_20:
	default:
		if ((error = rtw88_usb_read4(sc, REG_ADCCLK, &val)) != 0)
			return (error);
		val = (val & 0xFFCFFC00) | 0x10010000U;
		if ((error = rtw88_usb_write4(sc, REG_ADCCLK, val)) != 0)
			return (error);
		break;
	}
	if ((error = rtw88_usb_read4(sc, REG_ADC160, &val)) != 0)
		return (error);
	val |= (1U << 30);
	return (rtw88_usb_write4(sc, REG_ADC160, val));
}

static int
rtw88_set_channel_bb_2g(struct rtw88_usb_softc *sc, uint8_t channel,
    uint8_t bw, uint8_t primary_ch_idx)
{
	uint32_t val;
	int error;

	RTW88_ASSERT_LOCKED(sc);

	if ((error = rtw88_usb_read4(sc, REG_RXPSEL, &val)) != 0)
		return (error);
	val |= (1U << 28);
	if ((error = rtw88_usb_write4(sc, REG_RXPSEL, val)) != 0)
		return (error);
	if ((error = rtw88_usb_read4(sc, REG_CCK_CHECK, &val)) != 0)
		return (error);
	val &= ~(1U << 7);
	if ((error = rtw88_usb_write4(sc, REG_CCK_CHECK, val)) != 0)
		return (error);
	if ((error = rtw88_usb_read4(sc, REG_ENTXCCK, &val)) != 0)
		return (error);
	val &= ~(1U << 18);
	if ((error = rtw88_usb_write4(sc, REG_ENTXCCK, val)) != 0)
		return (error);
	if ((error = rtw88_usb_read4(sc, REG_RXCCAMSK, &val)) != 0)
		return (error);
	val = (val & ~0x0000FC00) | ((15U & 0x3F) << 10);
	if ((error = rtw88_usb_write4(sc, REG_RXCCAMSK, val)) != 0)
		return (error);

	if ((error = rtw88_usb_read4(sc, REG_TXSCALE_A, &val)) != 0)
		return (error);
	val &= ~0xF00;
	if ((error = rtw88_usb_write4(sc, REG_TXSCALE_A, val)) != 0)
		return (error);

	if ((error = rtw88_usb_read4(sc, REG_CLKTRK, &val)) != 0)
		return (error);
	val = (val & ~0x1FFE0000) | (0x96AU << 17);
	if ((error = rtw88_usb_write4(sc, REG_CLKTRK, val)) != 0)
		return (error);

	if (channel != 14) {
		if ((error = rtw88_usb_write4(sc, REG_TXSF2,
		    sc->sc_ch_param[0])) != 0)
			return (error);
		if ((error = rtw88_usb_read4(sc, REG_TXSF6, &val)) != 0)
			return (error);
		val = (val & ~0xFFFFU) | (sc->sc_ch_param[1] & 0xFFFFU);
		if ((error = rtw88_usb_write4(sc, REG_TXSF6, val)) != 0)
			return (error);
		if ((error = rtw88_usb_write4(sc, REG_TXFILTER,
		    sc->sc_ch_param[2])) != 0)
			return (error);
	}

	return (rtw88_set_channel_bb_bw(sc, bw, primary_ch_idx));
}

/*
 * 5 GHz (WLA) variant of the RFE band-switch.  Mirrors reference
 * rtw8821c_switch_rf_set(SWITCH_TO_WLA): set B_WL_SWITCH +
 * B_WLA_SWITCH, clear the BTG / CTRL / WLG bits.  Unlike the 2.4 GHz
 * paths, no CCA/LNA payloads need to be re-loaded.
 */
static int
rtw88_switch_rf_set_5g(struct rtw88_usb_softc *sc)
{
	uint32_t reg;
	int error;

	RTW88_ASSERT_LOCKED(sc);

	if ((error = rtw88_usb_setbits1(sc, REG_DMEM_CTRL_BB + 2,
	    BIT_WL_RST >> 16)) != 0)
		return (error);
	if ((error = rtw88_usb_setbits1(sc, REG_SYS_CTRL_BB + 3,
	    BIT_FEN_EN >> 24)) != 0)
		return (error);

	if ((error = rtw88_usb_read4(sc, REG_RFECTL_A, &reg)) != 0)
		return (error);
	reg |= B_WL_SWITCH | B_WLA_SWITCH;
	reg &= ~(B_BTG_SWITCH | B_CTRL_SWITCH | B_WLG_SWITCH);
	return (rtw88_usb_write4(sc, REG_RFECTL_A, reg));
}

/*
 * Per-channel RF retune for 5 GHz, 20 MHz BW.  Mirrors the channel>14
 * branch of reference rtw8821c_set_channel_rf(): RF18 takes BAND_5G plus
 * RFSI_GE (chan 100-140) or RFSI_GT (chan > 140), the WLA band-switch
 * fires, RF_LUTDBG bit 6 is cleared, and the XTALX2 channel-recal
 * pulse follows just like 2.4 GHz.
 */
static int
rtw88_set_channel_rf_5g(struct rtw88_usb_softc *sc, uint8_t channel, uint8_t bw)
{
	uint32_t rf_reg18;
	int error;

	RTW88_ASSERT_LOCKED(sc);

	if ((error = rtw88_read_rf_a(sc, RF_REG_18, RFREG_MASK,
	    &rf_reg18)) != 0)
		return (error);

	rf_reg18 &= ~(RF18_BAND_MASK | RF18_CHANNEL_MASK | RF18_RFSI_MASK |
	    RF18_BW_MASK);
	rf_reg18 |= RF18_BAND_5G;
	rf_reg18 |= channel & RF18_CHANNEL_MASK;
	switch (bw) {
	case RTW88_CHANNEL_WIDTH_40: rf_reg18 |= RF18_BW_40M; break;
	case RTW88_CHANNEL_WIDTH_80: rf_reg18 |= RF18_BW_80M; break;
	default:                     rf_reg18 |= RF18_BW_20M; break;
	}
	if (channel >= 100 && channel <= 140)
		rf_reg18 |= RF18_RFSI_GE;
	else if (channel > 140)
		rf_reg18 |= RF18_RFSI_GT;

	if ((error = rtw88_switch_rf_set_5g(sc)) != 0)
		return (error);
	if ((error = rtw88_write_rf_a_mask(sc, RF_LUTDBG, (1U << 6), 0)) != 0)
		return (error);

	if ((error = rtw88_phy_write_rf_a(sc, RF_REG_18, rf_reg18)) != 0)
		return (error);

	if ((error = rtw88_write_rf_a_mask(sc, RF_XTALX2, (1U << 19), 0)) != 0)
		return (error);
	return (rtw88_write_rf_a_mask(sc, RF_XTALX2, (1U << 19), 1));
}

/*
 * BB per-channel config for 5 GHz, 20 MHz, default region.  Mirrors
 * the channel>35 branch of reference rtw8821c_set_channel_bb() followed
 * by the set_bw 20 default tail (ADCCLK + ADC160).
 */
static int
rtw88_set_channel_bb_5g(struct rtw88_usb_softc *sc, uint8_t channel,
    uint8_t bw, uint8_t primary_ch_idx)
{
	uint32_t val;
	uint32_t txscale_v, clktrk_v;
	int error;

	RTW88_ASSERT_LOCKED(sc);

	if ((error = rtw88_usb_read4(sc, REG_ENTXCCK, &val)) != 0)
		return (error);
	val |= (1U << 18);
	if ((error = rtw88_usb_write4(sc, REG_ENTXCCK, val)) != 0)
		return (error);
	if ((error = rtw88_usb_read4(sc, REG_CCK_CHECK, &val)) != 0)
		return (error);
	val |= (1U << 7);
	if ((error = rtw88_usb_write4(sc, REG_CCK_CHECK, val)) != 0)
		return (error);
	if ((error = rtw88_usb_read4(sc, REG_RXPSEL, &val)) != 0)
		return (error);
	val &= ~(1U << 28);
	if ((error = rtw88_usb_write4(sc, REG_RXPSEL, val)) != 0)
		return (error);
	if ((error = rtw88_usb_read4(sc, REG_RXCCAMSK, &val)) != 0)
		return (error);
	val = (val & ~0x0000FC00) | ((15U & 0x3F) << 10);
	if ((error = rtw88_usb_write4(sc, REG_RXCCAMSK, val)) != 0)
		return (error);

	if (channel >= 36 && channel <= 64)
		txscale_v = 0x1U;
	else if (channel >= 100 && channel <= 144)
		txscale_v = 0x2U;
	else
		txscale_v = 0x3U;	/* channel >= 149 */
	if ((error = rtw88_usb_read4(sc, REG_TXSCALE_A, &val)) != 0)
		return (error);
	val = (val & ~0xF00) | (txscale_v << 8);
	if ((error = rtw88_usb_write4(sc, REG_TXSCALE_A, val)) != 0)
		return (error);

	if (channel >= 36 && channel <= 48)
		clktrk_v = 0x494U;
	else if (channel >= 52 && channel <= 64)
		clktrk_v = 0x453U;
	else if (channel >= 100 && channel <= 116)
		clktrk_v = 0x452U;
	else
		clktrk_v = 0x412U;	/* 118 .. 177 */
	if ((error = rtw88_usb_read4(sc, REG_CLKTRK, &val)) != 0)
		return (error);
	val = (val & ~0x1FFE0000) | (clktrk_v << 17);
	if ((error = rtw88_usb_write4(sc, REG_CLKTRK, val)) != 0)
		return (error);

	return (rtw88_set_channel_bb_bw(sc, bw, primary_ch_idx));
}

/*
 * Send one 8-byte H2C command via the chip's HMEBOX mailbox.  Boxes
 * are round-robined modulo 4; chip raises the box's BIT_INT_BOXn in
 * REG_HMETFR while it's busy so we poll until that clears before
 * writing the next.  msg_ext goes first (to HMEBOXn_EX), then the
 * 4-byte msg (HMEBOXn) -- writing msg is what arms the chip to act.
 *
 * This is pure EP0 and stays alive even when the bulk-OUT H2C-packet
 * path is gated.  Useful for waking firmware to start servicing the
 * data-TX path.
 */
static int
rtw88_h2c_mailbox(struct rtw88_usb_softc *sc, uint32_t msg, uint32_t msg_ext)
{
	static const uint16_t box_addr[4] = {
		REG_HMEBOX0, REG_HMEBOX1, REG_HMEBOX2, REG_HMEBOX3,
	};
	static const uint16_t box_ex_addr[4] = {
		REG_HMEBOX0_EX, REG_HMEBOX1_EX, REG_HMEBOX2_EX, REG_HMEBOX3_EX,
	};
	uint8_t box, tfr;
	int error, try;

	RTW88_ASSERT_LOCKED(sc);

	box = sc->sc_h2c_box_next & 0x3;
	for (try = 0; try < 30; try++) {
		if ((error = rtw88_usb_read1(sc, REG_HMETFR, &tfr)) != 0)
			return (error);
		if ((tfr & (1U << box)) == 0)
			break;
		DELAY(100);
	}
	if (try == 30)
		return (EBUSY);

	if ((error = rtw88_usb_write4(sc, box_ex_addr[box], msg_ext)) != 0)
		return (error);
	if ((error = rtw88_usb_write4(sc, box_addr[box], msg)) != 0)
		return (error);
	sc->sc_h2c_box_next = (sc->sc_h2c_box_next + 1) & 0x3;
	return (0);
}

/*
 * Tell the firmware which channel we're tuned to via H2C_CMD_WL_CH_INFO.
 * reference uses this after every set_channel.  Some firmware revisions
 * gate the TX servicing thread on this; sending it should give us a
 * good probe of whether the FW is alive on the EP0 H2C path.
 *
 *   msg byte 0: command id (0x66)
 *   msg byte 1: link  (0 = idle, 1 = linked)
 *   msg byte 2: channel  (1..14 on 2.4 GHz)
 *   msg byte 3: bandwidth (0 = 20 MHz)
 *   msg_ext   : 0
 */
static int
rtw88_h2c_wl_ch_info(struct rtw88_usb_softc *sc, uint8_t link, uint8_t channel,
    uint8_t bw)
{
	uint32_t msg = (uint32_t)H2C_CMD_WL_CH_INFO |
	    ((uint32_t)link << 8) |
	    ((uint32_t)channel << 16) |
	    ((uint32_t)bw << 24);

	return (rtw88_h2c_mailbox(sc, msg, 0));
}

/*
 * H2C_CMD_SET_PWR_MODE (cmd 0x20) -- force chip into ACTIVE power state.
 *
 * Linux usbmon trace (2026-06-25, rtw88_8821cu 6.8.0-124 against same AP)
 * shows Linux fires 10 of these during a single assoc cycle, alternating
 * between LEAVE_LPS (ACTIVE) and ENTER_LPS (mode=1, smart_ps=UAPSD).
 *
 * Our driver doesn't manage LPS at all. The chip's power state at boot is
 * something the firmware decides; if it defaults to a sleep mode, the chip
 * may miss SIFS-window HW ACK deadlines for unicast data, causing AP
 * retransmits and eventual DEAUTH reason=4 INACTIVITY under load.
 *
 * Fire ONE LEAVE_LPS at RUN time to lock the chip into ACTIVE.
 *
 * Field layout per reference fw.h SET_PWR_MODE_SET_*:
 *   word[0] bits  7:0  = cmd_id (0x20)
 *   word[0] bits 14:8  = mode (0 = ACTIVE, 1 = LPS)
 *   word[0] bits 19:16 = rlbm (0 = MIN)
 *   word[0] bits 23:20 = smart_ps (0 = LEGACY)
 *   word[0] bits 31:24 = awake_interval (1 beacon)
 *   word[1] bits  7:5  = port_id (0 = STA port)
 *   word[1] bits 15:8  = pwr_state (0x0c = RTW_ALL_ON)
 *
 * Encoded LEAVE_LPS = (cmd=0x20, mode=0, rlbm=0, smart_ps=0, awake=1,
 *                      port=0, state=0x0c)
 *   word[0] = 0x01000020
 *   word[1] = 0x00000c00
 */
static int
rtw88_h2c_pwr_mode_active(struct rtw88_usb_softc *sc)
{
	uint32_t msg = (uint32_t)H2C_CMD_SET_PWR_MODE |
	    (0u << 8)        |	/* mode = 0 ACTIVE */
	    (0u << 16)       |	/* rlbm = 0 */
	    (0u << 20)       |	/* smart_ps = 0 LEGACY */
	    (1u << 24);		/* awake_interval = 1 */
	uint32_t ex = (0u << 5) | (0x0cu << 8);
				/* port_id = 0 STA, pwr_state = 0x0c ALL_ON */
	return (rtw88_h2c_mailbox(sc, msg, ex));
}

/*
 * H2C_CMD_SET_PWR_MODE LPS-enter form.  Mirrors Linux
 * rtw_enter_lps_core: mode=LPS(1), rlbm=1, smart_ps=2 (UAPSD-style),
 * awake_interval = 1 beacon, pwr_state = RF_OFF.  Caller fires this
 * after idle TX/RX for >hw.rtw88_usb.lps_idle_ms to let the chip
 * sleep between beacons.  Pair with rtw88_h2c_pwr_mode_active to
 * wake before next TX.
 */
static int
rtw88_h2c_pwr_mode_lps(struct rtw88_usb_softc *sc, uint8_t awake_interval)
{
	uint32_t msg = (uint32_t)H2C_CMD_SET_PWR_MODE |
	    (1u << 8)        |	/* mode = 1 LPS */
	    (1u << 16)       |	/* rlbm = 1 (use awake_interval) */
	    (2u << 20)       |	/* smart_ps = 2 UAPSD-style */
	    ((uint32_t)awake_interval << 24);
	uint32_t ex = (0u << 5) | (0x04u << 8);
				/* port_id = 0 STA, pwr_state = 0x04 RF_OFF */
	return (rtw88_h2c_mailbox(sc, msg, ex));
}

/*
 * BT_WIFI_CONTROL (0x69) + COEX_TDMA_TYPE (0x60) H2C helpers extracted
 * to sys/dev/rtw88/rtw88_coex.c.  Use rtw88_coex_h2c_bt_wifi_control /
 * rtw88_coex_h2c_tdma_type against `&sc->sc_rtwdev` instead.
 */

/*
 * MEDIA_STATUS_RPT H2C (cmd 0x01) -- tells firmware "this MACID is
 * connected/disconnected" so the chip's TX scheduler routes ACK,
 * RX-status reports, and Rx-MAC unicast to the right STA slot.
 *
 * Linux's reference fires this from `rtw_sta_add` (mac80211 sta_state
 * transition) right after AUTH-RESP succeeds; bpftrace captured the
 * matching wire bytes "01 01 00 00 00 00 00 00" at auth+0.030s.
 *
 * Bit layout per fw.h MEDIA_STATUS_RPT_SET_OP_MODE/MACID:
 *   word[0] bits  7:0  = cmd_id (0x01)
 *   word[0] bit   8    = op_mode (connected = 1)
 *   word[0] bits 23:16 = mac_id
 *
 * Ours is a STATION-only port so mac_id is always 0 (vif default port).
 */
static int
rtw88_h2c_media_status_report(struct rtw88_usb_softc *sc, uint8_t mac_id,
    bool connect)
{
	uint32_t msg;

	msg = (uint32_t)H2C_CMD_MEDIA_STATUS_RPT |
	    ((uint32_t)(connect ? 1 : 0) << 8) |
	    ((uint32_t)mac_id << 16);
	return (rtw88_h2c_mailbox(sc, msg, 0));
}

/*
 * RA_INFO H2C (cmd 0x40) -- seeds the chip's rate-adaptation engine
 * with the per-STA rate mask + capabilities so the chip stops
 * transmitting data frames at the lowest fallback rate.
 *
 * Linux's `rtw_fw_send_ra_info` (fw.c:733) builds it from per-STA
 * info computed in `rtw_update_sta_info`.  For our STA-mode port we
 * use sensible 2.4 GHz / 11g/n / 1T1R / 20 MHz defaults; the AP's
 * actual capabilities can be plumbed later via net80211's
 * ic_newassoc once we have ASSOC traffic flowing.
 *
 * Bit layout per fw.h SET_RA_INFO_* macros (HMEBOX 8 bytes):
 *   word[0] bits  7:0  = cmd_id (0x40)
 *   word[0] bits 15:8  = mac_id
 *   word[0] bits 20:16 = rate_id (chip-specific table index)
 *   word[0] bits 22:21 = init_ra_lvl
 *   word[0] bit  23    = sgi_en
 *   word[0] bits 25:24 = bw_mode (0 = 20 MHz)
 *   word[0] bit  26    = ldpc
 *   word[0] bit  27    = no_update
 *   word[0] bits 29:28 = vht_en
 *   word[0] bit  30    = dis_pt (disable path tracker)
 *   word[1]            = ra_mask[31:0]
 */
static int
rtw88_h2c_ra_info(struct rtw88_usb_softc *sc, uint8_t mac_id,
    uint8_t rate_id, uint8_t bw_mode, uint8_t init_ra_lvl,
    bool sgi_en, bool ldpc_en, uint8_t vht_en, bool no_update,
    uint32_t ra_mask)
{
	uint32_t msg, msg_ext;

	msg = (uint32_t)H2C_CMD_RA_INFO |
	    ((uint32_t)mac_id << 8) |
	    ((uint32_t)(rate_id & 0x1F) << 16) |
	    ((uint32_t)(init_ra_lvl & 0x3) << 21) |
	    ((uint32_t)(sgi_en ? 1 : 0) << 23) |
	    ((uint32_t)(bw_mode & 0x3) << 24) |
	    ((uint32_t)(ldpc_en ? 1 : 0) << 26) |
	    ((uint32_t)(no_update ? 1 : 0) << 27) |
	    ((uint32_t)(vht_en & 0x3) << 28) |
	    /* DIS_PT = 1: disable path tracker on 1T1R */
	    (1U << 30);
	msg_ext = ra_mask;
	return (rtw88_h2c_mailbox(sc, msg, msg_ext));
}

/*
 * KEEP_ALIVE H2C (cmd 0x03) -- tells the firmware to emit NULL-data
 * frames at a fixed beacon-interval cadence so the AP doesn't time us
 * out for inactivity.  Linux's rtw_fw_set_keep_alive_cmd (fw.c:870)
 * uses pkt_type=NULL_PKT (0), adopt=1, enable=1, period=5 beacons.
 *
 * Without this the chip never originates anything between net80211
 * data bursts; APs running their default 10 s inactivity timer fire
 * DEAUTH reason 4 the moment a brief idle window opens.  Confirmed
 * from a Linux/QEMU oracle trace 2026-06-23.
 *
 * Bit layout per fw.h SET_KEEP_ALIVE_*:
 *   word[0] bits  7:0  = cmd_id (0x03)
 *   word[0] bit   8    = enable
 *   word[0] bit   9    = adopt
 *   word[0] bit  10    = pkt_type (0=NULL, 1=ARP)
 *   word[0] bits 23:16 = check_period (in beacon intervals)
 */
#define H2C_CMD_KEEP_ALIVE	0x03
#define H2C_CMD_DEFAULT_PORT	0x2c
/* Kept for future LPS/WoWLAN work; not called in the normal assoc path. */
static int __unused
rtw88_h2c_keep_alive(struct rtw88_usb_softc *sc, bool enable)
{
	uint32_t msg, msg_ext;

	/*
	 * Mirror Linux on-wire bytes captured 2026-06-23 from a
	 * working assoc: msg=0x00000103 msg_ext=0x02000000, i.e.
	 *   byte[0] = 0x03 KEEP_ALIVE cmd
	 *   byte[1] = 0x01 ENABLE
	 *   byte[7] = 0x02 (chip-specific; probably rsvd_page loc for
	 *                   NULL pkt -- "RSVD_NULL loc: 3" in Linux trace
	 *                   suggests +1 offset / different encoding; keep
	 *                   the trace value verbatim until RSVD_PAGE is
	 *                   ported)
	 */
	msg = (uint32_t)H2C_CMD_KEEP_ALIVE |
	    ((uint32_t)(enable ? 1 : 0) << 8);
	msg_ext = enable ? 0x02000000U : 0;
	return (rtw88_h2c_mailbox(sc, msg, msg_ext));
}

/*
 * DEFAULT_PORT H2C (cmd 0x2c) -- assigns one (portid, macid) pair as
 * the chip's default vif.  Linux's rtw_fw_default_port (fw.c:607)
 * sends it whenever rtwvif->net_type == RTW_NET_MGD_LINKED, i.e. on
 * successful ASSOC.  Without it the chip's TX scheduler doesn't know
 * which vif's beacon timer / mac_id pool to use for keep-alive and
 * pause/resume scheduling, and silently drops data traffic.
 *
 * Bit layout per fw.h RTW_H2C_DEFAULT_PORT_W0_*:
 *   word[0] bits  7:0  = cmd_id (0x2c)
 *   word[0] bits 15:8  = portid (vif index; 0 for our single STA vif)
 *   word[0] bits 23:16 = mac_id (0 for the AP we associated to)
 */
static int
rtw88_h2c_default_port(struct rtw88_usb_softc *sc, uint8_t portid,
    uint8_t mac_id)
{
	uint32_t msg;

	msg = (uint32_t)H2C_CMD_DEFAULT_PORT |
	    ((uint32_t)portid << 8) |
	    ((uint32_t)mac_id << 16);
	return (rtw88_h2c_mailbox(sc, msg, 0));
}

/*
 * RSVD_PAGE H2C (cmd 0x00) -- tells the firmware where the 4 frame
 * templates live in the chip's TXFF reserved area.  We upload only
 * the NULL template right now (loc 0, the start of RSVD_DRV); the
 * other three slots stay zero but the H2C still has to be sent so
 * the firmware's keep-alive timer picks up the NULL pointer.
 *
 * Bit layout per Linux fw.h SET_RSVD_PAGE_LOC_PROBE_RSP / _PSPOLL /
 * _NULL_DATA / _QOS_NULL_DATA:
 *   word[0] bits  7:0  = cmd_id (0x00)
 *   word[0] bits 15:8  = loc_probe_resp
 *   word[0] bits 23:16 = loc_ps_poll
 *   word[0] bits 31:24 = loc_null
 *   word[1] bits  7:0  = loc_qos_null
 */
#define	H2C_CMD_RSVD_PAGE	0x00
static int
rtw88_h2c_rsvd_page(struct rtw88_usb_softc *sc)
{
	uint32_t msg, msg_ext;

	/*
	 * loc indices match the buffer layout in rtw88_upload_rsvd_pages.
	 * Linux/QEMU oracle (2026-06-23) used probe_resp=0, ps_poll=1,
	 * qos_null=2, null=3.  We match.
	 */
	msg = (uint32_t)H2C_CMD_RSVD_PAGE |
	    (0U << 8)  |	/* probe_resp loc 0 */
	    (1U << 16) |	/* ps_poll loc 1 */
	    (3U << 24);		/* null loc 3 (referenced by KEEP_ALIVE) */
	msg_ext = 2U;		/* qos_null loc 2 (byte 4 of H2C) */
	return (rtw88_h2c_mailbox(sc, msg, msg_ext));
}

/*
 * Build all four STA-mode reserved-page frame templates (PROBE_RESP
 * stub, PS_POLL, QOS_NULL, NULL) into one buffer laid out at 128-byte
 * page boundaries, then upload via rtw88_send_firmware_pkt so the
 * firmware has the templates it needs for keep-alive, power-save
 * wake-up, and scan-offload.
 *
 * Layout (matches Linux rtw_build_rsvd_page; chip page_size = 128 B,
 * TX descriptor on the *first* page is supplied by send_firmware_pkt
 * itself so this buffer's bytes 0..79 land at offset 48 of page 0):
 *
 *   data[0..79]    = page 0 body slot   (PROBE_RESP stub, mostly zero)
 *   data[80..207]  = page 1             (PS_POLL,  16 bytes + zeros)
 *   data[208..335] = page 2             (QOS_NULL, 26 bytes + zeros)
 *   data[336..463] = page 3             (NULL,     24 bytes + zeros)
 *
 * total data = 464 B; wire = TX_DESC (48) + 464 = 512 B = 4 pages.
 *
 * The H2C RSVD_PAGE then tells the firmware that NULL = loc 3, etc.,
 * matching the Linux/QEMU oracle trace 2026-06-23:
 *   RSVD_PROBE_RESP loc: 0
 *   RSVD_PS_POLL    loc: 1
 *   RSVD_QOS_NULL   loc: 2
 *   RSVD_NULL       loc: 3
 *
 * PROBE_RESP is only used for active-scan offload which we don't
 * trigger; we still ship the page so the firmware's rsvd_page table
 * has a non-NULL loc 0 entry (some firmware paths validate every
 * advertised loc).
 */
#define	RTW88_RSVD_TOTAL_BYTES	464
#define	RTW88_RSVD_PAGE0_BYTES	80	/* page 0 minus 48-byte TX_DESC */
#define	RTW88_RSVD_PAGE_BYTES	128
static int
rtw88_upload_rsvd_pages(struct rtw88_usb_softc *sc, const uint8_t *ap_mac)
{
	uint8_t buf[RTW88_RSVD_TOTAL_BYTES];
	uint8_t *p;

	RTW88_ASSERT_LOCKED(sc);

	memset(buf, 0, sizeof(buf));

	/* Page 1: PS_POLL (16 bytes).
	 * fc[0]=0xA4  type=CTL(1), subtype=PS_POLL(10)
	 * fc[1]=0
	 * AID|0xC000 (chip stamps real AID; reserve high bits)
	 * BSSID, TA
	 */
	p = &buf[RTW88_RSVD_PAGE0_BYTES + 0 * RTW88_RSVD_PAGE_BYTES];
	p[0] = 0xA4;
	p[1] = 0x00;
	p[2] = 0x00; p[3] = 0xC0;	/* AID placeholder, high bits set */
	memcpy(&p[4],  ap_mac,     IEEE80211_ADDR_LEN);
	memcpy(&p[10], sc->sc_mac, IEEE80211_ADDR_LEN);

	/* Page 2: QOS_NULL (26 bytes).
	 * fc[0]=0xC8  type=DATA(2), subtype=QOS_NULL(12)
	 * fc[1]=0x01  ToDS
	 * duration=0
	 * addr1=BSSID, addr2=us, addr3=BSSID
	 * seq_ctrl=0, qos_ctrl=0
	 */
	p = &buf[RTW88_RSVD_PAGE0_BYTES + 1 * RTW88_RSVD_PAGE_BYTES];
	p[0] = 0xC8;
	p[1] = 0x01;
	memcpy(&p[4],  ap_mac,     IEEE80211_ADDR_LEN);
	memcpy(&p[10], sc->sc_mac, IEEE80211_ADDR_LEN);
	memcpy(&p[16], ap_mac,     IEEE80211_ADDR_LEN);

	/* Page 3: NULL data (24 bytes) -- the keep-alive template.
	 * fc[0]=0x48  type=DATA(2), subtype=NULL(4)
	 * fc[1]=0x01  ToDS
	 */
	p = &buf[RTW88_RSVD_PAGE0_BYTES + 2 * RTW88_RSVD_PAGE_BYTES];
	p[0] = 0x48;
	p[1] = 0x01;
	memcpy(&p[4],  ap_mac,     IEEE80211_ADDR_LEN);
	memcpy(&p[10], sc->sc_mac, IEEE80211_ADDR_LEN);
	memcpy(&p[16], ap_mac,     IEEE80211_ADDR_LEN);

	{
		int error;
		uint8_t cr1;

		error = rtw88_send_firmware_pkt(sc, RTW88_RSVD_DRV_ADDR,
		    sizeof(buf), buf);
		/*
		 * 2026-06-24: clear ENSWBCN after rsvd-page upload.
		 * rtw88_send_firmware_pkt setbits ENSWBCN to enable the
		 * chip's beacon-staging path during the upload, but never
		 * clears it afterwards. Linux's restore-leg pattern does.
		 * In STA mode ENSWBCN must be 0 -- leaving it on schedules
		 * a beacon TX in the SIFS window where HW ACK to AP should
		 * fire, starving ACK delivery (AP retransmits M1 in heavy
		 * bursts, supplicant declares WRONG_KEY, drops to SCANNING).
		 * See project_rtw88_drop_to_scanning_2026_06_24.md.
		 */
		if (rtw88_usb_read1(sc, REG_CR + 1, &cr1) == 0) {
			cr1 &= ~(uint8_t)(BIT_ENSWBCN >> 8);
			(void)rtw88_usb_write1(sc, REG_CR + 1, cr1);
		}
		return (error);
	}
}

/*
 * Write one 8-dword CAM entry containing a HW key.  Mirrors Linux's
 * rtw_sec_write_cam (sec.c:22).  The chip's CAM is indexed in 8-dword
 * units (REG_CAMCMD bits [4:0] = byte_offset = entry<<3 | dword_idx),
 * and the chip polls bit 31 to acknowledge each dword write.  We write
 * highest dword first because the polling on dword 0 latches the
 * full entry into the SA cache.
 *
 * Layout per Linux sec.c:
 *   dword 0: keyidx[1:0] | type[4:2] | group[6] | valid[15] | mac[0..1][16-31]
 *   dword 1: mac[2..5]
 *   dword 2..5: key[0..15] (16 bytes of CCMP/TKIP TK)
 *   dword 6..7: 0 (extended key, unused for AES)
 */
static int
rtw88_cam_write_entry(struct rtw88_usb_softc *sc, uint8_t slot,
    uint8_t type, uint8_t keyidx, bool group, const uint8_t *peer_mac,
    const uint8_t *key, size_t keylen)
{
	uint32_t addr = ((uint32_t)slot) << RTW88_CAM_ENTRY_SHIFT;
	uint32_t cmd, content;
	uint8_t mac[IEEE80211_ADDR_LEN];
	int i, j, error;

	RTW88_ASSERT_LOCKED(sc);

	if (keylen > 16)
		keylen = 16;
	memset(mac, 0, sizeof(mac));
	if (peer_mac != NULL)
		memcpy(mac, peer_mac, IEEE80211_ADDR_LEN);

	for (i = 7; i >= 0; i--) {
		switch (i) {
		case 0:
			content = ((uint32_t)(keyidx & 0x3)) |
			    (((uint32_t)type & 0x7) << 2) |
			    ((group ? 1U : 0U) << 6) |
			    (1U << 15) |	/* valid */
			    ((uint32_t)mac[0] << 16) |
			    ((uint32_t)mac[1] << 24);
			break;
		case 1:
			content = ((uint32_t)mac[2]) |
			    ((uint32_t)mac[3] << 8) |
			    ((uint32_t)mac[4] << 16) |
			    ((uint32_t)mac[5] << 24);
			break;
		case 6:
		case 7:
			content = 0;
			break;
		default:
			j = (i - 2) << 2;
			content = 0;
			if ((size_t)(j + 0) < keylen) content |=
			    (uint32_t)key[j + 0];
			if ((size_t)(j + 1) < keylen) content |=
			    ((uint32_t)key[j + 1]) << 8;
			if ((size_t)(j + 2) < keylen) content |=
			    ((uint32_t)key[j + 2]) << 16;
			if ((size_t)(j + 3) < keylen) content |=
			    ((uint32_t)key[j + 3]) << 24;
			break;
		}

		cmd = BIT_CAM_WRITE_ENABLE | BIT_CAM_POLLING |
		    (addr + (uint32_t)i);
		if ((error = rtw88_usb_write4(sc, REG_CAMWRITE, content)) != 0)
			return (error);
		if ((error = rtw88_usb_write4(sc, REG_CAMCMD, cmd)) != 0)
			return (error);
		/*
		 * Wait for chip-side polling to clear before the next
		 * dword write.  Without this, the dword 0 commit (which
		 * latches the full entry into the SA cache) sometimes
		 * runs while a prior write is still pending and the chip
		 * silently drops the dword 0 store.  Empirically validated
		 * 2026-06-26: prior to this wait, CAM-readback of slot 4
		 * post-install showed dword 0 = all-zero (valid=0, type=0,
		 * mac[0..1]=0); dwords 1..5 wrote correctly.
		 */
		{
			uint32_t poll;
			int k;
			for (k = 0; k < 16; k++) {
				if ((error = rtw88_usb_read4(sc, REG_CAMCMD,
				    &poll)) != 0)
					return (error);
				if ((poll & BIT_CAM_POLLING) == 0)
					break;
			}
		}
	}
	return (0);
}

/*
 * Read one dword from CAM slot at dword index 0..7.  Mirrors Linux
 * rtw_debugfs_get_dump_cam (debug.c:281): write CMD with POLLING bit
 * (no WRITE_ENABLE) and the byte-offset, then read REG_CAMREAD.
 * USB EP0 ordering means each request is serialized; we still poll
 * REG_CAMCMD for BIT_CAM_POLLING auto-clear before reading because
 * the chip's CAM-fetch on this register file is multi-cycle and
 * REG_CAMREAD may hold stale data until the polling bit drops.
 * Caller holds RTW88_LOCK.
 */
static int
rtw88_cam_read_dword(struct rtw88_usb_softc *sc, uint8_t slot,
    uint8_t dword_idx, uint32_t *out)
{
	uint32_t addr = ((uint32_t)slot) << RTW88_CAM_ENTRY_SHIFT;
	uint32_t cmd = BIT_CAM_POLLING | (addr + (uint32_t)dword_idx);
	uint32_t poll;
	int error, i;

	RTW88_ASSERT_LOCKED(sc);

	if ((error = rtw88_usb_write4(sc, REG_CAMCMD, cmd)) != 0)
		return (error);
	for (i = 0; i < 16; i++) {
		if ((error = rtw88_usb_read4(sc, REG_CAMCMD, &poll)) != 0)
			return (error);
		if ((poll & BIT_CAM_POLLING) == 0)
			break;
	}
	return (rtw88_usb_read4(sc, REG_CAMREAD, out));
}

static int
rtw88_cam_clear_entry(struct rtw88_usb_softc *sc, uint8_t slot)
{
	uint32_t addr = ((uint32_t)slot) << RTW88_CAM_ENTRY_SHIFT;
	uint32_t cmd;
	int error;

	RTW88_ASSERT_LOCKED(sc);

	cmd = BIT_CAM_WRITE_ENABLE | BIT_CAM_POLLING | addr;
	if ((error = rtw88_usb_write4(sc, REG_CAMWRITE, 0)) != 0)
		return (error);
	return (rtw88_usb_write4(sc, REG_CAMCMD, cmd));
}

/*
 * net80211 vap iv_key_set hook -- called when wpa_supplicant installs
 * a PTK / GTK via SIOCS80211 IEEE80211_IOC_WPAKEY.  We install the key
 * into the chip's CAM so the chip-side TX path (and the firmware's
 * KEEP_ALIVE NULL emission) can encrypt with the right TK.
 *
 * Slot assignment:
 *   - PAIRWISE (PTK): slot RTW88_CAM_PAIRWISE_SLOT (= 4), peer_mac = AP MAC
 *   - GROUP (GTK):    slot key->wk_keyix (0..3), broadcast peer mac
 *
 * Only WPA2 CCMP is wired up; WEP/TKIP stay on the software path.
 */
static int
rtw88_vap_key_set(struct ieee80211vap *vap, const struct ieee80211_key *k)
{
	struct rtw88_usb_softc *sc = vap->iv_ic->ic_softc;
	struct rtw88_cam_op *op;
	uint8_t type, slot, keylen;
	uint8_t bcast[IEEE80211_ADDR_LEN] = {
	    0xff, 0xff, 0xff, 0xff, 0xff, 0xff
	};
	const uint8_t *peer;
	bool group;

	if (k->wk_cipher == NULL)
		return (0);
	switch (k->wk_cipher->ic_cipher) {
	case IEEE80211_CIPHER_AES_CCM:
		type = RTW88_CAM_TYPE_AES;
		break;
	default:
		/* Fall through to software path. */
		return (1);
	}

	group = (k->wk_flags & IEEE80211_KEY_GROUP) != 0;
	if (group) {
		/*
		 * Group/GTK entries: MAC field is left zero, NOT broadcast.
		 * The CAM's MAC field is used by the chip's per-frame SA
		 * match on pairwise entries; group entries set the group
		 * flag so the chip skips SA match and looks up by keyidx
		 * (from the CCMP IV).  Storing broadcast here triggers the
		 * SA-match comparator unintentionally and the chip's RX
		 * CCMP engine flags ICV failure on every group frame.
		 * Matches Linux rtw_sec_write_cam (sec.c) which uses
		 * `memset(cam->addr, 0, ETH_ALEN)` for group keys.
		 */
		static const uint8_t zero_mac[IEEE80211_ADDR_LEN] = {0};
		peer = zero_mac;
		slot = k->wk_keyix & 0x3;
	} else {
		peer = (vap->iv_bss != NULL) ? vap->iv_bss->ni_bssid : bcast;
		slot = RTW88_CAM_PAIRWISE_SLOT;
	}
	(void)bcast;	/* may be unused if group path taken */

	keylen = (k->wk_keylen > 16) ? 16 : (uint8_t)k->wk_keylen;

	/*
	 * Capture the op under RTW88_LOCK; the actual EP0 traffic runs
	 * in sc_cam_task where no caller-held node lock can sleep-trap.
	 */
	RTW88_LOCK(sc);
	if (sc->sc_cam_q_n >= RTW88_CAM_QUEUE_DEPTH) {
		sc->sc_stat_cam_fails++;
		RTW88_UNLOCK(sc);
		device_printf(sc->sc_dev,
		    "cam queue full -- dropping install slot=%u\n", slot);
		return (0);
	}
	op = &sc->sc_cam_q[sc->sc_cam_q_n++];
	op->op = RTW88_CAM_OP_INSTALL;
	op->slot = slot;
	op->type = type;
	op->keyidx = (uint8_t)(k->wk_keyix & 0x3);
	op->group = group;
	memcpy(op->peer, peer, IEEE80211_ADDR_LEN);
	memset(op->key, 0, sizeof(op->key));
	if (keylen > 0)
		memcpy(op->key, k->wk_key, keylen);
	op->keylen = keylen;
	if (!group) {
		/*
		 * About to install the PTK.  Gate further Protected unicast
		 * TX through sc_tx_pending_q until sc_cam_task has actually
		 * programmed CAM slot 4 -- otherwise supplicant's immediate
		 * M4 races the install and is silently dropped by the chip
		 * (empty CAM slot for the destination MAC).
		 */
		sc->sc_ptk_install_pending = true;
	}
	RTW88_UNLOCK(sc);
	taskqueue_enqueue(sc->sc_cam_tq, &sc->sc_cam_task);
	return (1);
}

static int
rtw88_vap_key_delete(struct ieee80211vap *vap, const struct ieee80211_key *k)
{
	struct rtw88_usb_softc *sc = vap->iv_ic->ic_softc;
	struct rtw88_cam_op *op;
	uint8_t slot;

	if (k->wk_cipher == NULL ||
	    k->wk_cipher->ic_cipher != IEEE80211_CIPHER_AES_CCM)
		return (1);

	slot = ((k->wk_flags & IEEE80211_KEY_GROUP) != 0) ?
	    (k->wk_keyix & 0x3) : RTW88_CAM_PAIRWISE_SLOT;

	RTW88_LOCK(sc);
	if (sc->sc_cam_q_n >= RTW88_CAM_QUEUE_DEPTH) {
		sc->sc_stat_cam_fails++;
		RTW88_UNLOCK(sc);
		device_printf(sc->sc_dev,
		    "cam queue full -- dropping clear slot=%u\n", slot);
		return (1);
	}
	op = &sc->sc_cam_q[sc->sc_cam_q_n++];
	op->op = RTW88_CAM_OP_CLEAR;
	op->slot = slot;
	RTW88_UNLOCK(sc);
	taskqueue_enqueue(sc->sc_cam_tq, &sc->sc_cam_task);
	return (1);
}

/*
 * Drain the pending CAM install/clear queue.  Runs under sc_tq's
 * kthread (no caller-held net80211 node mutex).  Holds RTW88_LOCK
 * across the EP0 writes; usbd_do_request_flags drops the mtx around
 * cv_wait so the work is sleep-legal.
 *
 * Snapshot + zero the queue under RTW88_LOCK before draining so a
 * concurrent enqueue from a fresh iv_key_set/delete just appends to
 * an empty queue and re-enqueues this task -- no lost ops.
 */
static void
rtw88_cam_task(void *arg, int pending __unused)
{
	struct rtw88_usb_softc *sc = arg;
	struct rtw88_cam_op ops[RTW88_CAM_QUEUE_DEPTH];
	uint8_t n, i;
	int error;

	RTW88_LOCK(sc);
	n = sc->sc_cam_q_n;
	if (n > 0) {
		memcpy(ops, sc->sc_cam_q, (size_t)n * sizeof(ops[0]));
		sc->sc_cam_q_n = 0;
	}
	for (i = 0; i < n; i++) {
		struct rtw88_cam_op *op = &ops[i];

		if (op->op == RTW88_CAM_OP_INSTALL) {
			error = rtw88_cam_write_entry(sc, op->slot, op->type,
			    op->keyidx, op->group, op->peer, op->key,
			    (size_t)op->keylen);
			if (error != 0)
				device_printf(sc->sc_dev,
				    "cam_task install slot=%u failed: %d\n",
				    op->slot, error);
			else
				device_printf(sc->sc_dev,
				    "[T+%d ms] cam: installed %s key slot=%u "
				    "type=AES keyix=%u\n",
				    rtw88_assoc_ms(sc),
				    op->group ? "GTK" : "PTK",
				    op->slot, op->keyidx);

			/*
			 * PTK install just landed -- release any Protected
			 * unicast frames (M4 + any subsequent encrypted
			 * data) that rtw88_tx_enqueue_locked parked in
			 * sc_tx_pending_q.  Split by TID/EAPOL onto the same
			 * BE-vs-VO endpoints tx_enqueue_locked uses.  Held
			 * frames are Protected unicast, typically post-M4
			 * QoS data; route them by their AC.
			 */
			if (error == 0 && !op->group) {
				struct mbuf *pm;
				int released_be = 0, released_vo = 0;
				int m4_encapped = 0, m4_plaintext = 0;

				sc->sc_ptk_install_pending = false;
				while ((pm = mbufq_dequeue(
				    &sc->sc_tx_pending_q)) != NULL) {
					const uint8_t *fh = mtod(pm,
					    const uint8_t *);
					uint8_t fc0 = fh[0];
					uint8_t tid = 0;
					bool is_vo;
					uint16_t ki_d;
					bool is_eapol =
					    rtw88_eapol_key_info(pm, &ki_d);
					if ((fc0 & 0x80) != 0 &&
					    pm->m_pkthdr.len >= 26)
						tid = fh[24] & 0x07;
					is_vo = is_eapol || tid >= 4;
					/*
					 * M4 encap-deferred slot: this pm
					 * came through crypto_encap_if_needed
					 * with Protected=1 but NO SW CCMP
					 * ran (ni_ucastkey was CIPHER_NONE
					 * at that moment).  Now that CAM is
					 * programmed, retry the encap.
					 * Read ni from m_pkthdr.rcvif per
					 * net80211 raw_output convention.
					 */
					if (pm == sc->sc_m4_needs_encap) {
						struct ieee80211_node *ni =
						    (struct ieee80211_node *)
						    pm->m_pkthdr.rcvif;
						sc->sc_m4_needs_encap = NULL;
						if (ni != NULL &&
						    ieee80211_crypto_encap(ni,
						    pm) != NULL) {
							m4_encapped++;
						} else {
							struct ieee80211_frame
							    *wh2 = mtod(pm,
							    struct
							    ieee80211_frame *);
							wh2->i_fc[1] &=
							    ~IEEE80211_FC1_PROTECTED;
							m4_plaintext++;
						}
					}
					if (is_vo) {
						if (mbufq_enqueue(
						    &sc->sc_tx_data_vo_q,
						    pm) != 0) {
							m_freem(pm);
							continue;
						}
						released_vo++;
					} else {
						if (mbufq_enqueue(
						    &sc->sc_tx_data_q,
						    pm) != 0) {
							m_freem(pm);
							continue;
						}
						released_be++;
					}
				}
				if (m4_encapped + m4_plaintext > 0)
					device_printf(sc->sc_dev,
					    "[T+%d ms] cam: M4 drain-encap "
					    "%d encrypted, %d plaintext\n",
					    rtw88_assoc_ms(sc),
					    m4_encapped, m4_plaintext);
				if (released_be + released_vo > 0) {
					device_printf(sc->sc_dev,
					    "[T+%d ms] cam: released %d BE + "
					    "%d VO held Protected unicast "
					    "frame(s) post-PTK\n",
					    rtw88_assoc_ms(sc),
					    released_be, released_vo);
					if (released_be > 0)
						usbd_transfer_start(sc->sc_xfer[
						    RTW88_BULK_TX_LO]);
					if (released_vo > 0)
						usbd_transfer_start(sc->sc_xfer[
						    RTW88_BULK_TX_NORMAL]);
				}
			}
		} else {
			error = rtw88_cam_clear_entry(sc, op->slot);
			if (error != 0)
				device_printf(sc->sc_dev,
				    "cam_task clear slot=%u failed: %d\n",
				    op->slot, error);
		}
	}
	RTW88_UNLOCK(sc);
}

/*
 * EDCA per-AC parameter write.  Mirrors Linux's __rtw_conf_tx
 * (mac80211.c:345): single 32-bit register encodes TXOP/CWMAX/CWMIN/AIFS.
 * Caller supplies ecw_max/ecw_min already in log2 form -- net80211's
 * wmep_logcwmax/logcwmin are pre-log2 (same convention).
 *
 *   bits 26:16 TXOP_LMT  (11-bit unit, 32us)
 *   bits 15:12 CWMAX     (ecw = log2(cw+1))
 *   bits 11: 8 CWMIN     (ecw)
 *   bits  7: 0 AIFS      (us)
 */
static const uint16_t rtw88_ac_to_edca_reg[WME_NUM_AC] = {
	[WME_AC_BE] = REG_EDCA_BE_PARAM,
	[WME_AC_BK] = REG_EDCA_BK_PARAM,
	[WME_AC_VI] = REG_EDCA_VI_PARAM,
	[WME_AC_VO] = REG_EDCA_VO_PARAM,
};

static int
rtw88_conf_tx_one_ac(struct rtw88_usb_softc *sc, uint8_t ac,
    uint16_t txop, uint8_t ecw_max, uint8_t ecw_min, uint8_t aifs)
{
	uint32_t val;

	if (ac >= WME_NUM_AC)
		return (EINVAL);
	val = ((uint32_t)(txop & 0x7FF) << 16) |
	    ((uint32_t)(ecw_max & 0xF) << 12) |
	    ((uint32_t)(ecw_min & 0xF) << 8) |
	    ((uint32_t)aifs);
	return (rtw88_usb_write4(sc, rtw88_ac_to_edca_reg[ac], val));
}

/*
 * Walk net80211's per-AC wme params and push them to the chip.  Net80211
 * fires ic_wme.wme_update on (a) assoc-resp parse and (b) IBSS WME
 * change.  We previously stubbed this -- now we honour it.  Sub-AUTH
 * traffic doesn't flow through EDCA (MGMT goes via MGMT queue with
 * its own backoff), so this is post-ASSOC plumbing.
 */
static int
rtw88_conf_tx_all_ac_locked(struct rtw88_usb_softc *sc,
    struct ieee80211com *ic)
{
	const struct wmeParams *p;
	int ac, error;

	RTW88_ASSERT_LOCKED(sc);
	for (ac = 0; ac < WME_NUM_AC; ac++) {
		p = &ic->ic_wme.wme_chanParams.cap_wmeParams[ac];
		if ((error = rtw88_conf_tx_one_ac(sc, (uint8_t)ac,
		    p->wmep_txopLimit, p->wmep_logcwmax,
		    p->wmep_logcwmin, p->wmep_aifsn)) != 0)
			return (error);
	}
	return (0);
}

/*
 * Top-level "switch the radio to channel N at bandwidth W".  Caller
 * takes sc_mtx.  Dispatches to the 2.4 GHz or 5 GHz BB+RF paths
 * based on the channel number.  bw selects 20/40/80 MHz;
 * primary_ch_idx is the primary 20 MHz position within the wider
 * channel (0 = upper, 1 = lower, per Linux rtw88 convention).
 */
static int
rtw88_phy_set_channel(struct rtw88_usb_softc *sc, uint8_t channel,
    uint8_t bw, uint8_t primary_ch_idx)
{
	bool is_5g;
	int error;

	RTW88_LOCK(sc);
	if (channel < 1 || (channel > 14 && channel < 36) || channel > 177) {
		RTW88_UNLOCK(sc);
		return (EINVAL);
	}
	is_5g = (channel > 14);
	if (is_5g && !(sc->sc_chip->caps & RTW88_CAP_5GHZ)) {
		RTW88_UNLOCK(sc);
		return (EOPNOTSUPP);
	}
	if (!is_5g && !(sc->sc_chip->caps & RTW88_CAP_2GHZ)) {
		RTW88_UNLOCK(sc);
		return (EOPNOTSUPP);
	}
	if ((bw == RTW88_CHANNEL_WIDTH_40 &&
	     !(sc->sc_chip->caps & RTW88_CAP_HT40)) ||
	    (bw == RTW88_CHANNEL_WIDTH_80 &&
	     !(sc->sc_chip->caps & RTW88_CAP_VHT80))) {
		bw = RTW88_CHANNEL_WIDTH_20;
		primary_ch_idx = 0;
	}

	/*
	 * Match reference rtw_set_channel() ordering: BB tune first, then
	 * MAC-side housekeeping, then RF retune, finally RXDFIR.
	 */
	if (is_5g) {
		if ((error = rtw88_set_channel_bb_5g(sc, channel, bw,
		    primary_ch_idx)) != 0)
			goto out;
	} else {
		if ((error = rtw88_set_channel_bb_2g(sc, channel, bw,
		    primary_ch_idx)) != 0)
			goto out;
	}
	if ((error = rtw88_set_channel_mac(sc, channel, bw, primary_ch_idx))
	    != 0)
		goto out;
	if (is_5g) {
		if ((error = rtw88_set_channel_rf_5g(sc, channel, bw)) != 0)
			goto out;
	} else {
		if ((error = rtw88_set_channel_rf_2g(sc, channel, bw)) != 0)
			goto out;
	}
	if ((error = rtw88_set_channel_rxdfir(sc, bw)) != 0)
		goto out;
	sc->sc_cur_bw = bw;
	sc->sc_cur_primary_ch_idx = primary_ch_idx;
	/*
	 * Inform firmware of the new channel.  EP0 mailbox H2C cmd;
	 * non-fatal on failure (we still consider the channel set
	 * since the BB/RF writes already landed).
	 */
	if (rtw88_h2c_wl_ch_info(sc, 0, channel, 0) != 0)
		device_printf(sc->sc_dev,
		    "h2c_wl_ch_info chan=%u: mailbox busy/timeout\n",
		    channel);

	/*
	 * Enable the chip's per-packet TX status report engine.  Linux's
	 * 8821c rtw88 writes REG_FWHW_TXQ_CTRL byte 0 = 0x80 + byte 1 =
	 * 0x1f after every set_channel (qemu-trace-x86/run4/
	 * linux-bpftrace-auth.out, post-SET_CHANNEL block).  Without
	 * these, even with W2.SPE_RPT set on the mgmt TX desc the chip
	 * does not emit the CCX TX report C2H -- our 5000-frame rxtrc
	 * captures show 0 C2H of any type, matching the "no instrument"
	 * gap the byte-diff turn surfaced.
	 *
	 * rtw88xxa names byte 1 bits "enable Tx report" (rtw88xxa.c:1203,
	 * value 0x0f); 8821c uses 0x1f (one extra bit).  Byte 0 bit 7
	 * (0x80) is also part of the same write group, exact semantic
	 * unclear from comments but consistently set in the trace.
	 */
	(void)rtw88_usb_write1(sc, REG_FWHW_TXQ_CTRL, 0x80);
	(void)rtw88_usb_write1(sc, REG_FWHW_TXQ_CTRL + 1, 0x1F);

	/*
	 * TXAGC per-rate power-index table.  Captured 2026-07-02 from
	 * Linux rtw88_8821cu usbmon (parity_capture_2026_07_01/usbmon.pcap
	 * frames 30019-30033) at t=22.48s -- fires between the EDCA
	 * writes and M2 tx.  Session 4 memory `_missing_funcs_catalog`
	 * flagged this range as "silently drops without preconditions";
	 * the preconditions land in rtw88_set_channel_mac above.  Values
	 * are packed per-rate 0.5 dBm indices for 2.4 GHz channel 3-6.
	 *
	 * Without these writes, M2 goes out at the chip's default TX
	 * power (from phy_set_param seeding) which correlates with the
	 * M2->M3 wall pattern (AP retries M1, never advances) -- likely
	 * PA nonlinearity or under-drive causing AP-side CRC fail.
	 *
	 * TODO: derive from chip's TX-power lookup table + channel + regd
	 * instead of hardcoded values.  For now, hardcode ch3-6 defaults
	 * so we can validate the M3-wall hypothesis on DUT.
	 */
	if (!is_5g) {
		/* All values are LE u32 interpretations of the raw bytes
		 * Linux wrote in usbmon frames 30019-30033. */
		(void)rtw88_usb_write4(sc, 0x1D00, 0x2D2D2D2D);
		(void)rtw88_usb_write4(sc, 0x1D04, 0x35353535);
		/*
		 * Linux writes bytes 35 33 31 2f LE at 0x1D08 (verified via
		 * reg_monitor.py --handshake-diff on Linux usbmon capture
		 * 2026-07-02).  Prior value 0x2F313533 had bytes 0-1 swapped,
		 * costing ~1 dBm on one power-index rate.
		 */
		(void)rtw88_usb_write4(sc, 0x1D08, 0x2F313335);
		(void)rtw88_usb_write4(sc, 0x1D0C, 0x33333333);
		(void)rtw88_usb_write4(sc, 0x1D10, 0x2D2F3133);
		(void)rtw88_usb_write4(sc, 0x1D2C, 0x35373737);
		(void)rtw88_usb_write4(sc, 0x1D30, 0x2D2F3133);
		(void)rtw88_usb_write4(sc, 0x1D34, 0x0000292B);
	}
out:
	if (error == 0)
		sc->sc_cur_channel = channel;
	RTW88_UNLOCK(sc);
	return (error);
}

/*
 * PHY parameterisation -- the small, no-tables half of reference mainline
 * rtw8821c.c rtw8821c_phy_set_param().  Powers the BB and RF blocks
 * on, toggles the BB reset, clears the RXPSEL reset to let the chip
 * latch the table data we're about to load, walks the MAC table
 * (138 byte writes), applies the EFUSE crystal cap to the AFE
 * XTAL/PLL registers, and asserts RXPSEL reset post-table to commit.
 *
 * Crystal cap is the per-chip frequency-trim the OTP EFUSE holds for
 * its 40 MHz reference oscillator.  Without this write the radio's
 * channel center frequency is off by tens of kHz -- enough to put
 * any sniffed beacon outside the receiver bandwidth.
 *
 * The BB / AGC / RF tables (~6000 entries total) live in the
 * rtw88-rtl8821ctbl kmod and are loaded via rtw88_load_mac_tbl below.
 *
 * Caller must hold sc_mtx.
 */
static int
rtw88_load_mac_tbl(struct rtw88_usb_softc *sc)
{
	size_t i;
	int error;

	RTW88_ASSERT_LOCKED(sc);

	for (i = 0; i < RTW88_8821C_MAC_TBL_NENTRIES; i++) {
		error = rtw88_usb_write1(sc, rtw88_8821c_mac_tbl[i].addr,
		    rtw88_8821c_mac_tbl[i].value);
		if (error != 0) {
			device_printf(sc->sc_dev,
			    "mac_tbl[%zu] addr=0x%03x failed: %d\n",
			    i, rtw88_8821c_mac_tbl[i].addr, error);
			return (error);
		}
	}
	return (0);
}

/*
 * Binary "rtw88-rtl8821ctbl" firmware blob walker.
 *
 * Header layout (little-endian):
 *
 *   bytes 0..3  : 'R88T' magic
 *   byte  4     : format version (== 1)
 *   byte  5     : table count
 *   bytes 6..7  : reserved
 *
 * For each table:
 *
 *   byte  0     : kind  (1=MAC, 2=BB, 3=AGC, 4=RF_A)
 *   bytes 1..3  : reserved
 *   bytes 4..7  : u32 entry count = number of u32 words in payload
 *   payload     : u32[entry_count]
 *
 * Conditional sentinels embedded in tables:
 *
 *   bit 31 (pos)    -> the entry is a "positive" condition open; the
 *                      next word is a cond2 payload (currently unused
 *                      on 8821C but consumed for layout fidelity).
 *                      branch in bits 28..29 selects IF / ELIF / ELSE /
 *                      ENDIF.  IF/ELIF store the cond for matching;
 *                      ENDIF resets to "match" state; ELSE inverts.
 *
 *   bit 30 (neg)    -> evaluate the saved positive cond against
 *                      (sc_cut, sc_rfe, sc_pkg, INTF_USB).  Decide
 *                      whether subsequent normal entries fire.
 *
 *   neither bit set -> normal (addr, value) pair; if currently
 *                      "matched", invoke the per-kind config function.
 */

#define	RTW88_TBL_KIND_MAC	1
#define	RTW88_TBL_KIND_BB	2
#define	RTW88_TBL_KIND_AGC	3
#define	RTW88_TBL_KIND_RF_A	4

#define	RTW88_PHY_COND_BIT_POS		(1U << 31)
#define	RTW88_PHY_COND_BIT_NEG		(1U << 30)
#define	RTW88_PHY_COND_BRANCH(c)	(((c) >> 28) & 0x3)
#define	RTW88_PHY_COND_CUT(c)		(((c) >> 24) & 0xF)
#define	RTW88_PHY_COND_PKG(c)		(((c) >> 12) & 0xF)
#define	RTW88_PHY_COND_INTF(c)		(((c) >> 8) & 0xF)
#define	RTW88_PHY_COND_RFE(c)		((c) & 0xFF)

#define	RTW88_PHY_COND_BRANCH_IF	0
#define	RTW88_PHY_COND_BRANCH_ELIF	1
#define	RTW88_PHY_COND_BRANCH_ELSE	2
#define	RTW88_PHY_COND_BRANCH_ENDIF	3

#define	RTW88_INTF_USB			(1U << 1)	/* match reference INTF_USB */

static bool
rtw88_phy_cond_check(struct rtw88_usb_softc *sc, uint32_t cond)
{

	if (RTW88_PHY_COND_CUT(cond) != 0 &&
	    RTW88_PHY_COND_CUT(cond) != sc->sc_cut_version)
		return (false);
	if (RTW88_PHY_COND_PKG(cond) != 0 &&
	    RTW88_PHY_COND_PKG(cond) != sc->sc_pkg_type)
		return (false);
	if (RTW88_PHY_COND_INTF(cond) != 0 &&
	    RTW88_PHY_COND_INTF(cond) != RTW88_INTF_USB)
		return (false);
	/* 8821C: rfe must match exactly. */
	if (RTW88_PHY_COND_RFE(cond) != sc->sc_rfe_option)
		return (false);
	return (true);
}

/*
 * BB Serial Programmable Interface Interface (SIPI) bus addresses.
 * reference 8821C ops table: .rf_sipi_addr = {0xc90, 0xe90} -- path A
 * + path B.  RTL8821CU is 1T1R, only path A is wired.
 */
#define	RTW88_RF_SIPI_ADDR_A	0x0C90
#define	RTW88_RF_REG_ADDR_MASK	0xFF	/* SIPI command holds 8-bit addr */
#define	RTW88_RF_DATA_MASK	0xFFFFF	/* SIPI command holds 20-bit data */

/*
 * Compose + dispatch a SIPI write to the RF chip's serial port.
 * reference phy.c rtw_phy_write_rf_reg_sipi when mask == RFREG_MASK:
 *   data_and_addr = ((addr << 20) | (data & 0xfffff)) & 0xfffffff;
 *   write32(sipi_addr[path], data_and_addr);
 *   udelay(13);
 * For table-driven RF init the mask is always RFREG_MASK, so no
 * read-modify-write is needed.
 */
static int
rtw88_phy_write_rf_a(struct rtw88_usb_softc *sc, uint32_t addr, uint32_t data)
{
	uint32_t cmd;
	int error;

	RTW88_ASSERT_LOCKED(sc);

	cmd = ((addr & RTW88_RF_REG_ADDR_MASK) << 20) |
	    (data & RTW88_RF_DATA_MASK);
	cmd &= 0x0FFFFFFF;
	if ((error = rtw88_usb_write4(sc, RTW88_RF_SIPI_ADDR_A, cmd)) != 0)
		return (error);
	DELAY(13);
	return (0);
}

static int
rtw88_phy_apply(struct rtw88_usb_softc *sc, uint8_t kind, uint32_t addr,
    uint32_t data)
{

	/* BB-table delay sentinels (addr in 0xF9..0xFE means "sleep"). */
	if (kind == RTW88_TBL_KIND_BB) {
		switch (addr) {
		case 0xFE:
			usb_pause_mtx(&sc->sc_mtx, USB_MS_TO_TICKS(50));
			return (0);
		case 0xFD:
			usb_pause_mtx(&sc->sc_mtx, USB_MS_TO_TICKS(5));
			return (0);
		case 0xFC:
			usb_pause_mtx(&sc->sc_mtx, USB_MS_TO_TICKS(1));
			return (0);
		case 0xFB:
			DELAY(60);
			return (0);
		case 0xFA:
			DELAY(5);
			return (0);
		case 0xF9:
			DELAY(1);
			return (0);
		}
	}

	/* RF-table delay sentinels. */
	if (kind == RTW88_TBL_KIND_RF_A) {
		switch (addr) {
		case 0xFFE:
			usb_pause_mtx(&sc->sc_mtx, USB_MS_TO_TICKS(50));
			return (0);
		case 0xFE:
			DELAY(110);
			return (0);
		}
	}

	switch (kind) {
	case RTW88_TBL_KIND_MAC:
		return (rtw88_usb_write1(sc, (uint16_t)addr, (uint8_t)data));
	case RTW88_TBL_KIND_BB:
	case RTW88_TBL_KIND_AGC:
		return (rtw88_usb_write4(sc, (uint16_t)addr, data));
	case RTW88_TBL_KIND_RF_A: {
		int error;

		error = rtw88_phy_write_rf_a(sc, addr, data);
		if (error == 0)
			DELAY(1);
		return (error);
	    }
	default:
		return (EINVAL);
	}
}

static int
rtw88_phy_walk_table(struct rtw88_usb_softc *sc, uint8_t kind,
    const uint32_t *words, uint32_t nwords, uint32_t *applied_out)
{
	uint32_t pos_cond = 0;
	bool is_matched = true, is_skipped = false;
	uint32_t i, applied = 0;
	int error = 0;

	RTW88_ASSERT_LOCKED(sc);

	for (i = 0; i + 1 < nwords; i += 2) {
		uint32_t w0 = words[i];
		uint32_t w1 = words[i + 1];

		if (w0 & RTW88_PHY_COND_BIT_POS) {
			switch (RTW88_PHY_COND_BRANCH(w0)) {
			case RTW88_PHY_COND_BRANCH_ENDIF:
				is_matched = true;
				is_skipped = false;
				break;
			case RTW88_PHY_COND_BRANCH_ELSE:
				is_matched = is_skipped ? false : true;
				break;
			default:	/* IF, ELIF */
				pos_cond = w0;
				break;
			}
		} else if (w0 & RTW88_PHY_COND_BIT_NEG) {
			if (!is_skipped) {
				if (rtw88_phy_cond_check(sc, pos_cond)) {
					is_matched = true;
					is_skipped = true;
				} else {
					is_matched = false;
					is_skipped = false;
				}
			} else {
				is_matched = false;
			}
		} else if (is_matched) {
			error = rtw88_phy_apply(sc, kind, w0, w1);
			if (error != 0) {
				device_printf(sc->sc_dev,
				    "tbl kind=%u apply addr=0x%x: %d\n",
				    kind, w0, error);
				return (error);
			}
			applied++;
		}
	}
	if (applied_out != NULL)
		*applied_out = applied;
	return (0);
}

static int
rtw88_phy_load_blob_locked(struct rtw88_usb_softc *sc)
{
	const uint8_t *p;
	size_t remain;
	uint8_t version, ntables, kind;
	uint32_t nwords, applied;
	const char *kindname;
	int i, error;

	RTW88_ASSERT_LOCKED(sc);

	if (sc->sc_tbl_blob == NULL || sc->sc_tbl_size < 8)
		return (ENOENT);

	p = sc->sc_tbl_blob;
	remain = sc->sc_tbl_size;
	if (memcmp(p, "R88T", 4) != 0) {
		device_printf(sc->sc_dev, "table blob bad magic\n");
		return (EINVAL);
	}
	version = p[4];
	ntables = p[5];
	if (version != 1) {
		device_printf(sc->sc_dev,
		    "table blob bad version %u\n", version);
		return (EINVAL);
	}
	p += 8;
	remain -= 8;

	for (i = 0; i < ntables; i++) {
		if (remain < 8)
			return (EINVAL);
		kind = p[0];
		nwords = le32dec(p + 4);
		p += 8;
		remain -= 8;
		if ((size_t)nwords * 4 > remain) {
			device_printf(sc->sc_dev,
			    "table[%d] truncated (need %u words, have %zu)\n",
			    i, nwords, remain / 4);
			return (EINVAL);
		}

		applied = 0;
		switch (kind) {
		case RTW88_TBL_KIND_MAC: kindname = "MAC"; break;
		case RTW88_TBL_KIND_BB:  kindname = "BB";  break;
		case RTW88_TBL_KIND_AGC: kindname = "AGC"; break;
		case RTW88_TBL_KIND_RF_A: kindname = "RF_A"; break;
		default: kindname = "?"; break;
		}

		error = rtw88_phy_walk_table(sc, kind,
		    (const uint32_t *)p, nwords, &applied);
		if (error != 0)
			return (error);
		device_printf(sc->sc_dev,
		    "tbl %s: %u words, %u writes applied\n",
		    kindname, nwords, applied);

		p += (size_t)nwords * 4;
		remain -= (size_t)nwords * 4;
	}
	return (0);
}

/*
 * Walk the RF_A section of the blob a SECOND time.  Used after the
 * RFE-conditional agc_btg table fires so the RF writes land into a
 * chip with the BTG-mode AGC LUT already populated.  Matches Linux's
 * rtw_phy_load_tables order (agc_btg before rf_tbl) -- our blob's
 * fixed order forces us to re-walk RF_A explicitly here.
 */
static int
rtw88_phy_replay_rf_a(struct rtw88_usb_softc *sc)
{
	const uint8_t *p;
	size_t remain;
	uint8_t kind;
	uint32_t nwords, applied;
	int error;

	RTW88_ASSERT_LOCKED(sc);

	if (sc->sc_tbl_blob == NULL || sc->sc_tbl_size < 8)
		return (ENOENT);
	p = sc->sc_tbl_blob;
	remain = sc->sc_tbl_size;
	if (memcmp(p, "R88T", 4) != 0)
		return (EINVAL);
	p += 8;
	remain -= 8;

	while (remain >= 8) {
		kind = p[0];
		nwords = le32dec(p + 4);
		p += 8;
		remain -= 8;
		if ((size_t)nwords * 4 > remain)
			return (EINVAL);
		if (kind == RTW88_TBL_KIND_RF_A) {
			applied = 0;
			error = rtw88_phy_walk_table(sc, kind,
			    (const uint32_t *)p, nwords, &applied);
			if (error != 0)
				return (error);
			device_printf(sc->sc_dev,
			    "tbl RF_A (replay): %u writes applied\n",
			    applied);
			return (0);
		}
		p += (size_t)nwords * 4;
		remain -= (size_t)nwords * 4;
	}
	return (ENOENT);
}

static int
rtw88_phy_acquire_tbl(struct rtw88_usb_softc *sc)
{
	const struct firmware *fw;

	if (sc->sc_tbl_fw != NULL)
		return (0);
	fw = firmware_get("rtw88-rtl8821ctbl");
	if (fw == NULL)
		return (ENOENT);
	sc->sc_tbl_fw = fw;
	sc->sc_tbl_blob = fw->data;
	sc->sc_tbl_size = fw->datasize;
	return (0);
}

static void
rtw88_phy_release_tbl(struct rtw88_usb_softc *sc)
{

	if (sc->sc_tbl_fw != NULL) {
		firmware_put(sc->sc_tbl_fw, FIRMWARE_UNLOAD);
		sc->sc_tbl_fw = NULL;
		sc->sc_tbl_blob = NULL;
		sc->sc_tbl_size = 0;
	}
}

/*
 * Port of Linux's rtw_bf_phy_init (bf.c:347) + the trailing
 * grouping-bitmap write from rtw8821c_phy_bf_init (rtw8821c.c:151).
 * Linux calls this from the tail of rtw8821c_phy_set_param, AFTER
 * rtw_phy_init / pwrtrack_init.
 *
 * Register addresses + bit names lifted verbatim from bf.h.
 */
#define	RTW88_BF_REG_TXBF_CTRL			0x042CU
#define	RTW88_BF_REG_NDPA_OPT_CTRL		0x045FU
#define	RTW88_BF_REG_BBPSF_CTRL			0x06DCU
#define	RTW88_BF_REG_MU_TX_CTL			0x14C0U
#define	RTW88_BF_REG_WMAC_MU_BF_OPTION		0x167CU
#define	RTW88_BF_REG_WMAC_MU_BF_CTL		0x1680U
#define	RTW88_BF_REG_GROUPING_BITMAP_8821C	0x1C94U

#define	RTW88_BF_BIT_USE_NDPA_PARAMETER		(1U << 30)
#define	RTW88_BF_BIT_MU_P1_WAIT_STATE_EN	(1U << 16)
#define	RTW88_BF_BIT_EN_MU_MIMO			(1U << 7)
#define	RTW88_BF_BIT_WMAC_TXMU_ACKPOLICY_EN	(1U << 6)
#define	RTW88_BF_BIT_SHIFT_R_MU_RL		12
#define	RTW88_BF_BIT_MASK_R_MU_RL		(0xfU << 12)
#define	RTW88_BF_BIT_MASK_R_MU_TABLE_VALID	0x3fU
#define	RTW88_BF_BIT_SHIFT_CSI_RATE		24
#define	RTW88_BF_BIT_MASK_CSI_RATE		(0x3fU << 24)
#define	RTW88_BF_BIT_SHIFT_WMAC_TXMU_ACKPOLICY	4
#define	RTW88_BF_DESC_RATE6M			0x04U

static void
rtw88_bf_phy_init(struct rtw88_usb_softc *sc)
{
	uint32_t tmp32;
	uint8_t tmp8;
	const uint8_t retry_limit = 0xA;
	const uint8_t ndpa_rate = 0x10;
	const uint8_t ack_policy = 3;

	/* REG_MU_TX_CTL: set MU_P1_WAIT_STATE_EN + R_MU_RL=0xA,
	 * clear EN_MU_MIMO + R_MU_TABLE_VALID. */
	tmp32 = 0;
	(void)rtw88_usb_read4(sc, RTW88_BF_REG_MU_TX_CTL, &tmp32);
	tmp32 |= RTW88_BF_BIT_MU_P1_WAIT_STATE_EN;
	tmp32 &= ~RTW88_BF_BIT_MASK_R_MU_RL;
	tmp32 |= ((uint32_t)retry_limit << RTW88_BF_BIT_SHIFT_R_MU_RL) &
	    RTW88_BF_BIT_MASK_R_MU_RL;
	tmp32 &= ~RTW88_BF_BIT_EN_MU_MIMO;
	tmp32 &= ~RTW88_BF_BIT_MASK_R_MU_TABLE_VALID;
	(void)rtw88_usb_write4(sc, RTW88_BF_REG_MU_TX_CTL, tmp32);

	/* REG_WMAC_MU_BF_OPTION = (ack_policy<<4) | ACKPOLICY_EN. */
	tmp8 = (uint8_t)(ack_policy << RTW88_BF_BIT_SHIFT_WMAC_TXMU_ACKPOLICY);
	tmp8 |= RTW88_BF_BIT_WMAC_TXMU_ACKPOLICY_EN;
	(void)rtw88_usb_write1(sc, RTW88_BF_REG_WMAC_MU_BF_OPTION, tmp8);

	/* REG_WMAC_MU_BF_CTL = 0. */
	(void)rtw88_usb_write2(sc, RTW88_BF_REG_WMAC_MU_BF_CTL, 0);

	/* REG_TXBF_CTRL: set BIT(30) USE_NDPA_PARAMETER. */
	tmp32 = 0;
	(void)rtw88_usb_read4(sc, RTW88_BF_REG_TXBF_CTRL, &tmp32);
	tmp32 |= RTW88_BF_BIT_USE_NDPA_PARAMETER;
	(void)rtw88_usb_write4(sc, RTW88_BF_REG_TXBF_CTRL, tmp32);

	/* REG_NDPA_OPT_CTRL = 0x10. */
	(void)rtw88_usb_write1(sc, RTW88_BF_REG_NDPA_OPT_CTRL, ndpa_rate);

	/* REG_BBPSF_CTRL: CSI_RATE field = DESC_RATE6M (0x4). */
	tmp32 = 0;
	(void)rtw88_usb_read4(sc, RTW88_BF_REG_BBPSF_CTRL, &tmp32);
	tmp32 &= ~RTW88_BF_BIT_MASK_CSI_RATE;
	tmp32 |= ((uint32_t)RTW88_BF_DESC_RATE6M <<
	    RTW88_BF_BIT_SHIFT_CSI_RATE);
	(void)rtw88_usb_write4(sc, RTW88_BF_REG_BBPSF_CTRL, tmp32);

	/* 8821C-specific grouping bitmap (rtw8821c_phy_bf_init). */
	(void)rtw88_usb_write4(sc, RTW88_BF_REG_GROUPING_BITMAP_8821C,
	    0xAFFFAFFFU);
}

static int
rtw88_phy_set_param(struct rtw88_usb_softc *sc)
{
	uint8_t val;
	uint32_t val32;
	int error;

	RTW88_LOCK(sc);

	/* Power on BB block via SYS_FUNC_EN. */
	if ((error = rtw88_usb_read1(sc, REG_SYS_FUNC_EN, &val)) != 0)
		goto out;
	val |= BIT_FEN_PCIEA;
	if ((error = rtw88_usb_write1(sc, REG_SYS_FUNC_EN, val)) != 0)
		goto out;

	/* Toggle BB reset (low-high-low-high). */
	val |= BIT_FEN_BB_RSTB | BIT_FEN_BB_GLB_RST;
	if ((error = rtw88_usb_write1(sc, REG_SYS_FUNC_EN, val)) != 0)
		goto out;
	val &= ~(BIT_FEN_BB_RSTB | BIT_FEN_BB_GLB_RST);
	if ((error = rtw88_usb_write1(sc, REG_SYS_FUNC_EN, val)) != 0)
		goto out;
	val |= BIT_FEN_BB_RSTB | BIT_FEN_BB_GLB_RST;
	if ((error = rtw88_usb_write1(sc, REG_SYS_FUNC_EN, val)) != 0)
		goto out;

	/* RF enable. */
	if ((error = rtw88_usb_write1(sc, REG_RF_CTRL,
	    BIT_RF_EN | BIT_RF_RSTB | BIT_RF_SDM_RSTB)) != 0)
		goto out;
	DELAY(10);
	if ((error = rtw88_usb_write1(sc, REG_WLRF1 + 3,
	    BIT_RF_EN | BIT_RF_RSTB | BIT_RF_SDM_RSTB)) != 0)
		goto out;
	DELAY(10);

	/* Pre-table: clear RXPSEL reset so chip latches incoming writes. */
	if ((error = rtw88_usb_read4(sc, REG_RXPSEL, &val32)) != 0)
		goto out;
	val32 &= ~BIT_RX_PSEL_RST;
	if ((error = rtw88_usb_write4(sc, REG_RXPSEL, val32)) != 0)
		goto out;

	/*
	 * Load the BB/AGC/RF table blob.  Carries mac_tbl + BB (3360
	 * words), AGC (2400 words), and RF_A (5424 words).  Falls back to
	 * the static built-in mac_tbl if the blob kmod isn't loaded.  The
	 * walker honours conditional sentinels using sc_cut_version /
	 * sc_rfe_option / sc_pkg_type.
	 */
	if ((error = rtw88_phy_acquire_tbl(sc)) != 0) {
		device_printf(sc->sc_dev,
		    "table blob unavailable (%d); falling back to"
		    " static mac_tbl\n", error);
		if ((error = rtw88_load_mac_tbl(sc)) != 0)
			goto out;
	} else {
		if ((error = rtw88_phy_load_blob_locked(sc)) != 0)
			goto out;
	}

	/* Diagnostic checkpoint: RF readback IMMEDIATELY after blob walk. */
	{
		uint32_t v_7d, v_78, v_c7;
		(void)rtw88_phy_read_rf_a(sc, 0x7d, RTW88_RFREG_MASK, &v_7d);
		(void)rtw88_phy_read_rf_a(sc, 0x78, RTW88_RFREG_MASK, &v_78);
		(void)rtw88_phy_read_rf_a(sc, 0xc7, RTW88_RFREG_MASK, &v_c7);
		device_printf(sc->sc_dev,
		    "post-blob-walk canary: RF[0x7d]=0x%05x"
		    " RF[0x78]=0x%05x RF[0xc7]=0x%05x\n",
		    v_7d, v_78, v_c7);
	}

	/*
	 * 2026-06-25 RF readback investigation — closed unresolved.
	 *
	 * Linux usbmon trace at t=15.16-15.18 shows exact RF init
	 * sequence: RF[0xEE]=0 bank-switch, then writes to RF[0x70/75/
	 * 76/77/78/7D/7F/6A/65], then RF[0xEE]=0x8000 restore. Same SIPI
	 * encoding as us.
	 *
	 * Attempted to replicate the bank-switch + writes here.
	 * IMMEDIATE readback STILL shows RF[0x78]=0x88000 RF[0x7D]=0
	 * regardless of bank or write attempts. So the readback is
	 * misleading: RF[0x78] returns a fixed value (chip
	 * read-shadow != write-target). Linux likely sees the same
	 * readback but chip behavior reflects its writes.
	 *
	 * Worse: adding the explicit bank-switch + writes REGRESSED
	 * connection (chip stuck SCANNING/4WAY, never COMPLETED).
	 * So the RF[0xEE] bank-switch likely puts the chip in a state
	 * our subsequent phy_set_param doesn't restore correctly.
	 *
	 * Conclusion: our canary read is unreliable for RF[0x78]/RF[0x7D].
	 * Don't trust it as a diagnostic. Not the TX-load wedge cause.
	 */

	/*
	 * Apply the RFE-specific AGC BTG table when EFUSE rfe matches a
	 * BTG-mode RFE_DEF_EXT slot.  For 8821C: rfe=2 -> btg=2 ->
	 * rtw8821c_agc_btg_type2_tbl.  Without this the chip's BTG-mode
	 * AGC LUT is unpopulated and the firmware's DM authority over
	 * RF state becomes non-canonical, overriding our RF table writes
	 * at e.g. RF[0x7d], RF[0x78], RF[0xc7].  Mirror of
	 * rtw_phy_load_tables (phy.c:1879): walks the conditional table
	 * after the main AGC and before RF.  Our blob walker already
	 * processed RF_A before this point (blob order is fixed); to
	 * keep the agc_btg-before-rf_a ordering Linux uses, re-fire the
	 * RF_A walk from the blob right after agc_btg.
	 */
	if (sc->sc_rfe_option == 2 || sc->sc_rfe_option == 4) {
		uint32_t agc_btg_applied = 0;

		error = rtw88_phy_walk_table(sc, RTW88_TBL_KIND_BB,
		    rtw88_8821c_agc_btg_type2,
		    RTW88_8821C_AGC_BTG_TYPE2_NUM, &agc_btg_applied);
		if (error != 0) {
			device_printf(sc->sc_dev,
			    "agc_btg_type2 walk failed: %d\n", error);
			goto out;
		}
		device_printf(sc->sc_dev,
		    "tbl AGC_BTG_type2: %u words, %u writes applied\n",
		    RTW88_8821C_AGC_BTG_TYPE2_NUM, agc_btg_applied);

		/*
		 * Re-fire RF_A walk so RF writes land AFTER agc_btg
		 * configures the chip's BTG-mode AGC LUT.  Matches
		 * Linux's rtw_phy_load_tables order.
		 */
		if ((error = rtw88_phy_replay_rf_a(sc)) != 0)
			goto out;
	}

	/*
	 * Crystal cap calibration.  EFUSE xtal_k goes into REG_AFE_XTAL_CTRL
	 * bits 30:25 (0x7E000000) and REG_AFE_PLL_CTRL bits 6:1 (0x7E).
	 * Then clear bits 18 and 22 of REG_CCK0_FAREPORT (PHY init prep).
	 */
	if ((error = rtw88_usb_read4(sc, REG_AFE_XTAL_CTRL, &val32)) != 0)
		goto out;
	val32 = (val32 & ~BIT_MASK_XTAL_CAP) |
	    (((uint32_t)sc->sc_crystal_cap << 25) & BIT_MASK_XTAL_CAP);
	if ((error = rtw88_usb_write4(sc, REG_AFE_XTAL_CTRL, val32)) != 0)
		goto out;
	if ((error = rtw88_usb_read4(sc, REG_AFE_PLL_CTRL, &val32)) != 0)
		goto out;
	val32 = (val32 & ~BIT_MASK_PLL_CAP) |
	    (((uint32_t)sc->sc_crystal_cap << 1) & BIT_MASK_PLL_CAP);
	if ((error = rtw88_usb_write4(sc, REG_AFE_PLL_CTRL, val32)) != 0)
		goto out;
	if ((error = rtw88_usb_read4(sc, REG_CCK0_FAREPORT, &val32)) != 0)
		goto out;
	val32 &= ~((1U << 18) | (1U << 22));
	if ((error = rtw88_usb_write4(sc, REG_CCK0_FAREPORT, val32)) != 0)
		goto out;

	/* Post-table: assert RXPSEL reset to commit. */
	if ((error = rtw88_usb_read4(sc, REG_RXPSEL, &val32)) != 0)
		goto out;
	val32 |= BIT_RX_PSEL_RST;
	if ((error = rtw88_usb_write4(sc, REG_RXPSEL, val32)) != 0)
		goto out;

	/*
	 * Snapshot the post-init TXSF2 / TXSF6 / TXFILTER values for
	 * the BB per-channel writer.  reference stores these in hal->ch_param
	 * the same way -- they're the "baseline" CCK TX filter words
	 * that set_channel_bb composes back in for non-channel-14 cases.
	 */
	if ((error = rtw88_usb_read4(sc, REG_TXSF2, &sc->sc_ch_param[0])) != 0)
		goto out;
	if ((error = rtw88_usb_read4(sc, REG_TXSF6, &sc->sc_ch_param[1])) != 0)
		goto out;
	if ((error = rtw88_usb_read4(sc, REG_TXFILTER,
	    &sc->sc_ch_param[2])) != 0)
		goto out;

	/*
	 * Derive hal->rfe_btg from EFUSE rfe_option exactly like
	 * reference rtw8821c_read_efuse(): rfe in {2,4,7,a,c,f} -> BTG.
	 */
	switch (sc->sc_rfe_option) {
	case 0x02:
	case 0x04:
	case 0x07:
	case 0x0A:
	case 0x0C:
	case 0x0F:
		sc->sc_rfe_btg = true;
		break;
	default:
		sc->sc_rfe_btg = false;
		break;
	}

	device_printf(sc->sc_dev,
	    "phy_set_param: complete (xtal_k=0x%02x, rfe=0x%02x, cut=%u,"
	    " btg=%u, ch_param=[0x%08x, 0x%08x, 0x%08x])\n",
	    sc->sc_crystal_cap, sc->sc_rfe_option, sc->sc_cut_version,
	    (unsigned)sc->sc_rfe_btg,
	    sc->sc_ch_param[0], sc->sc_ch_param[1], sc->sc_ch_param[2]);

	/*
	 * Diagnostic checkpoint: read RF[0x7d] right after phy_set_param
	 * completes to determine if the walker's write of 0x07600 is
	 * still in the chip's RF shadow at this stage.  This narrows
	 * down where the non-latch happens.
	 */
	{
		uint32_t v_7d, v_78, v_c7;
		(void)rtw88_phy_read_rf_a(sc, 0x7d, RTW88_RFREG_MASK, &v_7d);
		(void)rtw88_phy_read_rf_a(sc, 0x78, RTW88_RFREG_MASK, &v_78);
		(void)rtw88_phy_read_rf_a(sc, 0xc7, RTW88_RFREG_MASK, &v_c7);
		device_printf(sc->sc_dev,
		    "post-phy_set_param canary: RF[0x7d]=0x%05x"
		    " RF[0x78]=0x%05x RF[0xc7]=0x%05x\n",
		    v_7d, v_78, v_c7);
	}

	rtw88_dig_start(sc);

	/* BF/MU-MIMO PHY init -- Linux fires this as the LAST step of
	 * rtw8821c_phy_set_param.  Touches 0x06DC / 0x14C0 / 0x167C /
	 * 0x042C via RMW so chip-default bits survive. */
	rtw88_bf_phy_init(sc);
out:
	RTW88_UNLOCK(sc);
	if (error != 0)
		device_printf(sc->sc_dev,
		    "phy_set_param failed: %d\n", error);
	return (error);
}

/*
 * peek/poke sysctls -- arbitrary EP0 vendor-request read/write
 * from userspace.  Pattern from feedback_mmio_range_monitor.md.
 * Set peek_addr first, then read peek_value to fetch the 4-byte
 * value at that address.  poke_value (write) issues a 4-byte
 * write to the current peek_addr.  Userspace shell scripts the
 * iteration -- ~1 ms per sysctl = ~4 s to dump a 4 KB range.
 */
static int
rtw88_peek_value_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct rtw88_usb_softc *sc = arg1;
	uint32_t val = 0;
	int error;

	RTW88_LOCK(sc);
	(void)rtw88_usb_read4(sc, sc->sc_peek_addr, &val);
	RTW88_UNLOCK(sc);
	error = sysctl_handle_int(oidp, &val, 0, req);
	return (error);
}

static int
rtw88_poke_value_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct rtw88_usb_softc *sc = arg1;
	uint32_t val = 0;
	int error;

	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	RTW88_LOCK(sc);
	(void)rtw88_usb_write4(sc, sc->sc_peek_addr, val);
	RTW88_UNLOCK(sc);
	return (0);
}

/*
 * Dump CAM slot N to dmesg.  Userspace writes the slot number; we
 * read all 8 dwords via REG_CAMREAD and decode:
 *   d0[1:0]  = keyidx
 *   d0[4:2]  = hw_key_type (4 = AES/CCMP)
 *   d0[6]    = group flag
 *   d0[15]   = valid
 *   d0[31:16]= mac[0..1]
 *   d1[31:0] = mac[2..5]
 *   d2..d5   = TK (16 bytes)
 *   d6,d7    = extended key (0 for AES)
 * Reviewer-requested 2026-06-26 hyp C validation (per-MACID CAM linkage):
 * confirms PTK lands in slot 4 with peer=AP-MAC, valid=1, group=0.
 */
static int
rtw88_dump_cam_slot_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct rtw88_usb_softc *sc = arg1;
	uint32_t val = 0;
	uint32_t d[8];
	uint8_t slot, keyidx, type, mac[6];
	bool group, valid;
	int error, i;

	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (val > 31)
		return (EINVAL);
	slot = (uint8_t)val;

	RTW88_LOCK(sc);
	for (i = 0; i < 8; i++) {
		if ((error = rtw88_cam_read_dword(sc, slot, (uint8_t)i,
		    &d[i])) != 0) {
			RTW88_UNLOCK(sc);
			device_printf(sc->sc_dev,
			    "cam_dump slot=%u dword=%d read failed: %d\n",
			    slot, i, error);
			return (0);
		}
	}
	RTW88_UNLOCK(sc);

	keyidx = (uint8_t)(d[0] & 0x3);
	type = (uint8_t)((d[0] >> 2) & 0x7);
	group = ((d[0] >> 6) & 0x1) != 0;
	valid = ((d[0] >> 15) & 0x1) != 0;
	mac[0] = (uint8_t)(d[0] >> 16);
	mac[1] = (uint8_t)(d[0] >> 24);
	mac[2] = (uint8_t)(d[1]);
	mac[3] = (uint8_t)(d[1] >> 8);
	mac[4] = (uint8_t)(d[1] >> 16);
	mac[5] = (uint8_t)(d[1] >> 24);

	device_printf(sc->sc_dev,
	    "cam_dump slot=%u valid=%d group=%d type=%u keyidx=%u "
	    "mac=%02x:%02x:%02x:%02x:%02x:%02x "
	    "key=%08x %08x %08x %08x ext=%08x %08x\n",
	    slot, valid, group, type, keyidx,
	    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
	    d[2], d[3], d[4], d[5], d[6], d[7]);
	return (0);
}

/*
 * Re-run phy_set_param to mimic Linux's IPS-wake chip re-init.  Kept
 * as a debug sysctl after the AUTH-wall investigation (2026-06-21):
 * if the chip-side PHY state ever regresses again, an operator can
 * force a re-run without unloading the module.
 */
static int
rtw88_reinit_phy_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct rtw88_usb_softc *sc = arg1;
	uint32_t val = 0;
	int error;

	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	(void)rtw88_phy_set_param(sc);
	return (0);
}

/*
 * LPS control sysctl.  Userspace writes 0 = ACTIVE, 1..15 = LPS with
 * the given awake_interval in beacons.  Round-trips through the same
 * SET_PWR_MODE H2C the fwka task uses to keep ACTIVE.  Substrate-only
 * today (no idle-timer auto-entry); a tuning operator can park the
 * chip in LPS via "sysctl dev.rtw88_usb.0.lps_state=2" during idle.
 */
static int
rtw88_lps_state_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct rtw88_usb_softc *sc = arg1;
	uint32_t val = 0;
	int error;

	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (val > 15)
		return (EINVAL);
	RTW88_LOCK(sc);
	if (val == 0)
		(void)rtw88_h2c_pwr_mode_active(sc);
	else
		(void)rtw88_h2c_pwr_mode_lps(sc, (uint8_t)val);
	RTW88_UNLOCK(sc);
	return (0);
}

/*
 * Reset all dev.rtw88_usb.N.stats.* counters to zero.  Useful for an
 * operator who wants to delineate an observation window (e.g., before
 * an `iperf` run) without unloading the driver.  Locks sc_mtx to
 * serialise against the RX/TX hot paths that increment counters; the
 * writes are 64-bit on 64-bit FreeBSD so individual stores are atomic,
 * but bulk-clearing under the lock keeps the window consistent.
 */
static int
rtw88_stats_reset_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct rtw88_usb_softc *sc = arg1;
	uint32_t val = 0;
	int error;

	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);

	RTW88_LOCK(sc);
	sc->sc_stat_tx_packets = 0;
	sc->sc_stat_tx_drops = 0;
	sc->sc_stat_tx_errors = 0;
	sc->sc_stat_tx_acked = 0;
	sc->sc_stat_tx_xretries = 0;
	sc->sc_rx_packets = 0;
	sc->sc_rx_frames = 0;
	sc->sc_stat_rx_errors = 0;
	sc->sc_stat_rx_drops = 0;
	sc->sc_stat_rx_cck = 0;
	sc->sc_stat_rx_ofdm = 0;
	sc->sc_stat_rx_mcs = 0;
	sc->sc_stat_usb_errors = 0;
	sc->sc_stat_cam_fails = 0;
	sc->sc_stat_fw_stalls = 0;
	sc->sc_stat_c2h = 0;
	sc->sc_stat_c2h_unknown = 0;
	sc->sc_stat_tx_data_qlen_max = 0;
	sc->sc_stat_tx_mgmt_qlen_max = 0;
	sc->sc_stat_tx_pending_qlen_max = 0;
	RTW88_UNLOCK(sc);
	return (0);
}

/*
 * Live mbufq depth read.  Helps an operator see whether TX is keeping
 * up with offered load or whether one of the queues is backing up
 * against the qdepth tunable.  Read-only, no lock — mbufq_len is a
 * 32-bit field updated under the queue's internal mutex and a torn
 * read at worst shows a 1-frame-old value.
 */
static int
rtw88_qlen_data_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct rtw88_usb_softc *sc = arg1;
	uint32_t v = mbufq_len(&sc->sc_tx_data_q);
	return (sysctl_handle_int(oidp, &v, 0, req));
}

static int
rtw88_qlen_mgmt_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct rtw88_usb_softc *sc = arg1;
	uint32_t v = mbufq_len(&sc->sc_tx_mgmt_q);
	return (sysctl_handle_int(oidp, &v, 0, req));
}

static int
rtw88_qlen_pending_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct rtw88_usb_softc *sc = arg1;
	uint32_t v = mbufq_len(&sc->sc_tx_pending_q);
	return (sysctl_handle_int(oidp, &v, 0, req));
}

/*
 * SIPI peek -- userspace sets rf_addr (8-bit RF register index),
 * then reads rf_value to fetch the masked RFREG_MASK value via
 * the direct-MMIO path A (rf_base_addr 0x2800 + (addr << 2)).
 * Companion to peek_addr/peek_value; lets us dump all 64 RF_A
 * registers from shell for the Linux-vs-FreeBSD diff at AUTH-time.
 */
static int
rtw88_rf_value_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct rtw88_usb_softc *sc = arg1;
	uint32_t val = 0;
	int error;

	RTW88_LOCK(sc);
	(void)rtw88_phy_read_rf_a(sc, sc->sc_rf_addr, RTW88_RFREG_MASK, &val);
	RTW88_UNLOCK(sc);
	error = sysctl_handle_int(oidp, &val, 0, req);
	return (error);
}

/*
 * Stand up the ieee80211com so userspace can clone wlan0 onto
 * rtw88_usb0.  Called from attach() after firmware download succeeds.
 * MAC address comes from EFUSE if the read succeeded; otherwise falls
 * back to a locally-administered placeholder so the interface still
 * attaches.
 */
static int
rtw88_net80211_attach(struct rtw88_usb_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
	static const uint8_t fake_mac[IEEE80211_ADDR_LEN] = {
		0x02, 0x00, 0xde, 0xad, 0xbe, 0xef
	};
	const uint8_t *mac = sc->sc_have_mac ? sc->sc_mac : fake_mac;

	ic->ic_softc = sc;
	ic->ic_name = device_get_nameunit(sc->sc_dev);
	ic->ic_phytype = IEEE80211_T_OFDM;
	ic->ic_opmode = IEEE80211_M_STA;

	/*
	 * STA + MONITOR.  Monitor support: vap_create accepts the mode,
	 * radiotap headers are attached unconditionally below, and RX
	 * frames go up via ieee80211_input_mimo_all so any bpf listener
	 * (regardless of VAP mode) gets the per-frame metadata.
	 */
	ic->ic_caps = IEEE80211_C_STA |
	    IEEE80211_C_MONITOR |
	    IEEE80211_C_SHPREAMBLE |
	    IEEE80211_C_SHSLOT |
	    IEEE80211_C_WPA |
	    IEEE80211_C_WME;

	/*
	 * Declare HT (802.11n) capabilities so net80211 accepts an AP's
	 * HT IEs in assoc-resp.  RTL8821CU is 1T1R 802.11ac, so 1 TX
	 * + 1 RX stream, 20 MHz BW, short GI 20.  SMPS off because the
	 * chip is single-stream.
	 *
	 * IEEE80211_HTC_AMPDU gated on hw.rtw88_usb.ampdu.  When off,
	 * net80211 doesn't negotiate BA sessions and the chip TXes single
	 * MPDUs.  When on, net80211 uses its default SW BA negotiation
	 * (ieee80211_addba_request/response/stop) and the driver tags TX
	 * desc AGG_EN + sets M_AMPDU on RX so net80211's SW reorder buffer
	 * engages.  No chip-side reorder programming needed for the SW
	 * reorder path.
	 */
	if (sc->sc_chip->caps & RTW88_CAP_HT20) {
		ic->ic_htcaps =
		    IEEE80211_HTC_HT |
		    IEEE80211_HTCAP_SHORTGI20 |
		    IEEE80211_HTCAP_SMPS_OFF |
		    IEEE80211_HTCAP_MAXAMSDU_3839 |
		    (4u << IEEE80211_HTCAP_MPDUDENSITY_S);
		if (sc->sc_chip->caps & RTW88_CAP_HT40)
			ic->ic_htcaps |=
			    IEEE80211_HTCAP_CHWIDTH40 |
			    IEEE80211_HTCAP_SHORTGI40;
		if (sc->sc_ampdu)
			ic->ic_htcaps |= IEEE80211_HTC_AMPDU;
	}

	/*
	 * 802.11ac VHT 80 MHz on 5 GHz, 1T1R, MCS0..9 single stream.
	 *   vht_cap_info: 11454-byte max MPDU, supported width = 80 only,
	 *                 SHORT_GI_80, RX_LDPC, TX_STBC, max-A-MPDU-len-exp 7.
	 *   supp_mcs:     stream-0 MCS0-9 (= 2), streams 1..7 not supported (= 3).
	 */
	if ((sc->sc_chip->caps & RTW88_CAP_VHT80) &&
	    (sc->sc_chip->caps & RTW88_CAP_5GHZ)) {
		ic->ic_flags_ext |= IEEE80211_FEXT_VHT;
		ic->ic_vht_cap.vht_cap_info =
		    IEEE80211_VHTCAP_MAX_MPDU_LENGTH_11454 |
		    IEEE80211_VHTCAP_SHORT_GI_80 |
		    IEEE80211_VHTCAP_RXLDPC |
		    IEEE80211_VHTCAP_TXSTBC |
		    (7u << IEEE80211_VHTCAP_MAX_A_MPDU_LENGTH_EXPONENT_SHIFT);
		ic->ic_vht_cap.supp_mcs.rx_mcs_map = htole16(0xfffeU);
		ic->ic_vht_cap.supp_mcs.tx_mcs_map = htole16(0xfffeU);
		ic->ic_vht_cap.supp_mcs.rx_highest = htole16(0);
		ic->ic_vht_cap.supp_mcs.tx_highest = htole16(0);
		ic->ic_vht_flags = IEEE80211_FVHT_VHT | IEEE80211_FVHT_USEVHT80;
	}
	ic->ic_txstream = sc->sc_chip->n_tx_chains;
	ic->ic_rxstream = sc->sc_chip->n_rx_chains;

	/*
	 * AES_CCM advertised in ic_cryptocaps only under hw_ccmp=1.  When
	 * a cipher is absent from ic_cryptocaps, net80211's crypto layer
	 * sets IEEE80211_KEY_SWCRYPT on keys of that cipher (see
	 * ieee80211_crypto.c) and SW-encrypts the body itself.  Under
	 * hw_ccmp=0 (default) we want that: the chip's security engine is
	 * off (REG_SEC_CONFIG=0) and net80211 produces a fully-formed
	 * CCMP frame the AP can decrypt without chip help.  Under hw_ccmp=1
	 * the chip's CCMP engine encrypts via CAM slot 4.
	 */
	ic->ic_cryptocaps = IEEE80211_CRYPTO_WEP |
	    IEEE80211_CRYPTO_TKIP;
	if (sc->sc_hw_ccmp)
		ic->ic_cryptocaps |= IEEE80211_CRYPTO_AES_CCM;

	IEEE80211_ADDR_COPY(ic->ic_macaddr, mac);

	rtw88_getradiocaps(ic, IEEE80211_CHAN_MAX, &ic->ic_nchans,
	    ic->ic_channels);

	ieee80211_ifattach(ic);

	ieee80211_radiotap_attach(ic,
	    &sc->sc_txtap.wt_ihdr, sizeof(sc->sc_txtap),
	    RTW88_TX_RADIOTAP_PRESENT,
	    &sc->sc_rxtap.wr_ihdr, sizeof(sc->sc_rxtap),
	    RTW88_RX_RADIOTAP_PRESENT);

	ic->ic_raw_xmit = rtw88_raw_xmit;
	ic->ic_scan_start = rtw88_scan_start;
	ic->ic_scan_end = rtw88_scan_end;
	ic->ic_getradiocaps = rtw88_getradiocaps;
	ic->ic_set_channel = rtw88_set_channel;
	ic->ic_vap_create = rtw88_vap_create;
	ic->ic_vap_delete = rtw88_vap_delete;
	ic->ic_parent = rtw88_parent;
	ic->ic_transmit = rtw88_transmit;
	ic->ic_wme.wme_update = rtw88_wme_update;
	ic->ic_update_promisc = rtw88_update_promisc;
	ic->ic_update_mcast = rtw88_update_mcast;
	ic->ic_update_chw = rtw88_set_channel;
	/*
	 * Leave ic_send_mgmt at the net80211 default
	 * (ieee80211_send_mgmt).  That's the function which actually
	 * builds AUTH/ASSOC_REQ/etc. frames and hands them to
	 * ic_raw_xmit -- replacing it with a stub that returns ENOTSUP
	 * silently drops every mgmt frame net80211 wants to originate,
	 * which manifests as "SCAN -> AUTH -> INIT" instantly because
	 * the AUTH_REQ never goes out on the air.
	 */
	ic->ic_newassoc = rtw88_newassoc;

	/*
	 * WEP/TKIP always use the SW path (no chip-CAM port for those
	 * ciphers).  AES_CCM is SW under hw_ccmp=0 (default) and HW under
	 * hw_ccmp=1, decided at attach time via the sc_hw_ccmp latch.
	 *
	 * Trivia: HW-path AES_CCM lets the firmware's KEEP_ALIVE NULL frame
	 * arrive at the AP correctly CCMP-encrypted, which the
	 * software-encrypt path could never deliver.
	 */
	{
		uint32_t sw_ciphers = IEEE80211_CRYPTO_WEP |
		    IEEE80211_CRYPTO_TKIP;
		if (!sc->sc_hw_ccmp)
			sw_ciphers |= IEEE80211_CRYPTO_AES_CCM;
		ieee80211_set_software_ciphers(ic, sw_ciphers);
	}

	if (bootverbose)
		ieee80211_announce(ic);

	{
		struct sysctl_ctx_list *ctx = device_get_sysctl_ctx(sc->sc_dev);
		struct sysctl_oid *tree = device_get_sysctl_tree(sc->sc_dev);
		struct sysctl_oid *stree;

		SYSCTL_ADD_STRING(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "version", CTLFLAG_RD, __DECONST(char *,
		    RTW88_DRIVER_VERSION), 0,
		    "rtw88_usb driver version");
		SYSCTL_ADD_STRING(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "chip_name", CTLFLAG_RD, __DECONST(char *,
		    sc->sc_chip->name), 0,
		    "chip family (per chip-spec table)");
		SYSCTL_ADD_U16(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "chip_id", CTLFLAG_RD,
		    __DECONST(uint16_t *, &sc->sc_chip->chip_id), 0,
		    "chip id (matches firmware-header signature)");
		SYSCTL_ADD_U16(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "chip_caps", CTLFLAG_RD,
		    __DECONST(uint16_t *, &sc->sc_chip->caps), 0,
		    "OR mask of RTW88_CAP_* bits the driver supports for "
		    "this chip (0x001=HT20 0x008=2GHZ 0x010=5GHZ ...)");

		/*
		 * dev.rtw88_usb.N.stats.* — non-debug operator counters.  All
		 * read-only, all bumped in their respective error paths.  Lets
		 * a stuck-link postmortem proceed without enabling tracing.
		 */
		stree = SYSCTL_ADD_NODE(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "stats", CTLFLAG_RD | CTLFLAG_MPSAFE, NULL,
		    "production stats counters");
		SYSCTL_ADD_U64(ctx, SYSCTL_CHILDREN(stree), OID_AUTO,
		    "tx_packets", CTLFLAG_RD, &sc->sc_stat_tx_packets, 0,
		    "TX mbufs successfully handed to the chip");
		SYSCTL_ADD_U64(ctx, SYSCTL_CHILDREN(stree), OID_AUTO,
		    "tx_drops", CTLFLAG_RD, &sc->sc_stat_tx_drops, 0,
		    "TX mbufs dropped at enqueue (queue full / oversize)");
		SYSCTL_ADD_U64(ctx, SYSCTL_CHILDREN(stree), OID_AUTO,
		    "tx_errors", CTLFLAG_RD, &sc->sc_stat_tx_errors, 0,
		    "TX USB xfer errors reported by the host controller");
		SYSCTL_ADD_U64(ctx, SYSCTL_CHILDREN(stree), OID_AUTO,
		    "rx_packets", CTLFLAG_RD, &sc->sc_rx_packets, 0,
		    "RX bulk-IN completions since attach");
		SYSCTL_ADD_U64(ctx, SYSCTL_CHILDREN(stree), OID_AUTO,
		    "rx_frames", CTLFLAG_RD, &sc->sc_rx_frames, 0,
		    "RX mbufs handed up to net80211");
		SYSCTL_ADD_U64(ctx, SYSCTL_CHILDREN(stree), OID_AUTO,
		    "rx_errors", CTLFLAG_RD, &sc->sc_stat_rx_errors, 0,
		    "RX frames with CRC or ICV error in the descriptor");
		SYSCTL_ADD_U64(ctx, SYSCTL_CHILDREN(stree), OID_AUTO,
		    "rx_drops", CTLFLAG_RD, &sc->sc_stat_rx_drops, 0,
		    "RX frames dropped (mbuf alloc fail / oversize)");
		SYSCTL_ADD_U64(ctx, SYSCTL_CHILDREN(stree), OID_AUTO,
		    "usb_errors", CTLFLAG_RD, &sc->sc_stat_usb_errors, 0,
		    "EP0 vendor-request read/write failures");
		SYSCTL_ADD_U64(ctx, SYSCTL_CHILDREN(stree), OID_AUTO,
		    "cam_fails", CTLFLAG_RD, &sc->sc_stat_cam_fails, 0,
		    "iv_key_set queue overflow (HW CAM install dropped)");
		SYSCTL_ADD_U64(ctx, SYSCTL_CHILDREN(stree), OID_AUTO,
		    "fw_stalls", CTLFLAG_RD, &sc->sc_stat_fw_stalls, 0,
		    "watchdog detected firmware stalled (heartbeat frozen)");
		SYSCTL_ADD_U64(ctx, SYSCTL_CHILDREN(stree), OID_AUTO,
		    "c2h", CTLFLAG_RD, &sc->sc_stat_c2h, 0,
		    "C2H firmware-to-host events received");
		SYSCTL_ADD_U64(ctx, SYSCTL_CHILDREN(stree), OID_AUTO,
		    "c2h_unknown", CTLFLAG_RD, &sc->sc_stat_c2h_unknown, 0,
		    "C2H events with unrecognised sub-cmd id");
		SYSCTL_ADD_U64(ctx, SYSCTL_CHILDREN(stree), OID_AUTO,
		    "rx_cck", CTLFLAG_RD, &sc->sc_stat_rx_cck, 0,
		    "RX frames at CCK rates (1/2/5.5/11 Mbps)");
		SYSCTL_ADD_U64(ctx, SYSCTL_CHILDREN(stree), OID_AUTO,
		    "rx_ofdm", CTLFLAG_RD, &sc->sc_stat_rx_ofdm, 0,
		    "RX frames at OFDM rates (6..54 Mbps)");
		SYSCTL_ADD_U64(ctx, SYSCTL_CHILDREN(stree), OID_AUTO,
		    "rx_mcs", CTLFLAG_RD, &sc->sc_stat_rx_mcs, 0,
		    "RX frames at HT MCS rates");
		SYSCTL_ADD_U8(ctx, SYSCTL_CHILDREN(stree), OID_AUTO,
		    "link_rssi", CTLFLAG_RD, &sc->sc_avg_rssi, 0,
		    "EWMA of link RSSI, Linux convention "
		    "(signal_dbm + 100, range 0..100); 0 = no sample yet");
		SYSCTL_ADD_U64(ctx, SYSCTL_CHILDREN(stree), OID_AUTO,
		    "tx_acked", CTLFLAG_RD, &sc->sc_stat_tx_acked, 0,
		    "CCX TX reports with success status from firmware");
		SYSCTL_ADD_U64(ctx, SYSCTL_CHILDREN(stree), OID_AUTO,
		    "tx_xretries", CTLFLAG_RD, &sc->sc_stat_tx_xretries, 0,
		    "CCX TX reports with retry-exhausted status from firmware");
		SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(stree), OID_AUTO,
		    "reset",
		    CTLTYPE_UINT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
		    sc, 0, rtw88_stats_reset_sysctl, "IU",
		    "write any value to zero all stats counters");
		SYSCTL_ADD_U8(ctx, SYSCTL_CHILDREN(stree), OID_AUTO,
		    "rate_id", CTLFLAG_RD, &sc->sc_rate_id, 0,
		    "current RA rate_id (3=BGN_20M_1SS, 6=BG, 5=GN_N1SS, 7=G)");
		SYSCTL_ADD_U8(ctx, SYSCTL_CHILDREN(stree), OID_AUTO,
		    "channel", CTLFLAG_RD, &sc->sc_cur_channel, 0,
		    "chip-tuned channel number");
		SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(stree), OID_AUTO,
		    "tx_data_qlen",
		    CTLTYPE_UINT | CTLFLAG_RD | CTLFLAG_MPSAFE,
		    sc, 0, rtw88_qlen_data_sysctl, "IU",
		    "TX data-frame mbufq backlog (capped by tx_data_qdepth)");
		SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(stree), OID_AUTO,
		    "tx_mgmt_qlen",
		    CTLTYPE_UINT | CTLFLAG_RD | CTLFLAG_MPSAFE,
		    sc, 0, rtw88_qlen_mgmt_sysctl, "IU",
		    "TX mgmt-frame mbufq backlog (capped by tx_mgmt_qdepth)");
		SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(stree), OID_AUTO,
		    "tx_pending_qlen",
		    CTLTYPE_UINT | CTLFLAG_RD | CTLFLAG_MPSAFE,
		    sc, 0, rtw88_qlen_pending_sysctl, "IU",
		    "protected-unicast frames held pending PTK install");
		SYSCTL_ADD_U32(ctx, SYSCTL_CHILDREN(stree), OID_AUTO,
		    "tx_data_qlen_max", CTLFLAG_RD,
		    &sc->sc_stat_tx_data_qlen_max, 0,
		    "high-watermark of tx_data_qlen since last stats.reset");
		SYSCTL_ADD_U32(ctx, SYSCTL_CHILDREN(stree), OID_AUTO,
		    "tx_mgmt_qlen_max", CTLFLAG_RD,
		    &sc->sc_stat_tx_mgmt_qlen_max, 0,
		    "high-watermark of tx_mgmt_qlen since last stats.reset");
		SYSCTL_ADD_U32(ctx, SYSCTL_CHILDREN(stree), OID_AUTO,
		    "tx_pending_qlen_max", CTLFLAG_RD,
		    &sc->sc_stat_tx_pending_qlen_max, 0,
		    "high-watermark of tx_pending_qlen since last stats.reset");

		SYSCTL_ADD_U16(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "peek_addr", CTLFLAG_RW, &sc->sc_peek_addr, 0,
		    "EP0 vendor-request addr for peek/poke");
		SYSCTL_ADD_U16(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "linkup_skip", CTLFLAG_RW, &sc->sc_linkup_skip, 0,
		    "bitfield to skip post-LINK_UP H2Cs for M4-loss bisect: "
		    "0x01=RA_INFO 0x02=MEDIA_STATUS 0x04=PWR_MODE 0x08=fwka "
		    "0x10=RSVD_PAGE 0x20=KEEP_ALIVE 0x40=DEFAULT_PORT "
		    "0x80=EDCA");
		SYSCTL_ADD_U8(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "linkup_iqk", CTLFLAG_RW, &sc->sc_linkup_iqk, 0,
		    "Tier 3c: re-run IQK at LINK_UP before H2C burst (0=off, "
		    "1=on).  Not Linux parity — hypothesis test for M3 wall. "
		    "Enabling stalls LINK_UP by up to 6 s.");
		SYSCTL_ADD_U8(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "m1_bcn_refire", CTLFLAG_RW, &sc->sc_m1_bcn_refire, 0,
		    "On M1 rx, re-fire REG_FIFOPAGE_CTRL_2 = pg_addr | "
		    "BCN_VALID_V1 (Linux-parity, reg_monitor.py "
		    "--handshake-diff finding 2026-07-02).  0=off, 1=on.");
		SYSCTL_ADD_UINT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "stat_m1_bcn_refire", CTLFLAG_RD,
		    &sc->sc_stat_m1_bcn_refire, 0,
		    "count of M1-rx-triggered BCN_VALID_V1 re-fires");
		SYSCTL_ADD_U8(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "eapol_qos_upgrade", CTLFLAG_RW,
		    &sc->sc_eapol_qos_upgrade, 0,
		    "Rewrite plain-Data EAPOL frames to QoS-Data TID=7 (VO) "
		    "in-place at TX time.  Matches Linux rtw88 OTA behavior; "
		    "net80211 strips QoS from EAPOL per legacy-AP workaround.  "
		    "0=off, 1=on.");
		SYSCTL_ADD_UINT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "stat_eapol_qos_upgrade", CTLFLAG_RD,
		    &sc->sc_stat_eapol_qos_upgrade, 0,
		    "count of EAPOL frames upgraded to QoS-Data");
		SYSCTL_ADD_U8(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "eapol_fullhex", CTLFLAG_RW, &sc->sc_eapol_fullhex, 0,
		    "Dump full M2/M4 tx mbuf as hex to dmesg for byte-for-byte "
		    "diff against Linux M2.  0=off, 1=on.");
		SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "peek_value",
		    CTLTYPE_UINT | CTLFLAG_RD | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
		    sc, 0, rtw88_peek_value_sysctl, "IU",
		    "4-byte read at peek_addr");
		SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "poke_value",
		    CTLTYPE_UINT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
		    sc, 0, rtw88_poke_value_sysctl, "IU",
		    "4-byte write at peek_addr");
		SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "dump_cam_slot",
		    CTLTYPE_UINT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
		    sc, 0, rtw88_dump_cam_slot_sysctl, "IU",
		    "dump CAM slot N to dmesg (valid/group/type/keyidx/mac/key)");
		SYSCTL_ADD_U8(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "rf_addr", CTLFLAG_RW, &sc->sc_rf_addr, 0,
		    "SIPI path-A RF register addr (0x00..0x3F)");
		SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "rf_value",
		    CTLTYPE_UINT | CTLFLAG_RD | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
		    sc, 0, rtw88_rf_value_sysctl, "IU",
		    "RFREG_MASK read at rf_addr (path A)");
		SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "reinit_phy",
		    CTLTYPE_UINT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
		    sc, 0, rtw88_reinit_phy_sysctl, "IU",
		    "re-run phy_set_param (IPS-wake equiv); write any value");
		SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "lps_state",
		    CTLTYPE_UINT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
		    sc, 0, rtw88_lps_state_sysctl, "IU",
		    "0 = ACTIVE, 1..15 = LPS with awake_interval=N beacons");
		SYSCTL_ADD_U32(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "rx_trace", CTLFLAG_RW, &sc->sc_rx_trace, 0,
		    "log the next N RX frames (w0/w3/fc0/da/sa)");
		SYSCTL_ADD_U32(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "c2h_trace", CTLFLAG_RW, &sc->sc_c2h_trace, 0,
		    "log the next N C2H packets raw (id/seq/payload[0..7])");
		SYSCTL_ADD_U32(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "tx_trace", CTLFLAG_RW, &sc->sc_tx_trace, 0,
		    "log the next N mgmt TX submissions (desc + 802.11 hdr)");
		SYSCTL_ADD_U32(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "write_trace", CTLFLAG_RW, &sc->sc_write_trace, 0,
		    "log the next N EP0 write_region calls with us timing");
		SYSCTL_ADD_U32(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "trace_mmio", CTLFLAG_RW, &sc->sc_trace_mmio, 0,
		    "1=emit W%d/R%d a=0x.. v=0x.. for every EP0 reg op "
		    "(linux-auth-trace format)");
	}

	if (rtw88_cdev_enable) {
		sc->sc_cdev = make_dev(&rtw88_cdevsw,
		    device_get_unit(sc->sc_dev),
		    UID_ROOT, GID_WHEEL, 0600, "%s%d", RTW88_USB_CDEV_NAME,
		    device_get_unit(sc->sc_dev));
		if (sc->sc_cdev == NULL)
			device_printf(sc->sc_dev,
			    "make_dev /dev/%s%d failed; RPC bridge "
			    "unavailable\n", RTW88_USB_CDEV_NAME,
			    device_get_unit(sc->sc_dev));
		else
			sc->sc_cdev->si_drv1 = sc;
	}

	return (0);
}

/*
 * Generic descriptor walker -- iterate every descriptor of `type` in the
 * active configuration and pass each to `cb`.  FreeBSD-native equivalent
 * of Linux's __usb_get_extra_descriptor pattern (walk unrecognised
 * extra-data bytes that follow standard descriptors).
 *
 * Returns the number of descriptors visited.  Cb receives the descriptor
 * head (caller casts to the appropriate struct based on `type`).  Returns
 * < 0 from cb to stop iteration early.
 */
static int
rtw88_walk_descriptors(struct rtw88_usb_softc *sc, uint8_t type,
    int (*cb)(struct rtw88_usb_softc *, const void *, void *),
    void *arg)
{
	void *desc = NULL;
	int visited = 0, ret;

	for (;;) {
		desc = usbd_find_descriptor(sc->sc_udev, desc, 0,
		    type, 0xff, 0, 0);
		if (desc == NULL)
			break;
		visited++;
		if (cb != NULL) {
			ret = cb(sc, desc, arg);
			if (ret < 0)
				break;
		}
	}
	return (visited);
}

/*
 * Walk the active USB interface's endpoint descriptors and record bulk
 * IN + bulk OUT addresses in the softc.  Uses only FreeBSD-native
 * usbd_find_descriptor; no LinuxKPI involvement.  Replaces the previous
 * hardcoded EP 0x05/0x06/0x08 assignment in the static usb_config[]
 * template -- chip variants and OEM rebadges may shuffle physical
 * endpoint addresses while still reporting the same Realtek VID/PID.
 *
 * Out endpoints are stored in encounter order (which on every variant
 * we've seen matches USB-descriptor ascending-address order).  The
 * chip's RQPN mapping for 8821C-family puts mgmt + H2C on the lowest
 * OUT EP, NORMAL FIFO on the next, LOW FIFO data on the highest -- so
 * the rqpn-equivalent picking in xfers_setup just indexes the array.
 */
static int
rtw88_discover_endpoints(struct rtw88_usb_softc *sc)
{
	struct usb_endpoint_descriptor *ed;
	void *desc = NULL;

	sc->sc_ep_bulk_in = 0;
	sc->sc_n_bulk_out = 0;
	memset(sc->sc_ep_bulk_out, 0, sizeof(sc->sc_ep_bulk_out));

	for (;;) {
		desc = usbd_find_descriptor(sc->sc_udev, desc, 0,
		    UDESC_ENDPOINT, 0xff, 0, 0);
		if (desc == NULL)
			break;
		ed = (struct usb_endpoint_descriptor *)desc;
		if ((ed->bmAttributes & UE_XFERTYPE) != UE_BULK)
			continue;
		if ((ed->bEndpointAddress & UE_DIR_IN) == UE_DIR_IN) {
			if (sc->sc_ep_bulk_in == 0)
				sc->sc_ep_bulk_in =
				    ed->bEndpointAddress & UE_ADDR;
		} else {
			if (sc->sc_n_bulk_out <
			    nitems(sc->sc_ep_bulk_out)) {
				sc->sc_ep_bulk_out[sc->sc_n_bulk_out++] =
				    ed->bEndpointAddress;
			}
		}
	}

	device_printf(sc->sc_dev,
	    "endpoints: bulk_in=0x%02x n_bulk_out=%u (0x%02x 0x%02x 0x%02x)\n",
	    sc->sc_ep_bulk_in, sc->sc_n_bulk_out,
	    sc->sc_ep_bulk_out[0], sc->sc_ep_bulk_out[1],
	    sc->sc_ep_bulk_out[2]);

	if (sc->sc_n_bulk_out < 1) {
		device_printf(sc->sc_dev,
		    "no bulk-OUT endpoints discovered; cannot attach\n");
		return (ENXIO);
	}
	return (0);
}

static int
rtw88_xfers_setup(struct rtw88_usb_softc *sc)
{
	struct usb_config cfg[RTW88_N_TRANSFER];
	uint8_t iface_index = 0;
	usb_error_t err;

	/*
	 * Build a runtime usb_config[] from the static template + the
	 * endpoint addresses we discovered earlier in
	 * rtw88_discover_endpoints.  Lets the same probe table match
	 * RTL8821CU / RTL8811CU and any OEM rebadge that shuffles
	 * physical EP addresses.  RX uses UE_ADDR_ANY (single IN bulk
	 * pipe -- framework finds it) but TX needs explicit assignment
	 * because the chip's TX scheduler is sensitive to EP-vs-QSEL
	 * pairing (memory: project_rtw88_eapol_rate_fix_2026_06_25).
	 */
	memcpy(cfg, rtw88_config, sizeof(cfg));
	if (sc->sc_n_bulk_out >= 1)
		cfg[RTW88_BULK_TX_HI].endpoint = sc->sc_ep_bulk_out[0];
	if (sc->sc_n_bulk_out >= 2)
		cfg[RTW88_BULK_TX_NORMAL].endpoint = sc->sc_ep_bulk_out[1];
	if (sc->sc_n_bulk_out >= 3)
		cfg[RTW88_BULK_TX_LO].endpoint = sc->sc_ep_bulk_out[2];
	/* H2C shares the HI pipe per Linux rtw88 rqpn_table[8821c]. */
	if (sc->sc_n_bulk_out >= 1)
		cfg[RTW88_BULK_TX_H2C].endpoint = sc->sc_ep_bulk_out[0];

	err = usbd_transfer_setup(sc->sc_udev, &iface_index, sc->sc_xfer,
	    cfg, RTW88_N_TRANSFER, sc, &sc->sc_mtx);
	if (err != USB_ERR_NORMAL_COMPLETION) {
		device_printf(sc->sc_dev,
		    "usbd_transfer_setup: %s\n", usbd_errstr(err));
		return (ENXIO);
	}

	device_printf(sc->sc_dev,
	    "bulk xfers ready: rx[0]=%p tx_hi[1]=%p tx_normal[2]=%p"
	    " tx_lo[3]=%p tx_h2c[4]=%p (out_eps=%u/%u/%u)\n",
	    sc->sc_xfer[RTW88_BULK_RX], sc->sc_xfer[RTW88_BULK_TX_HI],
	    sc->sc_xfer[RTW88_BULK_TX_NORMAL], sc->sc_xfer[RTW88_BULK_TX_LO],
	    sc->sc_xfer[RTW88_BULK_TX_H2C],
	    sc->sc_ep_bulk_out[0], sc->sc_ep_bulk_out[1],
	    sc->sc_ep_bulk_out[2]);

	/*
	 * Arm the RX pipe so the framework can start receiving as soon as
	 * the chip's firmware boot completes.  TX pipes stay idle until
	 * net80211 hands the driver a frame.
	 */
	RTW88_LOCK(sc);
	usbd_transfer_start(sc->sc_xfer[RTW88_BULK_RX]);
	RTW88_UNLOCK(sc);

	return (0);
}

static void
rtw88_xfers_teardown(struct rtw88_usb_softc *sc)
{

	usbd_transfer_unsetup(sc->sc_xfer, RTW88_N_TRANSFER);
}

/*
 * cdev for /dev/rtw88_usb0 -- RPC bridge primitives.  Two ioctls
 * (read / write) move one 1, 2, or 4-byte register access per call.
 * Caller must be root (mode 0600 below).  The driver lock is taken
 * around each access so concurrent ioctl-driven and net80211-driven
 * EP0 traffic don't interleave on the USB control endpoint.
 */
static int
rtw88_cdev_ioctl(struct cdev *cdev, u_long ioc, caddr_t arg,
    int flag __unused, struct thread *td __unused)
{
	struct rtw88_usb_softc *sc = cdev->si_drv1;
	struct rtw88_cdev_reg *r = (struct rtw88_cdev_reg *)arg;
	int err = 0;

	if (sc == NULL)
		return (ENXIO);
	if (ioc != RTW88_CDEV_IOC_READ && ioc != RTW88_CDEV_IOC_WRITE)
		return (ENOTTY);
	if (r->width != 1 && r->width != 2 && r->width != 4)
		return (EINVAL);

	RTW88_LOCK(sc);
	if (ioc == RTW88_CDEV_IOC_READ) {
		uint8_t v8;
		uint16_t v16;
		uint32_t v32;

		switch (r->width) {
		case 1:
			err = rtw88_usb_read1(sc, r->addr, &v8);
			r->value = v8;
			break;
		case 2:
			err = rtw88_usb_read2(sc, r->addr, &v16);
			r->value = v16;
			break;
		case 4:
			err = rtw88_usb_read4(sc, r->addr, &v32);
			r->value = v32;
			break;
		}
	} else {
		switch (r->width) {
		case 1:
			err = rtw88_usb_write1(sc, r->addr,
			    (uint8_t)r->value);
			break;
		case 2:
			err = rtw88_usb_write2(sc, r->addr,
			    (uint16_t)r->value);
			break;
		case 4:
			err = rtw88_usb_write4(sc, r->addr, r->value);
			break;
		}
	}
	RTW88_UNLOCK(sc);
	r->err = err;
	return (0);
}

static struct cdevsw rtw88_cdevsw = {
	.d_version	= D_VERSION,
	.d_ioctl	= rtw88_cdev_ioctl,
	.d_name		= "rtw88_usb",
};

/* ------------------------------------------------------------------ */
/* Bus-abstract HCI ops: USB back-end wrappers.                        */
/*                                                                     */
/* These convert the abstract `struct rtw88_dev *d` (used by the      */
/* subsystem TUs) back to the USB softc via `d->priv`, then delegate  */
/* to the existing rtw88_usb_read/write/h2c helpers.                  */
/* ------------------------------------------------------------------ */
static int
rtw88_usb_hci_read8(struct rtw88_dev *d, uint32_t a, uint8_t *v)
{
	return (rtw88_usb_read1((struct rtw88_usb_softc *)d->priv,
	    (uint16_t)a, v));
}
static int
rtw88_usb_hci_read16(struct rtw88_dev *d, uint32_t a, uint16_t *v)
{
	return (rtw88_usb_read2((struct rtw88_usb_softc *)d->priv,
	    (uint16_t)a, v));
}
static int
rtw88_usb_hci_read32(struct rtw88_dev *d, uint32_t a, uint32_t *v)
{
	return (rtw88_usb_read4((struct rtw88_usb_softc *)d->priv,
	    (uint16_t)a, v));
}
static int
rtw88_usb_hci_write8(struct rtw88_dev *d, uint32_t a, uint8_t v)
{
	return (rtw88_usb_write1((struct rtw88_usb_softc *)d->priv,
	    (uint16_t)a, v));
}
static int
rtw88_usb_hci_write16(struct rtw88_dev *d, uint32_t a, uint16_t v)
{
	return (rtw88_usb_write2((struct rtw88_usb_softc *)d->priv,
	    (uint16_t)a, v));
}
static int
rtw88_usb_hci_write32(struct rtw88_dev *d, uint32_t a, uint32_t v)
{
	return (rtw88_usb_write4((struct rtw88_usb_softc *)d->priv,
	    (uint16_t)a, v));
}
static int
rtw88_usb_hci_read_region(struct rtw88_dev *d, uint32_t a, void *b,
    uint32_t n)
{
	return (rtw88_usb_read_region((struct rtw88_usb_softc *)d->priv,
	    (uint16_t)a, b, (uint16_t)n));
}
static int
rtw88_usb_hci_write_region(struct rtw88_dev *d, uint32_t a, const void *b,
    uint32_t n)
{
	return (rtw88_usb_write_region((struct rtw88_usb_softc *)d->priv,
	    (uint16_t)a, b, (uint16_t)n));
}
static int
rtw88_usb_hci_h2c(struct rtw88_dev *d, uint8_t cmd __unused, uint32_t msg,
    uint32_t msg_ext)
{
	return (rtw88_h2c_mailbox((struct rtw88_usb_softc *)d->priv,
	    msg, msg_ext));
}

static const struct rtw88_hci_ops rtw88_usb_hci_ops = {
	.read8		= rtw88_usb_hci_read8,
	.read16		= rtw88_usb_hci_read16,
	.read32		= rtw88_usb_hci_read32,
	.write8		= rtw88_usb_hci_write8,
	.write16	= rtw88_usb_hci_write16,
	.write32	= rtw88_usb_hci_write32,
	.read_region	= rtw88_usb_hci_read_region,
	.write_region	= rtw88_usb_hci_write_region,
	.h2c		= rtw88_usb_hci_h2c,
};

/* ------------------------------------------------------------------ */
/* Chip capability table for RTL8821C family (8821CU / 8811CU).        */
/*                                                                     */
/* Consumed by the subsystem TUs via `rtwdev->chip`.  Values sourced   */
/* from the extracted inline efuse / coex code and Linux's rtw8821c.c. */
/* ------------------------------------------------------------------ */
static const struct rtw88_chip_info rtw88_8821c_chip_info = {
	.chip_id		= RTW88_CHIP_ID_8821C,
	.name			= "RTL8821C",

	/* EFUSE layout -- extracted from the old inline efuse block. */
	.efuse_size		= 512,
	.efuse_ptct_size	= 96,
	.efuse_mac_offset	= 0x107,	/* 8821CU MAC offset  */
	.efuse_xtal_k_offset	= 0x0B9,	/* shared with 8821CE */
	.efuse_rfe_offset	= 0x0CA,

	/* Coex defaults captured from the reference driver. */
	.coex_default_tdma		= 0,		/* WiFi-only */
	.coex_bt_wifi_ctrl_payload	= 0x00010c69,	/* 69/0c/01 */
	.coex_tdma_type_payload		= 0x00000060,
	/*
	 * rfe_option codes that select the BTG (BT-shared) antenna path
	 * on 8821CU: 0x02, 0x04, 0x07, 0x0A, 0x0C, 0x0F.  Encoded as a
	 * bitmap indexed by rfe_option so `rtw88_efuse.btcoex` resolves
	 * at read time.
	 */
	.coex_rfe_btg_mask		= (1U << 0x02) | (1U << 0x04) |
					  (1U << 0x07) | (1U << 0x0A) |
					  (1U << 0x0C) | (1U << 0x0F),

	.lps_deep_mode_supported	= 0,		/* deep-PS deferred */
	.bf_su_supported		= true,
	.bf_mu_supported		= false,

	.led_supported			= true,
	.led_gpio_mask			= (1U << 0),	/* LED0 only */

	.max_power_index		= 0x3F,

	.coex_cfg_init			= NULL,	/* use default 8821C-shaped */
	.coex_switch_rf_wonly		= NULL,
};

static int
rtw88_usb_attach(device_t self)
{
	struct usb_attach_arg *uaa = device_get_ivars(self);
	struct rtw88_usb_softc *sc = device_get_softc(self);
	struct usb_interface *iface;
	uint32_t sys_cfg1, sys_cfg2;
	uint8_t chip_ver, vendor_id, chip_tag;
	int error;

	sc->sc_dev = self;
	sc->sc_udev = uaa->device;

	/*
	 * Resolve the chip spec from the matched probe-table entry.
	 * uaa->driver_info is the unsigned-long index the USB framework
	 * copied from the matched STRUCT_USB_HOST_ID's USB_DRIVER_INFO
	 * slot.  Out-of-range = a probe-table entry pointed at a chip
	 * variant the spec table doesn't describe; refuse attach loudly.
	 */
	if (uaa->driver_info < RTW88_N_CHIP_SPECS) {
		sc->sc_chip = &rtw88_chip_specs[uaa->driver_info];
	} else {
		/*
		 * driver_info out of range = probe table grew but chip
		 * spec table didn't.  Loud failure beats silently masking
		 * as 8821C and producing wrong fw_name / signature / caps.
		 */
		device_printf(self,
		    "no chip spec for driver_info=%lu (table has %d entries)\n",
		    uaa->driver_info, RTW88_N_CHIP_SPECS);
		return (ENXIO);
	}
	device_printf(self,
	    "chip spec: %s (chip_id=0x%04x fw=%s caps=0x%03x %uT%uR)\n",
	    sc->sc_chip->name, sc->sc_chip->chip_id, sc->sc_chip->fw_name,
	    sc->sc_chip->caps, sc->sc_chip->n_tx_chains,
	    sc->sc_chip->n_rx_chains);
	sc->sc_hw_ccmp = (rtw88_hw_ccmp != 0);
	sc->sc_ampdu = (rtw88_ampdu != 0);
	/* Default to BGN_20M_1SS to match the pre-2026-06-26 constant the
	 * chip's RA engine had been seeded with — keeps M2 TX behaviour
	 * identical for HT APs until LINK_UP's per-link picker refines
	 * to BG for non-HT APs.  Wrong-default for non-HT links is
	 * preferable to wrong-default for HT links: for HT, BGN includes
	 * the rates BG would have allowed, so a too-broad seed is
	 * harmless; for the inverse, the chip RAs into MCS the AP can't
	 * demodulate. */
	sc->sc_rate_id = RTW88_RATEID_BGN_20M_1SS;
	mtx_init(&sc->sc_mtx, device_get_nameunit(self), MTX_NETWORK_LOCK,
	    MTX_DEF);

	/*
	 * Wire the bus/chip-abstract handle before anything downstream
	 * (mac_power_on -> coex_init, download_firmware, efuse_read, ...)
	 * needs to talk to a subsystem.  `priv` back-points at this softc;
	 * `hci_ops` bridges register access + H2C; `chip` supplies per-
	 * silicon parameters.
	 */
	sc->sc_rtwdev.priv = sc;
	sc->sc_rtwdev.dev = self;
	sc->sc_rtwdev.ic = &sc->sc_ic;
	sc->sc_rtwdev.mtx = &sc->sc_mtx;
	sc->sc_rtwdev.hci_type = RTW88_HCI_TYPE_USB;
	sc->sc_rtwdev.hci_ops = &rtw88_usb_hci_ops;
	sc->sc_rtwdev.chip = &rtw88_8821c_chip_info;
	sysctl_ctx_init(&sc->sc_rtwdev.sysctl_ctx);
	rtw88_regd_init(&sc->sc_rtwdev);
	rtw88_ps_attach(&sc->sc_rtwdev, 0);	/* STA mac_id 0 */
	rtw88_bf_attach(&sc->sc_rtwdev);
	callout_init(&sc->sc_dig_co, 1);
	TASK_INIT(&sc->sc_dig_task, 0, rtw88_dig_task, sc);
	callout_init(&sc->sc_fwka_co, 1);
	TASK_INIT(&sc->sc_fwka_task, 0, rtw88_fwka_task, sc);
	TASK_INIT(&sc->sc_prepare_tx_task, 0, rtw88_prepare_tx_task, sc);
	TASK_INIT(&sc->sc_post_state_task, 0, rtw88_post_state_task, sc);
	TASK_INIT(&sc->sc_cam_task, 0, rtw88_cam_task, sc);
	sc->sc_c2h_trace = RTW88_C2H_TRACE_BOOT;
	sc->sc_tq = taskqueue_create("rtw88_tq", M_WAITOK | M_ZERO,
	    taskqueue_thread_enqueue, &sc->sc_tq);
	taskqueue_start_threads(&sc->sc_tq, 1, PI_NET, "%s tq",
	    device_get_nameunit(self));
	sc->sc_cam_tq = taskqueue_create("rtw88_cam_tq", M_WAITOK | M_ZERO,
	    taskqueue_thread_enqueue, &sc->sc_cam_tq);
	taskqueue_start_threads(&sc->sc_cam_tq, 1, PI_NET, "%s cam_tq",
	    device_get_nameunit(self));
	cv_init(&sc->sc_tx_cv, "rtw88txcv");
	sc->sc_tx_buf = malloc(RTW88_TX_BUFSZ, M_USBDEV, M_WAITOK | M_ZERO);
	sc->sc_rx_stage = malloc(RTW88_RX_BUFSZ, M_USBDEV, M_WAITOK | M_ZERO);
	mbufq_init(&sc->sc_tx_mgmt_q,
	    rtw88_tx_mgmt_qdepth > 0 ? rtw88_tx_mgmt_qdepth : 64);
	mbufq_init(&sc->sc_tx_data_q,
	    rtw88_tx_data_qdepth > 0 ? rtw88_tx_data_qdepth : 256);
	mbufq_init(&sc->sc_tx_data_vo_q,
	    rtw88_tx_data_qdepth > 0 ? rtw88_tx_data_qdepth : 256);
	mbufq_init(&sc->sc_tx_pending_q, 16);

	device_set_usb_desc(self);

	/*
	 * Endpoint sanity.  rtw88 USB silicon presents a single interface
	 * with five bulk endpoints: one bulk-IN for RX and four bulk-OUT
	 * for TX (queues HI/NORMAL/LO + H2C cmd).  rtw88_xfers_setup
	 * claims each via a usb_config[] template; here we just verify
	 * the count so an unexpected device variant fails loudly.
	 */
	iface = usbd_get_iface(sc->sc_udev, 0);
	sc->sc_nendpoints = iface->idesc->bNumEndpoints;
	device_printf(self, "  bNumEndpoints=%u (expected 5)\n",
	    sc->sc_nendpoints);

	RTW88_LOCK(sc);
	error = rtw88_usb_read4(sc, REG_SYS_CFG1, &sys_cfg1);
	if (error != 0)
		goto fail;
	error = rtw88_usb_read4(sc, REG_SYS_CFG2, &sys_cfg2);
	if (error != 0)
		goto fail;
	RTW88_UNLOCK(sc);

	chip_ver = (sys_cfg1 >> SYS_CFG1_CHIP_VER_SHIFT) &
	    SYS_CFG1_CHIP_VER_MASK;
	sc->sc_cut_version = chip_ver;
	vendor_id = (sys_cfg1 >> SYS_CFG1_VENDOR_SHIFT) &
	    SYS_CFG1_VENDOR_MASK;
	chip_tag = (uint8_t)(sys_cfg2 >> 24);

	device_printf(self,
	    "RTL8821CU / RTL8811CU SYS_CFG1=0x%08x SYS_CFG2=0x%08x\n",
	    sys_cfg1, sys_cfg2);
	device_printf(self,
	    "  chip_ver=0x%x vendor_id=0x%x rtl_id=%d ldo=%d rf_type=%d"
	    " chip_tag=0x%02x\n",
	    chip_ver, vendor_id,
	    (sys_cfg1 & SYS_CFG1_BIT_RTL_ID) ? 1 : 0,
	    (sys_cfg1 & SYS_CFG1_BIT_LDO) ? 1 : 0,
	    (sys_cfg1 & SYS_CFG1_BIT_RF_TYPE) ? 1 : 0,
	    chip_tag);

	/*
	 * Firmware acquisition is best-effort -- without the sibling
	 * rtw88-rtl8821cufw.ko kmod loaded, the registration won't be
	 * found.  Either outcome keeps attach successful here; the
	 * actual download (rtw88_download_firmware below) is where
	 * ENOENT becomes terminal.
	 */
	(void)rtw88_fw_acquire(sc);

	error = rtw88_discover_endpoints(sc);
	if (error != 0)
		goto fail_xfers;

	/*
	 * Probe the config descriptor for class-specific (UDESC_CS_*) and
	 * interface-association (UDESC_IFACE_ASSOC) extras.  Today no
	 * 8821C variant we've seen ships any -- but counting them at
	 * attach makes a future RTL8822B / OEM rebadge with vendor
	 * configuration bytes immediately visible in dmesg.  Cheap
	 * forward-looking diagnostic; no behaviour change for known
	 * silicon.
	 */
	{
		int n_cs_iface = rtw88_walk_descriptors(sc,
		    UDESC_CS_INTERFACE, NULL, NULL);
		int n_cs_endpoint = rtw88_walk_descriptors(sc,
		    UDESC_CS_ENDPOINT, NULL, NULL);
		int n_iface_assoc = rtw88_walk_descriptors(sc,
		    UDESC_IFACE_ASSOC, NULL, NULL);
		if (n_cs_iface + n_cs_endpoint + n_iface_assoc > 0)
			device_printf(self,
			    "extra descriptors: CS_iface=%d CS_endpoint=%d "
			    "iface_assoc=%d (none consumed today)\n",
			    n_cs_iface, n_cs_endpoint, n_iface_assoc);
	}

	error = rtw88_xfers_setup(sc);
	if (error != 0)
		goto fail_xfers;

	/*
	 * Full firmware download.  On success the chip MCU advertises
	 * FW_READY; we then bring up the net80211 layer so userspace can
	 * clone wlan0 onto this device.  Firmware-download failure keeps
	 * the device attached for inspection but skips net80211 attach.
	 */
	error = rtw88_download_firmware(sc);
	if (error == 0) {
		sc->sc_fw_running = true;
		/*
		 * Best-effort EFUSE MAC read.  On failure the net80211
		 * attach falls back to a locally-administered placeholder
		 * so the interface still comes up.
		 */
		if (rtw88_efuse_read(&sc->sc_rtwdev) != 0) {
			device_printf(self,
			    "efuse MAC read failed; using placeholder\n");
		} else {
			/*
			 * Mirror the cached EFUSE fields back into the USB
			 * softc so existing consumers (mac_init phy paths,
			 * per-frame TX desc, phy_set_param, ...) don't need
			 * a per-site swap.  Follow-up refactor will delete
			 * these mirrored fields once every consumer reads
			 * from `sc_rtwdev.efuse.*` directly.
			 */
			memcpy(sc->sc_mac, sc->sc_rtwdev.efuse.mac_addr, 6);
			sc->sc_have_mac = true;
			sc->sc_crystal_cap = sc->sc_rtwdev.efuse.xtal_cap;
			sc->sc_rfe_option = sc->sc_rtwdev.efuse.rfe_option;
			sc->sc_pkg_type = sc->sc_rtwdev.efuse.pkg_type;
		}
		if ((error = rtw88_mac_init(sc)) != 0) {
			device_printf(self, "mac_init failed: %d\n", error);
			/* Continue to net80211 attach for inspection. */
			error = 0;
		}
		if ((error = rtw88_phy_set_param(sc)) != 0) {
			device_printf(self,
			    "phy_set_param failed: %d\n", error);
			error = 0;
		}
		error = rtw88_net80211_attach(sc);
		if (error != 0) {
			device_printf(self,
			    "net80211 attach failed: %d\n", error);
		} else {
			sc->sc_ic_attached = true;
			/*
			 * Post-net80211 subsystem wiring: LED node
			 * registration + debug sysctl umbrella.  Both
			 * hang off the caller-owned sysctl tree so
			 * they're torn down automatically at detach.
			 */
			(void)rtw88_led_attach(&sc->sc_rtwdev);
			(void)rtw88_debug_attach(&sc->sc_rtwdev,
			    device_get_sysctl_ctx(self),
			    device_get_sysctl_tree(self));
		}
	} else {
		device_printf(self,
		    "firmware download failed (%d); leaving attached for"
		    " inspection but skipping net80211 attach\n", error);
	}

	return (0);

fail:
	RTW88_UNLOCK(sc);
fail_xfers:
	/*
	 * Centralised attach failure cleanup.  Mirrors the steady-state
	 * detach order for the resources allocated up to this point:
	 * USB xfers, firmware handle, taskqueues, mbuf queues, malloc'd
	 * buffers, cv, mutex.  fail_xfers entered with sc_xfer empty
	 * (xfers_setup failed); usbd_transfer_unsetup tolerates that.
	 */
	rtw88_xfers_teardown(sc);
	rtw88_fw_release(sc);
	if (sc->sc_tx_buf != NULL)
		free(sc->sc_tx_buf, M_USBDEV);
	if (sc->sc_rx_stage != NULL)
		free(sc->sc_rx_stage, M_USBDEV);
	mbufq_drain(&sc->sc_tx_mgmt_q);
	mbufq_drain(&sc->sc_tx_data_q);
	mbufq_drain(&sc->sc_tx_data_vo_q);
	mbufq_drain(&sc->sc_tx_pending_q);
	taskqueue_free(sc->sc_cam_tq);
	taskqueue_free(sc->sc_tq);
	cv_destroy(&sc->sc_tx_cv);
	mtx_destroy(&sc->sc_mtx);
	return (error);
}

static int
rtw88_usb_detach(device_t self)
{
	struct rtw88_usb_softc *sc = device_get_softc(self);
	struct ieee80211com *ic = &sc->sc_ic;

	/*
	 * Destroy /dev/rtw88_usb0 FIRST.  destroy_dev synchronises with
	 * in-flight cdev ioctls; doing it before mtx_destroy guarantees no
	 * racing ioctl can take RTW88_LOCK on a destroyed mutex.
	 */
	if (sc->sc_cdev != NULL) {
		destroy_dev(sc->sc_cdev);
		sc->sc_cdev = NULL;
	}

	/*
	 * Tear down subsystem attach state.  LED cdev must go before we
	 * lose the driver instance; ps_detach idempotently returns ACTIVE.
	 * debug + regd + bf are pure in-memory teardown; sysctl OIDs are
	 * freed with the parent ctx by newbus.
	 */
	rtw88_led_detach(&sc->sc_rtwdev);
	rtw88_debug_detach(&sc->sc_rtwdev);
	rtw88_ps_detach(&sc->sc_rtwdev);
	sysctl_ctx_free(&sc->sc_rtwdev.sysctl_ctx);

	if (sc->sc_ic_attached)
		ieee80211_ifdetach(ic);
	RTW88_LOCK(sc);
	sc->sc_dig_active = false;
	RTW88_UNLOCK(sc);
	/*
	 * Drain order: kill the callout first so no new tasks get
	 * enqueued, drain the task (which sees sc_dig_active==false and
	 * returns without re-arming), then a second callout_drain to
	 * catch the corner case where the task was already past the
	 * entry check and armed the callout before we cleared active.
	 * After this no callout or task references the softc.
	 */
	callout_drain(&sc->sc_dig_co);
	taskqueue_drain(sc->sc_tq, &sc->sc_dig_task);
	callout_drain(&sc->sc_dig_co);
	sc->sc_fwka_active = false;
	callout_drain(&sc->sc_fwka_co);
	taskqueue_drain(sc->sc_tq, &sc->sc_fwka_task);
	callout_drain(&sc->sc_fwka_co);
	taskqueue_drain(sc->sc_tq, &sc->sc_prepare_tx_task);
	taskqueue_drain(sc->sc_tq, &sc->sc_post_state_task);
	taskqueue_drain(sc->sc_cam_tq, &sc->sc_cam_task);
	/*
	 * sc_m4_needs_encap references an mbuf that lives in
	 * sc_tx_pending_q or one of the data queues -- those are drained
	 * as part of xfers_teardown below.  Just null the marker.
	 */
	sc->sc_m4_needs_encap = NULL;
	taskqueue_free(sc->sc_cam_tq);
	taskqueue_free(sc->sc_tq);
	rtw88_xfers_teardown(sc);
	rtw88_phy_release_tbl(sc);
	rtw88_fw_release(sc);
	if (sc->sc_tx_buf != NULL)
		free(sc->sc_tx_buf, M_USBDEV);
	if (sc->sc_rx_stage != NULL)
		free(sc->sc_rx_stage, M_USBDEV);
	mbufq_drain(&sc->sc_tx_mgmt_q);
	mbufq_drain(&sc->sc_tx_data_q);
	mbufq_drain(&sc->sc_tx_data_vo_q);
	mbufq_drain(&sc->sc_tx_pending_q);
	if (sc->sc_tx_data_inflight != NULL) {
		m_freem(sc->sc_tx_data_inflight);
		sc->sc_tx_data_inflight = NULL;
	}
	if (sc->sc_tx_data_vo_inflight != NULL) {
		m_freem(sc->sc_tx_data_vo_inflight);
		sc->sc_tx_data_vo_inflight = NULL;
	}
	if (sc->sc_tx_mgmt_inflight != NULL) {
		m_freem(sc->sc_tx_mgmt_inflight);
		sc->sc_tx_mgmt_inflight = NULL;
	}
	cv_destroy(&sc->sc_tx_cv);
	mtx_destroy(&sc->sc_mtx);
	return (0);
}

static int
rtw88_usb_suspend(device_t self)
{
	struct rtw88_usb_softc *sc = device_get_softc(self);

	if (sc->sc_ic_attached)
		ieee80211_suspend_all(&sc->sc_ic);
	return (0);
}

static int
rtw88_usb_resume(device_t self)
{
	struct rtw88_usb_softc *sc = device_get_softc(self);

	if (sc->sc_ic_attached)
		ieee80211_resume_all(&sc->sc_ic);
	return (0);
}

static device_method_t rtw88_usb_methods[] = {
	DEVMETHOD(device_probe,		rtw88_usb_match),
	DEVMETHOD(device_attach,	rtw88_usb_attach),
	DEVMETHOD(device_detach,	rtw88_usb_detach),
	DEVMETHOD(device_suspend,	rtw88_usb_suspend),
	DEVMETHOD(device_resume,	rtw88_usb_resume),

	DEVMETHOD_END
};

static driver_t rtw88_usb_driver = {
	.name = "rtw88_usb",
	.methods = rtw88_usb_methods,
	.size = sizeof(struct rtw88_usb_softc),
};

DRIVER_MODULE(rtw88_usb, uhub, rtw88_usb_driver, NULL, NULL);
MODULE_DEPEND(rtw88_usb, usb, 1, 1, 1);
MODULE_DEPEND(rtw88_usb, firmware, 1, 1, 1);
MODULE_DEPEND(rtw88_usb, wlan, 1, 1, 1);
MODULE_VERSION(rtw88_usb, 1);
USB_PNP_HOST_INFO(rtw88_usb_devs);
