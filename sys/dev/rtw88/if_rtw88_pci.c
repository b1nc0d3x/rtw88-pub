/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * Realtek RTL8822BE / RTL8822CE / RTL8814AE PCIe 802.11ac skeleton.
 *
 * SCOPE OF THIS SKELETON
 * ----------------------
 * The USB front-end (if_rtw88_usb.c) grew as a monolith that combines
 * bus glue, register access, firmware download, MAC/PHY init, net80211
 * attach and TX/RX rings into a single 10K-LOC translation unit.  Any
 * PCI front-end would have to either:
 *
 *   (a) duplicate the whole thing with s/usb/pci/g and swap the bulk
 *       endpoints for DMA rings — ~10K new LOC, unmaintainable, or
 *   (b) refactor the USB monolith into a bus-agnostic "core" plus a
 *       thin front-end per bus (USB / PCI / SDIO) — a much larger
 *       multi-day cleanup that must precede any second front-end.
 *
 * This file is neither.  It is the smallest thing that lets the
 * kernel *see* an rtw88-compatible PCIe card at boot: probe against
 * the Linux VID/PID table, map BAR0, arm MSI, and print a signature
 * read from the chip.  Real TX/RX + net80211 attach are left to the
 * shared-core refactor.
 *
 * With this loaded, `pciconf -lv` will show
 *   `rtw88_pci0@…: <RTL8822BE 802.11ac WiFi>` on Realtek 0xB822, etc.,
 * and `sysctl dev.rtw88_pci.0.sig` returns a 32-bit value read from
 * offset 0xF0 of BAR0 (SYS_CFG1) — non-zero if the chip is powered
 * and register access works.
 *
 * Compat matrix (from Linux rtw8822be.c, rtw8822ce.c, rtw8814ae.c):
 *   0x10ec:0xB822  RTL8822BE
 *   0x10ec:0xC822  RTL8822CE
 *   0x10ec:0xC82F  RTL8822CE (alt SVID)
 *   0x10ec:0x8813  RTL8814AE
 */

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/systm.h>
#include <sys/sysctl.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include "rtw88.h"
#include "rtw88_efuse.h"

#define	RTW88_VENDOR_REALTEK	0x10ec

#define	RTW88_DEV_8822BE	0xB822
#define	RTW88_DEV_8822CE	0xC822
#define	RTW88_DEV_8822CE_ALT	0xC82F
#define	RTW88_DEV_8814AE	0x8813

/*
 * REG_SYS_CFG1: the same 0xF0 offset used across all Realtek WiFi
 * silicon since 8188.  Reads a non-zero magic when the chip is
 * powered.  It's the classic "am I alive" probe.
 */
#define	RTW88_REG_SYS_CFG1	0x00F0

struct rtw88_pci_dev {
	uint16_t	 vid;
	uint16_t	 did;
	const char	*descr;
};

static const struct rtw88_pci_dev rtw88_pci_devs[] = {
	{ RTW88_VENDOR_REALTEK, RTW88_DEV_8822BE,
	    "Realtek RTL8822BE 802.11ac WiFi" },
	{ RTW88_VENDOR_REALTEK, RTW88_DEV_8822CE,
	    "Realtek RTL8822CE 802.11ac WiFi" },
	{ RTW88_VENDOR_REALTEK, RTW88_DEV_8822CE_ALT,
	    "Realtek RTL8822CE 802.11ac WiFi" },
	{ RTW88_VENDOR_REALTEK, RTW88_DEV_8814AE,
	    "Realtek RTL8814AE 802.11ac WiFi" },
};

struct rtw88_pci_softc {
	device_t		 dev;

	struct resource		*bar0;
	int			 bar0_rid;

	struct resource		*irq;
	int			 irq_rid;
	int			 msi_count;
	void			*ih;

	struct mtx		 mtx;
	struct rtw88_dev	 rtwdev;	/* shared-core handle */

	uint16_t		 vid;
	uint16_t		 did;

	uint32_t		 sig;	/* SYS_CFG1 snapshot */
	uint8_t			 mac_str[18];	/* "xx:xx:xx:xx:xx:xx" */
};

static inline uint32_t
rtw88_pci_r32(struct rtw88_pci_softc *sc, bus_size_t off)
{
	return (bus_read_4(sc->bar0, off));
}

static inline void __unused
rtw88_pci_w32(struct rtw88_pci_softc *sc, bus_size_t off, uint32_t val)
{
	bus_write_4(sc->bar0, off, val);
}

/* ------------------------------------------------------------------ */
/* hci_ops implementation: PCI MMIO access via bus_space.             */
/* ------------------------------------------------------------------ */

static int
rtw88_pci_hci_read8(struct rtw88_dev *d, uint32_t addr, uint8_t *val)
{
	struct rtw88_pci_softc *sc = d->priv;
	*val = bus_read_1(sc->bar0, addr);
	return (0);
}

static int
rtw88_pci_hci_read16(struct rtw88_dev *d, uint32_t addr, uint16_t *val)
{
	struct rtw88_pci_softc *sc = d->priv;
	*val = bus_read_2(sc->bar0, addr);
	return (0);
}

static int
rtw88_pci_hci_read32(struct rtw88_dev *d, uint32_t addr, uint32_t *val)
{
	struct rtw88_pci_softc *sc = d->priv;
	*val = bus_read_4(sc->bar0, addr);
	return (0);
}

static int
rtw88_pci_hci_write8(struct rtw88_dev *d, uint32_t addr, uint8_t val)
{
	struct rtw88_pci_softc *sc = d->priv;
	bus_write_1(sc->bar0, addr, val);
	return (0);
}

static int
rtw88_pci_hci_write16(struct rtw88_dev *d, uint32_t addr, uint16_t val)
{
	struct rtw88_pci_softc *sc = d->priv;
	bus_write_2(sc->bar0, addr, val);
	return (0);
}

static int
rtw88_pci_hci_write32(struct rtw88_dev *d, uint32_t addr, uint32_t val)
{
	struct rtw88_pci_softc *sc = d->priv;
	bus_write_4(sc->bar0, addr, val);
	return (0);
}

static int
rtw88_pci_hci_read_region(struct rtw88_dev *d, uint32_t addr, void *buf,
    uint32_t len)
{
	struct rtw88_pci_softc *sc = d->priv;
	bus_read_region_1(sc->bar0, addr, buf, len);
	return (0);
}

static int
rtw88_pci_hci_write_region(struct rtw88_dev *d, uint32_t addr,
    const void *buf, uint32_t len)
{
	struct rtw88_pci_softc *sc = d->priv;
	bus_write_region_1(sc->bar0, addr, __DECONST(uint8_t *, buf), len);
	return (0);
}

static int
rtw88_pci_hci_h2c(struct rtw88_dev *d, uint8_t cmd, uint32_t msg,
    uint32_t msg_ext)
{
	/*
	 * TODO: implement H2C mailbox on PCI.  Same registers as USB
	 * (REG_HMEBOX/REG_HMETFR) — the USB path's rtw88_h2c_mailbox is
	 * already register-only, so it will migrate cleanly once we move
	 * it into the shared core.
	 */
	(void)d; (void)cmd; (void)msg; (void)msg_ext;
	return (EOPNOTSUPP);
}

static const struct rtw88_hci_ops rtw88_pci_hci_ops = {
	.read8		= rtw88_pci_hci_read8,
	.read16		= rtw88_pci_hci_read16,
	.read32		= rtw88_pci_hci_read32,
	.write8		= rtw88_pci_hci_write8,
	.write16	= rtw88_pci_hci_write16,
	.write32	= rtw88_pci_hci_write32,
	.read_region	= rtw88_pci_hci_read_region,
	.write_region	= rtw88_pci_hci_write_region,
	.h2c		= rtw88_pci_hci_h2c,
};

/*
 * TODO: per-chip rtw88_chip_info tables for 8822B/C/8814A live in
 * separate rtw88_88{22b,22c,14a}.c files once ported.  Until then, the
 * PCI front-end has no chip table to hand rtw88_dev.chip; subsystem
 * code that needs chip-specific offsets (EFUSE MAC offset etc.) will
 * short-circuit on chip == NULL.
 */

static void
rtw88_pci_intr(void *arg)
{
	struct rtw88_pci_softc *sc = arg;

	/*
	 * Skeleton: don't touch chip regs from ISR yet.  Real IRQ handling
	 * belongs with TX/RX rings which the shared-core refactor
	 * will bring in.
	 */
	(void)sc;
}

static const struct rtw88_pci_dev *
rtw88_pci_match(device_t dev)
{
	uint16_t vid = pci_get_vendor(dev);
	uint16_t did = pci_get_device(dev);
	unsigned i;

	for (i = 0; i < nitems(rtw88_pci_devs); i++) {
		if (rtw88_pci_devs[i].vid == vid &&
		    rtw88_pci_devs[i].did == did)
			return (&rtw88_pci_devs[i]);
	}
	return (NULL);
}

static int
rtw88_pci_probe(device_t dev)
{
	const struct rtw88_pci_dev *e = rtw88_pci_match(dev);

	if (e == NULL)
		return (ENXIO);
	device_set_desc(dev, e->descr);
	return (BUS_PROBE_DEFAULT);
}

static int
rtw88_pci_attach(device_t dev)
{
	struct rtw88_pci_softc *sc = device_get_softc(dev);
	struct sysctl_ctx_list *sctx;
	struct sysctl_oid *stree;

	sc->dev = dev;
	sc->vid = pci_get_vendor(dev);
	sc->did = pci_get_device(dev);

	pci_enable_busmaster(dev);

	/*
	 * BAR0 (rid 0x10) is memory-mapped registers on every rtw88 PCIe
	 * variant — same layout as the USB register wire, just accessed
	 * over the PCIe MMIO window.
	 */
	sc->bar0_rid = PCIR_BAR(0);
	sc->bar0 = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->bar0_rid, RF_ACTIVE);
	if (sc->bar0 == NULL) {
		device_printf(dev, "cannot map BAR0\n");
		return (ENXIO);
	}

	/*
	 * Prefer MSI over legacy INTx.  Fall back to line IRQ if MSI is
	 * unavailable / disabled by the platform.
	 */
	sc->msi_count = 1;
	sc->irq_rid = 0;
	if (pci_alloc_msi(dev, &sc->msi_count) == 0 && sc->msi_count >= 1) {
		sc->irq_rid = 1;
	} else {
		sc->msi_count = 0;
	}
	sc->irq = bus_alloc_resource_any(dev, SYS_RES_IRQ, &sc->irq_rid,
	    RF_ACTIVE | (sc->msi_count == 0 ? RF_SHAREABLE : 0));
	if (sc->irq == NULL) {
		device_printf(dev, "cannot allocate IRQ\n");
		goto fail_bar;
	}

	if (bus_setup_intr(dev, sc->irq, INTR_TYPE_NET | INTR_MPSAFE,
	    NULL, rtw88_pci_intr, sc, &sc->ih) != 0) {
		device_printf(dev, "cannot establish IRQ handler\n");
		goto fail_irq;
	}

	/* Probe signature — proves BAR0 is live. */
	sc->sig = rtw88_pci_r32(sc, RTW88_REG_SYS_CFG1);

	/*
	 * Wire the shared-core handle.  This lets rtw88_efuse / _coex /
	 * _ps / _bf / _led / _regd all work over PCI without a single
	 * bus-specific #ifdef in their code — they just call
	 * rtw_read32(rtwdev, ...) which dispatches through hci_ops.
	 */
	mtx_init(&sc->mtx, "rtw88_pci", NULL, MTX_DEF);
	sc->rtwdev.priv		= sc;
	sc->rtwdev.dev		= dev;
	sc->rtwdev.ic		= NULL;	/* net80211 attach deferred */
	sc->rtwdev.mtx		= &sc->mtx;
	sc->rtwdev.hci_type	= RTW88_HCI_TYPE_PCIE;
	sc->rtwdev.hci_ops	= &rtw88_pci_hci_ops;
	sc->rtwdev.chip		= NULL;	/* per-chip table TODO */

	sctx = device_get_sysctl_ctx(dev);
	stree = device_get_sysctl_tree(dev);
	SYSCTL_ADD_UINT(sctx, SYSCTL_CHILDREN(stree), OID_AUTO, "sig",
	    CTLFLAG_RD, &sc->sig, 0, "SYS_CFG1 (BAR0+0xF0)");
	SYSCTL_ADD_U16(sctx, SYSCTL_CHILDREN(stree), OID_AUTO, "vid",
	    CTLFLAG_RD, &sc->vid, 0, "PCI vendor");
	SYSCTL_ADD_U16(sctx, SYSCTL_CHILDREN(stree), OID_AUTO, "did",
	    CTLFLAG_RD, &sc->did, 0, "PCI device");
	SYSCTL_ADD_STRING(sctx, SYSCTL_CHILDREN(stree), OID_AUTO, "mac",
	    CTLFLAG_RD, sc->mac_str, 0, "EFUSE MAC (once chip table lands)");

	device_printf(dev,
	    "attached %04x:%04x SYS_CFG1=%#x hci_ops wired  (skeleton — "
	    "TX/RX + net80211 pending per-chip tables)\n",
	    sc->vid, sc->did, sc->sig);
	return (0);

fail_irq:
	bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rid, sc->irq);
	sc->irq = NULL;
	if (sc->msi_count > 0)
		pci_release_msi(dev);
fail_bar:
	bus_release_resource(dev, SYS_RES_MEMORY, sc->bar0_rid, sc->bar0);
	sc->bar0 = NULL;
	pci_disable_busmaster(dev);
	return (ENXIO);
}

static int
rtw88_pci_detach(device_t dev)
{
	struct rtw88_pci_softc *sc = device_get_softc(dev);

	if (sc->ih != NULL) {
		bus_teardown_intr(dev, sc->irq, sc->ih);
		sc->ih = NULL;
	}
	if (sc->irq != NULL) {
		bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rid, sc->irq);
		sc->irq = NULL;
	}
	if (sc->msi_count > 0) {
		pci_release_msi(dev);
		sc->msi_count = 0;
	}
	if (sc->bar0 != NULL) {
		bus_release_resource(dev, SYS_RES_MEMORY, sc->bar0_rid,
		    sc->bar0);
		sc->bar0 = NULL;
	}
	if (mtx_initialized(&sc->mtx))
		mtx_destroy(&sc->mtx);
	pci_disable_busmaster(dev);
	return (0);
}

static device_method_t rtw88_pci_methods[] = {
	DEVMETHOD(device_probe,		rtw88_pci_probe),
	DEVMETHOD(device_attach,	rtw88_pci_attach),
	DEVMETHOD(device_detach,	rtw88_pci_detach),
	DEVMETHOD_END
};

static driver_t rtw88_pci_driver = {
	"rtw88_pci",
	rtw88_pci_methods,
	sizeof(struct rtw88_pci_softc),
};

DRIVER_MODULE(rtw88_pci, pci, rtw88_pci_driver, 0, 0);
MODULE_VERSION(rtw88_pci, 1);
MODULE_PNP_INFO("U16:vendor;U16:device;D:#", pci, rtw88_pci,
    rtw88_pci_devs, nitems(rtw88_pci_devs));
