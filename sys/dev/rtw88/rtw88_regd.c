/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * rtw88 regulatory-domain subsystem.
 *
 * Country -> (regd_2g, regd_5g) lookup table extracted from Linux
 * `regd.c` (rtw_reg_map + rtw_reg_map_2g_5g).  Covers the countries
 * FreeBSD net80211's regdomain table already recognises; anything
 * else falls back to WW (worldwide).
 *
 * Chip-side power-index programming is a TODO -- the table is
 * available today so callers can inspect `rtwdev->regd.*` and
 * chip-init code can consult it, but the RF-power reprogram at
 * runtime regd change requires the phy TX-power table port.
 */

#include <sys/param.h>
#include <sys/systm.h>

#include "rtw88.h"
#include "rtw88_regd.h"
#include "rtw88_debug.h"

struct rtw88_regd_entry {
	char		country[3];
	uint8_t		regd_2g;
	uint8_t		regd_5g;
};

/*
 * Extract of the Linux rtw88 rtw_reg_map.  Add entries by consulting
 * `linux/drivers/net/wireless/realtek/rtw88/regd.c` and picking the
 * (regd_2g, regd_5g) pair for the target country.
 *
 * Order does not matter -- lookup is linear.  Trailing NUL terminator
 * required for strncmp match.
 */
static const struct rtw88_regd_entry rtw88_regd_table[] = {
	{ "US", RTW88_REGD_FCC,	 RTW88_REGD_FCC },
	{ "CA", RTW88_REGD_IC,	 RTW88_REGD_IC },
	{ "MX", RTW88_REGD_MEXICO, RTW88_REGD_MEXICO },
	{ "CL", RTW88_REGD_CHILE, RTW88_REGD_CHILE },
	{ "GB", RTW88_REGD_UK,	 RTW88_REGD_UK },
	{ "DE", RTW88_REGD_ETSI, RTW88_REGD_ETSI },
	{ "FR", RTW88_REGD_ETSI, RTW88_REGD_ETSI },
	{ "IT", RTW88_REGD_ETSI, RTW88_REGD_ETSI },
	{ "ES", RTW88_REGD_ETSI, RTW88_REGD_ETSI },
	{ "NL", RTW88_REGD_ETSI, RTW88_REGD_ETSI },
	{ "PL", RTW88_REGD_ETSI, RTW88_REGD_ETSI },
	{ "SE", RTW88_REGD_ETSI, RTW88_REGD_ETSI },
	{ "FI", RTW88_REGD_ETSI, RTW88_REGD_ETSI },
	{ "AT", RTW88_REGD_ETSI, RTW88_REGD_ETSI },
	{ "BE", RTW88_REGD_ETSI, RTW88_REGD_ETSI },
	{ "CH", RTW88_REGD_ETSI, RTW88_REGD_ETSI },
	{ "CZ", RTW88_REGD_ETSI, RTW88_REGD_ETSI },
	{ "DK", RTW88_REGD_ETSI, RTW88_REGD_ETSI },
	{ "IE", RTW88_REGD_ETSI, RTW88_REGD_ETSI },
	{ "NO", RTW88_REGD_ETSI, RTW88_REGD_ETSI },
	{ "PT", RTW88_REGD_ETSI, RTW88_REGD_ETSI },
	{ "GR", RTW88_REGD_ETSI, RTW88_REGD_ETSI },
	{ "HU", RTW88_REGD_ETSI, RTW88_REGD_ETSI },
	{ "TR", RTW88_REGD_ETSI, RTW88_REGD_ETSI },
	{ "RU", RTW88_REGD_ETSI, RTW88_REGD_ETSI },
	{ "UA", RTW88_REGD_UKRAINE, RTW88_REGD_UKRAINE },
	{ "AU", RTW88_REGD_ACMA, RTW88_REGD_ACMA },
	{ "NZ", RTW88_REGD_ACMA, RTW88_REGD_ACMA },
	{ "JP", RTW88_REGD_MKK,	 RTW88_REGD_MKK },
	{ "KR", RTW88_REGD_KCC,	 RTW88_REGD_KCC },
	{ "CN", RTW88_REGD_CN,	 RTW88_REGD_CN },
	{ "HK", RTW88_REGD_ETSI, RTW88_REGD_ETSI },
	{ "TW", RTW88_REGD_FCC,	 RTW88_REGD_FCC },
	{ "SG", RTW88_REGD_ETSI, RTW88_REGD_ETSI },
	{ "MY", RTW88_REGD_ETSI, RTW88_REGD_ETSI },
	{ "TH", RTW88_REGD_ETSI, RTW88_REGD_ETSI },
	{ "PH", RTW88_REGD_FCC,	 RTW88_REGD_FCC },
	{ "ID", RTW88_REGD_ETSI, RTW88_REGD_ETSI },
	{ "IN", RTW88_REGD_ETSI, RTW88_REGD_ETSI },
	{ "AE", RTW88_REGD_ETSI, RTW88_REGD_ETSI },
	{ "SA", RTW88_REGD_ETSI, RTW88_REGD_ETSI },
	{ "QA", RTW88_REGD_QATAR, RTW88_REGD_QATAR },
	{ "IL", RTW88_REGD_ETSI, RTW88_REGD_ETSI },
	{ "ZA", RTW88_REGD_ETSI, RTW88_REGD_ETSI },
	{ "BR", RTW88_REGD_FCC,	 RTW88_REGD_FCC },
	{ "AR", RTW88_REGD_FCC,	 RTW88_REGD_FCC },
	{ "",   RTW88_REGD_WW,   RTW88_REGD_WW  }, /* sentinel default */
};

static const struct rtw88_regd_entry *
rtw88_regd_lookup(const char iso2[3])
{
	const struct rtw88_regd_entry *e;

	for (e = rtw88_regd_table; e->country[0] != '\0'; e++) {
		if (e->country[0] == iso2[0] && e->country[1] == iso2[1])
			return (e);
	}
	/* Sentinel WW entry. */
	return (e);
}

void
rtw88_regd_init(struct rtw88_dev *d)
{
	d->regd.country[0] = 'W';	/* WW */
	d->regd.country[1] = 'W';
	d->regd.country[2] = '\0';
	d->regd.regd_2g = RTW88_REGD_WW;
	d->regd.regd_5g = RTW88_REGD_WW;
}

int
rtw88_regd_apply(struct rtw88_dev *d, const char iso2[3])
{
	const struct rtw88_regd_entry *e;

	if (iso2 == NULL)
		return (EINVAL);

	e = rtw88_regd_lookup(iso2);
	d->regd.country[0] = iso2[0];
	d->regd.country[1] = iso2[1];
	d->regd.country[2] = '\0';
	d->regd.regd_2g = e->regd_2g;
	d->regd.regd_5g = e->regd_5g;

	rtw_info(d, "regd: country=%c%c regd_2g=%u regd_5g=%u\n",
	    iso2[0], iso2[1], e->regd_2g, e->regd_5g);
	rtw88_debug_inc(d, RTW88_DBG_CNT_REGD_CHANGE);

	/*
	 * TODO: reprogram chip-side power index table.  Requires the
	 * phy TX-power table port; today the chip inherits its default
	 * power-index at phy_set_param and never changes it.  Not a
	 * regression -- the whole driver has always run at the phy
	 * default.
	 */
	return (0);
}

const struct rtw88_regd_state *
rtw88_regd_get(struct rtw88_dev *d)
{
	return (&d->regd);
}
