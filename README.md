# rtw88 — Realtek RTL8821CU / RTL8811CU USB driver for FreeBSD

A FreeBSD port of the Linux `rtw88` driver family covering the
RTL8821CU / RTL8811CU 802.11ac USB silicon.  Probe table matches the
Linux upstream `rtw_8821cu_id_table` (15 USB VID/PID pairs):

- Realtek (`0x0bda`): `0x2006`, `0x8731`, `0x8811`, `0xb820`, `0xb82b`,
  `0xc80c`, `0xc811`, `0xc820`, `0xc821`, `0xc82a`, `0xc82b`
- D-Link (`0x2001`): `0x331d`
- Edimax (`0x7392`): `0xc811`, `0xd811`
- Mercusys (`0x2c4e`): `0x0105`

STA mode, single VAP, HT20 1T1R, 2.4 GHz + 5 GHz.  SW CCMP by default
(validated end-to-end); HW CCMP and A-MPDU TX aggregation are gated
behind tunables awaiting hardware validation.

## Build

```
cd sys/modules/rtw88
make SYSDIR=/usr/src/sys
sudo cp if_rtw88_usb.ko /boot/modules/

cd ../rtw88-fw
sudo make install     # installs rtw88-rtl8821cufw.ko + rtw88-rtl8821ctbl.ko
```

Then:

```
sudo kldload if_rtw88_usb
sudo ifconfig wlan0 create wlandev rtw88_usb0
sudo wpa_supplicant -B -i wlan0 -c /etc/wpa_supplicant.conf
sudo dhclient wlan0
```

## Tunables

All settable in `loader.conf` (or `sysctl hw.rtw88_usb.*` before
`kldload`).  Latched at attach.

| Tunable                          | Default | What it does                                      |
| -------------------------------- | ------- | ------------------------------------------------- |
| `hw.rtw88_usb.hw_ccmp`           | `0`     | `0` = SW CCMP (validated). `1` = HW CCMP via CAM. |
| `hw.rtw88_usb.ampdu`             | `0`     | `1` = advertise `HTC_AMPDU` + enable TX `AGG_EN`. |
| `hw.rtw88_usb.ampdu_max_num`     | `31`    | AMPDU max sub-MPDUs per burst (1..31).            |
| `hw.rtw88_usb.ampdu_density`     | `4`     | AMPDU density code (0..7; 4 = 8us).               |
| `hw.rtw88_usb.tx_data_qdepth`    | `256`   | Data TX queue depth.                              |
| `hw.rtw88_usb.tx_mgmt_qdepth`    | `64`    | Mgmt TX queue depth.                              |
| `hw.rtw88_usb.cdev_enable`       | `0`     | `1` = create `/dev/rtw88_usbN` RPC-bridge cdev (chip RE only). |

HW CCMP and AMPDU are opt-in and require hardware validation before
turning on in production.

## Operator counters

Read-only counters under `dev.rtw88_usb.N.stats.*`:

| Counter        | Meaning                                                   |
| -------------- | --------------------------------------------------------- |
| `tx_packets`   | Mbufs successfully sent to the chip.                      |
| `tx_drops`     | Mbufs dropped at enqueue (queue full / oversize).         |
| `tx_errors`    | USB TX xfer errors reported by the host controller.       |
| `rx_packets`   | Bulk-IN completions since attach.                         |
| `rx_frames`    | Mbufs handed up to net80211.                              |
| `rx_errors`    | RX frames with CRC or ICV error in the descriptor.        |
| `rx_drops`    | RX frames dropped (mbuf alloc failure, oversize).         |
| `usb_errors`   | EP0 vendor-request read/write failures.                   |
| `cam_fails`    | `iv_key_set` queue overflow (CAM install dropped).        |
| `fw_stalls`    | Watchdog detected firmware TSFTR frozen for ~3s.          |
| `c2h`          | C2H firmware-to-host events received.                     |
| `c2h_unknown`  | C2H events with unrecognised sub-cmd id.                  |
| `rx_cck`       | RX frames at CCK rates (1/2/5.5/11 Mbps).                 |
| `rx_ofdm`      | RX frames at OFDM rates (6..54 Mbps).                     |
| `rx_mcs`       | RX frames at HT MCS rates.                                |
| `link_rssi`    | EWMA of link RSSI (signal_dbm + 100, range 0..100).       |
| `tx_acked`     | CCX TX reports with success status from firmware.         |
| `tx_xretries`  | CCX TX reports with retry-exhausted status from firmware. |
| `rate_id`      | Current RA table index (3 / 5 / 6 / 7).                   |
| `channel`      | Chip-tuned channel number.                                |
| `tx_data_qlen` | Live data-frame TX backlog (capped by `tx_data_qdepth`).  |
| `tx_mgmt_qlen` | Live mgmt-frame TX backlog (capped by `tx_mgmt_qdepth`).  |
| `tx_pending_qlen` | Protected-unicast frames held mid-handshake.           |
| `tx_data_qlen_max` | High-watermark of `tx_data_qlen` since `reset`.       |
| `tx_mgmt_qlen_max` | High-watermark of `tx_mgmt_qlen` since `reset`.       |
| `tx_pending_qlen_max` | High-watermark of `tx_pending_qlen` since `reset`. |
| `reset`        | Write any value to zero all stats counters.               |

Plus, outside the stats subtree:
- `dev.rtw88_usb.N.version` — driver version string.
- `dev.rtw88_usb.N.chip_name` — chip family ("RTL8821CU/8811CU 11ac" today).
- `dev.rtw88_usb.N.chip_id` — chip id integer (matches firmware sig).
- `dev.rtw88_usb.N.chip_caps` — OR of `RTW88_CAP_*` bits supported.

## Diagnosing problems

Start with the stats counter triage:

```
sysctl dev.rtw88_usb.0.stats
```

| Symptom                          | Counter to check  | What it means                       |
| -------------------------------- | ----------------- | ----------------------------------- |
| Slow throughput                  | `rx_cck` vs `rx_mcs` | All-CCK -> AP not negotiating HT |
| Random disconnects               | `fw_stalls`       | Chip RX wedged (kldunload+kldload to recover) |
| TX fail under load               | `tx_errors`, `tx_drops` | USB bus or chip queue saturation |
| Repeated 4-way failures          | `cam_fails`       | iv_key_set queue overflow during roam |
| EAPOL never arriving             | `usb_errors`      | EP0 vendor request errors |
| Weak signal complaints           | `link_rssi`       | < 30 = below -70 dBm |
| Investigating chip MCU activity  | `c2h`             | Should advance during association + traffic |

If all stats look clean but data still won't flow: capture from
the AP side too — the SW CCMP TX path in this driver is
byte-parity-checked against Linux upstream.

## What works / doesn't

Works: SCAN, AUTH, ASSOC, WPA2 4-way handshake, ICMP / DHCP /
general IPv4 traffic, channel switch between 2.4 GHz and 5 GHz,
graceful suspend/resume (`ieee80211_suspend_all`/`_resume_all`),
monitor mode + radiotap (RX and TX), promiscuous capture.

Doesn't yet: BT coex, HW scan offload, beamforming, dynamic TX power
control, real PHY IQK calibration.  Best-effort native ports (HT40,
VHT80, A-MSDU, LPS) are wired but unvalidated on hardware in this
release cycle.

## License

BSD-2-Clause.  See `LICENSE`.

## Acknowledgements

Patterns + register layouts referenced from upstream Linux `rtw88`
(`drivers/net/wireless/realtek/rtw88/`).  The FreeBSD `rtwn(4)` driver
provided the USB transport template for the Realtek vendor-request
EP0 protocol.
