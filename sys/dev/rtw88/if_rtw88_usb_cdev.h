/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * Userland-facing protocol for /dev/rtw88_usb0.  Exposes raw EP0
 * vendor-request register reads and writes so userspace (and a TCP
 * bridge to a Linux/QEMU guest running an instrumented rtw88 driver)
 * can drive the chip without taking it off FreeBSD.  Companion to the
 * cdev-RPC pattern proven on bwfm_sdio.
 */

#ifndef _DEV_RTW88_IF_RTW88_USB_CDEV_H_
#define _DEV_RTW88_IF_RTW88_USB_CDEV_H_

#include <sys/ioccom.h>
#include <sys/types.h>

#define RTW88_USB_CDEV_NAME	"rtw88_usb"

/*
 * One 4-byte register access at the given 16-bit chip address.
 * Width is one of 1, 2, 4 (rejected otherwise).  For reads, the
 * driver fills `value` and clears `err` to 0 on success; for writes,
 * the userland-supplied `value` is sent and `err` reflects the
 * usb_error_t (cast to int) returned by the EP0 transfer.
 */
struct rtw88_cdev_reg {
	uint16_t	addr;		/* chip register offset */
	uint8_t		width;		/* 1, 2, or 4 */
	uint8_t		_pad;
	uint32_t	value;		/* in for write, out for read */
	int32_t		err;		/* output: 0 on success, errno */
};

#define RTW88_CDEV_IOC_READ	_IOWR('W', 1, struct rtw88_cdev_reg)
#define RTW88_CDEV_IOC_WRITE	_IOWR('W', 2, struct rtw88_cdev_reg)

#endif /* _DEV_RTW88_IF_RTW88_USB_CDEV_H_ */
