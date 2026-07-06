# Out-of-tree kmod for the rtw88 USB driver port.  Sources live in the
# sibling sys/dev/rtw88/ directory under this repo, but the build
# consumes the active FreeBSD source tree (SRCTOP defaults to /usr/src
# when running from /usr/src).  Run as:
#
#   cd /home/b1nc0d3x/rtw88/sys/modules/rtw88
#   make SYSDIR=/usr/src/sys
#
.PATH:	${.CURDIR}/../../dev/rtw88

KMOD=	if_rtw88_usb

SRCS=	if_rtw88_usb.c

# Subsystem TUs -- bus/chip-abstract, consumed by the USB front-end.
SRCS+=	rtw88_efuse.c \
	rtw88_coex.c \
	rtw88_ps.c \
	rtw88_bf.c \
	rtw88_led.c \
	rtw88_regd.c \
	rtw88_debug.c

# Standard kmod headers + bus_if/device_if/usb_if generated stubs land
# automatically via bsd.kmod.mk.
SRCS+=	bus_if.h device_if.h usb_if.h opt_usb.h

CFLAGS+=	-I${.CURDIR}/../../dev/rtw88

.include <bsd.kmod.mk>
