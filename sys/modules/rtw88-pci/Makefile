# Skeleton PCI front-end for rtw88 — probe/BAR/IRQ only.
#
# Run as:
#   cd /home/b1nc0d3x/rtw88/sys/modules/rtw88-pci
#   make SYSDIR=/usr/src/sys

.PATH:	${.CURDIR}/../../dev/rtw88

KMOD=	if_rtw88_pci

SRCS=	if_rtw88_pci.c \
	device_if.h bus_if.h pci_if.h

.include <bsd.kmod.mk>
