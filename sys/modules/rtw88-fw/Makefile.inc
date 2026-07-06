# Common rules for the rtw88-* firmware kmods.  Modeled on
# sys/modules/rtwnfw/Makefile.inc.  The per-image Makefile sets:
#
#   KMOD	the module name (also the firmware_register name)
#   IMG		the basename of the .fw.uu blob in sys/contrib/dev/rtw88/
#
# Out-of-tree builds need to override SRCTOP to point at this repo so
# the .uu lookup resolves; in-tree consumers get it for free.

SRCTOP?=	${.CURDIR}/../../../..

_FIRM=		${IMG}.fw

CLEANFILES+=	${_FIRM}

FIRMWS=		${_FIRM}:${KMOD}:111

${_FIRM}: ${SRCTOP}/sys/contrib/dev/rtw88/${_FIRM}.uu
	uudecode -p ${.ALLSRC} > ${.TARGET}
