/* * Automatically @generated */
#include <sys/param.h>
#include <sys/errno.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/linker.h>
#include <sys/firmware.h>
#include <sys/systm.h>

extern char _binary_rtw88_rtl8821ctbl_fw_start[], _binary_rtw88_rtl8821ctbl_fw_end[];

static int
rtw88_rtl8821ctbl_fw_modevent(module_t mod, int type, void *unused){	const struct firmware *fp;
	int error;	switch (type) {	case MOD_LOAD:

		fp = firmware_register("rtw88-rtl8821ctbl", _binary_rtw88_rtl8821ctbl_fw_start , (size_t)(_binary_rtw88_rtl8821ctbl_fw_end - _binary_rtw88_rtl8821ctbl_fw_start), 111, NULL);
		if (fp == NULL)
			goto fail_0;
		return (0);
	fail_0:
		return (ENXIO);
	case MOD_UNLOAD:
		error = firmware_unregister("rtw88-rtl8821ctbl");
		return (error);	}	return (EINVAL);}static moduledata_t rtw88_rtl8821ctbl_fw_mod = {        "rtw88_rtl8821ctbl_fw",        rtw88_rtl8821ctbl_fw_modevent,        0};DECLARE_MODULE(rtw88_rtl8821ctbl_fw, rtw88_rtl8821ctbl_fw_mod, SI_SUB_DRIVERS, SI_ORDER_FIRST);MODULE_VERSION(rtw88_rtl8821ctbl_fw, 1);MODULE_DEPEND(rtw88_rtl8821ctbl_fw, firmware, 1, 1, 1);
