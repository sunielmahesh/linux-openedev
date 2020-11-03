r8822cs-os-drv-y += \
		osdep_service.o \
		linux/os_intfs.o \
		linux/ioctl_linux.o \
		linux/xmit_linux.o \
		linux/mlme_linux.o \
		linux/recv_linux.o \
		linux/ioctl_cfg80211.o \
		linux/rtw_cfgvendor.o \
		linux/wifi_regd.o \
		linux/rtw_android.o \
		linux/rtw_proc.o \
		linux/rtw_rhashtable.o

r8822cs-os-sdio-y += \
		linux/custom_gpio_linux.o \
		linux/sdio_intf.o \
		linux/sdio_ops_linux.o

obj-$(CONFIG_RTL8822CS_OS_DRV) += r8822cs-os-drv.o
obj-$(CONFIG_SDIO_HCI) += r8822cs-os-sdio.o
obj-$(CONFIG_MP_INCLUDED) += linux/ioctl_mp.o
