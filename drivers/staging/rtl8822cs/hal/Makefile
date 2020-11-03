r8822cs-hal-drv-y += \
		hal_intf.o \
		hal_com.o \
		hal_com_phycfg.o \
		hal_phy.o \
		hal_dm.o \
		hal_dm_acs.o \
		hal_btcoex_wifionly.o \
		hal_btcoex.o \
		hal_mp.o \
		hal_mcc.o \
		led/hal_led.o

r8822cs-hal-sdio-y += \
		hal_hci/hal_sdio.o \
		led/hal_sdio_led.o

obj-$(CONFIG_RTL8822CS_HAL_DRV) += r8822cs-hal-drv.o
obj-$(CONFIG_SDIO_HCI) += r8822cs-hal-sdio.o
