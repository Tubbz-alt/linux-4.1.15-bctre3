EXTRA_CFLAGS += $(USER_EXTRA_CFLAGS)
EXTRA_CFLAGS += -O1
#EXTRA_CFLAGS += -O3
#EXTRA_CFLAGS += -Wall
#EXTRA_CFLAGS += -Wextra
#EXTRA_CFLAGS += -Werror
#EXTRA_CFLAGS += -pedantic
#EXTRA_CFLAGS += -Wshadow -Wpointer-arith -Wcast-qual -Wstrict-prototypes -Wmissing-prototypes

EXTRA_CFLAGS += -Wno-unused-variable
EXTRA_CFLAGS += -Wno-unused-value
EXTRA_CFLAGS += -Wno-unused-label
EXTRA_CFLAGS += -Wno-unused-parameter
EXTRA_CFLAGS += -Wno-unused-function
EXTRA_CFLAGS += -Wno-unused
EXTRA_CFLAGS += -Wno-error=date-time

#EXTRA_CFLAGS += -Wno-uninitialized

EXTRA_CFLAGS += -I$(src)/include

EXTRA_LDFLAGS += --strip-debug

########################## WIFI IC ############################
CONFIG_RTL8723B = y
########################## Features ###########################
CONFIG_MP_INCLUDED = y
CONFIG_POWER_SAVING = y
CONFIG_USB_AUTOSUSPEND = n
CONFIG_HW_PWRP_DETECTION = n
CONFIG_WIFI_TEST = n
CONFIG_BT_COEXIST = y
CONFIG_RTL8192CU_REDEFINE_1X1 = n
CONFIG_INTEL_WIDI = n
CONFIG_WAPI_SUPPORT = n
CONFIG_EFUSE_CONFIG_FILE = n
CONFIG_EXT_CLK = n
CONFIG_TRAFFIC_PROTECT = y
CONFIG_LOAD_PHY_PARA_FROM_FILE = y
CONFIG_CALIBRATE_TX_POWER_BY_REGULATORY = n
CONFIG_CALIBRATE_TX_POWER_TO_MAX = n
CONFIG_RTW_ADAPTIVITY_EN = auto
CONFIG_RTW_ADAPTIVITY_MODE = normal
CONFIG_SKIP_SIGNAL_SCALE_MAPPING = n
######################## Wake On Lan ##########################
CONFIG_WOWLAN = n
CONFIG_GPIO_WAKEUP = n
CONFIG_PNO_SUPPORT = n
CONFIG_PNO_SET_DEBUG = n
CONFIG_AP_WOWLAN = n
######### Notify SDIO Host Keep Power During Syspend ##########
CONFIG_RTW_SDIO_PM_KEEP_POWER = y
###################### Platform Related #######################
CONFIG_PLATFORM_FS_MX61 = y
###############################################################

CONFIG_DRVEXT_MODULE = n

export TopDIR ?= $(shell pwd)

########### COMMON  #################################

HCI_NAME = usb


_OS_INTFS_FILES :=	os_dep/osdep_service.o \
			os_dep/linux/os_intfs.o \
			os_dep/linux/$(HCI_NAME)_intf.o \
			os_dep/linux/$(HCI_NAME)_ops_linux.o \
			os_dep/linux/ioctl_linux.o \
			os_dep/linux/xmit_linux.o \
			os_dep/linux/mlme_linux.o \
			os_dep/linux/recv_linux.o \
			os_dep/linux/ioctl_cfg80211.o \
			os_dep/linux/wifi_regd.o \
			os_dep/linux/rtw_android.o \
			os_dep/linux/rtw_proc.o

ifeq ($(CONFIG_SDIO_HCI), y)
_OS_INTFS_FILES += os_dep/linux/custom_gpio_linux.o
_OS_INTFS_FILES += os_dep/linux/$(HCI_NAME)_ops_linux.o
endif

ifeq ($(CONFIG_GSPI_HCI), y)
_OS_INTFS_FILES += os_dep/linux/custom_gpio_linux.o
_OS_INTFS_FILES += os_dep/linux/$(HCI_NAME)_ops_linux.o
endif


_HAL_INTFS_FILES :=	hal/hal_intf.o \
			hal/hal_com.o \
			hal/hal_com_phycfg.o \
			hal/hal_phy.o \
			hal/hal_btcoex.o \
			hal/hal_hci/hal_$(HCI_NAME).o \
			hal/led/hal_$(HCI_NAME)_led.o
			
_OUTSRC_FILES := hal/OUTSRC/odm_debug.o	\
		hal/OUTSRC/odm_AntDiv.o\
		hal/OUTSRC/odm_interface.o\
		hal/OUTSRC/odm_HWConfig.o\
		hal/OUTSRC/odm.o\
		hal/OUTSRC/HalPhyRf.o\
		hal/OUTSRC/odm_EdcaTurboCheck.o\
		hal/OUTSRC/odm_DIG.o\
		hal/OUTSRC/odm_PathDiv.o\
		hal/OUTSRC/odm_RaInfo.o\
		hal/OUTSRC/odm_DynamicBBPowerSaving.o\
		hal/OUTSRC/odm_DynamicTxPower.o\
		hal/OUTSRC/odm_CfoTracking.o\
		hal/OUTSRC/odm_NoiseMonitor.o
		
EXTRA_CFLAGS += -I$(src)/platform
_PLATFORM_FILES := platform/platform_ops.o

ifeq ($(CONFIG_BT_COEXIST), y)
EXTRA_CFLAGS += -I$(src)/hal/OUTSRC-BTCoexist
_OUTSRC_FILES += hal/OUTSRC-BTCoexist/HalBtc8188c2Ant.o \
				hal/OUTSRC-BTCoexist/HalBtc8192d2Ant.o \
				hal/OUTSRC-BTCoexist/HalBtc8192e1Ant.o \
				hal/OUTSRC-BTCoexist/HalBtc8192e2Ant.o \
				hal/OUTSRC-BTCoexist/HalBtc8723a1Ant.o \
				hal/OUTSRC-BTCoexist/HalBtc8723a2Ant.o \
				hal/OUTSRC-BTCoexist/HalBtc8723b1Ant.o \
				hal/OUTSRC-BTCoexist/HalBtc8723b2Ant.o \
				hal/OUTSRC-BTCoexist/HalBtc8812a1Ant.o \
				hal/OUTSRC-BTCoexist/HalBtc8812a2Ant.o \
				hal/OUTSRC-BTCoexist/HalBtc8821a1Ant.o \
				hal/OUTSRC-BTCoexist/HalBtc8821a2Ant.o \
				hal/OUTSRC-BTCoexist/HalBtc8821aCsr2Ant.o
endif
		
RTL871X = rtl8723b
MODULE_NAME = 8723bu
EXTRA_CFLAGS += -DCONFIG_RTL8723B

_HAL_INTFS_FILES += hal/HalPwrSeqCmd.o \
					hal/$(RTL871X)/Hal8723BPwrSeq.o\
					hal/$(RTL871X)/$(RTL871X)_sreset.o

_HAL_INTFS_FILES +=	hal/$(RTL871X)/$(RTL871X)_hal_init.o \
			hal/$(RTL871X)/$(RTL871X)_phycfg.o \
			hal/$(RTL871X)/$(RTL871X)_rf6052.o \
			hal/$(RTL871X)/$(RTL871X)_dm.o \
			hal/$(RTL871X)/$(RTL871X)_rxdesc.o \
			hal/$(RTL871X)/$(RTL871X)_cmd.o \
			

_HAL_INTFS_FILES +=	\
			hal/$(RTL871X)/$(HCI_NAME)/$(HCI_NAME)_halinit.o \
			hal/$(RTL871X)/$(HCI_NAME)/rtl$(MODULE_NAME)_led.o \
			hal/$(RTL871X)/$(HCI_NAME)/rtl$(MODULE_NAME)_xmit.o \
			hal/$(RTL871X)/$(HCI_NAME)/rtl$(MODULE_NAME)_recv.o

_HAL_INTFS_FILES += hal/$(RTL871X)/$(HCI_NAME)/$(HCI_NAME)_ops.o

ifeq ($(CONFIG_MP_INCLUDED), y)
_HAL_INTFS_FILES += hal/$(RTL871X)/$(RTL871X)_mp.o
endif

_OUTSRC_FILES += hal/OUTSRC/$(RTL871X)/HalHWImg8723B_BB.o\
								hal/OUTSRC/$(RTL871X)/HalHWImg8723B_MAC.o\
								hal/OUTSRC/$(RTL871X)/HalHWImg8723B_RF.o\
								hal/OUTSRC/$(RTL871X)/HalHWImg8723B_FW.o\
								hal/OUTSRC/$(RTL871X)/HalHWImg8723B_MP.o\
								hal/OUTSRC/$(RTL871X)/odm_RegConfig8723B.o\
								hal/OUTSRC/$(RTL871X)/HalPhyRf_8723B.o\
								hal/OUTSRC/$(RTL871X)/odm_RTL8723B.o


########### END OF PATH  #################################


ifeq ($(CONFIG_USB_AUTOSUSPEND), y)
EXTRA_CFLAGS += -DCONFIG_USB_AUTOSUSPEND
endif

ifeq ($(CONFIG_MP_INCLUDED), y)
EXTRA_CFLAGS += -DCONFIG_MP_INCLUDED
endif

ifeq ($(CONFIG_POWER_SAVING), y)
EXTRA_CFLAGS += -DCONFIG_POWER_SAVING
endif

ifeq ($(CONFIG_HW_PWRP_DETECTION), y)
EXTRA_CFLAGS += -DCONFIG_HW_PWRP_DETECTION
endif

ifeq ($(CONFIG_WIFI_TEST), y)
EXTRA_CFLAGS += -DCONFIG_WIFI_TEST
endif

ifeq ($(CONFIG_BT_COEXIST), y)
EXTRA_CFLAGS += -DCONFIG_BT_COEXIST
endif

ifeq ($(CONFIG_RTL8192CU_REDEFINE_1X1), y)
EXTRA_CFLAGS += -DRTL8192C_RECONFIG_TO_1T1R
endif

ifeq ($(CONFIG_INTEL_WIDI), y)
EXTRA_CFLAGS += -DCONFIG_INTEL_WIDI
endif

ifeq ($(CONFIG_WAPI_SUPPORT), y)
EXTRA_CFLAGS += -DCONFIG_WAPI_SUPPORT
endif


ifeq ($(CONFIG_EFUSE_CONFIG_FILE), y)
EXTRA_CFLAGS += -DCONFIG_EFUSE_CONFIG_FILE
EXTRA_CFLAGS += -DEFUSE_MAP_PATH=\"/system/etc/wifi/wifi_efuse_$(MODULE_NAME).map\"
EXTRA_CFLAGS += -DWIFIMAC_PATH=\"/data/wifimac.txt\"
endif

ifeq ($(CONFIG_EXT_CLK), y)
EXTRA_CFLAGS += -DCONFIG_EXT_CLK
endif

ifeq ($(CONFIG_TRAFFIC_PROTECT), y)
EXTRA_CFLAGS += -DCONFIG_TRAFFIC_PROTECT
endif

ifeq ($(CONFIG_LOAD_PHY_PARA_FROM_FILE), y)
EXTRA_CFLAGS += -DCONFIG_LOAD_PHY_PARA_FROM_FILE
endif

ifeq ($(CONFIG_CALIBRATE_TX_POWER_BY_REGULATORY), y)
EXTRA_CFLAGS += -DCONFIG_CALIBRATE_TX_POWER_BY_REGULATORY
endif

ifeq ($(CONFIG_CALIBRATE_TX_POWER_TO_MAX), y)
EXTRA_CFLAGS += -DCONFIG_CALIBRATE_TX_POWER_TO_MAX
endif

ifeq ($(CONFIG_RTW_ADAPTIVITY_EN), disable)
EXTRA_CFLAGS += -DCONFIG_RTW_ADAPTIVITY_EN=0
else ifeq ($(CONFIG_RTW_ADAPTIVITY_EN), enable)
EXTRA_CFLAGS += -DCONFIG_RTW_ADAPTIVITY_EN=1
else ifeq ($(CONFIG_RTW_ADAPTIVITY_EN), auto)
EXTRA_CFLAGS += -DCONFIG_RTW_ADAPTIVITY_EN=2
endif

ifeq ($(CONFIG_RTW_ADAPTIVITY_MODE), normal)
EXTRA_CFLAGS += -DCONFIG_RTW_ADAPTIVITY_MODE=0
else ifeq ($(CONFIG_RTW_ADAPTIVITY_MODE), carrier_sense)
EXTRA_CFLAGS += -DCONFIG_RTW_ADAPTIVITY_MODE=1
endif

ifeq ($(CONFIG_SKIP_SIGNAL_SCALE_MAPPING), y)
EXTRA_CFLAGS += -DCONFIG_SKIP_SIGNAL_SCALE_MAPPING
endif

ifeq ($(CONFIG_WOWLAN), y)
EXTRA_CFLAGS += -DCONFIG_WOWLAN
ifeq ($(CONFIG_SDIO_HCI), y)
EXTRA_CFLAGS += -DCONFIG_RTW_SDIO_PM_KEEP_POWER
endif
endif

ifeq ($(CONFIG_AP_WOWLAN), y)
EXTRA_CFLAGS += -DCONFIG_AP_WOWLAN
ifeq ($(CONFIG_SDIO_HCI), y)
EXTRA_CFLAGS += -DCONFIG_RTW_SDIO_PM_KEEP_POWER
endif
endif

ifeq ($(CONFIG_PNO_SUPPORT), y)
EXTRA_CFLAGS += -DCONFIG_PNO_SUPPORT
ifeq ($(CONFIG_PNO_SET_DEBUG), y)
EXTRA_CFLAGS += -DCONFIG_PNO_SET_DEBUG
endif
endif

ifeq ($(CONFIG_GPIO_WAKEUP), y)
EXTRA_CFLAGS += -DCONFIG_GPIO_WAKEUP
endif

ifeq ($(CONFIG_PLATFORM_FS_MX61), y)
EXTRA_CFLAGS += -DCONFIG_LITTLE_ENDIAN 
ARCH := arm
CROSS_COMPILE := /home/osboxes/projects/gcc-linaro-4.9-2014.11-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-
KSRC ?= /home/osboxes/projects/bsp/linux/rel_imx_4.1.15_1.0.0_ga 
endif

ifneq ($(USER_MODULE_NAME),)
MODULE_NAME := $(USER_MODULE_NAME)
endif

ifneq ($(KERNELRELEASE),)

rtk_core :=	core/rtw_cmd.o \
		core/rtw_security.o \
		core/rtw_debug.o \
		core/rtw_io.o \
		core/rtw_ioctl_query.o \
		core/rtw_ioctl_set.o \
		core/rtw_ieee80211.o \
		core/rtw_mlme.o \
		core/rtw_mlme_ext.o \
		core/rtw_wlan_util.o \
		core/rtw_vht.o \
		core/rtw_pwrctrl.o \
		core/rtw_rf.o \
		core/rtw_recv.o \
		core/rtw_sta_mgt.o \
		core/rtw_ap.o \
		core/rtw_xmit.o	\
		core/rtw_p2p.o \
		core/rtw_tdls.o \
		core/rtw_br_ext.o \
		core/rtw_iol.o \
		core/rtw_sreset.o \
		core/rtw_btcoex.o \
		core/rtw_beamforming.o \
		core/rtw_odm.o \
		core/efuse/rtw_efuse.o

$(MODULE_NAME)-y += $(rtk_core)

$(MODULE_NAME)-$(CONFIG_INTEL_WIDI) += core/rtw_intel_widi.o

$(MODULE_NAME)-$(CONFIG_WAPI_SUPPORT) += core/rtw_wapi.o	\
					core/rtw_wapi_sms4.o
					
$(MODULE_NAME)-y += $(_OS_INTFS_FILES)
$(MODULE_NAME)-y += $(_HAL_INTFS_FILES)
$(MODULE_NAME)-y += $(_OUTSRC_FILES)
$(MODULE_NAME)-y += $(_PLATFORM_FILES)

$(MODULE_NAME)-$(CONFIG_MP_INCLUDED) += core/rtw_mp.o \
					core/rtw_mp_ioctl.o

ifeq ($(CONFIG_RTL8723B), y)
$(MODULE_NAME)-$(CONFIG_MP_INCLUDED)+= core/rtw_bt_mp.o
endif

obj-$(CONFIG_RTL8723BU) := $(MODULE_NAME).o

ccflags-y += $(EXTRA_CFLAGS) 
ccflags-y += -Idrivers/net/wireless/rtl8723

else

export CONFIG_RTL8723BU = m

all: modules

modules:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KSRC) M=$(shell pwd)  modules

strip:
	$(CROSS_COMPILE)strip $(MODULE_NAME).ko --strip-unneeded

install:
	install -p -m 644 $(MODULE_NAME).ko  $(MODDESTDIR)
	/sbin/depmod -a ${KVER}

uninstall:
	rm -f $(MODDESTDIR)/$(MODULE_NAME).ko
	/sbin/depmod -a ${KVER}

.PHONY: modules clean

clean:
	cd hal/OUTSRC/ ; rm -fr */*.mod.c */*.mod */*.o */.*.cmd */*.ko
	cd hal/OUTSRC/ ; rm -fr *.mod.c *.mod *.o .*.cmd *.ko 
	cd hal/led ; rm -fr *.mod.c *.mod *.o .*.cmd *.ko
	cd hal ; rm -fr */*/*.mod.c */*/*.mod */*/*.o */*/.*.cmd */*/*.ko
	cd hal ; rm -fr */*.mod.c */*.mod */*.o */.*.cmd */*.ko
	cd hal ; rm -fr *.mod.c *.mod *.o .*.cmd *.ko
	cd core/efuse ; rm -fr *.mod.c *.mod *.o .*.cmd *.ko
	cd core ; rm -fr *.mod.c *.mod *.o .*.cmd *.ko
	cd os_dep/linux ; rm -fr *.mod.c *.mod *.o .*.cmd *.ko
	cd os_dep ; rm -fr *.mod.c *.mod *.o .*.cmd *.ko
	cd platform ; rm -fr *.mod.c *.mod *.o .*.cmd *.ko
	rm -fr Module.symvers ; rm -fr Module.markers ; rm -fr modules.order
	rm -fr *.mod.c *.mod *.o .*.cmd *.ko *~
	rm -fr .tmp_versions
endif

