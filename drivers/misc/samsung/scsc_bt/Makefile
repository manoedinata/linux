# SPDX-License-Identifier: GPL-2.0
#
# Maxwell BT
obj-$(CONFIG_SCSC_BT) += scsc_bt.o
scsc_bt-y += scsc_bt_module.o scsc_shm.o scsc_avdtp_detect.o
scsc_bt-$(CONFIG_SCSC_BT_BLUEZ) += scsc_bluez.o

ccflags-y += -I$(srctree)/include/scsc
ccflags-y += -I$(src)/../scsc/inc -include $(src)/../scsc/compat.h
ccflags-y += $(CONFIG_SAMSUNG_MAXWELL_EXTRA)
ccflags-$(CONFIG_SCSC_DEBUG) += -DCONFIG_SCSC_PRINTK
