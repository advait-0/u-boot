#
# Copyright 2025 NXP
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += comet.o

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
else
obj-$(CONFIG_USB_TCPC) += ../../nxp/common/tcpc.o
obj-$(CONFIG_MMC)      += ../../nxp/common/mmc.o
endif