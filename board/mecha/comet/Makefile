#
# Copyright 2019 NXP
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += comet.o
obj-y += ../../freescale/common/

ifdef CONFIG_XPL_BUILD	
obj-y += spl.o
ifdef CONFIG_IMX8M_LPDDR4_FREQ0_3200MTS
obj-y += lpddr4_timing_ndm.o
else
obj-$(CONFIG_IMX8M_LPDDR4_4GB) += lpddr4_timing_4gb.o
obj-$(CONFIG_IMX8M_LPDDR4_8GB) += lpddr4_timing_8gb.o
endif
endif
