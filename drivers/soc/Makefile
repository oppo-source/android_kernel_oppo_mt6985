# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the Linux Kernel SOC specific device drivers.
#

obj-$(CONFIG_ARCH_ACTIONS)	+= actions/
obj-y				+= aspeed/
obj-$(CONFIG_ARCH_AT91)		+= atmel/
obj-y				+= bcm/
obj-$(CONFIG_SOC_CANAAN)	+= canaan/
obj-$(CONFIG_ARCH_DOVE)		+= dove/
obj-$(CONFIG_MACH_DOVE)		+= dove/
obj-y				+= fsl/
obj-$(CONFIG_ARCH_GEMINI)	+= gemini/
obj-y				+= imx/
obj-y				+= ixp4xx/
obj-$(CONFIG_SOC_XWAY)		+= lantiq/
obj-$(CONFIG_LITEX_SOC_CONTROLLER) += litex/
obj-y				+= mediatek/
obj-y				+= amlogic/
obj-y				+= qcom/
obj-y				+= renesas/
obj-$(CONFIG_ARCH_ROCKCHIP)	+= rockchip/
obj-$(CONFIG_SOC_SAMSUNG)	+= samsung/
obj-$(CONFIG_SOC_SIFIVE)	+= sifive/
obj-y				+= sunxi/
obj-$(CONFIG_ARCH_TEGRA)	+= tegra/
obj-y				+= ti/
obj-$(CONFIG_ARCH_U8500)	+= ux500/
obj-$(CONFIG_PLAT_VERSATILE)	+= versatile/
obj-y				+= xilinx/

#ifdef OPLUS_FEATURE_OEM_OCDT
obj-$(CONFIG_OPLUS_SYSTEM_KERNEL)	+= oplus/system/
#endif /* OPLUS_FEATURE_OEM_OCDT */

#ifdef OPLUS_TP_BASIC
obj-$(CONFIG_OPLUS_DEVICE_IFNO)		+= oplus/device_info/
#endif /* OPLUS_TP_BASIC */

obj-$(CONFIG_OPLUS_FEATURE_MIDAS_GKI) += oplus/midas/

#ifdef OPLUS_DFR_FEATURE
obj-y += oplus/dfr/
#endif /* OPLUS_DFR_FEATURE */

#ifdef OPLUS_BOOT_FEATURE
obj-y += oplus/boot/
#endif /* OPLUS_BOOT_FEATURE */

#ifdef OPLUS_DFT_FEATURE
obj-y += oplus/dft/
#endif /* OPLUS_DFT_FEATURE */

#ifdef OPLUS_FEATURE_MM_FEEDBACK
obj-$(CONFIG_OPLUS_FEATURE_MM_FEEDBACK)	+= oplus/oplus_mm_fb/
#endif /* OPLUS_FEATURE_MM_FEEDBACK */
obj-y	+= oplus/storage/
