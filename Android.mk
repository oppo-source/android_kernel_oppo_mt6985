# SPDX-License-Identifier: GPL-2.0
# Copyright (C) 2019 MediaTek Inc.

LOCAL_PATH := $(call my-dir)

ifeq ($(notdir $(LOCAL_PATH)),$(strip $(LINUX_KERNEL_VERSION)))

include $(LOCAL_PATH)/kenv.mk

ifeq ($(wildcard $(TARGET_PREBUILT_KERNEL)),)
KERNEL_MAKE_DEPENDENCIES := $(shell find $(KERNEL_DIR) -name .git -prune -o -type f | sort)
KERNEL_MAKE_DEPENDENCIES += $(shell find vendor/mediatek/kernel_modules -name .git -prune -o -type f | sort)
ifdef MTK_GKI_PREBUILTS_DIR
KERNEL_MAKE_DEPENDENCIES += $(wildcard $(MTK_GKI_PREBUILTS_DIR)/*)
endif
ifdef MTK_GKI_BUILD_CONFIG
KERNEL_MAKE_DEPENDENCIES += $(shell find kernel/common-5.15 -name .git -prune -o -type f | sort)
endif

$(GEN_KERNEL_BUILD_CONFIG): PRIVATE_KERNEL_DEFCONFIG := $(KERNEL_DEFCONFIG)
$(GEN_KERNEL_BUILD_CONFIG): PRIVATE_KERNEL_DEFCONFIG_OVERLAYS := $(KERNEL_DEFCONFIG_OVERLAYS)
$(GEN_KERNEL_BUILD_CONFIG): PRIVATE_KERNEL_BUILD_CONFIG := $(REL_GEN_KERNEL_BUILD_CONFIG)
$(GEN_KERNEL_BUILD_CONFIG): PRIVATE_KERNEL_BUILD_CONFIG_OVERLAYS := $(addprefix $(KERNEL_DIR)/,$(KERNEL_BUILD_CONFIG_OVERLAYS))
$(GEN_KERNEL_BUILD_CONFIG): $(KERNEL_DIR)/kernel/configs/ext_modules.list
$(GEN_KERNEL_BUILD_CONFIG): $(KERNEL_DIR)/scripts/gen_build_config.py $(wildcard $(KERNEL_DIR)/build.config.*) $(build_config_file) $(KERNEL_CONFIG_FILE) $(LOCAL_PATH)/Android.mk
	$(hide) mkdir -p $(dir $@)
	$(hide) cd kernel && python $< --kernel-defconfig $(PRIVATE_KERNEL_DEFCONFIG) --kernel-defconfig-overlays "$(PRIVATE_KERNEL_DEFCONFIG_OVERLAYS)" --kernel-build-config-overlays "$(PRIVATE_KERNEL_BUILD_CONFIG_OVERLAYS)" -m $(TARGET_BUILD_VARIANT) -o $(PRIVATE_KERNEL_BUILD_CONFIG) && cd ..

.KATI_RESTAT: $(TARGET_KERNEL_CONFIG)
$(TARGET_KERNEL_CONFIG): PRIVATE_KERNEL_OUT := $(REL_KERNEL_OUT)
$(TARGET_KERNEL_CONFIG): PRIVATE_DIST_DIR := $(REL_KERNEL_OUT)
$(TARGET_KERNEL_CONFIG): PRIVATE_CC_WRAPPER := $(CCACHE_EXEC)
$(TARGET_KERNEL_CONFIG): PRIVATE_KERNEL_BUILD_CONFIG := $(REL_GEN_KERNEL_BUILD_CONFIG)
$(TARGET_KERNEL_CONFIG): $(wildcard kernel/build/*.sh) $(GEN_KERNEL_BUILD_CONFIG) $(KERNEL_MAKE_DEPENDENCIES) | kernel-outputmakefile
	$(hide) mkdir -p $(dir $@)
	if [ -f $@ ]; then cp -f -p $@ $@.timestamp; else touch $@.timestamp; fi
	$(hide) cd kernel && ENABLE_GKI_CHECKER=$(ENABLE_GKI_CHECKER) CC_WRAPPER=$(PRIVATE_CC_WRAPPER) SKIP_MRPROPER=1 BUILD_CONFIG=$(PRIVATE_KERNEL_BUILD_CONFIG) OUT_DIR=$(PRIVATE_KERNEL_OUT) DIST_DIR=$(PRIVATE_DIST_DIR) POST_DEFCONFIG_CMDS="exit 0" ./build/build.sh && cd ..
	if ! cmp -s $@.timestamp $@; then rm -f $@.timestamp; else mv -f $@.timestamp $@; fi

ifeq (yes,$(strip $(BUILD_KERNEL)))
$(KERNEL_ZIMAGE_OUT): PRIVATE_DIR := $(KERNEL_DIR)
$(KERNEL_ZIMAGE_OUT): PRIVATE_KERNEL_OUT := $(REL_KERNEL_OUT)
$(KERNEL_ZIMAGE_OUT): PRIVATE_DIST_DIR := $(REL_KERNEL_OUT)
$(KERNEL_ZIMAGE_OUT): PRIVATE_CC_WRAPPER := $(CCACHE_EXEC)
$(KERNEL_ZIMAGE_OUT): PRIVATE_KERNEL_BUILD_CONFIG := $(REL_GEN_KERNEL_BUILD_CONFIG)
ifeq (user,$(strip $(KERNEL_BUILD_VARIANT)))
  ifeq ($(KERNEL_GKI_CONFIG),)
ifneq (,$(strip $(shell grep "^CONFIG_ABI_MONITOR\s*=\s*y" $(KERNEL_CONFIG_FILE))))
$(KERNEL_ZIMAGE_OUT): PRIVATE_KERNEL_BUILD_SCRIPT := ./build/build_abi.sh
else
$(KERNEL_ZIMAGE_OUT): PRIVATE_KERNEL_BUILD_SCRIPT := ./build/build.sh
endif
$(KERNEL_ZIMAGE_OUT): PRIVATE_KERNEL_GKI_CONFIG :=
  else
$(KERNEL_ZIMAGE_OUT): PRIVATE_KERNEL_BUILD_SCRIPT := ./build/build.sh
$(KERNEL_ZIMAGE_OUT): PRIVATE_KERNEL_GKI_CONFIG := $(KERNEL_GKI_CONFIG)
  endif
else
$(KERNEL_ZIMAGE_OUT): PRIVATE_KERNEL_BUILD_SCRIPT := ./build/build.sh
$(KERNEL_ZIMAGE_OUT): PRIVATE_KERNEL_GKI_CONFIG :=
endif

PRIVATE_KERNEL_MAKE_OPTION := CHIPSET_COMPANY=$(CHIPSET_COMPANY)
PRIVATE_KERNEL_MAKE_OPTION += OPLUS_VND_BUILD_PLATFORM=$(OPLUS_VND_BUILD_PLATFORM)

#ifdef OPLUS_EDIT
PRIVATE_KERNEL_MAKE_OPTION += CONFIG_LTO_CLANG_THIN=$(CONFIG_LTO_CLANG_THIN)
#end

ifeq (user,$(strip $(TARGET_BUILD_VARIANT)))
$(KERNEL_ZIMAGE_OUT): $(TARGET_KERNEL_CONFIG) $(KERNEL_MAKE_DEPENDENCIES)
	$(hide) mkdir -p $(dir $@)
	$(hide) cd kernel && $(PRIVATE_KERNEL_MAKE_OPTION) CC_WRAPPER=$(PRIVATE_CC_WRAPPER) SKIP_MRPROPER=1 BUILD_CONFIG=$(PRIVATE_KERNEL_BUILD_CONFIG) OUT_DIR=$(PRIVATE_KERNEL_OUT) DIST_DIR=$(PRIVATE_DIST_DIR) SKIP_DEFCONFIG=1 $(PRIVATE_KERNEL_GKI_CONFIG)  $(PRIVATE_KERNEL_BUILD_SCRIPT) && cd ..
ifneq ($(KERNEL_GKI_CONFIG),)
ifeq ($(MTK_KERNEL_COMPRESS_FORMAT),gz)
	$(hide) export PATH=kernel/build/kernel/build-tools/path/linux-x86:$$PATH && lz4 -df $(patsubst %.gz,%.lz4,$@) $(patsubst %.gz,%.uncompress,$@) && gzip -nc $(patsubst %.gz,%.uncompress,$@) > $@
endif
endif
	$(hide) $(call fixup-kernel-cmd-file,$(KERNEL_OUT)/arch/$(KERNEL_TARGET_ARCH)/boot/compressed/.piggy.xzkern.cmd)
else
$(KERNEL_ZIMAGE_OUT): $(TARGET_KERNEL_CONFIG) $(KERNEL_MAKE_DEPENDENCIES)
	$(hide) mkdir -p $(dir $@)
	$(hide) cd kernel && $(PRIVATE_KERNEL_MAKE_OPTION) CC_WRAPPER=$(PRIVATE_CC_WRAPPER) SKIP_MRPROPER=1 BUILD_CONFIG=$(PRIVATE_KERNEL_BUILD_CONFIG) OUT_DIR=$(PRIVATE_KERNEL_OUT) DIST_DIR=$(PRIVATE_DIST_DIR) SKIP_DEFCONFIG=1 LTO=thin $(PRIVATE_KERNEL_GKI_CONFIG)  $(PRIVATE_KERNEL_BUILD_SCRIPT) && cd ..
ifneq ($(KERNEL_GKI_CONFIG),)
ifeq ($(MTK_KERNEL_COMPRESS_FORMAT),gz)
	$(hide) export PATH=kernel/build/kernel/build-tools/path/linux-x86:$$PATH && lz4 -df $(patsubst %.gz,%.lz4,$@) $(patsubst %.gz,%.uncompress,$@) && gzip -nc $(patsubst %.gz,%.uncompress,$@) > $@
endif
endif
	$(hide) $(call fixup-kernel-cmd-file,$(KERNEL_OUT)/arch/$(KERNEL_TARGET_ARCH)/boot/compressed/.piggy.xzkern.cmd)
endif

$(TARGET_PREBUILT_KERNEL): $(BUILT_KERNEL_TARGET) $(LOCAL_PATH)/Android.mk | $(ACP)
	$(copy-file-to-new-target)

endif#BUILD_KERNEL
endif #TARGET_PREBUILT_KERNEL is empty

ifeq (yes,$(strip $(BUILD_KERNEL)))
ifneq ($(strip $(TARGET_NO_KERNEL)),true)
$(INSTALLED_KERNEL_TARGET): $(BUILT_KERNEL_TARGET) $(LOCAL_PATH)/Android.mk | $(ACP)
	$(copy-file-to-target)
endif#TARGET_NO_KERNEL
endif#BUILD_KERNEL

.PHONY: kernel save-kernel kernel-savedefconfig kernel-menuconfig menuconfig-kernel savedefconfig-kernel clean-kernel
kernel: $(INSTALLED_KERNEL_TARGET)
save-kernel: $(TARGET_PREBUILT_KERNEL)

kernel-savedefconfig: $(TARGET_KERNEL_CONFIG)
	cp $(TARGET_KERNEL_CONFIG) $(KERNEL_CONFIG_FILE)

kernel-menuconfig:
	$(hide) mkdir -p $(KERNEL_OUT)
	$(PREBUILT_MAKE_PREFIX)/$(MAKE) -C $(KERNEL_DIR) $(KERNEL_MAKE_OPTION) menuconfig

menuconfig-kernel savedefconfig-kernel:
	$(hide) mkdir -p $(KERNEL_OUT)
	$(PREBUILT_MAKE_PREFIX)/$(MAKE) -C $(KERNEL_DIR) $(KERNEL_MAKE_OPTION) $(patsubst %config-kernel,%config,$@)

clean-kernel:
	$(hide) rm -rf $(KERNEL_OUT) $(INSTALLED_KERNEL_TARGET)

.PHONY: kernel-outputmakefile
kernel-outputmakefile: PRIVATE_DIR := $(KERNEL_DIR)
kernel-outputmakefile: PRIVATE_KERNEL_OUT := $(REL_KERNEL_OUT)/$(LINUX_KERNEL_VERSION)
kernel-outputmakefile:
	$(PREBUILT_MAKE_PREFIX)/$(MAKE) -C $(PRIVATE_DIR) O=$(PRIVATE_KERNEL_OUT) outputmakefile

### DTB build template
MTK_DTBIMAGE_DTS := $(addsuffix .dts,$(addprefix $(KERNEL_DIR)/arch/$(KERNEL_TARGET_ARCH)/boot/dts/,$(PLATFORM_DTB_NAME)))
include device/mediatek/build/core/build_dtbimage.mk

MTK_DTBOIMAGE_DTS := $(addsuffix .dts,$(addprefix $(KERNEL_DIR)/arch/$(KERNEL_TARGET_ARCH)/boot/dts/,$(PROJECT_DTB_NAMES)))
include device/mediatek/build/core/build_dtboimage.mk

endif #LINUX_KERNEL_VERSION
