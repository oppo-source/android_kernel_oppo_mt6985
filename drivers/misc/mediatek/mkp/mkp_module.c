// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2002 Richard Henderson
 * Copyright (C) 2001 Rusty Russell, 2002, 2010 Rusty Russell IBM.
 * Copyright (C) 2021 MediaTek Inc.
 */

#include <linux/of.h>
#include "mkp_module.h"
#include "mkp_api.h"

#define DRV_NUM_MAX 128
static const char *drv_skip_list[DRV_NUM_MAX] __ro_after_init;
static int drv_skip_num __ro_after_init;

/* mkp_drv_skip="ko_1", "ko_2", "ko_3", ..., "ko_128"; */
void __init update_drv_skip_by_dts(struct device_node *node)
{
	if (!node)
		return;

	drv_skip_num = of_property_read_string_array(node, "mkp_drv_skip",
			drv_skip_list, DRV_NUM_MAX);

	if (drv_skip_num < 0 && drv_skip_num >= DRV_NUM_MAX) {
		pr_info("%s: no valid mkp_drv_skip(%d)\n", __func__, drv_skip_num);
		drv_skip_num = 0;
	}
}

bool drv_skip(char *module_name)
{
	int i = 0;

	for (i = 0; i < drv_skip_num; i++) {
		if (strcmp(module_name, drv_skip_list[i]) == 0)
			return true;
	}
	return false;
}

static void frob_text(const struct module_layout *layout,
	enum helper_ops ops, uint32_t policy)
{
	int ret;

	BUG_ON((unsigned long)layout->base & (PAGE_SIZE-1));
	BUG_ON((unsigned long)layout->text_size & (PAGE_SIZE-1));
	ret = mkp_set_mapping_xxx_helper((unsigned long)layout->base, layout->text_size>>PAGE_SHIFT,
		policy, ops);
}
static void frob_rodata(const struct module_layout *layout,
	enum helper_ops ops, uint32_t policy)
{
	int ret;

	BUG_ON((unsigned long)layout->base & (PAGE_SIZE-1));
	BUG_ON((unsigned long)layout->text_size & (PAGE_SIZE-1));
	BUG_ON((unsigned long)layout->ro_size & (PAGE_SIZE-1));
	ret = mkp_set_mapping_xxx_helper((unsigned long)layout->base+layout->text_size,
		(layout->ro_size-layout->text_size)>>PAGE_SHIFT, policy, ops);
}
static void frob_ro_after_init(const struct module_layout *layout,
	enum helper_ops ops, uint32_t policy)
{
	int ret;

	BUG_ON((unsigned long)layout->base & (PAGE_SIZE-1));
	BUG_ON((unsigned long)layout->ro_size & (PAGE_SIZE-1));
	BUG_ON((unsigned long)layout->ro_after_init_size & (PAGE_SIZE-1));
	ret = mkp_set_mapping_xxx_helper((unsigned long)layout->base+layout->ro_size,
		(layout->ro_after_init_size-layout->ro_size)>>PAGE_SHIFT, policy, ops);
}
static void frob_writable_data(const struct module_layout *layout,
	enum helper_ops ops, uint32_t policy)
{
	int ret;

	BUG_ON((unsigned long)layout->base & (PAGE_SIZE-1));
	BUG_ON((unsigned long)layout->ro_after_init_size & (PAGE_SIZE-1));
	BUG_ON((unsigned long)layout->size & (PAGE_SIZE-1));
	ret = mkp_set_mapping_xxx_helper((unsigned long)layout->base+layout->ro_after_init_size,
		(layout->size-layout->ro_after_init_size)>>PAGE_SHIFT, policy, ops);
}

void module_enable_x(const struct module *mod, uint32_t policy)
{
	frob_text(&mod->core_layout, HELPER_MAPPING_X, policy);
	if (policy == MKP_POLICY_DRV)
		frob_text(&mod->init_layout, HELPER_MAPPING_X, policy);
}
void module_enable_ro(const struct module *mod, bool after_init, uint32_t policy)
{
	/* DO NOT CHANGE THE FOLLOWING ORDER */
	frob_text(&mod->core_layout, HELPER_MAPPING_RO, policy);
	frob_rodata(&mod->core_layout, HELPER_MAPPING_RO, policy);
	if (policy == MKP_POLICY_DRV) {
		frob_text(&mod->init_layout, HELPER_MAPPING_RO, policy);
		frob_rodata(&mod->init_layout, HELPER_MAPPING_RO, policy);
	}

	if (after_init)
		frob_ro_after_init(&mod->core_layout, HELPER_MAPPING_RO, policy);
}
void module_enable_nx(const struct module *mod, uint32_t policy)
{
	frob_rodata(&mod->core_layout, HELPER_MAPPING_NX, policy);
	/* DO NOT REMOVE THE FOLLOWING STEP */
	frob_ro_after_init(&mod->core_layout, HELPER_MAPPING_NX, policy);
	frob_writable_data(&mod->core_layout, HELPER_MAPPING_NX, policy);
	if (policy == MKP_POLICY_DRV) {
		frob_rodata(&mod->init_layout, HELPER_MAPPING_NX, policy);
		frob_writable_data(&mod->init_layout, HELPER_MAPPING_NX, policy);
	}
}
