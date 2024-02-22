/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#ifndef _MTK_DRM_RECOVERY_H
#define _MTK_DRM_RECOVERY_H

struct mtk_drm_private;

enum mtk_esd_chk_mode {
	READ_EINT,
	READ_LCM,
};

struct mtk_drm_esd_ctx {
	struct task_struct *disp_esd_chk_task;
	struct drm_crtc *crtc;
	struct timer_list esd_timer;
	struct mutex lock;
	wait_queue_head_t check_task_wq;
	wait_queue_head_t ext_te_wq;
	atomic_t ext_te_event;
	atomic_t check_wakeup;
	atomic_t target_time;
	int eint_irq;
	u32 chk_active;
	u32 chk_mode;
	u32 chk_sta;
	u32 chk_en;
};
int mtk_drm_esd_testing_process(struct mtk_drm_esd_ctx *esd_ctx, bool need_lock);
void mtk_disp_esd_attach_crtc(struct drm_crtc *crtc, struct mtk_ddp_comp *output_comp);
void mtk_disp_esd_check_switch(struct drm_crtc *crtc, bool enable);
void mtk_disp_chk_recover_init(struct drm_crtc *crtc);
long disp_dts_gpio_init(struct device *dev, struct mtk_drm_private *private);
long _set_state(struct drm_crtc *crtc, const char *name);

#endif
