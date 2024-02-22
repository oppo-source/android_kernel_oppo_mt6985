// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018 MediaTek Inc.
 * Author: Longfei Wang <longfei.wang@mediatek.com>
 */

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/slab.h>

#include "../mtk_vcodec_drv.h"
#include "../mtk_vcodec_util.h"
#include "../mtk_vcodec_enc.h"
#include "../venc_drv_base.h"
#include "../venc_ipi_msg.h"
#include "../venc_vcu_if.h"
#include "mtk_vcodec_enc_pm.h"
#include "mtk_vcodec_intr.h"
#include "mtk_vcu.h"



static unsigned int venc_h265_get_profile(struct venc_inst *inst,
	unsigned int profile)
{
	switch (profile) {
	case V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN:
		return 1;
	case V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10:
		return 2;
	case V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_STILL_PICTURE:
		return 4;
	default:
		mtk_vcodec_debug(inst, "unsupported profile %d", profile);
		return 1;
	}
}

static unsigned int venc_h265_get_level(struct venc_inst *inst,
	unsigned int level, unsigned int tier)
{
	switch (level) {
	case V4L2_MPEG_VIDEO_HEVC_LEVEL_1:
		return (tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN) ? 2 : 3;
	case V4L2_MPEG_VIDEO_HEVC_LEVEL_2:
		return (tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN) ? 8 : 9;
	case V4L2_MPEG_VIDEO_HEVC_LEVEL_2_1:
		return (tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN) ? 10 : 11;
	case V4L2_MPEG_VIDEO_HEVC_LEVEL_3:
		return (tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN) ? 13 : 14;
	case V4L2_MPEG_VIDEO_HEVC_LEVEL_3_1:
		return (tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN) ? 15 : 16;
	case V4L2_MPEG_VIDEO_HEVC_LEVEL_4:
		return (tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN) ? 18 : 19;
	case V4L2_MPEG_VIDEO_HEVC_LEVEL_4_1:
		return (tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN) ? 20 : 21;
	case V4L2_MPEG_VIDEO_HEVC_LEVEL_5:
		return (tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN) ? 23 : 24;
	case V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1:
		return (tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN) ? 25 : 26;
	case V4L2_MPEG_VIDEO_HEVC_LEVEL_5_2:
		return (tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN) ? 27 : 28;
	case V4L2_MPEG_VIDEO_HEVC_LEVEL_6:
		return (tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN) ? 29 : 30;
	case V4L2_MPEG_VIDEO_HEVC_LEVEL_6_1:
		return (tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN) ? 31 : 32;
	case V4L2_MPEG_VIDEO_HEVC_LEVEL_6_2:
		return (tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN) ? 33 : 34;
	default:
		mtk_vcodec_debug(inst, "unsupported level %d", level);
		return 25;
	}
}

static unsigned int venc_mpeg4_get_profile(struct venc_inst *inst,
	unsigned int profile)
{
	switch (profile) {
	case V4L2_MPEG_VIDEO_MPEG4_PROFILE_SIMPLE:
		return 0;
	case V4L2_MPEG_VIDEO_MPEG4_PROFILE_ADVANCED_SIMPLE:
		return 1;
	case V4L2_MPEG_VIDEO_MPEG4_PROFILE_CORE:
		return 2;
	case V4L2_MPEG_VIDEO_MPEG4_PROFILE_SIMPLE_SCALABLE:
		return 3;
	case V4L2_MPEG_VIDEO_MPEG4_PROFILE_ADVANCED_CODING_EFFICIENCY:
		return 4;
	default:
		mtk_vcodec_debug(inst, "unsupported mpeg4 profile %d", profile);
		return 100;
	}
}

static unsigned int venc_mpeg4_get_level(struct venc_inst *inst,
	unsigned int level)
{
	switch (level) {
	case V4L2_MPEG_VIDEO_MPEG4_LEVEL_0:
		return 0;
	case V4L2_MPEG_VIDEO_MPEG4_LEVEL_0B:
		return 1;
	case V4L2_MPEG_VIDEO_MPEG4_LEVEL_1:
		return 2;
	case V4L2_MPEG_VIDEO_MPEG4_LEVEL_2:
		return 3;
	case V4L2_MPEG_VIDEO_MPEG4_LEVEL_3:
		return 4;
	case V4L2_MPEG_VIDEO_MPEG4_LEVEL_3B:
		return 5;
	case V4L2_MPEG_VIDEO_MPEG4_LEVEL_4:
		return 6;
	case V4L2_MPEG_VIDEO_MPEG4_LEVEL_5:
		return 7;
	default:
		mtk_vcodec_debug(inst, "unsupported mpeg4 level %d", level);
		return 4;
	}
}

static int venc_encode_header(struct venc_inst *inst,
	struct mtk_vcodec_mem *bs_buf,
	unsigned int *bs_size)
{
	int ret = 0;

	mtk_vcodec_debug_enter(inst);
	if (bs_buf == NULL)
		inst->vsi->venc.venc_bs_va = 0;
	else
		inst->vsi->venc.venc_bs_va = (u64)(bs_buf->index + 1);

	inst->vsi->venc.venc_fb_va = 0;

	mtk_vcodec_debug(inst, "vsi venc_bs_va %lld",
			 inst->vsi->venc.venc_bs_va);

	ret = vcu_enc_encode(&inst->vcu_inst, VENC_BS_MODE_SEQ_HDR, NULL,
						 bs_buf, bs_size);

	return ret;
}

static int venc_encode_frame(struct venc_inst *inst,
	struct venc_frm_buf *frm_buf,
	struct mtk_vcodec_mem *bs_buf,
	unsigned int *bs_size)
{
	int ret = 0;
	unsigned int fm_fourcc = inst->ctx->q_data[MTK_Q_DATA_SRC].fmt->fourcc;
	unsigned int bs_fourcc = inst->ctx->q_data[MTK_Q_DATA_DST].fmt->fourcc;

	mtk_vcodec_debug_enter(inst);

	if (bs_buf == NULL)
		inst->vsi->venc.venc_bs_va = 0;
	else
		inst->vsi->venc.venc_bs_va = (u64)(bs_buf->index + 1);

	if (frm_buf == NULL)
		inst->vsi->venc.venc_fb_va = 0;
	else {
		inst->vsi->venc.venc_fb_va = (u64)(frm_buf->index + 1);
		inst->vsi->venc.timestamp = frm_buf->timestamp;
	}
	ret = vcu_enc_encode(&inst->vcu_inst, VENC_BS_MODE_FRAME, frm_buf,
						 bs_buf, bs_size);
	if (ret)
		return ret;

	++inst->frm_cnt;
	mtk_vcodec_debug(inst,
		 "Format: frame_va %lld (%c%c%c%c) bs_va:%lld (%c%c%c%c)",
		  inst->vsi->venc.venc_fb_va,
		  fm_fourcc & 0xFF, (fm_fourcc >> 8) & 0xFF,
		  (fm_fourcc >> 16) & 0xFF, (fm_fourcc >> 24) & 0xFF,
		  inst->vsi->venc.venc_bs_va,
		  bs_fourcc & 0xFF, (bs_fourcc >> 8) & 0xFF,
		  (bs_fourcc >> 16) & 0xFF, (bs_fourcc >> 24) & 0xFF);

	return ret;
}

static int venc_encode_frame_final(struct venc_inst *inst,
	struct venc_frm_buf *frm_buf,
	struct mtk_vcodec_mem *bs_buf,
	unsigned int *bs_size)
{
	int ret = 0;

	mtk_v4l2_debug(0, "check inst->vsi %p +", inst->vsi);
	if (inst == NULL || inst->vsi == NULL)
		return -EINVAL;

	if (bs_buf == NULL)
		inst->vsi->venc.venc_bs_va = 0;
	else
		inst->vsi->venc.venc_bs_va = (u64)(bs_buf->index + 1);
	if (frm_buf == NULL)
		inst->vsi->venc.venc_fb_va = 0;
	else
		inst->vsi->venc.venc_fb_va = (u64)(frm_buf->index + 1);

	ret = vcu_enc_encode(&inst->vcu_inst, VENC_BS_MODE_FRAME_FINAL, frm_buf,
						 bs_buf, bs_size);
	if (ret)
		return ret;

	*bs_size = inst->vcu_inst.bs_size;
	mtk_vcodec_debug(inst, "bs size %d <-", *bs_size);

	return ret;
}


static int venc_init(struct mtk_vcodec_ctx *ctx, unsigned long *handle)
{
	int ret = 0;
	struct venc_inst *inst;
	struct vcu_v4l2_callback_func cb;

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (!inst) {
		*handle = (unsigned long)NULL;
		return -ENOMEM;
	}

	inst->ctx = ctx;
	inst->vcu_inst.ctx = ctx;
	inst->vcu_inst.dev = VCU_FPTR(vcu_get_plat_device)(ctx->dev->plat_dev);
	inst->vcu_inst.id = IPI_VENC_COMMON;
	inst->hw_base = mtk_vcodec_get_enc_reg_addr(inst->ctx, VENC_SYS);
	inst->vcu_inst.handler = vcu_enc_ipi_handler;
	(*handle) = (unsigned long)inst;

	mtk_vcodec_debug_enter(inst);

	mtk_vcodec_add_ctx_list(ctx);

	ret = vcu_enc_init(&inst->vcu_inst);

	inst->vsi = (struct venc_vsi *)inst->vcu_inst.vsi;

	memset(&cb, 0, sizeof(struct vcu_v4l2_callback_func));
	cb.enc_prepare = venc_encode_prepare;
	cb.enc_unprepare = venc_encode_unprepare;
	cb.enc_pmqos_gce_begin = venc_encode_pmqos_gce_begin;
	cb.enc_pmqos_gce_end = venc_encode_pmqos_gce_end;
	cb.gce_timeout_dump = mtk_vcodec_gce_timeout_dump;
	VCU_FPTR(vcu_set_v4l2_callback)(inst->vcu_inst.dev, &cb);

	mtk_vcodec_debug_leave(inst);

	if (ret) {
		mtk_vcodec_del_ctx_list(ctx);
		kfree(inst);
		(*handle) = (unsigned long)NULL;
	}

	return ret;
}

static int venc_encode(unsigned long handle,
					   enum venc_start_opt opt,
					   struct venc_frm_buf *frm_buf,
					   struct mtk_vcodec_mem *bs_buf,
					   struct venc_done_result *result)
{
	int ret = 0;
	struct venc_inst *inst = (struct venc_inst *)handle;

	if (inst == NULL || inst->vsi == NULL)
		return -EINVAL;

	mtk_vcodec_debug(inst, "opt %d ->", opt);

	switch (opt) {
	case VENC_START_OPT_ENCODE_SEQUENCE_HEADER: {
		unsigned int bs_size_hdr = 0;

		ret = venc_encode_header(inst, bs_buf, &bs_size_hdr);
		if (ret)
			goto encode_err;

		result->bs_size = bs_size_hdr;
		result->is_key_frm = false;
		break;
	}

	case VENC_START_OPT_ENCODE_FRAME: {
		/* only run @ worker then send ipi
		 * VPU flush cmd binding ctx & handle
		 * or cause cmd calllback ctx error
		 */
		ret = venc_encode_frame(inst, frm_buf, bs_buf,
			&result->bs_size);
		if (ret)
			goto encode_err;
		result->is_key_frm = inst->vcu_inst.is_key_frm;
		break;
	}

	case VENC_START_OPT_ENCODE_FRAME_FINAL: {
		ret = venc_encode_frame_final(inst,
			frm_buf, bs_buf, &result->bs_size);
		if (ret)
			goto encode_err;
		result->is_key_frm = inst->vcu_inst.is_key_frm;
		break;
	}

	default:
		mtk_vcodec_err(inst, "venc_start_opt %d not supported", opt);
		ret = -EINVAL;
		break;
	}

encode_err:
	mtk_vcodec_debug(inst, "opt %d <-", opt);

	return ret;
}

static void venc_get_free_buffers(struct venc_inst *inst,
			     struct ring_input_list *list,
			     struct venc_done_result *pResult)
{
	u64 bs_index, fb_index;

	if (list->count < 0 || list->count >= VENC_MAX_FB_NUM) {
		mtk_vcodec_err(inst, "list count %d invalid ! (write_idx %d, read_idx %d)",
			list->count, list->write_idx, list->read_idx);
		if (list->write_idx < 0 || list->write_idx >= VENC_MAX_FB_NUM ||
		    list->read_idx < 0  || list->read_idx >= VENC_MAX_FB_NUM)
			list->write_idx = list->read_idx = 0;
		if (list->write_idx >= list->read_idx)
			list->count = list->write_idx - list->read_idx;
		else
			list->count = list->write_idx + VENC_MAX_FB_NUM - list->read_idx;
	}
	if (list->count == 0) {
		mtk_vcodec_debug(inst, "[FB] there is no free buffers");
		pResult->bs_va = 0;
		pResult->frm_va = 0;
		pResult->is_key_frm = false;
		pResult->bs_size = 0;
		return;
	}

	pResult->bs_size = list->bs_size[list->read_idx];
	pResult->is_key_frm = list->is_key_frm[list->read_idx];
	bs_index = list->venc_bs_va_list[list->read_idx];
	pResult->bs_va = (unsigned long)inst->ctx->bs_list[bs_index];
	fb_index = list->venc_fb_va_list[list->read_idx];
	pResult->frm_va = (unsigned long)inst->ctx->fb_list[fb_index];
	pResult->is_last_slc = list->is_last_slice[list->read_idx];

	mtk_vcodec_debug(inst, "read_idx=%d bsva %lx %lld frva %lx %lld bssize %d iskey %d is_last_slc=%d",
		list->read_idx,
		pResult->bs_va, bs_index,
		pResult->frm_va, fb_index,
		pResult->bs_size,
		pResult->is_key_frm,
		pResult->is_last_slc);

	list->read_idx = (list->read_idx == VENC_MAX_FB_NUM - 1U) ?
			 0U : list->read_idx + 1U;
	list->count--;
}

static void venc_get_resolution_change(struct venc_inst *inst,
			     struct venc_vcu_config *Config,
			     struct venc_resolution_change *pResChange)
{
	pResChange->width = Config->pic_w;
	pResChange->height = Config->pic_h;
	pResChange->framerate = Config->framerate;
	pResChange->resolutionchange = Config->resolutionChange;

	if (Config->resolutionChange)
		Config->resolutionChange = 0;

	mtk_vcodec_debug(inst, "get reschange %d %d %d %d\n",
		 pResChange->width,
		 pResChange->height,
		 pResChange->framerate,
		 pResChange->resolutionchange);
}


static int venc_get_param(unsigned long handle,
						  enum venc_get_param_type type,
						  void *out)
{
	int ret = 0;
	struct venc_inst *inst = (struct venc_inst *)handle;

	if (inst == NULL)
		return -EINVAL;

	mtk_vcodec_debug(inst, "%s: %d", __func__, type);
	inst->vcu_inst.ctx = inst->ctx;

	switch (type) {
	case GET_PARAM_VENC_CAP_FRAME_SIZES:
	case GET_PARAM_VENC_CAP_SUPPORTED_FORMATS:
		vcu_enc_query_cap(&inst->vcu_inst, type, out);
		break;
	case GET_PARAM_FREE_BUFFERS:
		if (inst->vsi == NULL)
			return -EINVAL;
		venc_get_free_buffers(inst, &inst->vsi->list_free, out);
		break;
	case GET_PARAM_ROI_RC_QP: {
		if (inst->vsi == NULL || out == NULL)
			return -EINVAL;
		*(int *)out = inst->vsi->config.roi_rc_qp;
		break;
	}
	case GET_PARAM_RESOLUTION_CHANGE:
		if (inst->vsi == NULL)
			return -EINVAL;
		venc_get_resolution_change(inst, &inst->vsi->config, out);
		break;
	case GET_PARAM_VENC_VCU_VPUD_LOG:
		VCU_FPTR(vcu_get_log)(out, LOG_PROPERTY_SIZE);
		break;
	default:
		mtk_vcodec_err(inst, "invalid get parameter type=%d", type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int venc_set_param(unsigned long handle,
	enum venc_set_param_type type,
	struct venc_enc_param *enc_prm)
{
	int i;
	int ret = 0;
	struct venc_inst *inst = (struct venc_inst *)handle;
	unsigned int fmt = 0;

	if (inst == NULL)
		return -EINVAL;

	mtk_vcodec_debug(inst, "->type=%d", type);

	switch (type) {
	case VENC_SET_PARAM_ENC:
		if (inst->vsi == NULL)
			return -EINVAL;
		inst->vsi->config.input_fourcc = enc_prm->input_yuv_fmt;
		inst->vsi->config.bitrate = enc_prm->bitrate;
		inst->vsi->config.pic_w = enc_prm->width;
		inst->vsi->config.pic_h = enc_prm->height;
		inst->vsi->config.buf_w = enc_prm->buf_width;
		inst->vsi->config.buf_h = enc_prm->buf_height;
		inst->vsi->config.gop_size = enc_prm->gop_size;
		inst->vsi->config.framerate = enc_prm->frm_rate;
		inst->vsi->config.intra_period = enc_prm->intra_period;
		inst->vsi->config.operationrate = enc_prm->operationrate;
		inst->vsi->config.bitratemode = enc_prm->bitratemode;
		inst->vsi->config.roion = enc_prm->roion;
		inst->vsi->config.scenario = enc_prm->scenario;
		inst->vsi->config.prependheader = enc_prm->prependheader;
		inst->vsi->config.heif_grid_size = enc_prm->heif_grid_size;
		inst->vsi->config.max_w = enc_prm->max_w;
		inst->vsi->config.max_h = enc_prm->max_h;
		inst->vsi->config.num_b_frame = enc_prm->num_b_frame;
		inst->vsi->config.slbc_ready = enc_prm->slbc_ready;
		inst->vsi->config.slbc_addr = enc_prm->slbc_addr;
		inst->vsi->config.i_qp = enc_prm->i_qp;
		inst->vsi->config.p_qp = enc_prm->p_qp;
		inst->vsi->config.b_qp = enc_prm->b_qp;
		inst->vsi->config.svp_mode = enc_prm->svp_mode;
		inst->vsi->config.highquality = enc_prm->highquality;
		inst->vsi->config.max_qp = enc_prm->max_qp;
		inst->vsi->config.min_qp = enc_prm->min_qp;
		inst->vsi->config.i_p_qp_delta = enc_prm->ip_qpdelta;
		inst->vsi->config.qp_control_mode = enc_prm->qp_control_mode;
		inst->vsi->config.frame_level_qp = enc_prm->framelvl_qp;
		inst->vsi->config.dummynal = enc_prm->dummynal;
		inst->vsi->config.hier_ref_layer = enc_prm->hier_ref_layer;
		inst->vsi->config.hier_ref_type = enc_prm->hier_ref_type;
		inst->vsi->config.temporal_layer_pcount = enc_prm->temporal_layer_pcount;
		inst->vsi->config.temporal_layer_bcount = enc_prm->temporal_layer_bcount;
		inst->vsi->config.max_ltr_num = enc_prm->max_ltr_num;

		if (enc_prm->color_desc) {
			memcpy(&inst->vsi->config.color_desc,
				enc_prm->color_desc,
				sizeof(struct mtk_color_desc));
		}

		if (enc_prm->multi_ref) {
			memcpy(&inst->vsi->config.multi_ref,
				enc_prm->multi_ref,
				sizeof(struct mtk_venc_multi_ref));
		}

		if (enc_prm->vui_info) {
			memcpy(&inst->vsi->config.vui_info,
				enc_prm->vui_info,
				sizeof(struct mtk_venc_vui_info));
		}

		inst->vsi->config.slice_header_spacing =
			enc_prm->slice_header_spacing;

		fmt = inst->ctx->q_data[MTK_Q_DATA_DST].fmt->fourcc;
		mtk_vcodec_debug(inst, "fmt:%u", fmt);

		if (fmt == V4L2_PIX_FMT_H264) {
			inst->vsi->config.profile = enc_prm->profile;
			inst->vsi->config.level = enc_prm->level;
		} else if (fmt == V4L2_PIX_FMT_H265 ||
				fmt == V4L2_PIX_FMT_HEIF) {
			inst->vsi->config.profile =
				venc_h265_get_profile(inst, enc_prm->profile);
			inst->vsi->config.level =
				venc_h265_get_level(inst, enc_prm->level,
					enc_prm->tier);
		} else if (fmt == V4L2_PIX_FMT_MPEG4) {
			inst->vsi->config.profile =
				venc_mpeg4_get_profile(inst, enc_prm->profile);
			inst->vsi->config.level =
				venc_mpeg4_get_level(inst, enc_prm->level);
		}
		inst->vsi->config.wfd = 0;
		ret = vcu_enc_set_param(&inst->vcu_inst, type, enc_prm);
		if (ret)
			break;

		for (i = 0; i < MTK_VCODEC_MAX_PLANES; i++) {
			enc_prm->sizeimage[i] =
				inst->vsi->sizeimage[i];
			mtk_vcodec_debug(inst, "sizeimage[%d] size=0x%x", i,
							 enc_prm->sizeimage[i]);
		}
		inst->ctx->async_mode = !(inst->vsi->sync_mode);

		break;
	case VENC_SET_PARAM_PREPEND_HEADER:
		inst->prepend_hdr = 1;
		ret = vcu_enc_set_param(&inst->vcu_inst, type, enc_prm);
		break;
	case VENC_SET_PARAM_COLOR_DESC:
		if (inst->vsi == NULL)
			return -EINVAL;
		memcpy(&inst->vsi->config.color_desc, enc_prm->color_desc,
			sizeof(struct mtk_color_desc));
		ret = vcu_enc_set_param(&inst->vcu_inst, type, enc_prm);
		break;
	case VENC_SET_PARAM_PROPERTY:
		mtk_vcodec_err(inst, "VCU not support SET_PARAM_VDEC_PROPERTY\n");
		break;
	case VENC_SET_PARAM_VCU_VPUD_LOG:
		ret = VCU_FPTR(vcu_set_log)(enc_prm->log);
		break;
	default:
		if (inst->vsi == NULL)
			return -EINVAL;
		ret = vcu_enc_set_param(&inst->vcu_inst, type, enc_prm);
		inst->ctx->async_mode = !(inst->vsi->sync_mode);
		break;
	}

	mtk_vcodec_debug_leave(inst);

	return ret;
}

static int venc_deinit(unsigned long handle)
{
	int ret = 0;
	struct venc_inst *inst = (struct venc_inst *)handle;

	mtk_vcodec_debug_enter(inst);

	ret = vcu_enc_deinit(&inst->vcu_inst);

	mtk_vcodec_del_ctx_list(inst->ctx);

	mtk_vcodec_debug_leave(inst);
	kfree(inst);

	return ret;
}

static const struct venc_common_if venc_if = {
	.init = venc_init,
	.encode = venc_encode,
	.get_param = venc_get_param,
	.set_param = venc_set_param,
	.deinit = venc_deinit,
};

const struct venc_common_if *get_enc_vcu_if(void);

const struct venc_common_if *get_enc_vcu_if(void)
{
	return &venc_if;
}
