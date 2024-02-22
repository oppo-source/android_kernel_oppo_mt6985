// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2016 MediaTek Inc.
 * Author: PoChun Lin <pochun.lin@mediatek.com>
 */

#include <linux/interrupt.h>
#include <media/v4l2-mem2mem.h>
#include <linux/mtk_vcu_controls.h>
#include <linux/delay.h>
#include "mtk_vcu.h"
#include "venc_vcu_if.h"
#include "venc_drv_if.h"
#include "mtk_vcodec_intr.h"
#include "mtk_vcodec_enc_pm.h"
#include "mtk_vcodec_enc.h"

static void handle_enc_init_msg(struct venc_vcu_inst *vcu, void *data)
{
	struct venc_vcu_ipi_msg_init *msg = data;

	if (vcu == NULL)
		return;
	vcu->inst_addr = msg->vcu_inst_addr;
	vcu->vsi = VCU_FPTR(vcu_mapping_dm_addr)(vcu->dev, msg->vcu_inst_addr);
}

static void handle_query_cap_ack_msg(struct venc_vcu_inst *vcu,
	struct venc_vcu_ipi_query_cap_ack *msg)
{
	void *data;
	int size = 0;

	if (vcu == NULL)
		return;
	mtk_vcodec_debug(vcu, "+ ap_inst_addr = 0x%lx, vcu_data_addr = 0x%x, id = %d",
		(uintptr_t)msg->ap_inst_addr, msg->vcu_data_addr, msg->id);
	/* mapping VCU address to kernel virtual address */
	data = VCU_FPTR(vcu_mapping_dm_addr)(vcu->dev, msg->vcu_data_addr);
	if (data == NULL)
		return;
	switch (msg->id) {
	case GET_PARAM_VENC_CAP_SUPPORTED_FORMATS:
		size = sizeof(struct mtk_video_fmt);
		memcpy((void *)mtk_venc_formats, data,
			size * MTK_MAX_ENC_CODECS_SUPPORT);
		break;
	case GET_PARAM_VENC_CAP_FRAME_SIZES:
		size = sizeof(struct mtk_codec_framesizes);
		memcpy((void *)mtk_venc_framesizes, data,
			size * MTK_MAX_ENC_CODECS_SUPPORT);
		break;
	default:
		break;
	}
	mtk_vcodec_debug(vcu, "- vcu_inst_addr = 0x%x", vcu->inst_addr);
}

static void handle_enc_waitisr_msg(struct venc_vcu_inst *vcu,
	void *data, uint32_t timeout)
{
	struct venc_vcu_ipi_msg_waitisr *msg = data;
	struct mtk_vcodec_ctx *ctx = vcu->ctx;

	msg->irq_status = ctx->irq_status;
	msg->timeout = timeout;
}

static int check_codec_id(struct venc_vcu_ipi_msg_common *msg, unsigned int fmt)
{
	int codec_id = 0, ret = 0;

	switch (fmt) {
	case V4L2_PIX_FMT_H264:
		codec_id = VENC_H264;
		break;
	case V4L2_PIX_FMT_VP8:
		codec_id = VENC_VP8;
		break;
	case V4L2_PIX_FMT_MPEG4:
		codec_id = VENC_MPEG4;
		break;
	case V4L2_PIX_FMT_H263:
		codec_id = VENC_H263;
		break;
	case V4L2_PIX_FMT_H265:
		codec_id = VENC_H265;
		break;
	case V4L2_PIX_FMT_HEIF:
		codec_id = VENC_HEIF;
		break;
	default:
		pr_info("%s fourcc not supported", __func__);
		break;
	}

	if (codec_id == 0) {
		mtk_v4l2_err("[error] venc unsupported fourcc\n");
		ret = -1;
	} else if (msg->codec_id == codec_id) {
		pr_info("%s ipi id %d is correct\n", __func__, msg->codec_id);
		ret = 0;
	} else {
		mtk_v4l2_debug(2, "[Info] ipi id %d is incorrect\n", msg->codec_id);
		ret = -1;
	}

	return ret;
}

static int handle_enc_get_bs_buf(struct venc_vcu_inst *vcu, void *data)
{
	struct mtk_vcodec_mem *pbs_buf;
	struct mtk_vcodec_ctx *ctx = vcu->ctx;
	struct venc_vsi *vsi = (struct venc_vsi *)vcu->vsi;
	struct venc_vcu_ipi_msg_get_bs *msg = (struct venc_vcu_ipi_msg_get_bs *)data;
	long timeout_jiff;
	int ret = 1;

	pbs_buf =  mtk_vcodec_get_bs(ctx);
	timeout_jiff = msecs_to_jiffies(1000);

	while (pbs_buf == NULL) {
		ret = wait_event_interruptible_timeout(
					vcu->ctx->bs_wq,
					 v4l2_m2m_num_dst_bufs_ready(
						 vcu->ctx->m2m_ctx) > 0 ||
					 vcu->ctx->state == MTK_STATE_FLUSH,
					 timeout_jiff);
		pbs_buf = mtk_vcodec_get_bs(ctx);
	}

	vsi->venc.venc_bs_va = (u64)(uintptr_t)pbs_buf;
	msg->bs_addr = pbs_buf->dma_addr;
	msg->bs_size = pbs_buf->size;
	pbs_buf->buf_fd = msg->bs_fd;

	return 1;
}
int vcu_enc_ipi_handler(void *data, unsigned int len, void *priv)
{
	struct mtk_vcodec_dev *dev = (struct mtk_vcodec_dev *)priv;
	struct venc_vcu_ipi_msg_common *msg = data;
	int msg_ctx_id;
	struct venc_vcu_inst *vcu;
	struct venc_vsi *vsi = NULL;
	struct venc_inst *inst = NULL;
	struct mtk_vcodec_ctx *ctx;
	int ret = 0;
	unsigned long flags;
	struct task_struct *task = NULL;
	struct list_head *p, *q;
	struct mtk_vcodec_ctx *temp_ctx;
	int msg_valid = 0;

	BUILD_BUG_ON(sizeof(struct venc_ap_ipi_msg_init) > SHARE_BUF_SIZE);
	BUILD_BUG_ON(sizeof(struct venc_ap_ipi_query_cap) > SHARE_BUF_SIZE);
	BUILD_BUG_ON(sizeof(struct venc_ap_ipi_msg_set_param) > SHARE_BUF_SIZE);
	BUILD_BUG_ON(sizeof(struct venc_ap_ipi_msg_enc) > SHARE_BUF_SIZE);
	BUILD_BUG_ON(sizeof(struct venc_ap_ipi_msg_deinit) > SHARE_BUF_SIZE);
	BUILD_BUG_ON(
		sizeof(struct venc_vcu_ipi_query_cap_ack) > SHARE_BUF_SIZE);
	BUILD_BUG_ON(sizeof(struct venc_vcu_ipi_msg_common) > SHARE_BUF_SIZE);
	BUILD_BUG_ON(sizeof(struct venc_vcu_ipi_msg_init) > SHARE_BUF_SIZE);
	BUILD_BUG_ON(
		sizeof(struct venc_vcu_ipi_msg_set_param) > SHARE_BUF_SIZE);
	BUILD_BUG_ON(sizeof(struct venc_vcu_ipi_msg_enc) > SHARE_BUF_SIZE);
	BUILD_BUG_ON(sizeof(struct venc_vcu_ipi_msg_deinit) > SHARE_BUF_SIZE);
	BUILD_BUG_ON(sizeof(struct venc_vcu_ipi_msg_waitisr) > SHARE_BUF_SIZE);

	VCU_FPTR(vcu_get_task)(&task, 0);
	if (msg == NULL || task == NULL ||
	   task->tgid != current->tgid ||
	   msg->ap_inst_addr == 0) {
		VCU_FPTR(vcu_put_task)();
		ret = -EINVAL;
		return ret;
	}
	VCU_FPTR(vcu_put_task)();

	msg_ctx_id = (int)msg->ap_inst_addr;

	/* Check IPI inst is valid */
	mutex_lock(&dev->ctx_mutex);
	msg_valid = 0;
	list_for_each_safe(p, q, &dev->ctx_list) {
		temp_ctx = list_entry(p, struct mtk_vcodec_ctx, list);
		inst = (struct venc_inst *)temp_ctx->drv_handle;
		if (inst != NULL && msg_ctx_id == temp_ctx->id) {
			vcu = &inst->vcu_inst;
			msg_valid = 1;
			break;
		}
	}
	if (!msg_valid) {
		mtk_v4l2_err(" msg msg_id %X vcu not exist %d\n",
			msg->msg_id, msg_ctx_id);
		mutex_unlock(&dev->ctx_mutex);
		ret = -EINVAL;
		return ret;
	}
	mutex_unlock(&dev->ctx_mutex);

	if (vcu->daemon_pid != current->tgid) {
		pr_info("%s, vcu->daemon_pid:%d != current %d\n",
			__func__, vcu->daemon_pid, current->tgid);
		return 1;
	}

	mtk_vcodec_debug(vcu, "msg_id %x inst %p status %d",
					 msg->msg_id, vcu, msg->status);

	if (vcu->abort)
		return -EINVAL;

	ctx = vcu->ctx;
	vsi = (struct venc_vsi *)vcu->vsi;
	switch (msg->msg_id) {
	case VCU_IPIMSG_ENC_INIT_DONE:
		handle_enc_init_msg(vcu, data);
		break;
	case VCU_IPIMSG_ENC_SET_PARAM_DONE:
		/* Prevent slowmotion with GCE mode on,
		 * user thread enter freezing while holding mutex (enc lock)
		 */
		current->flags |= PF_NOFREEZE;
		break;
	case VCU_IPIMSG_ENC_DEINIT_DONE:
		break;
	case VCU_IPIMSG_ENC_POWER_ON:
		/*use status to store core ID*/
		ctx->sysram_enable = vsi->config.sysram_enable;
		venc_encode_prepare(ctx, msg->status, &flags);
		msg->status = VENC_IPI_MSG_STATUS_OK;
		ret = 1;
		break;
	case VCU_IPIMSG_ENC_POWER_OFF:
		/*use status to store core ID*/
		ctx->sysram_enable = vsi->config.sysram_enable;
		venc_encode_unprepare(ctx, msg->status, &flags);
		msg->status = VENC_IPI_MSG_STATUS_OK;
		ret = 1;
		break;
	case VCU_IPIMSG_ENC_QUERY_CAP_DONE:
		handle_query_cap_ack_msg(vcu, data);
		break;
	case VCU_IPIMSG_ENC_WAIT_ISR:
		if (msg->status == MTK_VENC_CORE_0)
			vcodec_trace_count("VENC_HW_CORE_0", 2);
		else
			vcodec_trace_count("VENC_HW_CORE_1", 2);

		if (-1 == mtk_vcodec_wait_for_done_ctx(ctx, msg->status,
			MTK_INST_IRQ_RECEIVED,
			WAIT_INTR_TIMEOUT_MS)) {
			handle_enc_waitisr_msg(vcu, data, 1);
			mtk_vcodec_debug(vcu,
				"irq_status %x <-", ctx->irq_status);
		} else
			handle_enc_waitisr_msg(vcu, data, 0);

		if (msg->status == MTK_VENC_CORE_0)
			vcodec_trace_count("VENC_HW_CORE_0", 1);
		else
			vcodec_trace_count("VENC_HW_CORE_1", 1);

		ret = 1;
		break;
	case VCU_IPIMSG_ENC_PUT_BUFFER:
		mtk_enc_put_buf(ctx);
		ret = 1;
		break;
	case VCU_IPIMSG_ENC_ENCODE_DONE:
		break;
	case VCU_IPIMSG_ENC_CHECK_CODEC_ID:
		if (check_codec_id(msg, ctx->q_data[MTK_Q_DATA_DST].fmt->fourcc) == 0)
			msg->status = 0;
		else
			msg->status = -1;
		ret = 1;
		break;
	case VCU_IPIMSG_ENC_GET_BS_BUFFER:
		ret = handle_enc_get_bs_buf(vcu, data);
		break;
	default:
		mtk_vcodec_err(vcu, "unknown msg id %x", msg->msg_id);
		break;
	}

	/* deinit done timeout case handling do not touch vdec_vcu_inst
	 * or memory used after freed
	 */
	if (msg->msg_id != VCU_IPIMSG_ENC_DEINIT_DONE) {
		vcu->signaled = 1;
		vcu->failure = (msg->status != VENC_IPI_MSG_STATUS_OK) ? 1 : 0;
	}

	mtk_vcodec_debug_leave(vcu);
	return ret;
}

static int vcu_enc_send_msg(struct venc_vcu_inst *vcu, void *msg,
							int len)
{
	int status;
	struct task_struct *task = NULL;
	unsigned int suspend_block_cnt = 0;

	mtk_vcodec_debug_enter(vcu);

	if (!vcu->dev) {
		mtk_vcodec_err(vcu, "inst dev is NULL");
		return -EINVAL;
	}

	if (vcu->abort)
		return -EIO;

	while (vcu->ctx->dev->is_codec_suspending == 1) {
		suspend_block_cnt++;
		if (suspend_block_cnt > SUSPEND_TIMEOUT_CNT) {
			mtk_v4l2_debug(4, "VENC blocked by suspend\n");
			suspend_block_cnt = 0;
		}
		usleep_range(10000, 20000);
	}

	VCU_FPTR(vcu_get_task)(&task, 0);
	if (task == NULL ||
		vcu->daemon_pid != task->tgid) {
		if (task)
			mtk_vcodec_err(vcu, "send fail pid: inst %d curr %d",
				vcu->daemon_pid, task->tgid);
		VCU_FPTR(vcu_put_task)();
		vcu->abort = 1;
		return -EIO;
	}
	VCU_FPTR(vcu_put_task)();

	status = VCU_FPTR(vcu_ipi_send)(vcu->dev, vcu->id, msg, len, vcu->ctx->dev);
	if (status) {
		mtk_vcodec_err(vcu, "vcu_ipi_send msg_id %x len %d fail %d",
					   *(uint32_t *)msg, len, status);
		if (status == -EIO)
			vcu->abort = 1;
		return status;
	}
	mtk_vcodec_debug(vcu, "vcu_ipi_send msg_id %x len %d success",
			*(uint32_t *)msg, len);

	if (vcu->failure)
		return -EINVAL;

	mtk_vcodec_debug_leave(vcu);

	return 0;
}


void vcu_enc_set_pid(struct venc_vcu_inst *vcu)
{
	struct task_struct *task = NULL;

	VCU_FPTR(vcu_get_task)(&task, 0);
	if (task != NULL)
		vcu->daemon_pid = task->tgid;
	else
		vcu->daemon_pid = -1;
	VCU_FPTR(vcu_put_task)();
}

int vcu_enc_set_ctx(struct venc_vcu_inst *vcu,
	struct venc_frm_buf *frm_buf,
	struct mtk_vcodec_mem *bs_buf)
{
	int err = 0;
	struct vb2_buffer *src_vb = NULL;
	struct vb2_buffer *dst_vb = NULL;
	struct mtk_video_enc_buf *temp;

	if (frm_buf != NULL) {
		temp = container_of(frm_buf, struct mtk_video_enc_buf, frm_buf);
		src_vb = &temp->vb.vb2_buf;
	}
	if (bs_buf != NULL) {
		temp = container_of(bs_buf, struct mtk_video_enc_buf, bs_buf);
		dst_vb = &temp->vb.vb2_buf;
	}
	VCU_FPTR(vcu_set_codec_ctx)(vcu->dev,
		(void *)vcu->ctx,
		src_vb, dst_vb,
		VCU_VENC);

	return err;
}

int vcu_enc_clear_ctx(struct venc_vcu_inst *vcu)
{
	int err = 0;

	VCU_FPTR(vcu_clear_codec_ctx)(vcu->dev,
		(void *)vcu->ctx, VCU_VENC);

	return err;
}

int vcu_enc_init(struct venc_vcu_inst *vcu)
{
	int status;
	struct venc_ap_ipi_msg_init out;

	mtk_vcodec_debug_enter(vcu);

	if (sizeof(out) > SHARE_BUF_SIZE) {
		mtk_vcodec_err(vcu, "venc_ap_ipi_msg_init cannot be large than %d",
					   SHARE_BUF_SIZE);
		return -EINVAL;
	}

	vcu->signaled = 0;
	vcu->failure = 0;
	VCU_FPTR(vcu_get_ctx_ipi_binding_lock)(vcu->dev, &vcu->ctx_ipi_lock, VCU_VENC);

	status = VCU_FPTR(vcu_ipi_register)(vcu->dev,
		vcu->id, vcu->handler, NULL, vcu->ctx->dev);
	if (status) {
		mtk_vcodec_err(vcu, "vcu_ipi_register fail %d", status);
		return -EINVAL;
	}

	memset(&out, 0, sizeof(out));
	out.msg_id = AP_IPIMSG_ENC_INIT;
	out.ap_inst_addr = (unsigned long)vcu->ctx->id;
	out.ctx_id = vcu->ctx->id;

	vcu_enc_set_pid(vcu);
	status = vcu_enc_send_msg(vcu, &out, sizeof(out));

	if (status) {
		mtk_vcodec_err(vcu, "AP_IPIMSG_ENC_INIT fail");
		return -EINVAL;
	}
	mtk_vcodec_debug_leave(vcu);

	return 0;
}

int vcu_enc_query_cap(struct venc_vcu_inst *vcu, unsigned int id, void *out)
{
	struct venc_ap_ipi_query_cap msg;
	int err = 0;

	if (sizeof(msg) > SHARE_BUF_SIZE) {
		mtk_vcodec_err(vcu, "venc_ap_ipi_query_cap cannot be large than %d",
					   SHARE_BUF_SIZE);
		return -EINVAL;
	}

	mtk_vcodec_debug(vcu, "+ id=%X", AP_IPIMSG_ENC_QUERY_CAP);
	vcu->dev = VCU_FPTR(vcu_get_plat_device)(vcu->ctx->dev->plat_dev);
	if (vcu->dev  == NULL) {
		mtk_vcodec_err(vcu, "vcu device in not ready");
		return -EPROBE_DEFER;
	}

	vcu->id = (vcu->id == IPI_VCU_INIT) ? IPI_VENC_COMMON : vcu->id;
	vcu->handler = vcu_enc_ipi_handler;

	err = VCU_FPTR(vcu_ipi_register)(vcu->dev,
		vcu->id, vcu->handler, NULL, vcu->ctx->dev);
	if (err != 0) {
		mtk_vcodec_err(vcu, "vcu_ipi_register fail status=%d", err);
		return err;
	}

	memset(&msg, 0, sizeof(msg));
	msg.msg_id = AP_IPIMSG_ENC_QUERY_CAP;
	msg.id = id;
	msg.ap_inst_addr = (unsigned long)vcu->ctx->id;
	msg.ctx_id = vcu->ctx->id;

	vcu_enc_set_pid(vcu);
	err = vcu_enc_send_msg(vcu, &msg, sizeof(msg));
	mtk_vcodec_debug(vcu, "- id=%X ret=%d", msg.msg_id, err);

	return err;
}

int vcu_enc_set_param(struct venc_vcu_inst *vcu,
					  enum venc_set_param_type id,
					  struct venc_enc_param *enc_param)
{
	struct venc_ap_ipi_msg_set_param out;

	mtk_vcodec_debug(vcu, "id %d ->", id);
	if (sizeof(out) > SHARE_BUF_SIZE) {
		mtk_vcodec_err(vcu, "venc_ap_ipi_msg_set_param cannot be large than %d",
					   SHARE_BUF_SIZE);
		return -EINVAL;
	}

	memset(&out, 0, sizeof(out));
	out.msg_id = AP_IPIMSG_ENC_SET_PARAM;
	out.vcu_inst_addr = vcu->inst_addr;
	out.ctx_id = vcu->ctx->id;
	out.param_id = id;
	switch (id) {
	case VENC_SET_PARAM_ENC:
		out.data_item = 0;
		break;
	case VENC_SET_PARAM_FORCE_INTRA:
		out.data_item = 0;
		break;
	case VENC_SET_PARAM_ADJUST_BITRATE:
		out.data_item = 1;
		out.data[0] = enc_param->bitrate;
		break;
	case VENC_SET_PARAM_ADJUST_FRAMERATE:
		out.data_item = 1;
		out.data[0] = enc_param->frm_rate;
		break;
	case VENC_SET_PARAM_GOP_SIZE:
		out.data_item = 1;
		out.data[0] = enc_param->gop_size;
		break;
	case VENC_SET_PARAM_INTRA_PERIOD:
		out.data_item = 1;
		out.data[0] = enc_param->intra_period;
		break;
	case VENC_SET_PARAM_SKIP_FRAME:
		out.data_item = 0;
		break;
	case VENC_SET_PARAM_PREPEND_HEADER:
		out.data_item = 0;
		break;
	case VENC_SET_PARAM_SCENARIO:
		out.data_item = 1;
		out.data[0] = enc_param->scenario;
		break;
	case VENC_SET_PARAM_NONREFP:
		out.data_item = 1;
		out.data[0] = enc_param->nonrefp;
		break;
	case VENC_SET_PARAM_NONREFPFREQ:
		out.data_item = 1;
		out.data[0] = enc_param->nonrefpfreq;
		break;
	case VENC_SET_PARAM_DETECTED_FRAMERATE:
		out.data_item = 1;
		out.data[0] = enc_param->detectframerate;
		break;
	case VENC_SET_PARAM_RFS_ON:
		out.data_item = 1;
		out.data[0] = enc_param->rfs;
		break;
	case VENC_SET_PARAM_PREPEND_SPSPPS_TO_IDR:
		out.data_item = 1;
		out.data[0] = enc_param->prependheader;
		break;
	case VENC_SET_PARAM_OPERATION_RATE:
		out.data_item = 1;
		out.data[0] = enc_param->operationrate;
		break;
	case VENC_SET_PARAM_BITRATE_MODE:
		out.data_item = 1;
		out.data[0] = enc_param->bitratemode;
		break;
	case VENC_SET_PARAM_ROI_ON:
		out.data_item = 1;
		out.data[0] = enc_param->roion;
		break;
	case VENC_SET_PARAM_HEIF_GRID_SIZE:
		out.data_item = 1;
		out.data[0] = enc_param->heif_grid_size;
		break;
	case VENC_SET_PARAM_COLOR_DESC:
		out.data_item = 0; // passed via vsi
		break;
	case VENC_SET_PARAM_SEC_MODE:
		out.data_item = 1;
		out.data[0] = enc_param->svp_mode;
		break;
	case VENC_SET_PARAM_TSVC:
		out.data_item = 1;
		out.data[0] = enc_param->tsvc;
		break;
	case VENC_SET_PARAM_ENABLE_HIGHQUALITY:
		out.data_item = 1;
		out.data[0] = enc_param->highquality;
		break;
	case VENC_SET_PARAM_ADJUST_MAX_QP:
		out.data_item = 1;
		out.data[0] = enc_param->max_qp;
		break;
	case VENC_SET_PARAM_ADJUST_MIN_QP:
		out.data_item = 1;
		out.data[0] = enc_param->min_qp;
		break;
	case VENC_SET_PARAM_ADJUST_I_P_QP_DELTA:
		out.data_item = 1;
		out.data[0] = enc_param->ip_qpdelta;
		break;
	case VENC_SET_PARAM_ADJUST_QP_CONTROL_MODE:
		out.data_item = 1;
		out.data[0] = enc_param->qp_control_mode;
		break;
	case VENC_SET_PARAM_ADJUST_FRAME_LEVEL_QP:
		out.data_item = 1;
		out.data[0] = enc_param->framelvl_qp;
		break;
	case VENC_SET_PARAM_ENABLE_DUMMY_NAL:
		out.data_item = 1;
		out.data[0] = enc_param->dummynal;
		break;
	case VENC_SET_PARAM_TEMPORAL_LAYER_CNT:
		out.data_item = 2;
		out.data[0] = enc_param->temporal_layer_pcount;
		out.data[1] = enc_param->temporal_layer_bcount;
		break;
	default:
		mtk_vcodec_err(vcu, "id %d not supported", id);
		return -EINVAL;
	}

	if (vcu_enc_send_msg(vcu, &out, sizeof(out))) {
		mtk_vcodec_err(vcu,
			"AP_IPIMSG_ENC_SET_PARAM %d fail", id);
		return -EINVAL;
	}

	mtk_vcodec_debug(vcu, "id %d <-", id);

	return 0;
}

int vcu_enc_encode(struct venc_vcu_inst *vcu, unsigned int bs_mode,
				   struct venc_frm_buf *frm_buf,
				   struct mtk_vcodec_mem *bs_buf,
				   unsigned int *bs_size)
{

	struct venc_ap_ipi_msg_enc out;
	struct venc_ap_ipi_msg_set_param out_slb;
	struct venc_vsi *vsi = (struct venc_vsi *)vcu->vsi;
	unsigned int i, ret, ret_slb;

	mtk_vcodec_debug(vcu, "bs_mode %d ->", bs_mode);

	if (sizeof(out) > SHARE_BUF_SIZE) {
		mtk_vcodec_err(vcu, "venc_ap_ipi_msg_enc cannot be large than %d",
					   SHARE_BUF_SIZE);
		return -EINVAL;
	}

	memset(&out, 0, sizeof(out));
	out.msg_id = AP_IPIMSG_ENC_ENCODE;
	out.vcu_inst_addr = vcu->inst_addr;
	out.ctx_id = vcu->ctx->id;
	out.bs_mode = bs_mode;
	if (frm_buf) {
		out.fb_num_planes = frm_buf->num_planes;
		for (i = 0; i < frm_buf->num_planes; i++) {
			vsi->venc.input_addr[i] =
				frm_buf->fb_addr[i].dma_addr;
			vsi->venc.fb_dma[i] =
				frm_buf->fb_addr[i].dma_addr;
			out.input_size[i] =
				frm_buf->fb_addr[i].size;
			out.data_offset[i] =
				frm_buf->fb_addr[i].data_offset;
		}
		if (frm_buf->has_meta) {
			vsi->meta_addr = frm_buf->meta_addr;
			vsi->meta_size = sizeof(struct mtk_hdr_dynamic_info);
			vsi->meta_offset = frm_buf->meta_offset;
		} else {
			vsi->meta_size = 0;
			vsi->meta_addr = 0;
		}

		if (frm_buf->has_qpmap) {
			vsi->qpmap_addr = frm_buf->qpmap_dma_addr;
			vsi->qpmap_size = frm_buf->qpmap_dma->size;
		} else {
			vsi->qpmap_addr = 0;
			vsi->qpmap_size = 0;
		}

		if (frm_buf->dyparams_dma) {
			vsi->dynamicparams_addr = frm_buf->dyparams_dma_addr;
			vsi->dynamicparams_size = sizeof(struct inputqueue_dynamic_info);
		} else {
			vsi->dynamicparams_addr = 0;
			vsi->dynamicparams_size = 0;
		}

		mtk_vcodec_debug(vcu, " num_planes = %d input (dmabuf:%lx), size %d %llx",
			frm_buf->num_planes,
			(unsigned long)frm_buf->fb_addr[0].dmabuf,
			vsi->meta_size,
			vsi->meta_addr);
		mtk_vcodec_debug(vcu, "vsi qpmap addr %llx size%d",
			vsi->meta_addr, vsi->qpmap_size);
	} else {
		mtk_vcodec_debug(vcu, "frm_buf is null");
	}

	if (bs_buf) {
		vsi->venc.bs_addr = bs_buf->dma_addr;
		vsi->venc.bs_dma = bs_buf->dma_addr;
		out.bs_size = bs_buf->size;
		mtk_vcodec_debug(vcu, " output (dma:%lx)",
			(unsigned long)bs_buf->dmabuf);
	} else {
		mtk_vcodec_debug(vcu, "bs_buf is null");
	}

	mutex_lock(vcu->ctx_ipi_lock);
	if (vcu->ctx->use_slbc && atomic_read(&mtk_venc_slb_cb.release_slbc)) {
		memset(&out_slb, 0, sizeof(out_slb));
		out_slb.msg_id = AP_IPIMSG_ENC_SET_PARAM;
		out_slb.vcu_inst_addr = vcu->inst_addr;
		out_slb.ctx_id = vcu->ctx->id;
		out_slb.param_id = VENC_SET_PARAM_RELEASE_SLB;
		out_slb.data_item = 2;
		out_slb.data[0] = 1; //release_slb 1
		out_slb.data[1] = 0x0; //slbc_addr
		ret_slb = vcu_enc_send_msg(vcu, &out_slb, sizeof(out_slb));

		if (ret_slb)
			mtk_vcodec_err(vcu, "set VENC_SET_PARAM_RELEASE_SLB fail %d", ret_slb);
		else {
			mtk_v4l2_debug(0, "slbc_release, %p\n", &vcu->ctx->sram_data);
			slbc_release(&vcu->ctx->sram_data);
			vcu->ctx->use_slbc = 0;
			atomic_inc(&mtk_venc_slb_cb.later_cnt);
			if (vcu->ctx->enc_params.slbc_encode_performance)
				atomic_dec(&mtk_venc_slb_cb.perf_used_cnt);
			mtk_v4l2_debug(0, "slbc_release ref %d\n", vcu->ctx->sram_data.ref);
			if (vcu->ctx->sram_data.ref <= 0)
				atomic_set(&mtk_venc_slb_cb.release_slbc, 0);
		}

		mtk_v4l2_debug(0, "slb_cb %d/%d perf %d cnt %d/%d",
			atomic_read(&mtk_venc_slb_cb.release_slbc),
			atomic_read(&mtk_venc_slb_cb.request_slbc),
			vcu->ctx->enc_params.slbc_encode_performance,
			atomic_read(&mtk_venc_slb_cb.perf_used_cnt),
			atomic_read(&mtk_venc_slb_cb.later_cnt));
	} else if (!vcu->ctx->use_slbc && atomic_read(&mtk_venc_slb_cb.request_slbc)) {
		if (slbc_request(&vcu->ctx->sram_data) >= 0) {
			vcu->ctx->use_slbc = 1;
			vcu->ctx->slbc_addr = (unsigned int)(unsigned long)
			vcu->ctx->sram_data.paddr;
		} else {
			mtk_vcodec_err(vcu, "slbc_request fail\n");
			vcu->ctx->use_slbc = 0;
		}
		if (vcu->ctx->slbc_addr % 256 != 0 || vcu->ctx->slbc_addr == 0) {
			mtk_vcodec_err(vcu, "slbc_addr error 0x%x\n", vcu->ctx->slbc_addr);
			vcu->ctx->use_slbc = 0;
		}

		if (vcu->ctx->use_slbc == 1) {
			if (vcu->ctx->enc_params.slbc_encode_performance)
				atomic_inc(&mtk_venc_slb_cb.perf_used_cnt);

			atomic_dec(&mtk_venc_slb_cb.later_cnt);
			if (atomic_read(&mtk_venc_slb_cb.later_cnt) <= 0)
				atomic_set(&mtk_venc_slb_cb.request_slbc, 0);

			memset(&out_slb, 0, sizeof(out_slb));
			out_slb.msg_id = AP_IPIMSG_ENC_SET_PARAM;
			out_slb.vcu_inst_addr = vcu->inst_addr;
			out_slb.ctx_id = vcu->ctx->id;
			out_slb.param_id = VENC_SET_PARAM_RELEASE_SLB;
			out_slb.data_item = 2;
			out_slb.data[0] = 0; //release_slb 0
			out_slb.data[1] = vcu->ctx->slbc_addr;
			ret_slb = vcu_enc_send_msg(vcu, &out_slb, sizeof(out_slb));
			if (ret_slb) {
				mtk_vcodec_err(vcu, "set VENC_SET_PARAM_RELEASE_SLB fail %d",
				ret_slb);
			}
		}
		mtk_v4l2_debug(0, "slbc_request %d, 0x%x, 0x%llx\n",
		vcu->ctx->use_slbc, vcu->ctx->slbc_addr, vcu->ctx->sram_data.paddr);
		mtk_v4l2_debug(0, "slb_cb %d/%d perf %d cnt %d/%d",
			atomic_read(&mtk_venc_slb_cb.release_slbc),
			atomic_read(&mtk_venc_slb_cb.request_slbc),
			vcu->ctx->enc_params.slbc_encode_performance,
			atomic_read(&mtk_venc_slb_cb.perf_used_cnt),
			atomic_read(&mtk_venc_slb_cb.later_cnt));
	}

	vcu_enc_set_ctx(vcu, frm_buf, bs_buf);
	ret = vcu_enc_send_msg(vcu, &out, sizeof(out));
	mutex_unlock(vcu->ctx_ipi_lock);

	if (ret) {
		mtk_vcodec_err(vcu, "AP_IPIMSG_ENC_ENCODE %d fail %d", bs_mode, ret);
		return ret;
	}
	mtk_vcodec_debug(vcu, " vcu_enc_send_msg done");

	mtk_vcodec_debug(vcu, "bs_mode %d size %d key_frm %d <-",
		bs_mode, vcu->bs_size, vcu->is_key_frm);

	return 0;
}

int vcu_enc_deinit(struct venc_vcu_inst *vcu)
{
	struct venc_ap_ipi_msg_deinit out;
	int ret = 0;

	mtk_vcodec_debug_enter(vcu);

	if (sizeof(out) > SHARE_BUF_SIZE) {
		mtk_vcodec_err(vcu, "venc_ap_ipi_msg_deint cannot be large than %d",
					   SHARE_BUF_SIZE);
		return -EINVAL;
	}

	memset(&out, 0, sizeof(out));
	out.msg_id = AP_IPIMSG_ENC_DEINIT;
	out.vcu_inst_addr = vcu->inst_addr;
	out.ctx_id = vcu->ctx->id;

	mutex_lock(vcu->ctx_ipi_lock);
	ret = vcu_enc_send_msg(vcu, &out, sizeof(out));
	vcu_enc_clear_ctx(vcu);
	mutex_unlock(vcu->ctx_ipi_lock);
	current->flags &= ~PF_NOFREEZE;

	mtk_vcodec_debug_leave(vcu);

	return ret;
}

