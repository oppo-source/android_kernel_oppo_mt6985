// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 MediaTek Inc.
 * Authors:
 *	Stanley Chu <stanley.chu@mediatek.com>
 */
#include <linux/atomic.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/sched/clock.h>
#include <linux/seq_file.h>
#include <linux/smp.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/tracepoint.h>
#include "governor.h"
#if IS_ENABLED(CONFIG_MTK_AEE_IPANIC)
#include <mt-plat/mrdump.h>
#endif
#include "ufshcd.h"
#include "ufs-mediatek.h"
#include "ufs-mediatek-dbg.h"
#include "ufs-mediatek-trace.h"

#define MAX_CMD_HIST_ENTRY_CNT (500)
#define UFS_AEE_BUFFER_SIZE (100 * 1024)

/*
 * Currently only global variables are used.
 *
 * For scalability, may introduce multiple history
 * instances bound to each device in the future.
 */
static bool cmd_hist_initialized;
static bool cmd_hist_enabled;
static spinlock_t cmd_hist_lock;
static unsigned int cmd_hist_cnt;
static unsigned int cmd_hist_ptr = MAX_CMD_HIST_ENTRY_CNT - 1;
static struct cmd_hist_struct *cmd_hist;
static struct ufs_hba *ufshba;
static char *ufs_aee_buffer;

static void ufs_mtk_dbg_print_err_hist(char **buff, unsigned long *size,
				  struct seq_file *m, u32 id,
				  char *err_name)
{
	int i;
	bool found = false;
	struct ufs_event_hist *e;
	struct ufs_hba *hba = ufshba;

	if (id >= UFS_EVT_CNT)
		return;

	e = &hba->ufs_stats.event[id];

	for (i = 0; i < UFS_EVENT_HIST_LENGTH; i++) {
		int p = (i + e->pos) % UFS_EVENT_HIST_LENGTH;

		if (e->tstamp[p] == 0)
			continue;
		SPREAD_PRINTF(buff, size, m,
			"%s[%d] = 0x%x at %lld us\n", err_name, p,
			e->val[p], ktime_to_us(e->tstamp[p]));
		found = true;
	}

	if (!found)
		SPREAD_PRINTF(buff, size, m, "No record of %s\n", err_name);
}

static void ufs_mtk_dbg_print_info(char **buff, unsigned long *size,
			    struct seq_file *m)
{
	struct ufs_mtk_host *host;
	struct ufs_hba *hba = ufshba;
	struct ufs_hba_private *hba_priv;
	int i;

	if (!hba)
		return;

	hba_priv = (struct ufs_hba_private *)hba->android_vendor_data1;

	host = ufshcd_get_variant(hba);

	/* Host state */
	SPREAD_PRINTF(buff, size, m,
		      "UFS Host state=%d\n", hba->ufshcd_state);
	SPREAD_PRINTF(buff, size, m,
		      "outstanding reqs=0x%lx tasks=0x%lx\n",
		      hba->outstanding_reqs, hba->outstanding_tasks);
	SPREAD_PRINTF(buff, size, m,
		      "saved_err=0x%x, saved_uic_err=0x%x\n",
		      hba->saved_err, hba->saved_uic_err);
	SPREAD_PRINTF(buff, size, m,
		      "Device power mode=%d, UIC link state=%d\n",
		      hba->curr_dev_pwr_mode, hba->uic_link_state);
	SPREAD_PRINTF(buff, size, m,
		      "PM in progress=%d, sys. suspended=%d\n",
		      hba->pm_op_in_progress, hba->is_sys_suspended);
	SPREAD_PRINTF(buff, size, m,
		      "Auto BKOPS=%d, Host self-block=%d\n",
		      hba->auto_bkops_enabled,
		      hba->host->host_self_blocked);
	SPREAD_PRINTF(buff, size, m,
		      "Clk scale sup./en=%d/%d, min g.=G%d, polling_ms=%d, upthr=%d, downthr=%d\n",
		    !!ufshcd_is_clkscaling_supported(hba),
			hba->clk_scaling.is_enabled,
			hba->clk_scaling.min_gear,
			hba->vps->devfreq_profile.polling_ms,
			hba->vps->ondemand_data.upthreshold,
			hba->vps->ondemand_data.downdifferential
	);
	if (ufshcd_is_clkgating_allowed(hba))
		SPREAD_PRINTF(buff, size, m,
			      "Clk gate=%d, suspended=%d, active_reqs=%d\n",
			      hba->clk_gating.state,
			      hba->clk_gating.is_suspended,
			      hba->clk_gating.active_reqs);
	else
		SPREAD_PRINTF(buff, size, m,
			      "clk_gating is disabled\n");
#ifdef CONFIG_PM
	SPREAD_PRINTF(buff, size, m,
		      "Runtime PM: req=%d, status:%d, err:%d\n",
		      hba->dev->power.request,
		      hba->dev->power.runtime_status,
		      hba->dev->power.runtime_error);
#endif
	SPREAD_PRINTF(buff, size, m,
		      "error handling flags=0x%x, req. abort count=%d\n",
		      hba->eh_flags, hba->req_abort_count);
	SPREAD_PRINTF(buff, size, m,
		      "Host capabilities=0x%x, hba-caps=0x%x, mtk-caps:0x%x\n",
		      hba->capabilities, hba->caps, host->caps);
	SPREAD_PRINTF(buff, size, m,
		      "quirks=0x%x, dev. quirks=0x%x\n", hba->quirks,
		      hba->dev_quirks);
	SPREAD_PRINTF(buff, size, m,
		      "hba->ufs_version = 0x%x, hba->capabilities = 0x%x\n",
		      hba->ufs_version, hba->capabilities);
	SPREAD_PRINTF(buff, size, m,
		      "last_hibern8_exit_tstamp at %lld us, hibern8_exit_cnt = %d\n",
		      ktime_to_us(hba->ufs_stats.last_hibern8_exit_tstamp),
		      hba->ufs_stats.hibern8_exit_cnt);

	/* PWR info */
	SPREAD_PRINTF(buff, size, m,
		      "[RX, TX]: gear=[%d, %d], lane[%d, %d], pwr[%d, %d], rate = %d\n",
		      hba->pwr_info.gear_rx, hba->pwr_info.gear_tx,
		      hba->pwr_info.lane_rx, hba->pwr_info.lane_tx,
		      hba->pwr_info.pwr_rx,
		      hba->pwr_info.pwr_tx,
		      hba->pwr_info.hs_rate);

	/* Device info */
	SPREAD_PRINTF(buff, size, m,
		      "Device vendor=%.8s, model=%.16s, rev=%.4s\n",
		      hba->sdev_ufs_device->vendor,
		      hba->sdev_ufs_device->model, hba->sdev_ufs_device->rev);

	if (hba_priv->is_mcq_enabled) {
		SPREAD_PRINTF(buff, size, m,
				  "MCQ enable: yes\n");

		for (i = 0; i < hba_priv->mcq_q_cfg.sq_nr; i++)
			SPREAD_PRINTF(buff, size, m,
					  "MCQ sent packet, q_index=%d, count=%i\n",
					  i, hba_priv->mcq_q_cfg.sent_cmd_count[i]);
	} else {
		SPREAD_PRINTF(buff, size, m,
				  "MCQ enable: no\n");
	}

	/* Error history */
	ufs_mtk_dbg_print_err_hist(buff, size, m,
			      UFS_EVT_PA_ERR, "pa_err");
	ufs_mtk_dbg_print_err_hist(buff, size, m,
			      UFS_EVT_DL_ERR, "dl_err");
	ufs_mtk_dbg_print_err_hist(buff, size, m,
			      UFS_EVT_NL_ERR, "nl_err");
	ufs_mtk_dbg_print_err_hist(buff, size, m,
			      UFS_EVT_TL_ERR, "tl_err");
	ufs_mtk_dbg_print_err_hist(buff, size, m,
			      UFS_EVT_DME_ERR, "dme_err");
	ufs_mtk_dbg_print_err_hist(buff, size, m,
			      UFS_EVT_AUTO_HIBERN8_ERR,
			      "auto_hibern8_err");
	ufs_mtk_dbg_print_err_hist(buff, size, m,
			      UFS_EVT_FATAL_ERR, "fatal_err");
	ufs_mtk_dbg_print_err_hist(buff, size, m,
			      UFS_EVT_LINK_STARTUP_FAIL,
			      "link_startup_fail");
	ufs_mtk_dbg_print_err_hist(buff, size, m,
			      UFS_EVT_RESUME_ERR, "resume_fail");
	ufs_mtk_dbg_print_err_hist(buff, size, m,
			      UFS_EVT_SUSPEND_ERR, "suspend_fail");
	ufs_mtk_dbg_print_err_hist(buff, size, m,
			      UFS_EVT_WL_RES_ERR, "wlun resume_fail");
	ufs_mtk_dbg_print_err_hist(buff, size, m,
			      UFS_EVT_WL_SUSP_ERR, "wlun suspend_fail");
	ufs_mtk_dbg_print_err_hist(buff, size, m,
			      UFS_EVT_DEV_RESET, "dev_reset");
	ufs_mtk_dbg_print_err_hist(buff, size, m,
			      UFS_EVT_HOST_RESET, "host_reset");
	ufs_mtk_dbg_print_err_hist(buff, size, m,
			      UFS_EVT_ABORT, "task_abort");
}

static int cmd_hist_get_entry(void)
{
	unsigned long flags;
	unsigned int ptr;

	spin_lock_irqsave(&cmd_hist_lock, flags);
	cmd_hist_ptr++;
	if (cmd_hist_ptr >= MAX_CMD_HIST_ENTRY_CNT)
		cmd_hist_ptr = 0;
	ptr = cmd_hist_ptr;
	spin_unlock_irqrestore(&cmd_hist_lock, flags);

	cmd_hist_cnt++;

	/* Initialize common fields */
	cmd_hist[ptr].cpu = smp_processor_id();
	cmd_hist[ptr].duration = 0;
	cmd_hist[ptr].pid = current->pid;
	cmd_hist[ptr].time = local_clock();

	return ptr;
}

static int cmd_hist_get_prev_ptr(int ptr)
{
	if (ptr == 0)
		return MAX_CMD_HIST_ENTRY_CNT - 1;
	else
		return (ptr - 1);
}

static void probe_android_vh_ufs_send_tm_command(void *data, struct ufs_hba *hba,
						 int tag, int str_t)
{
	u8 tm_func;
	int ptr, lun, task_tag;
	enum cmd_hist_event event = CMD_UNKNOWN;
	enum ufs_trace_str_t _str_t = str_t;
	struct utp_task_req_desc *d = &hba->utmrdl_base_addr[tag];

	if (!cmd_hist_enabled)
		return;

	lun = (be32_to_cpu(d->header.dword_0) >> 8) & 0xFF;
	task_tag = be32_to_cpu(d->header.dword_0) & 0xFF;
	tm_func = (be32_to_cpu(d->header.dword_1) >> 16) & 0xFFFF;

	switch (_str_t){
	case UFS_TM_SEND:
		event = CMD_TM_SEND;
		break;
	case UFS_TM_COMP:
		event = CMD_TM_COMPLETED;
		break;
	case UFS_TM_ERR:
		event = CMD_TM_COMPLETED_ERR;
		break;
	default:
		pr_notice("%s: undefined TM command (0x%x)", __func__, _str_t);
		break;
	}

	ptr = cmd_hist_get_entry();

	cmd_hist[ptr].event = event;
	cmd_hist[ptr].cmd.tm.lun = lun;
	cmd_hist[ptr].cmd.tm.tag = tag;
	cmd_hist[ptr].cmd.tm.task_tag = task_tag;
	cmd_hist[ptr].cmd.tm.tm_func = tm_func;
}

static void cmd_hist_add_dev_cmd(struct ufs_hba *hba,
				 struct ufshcd_lrb *lrbp,
				 enum cmd_hist_event event)
{
	int ptr;

	ptr = cmd_hist_get_entry();

	cmd_hist[ptr].event = event;
	cmd_hist[ptr].cmd.dev.type = hba->dev_cmd.type;

	if (hba->dev_cmd.type == DEV_CMD_TYPE_NOP)
		return;

	cmd_hist[ptr].cmd.dev.tag = lrbp->task_tag;
	cmd_hist[ptr].cmd.dev.opcode =
		hba->dev_cmd.query.request.upiu_req.opcode;
	cmd_hist[ptr].cmd.dev.idn =
		hba->dev_cmd.query.request.upiu_req.idn;
	cmd_hist[ptr].cmd.dev.index =
		hba->dev_cmd.query.request.upiu_req.index;
	cmd_hist[ptr].cmd.dev.selector =
		hba->dev_cmd.query.request.upiu_req.selector;
}

static void probe_android_vh_ufs_send_command(void *data, struct ufs_hba *hba,
					      struct ufshcd_lrb *lrbp)
{
	if (!cmd_hist_enabled)
		return;

	if (lrbp->cmd)
		return;

	cmd_hist_add_dev_cmd(hba, lrbp, CMD_DEV_SEND);
}

static void probe_android_vh_ufs_compl_command(void *data, struct ufs_hba *hba,
					      struct ufshcd_lrb *lrbp)
{
	if (!cmd_hist_enabled)
		return;

	if (lrbp->cmd)
		return;

	cmd_hist_add_dev_cmd(hba, lrbp, CMD_DEV_COMPLETED);
}

static void probe_ufshcd_command(void *data, const char *dev_name,
				 enum ufs_trace_str_t str_t, unsigned int tag,
				 u32 doorbell, int transfer_len,
				 u32 intr, u64 lba, u8 opcode, u8 group_id)
{
	int ptr, ptr_cur;
	enum cmd_hist_event event;

	if (!cmd_hist_enabled)
		return;

	if (str_t == UFS_CMD_SEND)
		event = CMD_SEND;
	else if (str_t == UFS_CMD_COMP)
		event = CMD_COMPLETED;
	else
		return;

	ptr = cmd_hist_get_entry();

	cmd_hist[ptr].event = event;
	cmd_hist[ptr].cmd.utp.tag = tag;
	cmd_hist[ptr].cmd.utp.transfer_len = transfer_len;
	cmd_hist[ptr].cmd.utp.lba = lba;
	cmd_hist[ptr].cmd.utp.opcode = opcode;
	cmd_hist[ptr].cmd.utp.doorbell = doorbell;
	cmd_hist[ptr].cmd.utp.intr = intr;

	/* Need patch trace_ufshcd_command() first */
	cmd_hist[ptr].cmd.utp.crypt_en = 0;
	cmd_hist[ptr].cmd.utp.crypt_keyslot = 0;

	if (event == CMD_COMPLETED) {
		ptr_cur = ptr;
		ptr = cmd_hist_get_prev_ptr(ptr);
		while (1) {
			if (cmd_hist[ptr].cmd.utp.tag == tag) {
				cmd_hist[cmd_hist_ptr].duration =
					local_clock() - cmd_hist[ptr].time;
				break;
			}
			ptr = cmd_hist_get_prev_ptr(ptr);
			if (ptr == ptr_cur)
				break;
		}
	}
}

static void probe_ufshcd_uic_command(void *data, const char *dev_name,
				     enum ufs_trace_str_t str_t, u32 cmd,
				     u32 arg1, u32 arg2, u32 arg3)
{
	int ptr, ptr_cur;
	enum cmd_hist_event event;

	if (!cmd_hist_enabled)
		return;

	ptr = cmd_hist_get_entry();

	if (str_t == UFS_CMD_SEND)
		event = CMD_UIC_SEND;
	else
		event = CMD_UIC_CMPL_GENERAL;

	cmd_hist[ptr].event = event;
	cmd_hist[ptr].cmd.uic.cmd = cmd;
	cmd_hist[ptr].cmd.uic.arg1 = arg1;
	cmd_hist[ptr].cmd.uic.arg2 = arg2;
	cmd_hist[ptr].cmd.uic.arg3 = arg3;

	if (event == CMD_UIC_CMPL_GENERAL) {
		ptr_cur = ptr;
		ptr = cmd_hist_get_prev_ptr(ptr);
		while (1) {
			if (cmd_hist[ptr].cmd.uic.cmd == cmd) {
				cmd_hist[cmd_hist_ptr].duration =
					local_clock() - cmd_hist[ptr].time;
				break;
			}
			ptr = cmd_hist_get_prev_ptr(ptr);
			if (ptr == ptr_cur)
				break;
		}
	}
}

static void probe_ufshcd_clk_gating(void *data, const char *dev_name,
				    int state)
{
	int ptr;
#if IS_ENABLED(CONFIG_MTK_UFS_DEBUG)
	struct ufs_mtk_host *host = ufshcd_get_variant(ufshba);
	u32 val;
#endif

	if (!cmd_hist_enabled)
		return;

	ptr = cmd_hist_get_entry();

	cmd_hist[ptr].event = CMD_CLK_GATING;
	cmd_hist[ptr].cmd.clk_gating.state = state;

#if IS_ENABLED(CONFIG_MTK_UFS_DEBUG)
	if (state == CLKS_ON && host->mphy_base) {
		writel(0xC1000200, host->mphy_base + 0x20C0);
		cmd_hist[ptr].cmd.clk_gating.arg1 =
			readl(host->mphy_base + 0xA09C);
		cmd_hist[ptr].cmd.clk_gating.arg2 =
			readl(host->mphy_base + 0xA19C);
		writel(0, host->mphy_base + 0x20C0);
	} else if (state == REQ_CLKS_OFF && host->mphy_base) {
		writel(0xC1000200, host->mphy_base + 0x20C0);
		cmd_hist[ptr].cmd.clk_gating.arg1 =
			readl(host->mphy_base + 0xA09C);
		cmd_hist[ptr].cmd.clk_gating.arg2 =
			readl(host->mphy_base + 0xA19C);
		writel(0, host->mphy_base + 0x20C0);

		/* when req clk off, clear 2 line hw status */
		val = readl(host->mphy_base + 0xA800) | 0x02;
		writel(val, host->mphy_base + 0xA800);
		writel(val, host->mphy_base + 0xA800);
		val = val & (~0x02);
		writel(val, host->mphy_base + 0xA800);

		val = readl(host->mphy_base + 0xA900) | 0x02;
		writel(val, host->mphy_base + 0xA900);
		writel(val, host->mphy_base + 0xA900);
		val = val & (~0x02);
		writel(val, host->mphy_base + 0xA900);

		val = readl(host->mphy_base + 0xA804) | 0x02;
		writel(val, host->mphy_base + 0xA804);
		writel(val, host->mphy_base + 0xA804);
		val = val & (~0x02);
		writel(val, host->mphy_base + 0xA804);

		val = readl(host->mphy_base + 0xA904) | 0x02;
		writel(val, host->mphy_base + 0xA904);
		writel(val, host->mphy_base + 0xA904);
		val = val & (~0x02);
		writel(val, host->mphy_base + 0xA904);

		/* check status is already clear */
		if (readl(host->mphy_base + 0xA808) ||
			readl(host->mphy_base + 0xA908)) {

			pr_info("%s: [%d] clear fail 0x%x 0x%x\n",
				__func__, __LINE__,
				readl(host->mphy_base + 0xA808),
				readl(host->mphy_base + 0xA908)
				);
		}
	} else {
		cmd_hist[ptr].cmd.clk_gating.arg1 = 0;
		cmd_hist[ptr].cmd.clk_gating.arg2 = 0;
	}
	cmd_hist[ptr].cmd.clk_gating.arg3 = 0;
#endif
}

static void probe_ufshcd_profile_clk_scaling(void *data, const char *dev_name,
	const char *profile_info, s64 time_us, int err)
{
	int ptr;

	if (!cmd_hist_enabled)
		return;

	ptr = cmd_hist_get_entry();

	cmd_hist[ptr].event = CMD_CLK_SCALING;
	if (!strcmp(profile_info, "up"))
		cmd_hist[ptr].cmd.clk_scaling.state = CLKS_SCALE_UP;
	else
		cmd_hist[ptr].cmd.clk_scaling.state = CLKS_SCALE_DOWN;
	cmd_hist[ptr].cmd.clk_scaling.err = err;
}

static void probe_ufshcd_pm(void *data, const char *dev_name,
			    int err, s64 time_us,
			    int pwr_mode, int link_state,
			    enum ufsdbg_pm_state state)
{
	int ptr;

	if (!cmd_hist_enabled)
		return;

	ptr = cmd_hist_get_entry();

	cmd_hist[ptr].event = CMD_PM;
	cmd_hist[ptr].cmd.pm.state = state;
	cmd_hist[ptr].cmd.pm.err = err;
	cmd_hist[ptr].cmd.pm.time_us = time_us;
	cmd_hist[ptr].cmd.pm.pwr_mode = pwr_mode;
	cmd_hist[ptr].cmd.pm.link_state = link_state;
}

static void probe_ufshcd_runtime_suspend(void *data, const char *dev_name,
			    int err, s64 time_us,
			    int pwr_mode, int link_state)
{
	probe_ufshcd_pm(data, dev_name, err, time_us, pwr_mode, link_state,
			UFSDBG_RUNTIME_SUSPEND);
}

static void probe_ufshcd_runtime_resume(void *data, const char *dev_name,
			    int err, s64 time_us,
			    int pwr_mode, int link_state)
{
	probe_ufshcd_pm(data, dev_name, err, time_us, pwr_mode, link_state,
			UFSDBG_RUNTIME_RESUME);
}

static void probe_ufshcd_system_suspend(void *data, const char *dev_name,
			    int err, s64 time_us,
			    int pwr_mode, int link_state)
{
	probe_ufshcd_pm(data, dev_name, err, time_us, pwr_mode, link_state,
			UFSDBG_SYSTEM_SUSPEND);
}

static void probe_ufshcd_system_resume(void *data, const char *dev_name,
			    int err, s64 time_us,
			    int pwr_mode, int link_state)
{
	probe_ufshcd_pm(data, dev_name, err, time_us, pwr_mode, link_state,
			UFSDBG_SYSTEM_RESUME);
}
//bsp.storage.ufs 2021.10.14 add for /proc/devinfo/ufs
#ifdef OPLUS_DEVINFO_UFS
/*feature-devinfo-v001-1-begin*/
static int create_devinfo_ufs(struct scsi_device *sdev)
{
	static char temp_version[5] = {0};
	static char vendor[9] = {0};
	static char model[17] = {0};
	int ret = 0;

	pr_info("get ufs device vendor/model/rev\n");
	WARN_ON(!sdev);
	strncpy(temp_version, sdev->rev, 4);
	strncpy(vendor, sdev->vendor, 8);
	strncpy(model, sdev->model, 16);

	ret = register_device_proc("ufs_version", temp_version, vendor);

	if (ret) {
		pr_err("%s create ufs_version fail, ret=%d",__func__,ret);
		return ret;
	}

	ret = register_device_proc("ufs", model, vendor);

	if (ret) {
		pr_err("%s create ufs fail, ret=%d",__func__,ret);
	}

	return ret;
}
/*feature-devinfo-v001-1-end*/
static void probe_android_vh_ufs_update_sdev(void *data, struct scsi_device *sdev)
{
	pr_info_once("%s ret=%d",__func__,create_devinfo_ufs(sdev));
}
#endif

/* trace point enable condition */
enum tp_en {
	TP_EN_ALL,
	TP_EN_LEGACY,
	TP_EN_MCQ,
};

/*
 * Data structures to store tracepoints information
 */
struct tracepoints_table {
	const char *name;
	void *func;
	struct tracepoint *tp;
	bool init;
	enum tp_en en;
};

struct mod_tracepoints_table {
	const char *mod_name;
	const char *name;
	void *func;
	struct tracepoint *tp;
	bool init;
	enum tp_en en;
};


static struct mod_tracepoints_table mod_interests[] = {
	{
		.mod_name = "ufs_mediatek_mod",
		.name = "ufs_mtk_mcq_command",
		.func = probe_ufshcd_command,
		.en = TP_EN_MCQ
	}
};

static struct tracepoints_table interests[] = {
/* @ CL 6502432*/
	{.name = "ufshcd_command", .func = probe_ufshcd_command, .en = TP_EN_LEGACY},
	{.name = "ufshcd_uic_command", .func = probe_ufshcd_uic_command},
	{.name = "ufshcd_clk_gating", .func = probe_ufshcd_clk_gating},
	{
		.name = "ufshcd_profile_clk_scaling",
		.func = probe_ufshcd_profile_clk_scaling
	},
	{
		.name = "android_vh_ufs_send_command",
		.func = probe_android_vh_ufs_send_command
	},
	{
		.name = "android_vh_ufs_compl_command",
		.func = probe_android_vh_ufs_compl_command
	},
	{.name = "android_vh_ufs_send_tm_command", .func = probe_android_vh_ufs_send_tm_command},
	{.name = "ufshcd_wl_runtime_suspend", .func = probe_ufshcd_runtime_suspend},
	{.name = "ufshcd_wl_runtime_resume", .func = probe_ufshcd_runtime_resume},
	{.name = "ufshcd_wl_suspend", .func = probe_ufshcd_system_suspend},
	{.name = "ufshcd_wl_resume", .func = probe_ufshcd_system_resume},
#ifdef OPLUS_DEVINFO_UFS
	{.name = "android_vh_ufs_update_sdev", .func = probe_android_vh_ufs_update_sdev},
#endif
};

#define FOR_EACH_INTEREST(i) \
	for (i = 0; i < sizeof(interests) / sizeof(struct tracepoints_table); \
	i++)

#define FOR_EACH_MOD_INTEREST(i) \
	for (i = 0; i < sizeof(mod_interests) / sizeof(struct mod_tracepoints_table); \
	i++)


static void for_each_tracepoint_range(tracepoint_ptr_t *begin, tracepoint_ptr_t *end,
		void (*fct)(struct tracepoint *tp, void *priv),
		void *priv)
{
	tracepoint_ptr_t *iter;

	if (!begin)
		return;
	for (iter = begin; iter < end; iter++)
		fct(tracepoint_ptr_deref(iter), priv);
}

/*
 * Find the struct tracepoint* associated with a given tracepoint
 * name.
 */
static void lookup_tracepoints(struct tracepoint *tp, void *ignore)
{
	int i;

	FOR_EACH_INTEREST(i) {
		if (strcmp(interests[i].name, tp->name) == 0)
			interests[i].tp = tp;
	}
}

/*
 * Find the struct tracepoint* associated with a given tracepoint
 * name in a module.
 */
void lookup_mod_tracepoints(struct tracepoint *tp, void *priv)
{
	struct module *mod = priv;
	int i = 0;

	FOR_EACH_MOD_INTEREST(i) {
		if (!strcmp(mod_interests[i].mod_name, mod->name) &&
			!strcmp(mod_interests[i].name, tp->name)) {

			pr_info("%s: tracepoint %s from module %s found",
				THIS_MODULE->name, tp->name, mod->name);
			mod_interests[i].tp = tp;
		}
	}
}

int ufs_mtk_dbg_cmd_hist_enable(void)
{
	unsigned long flags;

	spin_lock_irqsave(&cmd_hist_lock, flags);
	if (!cmd_hist) {
		cmd_hist_enabled = false;
		spin_unlock_irqrestore(&cmd_hist_lock, flags);
		return -ENOMEM;
	}

	cmd_hist_enabled = true;
	spin_unlock_irqrestore(&cmd_hist_lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(ufs_mtk_dbg_cmd_hist_enable);

int ufs_mtk_dbg_cmd_hist_disable(void)
{
	unsigned long flags;

	spin_lock_irqsave(&cmd_hist_lock, flags);
	cmd_hist_enabled = false;
	spin_unlock_irqrestore(&cmd_hist_lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(ufs_mtk_dbg_cmd_hist_disable);

static void _cmd_hist_cleanup(void)
{
	ufs_mtk_dbg_cmd_hist_disable();
	vfree(cmd_hist);
}

#define CLK_GATING_STATE_MAX (4)

static char *clk_gating_state_str[CLK_GATING_STATE_MAX + 1] = {
	"clks_off",
	"clks_on",
	"req_clks_off",
	"req_clks_on",
	"unknown"
};

static void ufs_mtk_dbg_print_clk_gating_event(char **buff,
					unsigned long *size,
					struct seq_file *m, int ptr)
{
	struct timespec64 dur;
	int idx = cmd_hist[ptr].cmd.clk_gating.state;

	if (idx < 0 || idx >= CLK_GATING_STATE_MAX)
		idx = CLK_GATING_STATE_MAX;

	dur = ns_to_timespec64(cmd_hist[ptr].time);
	SPREAD_PRINTF(buff, size, m,
		"%3d-c(%d),%6llu.%09lu,%5d,%2d,%13s,arg1=0x%X,arg2=0x%X,arg3=0x%X\n",
		ptr,
		cmd_hist[ptr].cpu,
		dur.tv_sec, dur.tv_nsec,
		cmd_hist[ptr].pid,
		cmd_hist[ptr].event,
		clk_gating_state_str[idx],
		cmd_hist[ptr].cmd.clk_gating.arg1,
		cmd_hist[ptr].cmd.clk_gating.arg2,
		cmd_hist[ptr].cmd.clk_gating.arg3
		);
}

#define CLK_SCALING_STATE_MAX (2)

static char *clk_scaling_state_str[CLK_SCALING_STATE_MAX + 1] = {
	"clk scale down",
	"clk scale up",
	"unknown"
};

static void ufs_mtk_dbg_print_clk_scaling_event(char **buff,
					unsigned long *size,
					struct seq_file *m, int ptr)
{
	struct timespec64 dur;
	int idx = cmd_hist[ptr].cmd.clk_scaling.state;

	if (idx < 0 || idx >= CLK_SCALING_STATE_MAX)
		idx = CLK_SCALING_STATE_MAX;

	dur = ns_to_timespec64(cmd_hist[ptr].time);
	SPREAD_PRINTF(buff, size, m,
		"%3d-c(%d),%6llu.%09lu,%5d,%2d,%15s, err:%d\n",
		ptr,
		cmd_hist[ptr].cpu,
		dur.tv_sec, dur.tv_nsec,
		cmd_hist[ptr].pid,
		cmd_hist[ptr].event,
		clk_scaling_state_str[idx],
		cmd_hist[ptr].cmd.clk_scaling.err
		);
}

#define UFSDBG_PM_STATE_MAX (4)
static char *ufsdbg_pm_state_str[UFSDBG_PM_STATE_MAX + 1] = {
	"rs",
	"rr",
	"ss",
	"sr",
	"unknown"
};

static void ufs_mtk_dbg_print_pm_event(char **buff,
					unsigned long *size,
					struct seq_file *m, int ptr)
{
	struct timespec64 dur;
	int idx = cmd_hist[ptr].cmd.pm.state;
	int err = cmd_hist[ptr].cmd.pm.err;
	unsigned long time_us = cmd_hist[ptr].cmd.pm.time_us;
	int pwr_mode = cmd_hist[ptr].cmd.pm.pwr_mode;
	int link_state = cmd_hist[ptr].cmd.pm.link_state;

	if (idx < 0 || idx >= UFSDBG_PM_STATE_MAX)
		idx = UFSDBG_PM_STATE_MAX;

	dur = ns_to_timespec64(cmd_hist[ptr].time);
	SPREAD_PRINTF(buff, size, m,
		"%3d-c(%d),%6llu.%09lu,%5d,%2d,%3s, ret=%d, time_us=%8lu, pwr_mode=%d, link_status=%d\n",
		ptr,
		cmd_hist[ptr].cpu,
		dur.tv_sec, dur.tv_nsec,
		cmd_hist[ptr].pid,
		cmd_hist[ptr].event,
		ufsdbg_pm_state_str[idx],
		err,
		time_us,
		pwr_mode,
		link_state
		);
}

static void ufs_mtk_dbg_print_device_reset_event(char **buff,
					unsigned long *size,
					struct seq_file *m, int ptr)
{
	struct timespec64 dur;
	int idx = cmd_hist[ptr].cmd.clk_gating.state;

	if (idx < 0 || idx >= CLK_GATING_STATE_MAX)
		idx = CLK_GATING_STATE_MAX;

	dur = ns_to_timespec64(cmd_hist[ptr].time);
	SPREAD_PRINTF(buff, size, m,
		"%3d-c(%d),%6llu.%09lu,%5d,%2d,%13s\n",
		ptr,
		cmd_hist[ptr].cpu,
		dur.tv_sec, dur.tv_nsec,
		cmd_hist[ptr].pid,
		cmd_hist[ptr].event,
		"device reset"
		);
}

static void ufs_mtk_dbg_print_uic_event(char **buff, unsigned long *size,
				   struct seq_file *m, int ptr)
{
	struct timespec64 dur;

	dur = ns_to_timespec64(cmd_hist[ptr].time);
	SPREAD_PRINTF(buff, size, m,
		"%3d-u(%d),%6llu.%09lu,%5d,%2d,0x%2x,arg1=0x%X,arg2=0x%X,arg3=0x%X,\t%llu\n",
		ptr,
		cmd_hist[ptr].cpu,
		dur.tv_sec, dur.tv_nsec,
		cmd_hist[ptr].pid,
		cmd_hist[ptr].event,
		cmd_hist[ptr].cmd.uic.cmd,
		cmd_hist[ptr].cmd.uic.arg1,
		cmd_hist[ptr].cmd.uic.arg2,
		cmd_hist[ptr].cmd.uic.arg3,
		cmd_hist[ptr].duration
		);
}

static void ufs_mtk_dbg_print_utp_event(char **buff, unsigned long *size,
				   struct seq_file *m, int ptr)
{
	struct timespec64 dur;

	dur = ns_to_timespec64(cmd_hist[ptr].time);
	if (cmd_hist[ptr].cmd.utp.lba == 0xFFFFFFFFFFFFFFFF)
		cmd_hist[ptr].cmd.utp.lba = 0;
	SPREAD_PRINTF(buff, size, m,
		"%3d-r(%d),%6llu.%09lu,%5d,%2d,0x%2x,t=%2d,db:0x%8x,is:0x%8x,crypt:%d,%d,lba=%10llu,len=%6d,\t%llu\n",
		ptr,
		cmd_hist[ptr].cpu,
		dur.tv_sec, dur.tv_nsec,
		cmd_hist[ptr].pid,
		cmd_hist[ptr].event,
		cmd_hist[ptr].cmd.utp.opcode,
		cmd_hist[ptr].cmd.utp.tag,
		cmd_hist[ptr].cmd.utp.doorbell,
		cmd_hist[ptr].cmd.utp.intr,
		cmd_hist[ptr].cmd.utp.crypt_en,
		cmd_hist[ptr].cmd.utp.crypt_keyslot,
		cmd_hist[ptr].cmd.utp.lba,
		cmd_hist[ptr].cmd.utp.transfer_len,
		cmd_hist[ptr].duration
		);
}

static void ufs_mtk_dbg_print_dev_event(char **buff, unsigned long *size,
				      struct seq_file *m, int ptr)
{
	struct timespec64 dur;

	dur = ns_to_timespec64(cmd_hist[ptr].time);

	SPREAD_PRINTF(buff, size, m,
		"%3d-r(%d),%6llu.%09lu,%5d,%2d,%4u,t=%2d,op:%u,idn:%u,idx:%u,sel:%u\n",
		ptr,
		cmd_hist[ptr].cpu,
		dur.tv_sec, dur.tv_nsec,
		cmd_hist[ptr].pid,
		cmd_hist[ptr].event,
		cmd_hist[ptr].cmd.dev.type,
		cmd_hist[ptr].cmd.dev.tag,
		cmd_hist[ptr].cmd.dev.opcode,
		cmd_hist[ptr].cmd.dev.idn,
		cmd_hist[ptr].cmd.dev.index,
		cmd_hist[ptr].cmd.dev.selector
		);
}

static void ufs_mtk_dbg_print_tm_event(char **buff, unsigned long *size,
				   struct seq_file *m, int ptr)
{
	struct timespec64 dur;

	dur = ns_to_timespec64(cmd_hist[ptr].time);
	if (cmd_hist[ptr].cmd.utp.lba == 0xFFFFFFFFFFFFFFFF)
		cmd_hist[ptr].cmd.utp.lba = 0;
	SPREAD_PRINTF(buff, size, m,
		"%3d-r(%d),%6llu.%09lu,%5d,%2d,0x%2x,lun=%d,tag=%d,task_tag=%d\n",
		ptr,
		cmd_hist[ptr].cpu,
		dur.tv_sec, dur.tv_nsec,
		cmd_hist[ptr].pid,
		cmd_hist[ptr].event,
		cmd_hist[ptr].cmd.tm.tm_func,
		cmd_hist[ptr].cmd.tm.lun,
		cmd_hist[ptr].cmd.tm.tag,
		cmd_hist[ptr].cmd.tm.task_tag
		);
}

static void ufs_mtk_dbg_print_cmd_hist(char **buff, unsigned long *size,
				  u32 latest_cnt, struct seq_file *m)
{
	int ptr;
	int cnt;
	unsigned long flags;

	if (!cmd_hist_initialized)
		return;

	spin_lock_irqsave(&cmd_hist_lock, flags);

	if (!cmd_hist) {
		spin_unlock_irqrestore(&cmd_hist_lock, flags);
		return;
	}

	cnt = min_t(u32, cmd_hist_cnt, MAX_CMD_HIST_ENTRY_CNT);
	if (latest_cnt)
		cnt = min_t(u32, latest_cnt, cnt);

	ptr = cmd_hist_ptr;

	SPREAD_PRINTF(buff, size, m,
		      "UFS CMD History: Latest %d of total %d entries, ptr=%d\n",
		      latest_cnt, cnt, ptr);

	while (cnt) {
		if (cmd_hist[ptr].event < CMD_DEV_SEND)
			ufs_mtk_dbg_print_utp_event(buff, size, m, ptr);
		else if (cmd_hist[ptr].event < CMD_TM_SEND)
			ufs_mtk_dbg_print_dev_event(buff, size, m, ptr);
		else if (cmd_hist[ptr].event < CMD_UIC_SEND)
			ufs_mtk_dbg_print_tm_event(buff, size, m, ptr);
		else if (cmd_hist[ptr].event < CMD_REG_TOGGLE)
			ufs_mtk_dbg_print_uic_event(buff, size, m, ptr);
		else if (cmd_hist[ptr].event == CMD_CLK_GATING)
			ufs_mtk_dbg_print_clk_gating_event(buff, size, m, ptr);
		else if (cmd_hist[ptr].event == CMD_CLK_SCALING)
			ufs_mtk_dbg_print_clk_scaling_event(buff, size, m, ptr);
		else if (cmd_hist[ptr].event == CMD_PM)
			ufs_mtk_dbg_print_pm_event(buff, size, m, ptr);
		else if (cmd_hist[ptr].event == CMD_ABORTING)
			ufs_mtk_dbg_print_utp_event(buff, size, m, ptr);
		else if (cmd_hist[ptr].event == CMD_DEVICE_RESET)
			ufs_mtk_dbg_print_device_reset_event(buff, size,
							     m, ptr);
		cnt--;
		ptr--;
		if (ptr < 0)
			ptr = MAX_CMD_HIST_ENTRY_CNT - 1;
	}

	spin_unlock_irqrestore(&cmd_hist_lock, flags);

}

void ufs_mtk_dbg_dump(u32 latest_cnt)
{
	ufs_mtk_dbg_print_info(NULL, NULL, NULL);

	ufs_mtk_dbg_print_cmd_hist(NULL, NULL, latest_cnt, NULL);
}
EXPORT_SYMBOL_GPL(ufs_mtk_dbg_dump);

void ufs_mtk_dbg_get_aee_buffer(unsigned long *vaddr, unsigned long *size)
{
	unsigned long free_size = UFS_AEE_BUFFER_SIZE;
	char *buff;

	if (!cmd_hist) {
		pr_info("failed to dump UFS: null cmd history buffer");
		return;
	}

	if (!ufs_aee_buffer) {
		pr_info("failed to dump UFS: null AEE buffer");
		return;
	}

	buff = ufs_aee_buffer;
	ufs_mtk_dbg_print_info(&buff, &free_size, NULL);
	ufs_mtk_dbg_print_cmd_hist(&buff, &free_size,
				   MAX_CMD_HIST_ENTRY_CNT, NULL);

	/* retrun start location */
	*vaddr = (unsigned long)ufs_aee_buffer;
	*size = UFS_AEE_BUFFER_SIZE - free_size;

	ufs_mtk_dbg_cmd_hist_enable();
}
EXPORT_SYMBOL_GPL(ufs_mtk_dbg_get_aee_buffer);

#ifndef USER_BUILD_KERNEL
#define PROC_PERM		0660
#else
#define PROC_PERM		0440
#endif

static ssize_t ufs_debug_proc_write(struct file *file, const char *buf,
				 size_t count, loff_t *data)
{
	unsigned long op = UFSDBG_UNKNOWN;
	struct ufs_hba *hba = ufshba;
	struct ufs_mtk_host *host = ufshcd_get_variant(hba);
	char cmd_buf[16];

	if (count == 0 || count > 15)
		return -EINVAL;

	if (copy_from_user(cmd_buf, buf, count))
		return -EINVAL;

	cmd_buf[count] = '\0';
	if (kstrtoul(cmd_buf, 15, &op))
		return -EINVAL;

	if (op == UFSDBG_CMD_LIST_DUMP) {
		dev_info(hba->dev, "debug info and cmd history dump\n");
		ufs_mtk_dbg_dump(MAX_CMD_HIST_ENTRY_CNT);
	} else if (op == UFSDBG_CMD_LIST_ENABLE) {
		ufs_mtk_dbg_cmd_hist_enable();
		dev_info(hba->dev, "cmd history on\n");
	} else if (op == UFSDBG_CMD_LIST_DISABLE) {
		ufs_mtk_dbg_cmd_hist_disable();
		dev_info(hba->dev, "cmd history off\n");
	} else if (op == UFSDBG_CMD_QOS_ON) {
		if (host && host->qos_allowed) {
			host->qos_enabled = true;
			dev_info(hba->dev, "QoS on\n");
		}
	} else if (op == UFSDBG_CMD_QOS_OFF) {
		if (host && host->qos_allowed) {
			host->qos_enabled = false;
			dev_info(hba->dev, "QoS off\n");
		}
	}

	return count;
}

static int ufs_debug_proc_show(struct seq_file *m, void *v)
{
	ufs_mtk_dbg_print_info(NULL, NULL, m);
	ufs_mtk_dbg_print_cmd_hist(NULL, NULL, MAX_CMD_HIST_ENTRY_CNT,
				   m);
	return 0;
}

static int ufs_debug_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, ufs_debug_proc_show, inode->i_private);
}

static const struct proc_ops ufs_debug_proc_fops = {
	.proc_open = ufs_debug_proc_open,
	.proc_write = ufs_debug_proc_write,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

static int ufs_mtk_dbg_init_procfs(void)
{
	struct proc_dir_entry *prEntry;
	kuid_t uid;
	kgid_t gid;

	uid = make_kuid(&init_user_ns, 0);
	gid = make_kgid(&init_user_ns, 1001);

	/* Create "ufs_debug" node */
	prEntry = proc_create("ufs_debug", PROC_PERM, NULL,
			      &ufs_debug_proc_fops);

	if (prEntry)
		proc_set_user(prEntry, uid, gid);
	else
		pr_info("%s: failed to create ufs_debugn", __func__);

	return 0;
}

static ssize_t downdifferential_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", hba->vps->ondemand_data.downdifferential);
}

static ssize_t downdifferential_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	u32 value;
	int err = 0;

	if (kstrtou32(buf, 0, &value))
		return -EINVAL;

	mutex_lock(&hba->devfreq->lock);
	if (value > 100 || value > hba->vps->ondemand_data.upthreshold) {
		err = -EINVAL;
		goto out;
	}
	hba->vps->ondemand_data.downdifferential = value;

out:
	mutex_unlock(&hba->devfreq->lock);
	return err ? err : count;
}

static ssize_t upthreshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", hba->vps->ondemand_data.upthreshold);
}

static ssize_t upthreshold_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	u32 value;
	int err = 0;

	if (kstrtou32(buf, 0, &value))
		return -EINVAL;

	mutex_lock(&hba->devfreq->lock);
	if (value > 100 || value < hba->vps->ondemand_data.downdifferential) {
		err = -EINVAL;
		goto out;
	}
	hba->vps->ondemand_data.upthreshold = value;

out:
	mutex_unlock(&hba->devfreq->lock);
	return err ? err : count;
}

static ssize_t clkscale_control_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufs_mtk_host *host = ufshcd_get_variant(hba);
	ssize_t size = 0;
	int value, powerhal;

	value = atomic_read((&host->clkscale_control));
	powerhal = atomic_read((&host->clkscale_control_powerhal));
	if (!value)
		value = powerhal;

	size += sprintf(buf + size, "current: %d\n", value);
	size += sprintf(buf + size, "powerhal_set: %d\n", powerhal);
	size += sprintf(buf + size, "===== control manual =====\n");
	size += sprintf(buf + size, "0: free run\n");
	size += sprintf(buf + size, "1: scale down\n");
	size += sprintf(buf + size, "2: scale up\n");

	return size;
}

static ssize_t clkscale_control_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufs_mtk_host *host = ufshcd_get_variant(hba);
	atomic_t *set_target = &host->clkscale_control;
	unsigned long flags;
	const char *opcode = buf;
	u32 value;

	if (!strncmp(buf, "powerhal_set: ", 14)) {
		set_target = &host->clkscale_control_powerhal;
		opcode = buf + 14;
	}

	if (kstrtou32(opcode, 0, &value) || value > 2)
		return -EINVAL;

	atomic_set(set_target, value);

	ufshcd_rpm_get_sync(hba);
	spin_lock_irqsave(hba->host->host_lock, flags);
	if (!hba->clk_scaling.window_start_t) {
		hba->clk_scaling.window_start_t = ktime_sub_us(ktime_get(), 1);
		hba->clk_scaling.tot_busy_t = 0;
		hba->clk_scaling.busy_start_t = 0;
		hba->clk_scaling.is_busy_started = false;

	}
	spin_unlock_irqrestore(hba->host->host_lock, flags);
	update_devfreq(hba->devfreq);
	ufshcd_rpm_put(hba);

	return count;
}

static DEVICE_ATTR_RW(downdifferential);
static DEVICE_ATTR_RW(upthreshold);
static DEVICE_ATTR_RW(clkscale_control);

static struct attribute	*ufs_mtk_sysfs_clkscale_attrs[] = {
	&dev_attr_downdifferential.attr,
	&dev_attr_upthreshold.attr,
	&dev_attr_clkscale_control.attr,
	NULL
};

struct attribute_group ufs_mtk_sysfs_clkscale_group = {
	.name = "clkscale",
	.attrs = ufs_mtk_sysfs_clkscale_attrs,
};

void ufs_mtk_init_clk_scaling_sysfs(struct ufs_hba *hba)
{
	if (sysfs_create_group(&hba->dev->kobj, &ufs_mtk_sysfs_clkscale_group))
		dev_info(hba->dev, "Failed to create sysfs for clkscale_control\n");
}
EXPORT_SYMBOL_GPL(ufs_mtk_init_clk_scaling_sysfs);

void ufs_mtk_remove_clk_scaling_sysfs(struct ufs_hba *hba)
{
	sysfs_remove_group(&hba->dev->kobj, &ufs_mtk_sysfs_clkscale_group);
}
EXPORT_SYMBOL_GPL(ufs_mtk_remove_clk_scaling_sysfs);

static bool check_tp_enable(struct ufs_hba_private *hba_priv, enum tp_en en)
{
	if (en == TP_EN_ALL)
		return true;
	else if ((en == TP_EN_MCQ) && hba_priv->is_mcq_enabled)
		return true;
	else if ((en == TP_EN_LEGACY) && !hba_priv->is_mcq_enabled)
		return true;

	return false;
}


static void ufs_mtk_dbg_cleanup(struct ufs_hba *hba)
{
	int i;
	struct ufs_hba_private *hba_priv = (struct ufs_hba_private *)hba->android_vendor_data1;

	FOR_EACH_INTEREST(i) {
		if (interests[i].init) {
			if (!check_tp_enable(hba_priv, interests[i].en))
				continue;
			tracepoint_probe_unregister(interests[i].tp,
						    interests[i].func,
						    NULL);
		}
	}

	FOR_EACH_MOD_INTEREST(i) {
		if (mod_interests[i].init) {
			if (!check_tp_enable(hba_priv, mod_interests[i].en))
				continue;
			tracepoint_probe_unregister(mod_interests[i].tp,
						    mod_interests[i].func,
						    NULL);
		}
	}

	_cmd_hist_cleanup();
}

static int ufs_mtk_tp_module_notifier_handler(struct notifier_block *nb,
			unsigned long action, void *data)
{
	struct tp_module *tpmd = data;
	struct module *mod = tpmd->mod;
	int i = 0;

	/* search if current loaded module is in mod_interests */
	FOR_EACH_MOD_INTEREST(i) {
		if (!strcmp(tpmd->mod->name, mod_interests[i].mod_name)) {
			for_each_tracepoint_range(mod->tracepoints_ptrs,
				mod->tracepoints_ptrs + mod->num_tracepoints,
				lookup_mod_tracepoints, mod);
			break;
		}
	}

	return 0;
}

struct notifier_block tp_nb = {
	.notifier_call = ufs_mtk_tp_module_notifier_handler,
};

int ufs_mtk_dbg_register(struct ufs_hba *hba)
{
	int i, ret;
	struct ufs_hba_private *hba_priv = (struct ufs_hba_private *)hba->android_vendor_data1;

	/*
	 * Ignore any failure of AEE buffer allocation to still allow
	 * command history dump in procfs.
	 */
	ufs_aee_buffer = kzalloc(UFS_AEE_BUFFER_SIZE, GFP_NOFS);

	spin_lock_init(&cmd_hist_lock);
	ufshba = hba;
	cmd_hist_initialized = true;

	/* Install the tracepoints */
	for_each_kernel_tracepoint(lookup_tracepoints, NULL);

	FOR_EACH_INTEREST(i) {
		if (interests[i].tp == NULL) {
			pr_info("Error: %s not found\n",
				interests[i].name);
			/* Unload previously loaded */
			ufs_mtk_dbg_cleanup(hba);
			return -EINVAL;
		}
		if (!check_tp_enable(hba_priv, interests[i].en))
			continue;

		tracepoint_probe_register(interests[i].tp,
					  interests[i].func,
					  NULL);
		interests[i].init = true;
	}

/* @ CL 6502432*/
	FOR_EACH_MOD_INTEREST(i) {
		if (mod_interests[i].tp == NULL) {
			pr_info("Error: %s not found in modules. Check tracepoint or module load order.\n",
				mod_interests[i].name);
			ufs_mtk_dbg_cleanup(hba);
			return -EINVAL;
		}

		if (!check_tp_enable(hba_priv, mod_interests[i].en))
			continue;
		tracepoint_probe_register(mod_interests[i].tp,
					  mod_interests[i].func,
					  NULL);
		mod_interests[i].init = true;
	}

	/* Create control nodes in procfs */
	ret = ufs_mtk_dbg_init_procfs();

	/* Enable command history feature by default */
	if (!ret)
		ufs_mtk_dbg_cmd_hist_enable();
	else
		ufs_mtk_dbg_cleanup(hba);

	return ret;
}
EXPORT_SYMBOL_GPL(ufs_mtk_dbg_register);

static void __exit ufs_mtk_dbg_exit(void)
{
#if IS_ENABLED(CONFIG_MTK_AEE_IPANIC)
	mrdump_set_extra_dump(AEE_EXTRA_FILE_UFS, NULL);
#endif
	kfree(cmd_hist);
}

static int __init ufs_mtk_dbg_init(void)
{
	cmd_hist = kcalloc(MAX_CMD_HIST_ENTRY_CNT,
			   sizeof(struct cmd_hist_struct),
			   GFP_KERNEL);
#if IS_ENABLED(CONFIG_MTK_AEE_IPANIC)
	mrdump_set_extra_dump(AEE_EXTRA_FILE_UFS, ufs_mtk_dbg_get_aee_buffer);
#endif

/* @ CL 6502432*/
	register_tracepoint_module_notifier(&tp_nb);
	return 0;
}

module_init(ufs_mtk_dbg_init)
module_exit(ufs_mtk_dbg_exit)

MODULE_DESCRIPTION("MediaTek UFS Debugging Facility");
MODULE_AUTHOR("Stanley Chu <stanley.chu@mediatek.com>");
MODULE_LICENSE("GPL v2");
