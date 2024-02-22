// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022 MediaTek Inc.
 * Author: Dennis-YC Hsieh <dennis-yc.hsieh@mediatek.com>
 */

#include <dt-bindings/mml/mml-mt6985.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>

#include "mtk-mml-drm-adaptor.h"
#include "mtk-mml-color.h"
#include "mtk-mml-core.h"

#define TOPOLOGY_PLATFORM	"mt6985"
#define MML_DUAL_FRAME		(2500 * 1400)
#define AAL_MIN_WIDTH		50	/* TODO: define in tile? */
/* 2k size and pixel as upper bound */
#define MML_IR_WIDTH_2K		2560
#define MML_IR_HEIGHT_2K	1440
#define MML_IR_2K		(MML_IR_WIDTH_2K * MML_IR_HEIGHT_2K)
/* hd size and pixel as lower bound */
#define MML_IR_WIDTH		640
#define MML_IR_HEIGHT		480
#define MML_IR_MIN		(MML_IR_WIDTH * MML_IR_HEIGHT)
#define MML_IR_RSZ_MIN_RATIO	375	/* resize must lower than this ratio */
#define MML_IR_OUT_MIN_W	784	/* wqhd 1440/2+64=784 */

/* use OPP index 0(229Mhz) 1(273Mhz) 2(458Mhz) */
#define MML_IR_MAX_OPP		2

int mml_force_rsz;
module_param(mml_force_rsz, int, 0644);

enum topology_dual {
	MML_DUAL_NORMAL,
	MML_DUAL_DISABLE,
	MML_DUAL_ALWAYS,
};

int mml_dual;
module_param(mml_dual, int, 0644);

int mml_path_swap;
module_param(mml_path_swap, int, 0644);

int mml_path_mode;
module_param(mml_path_mode, int, 0644);

/* debug param
 * 0: (default)don't care, check dts property to enable racing
 * 1: force enable
 * 2: force disable
 */
int mml_racing;
module_param(mml_racing, int, 0644);

int mml_racing_rsz = 1;
module_param(mml_racing_rsz, int, 0644);

enum topology_scenario {
	PATH_MML_NOPQ_P0 = 0,
	PATH_MML_NOPQ_P1,
	PATH_MML_NOPQ_DD0,
	PATH_MML_NOPQ_DD1,
	PATH_MML_PQ_P0,
	PATH_MML_PQ_P1,
	PATH_MML_PQ_DD0,
	PATH_MML_PQ_DD1,
	PATH_MML_PQ_P2,
	PATH_MML_PQ_P3,
	PATH_MML_2OUT_P0,
	PATH_MML_2OUT_P1,
	PATH_MML_2IN_2OUT_P0,
	PATH_MML_2IN_2OUT_P1,
	PATH_MML_2IN_1OUT_P0,
	PATH_MML_2IN_1OUT_P1,
	PATH_MML_APUPQ_DD0,
	PATH_MML_APUPQ_DD1,
	PATH_MML_MAX
};

struct path_node {
	u8 eng;
	u8 next0;
	u8 next1;
};

/* check if engine is output dma engine */
static inline bool engine_wrot(u32 id)
{
	return id == MML_WROT0 || id == MML_WROT1 ||
		id == MML_WROT2 || id == MML_WROT3;
}

/* check if engine is input region pq rdma engine */
static inline bool engine_pq_rdma(u32 id)
{
	return id == MML_RDMA2 || id == MML_RDMA3;
}

/* check if engine is input region pq birsz engine */
static inline bool engine_pq_birsz(u32 id)
{
	return id == MML_BIRSZ0 || id == MML_BIRSZ1;
}

/* check if engine is region pq engine */
static inline bool engine_region_pq(u32 id)
{
	return id == MML_RDMA2 || id == MML_RDMA3 ||
		id == MML_BIRSZ0 || id == MML_BIRSZ1;
}

/* check if engine is tdshp engine */
static inline bool engine_tdshp(u32 id)
{
	return id == MML_TDSHP0 || id == MML_TDSHP1;
}

static const struct path_node path_map[PATH_MML_MAX][MML_MAX_PATH_NODES] = {
	[PATH_MML_NOPQ_P0] = {
		{MML_MMLSYS,},
		{MML_MUTEX,},
		{MML_RDMA0, MML_WROT0,},
		{MML_WROT0,},
	},
	[PATH_MML_NOPQ_P1] = {
		{MML_MMLSYS,},
		{MML_MUTEX,},
		{MML_RDMA1, MML_WROT1,},
		{MML_WROT1,},
	},
	[PATH_MML_NOPQ_DD0] = {
		{MML_MMLSYS,},
		{MML_MUTEX,},
		{MML_DLI0, MML_DLI0_SEL,},
		{MML_DLI0_SEL, MML_DLO0_SOUT,},
		{MML_DLO0_SOUT, MML_DLO0,},
		{MML_DLO0,},
	},
	[PATH_MML_NOPQ_DD1] = {
		{MML_MMLSYS,},
		{MML_MUTEX,},
		{MML_DLI1, MML_DLI1_SEL,},
		{MML_DLI1_SEL, MML_DLO1_SOUT,},
		{MML_DLO1_SOUT, MML_DLO1,},
		{MML_DLO1,},
	},
	[PATH_MML_PQ_P0] = {
		{MML_MMLSYS,},
		{MML_MUTEX,},
		{MML_RDMA0, MML_DLI0_SEL,},
		{MML_DLI0_SEL, MML_HDR0,},
		{MML_HDR0, MML_AAL0,},
		{MML_AAL0, MML_RSZ0,},
		{MML_RSZ0, MML_TDSHP0,},
		{MML_TDSHP0, MML_COLOR0,},
		{MML_COLOR0, MML_PQ0_SOUT,},
		{MML_PQ0_SOUT, MML_DLO0_SOUT,},
		{MML_DLO0_SOUT, MML_WROT0,},
		{MML_WROT0,},
	},
	[PATH_MML_PQ_P1] = {
		{MML_MMLSYS,},
		{MML_MUTEX,},
		{MML_RDMA1, MML_DLI1_SEL,},
		{MML_DLI1_SEL, MML_HDR1,},
		{MML_HDR1, MML_AAL1,},
		{MML_AAL1, MML_RSZ1,},
		{MML_RSZ1, MML_TDSHP1,},
		{MML_TDSHP1, MML_COLOR1,},
		{MML_COLOR1, MML_PQ1_SOUT,},
		{MML_PQ1_SOUT, MML_DLO1_SOUT,},
		{MML_DLO1_SOUT, MML_WROT1,},
		{MML_WROT1,},
	},
	[PATH_MML_PQ_DD0] = {
		{MML_MMLSYS,},
		{MML_MUTEX,},
		{MML_DLI0, MML_DLI0_SEL,},
		{MML_DLI0_SEL, MML_HDR0,},
		{MML_HDR0, MML_AAL0,},
		{MML_AAL0, MML_RSZ0,},
		{MML_RSZ0, MML_TDSHP0,},
		{MML_TDSHP0, MML_COLOR0,},
		{MML_COLOR0, MML_PQ0_SOUT,},
		{MML_PQ0_SOUT, MML_DLO0_SOUT,},
		{MML_DLO0_SOUT, MML_DLO0,},
		{MML_DLO0,},
	},
	[PATH_MML_PQ_DD1] = {
		{MML_MMLSYS,},
		{MML_MUTEX,},
		{MML_DLI1, MML_DLI1_SEL,},
		{MML_DLI1_SEL, MML_HDR1,},
		{MML_HDR1, MML_AAL1,},
		{MML_AAL1, MML_RSZ1,},
		{MML_RSZ1, MML_TDSHP1,},
		{MML_TDSHP1, MML_COLOR1,},
		{MML_COLOR1, MML_PQ1_SOUT,},
		{MML_PQ1_SOUT, MML_DLO1_SOUT,},
		{MML_DLO1_SOUT, MML_DLO1,},
		{MML_DLO1,},
	},
	[PATH_MML_PQ_P2] = {
		{MML_MMLSYS,},
		{MML_MUTEX,},
		{MML_RDMA0, MML_DLI0_SEL,},
		{MML_DLI0_SEL, MML_RSZ2,},
		{MML_RSZ2, MML_WROT2,},
		{MML_WROT2,},
	},
	[PATH_MML_PQ_P3] = {
		{MML_MMLSYS,},
		{MML_MUTEX,},
		{MML_RDMA1, MML_DLI1_SEL,},
		{MML_DLI1_SEL, MML_RSZ3,},
		{MML_RSZ3, MML_WROT3,},
		{MML_WROT3,},
	},
	[PATH_MML_2OUT_P0] = {
		{MML_MMLSYS,},
		{MML_MUTEX,},
		{MML_RDMA0, MML_DLI0_SEL,},
		{MML_DLI0_SEL, MML_HDR0, MML_RSZ2},
		{MML_HDR0, MML_AAL0,},
		{MML_AAL0, MML_RSZ0,},
		{MML_RSZ0, MML_TDSHP0,},
		{MML_TDSHP0, MML_COLOR0,},
		{MML_COLOR0, MML_PQ0_SOUT,},
		{MML_PQ0_SOUT, MML_DLO0_SOUT,},
		{MML_DLO0_SOUT, MML_WROT0,},
		{MML_RSZ2, MML_WROT2,},
		{MML_WROT0,},
		{MML_WROT2,},
	},
	[PATH_MML_2OUT_P1] = {
		{MML_MMLSYS,},
		{MML_MUTEX,},
		{MML_RDMA1, MML_DLI1_SEL,},
		{MML_DLI1_SEL, MML_HDR1, MML_RSZ3},
		{MML_HDR1, MML_AAL1,},
		{MML_AAL1, MML_RSZ1,},
		{MML_RSZ1, MML_TDSHP1,},
		{MML_TDSHP1, MML_COLOR1,},
		{MML_COLOR1, MML_PQ1_SOUT,},
		{MML_PQ1_SOUT, MML_DLO1_SOUT,},
		{MML_DLO1_SOUT, MML_WROT1,},
		{MML_RSZ3, MML_WROT3,},
		{MML_WROT1,},
		{MML_WROT3,},
	},
	[PATH_MML_2IN_2OUT_P0] = {
		{MML_MMLSYS,},
		{MML_MUTEX,},
		{MML_RDMA2,},
		{MML_BIRSZ0,},
		{MML_RDMA0, MML_DLI0_SEL,},
		{MML_DLI0_SEL, MML_HDR0, MML_RSZ2},
		{MML_HDR0, MML_AAL0,},
		{MML_AAL0, MML_RSZ0,},
		{MML_RSZ0, MML_TDSHP0,},
		{MML_TDSHP0, MML_COLOR0,},
		{MML_COLOR0, MML_PQ0_SOUT,},
		{MML_PQ0_SOUT, MML_DLO0_SOUT,},
		{MML_DLO0_SOUT, MML_WROT0,},
		{MML_RSZ2, MML_WROT2,},
		{MML_WROT0,},
		{MML_WROT2,},
	},
	[PATH_MML_2IN_2OUT_P1] = {
		{MML_MMLSYS,},
		{MML_MUTEX,},
		{MML_RDMA3,},
		{MML_BIRSZ1,},
		{MML_RDMA1, MML_DLI1_SEL,},
		{MML_DLI1_SEL, MML_HDR1, MML_RSZ3},
		{MML_HDR1, MML_AAL1,},
		{MML_AAL1, MML_RSZ1,},
		{MML_RSZ1, MML_TDSHP1,},
		{MML_TDSHP1, MML_COLOR1,},
		{MML_COLOR1, MML_PQ1_SOUT,},
		{MML_PQ1_SOUT, MML_DLO1_SOUT,},
		{MML_DLO1_SOUT, MML_WROT1,},
		{MML_RSZ3, MML_WROT3,},
		{MML_WROT1,},
		{MML_WROT3,},
	},
	[PATH_MML_2IN_1OUT_P0] = {
		{MML_MMLSYS,},
		{MML_MUTEX,},
		{MML_RDMA2,},
		{MML_BIRSZ0,},
		{MML_RDMA0, MML_DLI0_SEL,},
		{MML_DLI0_SEL, MML_HDR0,},
		{MML_HDR0, MML_AAL0,},
		{MML_AAL0, MML_RSZ0,},
		{MML_RSZ0, MML_TDSHP0,},
		{MML_TDSHP0, MML_COLOR0,},
		{MML_COLOR0, MML_PQ0_SOUT,},
		{MML_PQ0_SOUT, MML_DLO0_SOUT,},
		{MML_DLO0_SOUT, MML_WROT0,},
		{MML_WROT0,},
	},
	[PATH_MML_2IN_1OUT_P1] = {
		{MML_MMLSYS,},
		{MML_MUTEX,},
		{MML_RDMA3,},
		{MML_BIRSZ1,},
		{MML_RDMA1, MML_DLI1_SEL,},
		{MML_DLI1_SEL, MML_HDR1,},
		{MML_HDR1, MML_AAL1,},
		{MML_AAL1, MML_RSZ1,},
		{MML_RSZ1, MML_TDSHP1,},
		{MML_TDSHP1, MML_COLOR1,},
		{MML_COLOR1, MML_PQ1_SOUT,},
		{MML_PQ1_SOUT, MML_DLO1_SOUT,},
		{MML_DLO1_SOUT, MML_WROT1,},
		{MML_WROT1,},
	},
	[PATH_MML_APUPQ_DD0] = {
		{MML_MMLSYS,},
		{MML_MUTEX,},
		{MML_RDMA0, MML_DLI0_SEL,},
		{MML_DLI0_SEL, MML_HDR0,},
		{MML_HDR0, MML_AAL0,},
		{MML_AAL0, MML_RSZ0,},
		{MML_RSZ0, MML_TDSHP0,},
		{MML_TDSHP0, MML_COLOR0,},
		{MML_COLOR0, MML_PQ0_SOUT,},
		{MML_PQ0_SOUT, MML_DLO0_SOUT,},
		{MML_DLO0_SOUT, MML_DLO0,},
		{MML_DLO0,},
	},
	[PATH_MML_APUPQ_DD1] = {
		{MML_MMLSYS,},
		{MML_MUTEX,},
		{MML_RDMA1, MML_DLI1_SEL,},
		{MML_DLI1_SEL, MML_HDR1,},
		{MML_HDR1, MML_AAL1,},
		{MML_AAL1, MML_RSZ1,},
		{MML_RSZ1, MML_TDSHP1,},
		{MML_TDSHP1, MML_COLOR1,},
		{MML_COLOR1, MML_PQ1_SOUT,},
		{MML_PQ1_SOUT, MML_DLO1_SOUT,},
		{MML_DLO1_SOUT, MML_DLO1,},
		{MML_DLO1,},
	},
};

enum cmdq_clt_usage {
	MML_CLT_PIPE0,
	MML_CLT_PIPE1,
	MML_CLT_MAX
};

static const u8 clt_dispatch[PATH_MML_MAX] = {
	[PATH_MML_NOPQ_P0] = MML_CLT_PIPE0,
	[PATH_MML_NOPQ_P1] = MML_CLT_PIPE1,
	[PATH_MML_NOPQ_DD0] = MML_CLT_PIPE0,
	[PATH_MML_NOPQ_DD1] = MML_CLT_PIPE1,
	[PATH_MML_PQ_P0] = MML_CLT_PIPE0,
	[PATH_MML_PQ_P1] = MML_CLT_PIPE1,
	[PATH_MML_PQ_DD0] = MML_CLT_PIPE0,
	[PATH_MML_PQ_DD1] = MML_CLT_PIPE1,
	[PATH_MML_PQ_P2] = MML_CLT_PIPE0,
	[PATH_MML_PQ_P3] = MML_CLT_PIPE1,
	[PATH_MML_2OUT_P0] = MML_CLT_PIPE0,
	[PATH_MML_2OUT_P1] = MML_CLT_PIPE1,
	[PATH_MML_2IN_2OUT_P0] = MML_CLT_PIPE0,
	[PATH_MML_2IN_2OUT_P1] = MML_CLT_PIPE1,
	[PATH_MML_2IN_1OUT_P0] = MML_CLT_PIPE0,
	[PATH_MML_2IN_1OUT_P1] = MML_CLT_PIPE1,
	[PATH_MML_APUPQ_DD0] = MML_CLT_PIPE0,
	[PATH_MML_APUPQ_DD1] = MML_CLT_PIPE1,
};

/* mux sof group of mmlsys mout/sel */
enum mux_sof_group {
	MUX_SOF_GRP0 = 0,
	MUX_SOF_GRP1,
	MUX_SOF_GRP2,
	MUX_SOF_GRP3,
	MUX_SOF_GRP4,
	MUX_SOF_GRP5,
	MUX_SOF_GRP6,
	MUX_SOF_GRP7,
};

static const u8 grp_dispatch[PATH_MML_MAX] = {
	[PATH_MML_NOPQ_P0] = MUX_SOF_GRP1,
	[PATH_MML_NOPQ_P1] = MUX_SOF_GRP2,
	[PATH_MML_NOPQ_DD0] = MUX_SOF_GRP3,
	[PATH_MML_NOPQ_DD1] = MUX_SOF_GRP4,
	[PATH_MML_PQ_P0] = MUX_SOF_GRP1,
	[PATH_MML_PQ_P1] = MUX_SOF_GRP2,
	[PATH_MML_PQ_DD0] = MUX_SOF_GRP3,
	[PATH_MML_PQ_DD1] = MUX_SOF_GRP4,
	[PATH_MML_PQ_P2] = MUX_SOF_GRP1,
	[PATH_MML_PQ_P3] = MUX_SOF_GRP2,
	[PATH_MML_2OUT_P0] = MUX_SOF_GRP1,
	[PATH_MML_2OUT_P1] = MUX_SOF_GRP2,
	[PATH_MML_2IN_2OUT_P0] = MUX_SOF_GRP1,
	[PATH_MML_2IN_2OUT_P1] = MUX_SOF_GRP2,
	[PATH_MML_2IN_1OUT_P0] = MUX_SOF_GRP1,
	[PATH_MML_2IN_1OUT_P1] = MUX_SOF_GRP2,
	[PATH_MML_APUPQ_DD0] = MUX_SOF_GRP1,
	[PATH_MML_APUPQ_DD1] = MUX_SOF_GRP2,
};

/* reset bit to each engine,
 * reverse of MMSYS_SW0_RST_B_REG and MMSYS_SW1_RST_B_REG
 */
static u8 engine_reset_bit[MML_ENGINE_TOTAL] = {
	[MML_RDMA0] = 3,
	[MML_RDMA2] = 4,
	[MML_HDR0] = 5,
	[MML_AAL0] = 6,
	[MML_RSZ0] = 7,
	[MML_TDSHP0] = 8,
	[MML_COLOR0] = 9,
	[MML_WROT0] = 10,
	[MML_DLI0] = 12,
	[MML_DLI1] = 13,
	[MML_RDMA1] = 15,
	[MML_RDMA3] = 16,
	[MML_HDR1] = 17,
	[MML_AAL1] = 18,
	[MML_RSZ1] = 19,
	[MML_TDSHP1] = 20,
	[MML_COLOR1] = 21,
	[MML_WROT1] = 22,
	[MML_RSZ2] = 24,
	[MML_WROT2] = 25,
	[MML_DLO0] = 26,
	[MML_RSZ3] = 28,
	[MML_WROT3] = 29,
	[MML_DLO1] = 30,
	[MML_BIRSZ0] = 35,
	[MML_BIRSZ1] = 36,
};

/* 6.6 ms as dc mode active time threshold by:
 * 1 / (fps * vblank) = 1000000 / 120 / 1.25 = 6666us
 */
#define MML_DC_ACT_DUR	6600
static u32 opp_pixel_table[MML_MAX_OPPS];

static void tp_dump_path(const struct mml_topology_path *path)
{
	u8 i;

	for (i = 0; i < path->node_cnt; i++) {
		mml_log(
			"[topology]engine %u (%p) prev %p %p next %p %p comp %p tile idx %u out %u",
			path->nodes[i].id, &path->nodes[i],
			path->nodes[i].prev[0], path->nodes[i].prev[1],
			path->nodes[i].next[0], path->nodes[i].next[1],
			path->nodes[i].comp,
			path->nodes[i].tile_eng_idx,
			path->nodes[i].out_idx);
	}
}

static void tp_dump_path_short(struct mml_topology_path *path)
{
	char path_desc[64];
	u32 len = 0;
	u8 i;

	for (i = 0; i < path->node_cnt; i++)
		len += snprintf(path_desc + len, sizeof(path_desc) - len, " %u",
			path->nodes[i].id);
	mml_log("[topology]engines:%s", path_desc);
}

static void tp_parse_path(struct mml_dev *mml, struct mml_topology_path *path,
	const struct path_node *route)
{
	struct mml_path_node *prev[2] = {0};
	struct mml_path_node *pq_rdma = NULL;
	struct mml_path_node *pq_birsz = NULL;
	u8 connect_eng[2] = {0};
	u8 i, tile_idx, out_eng_idx;

	for (i = 0; i < MML_MAX_PATH_NODES; i++) {
		const u8 eng = route[i].eng;
		const u8 next0 = route[i].next0;
		const u8 next1 = route[i].next1;

		if (!eng) {
			path->node_cnt = i;
			break;
		}

		/* assign current engine */
		path->nodes[i].id = eng;
		path->nodes[i].comp = mml_dev_get_comp_by_id(mml, eng);
		if (!path->nodes[i].comp)
			mml_err("[topology]no comp idx:%u engine:%u", i, eng);

		/* assign reset bits for this path */
		path->reset_bits |= 1LL << engine_reset_bit[eng];

		if (eng == MML_MMLSYS) {
			path->mmlsys = path->nodes[i].comp;
			path->mmlsys_idx = i;
			continue;
		} else if (eng == MML_MUTEX) {
			path->mutex = path->nodes[i].comp;
			path->mutex_idx = i;
			continue;
		} else if (engine_pq_rdma(eng)) {
			pq_rdma = &path->nodes[i];
			continue;
		} else if (engine_pq_birsz(eng)) {
			pq_birsz = &path->nodes[i];
			continue;
		} else if (engine_tdshp(eng)) {
			if (pq_rdma && pq_birsz) {
				pq_birsz->prev[0] = pq_rdma;
				pq_rdma->next[0] = pq_birsz;
				pq_birsz->next[0] = &path->nodes[i];
				path->nodes[i].prev[1] = pq_birsz;
			}
		}

		if (eng == MML_RDMA0)
			path->hw_pipe = 0;
		else if (eng == MML_RDMA1)
			path->hw_pipe = 1;

		/* check cursor for 2 out and link if id match */
		if (!connect_eng[0] && next0) {
			/* First engine case, set current out 0 to this engine.
			 * And config next engines
			 */
			prev[0] = &path->nodes[i];
			connect_eng[0] = next0;
			path->nodes[i].out_idx = 0;
		} else if (connect_eng[0] == eng) {
			/* connect out 0 */
			prev[0]->next[0] = &path->nodes[i];
			/* replace current out 0 to this engine */
			path->nodes[i].prev[0] = prev[0];
			prev[0] = &path->nodes[i];
			connect_eng[0] = next0;
			path->nodes[i].out_idx = 0;

			/* also assign 1 in 2 out case,
			 * must branch from line 0
			 */
			if (!connect_eng[1] && next1) {
				prev[1] = &path->nodes[i];
				connect_eng[1] = next1;
			}
		} else if (connect_eng[1] == eng) {
			/* connect out 1 */
			if (!prev[1]->next[0])
				prev[1]->next[0] = &path->nodes[i];
			else
				prev[1]->next[1] = &path->nodes[i];
			/* replace current out 1 to this engine */
			path->nodes[i].prev[0] = prev[1];
			prev[1] = &path->nodes[i];
			connect_eng[1] = next0;
			path->nodes[i].out_idx = 1;

			/* at most 2 out in one path,
			 * cannot branch from line 1
			 */
			if (next1)
				mml_err("[topology]%s wrong path index %u engine %u",
					__func__, i, eng);
		} else {
			mml_err("[topology]connect fail idx:%u engine:%u next0:%u next1:%u from:%u %u",
				i, eng, next0, next1,
				connect_eng[0], connect_eng[1]);
		}
	}
	path->node_cnt = i;

	/* 0: reset
	 * 1: not reset
	 * so we need to reverse the bits
	 */
	path->reset_bits = ~path->reset_bits;
	mml_msg("[topology]reset bits %#llx", path->reset_bits);

	/* collect tile engines */
	tile_idx = 0;
	for (i = 0; i < path->node_cnt; i++) {
		if ((!path->nodes[i].prev[0] && !path->nodes[i].next[0]) ||
		    engine_region_pq(path->nodes[i].id)) {
			path->nodes[i].tile_eng_idx = ~0;
			continue;
		}
		path->nodes[i].tile_eng_idx = tile_idx;
		path->tile_engines[tile_idx++] = i;
	}
	path->tile_engine_cnt = tile_idx;

	/* scan region pq in engines */
	for (i = 0; i < path->node_cnt; i++) {
		if (engine_pq_rdma(path->nodes[i].id)) {
			path->nodes[i].tile_eng_idx = path->tile_engine_cnt;
			if (path->tile_engine_cnt < MML_MAX_PATH_NODES)
				path->tile_engines[path->tile_engine_cnt] = i;
			else
				mml_err("[topology]RDMA tile_engines idx %d >= MML_MAX_PATH_NODES",
					path->tile_engine_cnt);
			if (path->pq_rdma_id)
				mml_err("[topology]multiple pq rdma engines: was %hhu now %hhu",
					path->pq_rdma_id,
					path->nodes[i].id);
			path->pq_rdma_id = path->nodes[i].id;
		} else if (engine_pq_birsz(path->nodes[i].id)) {
			path->nodes[i].tile_eng_idx = path->tile_engine_cnt + 1;
			if (path->tile_engine_cnt + 1 < MML_MAX_PATH_NODES)
				path->tile_engines[path->tile_engine_cnt + 1] = i;
			else
				mml_err("[topology]BIRSZ tile_engines idx %d >= MML_MAX_PATH_NODES",
					path->tile_engine_cnt + 1);
		}
	}

	/* scan out engines */
	for (i = 0; i < path->node_cnt; i++) {
		if (!engine_wrot(path->nodes[i].id))
			continue;
		out_eng_idx = path->nodes[i].out_idx;
		if (path->out_engine_ids[out_eng_idx])
			mml_err("[topology]multiple out engines: was %u now %u on out idx:%u",
				path->out_engine_ids[out_eng_idx],
				path->nodes[i].id, out_eng_idx);
		path->out_engine_ids[out_eng_idx] = path->nodes[i].id;
	}

	if (path->tile_engine_cnt == 2)
		path->alpharot = true;
}

static s32 tp_init_cache(struct mml_dev *mml, struct mml_topology_cache *cache,
	struct cmdq_client **clts, u32 clt_cnt)
{
	u32 i;

	if (clt_cnt < MML_CLT_MAX) {
		mml_err("[topology]%s not enough cmdq clients to all paths",
			__func__);
		return -ECHILD;
	}
	if (ARRAY_SIZE(cache->paths) < PATH_MML_MAX) {
		mml_err("[topology]%s not enough path cache for all paths",
			__func__);
		return -ECHILD;
	}

	for (i = 0; i < PATH_MML_MAX; i++) {
		struct mml_topology_path *path = &cache->paths[i];

		tp_parse_path(mml, path, path_map[i]);
		if (mtk_mml_msg) {
			mml_log("[topology]dump path %u count %u clt id %u",
				i, path->node_cnt, clt_dispatch[i]);
			tp_dump_path(path);
		}

		/* now dispatch cmdq client (channel) to path */
		path->clt = clts[clt_dispatch[i]];
		path->clt_id = clt_dispatch[i];
		path->mux_group = grp_dispatch[i];
	}

	return 0;
}

static inline bool tp_need_resize(struct mml_frame_info *info)
{
	u32 w = info->dest[0].data.width;
	u32 h = info->dest[0].data.height;

	if (info->dest[0].rotate == MML_ROT_90 ||
		info->dest[0].rotate == MML_ROT_270)
		swap(w, h);

	mml_msg("[topology]%s target %ux%u crop %ux%u",
		__func__, w, h,
		info->dest[0].crop.r.width,
		info->dest[0].crop.r.height);

	return info->dest_cnt != 1 ||
		info->dest[0].crop.r.width != w ||
		info->dest[0].crop.r.height != h ||
		info->dest[0].crop.x_sub_px ||
		info->dest[0].crop.y_sub_px ||
		info->dest[0].crop.w_sub_px ||
		info->dest[0].crop.h_sub_px ||
		info->dest[0].compose.width != info->dest[0].data.width ||
		info->dest[0].compose.height != info->dest[0].data.height;
}

static void tp_select_path(struct mml_topology_cache *cache,
	struct mml_frame_config *cfg,
	struct mml_topology_path **path)
{
	enum topology_scenario scene[2] = {0};
	bool en_rsz, en_pq;

	if (cfg->info.mode == MML_MODE_RACING) {
		/* always rdma to wrot for racing case */
		scene[0] = PATH_MML_NOPQ_P0;
		scene[1] = PATH_MML_NOPQ_P1;
		goto done;
	} else if (cfg->info.mode == MML_MODE_APUDC) {
		scene[0] = PATH_MML_APUPQ_DD0;
		scene[1] = PATH_MML_APUPQ_DD1;
		goto done;
	}

	en_rsz = tp_need_resize(&cfg->info);
	if (mml_force_rsz)
		en_rsz = true;
	en_pq = en_rsz || cfg->info.dest[0].pq_config.en;

	if (cfg->info.mode == MML_MODE_DDP_ADDON) {
		/* direct-link in/out for addon case */
		if (cfg->info.dest_cnt == 2) {
			/* TODO: ddp addon 2out */
			scene[0] = PATH_MML_PQ_DD0; /* PATH_MML_2OUT_DD0 */
			scene[1] = PATH_MML_PQ_DD1; /* PATH_MML_2OUT_DD1 */
		} else {
			scene[0] = PATH_MML_PQ_DD0;
			scene[1] = PATH_MML_PQ_DD1;
		}
	} else if (!en_pq) {
		/* dual pipe, rdma to wrot */
		scene[0] = PATH_MML_NOPQ_P0;
		scene[1] = PATH_MML_NOPQ_P1;
	} else if (cfg->info.dest_cnt == 2) {
		if (cfg->info.dest[0].pq_config.en_region_pq) {
			scene[0] = PATH_MML_2IN_2OUT_P0;
			scene[1] = PATH_MML_2IN_2OUT_P1;
		} else {
			scene[0] = PATH_MML_2OUT_P0;
			scene[1] = PATH_MML_2OUT_P1;
		}
	} else if (mml_force_rsz == 2) {
		scene[0] = PATH_MML_PQ_P2;
		scene[1] = PATH_MML_PQ_P3;
	} else {
		if (cfg->info.dest[0].pq_config.en_region_pq) {
			scene[0] = PATH_MML_2IN_1OUT_P0;
			scene[1] = PATH_MML_2IN_1OUT_P1;
		} else {
			/* 1 in 1 out with PQs */
			scene[0] = PATH_MML_PQ_P0;
			scene[1] = PATH_MML_PQ_P1;
		}
	}

done:
	path[0] = &cache->paths[scene[0]];
	path[1] = &cache->paths[scene[1]];

	if (mml_path_swap)
		swap(path[0], path[1]);
}

static inline bool tp_need_dual(struct mml_frame_config *cfg)
{
	const struct mml_frame_data *src = &cfg->info.src;
	const struct mml_frame_data *dest = &cfg->info.dest[0].data;
	u32 min_crop_w, i;

	if (cfg->info.mode == MML_MODE_RACING)
		return true;

	if (src->width * src->height < MML_DUAL_FRAME &&
	    dest->width * dest->height < MML_DUAL_FRAME)
		return false;

	min_crop_w = cfg->info.dest[0].crop.r.width;
	for (i = 1; i < cfg->info.dest_cnt; i++)
		min_crop_w = min(min_crop_w, cfg->info.dest[i].crop.r.width);

	if (min_crop_w <= AAL_MIN_WIDTH)
		return false;

	return true;
}

static s32 tp_select(struct mml_topology_cache *cache,
	struct mml_frame_config *cfg)
{
	struct mml_topology_path *path[2] = {0};

	if (mml_dual == MML_DUAL_DISABLE)
		cfg->dual = false;
	else if (mml_dual == MML_DUAL_ALWAYS)
		cfg->dual = true;
	else /* (mml_dual == MML_DUAL_NORMAL) */
		cfg->dual = tp_need_dual(cfg);

	if (cfg->info.mode == MML_MODE_DDP_ADDON) {
		cfg->dual = cfg->disp_dual;
		cfg->framemode = true;
		cfg->nocmd = true;
	} else if (cfg->info.mode == MML_MODE_APUDC) {
		cfg->dual = true;
	} else if (cfg->info.mode == MML_MODE_SRAM_READ) {
		cfg->dual = false;
	}
	cfg->shadow = true;

	tp_select_path(cache, cfg, path);

	if (!path[0])
		return -EPERM;

	cfg->path[0] = path[0];
	cfg->path[1] = path[1];

	if (path[0]->alpharot) {
		u32 i;

		cfg->alpharot = MML_FMT_IS_ARGB(cfg->info.src.format);
		for (i = 0; i < cfg->info.dest_cnt && cfg->alpharot; i++)
			if (!MML_FMT_IS_ARGB(cfg->info.dest[i].data.format))
				cfg->alpharot = false;
	}

	tp_dump_path_short(path[0]);
	if (cfg->dual && path[1])
		tp_dump_path_short(path[1]);

	return 0;
}

static enum mml_mode tp_query_mode(struct mml_dev *mml, struct mml_frame_info *info,
	u32 *reason)
{
	struct mml_topology_cache *tp;
	u32 pixel;

	if (unlikely(mml_path_mode))
		return mml_path_mode;

	if (unlikely(mml_racing)) {
		if (mml_racing == 2)
			goto decouple;
	} else if (!mml_racing_enable(mml))
		goto decouple;

	/* skip all racing mode check if use prefer dc */
	if (info->mode == MML_MODE_MML_DECOUPLE ||
		info->mode == MML_MODE_MDP_DECOUPLE) {
		*reason = mml_query_userdc;
		goto decouple_user;
	}

	if (info->mode == MML_MODE_APUDC &&
		info->src.width == 1920 && info->src.height == 1080) {
		*reason = mml_query_apudc;
		goto decouple_user;
	}

	/* TODO: should REMOVE after inlinerot resize ready */
	if (unlikely(!mml_racing_rsz) && tp_need_resize(info)) {
		*reason = mml_query_norsz;
		goto decouple;
	}

	/* secure content cannot output to sram */
	if (info->src.secure || info->dest[0].data.secure) {
		*reason = mml_query_sec;
		goto decouple;
	}

	/* no pq support for racing mode */
	if (info->dest[0].pq_config.en_dc ||
		info->dest[0].pq_config.en_color ||
		info->dest[0].pq_config.en_hdr ||
		info->dest[0].pq_config.en_ccorr ||
		info->dest[0].pq_config.en_dre ||
		info->dest[0].pq_config.en_region_pq) {
		*reason = mml_query_pqen;
		goto decouple;
	}

	/* racing only support 1 out */
	if (info->dest_cnt > 1) {
		*reason = mml_query_2out;
		goto decouple;
	}

	if (MML_FMT_IS_RGB(info->dest[0].data.format))
		goto decouple;

	if (!MML_FMT_COMPRESS(info->src.format)) {
		*reason = mml_query_format;
		goto decouple;
	}
	/* get mid opp frequency */
	tp = mml_topology_get_cache(mml);
	if (!tp || !tp->opp_cnt) {
		mml_err("not support racing due to opp not ready");
		goto decouple;
	}

	pixel = max(info->src.width * info->src.height,
		info->dest[0].data.width * info->dest[0].data.height);

	if (info->act_time) {
		u32 i, dc_opp, ir_freq, ir_opp;
		u32 pipe_pixel = pixel / 2;

		if (!tp->opp_cnt) {
			mml_err("no opp table support");
			goto decouple;
		}

		if (!opp_pixel_table[0]) {
			for (i = 0; i < ARRAY_SIZE(opp_pixel_table); i++) {
				opp_pixel_table[i] = tp->opp_speeds[i] * MML_DC_ACT_DUR;
				mml_log("[topology]Racing pixel OPP %u: %u",
					i, opp_pixel_table[i]);
			}
		}

		for (i = 0; i < tp->opp_cnt; i++)
			if (pipe_pixel < opp_pixel_table[i])
				break;
		dc_opp = min_t(u32, i, ARRAY_SIZE(opp_pixel_table) - 1);
		if (dc_opp > MML_IR_MAX_OPP) {
			*reason = mml_query_opp_out;
			goto decouple;
		}

		ir_freq = pipe_pixel * 1000 / info->act_time;
		for (i = 0; i < tp->opp_cnt; i++)
			if (ir_freq < tp->opp_speeds[i])
				break;
		ir_opp = min_t(u32, i, ARRAY_SIZE(opp_pixel_table) - 1);

		/* simple check if ir mode need higher opp */
		if (ir_opp > dc_opp) {
			*reason = mml_query_acttime;
			goto decouple;
		}
	}

	/* only support FHD/2K with rotate 90/270 case for now */
	if (info->dest[0].rotate == MML_ROT_0 || info->dest[0].rotate == MML_ROT_180) {
		*reason = mml_query_rot;
		goto decouple;
	}
	if (info->dest[0].crop.r.width > MML_IR_WIDTH_2K ||
		info->dest[0].crop.r.height > MML_IR_HEIGHT_2K ||
		pixel > MML_IR_2K) {
		*reason = mml_query_highpixel;
		goto decouple;
	}
	if (info->dest[0].crop.r.width < MML_IR_WIDTH ||
		info->dest[0].crop.r.height < MML_IR_HEIGHT ||
		pixel < MML_IR_MIN) {
		*reason = mml_query_lowpixel;
		goto decouple;
	}

	/* destination width must cross display pipe width */
	if (info->dest[0].data.width < MML_IR_OUT_MIN_W) {
		*reason = mml_query_outwidth;
		goto decouple;
	}

	if (info->dest[0].data.width * info->dest[0].data.height * 1000 /
		info->dest[0].crop.r.width / info->dest[0].crop.r.height <
		MML_IR_RSZ_MIN_RATIO) {
		*reason = mml_query_rszratio;
		goto decouple;
	}

	if (!MML_FMT_COMPRESS(info->src.format)) {
		*reason = mml_query_format;
		goto decouple;
	}

	return MML_MODE_RACING;

decouple:
	return MML_MODE_MML_DECOUPLE;
decouple_user:
	return info->mode;
}

static struct cmdq_client *get_racing_clt(struct mml_topology_cache *cache, u32 pipe)
{
	/* use NO PQ path as inline rot path for this platform */
	return cache->paths[PATH_MML_NOPQ_P0 + pipe].clt;
}

static const struct mml_topology_ops tp_ops_mt6985 = {
	.query_mode = tp_query_mode,
	.init_cache = tp_init_cache,
	.select = tp_select,
	.get_racing_clt = get_racing_clt,
};

static __init int mml_topology_ip_init(void)
{
	return mml_topology_register_ip(TOPOLOGY_PLATFORM, &tp_ops_mt6985);
}
module_init(mml_topology_ip_init);

static __exit void mml_topology_ip_exit(void)
{
	mml_topology_unregister_ip(TOPOLOGY_PLATFORM);
}
module_exit(mml_topology_ip_exit);

MODULE_AUTHOR("Dennis-YC Hsieh <dennis-yc.hsieh@mediatek.com>");
MODULE_DESCRIPTION("MediaTek SoC display MML for MT6985");
MODULE_LICENSE("GPL v2");
