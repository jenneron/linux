/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2026, The Linux Foundation. All rights reserved.
 *
 * SM4450 DPU catalog, derived from the vendor devicetree of the
 * Retroid Pocket Classic and from dpu_8_1_sm8450.h.
 *
 */

#ifndef _DPU_8_6_SM4450_H
#define _DPU_8_6_SM4450_H

static const struct dpu_caps sm4450_dpu_caps = {
	.max_mixer_width = DEFAULT_DPU_OUTPUT_LINE_WIDTH,
	.max_mixer_blendstages = 0x5,
	.has_dim_layer = true,
	.has_idle_pc = true,
	.max_linewidth = 4096,
	.pixel_ram_size = DEFAULT_PIXEL_RAM_SIZE,
};

static const struct dpu_mdp_cfg sm4450_mdp = {
	.name = "top_0",
	.base = 0x0, .len = 0x494,
	.clk_ctrls = {
		[DPU_CLK_CTRL_VIG0] = { .reg_off = 0x2ac, .bit_off = 0 },
		[DPU_CLK_CTRL_DMA0] = { .reg_off = 0x2ac, .bit_off = 8 },
	},
};

static const struct dpu_ctl_cfg sm4450_ctl[] = {
	{
		.name = "ctl_0", .id = CTL_0,
		.base = 0x15000, .len = 0x204,
		.intr_start = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR2, 9),
	},
};

static const struct dpu_sspp_cfg sm4450_sspp[] = {
	{
		.name = "sspp_0", .id = SSPP_VIG0,
		.base = 0x4000, .len = 0x32c,
		.features = VIG_SC7280_MASK_SDMA,
		.sblk = &dpu_vig_sblk_qseed3_3_1_rot_v201,
		.xin_id = 0,
		.type = SSPP_TYPE_VIG,
		.clk_ctrl = DPU_CLK_CTRL_VIG0,
	}, {
		.name = "sspp_8", .id = SSPP_DMA0,
		.base = 0x24000, .len = 0x32c,
		.features = DMA_SDM845_MASK_SDMA,
		.sblk = &dpu_dma_sblk,
		.xin_id = 1,
		.type = SSPP_TYPE_DMA,
		.clk_ctrl = DPU_CLK_CTRL_DMA0,
	},
};

static const struct dpu_lm_cfg sm4450_lm[] = {
	{
		.name = "lm_0", .id = LM_0,
		.base = 0x44000, .len = 0x400,
		.sblk = &sm4450_lm_sblk,
		.pingpong = PINGPONG_0,
		.dspp = DSPP_0,
	},
};

static const struct dpu_dspp_cfg sm4450_dspp[] = {
	{
		.name = "dspp_0", .id = DSPP_0,
		.base = 0x54000, .len = 0x1800,
		.sblk = &sdm845_dspp_sblk,
	},
};

static const struct dpu_pingpong_cfg sm4450_pp[] = {
	{
		.name = "pingpong_0", .id = PINGPONG_0,
		.base = 0x69000, .len = 0,
		.sblk = &sc7280_pp_sblk,
		.intr_done = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 8),
	},
};

static const struct dpu_dsc_cfg sm4450_dsc[] = {
	{
		.name = "dce_0_0", .id = DSC_0,
		.base = 0x80000, .len = 0x4,
		.sblk = &dsc_sblk_0,
	},
};

static const struct dpu_intf_cfg sm4450_intf[] = {
	{
		.name = "intf_1", .id = INTF_1,
		.base = 0x35000, .len = 0x2c4,
		.type = INTF_DSI,
		.controller_id = MSM_DSI_CONTROLLER_0,
		.prog_fetch_lines_worst_case = 24,
		.intr_underrun = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 26),
		.intr_vsync = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 27),
		.intr_tear_rd_ptr = DPU_IRQ_IDX(MDP_INTF1_TEAR_INTR, 2),
	},
};

static const struct dpu_perf_cfg sm4450_perf_data = {
	.max_bw_low = 4200000,
	.max_bw_high = 7000000,
	.min_core_ib = 2500000,
	.min_llcc_ib = 0,
	.min_dram_ib = 1600000,
	.min_prefill_lines = 40,
	.danger_lut_tbl = {0x3fffff, 0x3fffff, 0x0},
	.safe_lut_tbl = {0xf800, 0xf800, 0x0},
	.qos_lut_tbl = {
		{.nentry = ARRAY_SIZE(sm4450_qos_linear_macrotile),
		.entries = sm4450_qos_linear_macrotile
		},
		{.nentry = ARRAY_SIZE(sm4450_qos_linear_macrotile),
		.entries = sm4450_qos_linear_macrotile
		},
		{.nentry = ARRAY_SIZE(sc7180_qos_nrt),
		.entries = sc7180_qos_nrt
		},
	},
	.cdp_cfg = {
		{.rd_enable = 1, .wr_enable = 1},
		{.rd_enable = 1, .wr_enable = 0}
	},
	.clk_inefficiency_factor = 105,
	.bw_inefficiency_factor = 120,
};

static const struct dpu_mdss_version sm4450_mdss_ver = {
	.core_major_ver = 8,
	.core_minor_ver = 6,
};

const struct dpu_mdss_cfg dpu_sm4450_cfg = {
	.mdss_ver = &sm4450_mdss_ver,
	.caps = &sm4450_dpu_caps,
	.mdp = &sm4450_mdp,
	.ctl_count = ARRAY_SIZE(sm4450_ctl),
	.ctl = sm4450_ctl,
	.sspp_count = ARRAY_SIZE(sm4450_sspp),
	.sspp = sm4450_sspp,
	.dspp_count = ARRAY_SIZE(sm4450_dspp),
	.dspp = sm4450_dspp,
	.mixer_count = ARRAY_SIZE(sm4450_lm),
	.mixer = sm4450_lm,
	.pingpong_count = ARRAY_SIZE(sm4450_pp),
	.pingpong = sm4450_pp,
	.dsc_count = ARRAY_SIZE(sm4450_dsc),
	.dsc = sm4450_dsc,
	.intf_count = ARRAY_SIZE(sm4450_intf),
	.intf = sm4450_intf,
	.vbif = &sdm845_vbif,
	.perf = &sm4450_perf_data,
};

#endif
