// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-fence.h>
#include <linux/interconnect.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/kref.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/pm_opp.h>
#include <linux/pm_qos.h>
#include <linux/pm_runtime.h>
#include <linux/workqueue.h>
#include <linux/soc/qcom/ubwc.h>

#include <drm/drm_mode.h>
#include <drm/drm_gem.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_vblank.h>

#include "msm_gem.h"
#include "dpu_kms.h"
#include "dpu_formats.h"
#include "dpu_hw_rotator.h"

#define ROTTOP_OFF		0x800
#define ROTTOP_HW_VERSION	(ROTTOP_OFF + 0x00)
#define ROTTOP_SW_RESET_OVERRIDE (ROTTOP_OFF + 0x28)
#define ROTTOP_INTR_EN		(ROTTOP_OFF + 0x30)
#define ROTTOP_INTR_STATUS	(ROTTOP_OFF + 0x34)
#define ROTTOP_INTR_CLEAR	(ROTTOP_OFF + 0x38)
#define ROTTOP_START_CTRL	(ROTTOP_OFF + 0x40)
#define ROTTOP_STATUS		(ROTTOP_OFF + 0x44)
#define ROTTOP_OP_MODE		(ROTTOP_OFF + 0x48)
#define ROTTOP_OP_MODE_START	BIT(0)
#define ROTTOP_OP_MODE_ROT90	BIT(1)
#define ROTTOP_DNSC		(ROTTOP_OFF + 0x4C)
#define ROTTOP_UBWC_DEC_VERSION	(ROTTOP_OFF + 0x58)
#define ROTTOP_OP_MODE_SBUF_MASK (BIT(5) | BIT(4) | BIT(1) | BIT(0))
#define ROT_DONE_MASK		0x1
#define ROT_BUSY_BIT		BIT(0)
#define ROT_ERROR_BIT		BIT(8)
#define ROT_HW_VERSION_1_0	0x10000000

#define SSPP_OFF		0x900
#define ROT_SSPP_SRC_SIZE	(SSPP_OFF + 0x00)
#define ROT_SSPP_SRC_IMG_SIZE	(SSPP_OFF + 0x04)
#define ROT_SSPP_SRC_XY		(SSPP_OFF + 0x08)
#define ROT_SSPP_OUT_SIZE	(SSPP_OFF + 0x0C)
#define ROT_SSPP_OUT_XY		(SSPP_OFF + 0x10)
#define ROT_SSPP_SRC0_ADDR	(SSPP_OFF + 0x14)
#define ROT_SSPP_SRC1_ADDR	(SSPP_OFF + 0x18)
#define ROT_SSPP_SRC2_ADDR	(SSPP_OFF + 0x1C)
#define ROT_SSPP_SRC3_ADDR	(SSPP_OFF + 0x20)
#define ROT_SSPP_SRC_YSTRIDE0	(SSPP_OFF + 0x24)
#define ROT_SSPP_SRC_YSTRIDE1	(SSPP_OFF + 0x28)
#define ROT_SSPP_SRC_FORMAT	(SSPP_OFF + 0x30)
#define ROT_SSPP_SRC_UNPACK_PATTERN (SSPP_OFF + 0x34)
#define ROT_SSPP_SRC_OP_MODE	(SSPP_OFF + 0x38)
#define ROT_SSPP_UBWC_STATIC_CTRL (SSPP_OFF + 0x44)
#define ROT_SSPP_FETCH_CONFIG	(SSPP_OFF + 0x48)
#define ROT_SSPP_DANGER_LUT	(SSPP_OFF + 0x60)
#define ROT_SSPP_SAFE_LUT	(SSPP_OFF + 0x64)
#define ROT_SSPP_SRC_ADDR_SW_STATUS (SSPP_OFF + 0x70)
#define ROT_SSPP_CREQ_LUT_0	(SSPP_OFF + 0x74)
#define ROT_SSPP_CREQ_LUT_1	(SSPP_OFF + 0x78)
#define ROT_SSPP_UBWC_ERROR_STATUS (SSPP_OFF + 0x138)
#define ROT_SSPP_CDP_CNTL	(SSPP_OFF + 0x134)

#define WB_OFF			0xB00
#define ROT_WB_DST_FORMAT	(WB_OFF + 0x000)
#define ROT_WB_DST_OP_MODE	(WB_OFF + 0x004)
#define ROT_WB_DST_PACK_PATTERN	(WB_OFF + 0x008)
#define ROT_WB_DST0_ADDR	(WB_OFF + 0x00C)
#define ROT_WB_DST1_ADDR	(WB_OFF + 0x010)
#define ROT_WB_DST2_ADDR	(WB_OFF + 0x014)
#define ROT_WB_DST3_ADDR	(WB_OFF + 0x018)
#define ROT_WB_DST_YSTRIDE0	(WB_OFF + 0x01C)
#define ROT_WB_DST_YSTRIDE1	(WB_OFF + 0x020)
#define ROT_WB_DST_WRITE_CONFIG	(WB_OFF + 0x048)
#define ROT_WB_OUT_SIZE		(WB_OFF + 0x074)
#define ROT_WB_DANGER_LUT	(WB_OFF + 0x084)
#define ROT_WB_SAFE_LUT		(WB_OFF + 0x088)
#define ROT_WB_CREQ_LUT_0	(WB_OFF + 0x098)
#define ROT_WB_CREQ_LUT_1	(WB_OFF + 0x09C)
#define ROT_WB_DST_ADDR_SW_STATUS (WB_OFF + 0x2B0)
#define ROT_WB_CDP_CNTL		(WB_OFF + 0x2B4)
#define ROT_WB_UBWC_ERROR_STATUS (WB_OFF + 0x2BC)
#define ROT_WB_OUT_IMG_SIZE	(WB_OFF + 0x2C0)
#define ROT_WB_OUT_XY		(WB_OFF + 0x2C4)

#define SRC_FORMAT_ALPHA_EN	BIT(8)
#define SRC_FORMAT_ROT90	BIT(11)
#define SRC_FORMAT_DX		BIT(14)
#define SRC_FORMAT_UNPACK_TIGHT	BIT(17)
#define SRC_FORMAT_UNPACK_MSB	BIT(18)
#define DST_FORMAT_PACK_DX	BIT(21)
#define SRC_OP_BWC_DEC_EN	BIT(0)
#define SRC_OP_FLIP_LR		BIT(13)
#define SRC_OP_FLIP_UD		BIT(14)
#define SRC_OP_PE_OVERRIDE	BIT(31)
#define WB_OP_BWC_ENC_EN	BIT(0)

#define WB_NO_PARTIAL_WRITES	BIT(10)
#define PARTIAL_WRITE_ALIGN	0x1f

#define CDP_CNTL_ENABLE		BIT(0)
#define CDP_CNTL_UBWC_META	BIT(1)
#define CDP_CNTL_TILE_AMORTIZE	BIT(2)
#define CDP_CNTL_PRELOAD_64	BIT(3)

#define FETCH_CFG		0x00087
#define UBWC_CTRL_V3_PLUS	BIT(30)


#define SID_ROT_RD		0x20
#define SID_ROT_WR		0x24
#define ROT_SID_ID		0x1c

#define VBIF_XIN_SSPP		0
#define VBIF_XIN_WB		1
#define VBIF_WRITE_GATHER_EN	0x0AC
#define VBIF_OUT_AXI_AMEMTYPE_CONF0 0x160
#define VBIF_XIN_HALT_CTRL0	0x200
#define VBIF_XIN_HALT_CTRL1	0x204
#define VBIF_QOS_RP_REMAP_000	0x550
#define VBIF_QOS_LVL_REMAP_000	0x590
#define VBIF_QOS_LEVELS		8
#define VBIF_QOS_PRIORITY	3
#define VBIF_MEMTYPE		3

#define ROT_ICC_IDLE_MS		100

#define ROT_OPP_IDLE_MS		1

/* the first 128-line strip plus the scanout's prefetch, with margin */
#define ROT_RACE_MIN_US		800

#define ROT_QOS_LATENCY_US	100

static const struct of_device_id dpu_hw_rotator_of_match[] = {
	{ .compatible = "qcom,sm6115-sde-rotator" },
	{ .compatible = "qcom,sm8250-sde-rotator" },
	{ }
};

static inline void rwr(struct dpu_hw_rotator *r, u32 off, u32 v)
{
	writel_relaxed(v, r->base + off);
}

static inline u32 rrd(struct dpu_hw_rotator *r, u32 off)
{
	return readl_relaxed(r->base + off);
}

static void rot_sid_setup(struct dpu_hw_rotator *rot)
{
	if (!rot->sid)
		return;
	writel_relaxed(ROT_SID_ID, rot->sid + SID_ROT_RD);
	writel_relaxed(ROT_SID_ID, rot->sid + SID_ROT_WR);
}


static void rot_vbif_setup(struct dpu_hw_rotator *rot)
{
	u32 qos = (VBIF_QOS_PRIORITY << (VBIF_XIN_SSPP * 4)) |
		  (VBIF_QOS_PRIORITY << (VBIF_XIN_WB * 4));
	u32 i;

	if (!rot->vbif)
		return;

	for (i = 0; i < VBIF_QOS_LEVELS; i++) {
		writel_relaxed(qos, rot->vbif + VBIF_QOS_RP_REMAP_000 + i * 8);
		writel_relaxed(qos, rot->vbif + VBIF_QOS_LVL_REMAP_000 + i * 8);
	}
	writel_relaxed((VBIF_MEMTYPE << (VBIF_XIN_SSPP * 4)) |
		       (VBIF_MEMTYPE << (VBIF_XIN_WB * 4)),
		       rot->vbif + VBIF_OUT_AXI_AMEMTYPE_CONF0);

	writel_relaxed(BIT(VBIF_XIN_WB), rot->vbif + VBIF_WRITE_GATHER_EN);
}

static void rot_vbif_halt_xin(struct dpu_hw_rotator *rot, u32 xin)
{
	u32 v;

	if (!rot->vbif)
		return;

	v = readl_relaxed(rot->vbif + VBIF_XIN_HALT_CTRL0);
	writel_relaxed(v | BIT(xin), rot->vbif + VBIF_XIN_HALT_CTRL0);
	readl_relaxed_poll_timeout(rot->vbif + VBIF_XIN_HALT_CTRL1, v,
				   v & BIT(xin), 100, 5000);
	v = readl_relaxed(rot->vbif + VBIF_XIN_HALT_CTRL0);
	writel_relaxed(v & ~BIT(xin), rot->vbif + VBIF_XIN_HALT_CTRL0);
}

static bool rot_irq_ok(struct dpu_hw_rotator *rot)
{
	return rot->irq > 0 && !rot->irq_dead;
}

static irqreturn_t rot_isr(int irq, void *data)
{
	struct dpu_hw_rotator *rot = data;

	if (!rot->job_pending)
		return IRQ_NONE;
	if (!(rrd(rot, ROTTOP_INTR_STATUS) & ROT_DONE_MASK))
		return IRQ_NONE;

	rwr(rot, ROTTOP_INTR_EN, 0);
	rwr(rot, ROTTOP_INTR_CLEAR, ROT_DONE_MASK);
	rot->job_pending = false;
	complete(&rot->job_done);
	return IRQ_HANDLED;
}


static void rot_reset(struct dpu_hw_rotator *rot)
{
	u32 op_mode;

	rwr(rot, ROTTOP_SW_RESET_OVERRIDE, 1);
	wmb();
	usleep_range(10000, 20000);
	op_mode = rrd(rot, ROTTOP_OP_MODE);
	rwr(rot, ROTTOP_OP_MODE, op_mode & ~ROTTOP_OP_MODE_SBUF_MASK);
	rwr(rot, ROTTOP_SW_RESET_OVERRIDE, 0);
	rot_vbif_halt_xin(rot, VBIF_XIN_SSPP);
	rot_vbif_halt_xin(rot, VBIF_XIN_WB);
	rwr(rot, ROTTOP_INTR_EN, 0);
	rwr(rot, ROTTOP_INTR_CLEAR, ROT_DONE_MASK);
}

bool dpu_hw_rotator_accepts(struct dpu_hw_rotator *rot,
			    const struct msm_format *fmt)
{
	if (IS_ERR_OR_NULL(rot) || fmt->bpp != 4 || MSM_FORMAT_IS_YUV(fmt))
		return false;
	if (MSM_FORMAT_IS_LINEAR(fmt))
		return true;
	return MSM_FORMAT_IS_UBWC(fmt) && rot->has_ubwc;
}

int dpu_hw_rotator_dst_layout(const struct msm_format *fmt, u32 width,
			      u32 height, u32 iova,
			      struct dpu_hw_fmt_layout *layout)
{
	int ret;

	if (MSM_FORMAT_IS_UBWC(fmt)) {
		ret = dpu_format_get_plane_sizes_ubwc(fmt, width, height,
						      layout);
		if (ret)
			return ret;
		layout->plane_addr[0] = iova + layout->plane_size[2];
		layout->plane_addr[2] = iova;
		return 0;
	}

	memset(layout, 0, sizeof(*layout));
	layout->num_planes = 1;
	layout->width = width;
	layout->height = height;
	layout->plane_pitch[0] = width * fmt->bpp;
	layout->plane_size[0] = layout->plane_pitch[0] * height;
	layout->plane_addr[0] = iova;
	layout->total_size = layout->plane_size[0];
	return 0;
}

static u32 rot_ubwc_static_ctrl(const struct qcom_ubwc_cfg_data *cfg, u32 hbb)
{
	if (cfg->ubwc_enc_version >= UBWC_4_0)
		return UBWC_CTRL_V3_PLUS;
	if (cfg->ubwc_enc_version >= UBWC_3_0)
		return UBWC_CTRL_V3_PLUS | qcom_ubwc_swizzle(cfg) | (hbb << 4);
	return (qcom_ubwc_swizzle(cfg) & 1) | (hbb << 4);
}


static unsigned int dpu_rot_clk_mhz;
module_param(dpu_rot_clk_mhz, uint, 0644);
MODULE_PARM_DESC(dpu_rot_clk_mhz,
		 "rotator clock in MHz for a job that can't race the scanout (0: fastest OPP, or the DPU core clock without an OPP table)");

static bool dpu_rot_race = true;
module_param(dpu_rot_race, bool, 0644);
MODULE_PARM_DESC(dpu_rot_race,
		 "scan out a rotated frame once the rotator starts on it; it writes top down faster than the scanout, even at the lowest OPP");

static bool rot_has_opp(struct dpu_hw_rotator *rot)
{
	unsigned long rate;
	struct dev_pm_opp *opp;

	if (rot->opp_max || !rot->dev || !device_is_bound(rot->dev) ||
	    dev_pm_opp_get_opp_count(rot->dev) <= 0)
		return rot->opp_max;

	rate = 0;
	opp = dev_pm_opp_find_freq_ceil(rot->dev, &rate);
	if (IS_ERR(opp))
		return false;
	dev_pm_opp_put(opp);
	rot->opp_min = rate;

	rate = ULONG_MAX;
	opp = dev_pm_opp_find_freq_floor(rot->dev, &rate);
	if (IS_ERR(opp))
		return false;
	dev_pm_opp_put(opp);
	rot->opp_max = rate;
	return true;
}

/* with an OPP table the rail is raised before the clock and lowered after */
static void rot_clk_set_rate(struct dpu_hw_rotator *rot, bool job)
{
	unsigned long rate = dpu_rot_clk_mhz * 1000000UL;

	if (rot_has_opp(rot)) {
		if (!job)
			rate = rot->opp_min;
		else if (!rate)
			rate = rot->opp_max;
		dev_pm_opp_set_rate(rot->dev, rate);
		return;
	}

	if (!rate && rot->core_clk)
		rate = clk_get_rate(rot->core_clk);
	if (rate)
		clk_set_rate(rot->rot_clk, clk_round_rate(rot->rot_clk, rate));
}

static void rot_opp_idle(struct work_struct *work)
{
	struct dpu_hw_rotator *rot = container_of(to_delayed_work(work),
						  struct dpu_hw_rotator, opp_idle);

	rot_clk_set_rate(rot, false);
}

static void rot_bw_idle(struct work_struct *work)
{
	struct dpu_hw_rotator *rot = container_of(to_delayed_work(work),
						  struct dpu_hw_rotator, bw_idle);

	mutex_lock(&rot->bw_lock);
	if (rot->bw) {
		icc_set_bw(rot->mem_path, 0, 0);
		rot->bw = 0;
	}
	mutex_unlock(&rot->bw_lock);
}

/* read + write traffic at the display rate, as downstream votes it */
static void rot_bw_kick(struct dpu_hw_rotator *rot, u64 bw)
{
	if (!rot->mem_path)
		return;

	mutex_lock(&rot->bw_lock);
	if (rot->bw != bw) {
		icc_set_bw(rot->mem_path, Bps_to_icc(bw), Bps_to_icc(bw));
		rot->bw = bw;
	}
	mutex_unlock(&rot->bw_lock);
	mod_delayed_work(system_dfl_wq, &rot->bw_idle,
			 msecs_to_jiffies(ROT_ICC_IDLE_MS));
}


void dpu_hw_rotator_resume(struct dpu_hw_rotator *rot)
{
	if (IS_ERR_OR_NULL(rot))
		return;
	/* the rotator's rail vote only counts while it is runtime active */
	if (rot_has_opp(rot) && !rot->rpm_active)
		rot->rpm_active = !pm_runtime_resume_and_get(rot->dev);
	rot_clk_set_rate(rot, false);
	if (clk_prepare_enable(rot->rot_clk))
		return;
	rot_sid_setup(rot);
	rot_vbif_setup(rot);
}

void dpu_hw_rotator_suspend(struct dpu_hw_rotator *rot)
{
	if (IS_ERR_OR_NULL(rot))
		return;
	cancel_delayed_work_sync(&rot->bw_idle);
	rot_bw_idle(&rot->bw_idle.work);
	cancel_delayed_work_sync(&rot->opp_idle);
	clk_disable_unprepare(rot->rot_clk);
	if (rot->opp_max)
		dev_pm_opp_set_rate(rot->dev, 0);
	if (rot->rpm_active) {
		pm_runtime_put_sync(rot->dev);
		rot->rpm_active = false;
	}
}

int dpu_hw_rotator_rotate(struct dpu_hw_rotator *rot,
			  const struct msm_format *fmt,
			  const struct dpu_hw_fmt_layout *src,
			  const struct dpu_hw_fmt_layout *dst,
			  u32 rotation, u32 fps)
{
	bool rot90 = rotation & (DRM_MODE_ROTATE_90 | DRM_MODE_ROTATE_270);
	bool ubwc = MSM_FORMAT_IS_UBWC(fmt);
	u32 w = src->width, h = src->height;
	u32 dw = dst->width, dh = dst->height;
	u32 op_mode = SRC_OP_PE_OVERRIDE;
	u32 fetch_cfg = FETCH_CFG;
	u32 src_format, dst_format, unpack, ubwc_ctrl = 0, write_cfg = 0;
	u32 cdp = CDP_CNTL_ENABLE | CDP_CNTL_PRELOAD_64;
	u32 status, intr;
	int ret;

	if (rot->has_ubwc) {
		fetch_cfg |= rot->hbb << 18;
		ubwc_ctrl = rot_ubwc_static_ctrl(rot->dpu_kms->mdss, rot->hbb);
		write_cfg = rot->hbb << 8;
	}
	if (ubwc) {
		op_mode |= SRC_OP_BWC_DEC_EN;
		cdp |= CDP_CNTL_UBWC_META | CDP_CNTL_TILE_AMORTIZE;
	} else {
		unsigned int i;


		write_cfg |= WB_NO_PARTIAL_WRITES;
		for (i = 0; i < ARRAY_SIZE(dst->plane_pitch); i++)
			if (dst->plane_pitch[i] & PARTIAL_WRITE_ALIGN)
				write_cfg &= ~WB_NO_PARTIAL_WRITES;
	}

	src_format = (fmt->chroma_sample << 23) | (fmt->fetch_type << 19) |
		     (fmt->bpc_a << 6) | (fmt->bpc_r_cr << 4) |
		     (fmt->bpc_b_cb << 2) | (fmt->bpc_g_y << 0);
	if (rot90)
		src_format |= SRC_FORMAT_ROT90;
	if (fmt->alpha_enable && fmt->fetch_type == MDP_PLANE_INTERLEAVED)
		src_format |= SRC_FORMAT_ALPHA_EN;
	if (MSM_FORMAT_IS_DX(fmt))
		src_format |= SRC_FORMAT_DX;
	src_format |= ((fmt->unpack_count - 1) << 12) |
		      ((fmt->flags & MSM_FORMAT_FLAG_UNPACK_TIGHT ? 1 : 0) << 17) |
		      ((fmt->flags & MSM_FORMAT_FLAG_UNPACK_ALIGN_MSB ? 1 : 0) << 18) |
		      ((fmt->bpp - 1) << 9) |
		      ((fmt->fetch_mode & 3) << 30);
	unpack = (fmt->element[3] << 24) | (fmt->element[2] << 16) |
		 (fmt->element[1] << 8) | (fmt->element[0] << 0);

	dst_format = src_format & ~(SRC_FORMAT_ROT90 | SRC_FORMAT_DX |
				    SRC_FORMAT_UNPACK_TIGHT | SRC_FORMAT_UNPACK_MSB);
	dst_format |= SRC_FORMAT_ALPHA_EN;
	if (MSM_FORMAT_IS_DX(fmt))
		dst_format |= DST_FORMAT_PACK_DX;


	if (rotation & (DRM_MODE_ROTATE_90 | DRM_MODE_ROTATE_180))
		op_mode ^= (SRC_OP_FLIP_LR | SRC_OP_FLIP_UD);

	rot_bw_kick(rot, (u64)(src->total_size + dst->total_size) * fps);
	cancel_delayed_work_sync(&rot->opp_idle);
	rot_clk_set_rate(rot, !rot->race_job);
	rot_sid_setup(rot);
	rot_vbif_setup(rot);


	rwr(rot, ROTTOP_START_CTRL, 0);

	rwr(rot, ROT_SSPP_SRC_SIZE, (h << 16) | w);
	rwr(rot, ROT_SSPP_SRC_IMG_SIZE, (h << 16) | w);
	rwr(rot, ROT_SSPP_SRC_XY, 0);
	rwr(rot, ROT_SSPP_OUT_SIZE, (h << 16) | w);
	rwr(rot, ROT_SSPP_OUT_XY, 0);
	rwr(rot, ROT_SSPP_SRC0_ADDR, src->plane_addr[0]);
	rwr(rot, ROT_SSPP_SRC1_ADDR, src->plane_addr[1]);
	rwr(rot, ROT_SSPP_SRC2_ADDR, src->plane_addr[2]);
	rwr(rot, ROT_SSPP_SRC3_ADDR, src->plane_addr[3]);
	rwr(rot, ROT_SSPP_SRC_YSTRIDE0,
	    src->plane_pitch[0] | (src->plane_pitch[1] << 16));
	rwr(rot, ROT_SSPP_SRC_YSTRIDE1,
	    src->plane_pitch[2] | (src->plane_pitch[3] << 16));
	rwr(rot, ROT_SSPP_SRC_FORMAT, src_format);
	rwr(rot, ROT_SSPP_SRC_UNPACK_PATTERN, unpack);
	rwr(rot, ROT_SSPP_SRC_OP_MODE, op_mode);
	rwr(rot, ROT_SSPP_FETCH_CONFIG, fetch_cfg);
	rwr(rot, ROT_SSPP_UBWC_STATIC_CTRL, ubwc_ctrl);
	rwr(rot, ROT_SSPP_SRC_ADDR_SW_STATUS, 0);
	rwr(rot, ROT_SSPP_CREQ_LUT_0, 0);
	rwr(rot, ROT_SSPP_CREQ_LUT_1, 0);
	rwr(rot, ROT_SSPP_DANGER_LUT, 0);
	rwr(rot, ROT_SSPP_SAFE_LUT, 0xffff);
	rwr(rot, ROT_SSPP_CDP_CNTL, cdp);

	rwr(rot, ROT_WB_DST_FORMAT, dst_format);
	rwr(rot, ROT_WB_DST_OP_MODE, ubwc ? WB_OP_BWC_ENC_EN : 0);
	rwr(rot, ROT_WB_DST_PACK_PATTERN, unpack);
	rwr(rot, ROT_WB_DST0_ADDR, dst->plane_addr[0]);
	rwr(rot, ROT_WB_DST1_ADDR, dst->plane_addr[1]);
	rwr(rot, ROT_WB_DST2_ADDR, dst->plane_addr[2]);
	rwr(rot, ROT_WB_DST3_ADDR, dst->plane_addr[3]);
	rwr(rot, ROT_WB_DST_YSTRIDE0,
	    dst->plane_pitch[0] | (dst->plane_pitch[1] << 16));
	rwr(rot, ROT_WB_DST_YSTRIDE1,
	    dst->plane_pitch[2] | (dst->plane_pitch[3] << 16));
	rwr(rot, ROT_WB_OUT_SIZE, (dh << 16) | dw);
	rwr(rot, ROT_WB_OUT_IMG_SIZE, (dh << 16) | dw);
	rwr(rot, ROT_WB_OUT_XY, 0);
	rwr(rot, ROT_WB_DST_WRITE_CONFIG, write_cfg);
	rwr(rot, ROT_WB_DST_ADDR_SW_STATUS, 0);
	rwr(rot, ROT_WB_CREQ_LUT_0, 0);
	rwr(rot, ROT_WB_CREQ_LUT_1, 0);
	rwr(rot, ROT_WB_DANGER_LUT, 0);
	rwr(rot, ROT_WB_SAFE_LUT, 0xffff);
	rwr(rot, ROT_WB_CDP_CNTL, cdp);

	rwr(rot, ROTTOP_DNSC, 0);
	rwr(rot, ROTTOP_OP_MODE,
	    ROTTOP_OP_MODE_START | (rot90 ? ROTTOP_OP_MODE_ROT90 : 0));
	rwr(rot, ROTTOP_INTR_CLEAR, ROT_DONE_MASK);
	if (rot_irq_ok(rot)) {
		reinit_completion(&rot->job_done);
		rot->job_pending = true;
		cpu_latency_qos_add_request(&rot->qos, ROT_QOS_LATENCY_US);
		rwr(rot, ROTTOP_INTR_EN, ROT_DONE_MASK);
	} else {
		rwr(rot, ROTTOP_INTR_EN, 0);
	}
	wmb();
	rwr(rot, ROTTOP_START_CTRL, 1);
	if (rot->on_start)
		rot->on_start(rot->on_start_data);


	if (rot_irq_ok(rot)) {
		ret = wait_for_completion_timeout(&rot->job_done,
						  msecs_to_jiffies(300))
		      ? 0 : -ETIMEDOUT;
		cpu_latency_qos_remove_request(&rot->qos);
		intr = ret ? rrd(rot, ROTTOP_INTR_STATUS) : ROT_DONE_MASK;
		if (ret) {

			rwr(rot, ROTTOP_INTR_EN, 0);
			rot->job_pending = false;

			if (intr & ROT_DONE_MASK) {
				rot->irq_dead = true;
				DRM_WARN("dpu rotator: irq never fired, polling instead\n");
				ret = 0;
			}
		}
	} else {
		ret = readl_relaxed_poll_timeout(rot->base + ROTTOP_INTR_STATUS,
						 intr, intr & ROT_DONE_MASK,
						 100, 300000);
	}
	status = rrd(rot, ROTTOP_STATUS);
	if (ret || (status & (ROT_ERROR_BIT | ROT_BUSY_BIT))) {
		pr_err("dpu_hw_rotator: %s (status %#x intr %#x)\n",
		       ret ? "timeout" : "error", status, intr);
		rot_reset(rot);
		ret = ret ? ret : -EIO;
	}
	rwr(rot, ROTTOP_INTR_CLEAR, ROT_DONE_MASK);
	mod_delayed_work(system_dfl_wq, &rot->opp_idle,
			 msecs_to_jiffies(ROT_OPP_IDLE_MS));
	return ret;
}

struct dpu_rot_job {
	struct work_struct work;
	struct dpu_hw_rotator *rot;
	struct drm_framebuffer *fb;
	const struct msm_format *fmt;
	struct dpu_hw_fmt_layout src;
	struct dpu_hw_fmt_layout dst;
	u32 rotation;
	u32 fps;
	struct dma_fence *in_fence;
	struct dma_fence_cb cb;
	struct dma_fence fence;
	/* signalled when the frame may scan out: at the start when racing, else with fence */
	struct dma_fence scanout;
	struct drm_crtc *crtc;
	struct kref ref;
	spinlock_t lock;
	struct rcu_head rcu;
};

static const char *rot_fence_get_driver_name(struct dma_fence *f)
{
	return "dpu_rotator";
}

static const char *rot_fence_get_timeline_name(struct dma_fence *f)
{
	return "rot";
}

static const char *rot_scanout_fence_get_timeline_name(struct dma_fence *f)
{
	return "rot-scanout";
}

static void rot_job_free(struct kref *ref)
{
	struct dpu_rot_job *job = container_of(ref, struct dpu_rot_job, ref);

	kfree_rcu(job, rcu);
}

static void rot_fence_release(struct dma_fence *f)
{
	kref_put(&container_of(f, struct dpu_rot_job, fence)->ref, rot_job_free);
}

static void rot_scanout_fence_release(struct dma_fence *f)
{
	kref_put(&container_of(f, struct dpu_rot_job, scanout)->ref, rot_job_free);
}

static const struct dma_fence_ops rot_fence_ops = {
	.get_driver_name = rot_fence_get_driver_name,
	.get_timeline_name = rot_fence_get_timeline_name,
	.release = rot_fence_release,
};

static const struct dma_fence_ops rot_scanout_fence_ops = {
	.get_driver_name = rot_fence_get_driver_name,
	.get_timeline_name = rot_scanout_fence_get_timeline_name,
	.release = rot_scanout_fence_release,
};

/* time until the next frame starts scanning out */
static s64 rot_next_frame_us(struct drm_crtc *crtc)
{
	struct drm_vblank_crtc *vblank = drm_crtc_vblank_crtc(crtc);
	ktime_t now = ktime_get(), last;
	s64 dur = vblank->framedur_ns;

	drm_crtc_vblank_count_and_time(crtc, &last);
	if (!dur || !last)
		return 0;
	last = ktime_add_ns(last, (div64_s64(ktime_to_ns(ktime_sub(now, last)), dur) + 1) * dur);
	return ktime_us_delta(last, now);
}

static void rot_job_started(void *data)
{
	struct dpu_rot_job *job = data;

	if (job->rot->race_job && rot_next_frame_us(job->crtc) >= ROT_RACE_MIN_US)
		dma_fence_signal(&job->scanout);
}

static void rot_job_work(struct work_struct *work)
{
	struct dpu_rot_job *job = container_of(work, struct dpu_rot_job, work);
	struct dpu_hw_rotator *rot = job->rot;
	struct device *dev = &rot->dpu_kms->pdev->dev;

	pm_runtime_get_sync(dev);
	rot->on_start_data = job;
	rot->on_start = rot_job_started;
	/* a job that will race needs no more than the lowest OPP; one that can't gets the fast clock */
	rot->race_job = dpu_rot_race && job->crtc &&
			rot_next_frame_us(job->crtc) >= ROT_RACE_MIN_US;
	dpu_hw_rotator_rotate(rot, job->fmt, &job->src, &job->dst,
			      job->rotation, job->fps);
	rot->race_job = false;
	rot->on_start = NULL;
	pm_runtime_put_sync(dev);
	dma_fence_signal(&job->scanout);
	dma_fence_signal(&job->fence);
	dma_fence_put(job->in_fence);
	msm_framebuffer_cleanup(job->fb, false);
	drm_framebuffer_put(job->fb);
	dma_fence_put(&job->scanout);
	dma_fence_put(&job->fence);
}

static void rot_job_in_fence_cb(struct dma_fence *f, struct dma_fence_cb *cb)
{
	struct dpu_rot_job *job = container_of(cb, struct dpu_rot_job, cb);

	queue_work(job->rot->wq, &job->work);
}

struct dma_fence *dpu_hw_rotator_queue(struct dpu_hw_rotator *rot,
				       struct drm_framebuffer *fb,
				       struct dma_fence *in_fence,
				       const struct dpu_hw_fmt_layout *dst,
				       u32 rotation, u32 fps,
				       struct drm_crtc *crtc,
				       struct dma_fence **scanout)
{
	struct dpu_rot_job *job;

	if (!rot->wq)
		return NULL;

	job = kzalloc(sizeof(*job), GFP_KERNEL);
	if (!job)
		return NULL;

	if (msm_framebuffer_prepare(fb, false)) {
		kfree(job);
		return NULL;
	}
	if (dpu_format_populate_plane_sizes(fb, &job->src)) {
		msm_framebuffer_cleanup(fb, false);
		kfree(job);
		return NULL;
	}
	dpu_format_populate_addrs(fb, &job->src);
	drm_framebuffer_get(fb);

	job->rot = rot;
	job->fb = fb;
	job->fmt = msm_framebuffer_format(fb);
	job->dst = *dst;
	job->rotation = rotation;
	job->fps = fps;
	job->in_fence = in_fence;
	job->crtc = crtc;
	spin_lock_init(&job->lock);
	INIT_WORK(&job->work, rot_job_work);
	/* one job reference per fence; the worker holds a reference on each */
	kref_init(&job->ref);
	dma_fence_init(&job->fence, &rot_fence_ops, &job->lock,
		       rot->fence_context,
		       atomic64_inc_return(&rot->fence_seqno));
	dma_fence_get(&job->fence);
	kref_get(&job->ref);
	dma_fence_init(&job->scanout, &rot_scanout_fence_ops, &job->lock,
		       rot->scanout_context,
		       atomic64_inc_return(&rot->scanout_seqno));
	*scanout = dma_fence_get(&job->scanout);

	if (!in_fence || dma_fence_add_callback(in_fence, &job->cb,
						rot_job_in_fence_cb))
		queue_work(rot->wq, &job->work);

	return &job->fence;
}

struct dpu_hw_rotator *dpu_hw_rotator_init(struct drm_device *dev,
					   struct dpu_kms *dpu_kms)
{
	struct dpu_hw_rotator *rot;
	struct device_node *np;
	struct platform_device *pdev;
	struct resource res;
	int ret;

	np = of_find_matching_node(NULL, dpu_hw_rotator_of_match);
	if (!np)
		return NULL;
	if (!of_device_is_available(np)) {
		of_node_put(np);
		return NULL;
	}

	rot = devm_kzalloc(dev->dev, sizeof(*rot), GFP_KERNEL);
	if (!rot) {
		ret = -ENOMEM;
		goto put_node;
	}
	rot->dpu_kms = dpu_kms;

	ret = of_address_to_resource(np, 0, &res);
	if (ret)
		goto put_node;
	rot->base = devm_ioremap(dev->dev, res.start, resource_size(&res));
	if (!rot->base) {
		ret = -ENOMEM;
		goto put_node;
	}
	if (!of_address_to_resource(np, 1, &res))
		rot->vbif = devm_ioremap(dev->dev, res.start,
					 resource_size(&res));
	if (!of_address_to_resource(np, 2, &res))
		rot->sid = devm_ioremap(dev->dev, res.start,
					resource_size(&res));

	rot->rot_clk = of_clk_get_by_name(np, "rot");
	if (IS_ERR(rot->rot_clk)) {
		ret = PTR_ERR(rot->rot_clk);
		goto put_node;
	}
	rot->core_clk = msm_clk_bulk_get_clock(dpu_kms->clocks,
					       dpu_kms->num_clocks, "core");

	pdev = of_find_device_by_node(np);
	if (pdev) {
		rot->dev = &pdev->dev;
		rot->mem_path = of_icc_get(&pdev->dev, "mem");
		if (IS_ERR(rot->mem_path)) {
			DRM_DEV_INFO(dev->dev, "dpu rotator: no bandwidth path (%ld)\n",
				     PTR_ERR(rot->mem_path));
			rot->mem_path = NULL;
		}
	}
	mutex_init(&rot->bw_lock);
	INIT_DELAYED_WORK(&rot->bw_idle, rot_bw_idle);
	INIT_DELAYED_WORK(&rot->opp_idle, rot_opp_idle);


	rot_clk_set_rate(rot, false);
	ret = clk_prepare_enable(rot->rot_clk);
	if (ret)
		goto put_node;
	rot->hw_version = rrd(rot, ROTTOP_HW_VERSION);
	rot->has_ubwc = rrd(rot, ROTTOP_UBWC_DEC_VERSION) != 0;
	rot->hbb = dpu_kms->mdss->highest_bank_bit - 13;
	rot_reset(rot);
	rot_sid_setup(rot);
	rot_vbif_setup(rot);

	init_completion(&rot->job_done);
	rot->irq = of_irq_get(np, 0);
	if (rot->irq > 0 &&
	    devm_request_irq(dev->dev, rot->irq, rot_isr, IRQF_SHARED,
			     "dpu-rotator", rot))
		rot->irq = 0;

	rot->wq = alloc_ordered_workqueue("dpu_rotator", WQ_HIGHPRI);
	rot->fence_context = dma_fence_context_alloc(1);
	rot->scanout_context = dma_fence_context_alloc(1);

	DRM_DEV_INFO(dev->dev, "dpu rotator %#x ubwc %d vbif %d sid %d\n",
		     rot->hw_version, rot->has_ubwc, !!rot->vbif, !!rot->sid);
	of_node_put(np);
	return rot;

put_node:
	of_node_put(np);
	return ERR_PTR(ret);
}

static int dpu_hw_rotator_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;

	ret = devm_pm_opp_set_clkname(dev, "rot");
	if (ret)
		return ret;
	ret = devm_pm_opp_of_add_table(dev);
	if (ret && ret != -ENODEV)
		return ret;

	return devm_pm_runtime_enable(dev);
}

static const struct of_device_id dpu_hw_rotator_stub_match[] = {
	{ .compatible = "qcom,sm8250-sde-rotator" },
	{ }
};

struct platform_driver dpu_hw_rotator_driver = {
	.probe = dpu_hw_rotator_probe,
	.driver = {
		.name = "dpu_sde_rotator",
		.of_match_table = dpu_hw_rotator_stub_match,
		.suppress_bind_attrs = true,
	},
};
