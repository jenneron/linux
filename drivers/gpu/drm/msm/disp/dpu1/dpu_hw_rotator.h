/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _DPU_HW_ROTATOR_H
#define _DPU_HW_ROTATOR_H

#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/io.h>

struct clk;
struct drm_crtc;
struct dpu_hw_fmt_layout;
struct dpu_kms;
struct drm_device;
struct drm_framebuffer;
struct dma_fence;
struct icc_path;
struct msm_format;
struct platform_driver;
struct workqueue_struct;

struct dpu_hw_rotator {
	void __iomem *base;
	void __iomem *vbif;
	void __iomem *sid;
	struct dpu_kms *dpu_kms;
	struct clk *rot_clk;
	struct clk *core_clk;
	struct icc_path *mem_path;
	struct device *dev;
	unsigned long opp_min, opp_max;
	struct delayed_work opp_idle;
	bool rpm_active;

	struct mutex bw_lock;
	struct delayed_work bw_idle;
	u64 bw;
	struct workqueue_struct *wq;
	u64 fence_context;
	atomic64_t fence_seqno;
	u64 scanout_context;
	atomic64_t scanout_seqno;
	void (*on_start)(void *data);
	void *on_start_data;
	bool race_job;
	u32 hw_version;
	u32 hbb;
	bool has_ubwc;

	int irq;
	bool irq_dead;
	bool job_pending;
	struct completion job_done;
	struct pm_qos_request qos;
};

bool dpu_hw_rotator_accepts(struct dpu_hw_rotator *rot,
			    const struct msm_format *fmt);

int dpu_hw_rotator_dst_layout(const struct msm_format *fmt, u32 width,
			      u32 height, u32 iova,
			      struct dpu_hw_fmt_layout *layout);

int dpu_hw_rotator_rotate(struct dpu_hw_rotator *rot,
			  const struct msm_format *fmt,
			  const struct dpu_hw_fmt_layout *src,
			  const struct dpu_hw_fmt_layout *dst,
			  u32 rotation, u32 fps);

struct dma_fence *dpu_hw_rotator_queue(struct dpu_hw_rotator *rot,
				       struct drm_framebuffer *fb,
				       struct dma_fence *in_fence,
				       const struct dpu_hw_fmt_layout *dst,
				       u32 rotation, u32 fps,
				       struct drm_crtc *crtc,
				       struct dma_fence **scanout);

struct dpu_hw_rotator *dpu_hw_rotator_init(struct drm_device *dev,
					   struct dpu_kms *dpu_kms);

void dpu_hw_rotator_resume(struct dpu_hw_rotator *rot);
void dpu_hw_rotator_suspend(struct dpu_hw_rotator *rot);

extern struct platform_driver dpu_hw_rotator_driver;

#endif
