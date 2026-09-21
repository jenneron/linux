// SPDX-License-Identifier: GPL-2.0-only

#include <linux/dma-fence.h>
#include <linux/kref.h>
#include <linux/slab.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem.h>

#include "msm_drv.h"
#include "msm_gem.h"
#include "dpu_formats.h"
#include "dpu_hw_rotator.h"
#include "dpu_hw_sspp.h"
#include "dpu_kms.h"
#include "dpu_plane.h"
#include "dpu_plane_rot.h"
#include "dpu_rm.h"

#define MAX_UPSCALE_RATIO	20
#define MAX_DOWNSCALE_RATIO	4

struct dpu_plane_rot_bufs {
	struct kref ref;
	struct drm_gpuvm *vm;
	struct drm_gem_object *bo[3];
	u64 iova[3];

	struct dma_fence *fence[3];
	size_t sz;
};

static struct dpu_kms *dpu_plane_rot_kms(struct drm_plane *plane)
{
	struct msm_drm_private *priv = plane->dev->dev_private;

	return to_dpu_kms(priv->kms);
}

static bool dpu_plane_rot_needed(struct dpu_kms *dpu_kms,
				     struct drm_plane_state *state)
{
	if (!(state->rotation & (DRM_MODE_ROTATE_90 | DRM_MODE_ROTATE_270)))
		return false;
	if (state->rotation & DRM_MODE_REFLECT_MASK)
		return false;

	return dpu_hw_rotator_accepts(dpu_kms->rotator,
				      msm_framebuffer_format(state->fb));
}

static u32 dpu_plane_rot_fps(struct drm_plane_state *state)
{
	struct drm_crtc_state *crtc_state = NULL;
	int fps;

	if (state->state)
		crtc_state = drm_atomic_get_new_crtc_state(state->state,
							   state->crtc);
	if (!crtc_state)
		crtc_state = state->crtc->state;
	fps = drm_mode_vrefresh(&crtc_state->mode);

	return fps > 0 ? fps : 60;
}

static void dpu_plane_rot_bufs_release(struct kref *ref)
{
	struct dpu_plane_rot_bufs *bufs =
		container_of(ref, struct dpu_plane_rot_bufs, ref);
	int i;

	for (i = 0; i < 3; i++) {
		if (bufs->fence[i]) {
			dma_fence_wait(bufs->fence[i], false);
			dma_fence_put(bufs->fence[i]);
		}
		if (!bufs->bo[i])
			continue;
		if (bufs->iova[i])
			msm_gem_unpin_iova(bufs->bo[i], bufs->vm);
		drm_gem_object_put(bufs->bo[i]);
	}
	kfree(bufs);
}

void dpu_plane_rot_bufs_put(struct dpu_plane_rot_bufs *bufs)
{
	if (bufs)
		kref_put(&bufs->ref, dpu_plane_rot_bufs_release);
}

static int dpu_plane_rot_alloc(struct drm_plane *plane,
			       struct dpu_plane_rot *rot, size_t sz)
{
	struct dpu_kms *dpu_kms = dpu_plane_rot_kms(plane);
	struct dpu_plane_rot_bufs *bufs;
	int i, ret;

	if (rot->bufs && rot->bufs->sz >= sz)
		return 0;

	bufs = kzalloc(sizeof(*bufs), GFP_KERNEL);
	if (!bufs)
		return -ENOMEM;
	kref_init(&bufs->ref);
	bufs->vm = dpu_kms->base.vm;

	for (i = 0; i < 3; i++) {
		bufs->bo[i] = msm_gem_new(plane->dev, sz, MSM_BO_WC, NULL);
		if (IS_ERR(bufs->bo[i])) {
			ret = PTR_ERR(bufs->bo[i]);
			bufs->bo[i] = NULL;
			goto err;
		}
		ret = msm_gem_get_and_pin_iova(bufs->bo[i], bufs->vm,
					       &bufs->iova[i]);
		if (ret)
			goto err;
	}
	bufs->sz = sz;


	dpu_plane_rot_bufs_put(rot->bufs);
	rot->bufs = bufs;
	return 0;
err:
	dpu_plane_rot_bufs_put(bufs);
	return ret;
}

static int dpu_plane_rot_check_geometry(struct drm_plane_state *new_plane_state,
					const struct drm_crtc_state *crtc_state,
					const struct dpu_mdss_cfg *catalog)
{
	struct dpu_plane_state *pstate = to_dpu_plane_state(new_plane_state);
	struct drm_framebuffer *fb = new_plane_state->fb;
	struct dpu_sw_pipe_cfg *pipe_cfg = &pstate->pipe_cfg[0];
	struct dpu_sw_pipe_cfg *r_pipe_cfg = &pstate->pipe_cfg[1];
	struct dpu_hw_sspp *sspp = pstate->pipe[0].sspp;
	const struct dpu_sspp_sub_blks *sblk = sspp->cap->sblk;
	int min_scale = FRAC_16_16(1, MAX_UPSCALE_RATIO);
	int max_scale = MAX_DOWNSCALE_RATIO << 16;
	u32 rot_w = fb->height, rot_h = fb->width;
	int ret;

	ret = drm_atomic_helper_check_plane_state(new_plane_state, crtc_state,
						  min_scale, max_scale, true, true);
	if (ret)
		return ret;

	if (!new_plane_state->visible)
		return 0;

	new_plane_state->src.x1 = 0;
	new_plane_state->src.y1 = 0;
	new_plane_state->src.x2 = fb->width << 16;
	new_plane_state->src.y2 = fb->height << 16;

	pstate->stage = DPU_STAGE_0 + pstate->base.normalized_zpos;
	if (pstate->stage >= catalog->caps->max_mixer_blendstages) {
		DPU_ERROR("> %d plane stages assigned\n",
			  catalog->caps->max_mixer_blendstages - DPU_STAGE_0);
		return -EINVAL;
	}

	if (rot_w > catalog->caps->max_linewidth) {
		pr_info_ratelimited("dpu rotator: reject - rot_w %u > linewidth %u\n",
				    rot_w, catalog->caps->max_linewidth);
		return -E2BIG;
	}

	pipe_cfg->src_rect.x1 = 0;
	pipe_cfg->src_rect.y1 = 0;
	pipe_cfg->src_rect.x2 = rot_w;
	pipe_cfg->src_rect.y2 = rot_h;
	pipe_cfg->dst_rect = new_plane_state->dst;
	pipe_cfg->rotation = 0;

	if (!(sblk->scaler_blk.len && sspp->ops.setup_scaler) &&
	    (rot_w != drm_rect_width(&pipe_cfg->dst_rect) ||
	     rot_h != drm_rect_height(&pipe_cfg->dst_rect))) {
		pr_info_ratelimited("dpu rotator: reject - non-scaler %ux%u != dst %ux%u\n",
				    rot_w, rot_h, drm_rect_width(&pipe_cfg->dst_rect),
				    drm_rect_height(&pipe_cfg->dst_rect));
		return -ERANGE;
	}

	memset(r_pipe_cfg, 0, sizeof(*r_pipe_cfg));

	ret = dpu_format_populate_plane_sizes(fb, &pstate->layout);
	if (ret)
		return ret;

	pstate->pipe[0].multirect_index = DPU_SSPP_RECT_SOLO;
	pstate->pipe[0].multirect_mode = DPU_SSPP_MULTIRECT_NONE;
	pstate->pipe[1].sspp = NULL;

	pstate->needs_qos_remap = drm_atomic_crtc_needs_modeset(crtc_state);
	pstate->needs_rotate = true;
	return 0;
}

int dpu_plane_rot_check(struct drm_plane *plane,
			struct drm_plane_state *plane_state,
			const struct drm_crtc_state *crtc_state,
			struct dpu_global_state *global_state,
			struct drm_crtc *crtc,
			const struct dpu_mdss_cfg *catalog)
{
	struct dpu_kms *dpu_kms = dpu_plane_rot_kms(plane);
	struct dpu_plane_state *pstate = to_dpu_plane_state(plane_state);
	struct dpu_rm_sspp_requirements reqs;
	int ret;

	if (!crtc_state)
		return 0;
	if (!dpu_plane_rot_needed(dpu_kms, plane_state))
		return (plane_state->rotation & (DRM_MODE_ROTATE_90 | DRM_MODE_ROTATE_270)) ? -EINVAL : 0;

	reqs.yuv = false;
	reqs.scale = (plane_state->fb->height != drm_rect_width(&plane_state->dst)) ||
		     (plane_state->fb->width != drm_rect_height(&plane_state->dst));
	reqs.rot90 = false;
	pstate->pipe[0].sspp = dpu_rm_reserve_sspp(&dpu_kms->rm, global_state,
						   crtc, &reqs);
	if (!pstate->pipe[0].sspp)
		return -EINVAL;

	ret = dpu_plane_rot_check_geometry(plane_state, crtc_state, catalog);

	return ret ? ret : 1;
}

int dpu_plane_rot_prepare(struct drm_plane *plane, struct dpu_plane_rot *rot,
			  struct drm_plane_state *new_state)
{
	struct dpu_kms *dpu_kms = dpu_plane_rot_kms(plane);
	struct dpu_plane_state *pstate = to_dpu_plane_state(new_state);
	struct drm_framebuffer *fb = new_state->fb;
	const struct msm_format *fmt = msm_framebuffer_format(fb);
	unsigned int idx = (rot->idx + 1) % 3;
	struct dpu_plane_rot_bufs *bufs;
	struct dpu_hw_fmt_layout dst;
	struct dma_fence *done, *scanout = NULL;
	int ret;

	ret = dpu_hw_rotator_dst_layout(fmt, fb->height, fb->width, 0, &dst);
	if (ret)
		return ret;

	ret = dpu_plane_rot_alloc(plane, rot, dst.total_size);
	if (ret) {
		DPU_ERROR("failed to alloc rotator scratch\n");
		return ret;
	}


	bufs = rot->bufs;
	kref_get(&bufs->ref);
	pstate->rot_bufs = bufs;

	dpu_hw_rotator_dst_layout(fmt, fb->height, fb->width,
				  (u32)bufs->iova[idx], &dst);
	done = dpu_hw_rotator_queue(dpu_kms->rotator, fb, new_state->fence,
				    &dst, new_state->rotation,
				    dpu_plane_rot_fps(new_state),
				    new_state->crtc, &scanout);
	if (done) {
		/* scratch reuse waits for the rotate, the flip only for scanout */
		dma_fence_put(bufs->fence[idx]);
		bufs->fence[idx] = done;
		new_state->fence = scanout;
		pstate->rot_dst_iova = (u32)bufs->iova[idx];
		rot->idx = idx;
	}

	return 0;
}

void dpu_plane_rot_update(struct drm_plane *plane, struct dpu_plane_rot *rot,
			  struct drm_plane_state *new_state)
{
	struct dpu_kms *dpu_kms = dpu_plane_rot_kms(plane);
	struct dpu_plane_state *pstate = to_dpu_plane_state(new_state);
	struct dpu_plane_rot_bufs *bufs = pstate->rot_bufs;
	struct drm_framebuffer *fb = new_state->fb;
	const struct msm_format *fmt = msm_framebuffer_format(fb);
	u32 dst_iova = pstate->rot_dst_iova;
	struct dpu_hw_fmt_layout dst;

	if (!bufs)
		return;

	if (!dst_iova) {
		unsigned int idx = (rot->idx + 1) % 3;

		if (bufs->fence[idx])
			dma_fence_wait(bufs->fence[idx], false);
		dst_iova = (u32)bufs->iova[idx];
		if (dpu_hw_rotator_dst_layout(fmt, fb->height, fb->width,
					      dst_iova, &dst))
			return;
		dpu_hw_rotator_rotate(dpu_kms->rotator, fmt, &pstate->layout,
				      &dst, new_state->rotation,
				      dpu_plane_rot_fps(new_state));
		rot->idx = idx;
	} else if (dpu_hw_rotator_dst_layout(fmt, fb->height, fb->width,
					     dst_iova, &dst)) {
		return;
	}

	pstate->layout = dst;
}
