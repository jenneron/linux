/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef _DPU_PLANE_ROT_H
#define _DPU_PLANE_ROT_H

#include <linux/types.h>

struct dpu_global_state;
struct dpu_mdss_cfg;
struct drm_crtc;
struct drm_crtc_state;
struct drm_gem_object;
struct drm_plane;
struct drm_plane_state;

struct dpu_plane_rot_bufs;

struct dpu_plane_rot {

	struct dpu_plane_rot_bufs *bufs;
	unsigned int idx;
};

void dpu_plane_rot_bufs_put(struct dpu_plane_rot_bufs *bufs);

int dpu_plane_rot_check(struct drm_plane *plane,
			struct drm_plane_state *plane_state,
			const struct drm_crtc_state *crtc_state,
			struct dpu_global_state *global_state,
			struct drm_crtc *crtc,
			const struct dpu_mdss_cfg *catalog);

int dpu_plane_rot_prepare(struct drm_plane *plane, struct dpu_plane_rot *rot,
			  struct drm_plane_state *new_state);

void dpu_plane_rot_update(struct drm_plane *plane, struct dpu_plane_rot *rot,
			  struct drm_plane_state *new_state);

#endif
