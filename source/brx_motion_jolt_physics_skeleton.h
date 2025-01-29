//
// Copyright (C) YuqiaoZhang(HanetakaChou)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//

#include "../include/brx_motion.h"
#include "../../McRT-Malloc/include/mcrt_vector.h"
#include "../../McRT-Malloc/include/mcrt_string.h"

class brx_motion_jolt_physics_skeleton final : public brx_motion_skeleton
{
	mcrt_vector<mcrt_string> m_skeleton_joint_names;
	mcrt_vector<int32_t> m_skeleton_joint_parent_indices;
	mcrt_vector<brx_motion_skeleton_joint_transform> m_skeleton_bind_pose_joint_transforms;

public:
	brx_motion_jolt_physics_skeleton();
	~brx_motion_jolt_physics_skeleton();
	void init(uint32_t skeleton_joint_count, char const *const *skeleton_joint_names, uint32_t const *skeleton_joint_parent_indices, brx_motion_skeleton_joint_transform const *skeleton_bind_pose_joint_transforms, uint32_t const vrm_skeleton_joint_indices[BRX_MOTION_VRM_SKELETON_JOINT_NAME_COUNT]);
	void uninit();
	uint32_t get_skeleton_joint_count() const override;
	mcrt_vector<mcrt_string> const *get_skeleton_joint_names() const;
	int32_t const *get_skeleton_joint_parent_indices() const;
	brx_motion_skeleton_joint_transform const *get_skeleton_bind_pose_joint_transforms() const override;
};

class brx_motion_jolt_physics_skeleton_animation final : public brx_motion_skeleton_animation
{
	mcrt_vector<mcrt_string> m_skeleton_joint_names;
	// Model Space
	mcrt_vector<brx_motion_skeleton_joint_transform> m_skeleton_animation_joint_transforms;
	uint32_t m_animation_frame_rate;

public:
	brx_motion_jolt_physics_skeleton_animation();
	~brx_motion_jolt_physics_skeleton_animation();
	void init(uint32_t skeleton_joint_count, char const *const *skeleton_joint_names, uint32_t animation_frame_rate, uint32_t animation_frame_count, brx_motion_skeleton_joint_transform const *skeleton_animation_joint_transforms);
	void uninit();
	uint32_t get_skeleton_joint_count() const;
	mcrt_vector<mcrt_string> const *get_skeleton_joint_names() const;
	uint32_t get_animation_frame_count() const;
	brx_motion_skeleton_joint_transform const *get_skeleton_animation_joint_transforms() const;
	uint32_t get_animation_frame_rate() const;
};

class brx_motion_jolt_physics_skeleton_animation_instance final : public brx_motion_skeleton_animation_instance
{
	mcrt_vector<brx_motion_skeleton_joint_transform> m_skeleton_animation_joint_transforms;
	uint32_t m_animation_frame_rate;
	uint32_t m_animation_frame_count;
	uint32_t m_skeleton_joint_count;
	float m_animation_duration;
	float m_animation_time;

public:
	brx_motion_jolt_physics_skeleton_animation_instance();
	~brx_motion_jolt_physics_skeleton_animation_instance();
	void init(brx_motion_skeleton_animation *skeleton_animation, brx_motion_skeleton *skeleton);
	void uninit();

private:
	void step(float delta_time) override;
	brx_motion_skeleton_joint_transform const *get_skeleton_animation_joint_transforms() const override;
};
