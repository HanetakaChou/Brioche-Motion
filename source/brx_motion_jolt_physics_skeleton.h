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

// #define JPH_DEBUG_RENDERER
// #define JPH_PROFILE_ENABLED
// #define JPH_OBJECT_STREAM
// #define JPH_USE_AVX2
// #define JPH_USE_AVX
// #define JPH_USE_SSE4_1
// #define JPH_USE_SSE4_2
// #define JPH_USE_LZCNT
// #define JPH_USE_TZCNT
// #define JPH_USE_F16C
// #define JPH_USE_FMADD
#include <Jolt/Jolt.h>
#include <Jolt/Skeleton/Skeleton.h>
#include <Jolt/Skeleton/SkeletonPose.h>
#include <Jolt/Skeleton/SkeletonMapper.h>

class brx_motion_jolt_physics_skeleton final : public brx_motion_skeleton
{
	JPH::Skeleton m_skeleton;
	JPH::SkeletonPose m_pose;
	JPH::SkeletonMapper m_neck_to_head_reaching_ik;
	JPH::SkeletonMapper m_shoulder_to_arm_reaching_ik;
	JPH::SkeletonMapper m_hip_to_leg_reaching_ik;

public:
	brx_motion_jolt_physics_skeleton();
	~brx_motion_jolt_physics_skeleton();
	void init(int32_t joint_count, char const *const *joint_names, int32_t const *parent_indices, int32_t const vrm_joint_indices[BRX_MOTION_VRM_JOINT_NAME_COUNT]);
	void uninit();

private:
	void brx_motion_skelton_get_pose(float (*out_joint_model_space_rotations)[4], float (*out_joint_model_space_translations)[3]) const override;
	void destroy() override;
};

class brx_motion_jolt_physics_skeleton_animation final : public brx_motion_skeleton_animation
{
	mcrt_vector<JPH::String> m_joint_names;
	mcrt_vector<JPH::SkeletonPose::JointState> m_poses_model_space;

public:
	brx_motion_jolt_physics_skeleton_animation();
	~brx_motion_jolt_physics_skeleton_animation();
	void init(int32_t joint_count, char const *const *joint_names, uint32_t frame_count, float const (*joint_model_space_rotations)[4], float const (*joint_model_space_translations)[3]);
	void uninit();

private:
};