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

#ifndef _BRX_MOTION_ANIMATION_H_
#define _BRX_MOTION_ANIMATION_H_ 1

#include "../include/brx_motion.h"
#include "../../McRT-Malloc/include/mcrt_vector.h"
#include "../../McRT-Malloc/include/mcrt_unordered_map.h"
#include "../../Brioche-Physics/include/brx_physics.h"
#include "brx_motion_video_detector.h"
#if defined(__GNUC__)
// GCC or CLANG
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunknown-pragmas"
#include <DirectXMath.h>
#pragma GCC diagnostic pop
#elif defined(_MSC_VER)
// MSVC or CLANG-CL
#include <DirectXMath.h>
#else
#error Unknown Compiler
#endif

struct brx_animation_rigid_transform
{
	DirectX::XMFLOAT4 m_rotation;
	DirectX::XMFLOAT3 m_translation;
};

struct brx_animation_skeleton_joint_constraint
{
	BRX_MOTION_SKELETON_JOINT_CONSTRAINT_TYPE m_constraint_type;

	union
	{
		struct
		{
			uint32_t m_source_joint_index;
			// uint32_t m_source_weight_count;
			// float *m_source_weights;
			uint32_t m_destination_joint_index;
			bool m_copy_rotation;
			bool m_copy_translation;
		} m_copy_transform;

		struct
		{
			uint32_t m_ik_end_effector_index;
			// uint32_t m_ik_joint_count;
			// uint32_t *m_ik_joint_indices;
			uint32_t m_target_joint_index;
			DirectX::XMFLOAT3 m_ik_two_joints_hinge_joint_axis_local_space;
			float m_cosine_max_ik_two_joints_hinge_joint_angle;
			float m_cosine_min_ik_two_joints_hinge_joint_angle;
		} m_inverse_kinematics;
	};
};

struct brx_animation_physics_rigid_body
{
	float m_model_space_rotation[4];
	float m_model_space_translation[3];
	BRX_PHYSICS_RIGID_BODY_SHAPE_TYPE m_shape_type;
	float m_shape_size[3];
	BRX_PHYSICS_RIGID_BODY_MOTION_TYPE m_motion_type;
	uint32_t m_collision_filter_group;
	uint32_t m_collision_filter_mask;
	float m_mass;
	float m_linear_damping;
	float m_angular_damping;
	float m_friction;
	float m_restitution;
};

struct brx_animation_physics_constraint
{
	uint32_t m_rigid_body_a_index;
	uint32_t m_rigid_body_b_index;
	BRX_PHYSICS_CONSTRAINT_TYPE m_constraint_type;
	float m_pivot[3];
	float m_twist_axis[3];
	float m_plane_axis[3];
	float m_normal_axis[3];
	float m_twist_limit[2];
	float m_plane_limit[2];
	float m_normal_limit[2];
};

struct brx_animation_ragdoll_direct_mapping
{
	uint32_t m_joint_index_a;
	uint32_t m_joint_index_b;
	DirectX::XMFLOAT4X4 m_a_to_b_transform_model_space;
};

class brx_motion_animation_animation final : public brx_motion_animation
{
	uint32_t m_ref_count;

	mcrt_unordered_map<BRX_MOTION_MORPH_TARGET_NAME, uint32_t> m_weight_channel_indices;
	mcrt_vector<float> m_weights;

	mcrt_unordered_map<BRX_MOTION_SKELETON_JOINT_NAME, uint32_t> m_rigid_transform_channel_indices;
	mcrt_vector<brx_animation_rigid_transform> m_rigid_transforms;

	mcrt_unordered_map<BRX_MOTION_SKELETON_JOINT_CONSTRAINT_NAME, uint32_t> m_switch_channel_indices;
	mcrt_vector<uint8_t> m_switches;

public:
	inline brx_motion_animation_animation(mcrt_unordered_map<BRX_MOTION_MORPH_TARGET_NAME, uint32_t> &&weight_channel_indices, mcrt_vector<float> weights, mcrt_unordered_map<BRX_MOTION_SKELETON_JOINT_NAME, uint32_t> &&rigid_transform_channel_indices, mcrt_vector<brx_animation_rigid_transform> &&rigid_transforms, mcrt_unordered_map<BRX_MOTION_SKELETON_JOINT_CONSTRAINT_NAME, uint32_t> &&switch_channel_indices, mcrt_vector<uint8_t> &&switches);
	inline ~brx_motion_animation_animation();
	inline void retain();
	inline uint32_t internal_release();

	inline uint32_t get_frame_count() const;

	inline float const *get_weight(uint32_t frame_index, BRX_MOTION_MORPH_TARGET_NAME channel_name) const;

	inline brx_animation_rigid_transform const *get_rigid_transform(uint32_t frame_index, BRX_MOTION_SKELETON_JOINT_NAME channel_name) const;

	inline uint8_t const *get_switch(uint32_t frame_index, BRX_MOTION_SKELETON_JOINT_CONSTRAINT_NAME channel_name) const;
};

class brx_motion_animation_animation_instance final : public brx_motion_animation_instance
{
	uint32_t m_ref_count;

	brx_motion_animation_animation *m_animation;

	float m_remainder_time;

	float m_delta_time;

	uint32_t m_frame_index;

	bool m_continuous;

public:
	inline brx_motion_animation_animation_instance();
	inline ~brx_motion_animation_animation_instance();
	inline void init(brx_motion_animation_animation *animation);
	inline void uninit();
	inline void retain();
	inline uint32_t internal_release();

private:
	void step(float delta_time) override;
	float get_morph_target_weight(BRX_MOTION_MORPH_TARGET_NAME morph_target_name) const override;

public:
	inline brx_motion_animation_animation const *get_animation() const;
	inline float get_delta_time() const;
	uint32_t get_frame_index() const override;
	inline bool get_continuous() const;
};

class brx_motion_animation_skeleton final : public brx_motion_skeleton
{
	uint32_t m_ref_count;

	mcrt_vector<BRX_MOTION_SKELETON_JOINT_NAME> m_animation_skeleton_joint_names;
	mcrt_vector<uint32_t> m_animation_skeleton_joint_parent_indices;
	mcrt_vector<brx_animation_rigid_transform> m_animation_skeleton_bind_pose_local_space;
	mcrt_vector<DirectX::XMFLOAT4X4> m_animation_skeleton_bind_pose_inverse_model_space;

	mcrt_unordered_map<BRX_MOTION_SKELETON_JOINT_NAME, uint32_t> m_animation_skeleton_joint_indices;

	mcrt_vector<BRX_MOTION_SKELETON_JOINT_CONSTRAINT_NAME> m_animation_skeleton_joint_constraint_names;
	mcrt_vector<brx_animation_skeleton_joint_constraint> m_animation_skeleton_joint_constraints;
	mcrt_vector<mcrt_vector<uint32_t>> m_animation_skeleton_joint_constraints_storages;

	mcrt_vector<brx_animation_physics_rigid_body> m_ragdoll_skeleton_rigid_bodies;
	mcrt_vector<brx_animation_physics_constraint> m_ragdoll_skeleton_constraints;

	mcrt_vector<brx_animation_ragdoll_direct_mapping> m_animation_to_ragdoll_direct_mapping;
	mcrt_vector<brx_animation_ragdoll_direct_mapping> m_ragdoll_to_animation_direct_mapping;

public:
	inline brx_motion_animation_skeleton(mcrt_vector<BRX_MOTION_SKELETON_JOINT_NAME> &&animation_skeleton_joint_names, mcrt_vector<uint32_t> &&animation_skeleton_joint_parent_indices, mcrt_vector<brx_animation_rigid_transform> &&animation_skeleton_bind_pose_local_space, mcrt_vector<DirectX::XMFLOAT4X4> &&animation_skeleton_bind_pose_inverse_model_space, mcrt_vector<BRX_MOTION_SKELETON_JOINT_CONSTRAINT_NAME> &&animation_skeleton_joint_constraint_names, mcrt_vector<brx_animation_skeleton_joint_constraint> &&animation_skeleton_joint_constraints, mcrt_vector<mcrt_vector<uint32_t>> &&animation_skeleton_joint_constraints_storages, mcrt_vector<brx_animation_physics_rigid_body> &&ragdoll_skeleton_rigid_bodies, mcrt_vector<brx_animation_physics_constraint> &&ragdoll_skeleton_constraints, mcrt_vector<brx_animation_ragdoll_direct_mapping> &&animation_to_ragdoll_direct_mapping, mcrt_vector<brx_animation_ragdoll_direct_mapping> &&ragdoll_to_animation_direct_mapping);
	inline ~brx_motion_animation_skeleton();
	inline void retain();
	inline uint32_t internal_release();

	inline uint32_t get_animation_skeleton_joint_count() const;
	inline BRX_MOTION_SKELETON_JOINT_NAME const *get_animation_skeleton_joint_names() const;
	inline uint32_t const *get_animation_skeleton_joint_parent_indices() const;
	inline brx_animation_rigid_transform const *get_animation_skeleton_bind_pose_local_space() const;
	inline DirectX::XMFLOAT4X4 const *get_animation_skeleton_bind_pose_inverse_model_space() const;

	inline uint32_t get_animation_skeleton_joint_index(BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name) const;

	inline uint32_t get_animation_skeleton_joint_constraint_count() const;
	inline BRX_MOTION_SKELETON_JOINT_CONSTRAINT_NAME const *get_animation_skeleton_joint_constraint_names() const;
	inline brx_animation_skeleton_joint_constraint const *get_animation_skeleton_joint_constraints() const;
	inline uint32_t get_animation_skeleton_joint_constraints_storage_size(uint32_t animation_skeleton_joint_constraint_index) const;
	inline uint32_t const *get_animation_skeleton_joint_constraints_storage_base(uint32_t animation_skeleton_joint_constraint_index) const;

	inline uint32_t get_ragdoll_skeleton_rigid_body_count() const;
	inline brx_animation_physics_rigid_body const *get_ragdoll_skeleton_rigid_bodies() const;

	inline uint32_t get_ragdoll_skeleton_constraint_count() const;
	inline brx_animation_physics_constraint const *get_ragdoll_skeleton_constraints() const;

	inline uint32_t get_animation_to_ragdoll_direct_mapping_count() const;
	inline brx_animation_ragdoll_direct_mapping const *get_animation_to_ragdoll_direct_mappings() const;

	inline uint32_t get_ragdoll_to_animation_direct_mapping_count() const;
	inline brx_animation_ragdoll_direct_mapping const *get_ragdoll_to_animation_direct_mappings() const;
};

class brx_motion_animation_skeleton_instance final : public brx_motion_skeleton_instance
{
	brx_motion_animation_skeleton *m_skeleton;

	mcrt_vector<brx_motion_rigid_transform> m_skin_transforms;

	brx_physics_world *m_physics_world;
	mcrt_vector<brx_physics_rigid_body *> m_physics_rigid_bodies;
	mcrt_vector<brx_physics_constraint *> m_physics_constraints;

	brx_motion_video_detector const *m_morph_input_video_detector;
	uint32_t m_morph_input_pose_index;
	uint32_t m_morph_input_face_index;
	uint32_t m_morph_input_hand_index;

	DirectX::XMFLOAT4 m_video_detector_face_morph_joint_rotations_local_space[INTERNAL_VIDEO_DETECTOR_FACE_MORPH_JOINT_NAME_COUNT];

	brx_motion_animation_animation_instance const *m_morph_input_animation_instance;

	brx_motion_video_detector const *m_joint_input_video_detector;
	uint32_t m_joint_input_pose_index;
	uint32_t m_joint_input_face_index;
	uint32_t m_joint_input_hand_index;

	bool m_video_detector_pose_skeleton_joint_translations_model_space_valid[INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_COUNT];
	DirectX::XMFLOAT3 m_video_detector_pose_skeleton_joint_translations_model_space[INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_COUNT];
	DirectX::XMFLOAT4 m_video_detector_pose_skeleton_joint_rotations_local_space[11];

	DirectX::XMFLOAT4 m_video_detector_face_skeleton_joint_rotations_local_space[INTERNAL_VIDEO_DETECTOR_FACE_SKELETON_JOINT_NAME_COUNT];

	bool m_video_detector_hand_skeleton_joint_translations_model_space_valid[INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_COUNT];
	DirectX::XMFLOAT3 m_video_detector_hand_skeleton_joint_translations_model_space[INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_COUNT];
	DirectX::XMFLOAT4 m_video_detector_hand_skeleton_joint_rotations_local_space[32];

	brx_motion_animation_animation_instance const *m_joint_input_animation_instance;

	// only joint input can affect ragdoll
	bool m_joint_input_continuous;

public:
	inline brx_motion_animation_skeleton_instance();
	inline ~brx_motion_animation_skeleton_instance();
	inline void init(brx_motion_animation_skeleton *skeleton);
	inline void uninit();

private:
	inline void invalidate_morph_input_video_detector();
	inline void invalidate_joint_input_video_detector();
	void set_morph_input_video_detector(brx_motion_video_detector const *video_detector) override;
	void set_morph_input_video_detector_pose_index(uint32_t pose_index) override;
	void set_morph_input_video_detector_face_index(uint32_t face_index) override;
	void set_morph_input_video_detector_hand_index(uint32_t hand_index) override;
	void set_morph_input_animation_instance(brx_motion_animation_instance const *animation_instance) override;
	void set_joint_input_video_detector(brx_motion_video_detector const *video_detector) override;
	void set_joint_input_video_detector_pose_index(uint32_t pose_index) override;
	void set_joint_input_video_detector_face_index(uint32_t face_index) override;
	void set_joint_input_video_detector_hand_index(uint32_t hand_index) override;
	void set_joint_input_animation_instance(brx_motion_animation_instance const *animation_instance) override;
	brx_motion_video_detector *get_morph_input_video_detector() const override;
	uint32_t get_morph_input_video_detector_pose_index() const override;
	uint32_t get_morph_input_video_detector_face_index() const override;
	uint32_t get_morph_input_video_detector_hand_index() const override;
	brx_motion_animation_instance *get_morph_input_animation_instance() const override;
	brx_motion_video_detector *get_joint_input_video_detector() const override;
	uint32_t get_joint_input_video_detector_pose_index() const override;
	uint32_t get_joint_input_video_detector_face_index() const override;
	uint32_t get_joint_input_video_detector_hand_index() const override;
	brx_motion_animation_instance *get_joint_input_animation_instance() const override;
	void step(BRX_MOTION_PHYSICS_RAGDOLL_QUALITY physics_ragdoll_quality) override;
	inline void animation_skeleton_joint_constraint(mcrt_vector<DirectX::XMFLOAT4X4> const &animation_skeleton_bind_pose_local_space, mcrt_vector<DirectX::XMFLOAT4X4> &animation_skeleton_animation_pose_local_space);
	inline void ragdoll(mcrt_vector<DirectX::XMFLOAT4X4> &animation_skeleton_animation_pose_model_space, float physics_delta_time, uint32_t physics_max_substep_count, float physics_substep_delta_time);
	uint32_t get_skin_transform_count() const override;
	brx_motion_rigid_transform const *get_skin_transforms() const override;
};

#endif
