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

#ifndef _BRX_MOTION_FORMAT_H_
#define _BRX_MOTION_FORMAT_H_ 1

#include <cstddef>
#include <cstdint>

#include "../../Brioche-Physics/include/brx_physics_format.h"

// https://github.com/vrm-c/vrm-specification/blob/master/specification/0.0/README.md#predefined-expression-name
// https://github.com/vrm-c/vrm-specification/blob/master/specification/VRMC_vrm-1.0/expressions.md#preset-expressions
enum BRX_MOTION_VRM_MORPH_TARGET_NAME
{
    BRX_MOTION_VRM_MORPH_TARGET_NAME_NEUTRAL = 0,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_A = 1,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_I = 2,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_U = 3,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_E = 4,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_O = 5,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_BLINK = 6,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_BLINK_L = 7,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_BLINK_R = 8,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_FUN = 9,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_ANGRY = 10,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_SORROW = 11,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_JOY = 12,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_SURPRISED = 13,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_LOOK_UP = 14,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_LOOK_DOWN = 15,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_LOOK_LEFT = 16,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_LOOK_RIGHT = 17,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_COUNT = 18
};

// https://developer.apple.com/documentation/arkit/arfaceanchor/blendshapelocation
static constexpr uint32_t const BRX_MOTION_ARKIT_MORPH_TARGET_NAME_COUNT = 52U;

// https://github.com/vrm-c/vrm-specification/blob/master/specification/0.0/README.md#defined-bones
// https://github.com/vrm-c/vrm-specification/blob/master/specification/VRMC_vrm-1.0/humanoid.md#list-of-humanoid-bones
// https://github.com/saturday06/VRM-Addon-for-Blender/blob/main/src/io_scene_vrm/common/human_bone_mapper/mmd_mapping.py
// https://docs.unity3d.com/ScriptReference/HumanBodyBones.html
// https://developer.apple.com/documentation/arkit/validating-a-model-for-motion-capture
// https://developer.apple.com/documentation/arkit/arskeleton/jointname
// https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker
// https://cmu-perceptual-computing-lab.github.io/openpose/web/html/doc/md_doc_02_output.html
enum BRX_MOTION_VRM_SKELETON_JOINT_NAME
{
    BRX_MOTION_VRM_SKELETON_JOINT_NAME_HIPS = 0,
    BRX_MOTION_VRM_SKELETON_JOINT_NAME_COUNT = 55
};

enum BRX_MOTION_JOINT_CONSTRAINT_TYPE : uint32_t
{
    BRX_JOINT_CONSTRAINT_COPY_TRANSFORM = 0,
    BRX_JOINT_CONSTRAINT_INVERSE_KINEMATICS = 1
};

static constexpr uint32_t const BRX_MOTION_UINT32_INDEX_INVALID = static_cast<uint32_t>(~static_cast<uint32_t>(0U));

struct brx_motion_rigid_transform
{
    float m_rotation[4];
    float m_translation[3];
};

struct brx_motion_joint_constraint
{
    BRX_MOTION_JOINT_CONSTRAINT_TYPE m_constraint_type;

    union
    {
        struct
        {
            uint32_t m_source_joint_index;
            uint32_t m_source_weight_count;
            float *m_source_weights;
            uint32_t m_destination_joint_index;
            bool m_copy_rotation;
            bool m_copy_translation;
        } m_copy_transform;

        struct
        {
            uint32_t m_ik_end_effector_index;
            uint32_t m_ik_joint_count;
            uint32_t *m_ik_joint_indices;
            uint32_t m_target_joint_index;
            float m_ik_two_joints_hinge_joint_axis_local_space[3];
            float m_cosine_max_ik_two_joints_hinge_joint_angle;
            float m_cosine_min_ik_two_joints_hinge_joint_angle;
        } m_inverse_kinematics;
    };
};

struct brx_motion_ragdoll_rigid_body
{
    brx_motion_rigid_transform m_model_space_transform;
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

struct brx_motion_ragdoll_constraint
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

struct brx_motion_ragdoll_direct_mapping
{
    uint32_t m_joint_index_a;
    uint32_t m_joint_index_b;
    float m_a_to_b_transform_model_space;
};

#endif
