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

#include "brx_motion_animation.h"
#include "brx_animation_ik_one_joint.h"
#include "brx_animation_ik_two_joints.h"
#include "brx_animation_ik_ccd.h"
#include "brx_motion_media_pipe_video_detector.h"
#include "../../McRT-Malloc/include/mcrt_malloc.h"
#include <cassert>
#include <new>

static brx_physics_context *g_physics_context = NULL;
#ifndef NDEBUG
static int32_t g_physics_context_ref_count = 0;
#endif
class _internal_physics_context_uninit
{
public:
    inline ~_internal_physics_context_uninit()
    {
        assert(0 == g_physics_context_ref_count);

        if (NULL != g_physics_context)
        {
            brx_physics_destroy_context(g_physics_context);
            g_physics_context = NULL;
        }
    }
};
static _internal_physics_context_uninit _internal_physics_context_uninit_instance;

static inline BRX_PHYSICS_RIGID_BODY_SHAPE_TYPE wrap(BRX_MOTION_PHYSICS_RIGID_BODY_SHAPE_TYPE shape_type)
{
    static_assert(static_cast<uint32_t>(BRX_PHYSICS_RIGID_BODY_SHAPE_SPHERE) == static_cast<uint32_t>(BRX_MOTION_PHYSICS_RIGID_BODY_SHAPE_SPHERE), "");
    static_assert(static_cast<uint32_t>(BRX_PHYSICS_RIGID_BODY_SHAPE_BOX) == static_cast<uint32_t>(BRX_MOTION_PHYSICS_RIGID_BODY_SHAPE_BOX), "");
    static_assert(static_cast<uint32_t>(BRX_PHYSICS_RIGID_BODY_SHAPE_CAPSULE) == static_cast<uint32_t>(BRX_MOTION_PHYSICS_RIGID_BODY_SHAPE_CAPSULE), "");
    assert(BRX_MOTION_PHYSICS_RIGID_BODY_SHAPE_SPHERE == shape_type || BRX_MOTION_PHYSICS_RIGID_BODY_SHAPE_BOX == shape_type || BRX_MOTION_PHYSICS_RIGID_BODY_SHAPE_CAPSULE == shape_type);
    return static_cast<BRX_PHYSICS_RIGID_BODY_SHAPE_TYPE>(shape_type);
}

static inline BRX_PHYSICS_RIGID_BODY_MOTION_TYPE wrap(BRX_MOTION_PHYSICS_RIGID_BODY_MOTION_TYPE motion_type)
{
    static_assert(static_cast<uint32_t>(BRX_PHYSICS_RIGID_BODY_MOTION_FIXED) == static_cast<uint32_t>(BRX_MOTION_PHYSICS_RIGID_BODY_MOTION_FIXED), "");
    static_assert(static_cast<uint32_t>(BRX_PHYSICS_RIGID_BODY_MOTION_KEYFRAME) == static_cast<uint32_t>(BRX_MOTION_PHYSICS_RIGID_BODY_MOTION_KEYFRAME), "");
    static_assert(static_cast<uint32_t>(BRX_PHYSICS_RIGID_BODY_MOTION_DYNAMIC) == static_cast<uint32_t>(BRX_MOTION_PHYSICS_RIGID_BODY_MOTION_DYNAMIC), "");
    assert(BRX_MOTION_PHYSICS_RIGID_BODY_MOTION_FIXED == motion_type || BRX_MOTION_PHYSICS_RIGID_BODY_MOTION_KEYFRAME == motion_type || BRX_MOTION_PHYSICS_RIGID_BODY_MOTION_DYNAMIC == motion_type);
    return static_cast<BRX_PHYSICS_RIGID_BODY_MOTION_TYPE>(motion_type);
}

static inline BRX_PHYSICS_CONSTRAINT_TYPE wrap(BRX_MOTION_PHYSICS_CONSTRAINT_TYPE constraint_type)
{
    static_assert(static_cast<uint32_t>(BRX_PHYSICS_CONSTRAINT_FIXED) == static_cast<uint32_t>(BRX_MOTION_PHYSICS_CONSTRAINT_FIXED), "");
    static_assert(static_cast<uint32_t>(BRX_PHYSICS_CONSTRAINT_BALL_AND_SOCKET) == static_cast<uint32_t>(BRX_MOTION_PHYSICS_CONSTRAINT_BALL_AND_SOCKET), "");
    static_assert(static_cast<uint32_t>(BRX_PHYSICS_CONSTRAINT_HINGE) == static_cast<uint32_t>(BRX_MOTION_PHYSICS_CONSTRAINT_HINGE), "");
    static_assert(static_cast<uint32_t>(BRX_PHYSICS_CONSTRAINT_PRISMATIC) == static_cast<uint32_t>(BRX_MOTION_PHYSICS_CONSTRAINT_PRISMATIC), "");
    static_assert(static_cast<uint32_t>(BRX_PHYSICS_CONSTRAINT_RAGDOLL) == static_cast<uint32_t>(BRX_MOTION_PHYSICS_CONSTRAINT_RAGDOLL), "");
    assert(BRX_MOTION_PHYSICS_CONSTRAINT_FIXED == constraint_type || BRX_MOTION_PHYSICS_CONSTRAINT_BALL_AND_SOCKET == constraint_type || BRX_MOTION_PHYSICS_CONSTRAINT_HINGE == constraint_type || BRX_MOTION_PHYSICS_CONSTRAINT_PRISMATIC == constraint_type || BRX_MOTION_PHYSICS_CONSTRAINT_RAGDOLL == constraint_type);
    return static_cast<BRX_PHYSICS_CONSTRAINT_TYPE>(constraint_type);
}

static constexpr float const INTERNAL_SCALE_EPSILON = 1E-4F;

static constexpr float const INTERNAL_ROTATION_EPSILON = 1E-6F;

static constexpr float const INTERNAL_TRANSLATION_EPSILON = 1E-3F;

static constexpr float const INTERNAL_FACE_SKELETON_JOINT_ROTATION_BLEND_FACTOR = (1.0F - 0.25F);

static constexpr float const INTERNAL_POSE_SKELETON_JOINT_TRANSLATION_BLEND_FACTOR = (1.0F - 0.25F);

static constexpr float const INTERNAL_POSE_SKELETON_JOINT_ROTATION_BLEND_FACTOR = (1.0F - 0.25F);

// [setupFps](https://github.com/MMD-Blender/blender_mmd_tools/blob/main/mmd_tools/auto_scene_setup.py#L27)
static constexpr float const INTERNAL_FRAME_DELTA_TIME = 1.0F / 30.0F;
//
static constexpr uint32_t const INTERNAL_FRAME_MAXIMUM_SUBSTEP_COUNT = 6U;
// [VMDAnimation::SyncPhysics](https://github.com/benikabocha/saba/blob/master/src/Saba/Model/MMD/VMDAnimation.cpp#L374)
static constexpr float const INTERNAL_FRAME_SUBSTEP_DELTA_TIME = 1.0F / 60.0F;

static constexpr float const INTERNAL_PHYSICS_SYNCHRONIZATION_DELTA_TIME = 1.0F;
static constexpr uint32_t const INTERNAL_PHYSICS_SYNCHRONIZATION_MAXIMUM_SUBSTEP_COUNT = 90U;
static constexpr float const INTERNAL_PHYSICS_SYNCHRONIZATION_SUBSTEP_DELTA_TIME = 1.0F / 90.0F;

static inline void internal_video_detector_pose_ik_one_joint_solve(uint32_t const in_ball_and_socket_joint_animation_skeleton_joint_index, uint32_t const in_end_effector_animation_skeleton_joint_index, DirectX::XMFLOAT3 const &in_target_displacement_model_space, uint32_t const *const in_animation_skeleton_joint_parent_indices, DirectX::XMFLOAT4X4 const *const in_animation_skeleton_animation_pose_local_space, DirectX::XMFLOAT4 *const out_ball_and_socket_joint_rotation_local_space);

static inline DirectX::XMFLOAT4X4 internal_calculate_transform_model_space(uint32_t const *const in_animation_skeleton_joint_parent_indices, DirectX::XMFLOAT4X4 const *const in_animation_skeleton_local_space, uint32_t const in_animation_skeleton_joint_index);

extern "C" brx_motion_animation *brx_motion_create_animation(uint32_t frame_count, uint32_t weight_channel_count, BRX_MOTION_MORPH_TARGET_NAME const *wrapped_weight_channel_names, float const *wrapped_weights, uint32_t rigid_transform_channel_count, BRX_MOTION_SKELETON_JOINT_NAME const *wrapped_rigid_transform_channel_names, brx_motion_rigid_transform const *wrapped_rigid_transforms, uint32_t switch_channel_count, BRX_MOTION_SKELETON_JOINT_CONSTRAINT_NAME const *wrapped_switch_channel_names, uint8_t const *wrapped_switches)
{
    mcrt_unordered_map<BRX_MOTION_MORPH_TARGET_NAME, uint32_t> weight_channel_indices;
    mcrt_vector<float> weights(static_cast<size_t>(static_cast<size_t>(weight_channel_count) * static_cast<size_t>(frame_count)));
    {
        for (uint32_t weight_channel_index = 0U; weight_channel_index < weight_channel_count; ++weight_channel_index)
        {
            auto const found_weight_channel_index = weight_channel_indices.find(wrapped_weight_channel_names[weight_channel_index]);
            assert(weight_channel_indices.end() == found_weight_channel_index);
            weight_channel_indices.emplace_hint(found_weight_channel_index, wrapped_weight_channel_names[weight_channel_index], weight_channel_index);
        }

        for (uint32_t frame_index = 0U; frame_index < frame_count; ++frame_index)
        {
            for (uint32_t weight_channel_index = 0U; weight_channel_index < weight_channel_count; ++weight_channel_index)
            {
                weights[weight_channel_count * frame_index + weight_channel_index] = wrapped_weights[weight_channel_count * frame_index + weight_channel_index];
            }
        }
    }

    mcrt_unordered_map<BRX_MOTION_SKELETON_JOINT_NAME, uint32_t> rigid_transform_channel_indices;
    mcrt_vector<brx_animation_rigid_transform> rigid_transforms(static_cast<size_t>(static_cast<size_t>(rigid_transform_channel_count) * static_cast<size_t>(frame_count)));
    {
        for (uint32_t rigid_transform_channel_index = 0U; rigid_transform_channel_index < rigid_transform_channel_count; ++rigid_transform_channel_index)
        {
            auto const found_rigid_transform_channel_index = rigid_transform_channel_indices.find(wrapped_rigid_transform_channel_names[rigid_transform_channel_index]);
            assert(rigid_transform_channel_indices.end() == found_rigid_transform_channel_index);
            rigid_transform_channel_indices.emplace_hint(found_rigid_transform_channel_index, wrapped_rigid_transform_channel_names[rigid_transform_channel_index], rigid_transform_channel_index);
        }

        for (uint32_t frame_index = 0U; frame_index < frame_count; ++frame_index)
        {
            for (uint32_t rigid_transform_channel_index = 0U; rigid_transform_channel_index < rigid_transform_channel_count; ++rigid_transform_channel_index)
            {
                rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_rotation = DirectX::XMFLOAT4(wrapped_rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_rotation[0], wrapped_rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_rotation[1], wrapped_rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_rotation[2], wrapped_rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_rotation[3]);

                rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_translation = DirectX::XMFLOAT3(wrapped_rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_translation[0], wrapped_rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_translation[1], wrapped_rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_translation[2]);
            }
        }
    }

    mcrt_unordered_map<BRX_MOTION_SKELETON_JOINT_CONSTRAINT_NAME, uint32_t> switch_channel_indices;
    mcrt_vector<uint8_t> switches(static_cast<size_t>(static_cast<size_t>(switch_channel_count) * static_cast<size_t>(frame_count)));
    {
        for (uint32_t switch_channel_index = 0U; switch_channel_index < switch_channel_count; ++switch_channel_index)
        {
            auto const found_switch_channel_index = switch_channel_indices.find(wrapped_switch_channel_names[switch_channel_index]);
            assert(switch_channel_indices.end() == found_switch_channel_index);
            switch_channel_indices.emplace_hint(found_switch_channel_index, wrapped_switch_channel_names[switch_channel_index], switch_channel_index);
        }

        for (uint32_t frame_index = 0U; frame_index < frame_count; ++frame_index)
        {
            for (uint32_t switch_channel_index = 0U; switch_channel_index < switch_channel_count; ++switch_channel_index)
            {
                switches[switch_channel_count * frame_index + switch_channel_index] = wrapped_switches[switch_channel_count * frame_index + switch_channel_index];
            }
        }
    }

    void *new_unwrapped_animation_base = mcrt_malloc(sizeof(brx_motion_animation_animation), alignof(brx_motion_animation_animation));
    assert(NULL != new_unwrapped_animation_base);

    brx_motion_animation_animation *new_unwrapped_animation = new (new_unwrapped_animation_base) brx_motion_animation_animation{std::move(weight_channel_indices), std::move(weights), std::move(rigid_transform_channel_indices), std::move(rigid_transforms), std::move(switch_channel_indices), std::move(switches)};

    return new_unwrapped_animation;
}

static inline void release_animation(brx_motion_animation_animation *release_unwrapped_animation)
{
    if (0U == release_unwrapped_animation->internal_release())
    {
        brx_motion_animation_animation *delete_unwrapped_animation = release_unwrapped_animation;

        delete_unwrapped_animation->~brx_motion_animation_animation();
        mcrt_free(delete_unwrapped_animation);
    }
}

extern "C" void brx_motion_destroy_animation(brx_motion_animation *wrapped_animation)
{
    assert(NULL != wrapped_animation);
    brx_motion_animation_animation *release_unwrapped_animation = static_cast<brx_motion_animation_animation *>(wrapped_animation);

    release_animation(release_unwrapped_animation);
}

extern "C" brx_motion_animation_instance *brx_motion_create_animation_instance(brx_motion_animation *wrapped_animation)
{
    void *new_unwrapped_animation_instance_base = mcrt_malloc(sizeof(brx_motion_animation_animation_instance), alignof(brx_motion_animation_animation_instance));
    assert(NULL != new_unwrapped_animation_instance_base);

    brx_motion_animation_animation_instance *new_unwrapped_animation_instance = new (new_unwrapped_animation_instance_base) brx_motion_animation_animation_instance{};

    new_unwrapped_animation_instance->init(static_cast<brx_motion_animation_animation *>(wrapped_animation));

    return new_unwrapped_animation_instance;
}

static inline void release_animation_instance(brx_motion_animation_animation_instance *release_unwrapped_animation_instance)
{
    if (0U == release_unwrapped_animation_instance->internal_release())
    {
        brx_motion_animation_animation_instance *delete_unwrapped_animation_instance = release_unwrapped_animation_instance;

        delete_unwrapped_animation_instance->uninit();

        delete_unwrapped_animation_instance->~brx_motion_animation_animation_instance();
        mcrt_free(delete_unwrapped_animation_instance);
    }
}

extern "C" void brx_motion_destroy_animation_instance(brx_motion_animation_instance *wrapped_animation_instance)
{
    assert(NULL != wrapped_animation_instance);
    brx_motion_animation_animation_instance *release_unwrapped_animation_instance = static_cast<brx_motion_animation_animation_instance *>(wrapped_animation_instance);

    release_animation_instance(release_unwrapped_animation_instance);
}

extern "C" brx_motion_skeleton *brx_motion_create_skeleton(uint32_t animation_skeleton_joint_count, BRX_MOTION_SKELETON_JOINT_NAME const *wrapped_animation_skeleton_joint_names, uint32_t const *wrapped_animation_skeleton_joint_parent_indices, brx_motion_rigid_transform const *wrapped_animation_skeleton_bind_pose_local_space, uint32_t animation_skeleton_joint_constraint_count, BRX_MOTION_SKELETON_JOINT_CONSTRAINT_NAME const *wrapped_animation_skeleton_joint_constraint_names, brx_motion_skeleton_joint_constraint const *wrapped_animation_skeleton_joint_constraints, uint32_t ragdoll_skeleton_rigid_body_count, brx_motion_physics_rigid_body const *wrapped_ragdoll_skeleton_rigid_bodies, uint32_t ragdoll_skeleton_constraint_count, brx_motion_physics_constraint const *wrapped_ragdoll_skeleton_constraints, uint32_t animation_to_ragdoll_direct_mapping_count, brx_motion_ragdoll_direct_mapping const *wrapped_animation_to_ragdoll_direct_mappings, uint32_t ragdoll_to_animation_direct_mapping_count, brx_motion_ragdoll_direct_mapping const *wrapped_ragdoll_to_animation_direct_mappings)
{
    mcrt_vector<BRX_MOTION_SKELETON_JOINT_NAME> animation_skeleton_joint_names(static_cast<size_t>(animation_skeleton_joint_count));
    mcrt_vector<uint32_t> animation_skeleton_joint_parent_indices(static_cast<size_t>(animation_skeleton_joint_count));
    mcrt_vector<brx_animation_rigid_transform> animation_skeleton_bind_pose_local_space(static_cast<size_t>(animation_skeleton_joint_count));
    mcrt_vector<DirectX::XMFLOAT4X4> animation_skeleton_bind_pose_inverse_model_space(static_cast<size_t>(animation_skeleton_joint_count));
    {
        mcrt_vector<DirectX::XMFLOAT4X4> animation_skeleton_bind_pose_model_space(static_cast<size_t>(animation_skeleton_joint_count));

        for (uint32_t animation_skeleton_joint_index = 0U; animation_skeleton_joint_index < animation_skeleton_joint_count; ++animation_skeleton_joint_index)
        {
            animation_skeleton_joint_names[animation_skeleton_joint_index] = wrapped_animation_skeleton_joint_names[animation_skeleton_joint_index];

            animation_skeleton_joint_parent_indices[animation_skeleton_joint_index] = wrapped_animation_skeleton_joint_parent_indices[animation_skeleton_joint_index];

            animation_skeleton_bind_pose_local_space[animation_skeleton_joint_index].m_rotation = DirectX::XMFLOAT4{wrapped_animation_skeleton_bind_pose_local_space[animation_skeleton_joint_index].m_rotation[0], wrapped_animation_skeleton_bind_pose_local_space[animation_skeleton_joint_index].m_rotation[1], wrapped_animation_skeleton_bind_pose_local_space[animation_skeleton_joint_index].m_rotation[2], wrapped_animation_skeleton_bind_pose_local_space[animation_skeleton_joint_index].m_rotation[3]};

            animation_skeleton_bind_pose_local_space[animation_skeleton_joint_index].m_translation = DirectX::XMFLOAT3{wrapped_animation_skeleton_bind_pose_local_space[animation_skeleton_joint_index].m_translation[0], wrapped_animation_skeleton_bind_pose_local_space[animation_skeleton_joint_index].m_translation[1], wrapped_animation_skeleton_bind_pose_local_space[animation_skeleton_joint_index].m_translation[2]};

            DirectX::XMMATRIX current_animation_skeleton_bind_pose_local_space_matrix = DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(DirectX::XMLoadFloat4(&animation_skeleton_bind_pose_local_space[animation_skeleton_joint_index].m_rotation)), DirectX::XMMatrixTranslationFromVector(DirectX::XMLoadFloat3(&animation_skeleton_bind_pose_local_space[animation_skeleton_joint_index].m_translation)));

            if (BRX_MOTION_UINT32_INDEX_INVALID != animation_skeleton_joint_parent_indices[animation_skeleton_joint_index])
            {
                assert(animation_skeleton_joint_parent_indices[animation_skeleton_joint_index] < animation_skeleton_joint_index);

                DirectX::XMStoreFloat4x4(&animation_skeleton_bind_pose_model_space[animation_skeleton_joint_index], DirectX::XMMatrixMultiply(current_animation_skeleton_bind_pose_local_space_matrix, DirectX::XMLoadFloat4x4(&animation_skeleton_bind_pose_model_space[animation_skeleton_joint_parent_indices[animation_skeleton_joint_index]])));
            }
            else
            {
                DirectX::XMStoreFloat4x4(&animation_skeleton_bind_pose_model_space[animation_skeleton_joint_index], current_animation_skeleton_bind_pose_local_space_matrix);
            }

            {
                DirectX::XMVECTOR unused_determinant;
                DirectX::XMStoreFloat4x4(&animation_skeleton_bind_pose_inverse_model_space[animation_skeleton_joint_index], DirectX::XMMatrixInverse(&unused_determinant, DirectX::XMLoadFloat4x4(&animation_skeleton_bind_pose_model_space[animation_skeleton_joint_index])));
            }
        }
    }

    mcrt_vector<BRX_MOTION_SKELETON_JOINT_CONSTRAINT_NAME> animation_skeleton_joint_constraint_names(static_cast<size_t>(animation_skeleton_joint_constraint_count));
    mcrt_vector<brx_animation_skeleton_joint_constraint> animation_skeleton_joint_constraints(static_cast<size_t>(animation_skeleton_joint_constraint_count));
    mcrt_vector<mcrt_vector<uint32_t>> animation_skeleton_joint_constraints_storages(static_cast<size_t>(animation_skeleton_joint_constraint_count));
    {
        for (uint32_t animation_skeleton_joint_constraint_index = 0U; animation_skeleton_joint_constraint_index < animation_skeleton_joint_constraint_count; ++animation_skeleton_joint_constraint_index)
        {
            animation_skeleton_joint_constraint_names[animation_skeleton_joint_constraint_index] = wrapped_animation_skeleton_joint_constraint_names[animation_skeleton_joint_constraint_index];

            animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_constraint_type = wrapped_animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_constraint_type;

            if (BRX_MOTION_SKELETON_JOINT_CONSTRAINT_COPY_TRANSFORM == animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_constraint_type)
            {
                animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_copy_transform.m_source_joint_index = wrapped_animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_copy_transform.m_source_joint_index;

                uint32_t const source_weight_count = wrapped_animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_copy_transform.m_source_weight_count;

                animation_skeleton_joint_constraints_storages[animation_skeleton_joint_constraint_index].resize(source_weight_count);

                float *const source_weights = reinterpret_cast<float *>(animation_skeleton_joint_constraints_storages[animation_skeleton_joint_constraint_index].data());

                for (uint32_t source_weight_index = 0U; source_weight_index < source_weight_count; ++source_weight_index)
                {
                    source_weights[source_weight_index] = wrapped_animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_copy_transform.m_source_weights[source_weight_index];
                }

                animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_copy_transform.m_destination_joint_index = wrapped_animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_copy_transform.m_destination_joint_index;

                animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_copy_transform.m_copy_rotation = wrapped_animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_copy_transform.m_copy_rotation;

                animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_copy_transform.m_copy_translation = wrapped_animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_copy_transform.m_copy_translation;
            }
            else
            {
                assert(BRX_MOTION_SKELETON_JOINT_CONSTRAINT_INVERSE_KINEMATICS == animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_constraint_type);

                animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_inverse_kinematics.m_ik_end_effector_index = wrapped_animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_inverse_kinematics.m_ik_end_effector_index;

                uint32_t const ik_joint_count = wrapped_animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_inverse_kinematics.m_ik_joint_count;

                animation_skeleton_joint_constraints_storages[animation_skeleton_joint_constraint_index].resize(ik_joint_count);

                uint32_t *const ik_joint_indices = animation_skeleton_joint_constraints_storages[animation_skeleton_joint_constraint_index].data();

                for (uint32_t ik_joint_index = 0U; ik_joint_index < ik_joint_count; ++ik_joint_index)
                {
                    ik_joint_indices[ik_joint_index] = wrapped_animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_inverse_kinematics.m_ik_joint_indices[ik_joint_index];
                }

                animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_inverse_kinematics.m_target_joint_index = wrapped_animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_inverse_kinematics.m_target_joint_index;

                animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_inverse_kinematics.m_ik_two_joints_hinge_joint_axis_local_space.x = wrapped_animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_inverse_kinematics.m_ik_two_joints_hinge_joint_axis_local_space[0];

                animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_inverse_kinematics.m_ik_two_joints_hinge_joint_axis_local_space.y = wrapped_animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_inverse_kinematics.m_ik_two_joints_hinge_joint_axis_local_space[1];

                animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_inverse_kinematics.m_ik_two_joints_hinge_joint_axis_local_space.z = wrapped_animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_inverse_kinematics.m_ik_two_joints_hinge_joint_axis_local_space[2];

                animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_inverse_kinematics.m_cosine_max_ik_two_joints_hinge_joint_angle = wrapped_animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_inverse_kinematics.m_cosine_max_ik_two_joints_hinge_joint_angle;

                animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_inverse_kinematics.m_cosine_min_ik_two_joints_hinge_joint_angle = wrapped_animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_inverse_kinematics.m_cosine_min_ik_two_joints_hinge_joint_angle;
            }
        }
    }

    mcrt_vector<brx_animation_physics_rigid_body> ragdoll_skeleton_rigid_bodies(static_cast<size_t>(ragdoll_skeleton_rigid_body_count));
    {
        for (uint32_t ragdoll_skeleton_rigid_body_index = 0U; ragdoll_skeleton_rigid_body_index < ragdoll_skeleton_rigid_body_count; ++ragdoll_skeleton_rigid_body_index)
        {
            ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_model_space_rotation[0] = wrapped_ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_model_space_transform.m_rotation[0];
            ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_model_space_rotation[1] = wrapped_ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_model_space_transform.m_rotation[1];
            ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_model_space_rotation[2] = wrapped_ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_model_space_transform.m_rotation[2];
            ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_model_space_rotation[3] = wrapped_ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_model_space_transform.m_rotation[3];

            ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_model_space_translation[0] = wrapped_ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_model_space_transform.m_translation[0];
            ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_model_space_translation[1] = wrapped_ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_model_space_transform.m_translation[1];
            ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_model_space_translation[2] = wrapped_ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_model_space_transform.m_translation[2];

            ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_shape_type = wrap(wrapped_ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_shape_type);

            ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_shape_size[0] = wrapped_ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_shape_size[0];
            ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_shape_size[1] = wrapped_ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_shape_size[1];
            ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_shape_size[2] = wrapped_ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_shape_size[2];

            ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_motion_type = wrap(wrapped_ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_motion_type);

            ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_collision_filter_group = wrapped_ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_collision_filter_group;

            ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_collision_filter_mask = wrapped_ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_collision_filter_mask;

            ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_mass = wrapped_ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_mass;

            ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_linear_damping = wrapped_ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_linear_damping;

            ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_angular_damping = wrapped_ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_angular_damping;

            ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_friction = wrapped_ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_friction;

            ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_restitution = wrapped_ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index].m_restitution;
        }
    }

    mcrt_vector<brx_animation_physics_constraint> ragdoll_skeleton_constraints(static_cast<size_t>(ragdoll_skeleton_constraint_count));
    {
        for (uint32_t ragdoll_skeleton_constraint_index = 0U; ragdoll_skeleton_constraint_index < ragdoll_skeleton_constraint_count; ++ragdoll_skeleton_constraint_index)
        {
            ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_rigid_body_a_index = wrapped_ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_rigid_body_a_index;

            ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_rigid_body_b_index = wrapped_ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_rigid_body_b_index;

            ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_constraint_type = wrap(wrapped_ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_constraint_type);

            ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_pivot[0] = wrapped_ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_pivot[0];
            ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_pivot[1] = wrapped_ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_pivot[1];
            ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_pivot[2] = wrapped_ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_pivot[2];

            ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_twist_axis[0] = wrapped_ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_twist_axis[0];
            ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_twist_axis[1] = wrapped_ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_twist_axis[1];
            ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_twist_axis[2] = wrapped_ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_twist_axis[2];

            ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_plane_axis[0] = wrapped_ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_plane_axis[0];
            ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_plane_axis[1] = wrapped_ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_plane_axis[1];
            ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_plane_axis[2] = wrapped_ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_plane_axis[2];

            ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_normal_axis[0] = wrapped_ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_normal_axis[0];
            ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_normal_axis[1] = wrapped_ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_normal_axis[1];
            ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_normal_axis[2] = wrapped_ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_normal_axis[2];

            ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_twist_limit[0] = wrapped_ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_twist_limit[0];
            ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_twist_limit[1] = wrapped_ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_twist_limit[1];

            ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_plane_limit[0] = wrapped_ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_plane_limit[0];
            ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_plane_limit[1] = wrapped_ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_plane_limit[1];

            ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_normal_limit[0] = wrapped_ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_normal_limit[0];
            ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_normal_limit[1] = wrapped_ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index].m_normal_limit[1];
        }
    }

    mcrt_vector<brx_animation_ragdoll_direct_mapping> animation_to_ragdoll_direct_mapping(static_cast<size_t>(animation_to_ragdoll_direct_mapping_count));
    {
        for (uint32_t animation_to_ragdoll_direct_mapping_index = 0U; animation_to_ragdoll_direct_mapping_index < animation_to_ragdoll_direct_mapping_count; ++animation_to_ragdoll_direct_mapping_index)
        {
            animation_to_ragdoll_direct_mapping[animation_to_ragdoll_direct_mapping_index].m_joint_index_a = wrapped_animation_to_ragdoll_direct_mappings[animation_to_ragdoll_direct_mapping_index].m_joint_index_a;

            animation_to_ragdoll_direct_mapping[animation_to_ragdoll_direct_mapping_index].m_joint_index_b = wrapped_animation_to_ragdoll_direct_mappings[animation_to_ragdoll_direct_mapping_index].m_joint_index_b;

            animation_to_ragdoll_direct_mapping[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space.m[0][0] = wrapped_animation_to_ragdoll_direct_mappings[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space[0][0];
            animation_to_ragdoll_direct_mapping[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space.m[0][1] = wrapped_animation_to_ragdoll_direct_mappings[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space[0][1];
            animation_to_ragdoll_direct_mapping[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space.m[0][2] = wrapped_animation_to_ragdoll_direct_mappings[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space[0][2];
            animation_to_ragdoll_direct_mapping[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space.m[0][3] = wrapped_animation_to_ragdoll_direct_mappings[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space[0][3];

            animation_to_ragdoll_direct_mapping[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space.m[1][0] = wrapped_animation_to_ragdoll_direct_mappings[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space[1][0];
            animation_to_ragdoll_direct_mapping[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space.m[1][1] = wrapped_animation_to_ragdoll_direct_mappings[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space[1][1];
            animation_to_ragdoll_direct_mapping[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space.m[1][2] = wrapped_animation_to_ragdoll_direct_mappings[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space[1][2];
            animation_to_ragdoll_direct_mapping[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space.m[1][3] = wrapped_animation_to_ragdoll_direct_mappings[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space[1][3];

            animation_to_ragdoll_direct_mapping[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space.m[2][0] = wrapped_animation_to_ragdoll_direct_mappings[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space[2][0];
            animation_to_ragdoll_direct_mapping[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space.m[2][1] = wrapped_animation_to_ragdoll_direct_mappings[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space[2][1];
            animation_to_ragdoll_direct_mapping[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space.m[2][2] = wrapped_animation_to_ragdoll_direct_mappings[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space[2][2];
            animation_to_ragdoll_direct_mapping[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space.m[2][3] = wrapped_animation_to_ragdoll_direct_mappings[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space[2][3];

            animation_to_ragdoll_direct_mapping[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space.m[3][0] = wrapped_animation_to_ragdoll_direct_mappings[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space[3][0];
            animation_to_ragdoll_direct_mapping[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space.m[3][1] = wrapped_animation_to_ragdoll_direct_mappings[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space[3][1];
            animation_to_ragdoll_direct_mapping[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space.m[3][2] = wrapped_animation_to_ragdoll_direct_mappings[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space[3][2];
            assert(1.0F == wrapped_animation_to_ragdoll_direct_mappings[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space[3][3]);
            animation_to_ragdoll_direct_mapping[animation_to_ragdoll_direct_mapping_index].m_a_to_b_transform_model_space.m[3][3] = 1.0F;
        }
    }

    mcrt_vector<brx_animation_ragdoll_direct_mapping> ragdoll_to_animation_direct_mapping(static_cast<size_t>(ragdoll_to_animation_direct_mapping_count));
    {
        for (uint32_t ragdoll_to_animation_direct_mapping_index = 0U; ragdoll_to_animation_direct_mapping_index < ragdoll_to_animation_direct_mapping_count; ++ragdoll_to_animation_direct_mapping_index)
        {
            ragdoll_to_animation_direct_mapping[ragdoll_to_animation_direct_mapping_index].m_joint_index_a = wrapped_ragdoll_to_animation_direct_mappings[ragdoll_to_animation_direct_mapping_index].m_joint_index_a;

            ragdoll_to_animation_direct_mapping[ragdoll_to_animation_direct_mapping_index].m_joint_index_b = wrapped_ragdoll_to_animation_direct_mappings[ragdoll_to_animation_direct_mapping_index].m_joint_index_b;

            ragdoll_to_animation_direct_mapping[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space.m[0][0] = wrapped_ragdoll_to_animation_direct_mappings[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space[0][0];
            ragdoll_to_animation_direct_mapping[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space.m[0][1] = wrapped_ragdoll_to_animation_direct_mappings[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space[0][1];
            ragdoll_to_animation_direct_mapping[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space.m[0][2] = wrapped_ragdoll_to_animation_direct_mappings[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space[0][2];
            ragdoll_to_animation_direct_mapping[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space.m[0][3] = wrapped_ragdoll_to_animation_direct_mappings[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space[0][3];

            ragdoll_to_animation_direct_mapping[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space.m[1][0] = wrapped_ragdoll_to_animation_direct_mappings[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space[1][0];
            ragdoll_to_animation_direct_mapping[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space.m[1][1] = wrapped_ragdoll_to_animation_direct_mappings[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space[1][1];
            ragdoll_to_animation_direct_mapping[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space.m[1][2] = wrapped_ragdoll_to_animation_direct_mappings[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space[1][2];
            ragdoll_to_animation_direct_mapping[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space.m[1][3] = wrapped_ragdoll_to_animation_direct_mappings[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space[1][3];

            ragdoll_to_animation_direct_mapping[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space.m[2][0] = wrapped_ragdoll_to_animation_direct_mappings[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space[2][0];
            ragdoll_to_animation_direct_mapping[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space.m[2][1] = wrapped_ragdoll_to_animation_direct_mappings[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space[2][1];
            ragdoll_to_animation_direct_mapping[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space.m[2][2] = wrapped_ragdoll_to_animation_direct_mappings[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space[2][2];
            ragdoll_to_animation_direct_mapping[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space.m[2][3] = wrapped_ragdoll_to_animation_direct_mappings[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space[2][3];

            ragdoll_to_animation_direct_mapping[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space.m[3][0] = wrapped_ragdoll_to_animation_direct_mappings[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space[3][0];
            ragdoll_to_animation_direct_mapping[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space.m[3][1] = wrapped_ragdoll_to_animation_direct_mappings[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space[3][1];
            ragdoll_to_animation_direct_mapping[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space.m[3][2] = wrapped_ragdoll_to_animation_direct_mappings[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space[3][2];
            assert(1.0F == wrapped_ragdoll_to_animation_direct_mappings[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space[3][3]);
            ragdoll_to_animation_direct_mapping[ragdoll_to_animation_direct_mapping_index].m_a_to_b_transform_model_space.m[3][3] = 1.0F;
        }
    }

    void *new_unwrapped_skeleton_base = mcrt_malloc(sizeof(brx_motion_animation_skeleton), alignof(brx_motion_animation_skeleton));
    assert(NULL != new_unwrapped_skeleton_base);

    brx_motion_animation_skeleton *new_unwrapped_skeleton = new (new_unwrapped_skeleton_base) brx_motion_animation_skeleton{std::move(animation_skeleton_joint_names), std::move(animation_skeleton_joint_parent_indices), std::move(animation_skeleton_bind_pose_local_space), std::move(animation_skeleton_bind_pose_inverse_model_space), std::move(animation_skeleton_joint_constraint_names), std::move(animation_skeleton_joint_constraints), std::move(animation_skeleton_joint_constraints_storages), std::move(ragdoll_skeleton_rigid_bodies), std::move(ragdoll_skeleton_constraints), std::move(animation_to_ragdoll_direct_mapping), std::move(ragdoll_to_animation_direct_mapping)};

    return new_unwrapped_skeleton;
}

static inline void release_skeleton(brx_motion_animation_skeleton *release_unwrapped_skeleton)
{
    if (0U == release_unwrapped_skeleton->internal_release())
    {
        brx_motion_animation_skeleton *delete_unwrapped_skeleton = release_unwrapped_skeleton;

        delete_unwrapped_skeleton->~brx_motion_animation_skeleton();
        mcrt_free(delete_unwrapped_skeleton);
    }
}

extern "C" void brx_motion_destroy_skeleton(brx_motion_skeleton *wrapped_skeleton)
{
    assert(NULL != wrapped_skeleton);
    brx_motion_animation_skeleton *release_unwrapped_skeleton = static_cast<brx_motion_animation_skeleton *>(wrapped_skeleton);

    release_skeleton(release_unwrapped_skeleton);
}

extern "C" brx_motion_skeleton_instance *brx_motion_create_skeleton_instance(brx_motion_skeleton *wrapped_skeleton)
{
    void *new_unwrapped_skeleton_instance_base = mcrt_malloc(sizeof(brx_motion_animation_skeleton_instance), alignof(brx_motion_animation_skeleton_instance));
    assert(NULL != new_unwrapped_skeleton_instance_base);

    brx_motion_animation_skeleton_instance *new_unwrapped_skeleton_instance = new (new_unwrapped_skeleton_instance_base) brx_motion_animation_skeleton_instance{};

    new_unwrapped_skeleton_instance->init(static_cast<brx_motion_animation_skeleton *>(wrapped_skeleton));

    return new_unwrapped_skeleton_instance;
}

extern "C" void brx_motion_destroy_skeleton_instance(brx_motion_skeleton_instance *wrapped_skeleton_instance)
{
    assert(NULL != wrapped_skeleton_instance);
    brx_motion_animation_skeleton_instance *delete_unwrapped_skeleton_instance = static_cast<brx_motion_animation_skeleton_instance *>(wrapped_skeleton_instance);

    delete_unwrapped_skeleton_instance->uninit();

    delete_unwrapped_skeleton_instance->~brx_motion_animation_skeleton_instance();
    mcrt_free(delete_unwrapped_skeleton_instance);
}

inline brx_motion_animation_animation::brx_motion_animation_animation(
    mcrt_unordered_map<BRX_MOTION_MORPH_TARGET_NAME, uint32_t> &&weight_channel_indices,
    mcrt_vector<float> weights,
    mcrt_unordered_map<BRX_MOTION_SKELETON_JOINT_NAME, uint32_t> &&rigid_transform_channel_indices,
    mcrt_vector<brx_animation_rigid_transform> &&rigid_transforms,
    mcrt_unordered_map<BRX_MOTION_SKELETON_JOINT_CONSTRAINT_NAME, uint32_t> &&switch_channel_indices,
    mcrt_vector<uint8_t> &&switches)
    : m_ref_count(1U),
      m_weight_channel_indices(std::move(weight_channel_indices)),
      m_weights(std::move(weights)),
      m_rigid_transform_channel_indices(std::move(rigid_transform_channel_indices)),
      m_rigid_transforms(std::move(rigid_transforms)),
      m_switch_channel_indices(std::move(switch_channel_indices)),
      m_switches(std::move(switches))
{
}

inline brx_motion_animation_animation::~brx_motion_animation_animation()
{
    assert(0U == this->m_ref_count);
}

inline void brx_motion_animation_animation::retain()
{
    assert(this->m_ref_count > 0U);
    assert(this->m_ref_count < static_cast<uint32_t>(UINT32_MAX));
    ++this->m_ref_count;
}

inline uint32_t brx_motion_animation_animation::internal_release()
{
    assert(this->m_ref_count > 0U);
    --this->m_ref_count;
    return this->m_ref_count;
}

inline uint32_t brx_motion_animation_animation::get_frame_count() const
{
    if ((!this->m_weight_channel_indices.empty()) && (!this->m_rigid_transform_channel_indices.empty()) && (!this->m_switch_channel_indices.empty()))
    {
        assert(0U == (this->m_weights.size() % this->m_weight_channel_indices.size()));
        assert(0U == (this->m_rigid_transforms.size() % this->m_rigid_transform_channel_indices.size()));
        assert(0U == (this->m_switches.size() % this->m_switch_channel_indices.size()));

        uint32_t const frame_count = this->m_rigid_transforms.size() / this->m_rigid_transform_channel_indices.size();

        assert((this->m_weights.size() / this->m_weight_channel_indices.size()) == frame_count);
        assert((this->m_switches.size() / this->m_switch_channel_indices.size()) == frame_count);

        return frame_count;
    }
    else if ((!this->m_weight_channel_indices.empty()) && (!this->m_rigid_transform_channel_indices.empty()))
    {
        assert(this->m_switch_channel_indices.empty());
        assert(this->m_switches.empty());
        assert(0U == (this->m_weights.size() % this->m_weight_channel_indices.size()));
        assert(0U == (this->m_rigid_transforms.size() % this->m_rigid_transform_channel_indices.size()));

        uint32_t const frame_count = this->m_rigid_transforms.size() / this->m_rigid_transform_channel_indices.size();

        assert((this->m_weights.size() / this->m_weight_channel_indices.size()) == frame_count);

        return frame_count;
    }
    else if ((!this->m_weight_channel_indices.empty()) && (!this->m_switch_channel_indices.empty()))
    {
        assert(this->m_rigid_transform_channel_indices.empty());
        assert(this->m_rigid_transforms.empty());
        assert(0U == (this->m_weights.size() % this->m_weight_channel_indices.size()));
        assert(0U == (this->m_switches.size() % this->m_switch_channel_indices.size()));

        uint32_t const frame_count = this->m_weights.size() / this->m_weight_channel_indices.size();

        assert((this->m_switches.size() / this->m_switch_channel_indices.size()) == frame_count);

        return frame_count;
    }
    else if ((!this->m_rigid_transform_channel_indices.empty()) && (!this->m_switch_channel_indices.empty()))
    {
        assert(this->m_weight_channel_indices.empty());
        assert(this->m_weights.empty());
        assert(0U == (this->m_rigid_transforms.size() % this->m_rigid_transform_channel_indices.size()));
        assert(0U == (this->m_switches.size() % this->m_switch_channel_indices.size()));

        uint32_t const frame_count = this->m_rigid_transforms.size() / this->m_rigid_transform_channel_indices.size();

        assert((this->m_switches.size() / this->m_switch_channel_indices.size()) == frame_count);

        return frame_count;
    }
    else if (!this->m_weight_channel_indices.empty())
    {
        assert(this->m_rigid_transform_channel_indices.empty());
        assert(this->m_rigid_transforms.empty());
        assert(this->m_switch_channel_indices.empty());
        assert(this->m_switches.empty());
        assert(0U == (this->m_weights.size() % this->m_weight_channel_indices.size()));

        uint32_t const frame_count = this->m_weights.size() / this->m_weight_channel_indices.size();

        return frame_count;
    }
    else if (!this->m_rigid_transform_channel_indices.empty())
    {
        assert(this->m_weight_channel_indices.empty());
        assert(this->m_weights.empty());
        assert(this->m_switch_channel_indices.empty());
        assert(this->m_switches.empty());
        assert(0U == (this->m_rigid_transforms.size() % this->m_rigid_transform_channel_indices.size()));

        uint32_t const frame_count = this->m_rigid_transforms.size() / this->m_rigid_transform_channel_indices.size();

        return frame_count;
    }
    else if (!this->m_switch_channel_indices.empty())
    {
        assert(this->m_weight_channel_indices.empty());
        assert(this->m_weights.empty());
        assert(this->m_rigid_transform_channel_indices.empty());
        assert(this->m_rigid_transforms.empty());
        assert(0U == (this->m_switches.size() % this->m_switch_channel_indices.size()));

        uint32_t const frame_count = (this->m_switches.size() / this->m_switch_channel_indices.size());

        return frame_count;
    }
    else
    {
        assert(this->m_weight_channel_indices.empty());
        assert(this->m_weights.empty());
        assert(this->m_rigid_transform_channel_indices.empty());
        assert(this->m_rigid_transforms.empty());
        assert(this->m_switch_channel_indices.empty());
        assert(this->m_switches.empty());

        return 0U;
    }
}

inline float const *brx_motion_animation_animation::get_weight(uint32_t frame_index, BRX_MOTION_MORPH_TARGET_NAME channel_name) const
{
    float const *weight;
    {
        auto const found_channel_index = this->m_weight_channel_indices.find(channel_name);
        if (this->m_weight_channel_indices.end() != found_channel_index)
        {
            uint32_t const channel_count = this->m_weight_channel_indices.size();
            uint32_t const channel_index = found_channel_index->second;
            assert(channel_index < channel_count);
            assert(frame_index < this->get_frame_count());
            weight = (this->m_weights.data() + (channel_count * frame_index + channel_index));
        }
        else
        {
            weight = NULL;
        }
    }
    return weight;
}

inline brx_animation_rigid_transform const *brx_motion_animation_animation::get_rigid_transform(uint32_t frame_index, BRX_MOTION_SKELETON_JOINT_NAME channel_name) const
{
    brx_animation_rigid_transform const *rigid_transform;
    {
        auto const found_channel_index = this->m_rigid_transform_channel_indices.find(channel_name);
        if (this->m_rigid_transform_channel_indices.end() != found_channel_index)
        {
            uint32_t const channel_count = this->m_rigid_transform_channel_indices.size();
            uint32_t const channel_index = found_channel_index->second;
            assert(channel_index < channel_count);
            assert(frame_index < this->get_frame_count());
            rigid_transform = (this->m_rigid_transforms.data() + (channel_count * frame_index + channel_index));
        }
        else
        {
            rigid_transform = NULL;
        }
    }
    return rigid_transform;
}

inline uint8_t const *brx_motion_animation_animation::get_switch(uint32_t frame_index, BRX_MOTION_SKELETON_JOINT_CONSTRAINT_NAME channel_name) const
{
    uint8_t const *_switch;
    {
        auto const found_channel_index = this->m_switch_channel_indices.find(channel_name);
        if (this->m_switch_channel_indices.end() != found_channel_index)
        {
            uint32_t const channel_count = this->m_switch_channel_indices.size();
            uint32_t const channel_index = found_channel_index->second;
            assert(channel_index < channel_count);
            assert(frame_index < this->get_frame_count());
            _switch = (this->m_switches.data() + (channel_count * frame_index + channel_index));
        }
        else
        {
            _switch = NULL;
        }
    }
    return _switch;
}

inline brx_motion_animation_animation_instance::brx_motion_animation_animation_instance()
    : m_ref_count(0U),
      m_animation(NULL),
      m_remainder_time(0.0F),
      m_delta_time(0.0F),
      m_frame_index(0U),
      m_continuous(false)
{
}

inline brx_motion_animation_animation_instance::~brx_motion_animation_animation_instance()
{
    assert(0U == this->m_ref_count);
    assert(NULL == this->m_animation);
}

inline void brx_motion_animation_animation_instance::init(brx_motion_animation_animation *animation)
{
    assert(0U == this->m_ref_count);
    this->m_ref_count = 1U;

    assert(NULL == this->m_animation);
    animation->retain();
    this->m_animation = animation;
}

inline void brx_motion_animation_animation_instance::uninit()
{
    assert(0U == this->m_ref_count);

    assert(NULL != this->m_animation);
    release_animation(this->m_animation);
    this->m_animation = NULL;
}

inline void brx_motion_animation_animation_instance::retain()
{
    assert(this->m_ref_count > 0U);
    assert(this->m_ref_count < static_cast<uint32_t>(UINT32_MAX));
    ++this->m_ref_count;
}

inline uint32_t brx_motion_animation_animation_instance::internal_release()
{
    assert(this->m_ref_count > 0U);
    --this->m_ref_count;
    return this->m_ref_count;
}

void brx_motion_animation_animation_instance::step(float delta_time)
{
    assert(delta_time >= 0.0F);

    assert(this->m_remainder_time >= 0.0F);
    assert(this->m_remainder_time < INTERNAL_FRAME_DELTA_TIME);
    this->m_remainder_time = (this->m_remainder_time + delta_time);

    uint32_t delta_frame_count;
    if (this->m_remainder_time >= INTERNAL_FRAME_DELTA_TIME)
    {
        delta_frame_count = static_cast<uint32_t>(this->m_remainder_time * (1.0F / INTERNAL_FRAME_DELTA_TIME));
        this->m_remainder_time = (this->m_remainder_time - INTERNAL_FRAME_DELTA_TIME * delta_frame_count);
    }
    else
    {
        delta_frame_count = 0U;
    }
    assert(delta_frame_count <= 30U);
    assert(this->m_remainder_time >= 0.0F);
    assert(this->m_remainder_time < INTERNAL_FRAME_DELTA_TIME);

    this->m_delta_time = INTERNAL_FRAME_DELTA_TIME * delta_frame_count;

    assert(NULL != this->m_animation);
    uint32_t const frame_count = this->m_animation->get_frame_count();

    if (frame_count > 0U)
    {
        this->m_frame_index = (this->m_frame_index + delta_frame_count);
        this->m_continuous = true;
        if (this->m_frame_index >= frame_count)
        {
            this->m_frame_index = this->m_frame_index % frame_count;
            this->m_continuous = false;
        }
        assert(this->m_frame_index >= 0U);
        assert(this->m_frame_index < frame_count);
    }
    else
    {
        assert(false);
    }
}

float brx_motion_animation_animation_instance::get_morph_target_weight(BRX_MOTION_MORPH_TARGET_NAME morph_target_name) const
{
    float const *const weight = this->m_animation->get_weight(this->m_frame_index, morph_target_name);
    return ((NULL != weight) ? (*weight) : 0.0F);
}

inline brx_motion_animation_animation const *brx_motion_animation_animation_instance::get_animation() const
{
    return this->m_animation;
}

inline float brx_motion_animation_animation_instance::get_delta_time() const
{
    return this->m_delta_time;
}

uint32_t brx_motion_animation_animation_instance::get_frame_index() const
{
    return this->m_frame_index;
}

inline bool brx_motion_animation_animation_instance::get_continuous() const
{
    return this->m_continuous;
}

inline brx_motion_animation_skeleton::brx_motion_animation_skeleton(
    mcrt_vector<BRX_MOTION_SKELETON_JOINT_NAME> &&animation_skeleton_joint_names,
    mcrt_vector<uint32_t> &&animation_skeleton_joint_parent_indices,
    mcrt_vector<brx_animation_rigid_transform> &&animation_skeleton_bind_pose_local_space,
    mcrt_vector<DirectX::XMFLOAT4X4> &&animation_skeleton_bind_pose_inverse_model_space,
    mcrt_vector<BRX_MOTION_SKELETON_JOINT_CONSTRAINT_NAME> &&animation_skeleton_joint_constraint_names,
    mcrt_vector<brx_animation_skeleton_joint_constraint> &&animation_skeleton_joint_constraints,
    mcrt_vector<mcrt_vector<uint32_t>> &&animation_skeleton_joint_constraints_storages,
    mcrt_vector<brx_animation_physics_rigid_body> &&ragdoll_skeleton_rigid_bodies,
    mcrt_vector<brx_animation_physics_constraint> &&ragdoll_skeleton_constraints,
    mcrt_vector<brx_animation_ragdoll_direct_mapping> &&animation_to_ragdoll_direct_mapping,
    mcrt_vector<brx_animation_ragdoll_direct_mapping> &&ragdoll_to_animation_direct_mapping)
    : m_ref_count(1U),
      m_animation_skeleton_joint_names(std::move(animation_skeleton_joint_names)),
      m_animation_skeleton_joint_parent_indices(std::move(animation_skeleton_joint_parent_indices)),
      m_animation_skeleton_bind_pose_local_space(std::move(animation_skeleton_bind_pose_local_space)),
      m_animation_skeleton_bind_pose_inverse_model_space(std::move(animation_skeleton_bind_pose_inverse_model_space)),
      m_animation_skeleton_joint_constraint_names(std::move(animation_skeleton_joint_constraint_names)),
      m_animation_skeleton_joint_constraints(std::move(animation_skeleton_joint_constraints)),
      m_animation_skeleton_joint_constraints_storages(std::move(animation_skeleton_joint_constraints_storages)),
      m_ragdoll_skeleton_rigid_bodies(std::move(ragdoll_skeleton_rigid_bodies)),
      m_ragdoll_skeleton_constraints(std::move(ragdoll_skeleton_constraints)),
      m_animation_to_ragdoll_direct_mapping(std::move(animation_to_ragdoll_direct_mapping)),
      m_ragdoll_to_animation_direct_mapping(std::move(ragdoll_to_animation_direct_mapping))
{
    this->m_animation_skeleton_joint_indices.reserve(BRX_MOTION_SKELETON_JOINT_NAME_MMD_COUNT);

    uint32_t const animation_skeleton_joint_count = this->get_animation_skeleton_joint_count();

    for (uint32_t animation_skeleton_joint_index = 0; animation_skeleton_joint_index < animation_skeleton_joint_count; ++animation_skeleton_joint_index)
    {
        BRX_MOTION_SKELETON_JOINT_NAME const skeleton_joint_name = this->get_animation_skeleton_joint_names()[animation_skeleton_joint_index];

        if (BRX_MOTION_SKELETON_JOINT_NAME_INVALID != skeleton_joint_name)
        {
            auto const found_animation_skeleton_joint_index = this->m_animation_skeleton_joint_indices.find(skeleton_joint_name);

            if (this->m_animation_skeleton_joint_indices.end() == found_animation_skeleton_joint_index)
            {
                this->m_animation_skeleton_joint_indices.emplace_hint(found_animation_skeleton_joint_index, skeleton_joint_name, animation_skeleton_joint_index);
            }
            else
            {
                assert((skeleton_joint_name == BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_BREAST_1) || (skeleton_joint_name == BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_BREAST_1));
            }
        }
    }
}

inline brx_motion_animation_skeleton::~brx_motion_animation_skeleton()
{
    assert(0U == this->m_ref_count);
}

inline void brx_motion_animation_skeleton::retain()
{
    assert(this->m_ref_count > 0U);
    assert(this->m_ref_count < static_cast<uint32_t>(UINT32_MAX));
    ++this->m_ref_count;
}

inline uint32_t brx_motion_animation_skeleton::internal_release()
{
    assert(this->m_ref_count > 0U);
    --this->m_ref_count;
    return this->m_ref_count;
}

inline uint32_t brx_motion_animation_skeleton::get_animation_skeleton_joint_count() const
{
    uint32_t const animation_skeleton_joint_count = this->m_animation_skeleton_joint_names.size();
    assert(this->m_animation_skeleton_joint_parent_indices.size() == animation_skeleton_joint_count);
    assert(this->m_animation_skeleton_bind_pose_local_space.size() == animation_skeleton_joint_count);
    assert(this->m_animation_skeleton_bind_pose_inverse_model_space.size() == animation_skeleton_joint_count);
    return animation_skeleton_joint_count;
}

inline BRX_MOTION_SKELETON_JOINT_NAME const *brx_motion_animation_skeleton::get_animation_skeleton_joint_names() const
{
    return this->m_animation_skeleton_joint_names.data();
}

inline uint32_t const *brx_motion_animation_skeleton::get_animation_skeleton_joint_parent_indices() const
{
    return this->m_animation_skeleton_joint_parent_indices.data();
}

inline brx_animation_rigid_transform const *brx_motion_animation_skeleton::get_animation_skeleton_bind_pose_local_space() const
{
    return this->m_animation_skeleton_bind_pose_local_space.data();
}

inline DirectX::XMFLOAT4X4 const *brx_motion_animation_skeleton::get_animation_skeleton_bind_pose_inverse_model_space() const
{
    return this->m_animation_skeleton_bind_pose_inverse_model_space.data();
}

inline uint32_t brx_motion_animation_skeleton::get_animation_skeleton_joint_index(BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name) const
{
    uint32_t animation_skeleton_joint_index;
    {
        auto const found_animation_skeleton_joint_index = this->m_animation_skeleton_joint_indices.find(skeleton_joint_name);
        if (this->m_animation_skeleton_joint_indices.end() != found_animation_skeleton_joint_index)
        {
            animation_skeleton_joint_index = found_animation_skeleton_joint_index->second;
        }
        else
        {
            animation_skeleton_joint_index = BRX_MOTION_UINT32_INDEX_INVALID;
        }
    }
    return animation_skeleton_joint_index;
}

inline uint32_t brx_motion_animation_skeleton::get_animation_skeleton_joint_constraint_count() const
{
    uint32_t const animation_skeleton_joint_constraint_count = this->m_animation_skeleton_joint_constraint_names.size();
    assert(this->m_animation_skeleton_joint_constraints.size() == animation_skeleton_joint_constraint_count);
    assert(this->m_animation_skeleton_joint_constraints_storages.size() == animation_skeleton_joint_constraint_count);
    return animation_skeleton_joint_constraint_count;
}

inline BRX_MOTION_SKELETON_JOINT_CONSTRAINT_NAME const *brx_motion_animation_skeleton::get_animation_skeleton_joint_constraint_names() const
{
    return this->m_animation_skeleton_joint_constraint_names.data();
}

inline brx_animation_skeleton_joint_constraint const *brx_motion_animation_skeleton::get_animation_skeleton_joint_constraints() const
{
    return this->m_animation_skeleton_joint_constraints.data();
}

inline uint32_t brx_motion_animation_skeleton::get_animation_skeleton_joint_constraints_storage_size(uint32_t animation_skeleton_joint_constraint_index) const
{
    return this->m_animation_skeleton_joint_constraints_storages[animation_skeleton_joint_constraint_index].size();
}

inline uint32_t const *brx_motion_animation_skeleton::get_animation_skeleton_joint_constraints_storage_base(uint32_t animation_skeleton_joint_constraint_index) const
{
    return this->m_animation_skeleton_joint_constraints_storages[animation_skeleton_joint_constraint_index].data();
}

inline uint32_t brx_motion_animation_skeleton::get_ragdoll_skeleton_rigid_body_count() const
{
    return this->m_ragdoll_skeleton_rigid_bodies.size();
}

inline brx_animation_physics_rigid_body const *brx_motion_animation_skeleton::get_ragdoll_skeleton_rigid_bodies() const
{
    return this->m_ragdoll_skeleton_rigid_bodies.data();
}

inline uint32_t brx_motion_animation_skeleton::get_ragdoll_skeleton_constraint_count() const
{
    return this->m_ragdoll_skeleton_constraints.size();
}

inline brx_animation_physics_constraint const *brx_motion_animation_skeleton::get_ragdoll_skeleton_constraints() const
{
    return this->m_ragdoll_skeleton_constraints.data();
}

inline uint32_t brx_motion_animation_skeleton::get_animation_to_ragdoll_direct_mapping_count() const
{
    return this->m_animation_to_ragdoll_direct_mapping.size();
}

inline brx_animation_ragdoll_direct_mapping const *brx_motion_animation_skeleton::get_animation_to_ragdoll_direct_mappings() const
{
    return this->m_animation_to_ragdoll_direct_mapping.data();
}

inline uint32_t brx_motion_animation_skeleton::get_ragdoll_to_animation_direct_mapping_count() const
{
    return this->m_ragdoll_to_animation_direct_mapping.size();
}

inline brx_animation_ragdoll_direct_mapping const *brx_motion_animation_skeleton::get_ragdoll_to_animation_direct_mappings() const
{
    return this->m_ragdoll_to_animation_direct_mapping.data();
}

inline brx_motion_animation_skeleton_instance::brx_motion_animation_skeleton_instance() : m_skeleton(NULL), m_skin_transforms{}, m_physics_world(NULL), m_physics_rigid_bodies{}, m_physics_constraints{}, m_input_video_detector(NULL), m_input_face_index(BRX_MOTION_UINT32_INDEX_INVALID), m_input_pose_index(BRX_MOTION_UINT32_INDEX_INVALID), m_input_animation_instance(NULL), m_input_continuous(false)
{
}

inline brx_motion_animation_skeleton_instance::~brx_motion_animation_skeleton_instance()
{
    assert(NULL == this->m_skeleton);
    assert(this->m_skin_transforms.empty());
    assert(NULL == this->m_physics_world);
    assert(this->m_physics_rigid_bodies.empty());
    assert(this->m_physics_constraints.empty());
    assert(NULL == this->m_input_video_detector);
    assert(NULL == this->m_input_animation_instance);
}

inline void brx_motion_animation_skeleton_instance::init(brx_motion_animation_skeleton *skeleton)
{
    assert(NULL == this->m_skeleton);
    skeleton->retain();
    this->m_skeleton = skeleton;

    assert(this->m_skin_transforms.empty());
    this->m_skin_transforms.resize(skeleton->get_animation_skeleton_joint_count());

    uint32_t const ragdoll_skeleton_rigid_body_count = skeleton->get_ragdoll_skeleton_rigid_body_count();

    brx_animation_physics_rigid_body const *const ragdoll_skeleton_rigid_bodies = skeleton->get_ragdoll_skeleton_rigid_bodies();

    uint32_t const ragdoll_skeleton_constraint_count = skeleton->get_ragdoll_skeleton_constraint_count();

    brx_animation_physics_constraint const *const ragdoll_skeleton_constraints = skeleton->get_ragdoll_skeleton_constraints();

    {
        if (NULL == g_physics_context)
        {
            assert(0 == g_physics_context_ref_count);

            g_physics_context = brx_physics_create_context();
        }

        assert(g_physics_context_ref_count >= 0);
#ifndef NDEBUG
        ++g_physics_context_ref_count;
#endif
    }
    assert(NULL != g_physics_context);

    assert(NULL == this->m_physics_world);
    {
        float const gravity[3] = {0, -9.8F, 0};
        this->m_physics_world = brx_physics_create_world(g_physics_context, gravity);
        assert(NULL != this->m_physics_world);
    }

    assert(this->m_physics_rigid_bodies.empty());
    this->m_physics_rigid_bodies.resize(ragdoll_skeleton_rigid_body_count);
    {
        for (uint32_t ragdoll_skeleton_rigid_body_index = 0U; ragdoll_skeleton_rigid_body_index < ragdoll_skeleton_rigid_body_count; ++ragdoll_skeleton_rigid_body_index)
        {
            brx_animation_physics_rigid_body const &ragdoll_skeleton_rigid_body = ragdoll_skeleton_rigid_bodies[ragdoll_skeleton_rigid_body_index];

            brx_physics_rigid_body *const physics_rigid_body = brx_physics_create_rigid_body(g_physics_context, this->m_physics_world, ragdoll_skeleton_rigid_body.m_model_space_rotation, ragdoll_skeleton_rigid_body.m_model_space_translation, ragdoll_skeleton_rigid_body.m_shape_type, ragdoll_skeleton_rigid_body.m_shape_size, ragdoll_skeleton_rigid_body.m_motion_type, ragdoll_skeleton_rigid_body.m_collision_filter_group, ragdoll_skeleton_rigid_body.m_collision_filter_mask, ragdoll_skeleton_rigid_body.m_mass, ragdoll_skeleton_rigid_body.m_linear_damping, ragdoll_skeleton_rigid_body.m_angular_damping, ragdoll_skeleton_rigid_body.m_friction, ragdoll_skeleton_rigid_body.m_restitution);
            assert(NULL != physics_rigid_body);

            brx_physics_world_add_rigid_body(g_physics_context, this->m_physics_world, physics_rigid_body);

            this->m_physics_rigid_bodies[ragdoll_skeleton_rigid_body_index] = physics_rigid_body;
        }
    }

    assert(this->m_physics_constraints.empty());
    this->m_physics_constraints.resize(ragdoll_skeleton_constraint_count);
    {
        for (uint32_t ragdoll_skeleton_constraint_index = 0U; ragdoll_skeleton_constraint_index < ragdoll_skeleton_constraint_count; ++ragdoll_skeleton_constraint_index)
        {
            brx_animation_physics_constraint const &ragdoll_skeleton_constraint = ragdoll_skeleton_constraints[ragdoll_skeleton_constraint_index];

            brx_physics_constraint *const physics_constraint = brx_physics_create_constraint(g_physics_context, this->m_physics_world, this->m_physics_rigid_bodies[ragdoll_skeleton_constraint.m_rigid_body_a_index], this->m_physics_rigid_bodies[ragdoll_skeleton_constraint.m_rigid_body_b_index], ragdoll_skeleton_constraint.m_constraint_type, ragdoll_skeleton_constraint.m_pivot, ragdoll_skeleton_constraint.m_twist_axis, ragdoll_skeleton_constraint.m_plane_axis, ragdoll_skeleton_constraint.m_normal_axis, ragdoll_skeleton_constraint.m_twist_limit, ragdoll_skeleton_constraint.m_plane_limit, ragdoll_skeleton_constraint.m_normal_limit);

            brx_physics_world_add_constraint(g_physics_context, this->m_physics_world, physics_constraint);

            this->m_physics_constraints[ragdoll_skeleton_constraint_index] = physics_constraint;
        }
    }
}

extern void internal_retain_video_detector(brx_motion_video_detector *wrapped_video_detector);

extern void internal_release_video_detector(brx_motion_video_detector *wrapped_video_detector);

inline void brx_motion_animation_skeleton_instance::uninit()
{
    if (NULL != this->m_input_animation_instance)
    {
        release_animation_instance(const_cast<brx_motion_animation_animation_instance *>(this->m_input_animation_instance));
        this->m_input_animation_instance = NULL;
    }

    if (NULL != this->m_input_video_detector)
    {
        internal_release_video_detector(const_cast<brx_motion_video_detector *>(this->m_input_video_detector));
        this->m_input_video_detector = NULL;
    }

    for (brx_physics_constraint *const physics_constraint : this->m_physics_constraints)
    {
        assert(NULL != physics_constraint);
        brx_physics_world_remove_constraint(g_physics_context, this->m_physics_world, physics_constraint);
        brx_physics_destroy_constraint(g_physics_context, this->m_physics_world, physics_constraint);
    }
    this->m_physics_constraints.clear();

    for (brx_physics_rigid_body *const physics_rigid_body : this->m_physics_rigid_bodies)
    {
        assert(NULL != physics_rigid_body);
        brx_physics_world_remove_rigid_body(g_physics_context, this->m_physics_world, physics_rigid_body);
        brx_physics_destroy_rigid_body(g_physics_context, this->m_physics_world, physics_rigid_body);
    }
    this->m_physics_rigid_bodies.clear();

    assert(NULL != this->m_physics_world);
    brx_physics_destroy_world(g_physics_context, this->m_physics_world);
    this->m_physics_world = NULL;

    assert(NULL != g_physics_context);
    {
        assert(g_physics_context_ref_count > 0U);
#ifndef NDEBUG
        --g_physics_context_ref_count;
#endif
    }

    assert(!this->m_skin_transforms.empty());
    this->m_skin_transforms.clear();

    assert(NULL != this->m_skeleton);
    release_skeleton(this->m_skeleton);
    this->m_skeleton = NULL;
}

void brx_motion_animation_skeleton_instance::set_input_video_detector(brx_motion_video_detector const *video_detector)
{
    if (this->m_input_video_detector != video_detector)
    {
        if (NULL != this->m_input_video_detector)
        {
            internal_release_video_detector(const_cast<brx_motion_video_detector *>(this->m_input_video_detector));
            this->m_input_video_detector = NULL;
        }

        if (NULL != video_detector)
        {
            internal_retain_video_detector(const_cast<brx_motion_video_detector *>(video_detector));
            this->m_input_video_detector = video_detector;
        }

        if (NULL != this->m_input_animation_instance)
        {
            release_animation_instance(const_cast<brx_motion_animation_animation_instance *>(this->m_input_animation_instance));
            this->m_input_animation_instance = NULL;
        }

        constexpr uint32_t const video_detector_face_skeleton_joint_count = (sizeof(this->m_video_detector_face_skeleton_joint_rotations_local_space) / sizeof(this->m_video_detector_face_skeleton_joint_rotations_local_space[0]));
        for (uint32_t video_detector_face_skeleton_joint_index = 0U; video_detector_face_skeleton_joint_index < video_detector_face_skeleton_joint_count; ++video_detector_face_skeleton_joint_index)
        {
            DirectX::XMStoreFloat4(&this->m_video_detector_face_skeleton_joint_rotations_local_space[video_detector_face_skeleton_joint_index], DirectX::XMQuaternionIdentity());
        }

        constexpr uint32_t const video_detector_pose_skeleton_joint_translation_count = (sizeof(this->m_video_detector_pose_skeleton_joint_translations_model_space) / sizeof(this->m_video_detector_pose_skeleton_joint_translations_model_space[0]));
        static_assert((sizeof(this->m_video_detector_pose_skeleton_joint_translations_model_space_valid) / sizeof(this->m_video_detector_pose_skeleton_joint_translations_model_space_valid[0])) == video_detector_pose_skeleton_joint_translation_count, "");
        for (uint32_t video_detector_pose_skeleton_joint_translation_index = 0U; video_detector_pose_skeleton_joint_translation_index < video_detector_pose_skeleton_joint_translation_count; ++video_detector_pose_skeleton_joint_translation_index)
        {
            this->m_video_detector_pose_skeleton_joint_translations_model_space_valid[video_detector_pose_skeleton_joint_translation_index] = false;
            DirectX::XMStoreFloat3(&this->m_video_detector_pose_skeleton_joint_translations_model_space[video_detector_pose_skeleton_joint_translation_index], DirectX::XMVectorZero());
        }

        constexpr uint32_t const video_detector_pose_skeleton_joint_rotation_count = (sizeof(this->m_video_detector_pose_skeleton_joint_rotations_local_space) / sizeof(this->m_video_detector_pose_skeleton_joint_rotations_local_space[0]));
        for (uint32_t video_detector_pose_skeleton_joint_rotation_index = 0U; video_detector_pose_skeleton_joint_rotation_index < video_detector_pose_skeleton_joint_rotation_count; ++video_detector_pose_skeleton_joint_rotation_index)
        {
            DirectX::XMStoreFloat4(&this->m_video_detector_pose_skeleton_joint_rotations_local_space[video_detector_pose_skeleton_joint_rotation_index], DirectX::XMQuaternionIdentity());
        }

        this->m_input_continuous = false;
    }
    else
    {
        assert(NULL == video_detector);
    }

    if (NULL == this->m_input_video_detector)
    {
        this->m_input_face_index = BRX_MOTION_UINT32_INDEX_INVALID;

        this->m_input_pose_index = BRX_MOTION_UINT32_INDEX_INVALID;
    }
}

void brx_motion_animation_skeleton_instance::set_input_video_detector_face_index(uint32_t face_index)
{
    if (this->m_input_face_index != face_index)
    {
        this->m_input_face_index = face_index;

        constexpr uint32_t const video_detector_face_skeleton_joint_count = (sizeof(this->m_video_detector_face_skeleton_joint_rotations_local_space) / sizeof(this->m_video_detector_face_skeleton_joint_rotations_local_space[0]));
        for (uint32_t video_detector_face_skeleton_joint_index = 0U; video_detector_face_skeleton_joint_index < video_detector_face_skeleton_joint_count; ++video_detector_face_skeleton_joint_index)
        {
            DirectX::XMStoreFloat4(&this->m_video_detector_face_skeleton_joint_rotations_local_space[video_detector_face_skeleton_joint_index], DirectX::XMQuaternionIdentity());
        }

        constexpr uint32_t const video_detector_pose_skeleton_joint_translation_count = (sizeof(this->m_video_detector_pose_skeleton_joint_translations_model_space) / sizeof(this->m_video_detector_pose_skeleton_joint_translations_model_space[0]));
        static_assert((sizeof(this->m_video_detector_pose_skeleton_joint_translations_model_space_valid) / sizeof(this->m_video_detector_pose_skeleton_joint_translations_model_space_valid[0])) == video_detector_pose_skeleton_joint_translation_count, "");
        for (uint32_t video_detector_pose_skeleton_joint_translation_index = 0U; video_detector_pose_skeleton_joint_translation_index < video_detector_pose_skeleton_joint_translation_count; ++video_detector_pose_skeleton_joint_translation_index)
        {
            this->m_video_detector_pose_skeleton_joint_translations_model_space_valid[video_detector_pose_skeleton_joint_translation_index] = false;
            DirectX::XMStoreFloat3(&this->m_video_detector_pose_skeleton_joint_translations_model_space[video_detector_pose_skeleton_joint_translation_index], DirectX::XMVectorZero());
        }

        constexpr uint32_t const video_detector_pose_skeleton_joint_rotation_count = (sizeof(this->m_video_detector_pose_skeleton_joint_rotations_local_space) / sizeof(this->m_video_detector_pose_skeleton_joint_rotations_local_space[0]));
        for (uint32_t video_detector_pose_skeleton_joint_rotation_index = 0U; video_detector_pose_skeleton_joint_rotation_index < video_detector_pose_skeleton_joint_rotation_count; ++video_detector_pose_skeleton_joint_rotation_index)
        {
            DirectX::XMStoreFloat4(&this->m_video_detector_pose_skeleton_joint_rotations_local_space[video_detector_pose_skeleton_joint_rotation_index], DirectX::XMQuaternionIdentity());
        }

        this->m_input_continuous = false;
    }
    else
    {
        assert(BRX_MOTION_UINT32_INDEX_INVALID == face_index);
    }
}

void brx_motion_animation_skeleton_instance::set_input_video_detector_pose_index(uint32_t pose_index)
{
    if (this->m_input_pose_index != pose_index)
    {
        this->m_input_pose_index = pose_index;

        constexpr uint32_t const video_detector_face_skeleton_joint_count = (sizeof(this->m_video_detector_face_skeleton_joint_rotations_local_space) / sizeof(this->m_video_detector_face_skeleton_joint_rotations_local_space[0]));
        for (uint32_t video_detector_face_skeleton_joint_index = 0U; video_detector_face_skeleton_joint_index < video_detector_face_skeleton_joint_count; ++video_detector_face_skeleton_joint_index)
        {
            DirectX::XMStoreFloat4(&this->m_video_detector_face_skeleton_joint_rotations_local_space[video_detector_face_skeleton_joint_index], DirectX::XMQuaternionIdentity());
        }

        constexpr uint32_t const video_detector_pose_skeleton_joint_translation_count = (sizeof(this->m_video_detector_pose_skeleton_joint_translations_model_space) / sizeof(this->m_video_detector_pose_skeleton_joint_translations_model_space[0]));
        static_assert((sizeof(this->m_video_detector_pose_skeleton_joint_translations_model_space_valid) / sizeof(this->m_video_detector_pose_skeleton_joint_translations_model_space_valid[0])) == video_detector_pose_skeleton_joint_translation_count, "");
        for (uint32_t video_detector_pose_skeleton_joint_translation_index = 0U; video_detector_pose_skeleton_joint_translation_index < video_detector_pose_skeleton_joint_translation_count; ++video_detector_pose_skeleton_joint_translation_index)
        {
            this->m_video_detector_pose_skeleton_joint_translations_model_space_valid[video_detector_pose_skeleton_joint_translation_index] = false;
            DirectX::XMStoreFloat3(&this->m_video_detector_pose_skeleton_joint_translations_model_space[video_detector_pose_skeleton_joint_translation_index], DirectX::XMVectorZero());
        }

        constexpr uint32_t const video_detector_pose_skeleton_joint_rotation_count = (sizeof(this->m_video_detector_pose_skeleton_joint_rotations_local_space) / sizeof(this->m_video_detector_pose_skeleton_joint_rotations_local_space[0]));
        for (uint32_t video_detector_pose_skeleton_joint_rotation_index = 0U; video_detector_pose_skeleton_joint_rotation_index < video_detector_pose_skeleton_joint_rotation_count; ++video_detector_pose_skeleton_joint_rotation_index)
        {
            DirectX::XMStoreFloat4(&this->m_video_detector_pose_skeleton_joint_rotations_local_space[video_detector_pose_skeleton_joint_rotation_index], DirectX::XMQuaternionIdentity());
        }

        this->m_input_continuous = false;
    }
    else
    {
        assert(BRX_MOTION_UINT32_INDEX_INVALID == pose_index);
    }
}

void brx_motion_animation_skeleton_instance::set_input_animation_instance(brx_motion_animation_instance const *wrapped_animation_instance)
{
    if (this->m_input_animation_instance != wrapped_animation_instance)
    {
        if (NULL != this->m_input_video_detector)
        {
            internal_release_video_detector(const_cast<brx_motion_video_detector *>(this->m_input_video_detector));
            this->m_input_video_detector = NULL;
        }

        if (NULL != this->m_input_animation_instance)
        {
            release_animation_instance(const_cast<brx_motion_animation_animation_instance *>(this->m_input_animation_instance));
            this->m_input_animation_instance = NULL;
        }

        if (NULL != wrapped_animation_instance)
        {
            brx_motion_animation_animation_instance *const unwrapped_animation_instance = const_cast<brx_motion_animation_animation_instance *>(static_cast<brx_motion_animation_animation_instance const *>(wrapped_animation_instance));
            unwrapped_animation_instance->retain();
            this->m_input_animation_instance = unwrapped_animation_instance;
        }

        this->m_input_continuous = false;
    }
    else
    {
        assert(NULL == wrapped_animation_instance);
    }

    assert(NULL == this->m_input_video_detector);

    this->m_input_face_index = BRX_MOTION_UINT32_INDEX_INVALID;

    this->m_input_pose_index = BRX_MOTION_UINT32_INDEX_INVALID;
}

brx_motion_video_detector *brx_motion_animation_skeleton_instance::get_input_video_detector() const
{
    return const_cast<brx_motion_video_detector *>(this->m_input_video_detector);
}

uint32_t brx_motion_animation_skeleton_instance::get_input_video_detector_face_index() const
{
    return this->m_input_face_index;
}

uint32_t brx_motion_animation_skeleton_instance::get_input_video_detector_pose_index() const
{
    return this->m_input_pose_index;
}

brx_motion_animation_instance *brx_motion_animation_skeleton_instance::get_input_animation_instance() const
{
    return const_cast<brx_motion_animation_animation_instance *>(this->m_input_animation_instance);
}

void brx_motion_animation_skeleton_instance::step(BRX_MOTION_PHYSICS_RAGDOLL_QUALITY physics_ragdoll_quality)
{
    uint32_t const animation_skeleton_joint_count = this->m_skeleton->get_animation_skeleton_joint_count();

    mcrt_vector<DirectX::XMFLOAT4X4> animation_skeleton_bind_pose_local_space(static_cast<size_t>(animation_skeleton_joint_count));
    mcrt_vector<DirectX::XMFLOAT4X4> animation_skeleton_animation_pose_local_space(static_cast<size_t>(animation_skeleton_joint_count));

    assert(NULL == this->m_input_video_detector || NULL == this->m_input_animation_instance);

    if (NULL != this->m_input_video_detector)
    {
        assert((BRX_MOTION_UINT32_INDEX_INVALID == this->m_input_face_index) || (this->m_input_face_index < this->m_input_video_detector->get_face_count()));
        assert((BRX_MOTION_UINT32_INDEX_INVALID == this->m_input_pose_index) || (this->m_input_pose_index < this->m_input_video_detector->get_pose_count()));

        brx_motion_media_pipe_video_detector const *const input_video_detector = static_cast<brx_motion_media_pipe_video_detector const *>(this->m_input_video_detector);

        for (uint32_t animation_skeleton_joint_index = 0; animation_skeleton_joint_index < animation_skeleton_joint_count; ++animation_skeleton_joint_index)
        {
            DirectX::XMFLOAT4 const &animation_skeleton_bind_pose_rotation_local_space = this->m_skeleton->get_animation_skeleton_bind_pose_local_space()[animation_skeleton_joint_index].m_rotation;
            DirectX::XMFLOAT3 const &animation_skeleton_bind_pose_translation_local_space = this->m_skeleton->get_animation_skeleton_bind_pose_local_space()[animation_skeleton_joint_index].m_translation;

            // bind pose rotation always zero
            assert(DirectX::XMVector4EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(DirectX::XMLoadFloat4(&animation_skeleton_bind_pose_rotation_local_space), DirectX::XMQuaternionIdentity())), DirectX::XMVectorReplicate(INTERNAL_ROTATION_EPSILON))));

            DirectX::XMStoreFloat4x4(&animation_skeleton_bind_pose_local_space[animation_skeleton_joint_index], DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(DirectX::XMLoadFloat4(&animation_skeleton_bind_pose_rotation_local_space)), DirectX::XMMatrixTranslationFromVector(DirectX::XMLoadFloat3(&animation_skeleton_bind_pose_translation_local_space))));

            DirectX::XMFLOAT4 const &animation_skeleton_animation_pose_rotation_local_space = animation_skeleton_bind_pose_rotation_local_space;
            DirectX::XMFLOAT3 const &animation_skeleton_animation_pose_translation_local_space = animation_skeleton_bind_pose_translation_local_space;

            DirectX::XMStoreFloat4x4(&animation_skeleton_animation_pose_local_space[animation_skeleton_joint_index], DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(DirectX::XMLoadFloat4(&animation_skeleton_animation_pose_rotation_local_space)), DirectX::XMMatrixTranslationFromVector(DirectX::XMLoadFloat3(&animation_skeleton_animation_pose_translation_local_space))));
        }

        if (this->m_input_face_index < this->m_input_video_detector->get_face_count())
        {
            constexpr BRX_MOTION_SKELETON_JOINT_NAME const internal_video_detector_face_skeleton_joints[] = {
                BRX_MOTION_SKELETON_JOINT_NAME_MMD_HEAD,
                BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_EYE,
                BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_EYE};

            constexpr uint32_t const video_detector_face_skeleton_joint_count = sizeof(internal_video_detector_face_skeleton_joints) / sizeof(internal_video_detector_face_skeleton_joints[0]);
            static_assert((sizeof(this->m_video_detector_face_skeleton_joint_rotations_local_space) / sizeof(this->m_video_detector_face_skeleton_joint_rotations_local_space[0])) == video_detector_face_skeleton_joint_count, "");

            for (uint32_t video_detector_face_skeleton_joint_index = 0U; video_detector_face_skeleton_joint_index < video_detector_face_skeleton_joint_count; ++video_detector_face_skeleton_joint_index)
            {
                BRX_MOTION_SKELETON_JOINT_NAME const animation_skeleton_joint_name = internal_video_detector_face_skeleton_joints[video_detector_face_skeleton_joint_index];

                uint32_t const animation_skeleton_joint_index = this->m_skeleton->get_animation_skeleton_joint_index(animation_skeleton_joint_name);
                DirectX::XMFLOAT4 const *const video_detector_face_rotation_model_space = input_video_detector->get_face_skeleton_joint_rotation(this->m_input_face_index, animation_skeleton_joint_name);

                if ((BRX_MOTION_UINT32_INDEX_INVALID != animation_skeleton_joint_index) && (NULL != video_detector_face_rotation_model_space))
                {
                    DirectX::XMFLOAT4 const &current_rotation_local_space = (*video_detector_face_rotation_model_space);

                    DirectX::XMFLOAT4 const &previous_rotation_local_space = this->m_video_detector_face_skeleton_joint_rotations_local_space[video_detector_face_skeleton_joint_index];

                    DirectX::XMFLOAT4 blend_rotation_local_space;
                    DirectX::XMStoreFloat4(&blend_rotation_local_space, DirectX::XMQuaternionSlerp(DirectX::XMLoadFloat4(&previous_rotation_local_space), DirectX::XMLoadFloat4(&current_rotation_local_space), INTERNAL_FACE_SKELETON_JOINT_ROTATION_BLEND_FACTOR));

                    // TODO: use current or blend?
                    this->m_video_detector_face_skeleton_joint_rotations_local_space[video_detector_face_skeleton_joint_index] = blend_rotation_local_space;

                    DirectX::XMStoreFloat4x4(&animation_skeleton_animation_pose_local_space[animation_skeleton_joint_index], DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(DirectX::XMLoadFloat4(&blend_rotation_local_space)), DirectX::XMMatrixTranslationFromVector(DirectX::XMLoadFloat3(&this->m_skeleton->get_animation_skeleton_bind_pose_local_space()[animation_skeleton_joint_index].m_translation))));
                }
                else
                {
                    assert(false);
                }
            }
        }
        else
        {
            assert(BRX_MOTION_UINT32_INDEX_INVALID == this->m_input_face_index);
        }

        if (this->m_input_pose_index < this->m_input_video_detector->get_pose_count())
        {
            // [ARKit: Validating a Model for Motion Capture](https://developer.apple.com/documentation/arkit/validating-a-model-for-motion-capture)
            constexpr BRX_MOTION_SKELETON_JOINT_NAME const internal_video_detector_pose_skeleton_joint_translations[] = {
                BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_SHOULDER,
                BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_ARM,
                BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_ELBOW,
                BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_WRIST,
                BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_INDEX_FINGER_1,
                BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_LEG,
                BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_KNEE,
                BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_ANKLE,
                BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_TOE_TIP,
                BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_SHOULDER,
                BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_ARM,
                BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_ELBOW,
                BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_WRIST,
                BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_INDEX_FINGER_1,
                BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_LEG,
                BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_KNEE,
                BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_ANKLE,
                BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_TOE_TIP};
            constexpr uint32_t const video_detector_pose_skeleton_joint_translation_count = sizeof(internal_video_detector_pose_skeleton_joint_translations) / sizeof(internal_video_detector_pose_skeleton_joint_translations[0]);
            static_assert((sizeof(this->m_video_detector_pose_skeleton_joint_translations_model_space_valid) / sizeof(this->m_video_detector_pose_skeleton_joint_translations_model_space_valid[0])) == video_detector_pose_skeleton_joint_translation_count, "");
            static_assert((sizeof(this->m_video_detector_pose_skeleton_joint_translations_model_space) / sizeof(this->m_video_detector_pose_skeleton_joint_translations_model_space[0])) == video_detector_pose_skeleton_joint_translation_count, "");

            for (uint32_t video_detector_pose_skeleton_joint_translation_index = 0U; video_detector_pose_skeleton_joint_translation_index < video_detector_pose_skeleton_joint_translation_count; ++video_detector_pose_skeleton_joint_translation_index)
            {
                BRX_MOTION_SKELETON_JOINT_NAME const animation_skeleton_joint_name = internal_video_detector_pose_skeleton_joint_translations[video_detector_pose_skeleton_joint_translation_index];

                DirectX::XMFLOAT3 const *const video_detector_pose_translation_model_space = input_video_detector->get_pose_skeleton_joint_translation(this->m_input_pose_index, animation_skeleton_joint_name);

                if (NULL != video_detector_pose_translation_model_space)
                {
                    if (this->m_video_detector_pose_skeleton_joint_translations_model_space_valid[video_detector_pose_skeleton_joint_translation_index])
                    {
                        DirectX::XMFLOAT3 const &current_translation_model_space = (*video_detector_pose_translation_model_space);

                        DirectX::XMFLOAT3 const &previous_translation_model_space = this->m_video_detector_pose_skeleton_joint_translations_model_space[video_detector_pose_skeleton_joint_translation_index];

                        DirectX::XMFLOAT3 blend_translation_model_space;
                        DirectX::XMStoreFloat3(&blend_translation_model_space, DirectX::XMVectorLerp(DirectX::XMLoadFloat3(&previous_translation_model_space), DirectX::XMLoadFloat3(&current_translation_model_space), INTERNAL_POSE_SKELETON_JOINT_TRANSLATION_BLEND_FACTOR));

                        this->m_video_detector_pose_skeleton_joint_translations_model_space[video_detector_pose_skeleton_joint_translation_index] = blend_translation_model_space;
                    }
                    else
                    {
                        this->m_video_detector_pose_skeleton_joint_translations_model_space_valid[video_detector_pose_skeleton_joint_translation_index] = true;

                        this->m_video_detector_pose_skeleton_joint_translations_model_space[video_detector_pose_skeleton_joint_translation_index] = (*video_detector_pose_translation_model_space);
                    }
                }
                else
                {
                    this->m_video_detector_pose_skeleton_joint_translations_model_space_valid[video_detector_pose_skeleton_joint_translation_index] = false;

                    DirectX::XMStoreFloat3(&this->m_video_detector_pose_skeleton_joint_translations_model_space[video_detector_pose_skeleton_joint_translation_index], DirectX::XMVectorZero());
                }
            }

            // TODO: handle shoulder arm

            constexpr uint32_t const internal_video_detector_pose_skeleton_joint_rotation_indices[][2] = {
                // {0, 1},
                {1, 2},
                {2, 3},
                {3, 4},
                {5, 6},
                {6, 7},
                {7, 8},
                // {9, 10},
                {10, 11},
                {11, 12},
                {12, 13},
                {14, 15},
                {15, 16},
                {16, 17}};

            constexpr BRX_MOTION_SKELETON_JOINT_NAME const internal_video_detector_pose_skeleton_joint_rotation_names[][2] = {
                // {BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_SHOULDER, BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_ARM},
                {BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_ARM, BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_ELBOW},
                {BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_ELBOW, BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_WRIST},
                {BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_WRIST, BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_INDEX_FINGER_1},
                {BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_LEG, BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_KNEE},
                {BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_KNEE, BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_ANKLE},
                {BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_ANKLE, BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_TOE_TIP},
                // {BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_SHOULDER, BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_ARM},
                {BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_ARM, BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_ELBOW},
                {BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_ELBOW, BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_WRIST},
                {BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_WRIST, BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_INDEX_FINGER_1},
                {BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_LEG, BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_KNEE},
                {BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_KNEE, BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_ANKLE},
                {BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_ANKLE, BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_TOE_TIP}};

            constexpr uint32_t const video_detector_pose_skeleton_joint_rotation_count = sizeof(internal_video_detector_pose_skeleton_joint_rotation_indices) / sizeof(internal_video_detector_pose_skeleton_joint_rotation_indices[0]);
            static_assert((sizeof(internal_video_detector_pose_skeleton_joint_rotation_names) / sizeof(internal_video_detector_pose_skeleton_joint_rotation_names[0])) == video_detector_pose_skeleton_joint_rotation_count, "");
            static_assert((sizeof(this->m_video_detector_pose_skeleton_joint_rotations_local_space) / sizeof(this->m_video_detector_pose_skeleton_joint_rotations_local_space[0])) == video_detector_pose_skeleton_joint_rotation_count, "");

            // use the previous rotation as the initial state for IK solver
            for (uint32_t video_detector_pose_skeleton_joint_rotation_index = 0U; video_detector_pose_skeleton_joint_rotation_index < video_detector_pose_skeleton_joint_rotation_count; ++video_detector_pose_skeleton_joint_rotation_index)
            {
                BRX_MOTION_SKELETON_JOINT_NAME const ball_and_socket_joint_animation_skeleton_joint_name = internal_video_detector_pose_skeleton_joint_translations[internal_video_detector_pose_skeleton_joint_rotation_indices[video_detector_pose_skeleton_joint_rotation_index][0]];
                assert(internal_video_detector_pose_skeleton_joint_rotation_names[video_detector_pose_skeleton_joint_rotation_index][0] == ball_and_socket_joint_animation_skeleton_joint_name);

                uint32_t const ball_and_socket_joint_animation_skeleton_joint_index = this->m_skeleton->get_animation_skeleton_joint_index(ball_and_socket_joint_animation_skeleton_joint_name);

                if (BRX_MOTION_UINT32_INDEX_INVALID != ball_and_socket_joint_animation_skeleton_joint_index)
                {
                    DirectX::XMFLOAT4 const &previous_ball_and_socket_joint_rotation_local_space = this->m_video_detector_pose_skeleton_joint_rotations_local_space[video_detector_pose_skeleton_joint_rotation_index];

                    DirectX::XMVECTOR ball_and_socket_joint_animation_skeleton_animation_pose_translation_local_space;
                    {
                        DirectX::XMVECTOR ball_and_socket_joint_animation_skeleton_animation_pose_scale_model_space;
                        DirectX::XMVECTOR ball_and_socket_joint_animation_skeleton_animation_pose_rotation_model_space;
                        bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&ball_and_socket_joint_animation_skeleton_animation_pose_scale_model_space, &ball_and_socket_joint_animation_skeleton_animation_pose_rotation_model_space, &ball_and_socket_joint_animation_skeleton_animation_pose_translation_local_space, DirectX::XMLoadFloat4x4(&animation_skeleton_animation_pose_local_space[ball_and_socket_joint_animation_skeleton_joint_index]));
                        assert(directx_xm_matrix_decompose);

                        assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(ball_and_socket_joint_animation_skeleton_animation_pose_scale_model_space, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
                    }
                    assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(ball_and_socket_joint_animation_skeleton_animation_pose_translation_local_space, DirectX::XMLoadFloat3(&this->m_skeleton->get_animation_skeleton_bind_pose_local_space()[ball_and_socket_joint_animation_skeleton_joint_index].m_translation))), DirectX::XMVectorReplicate(INTERNAL_TRANSLATION_EPSILON))));

                    DirectX::XMStoreFloat4x4(&animation_skeleton_animation_pose_local_space[ball_and_socket_joint_animation_skeleton_joint_index], DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(DirectX::XMLoadFloat4(&previous_ball_and_socket_joint_rotation_local_space)), DirectX::XMMatrixTranslationFromVector(ball_and_socket_joint_animation_skeleton_animation_pose_translation_local_space)));
                }
                else
                {
                    assert(false);
                }
            }

            for (uint32_t video_detector_pose_skeleton_joint_rotation_index = 0U; video_detector_pose_skeleton_joint_rotation_index < video_detector_pose_skeleton_joint_rotation_count; ++video_detector_pose_skeleton_joint_rotation_index)
            {
                uint32_t const ball_and_socket_joint_animation_skeleton_joint_translation_index = internal_video_detector_pose_skeleton_joint_rotation_indices[video_detector_pose_skeleton_joint_rotation_index][0];

                uint32_t const end_effector_animation_skeleton_joint_translation_index = internal_video_detector_pose_skeleton_joint_rotation_indices[video_detector_pose_skeleton_joint_rotation_index][1];

                BRX_MOTION_SKELETON_JOINT_NAME const ball_and_socket_joint_animation_skeleton_joint_name = internal_video_detector_pose_skeleton_joint_translations[ball_and_socket_joint_animation_skeleton_joint_translation_index];
                assert(internal_video_detector_pose_skeleton_joint_rotation_names[video_detector_pose_skeleton_joint_rotation_index][0] == ball_and_socket_joint_animation_skeleton_joint_name);

                BRX_MOTION_SKELETON_JOINT_NAME const end_effector_animation_skeleton_joint_name = internal_video_detector_pose_skeleton_joint_translations[end_effector_animation_skeleton_joint_translation_index];
                assert(internal_video_detector_pose_skeleton_joint_rotation_names[video_detector_pose_skeleton_joint_rotation_index][1] == end_effector_animation_skeleton_joint_name);

                uint32_t const ball_and_socket_joint_animation_skeleton_joint_index = this->m_skeleton->get_animation_skeleton_joint_index(ball_and_socket_joint_animation_skeleton_joint_name);

                uint32_t const end_effector_animation_skeleton_joint_index = this->m_skeleton->get_animation_skeleton_joint_index(end_effector_animation_skeleton_joint_name);

                if ((BRX_MOTION_UINT32_INDEX_INVALID != ball_and_socket_joint_animation_skeleton_joint_index) && (BRX_MOTION_UINT32_INDEX_INVALID != end_effector_animation_skeleton_joint_index))
                {
                    if ((this->m_video_detector_pose_skeleton_joint_translations_model_space_valid[ball_and_socket_joint_animation_skeleton_joint_translation_index]) && (this->m_video_detector_pose_skeleton_joint_translations_model_space_valid[end_effector_animation_skeleton_joint_translation_index]))
                    {
                        DirectX::XMFLOAT3 const &ball_and_socket_joint_video_detector_pose_translation_model_space = this->m_video_detector_pose_skeleton_joint_translations_model_space[ball_and_socket_joint_animation_skeleton_joint_translation_index];

                        DirectX::XMFLOAT3 const &end_effector_video_detector_pose_translation_model_space = this->m_video_detector_pose_skeleton_joint_translations_model_space[end_effector_animation_skeleton_joint_translation_index];

                        DirectX::XMFLOAT3 target_displacement_model_space;
                        {
                            // TODO: do we really need to scale to make sure the translation is high enough?
                            DirectX::XMStoreFloat3(&target_displacement_model_space, DirectX::XMVectorScale(DirectX::XMVector3Normalize(DirectX::XMVectorSubtract(DirectX::XMLoadFloat3(&end_effector_video_detector_pose_translation_model_space), DirectX::XMLoadFloat3(&ball_and_socket_joint_video_detector_pose_translation_model_space))), 5.0F));
                        }

                        DirectX::XMFLOAT4 current_ball_and_socket_joint_rotation_local_space;
                        internal_video_detector_pose_ik_one_joint_solve(ball_and_socket_joint_animation_skeleton_joint_index, end_effector_animation_skeleton_joint_index, target_displacement_model_space, this->m_skeleton->get_animation_skeleton_joint_parent_indices(), animation_skeleton_animation_pose_local_space.data(), &current_ball_and_socket_joint_rotation_local_space);

                        DirectX::XMFLOAT4 const &previous_ball_and_socket_joint_rotation_local_space = this->m_video_detector_pose_skeleton_joint_rotations_local_space[video_detector_pose_skeleton_joint_rotation_index];

                        DirectX::XMFLOAT4 blend_ball_and_socket_joint_rotation_local_space;
                        DirectX::XMStoreFloat4(&blend_ball_and_socket_joint_rotation_local_space, DirectX::XMQuaternionSlerp(DirectX::XMLoadFloat4(&previous_ball_and_socket_joint_rotation_local_space), DirectX::XMLoadFloat4(&current_ball_and_socket_joint_rotation_local_space), INTERNAL_POSE_SKELETON_JOINT_ROTATION_BLEND_FACTOR));

                        // TODO: use current or blend?
                        this->m_video_detector_pose_skeleton_joint_rotations_local_space[video_detector_pose_skeleton_joint_rotation_index] = blend_ball_and_socket_joint_rotation_local_space;

                        DirectX::XMVECTOR ball_and_socket_joint_animation_skeleton_animation_pose_translation_local_space;
                        {
                            DirectX::XMVECTOR ball_and_socket_joint_animation_skeleton_animation_pose_scale_model_space;
                            DirectX::XMVECTOR ball_and_socket_joint_animation_skeleton_animation_pose_rotation_model_space;
                            bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&ball_and_socket_joint_animation_skeleton_animation_pose_scale_model_space, &ball_and_socket_joint_animation_skeleton_animation_pose_rotation_model_space, &ball_and_socket_joint_animation_skeleton_animation_pose_translation_local_space, DirectX::XMLoadFloat4x4(&animation_skeleton_animation_pose_local_space[ball_and_socket_joint_animation_skeleton_joint_index]));
                            assert(directx_xm_matrix_decompose);

                            assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(ball_and_socket_joint_animation_skeleton_animation_pose_scale_model_space, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
                        }
                        assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(ball_and_socket_joint_animation_skeleton_animation_pose_translation_local_space, DirectX::XMLoadFloat3(&this->m_skeleton->get_animation_skeleton_bind_pose_local_space()[ball_and_socket_joint_animation_skeleton_joint_index].m_translation))), DirectX::XMVectorReplicate(INTERNAL_TRANSLATION_EPSILON))));

                        DirectX::XMStoreFloat4x4(&animation_skeleton_animation_pose_local_space[ball_and_socket_joint_animation_skeleton_joint_index], DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(DirectX::XMLoadFloat4(&blend_ball_and_socket_joint_rotation_local_space)), DirectX::XMMatrixTranslationFromVector(ball_and_socket_joint_animation_skeleton_animation_pose_translation_local_space)));
                    }
                }
                else
                {
                    assert(false);
                }
            }
        }
        else
        {
            assert(BRX_MOTION_UINT32_INDEX_INVALID == this->m_input_pose_index);
        }
    }
    else if (NULL != this->m_input_animation_instance)
    {
        brx_motion_animation_animation const *const input_animation = this->m_input_animation_instance->get_animation();
        assert(NULL != input_animation);

        for (uint32_t animation_skeleton_joint_index = 0; animation_skeleton_joint_index < animation_skeleton_joint_count; ++animation_skeleton_joint_index)
        {
            DirectX::XMFLOAT4 const &animation_skeleton_bind_pose_rotation_local_space = this->m_skeleton->get_animation_skeleton_bind_pose_local_space()[animation_skeleton_joint_index].m_rotation;
            DirectX::XMFLOAT3 const &animation_skeleton_bind_pose_translation_local_space = this->m_skeleton->get_animation_skeleton_bind_pose_local_space()[animation_skeleton_joint_index].m_translation;

            DirectX::XMStoreFloat4x4(&animation_skeleton_bind_pose_local_space[animation_skeleton_joint_index], DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(DirectX::XMLoadFloat4(&animation_skeleton_bind_pose_rotation_local_space)), DirectX::XMMatrixTranslationFromVector(DirectX::XMLoadFloat3(&animation_skeleton_bind_pose_translation_local_space))));

            DirectX::XMFLOAT4 animation_skeleton_animation_pose_rotation_local_space;
            DirectX::XMFLOAT3 animation_skeleton_animation_pose_translation_local_space;
            {
                BRX_MOTION_SKELETON_JOINT_NAME const skeleton_joint_name = this->m_skeleton->get_animation_skeleton_joint_names()[animation_skeleton_joint_index];

                brx_animation_rigid_transform const *animation_skeleton_delta_transform = NULL;
                if (NULL != (animation_skeleton_delta_transform = input_animation->get_rigid_transform(this->m_input_animation_instance->get_frame_index(), skeleton_joint_name)))
                {
                    // [VRM 1.0: About Pose Data Compatibility](https://github.com/vrm-c/vrm-specification/blob/master/specification/VRMC_vrm_animation-1.0/how_to_transform_human_pose.md)

                    // bind pose rotation always zero
                    assert(DirectX::XMVector4EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(DirectX::XMLoadFloat4(&animation_skeleton_bind_pose_rotation_local_space), DirectX::XMQuaternionIdentity())), DirectX::XMVectorReplicate(INTERNAL_ROTATION_EPSILON))));
                    animation_skeleton_animation_pose_rotation_local_space = animation_skeleton_delta_transform->m_rotation;

                    DirectX::XMStoreFloat3(&animation_skeleton_animation_pose_translation_local_space, DirectX::XMVectorAdd(DirectX::XMLoadFloat3(&animation_skeleton_bind_pose_translation_local_space), DirectX::XMLoadFloat3(&animation_skeleton_delta_transform->m_translation)));
                }
                else
                {
                    animation_skeleton_animation_pose_rotation_local_space = animation_skeleton_bind_pose_rotation_local_space;
                    animation_skeleton_animation_pose_translation_local_space = animation_skeleton_bind_pose_translation_local_space;
                }
            }

            DirectX::XMStoreFloat4x4(&animation_skeleton_animation_pose_local_space[animation_skeleton_joint_index], DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(DirectX::XMLoadFloat4(&animation_skeleton_animation_pose_rotation_local_space)), DirectX::XMMatrixTranslationFromVector(DirectX::XMLoadFloat3(&animation_skeleton_animation_pose_translation_local_space))));
        }
    }
    else
    {
        for (uint32_t animation_skeleton_joint_index = 0; animation_skeleton_joint_index < animation_skeleton_joint_count; ++animation_skeleton_joint_index)
        {
            DirectX::XMFLOAT4 const &animation_skeleton_bind_pose_rotation_local_space = this->m_skeleton->get_animation_skeleton_bind_pose_local_space()[animation_skeleton_joint_index].m_rotation;
            DirectX::XMFLOAT3 const &animation_skeleton_bind_pose_translation_local_space = this->m_skeleton->get_animation_skeleton_bind_pose_local_space()[animation_skeleton_joint_index].m_translation;

            DirectX::XMStoreFloat4x4(&animation_skeleton_bind_pose_local_space[animation_skeleton_joint_index], DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(DirectX::XMLoadFloat4(&animation_skeleton_bind_pose_rotation_local_space)), DirectX::XMMatrixTranslationFromVector(DirectX::XMLoadFloat3(&animation_skeleton_bind_pose_translation_local_space))));

            DirectX::XMFLOAT4 const &animation_skeleton_animation_pose_rotation_local_space = animation_skeleton_bind_pose_rotation_local_space;
            DirectX::XMFLOAT3 const &animation_skeleton_animation_pose_translation_local_space = animation_skeleton_bind_pose_translation_local_space;

            DirectX::XMStoreFloat4x4(&animation_skeleton_animation_pose_local_space[animation_skeleton_joint_index], DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(DirectX::XMLoadFloat4(&animation_skeleton_animation_pose_rotation_local_space)), DirectX::XMMatrixTranslationFromVector(DirectX::XMLoadFloat3(&animation_skeleton_animation_pose_translation_local_space))));
        }
    }

    this->animation_skeleton_joint_constraint(animation_skeleton_bind_pose_local_space, animation_skeleton_animation_pose_local_space);

    mcrt_vector<DirectX::XMFLOAT4X4> animation_skeleton_animation_pose_model_space(static_cast<size_t>(animation_skeleton_joint_count));
    for (uint32_t current_animation_skeleton_joint_index = 0; current_animation_skeleton_joint_index < static_cast<uint32_t>(animation_skeleton_joint_count); ++current_animation_skeleton_joint_index)
    {
        uint32_t const parent_animation_skeleton_joint_index = this->m_skeleton->get_animation_skeleton_joint_parent_indices()[current_animation_skeleton_joint_index];
        if (BRX_MOTION_UINT32_INDEX_INVALID != parent_animation_skeleton_joint_index)
        {
            assert(parent_animation_skeleton_joint_index < current_animation_skeleton_joint_index);
            DirectX::XMStoreFloat4x4(&animation_skeleton_animation_pose_model_space[current_animation_skeleton_joint_index], DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&animation_skeleton_animation_pose_local_space[current_animation_skeleton_joint_index]), DirectX::XMLoadFloat4x4(&animation_skeleton_animation_pose_model_space[parent_animation_skeleton_joint_index])));
        }
        else
        {
            animation_skeleton_animation_pose_model_space[current_animation_skeleton_joint_index] = animation_skeleton_animation_pose_local_space[current_animation_skeleton_joint_index];
        }
    }

    if ((BRX_MOTION_PHYSICS_RAGDOLL_QUALITY_LOW == physics_ragdoll_quality) || (BRX_MOTION_PHYSICS_RAGDOLL_QUALITY_MEDIUM == physics_ragdoll_quality) || (BRX_MOTION_PHYSICS_RAGDOLL_QUALITY_HIGH == physics_ragdoll_quality))
    {
        // TODO: implement the physics synchronization

        float physics_delta_time;
        uint32_t physics_max_substep_count;
        float physics_substep_delta_time;
        if (NULL != this->m_input_animation_instance)
        {
            if (this->m_input_continuous)
            {
                if (this->m_input_animation_instance->get_continuous())
                {
                    physics_delta_time = this->m_input_animation_instance->get_delta_time();

                    // TODO: select the best configuration???
                    switch (physics_ragdoll_quality)
                    {
                    case BRX_MOTION_PHYSICS_RAGDOLL_QUALITY_LOW:
                    {
                        physics_max_substep_count = 3U;
                        physics_substep_delta_time = 1.0 / 30.0F;
                    }
                    break;
                    case BRX_MOTION_PHYSICS_RAGDOLL_QUALITY_MEDIUM:
                    {
                        physics_max_substep_count = 6U;
                        physics_substep_delta_time = 1.0 / 60.0F;
                    }
                    break;
                    default:
                    {
                        assert(BRX_MOTION_PHYSICS_RAGDOLL_QUALITY_HIGH == physics_ragdoll_quality);
                        physics_max_substep_count = 9U;
                        physics_substep_delta_time = 1.0 / 90.0F;
                    }
                    }
                }
                else
                {
                    physics_delta_time = INTERNAL_PHYSICS_SYNCHRONIZATION_DELTA_TIME;
                    physics_max_substep_count = INTERNAL_PHYSICS_SYNCHRONIZATION_MAXIMUM_SUBSTEP_COUNT;
                    physics_substep_delta_time = INTERNAL_PHYSICS_SYNCHRONIZATION_SUBSTEP_DELTA_TIME;
                }
            }
            else
            {
                physics_delta_time = INTERNAL_PHYSICS_SYNCHRONIZATION_DELTA_TIME;
                physics_max_substep_count = INTERNAL_PHYSICS_SYNCHRONIZATION_MAXIMUM_SUBSTEP_COUNT;
                physics_substep_delta_time = INTERNAL_PHYSICS_SYNCHRONIZATION_SUBSTEP_DELTA_TIME;
                this->m_input_continuous = true;
            }
        }
        else
        {
            if (this->m_input_continuous)
            {
                physics_delta_time = INTERNAL_FRAME_DELTA_TIME;
                physics_max_substep_count = INTERNAL_FRAME_MAXIMUM_SUBSTEP_COUNT;
                physics_substep_delta_time = INTERNAL_FRAME_SUBSTEP_DELTA_TIME;
            }
            else
            {
                physics_delta_time = INTERNAL_PHYSICS_SYNCHRONIZATION_DELTA_TIME;
                physics_max_substep_count = INTERNAL_PHYSICS_SYNCHRONIZATION_MAXIMUM_SUBSTEP_COUNT;
                physics_substep_delta_time = INTERNAL_PHYSICS_SYNCHRONIZATION_SUBSTEP_DELTA_TIME;
                this->m_input_continuous = true;
            }
        }

        this->ragdoll(animation_skeleton_animation_pose_model_space, physics_delta_time, physics_max_substep_count, physics_substep_delta_time);
    }
    else
    {
        assert(BRX_MOTION_PHYSICS_RAGDOLL_QUALITY_DISABLED == physics_ragdoll_quality);
        this->m_input_continuous = false;
    }

    assert(this->m_skin_transforms.size() == animation_skeleton_joint_count);

    for (uint32_t animation_skeleton_joint_index = 0; animation_skeleton_joint_index < animation_skeleton_joint_count; ++animation_skeleton_joint_index)
    {
        DirectX::XMFLOAT4 skin_rotation;
        DirectX::XMFLOAT3 skin_translation;
        {
            DirectX::XMMATRIX skin_transform = DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&this->m_skeleton->get_animation_skeleton_bind_pose_inverse_model_space()[animation_skeleton_joint_index]), DirectX::XMLoadFloat4x4(&animation_skeleton_animation_pose_model_space[animation_skeleton_joint_index]));

            DirectX::XMVECTOR skin_scale;
            DirectX::XMVECTOR simd_skin_rotation;
            DirectX::XMVECTOR simd_skin_translation;
            bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&skin_scale, &simd_skin_rotation, &simd_skin_translation, skin_transform);
            assert(directx_xm_matrix_decompose);

            assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(skin_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));

            DirectX::XMStoreFloat4(&skin_rotation, simd_skin_rotation);

            DirectX::XMStoreFloat3(&skin_translation, simd_skin_translation);
        }

        this->m_skin_transforms[animation_skeleton_joint_index].m_rotation[0] = skin_rotation.x;
        this->m_skin_transforms[animation_skeleton_joint_index].m_rotation[1] = skin_rotation.y;
        this->m_skin_transforms[animation_skeleton_joint_index].m_rotation[2] = skin_rotation.z;
        this->m_skin_transforms[animation_skeleton_joint_index].m_rotation[3] = skin_rotation.w;

        this->m_skin_transforms[animation_skeleton_joint_index].m_translation[0] = skin_translation.x;
        this->m_skin_transforms[animation_skeleton_joint_index].m_translation[1] = skin_translation.y;
        this->m_skin_transforms[animation_skeleton_joint_index].m_translation[2] = skin_translation.z;
    }
}

inline void brx_motion_animation_skeleton_instance::animation_skeleton_joint_constraint(mcrt_vector<DirectX::XMFLOAT4X4> const &animation_skeleton_bind_pose_local_space, mcrt_vector<DirectX::XMFLOAT4X4> &animation_skeleton_animation_pose_local_space)
{
    uint32_t const animation_skeleton_joint_constraint_count = this->m_skeleton->get_animation_skeleton_joint_constraint_count();

    for (uint32_t animation_skeleton_joint_constraint_index = 0U; animation_skeleton_joint_constraint_index < animation_skeleton_joint_constraint_count; ++animation_skeleton_joint_constraint_index)
    {
        BRX_MOTION_SKELETON_JOINT_CONSTRAINT_NAME const animation_skeleton_joint_constraint_name = this->m_skeleton->get_animation_skeleton_joint_constraint_names()[animation_skeleton_joint_constraint_index];

        bool animation_skeleton_joint_constraint_enable;
        {
            assert(NULL == this->m_input_video_detector || NULL == this->m_input_animation_instance);

            if (NULL != this->m_input_video_detector)
            {
                if ((BRX_MOTION_SKELETON_JOINT_CONSTRAINT_NAME_MMD_IK_RIGHT_ANKLE == animation_skeleton_joint_constraint_name) || (BRX_MOTION_SKELETON_JOINT_CONSTRAINT_NAME_MMD_IK_RIGHT_TOE_TIP == animation_skeleton_joint_constraint_name) || (BRX_MOTION_SKELETON_JOINT_CONSTRAINT_NAME_MMD_IK_LEFT_ANKLE == animation_skeleton_joint_constraint_name) || (BRX_MOTION_SKELETON_JOINT_CONSTRAINT_NAME_MMD_IK_LEFT_TOE_TIP == animation_skeleton_joint_constraint_name))
                {
                    animation_skeleton_joint_constraint_enable = false;
                }
                else
                {
                    animation_skeleton_joint_constraint_enable = true;
                }
            }
            else if (NULL != this->m_input_animation_instance)
            {
                brx_motion_animation_animation const *const input_animation = this->m_input_animation_instance->get_animation();
                assert(NULL != input_animation);

                uint8_t const *animation_skeleton_joint_constraint_switch = animation_skeleton_joint_constraint_switch = input_animation->get_switch(this->m_input_animation_instance->get_frame_index(), animation_skeleton_joint_constraint_name);

                if (NULL != animation_skeleton_joint_constraint_switch)
                {
                    animation_skeleton_joint_constraint_enable = (*animation_skeleton_joint_constraint_switch);
                }
                else
                {
                    animation_skeleton_joint_constraint_enable = true;
                }
            }
            else
            {
                animation_skeleton_joint_constraint_enable = true;
            }
        }

        if (animation_skeleton_joint_constraint_enable)
        {
            brx_animation_skeleton_joint_constraint const &animation_skeleton_joint_constraint = this->m_skeleton->get_animation_skeleton_joint_constraints()[animation_skeleton_joint_constraint_index];

            if (BRX_MOTION_SKELETON_JOINT_CONSTRAINT_COPY_TRANSFORM == animation_skeleton_joint_constraint.m_constraint_type)
            {
                // NOTE: the constraint may change the animation pose
                // the "delta rotation" may be different from the VMD animation and should be calculated
                DirectX::XMVECTOR source_delta_rotation_local_space;
                DirectX::XMVECTOR source_delta_translation_local_space;
                {
                    DirectX::XMVECTOR source_rotation_bind_pose_local_space;
                    DirectX::XMVECTOR source_translation_bind_pose_local_space;
                    {
                        DirectX::XMFLOAT4X4 source_transform_bind_pose_local_space = animation_skeleton_bind_pose_local_space[animation_skeleton_joint_constraint.m_copy_transform.m_source_joint_index];

                        DirectX::XMVECTOR source_scale_bind_pose_local_space;
                        bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&source_scale_bind_pose_local_space, &source_rotation_bind_pose_local_space, &source_translation_bind_pose_local_space, DirectX::XMLoadFloat4x4(&source_transform_bind_pose_local_space));
                        assert(directx_xm_matrix_decompose);

                        assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(source_scale_bind_pose_local_space, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
                    }

                    DirectX::XMVECTOR source_rotation_animation_pose_local_space;
                    DirectX::XMVECTOR source_translation_animation_pose_local_space;
                    {
                        DirectX::XMFLOAT4X4 source_transform_animation_pose_local_space = animation_skeleton_animation_pose_local_space[animation_skeleton_joint_constraint.m_copy_transform.m_source_joint_index];

                        DirectX::XMVECTOR source_scale_animation_pose_local_space;
                        bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&source_scale_animation_pose_local_space, &source_rotation_animation_pose_local_space, &source_translation_animation_pose_local_space, DirectX::XMLoadFloat4x4(&source_transform_animation_pose_local_space));
                        assert(directx_xm_matrix_decompose);

                        assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(source_scale_animation_pose_local_space, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
                    }

                    // bind pose rotation always zero
                    assert(DirectX::XMVector4EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(source_rotation_bind_pose_local_space, DirectX::XMQuaternionIdentity())), DirectX::XMVectorReplicate(INTERNAL_ROTATION_EPSILON))));

                    source_delta_rotation_local_space = source_rotation_animation_pose_local_space;
                    source_delta_translation_local_space = DirectX::XMVectorSubtract(source_translation_animation_pose_local_space, source_translation_bind_pose_local_space);
                }

                DirectX::XMVECTOR destination_rotation_animation_pose_local_space;
                DirectX::XMVECTOR destination_translation_animation_pose_local_space;
                {
                    DirectX::XMFLOAT4X4 destination_transform_animation_pose_local_space = animation_skeleton_animation_pose_local_space[animation_skeleton_joint_constraint.m_copy_transform.m_destination_joint_index];

                    DirectX::XMVECTOR destination_scale_animation_pose_local_space;
                    bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&destination_scale_animation_pose_local_space, &destination_rotation_animation_pose_local_space, &destination_translation_animation_pose_local_space, DirectX::XMLoadFloat4x4(&destination_transform_animation_pose_local_space));
                    assert(directx_xm_matrix_decompose);

                    assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(destination_scale_animation_pose_local_space, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
                }

                {
                    assert(animation_skeleton_joint_constraint.m_copy_transform.m_copy_rotation || animation_skeleton_joint_constraint.m_copy_transform.m_copy_translation);

                    uint32_t const source_weight_count = this->m_skeleton->get_animation_skeleton_joint_constraints_storage_size(animation_skeleton_joint_constraint_index);

                    static_assert(sizeof(float) == sizeof(uint32_t), "");
                    float const *const source_weights = reinterpret_cast<float const *>(this->m_skeleton->get_animation_skeleton_joint_constraints_storage_base(animation_skeleton_joint_constraint_index));

                    if (animation_skeleton_joint_constraint.m_copy_transform.m_copy_rotation)
                    {
                        DirectX::XMVECTOR append_rotation = source_delta_rotation_local_space;

                        for (uint32_t source_weight_index = 0U; source_weight_index < source_weight_count; ++source_weight_index)
                        {
                            append_rotation = DirectX::XMQuaternionSlerp(DirectX::XMQuaternionIdentity(), append_rotation, source_weights[source_weight_index]);
                        }

                        destination_rotation_animation_pose_local_space = DirectX::XMQuaternionMultiply(append_rotation, destination_rotation_animation_pose_local_space);
                    }

                    if (animation_skeleton_joint_constraint.m_copy_transform.m_copy_translation)
                    {
                        DirectX::XMVECTOR append_translation = source_delta_translation_local_space;

                        for (uint32_t source_weight_index = 0U; source_weight_index < source_weight_count; ++source_weight_index)
                        {
                            append_translation = DirectX::XMVectorScale(append_translation, source_weights[source_weight_index]);
                        }

                        destination_translation_animation_pose_local_space = DirectX::XMVectorAdd(append_translation, destination_translation_animation_pose_local_space);
                    }
                }

                DirectX::XMStoreFloat4x4(&animation_skeleton_animation_pose_local_space[animation_skeleton_joint_constraint.m_copy_transform.m_destination_joint_index], DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(destination_rotation_animation_pose_local_space), DirectX::XMMatrixTranslationFromVector(destination_translation_animation_pose_local_space)));
            }
            else
            {
                assert(BRX_MOTION_SKELETON_JOINT_CONSTRAINT_INVERSE_KINEMATICS == animation_skeleton_joint_constraint.m_constraint_type);

                DirectX::XMFLOAT3 target_position_model_space;
                {
                    DirectX::XMFLOAT4X4 const target_transform_model_space = internal_calculate_transform_model_space(this->m_skeleton->get_animation_skeleton_joint_parent_indices(), animation_skeleton_animation_pose_local_space.data(), animation_skeleton_joint_constraint.m_inverse_kinematics.m_target_joint_index);

                    DirectX::XMVECTOR target_scale_model_space;
                    DirectX::XMVECTOR target_rotation_model_space;
                    DirectX::XMVECTOR target_translation_model_space;
                    bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&target_scale_model_space, &target_rotation_model_space, &target_translation_model_space, DirectX::XMLoadFloat4x4(&target_transform_model_space));
                    assert(directx_xm_matrix_decompose);

                    assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(target_scale_model_space, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));

                    DirectX::XMStoreFloat3(&target_position_model_space, target_translation_model_space);
                }

                uint32_t const ik_joint_count = this->m_skeleton->get_animation_skeleton_joint_constraints_storage_size(animation_skeleton_joint_constraint_index);

                uint32_t const *const ik_joint_indices = this->m_skeleton->get_animation_skeleton_joint_constraints_storage_base(animation_skeleton_joint_constraint_index);

                mcrt_vector<DirectX::XMFLOAT4X4> ik_joints_local_space(static_cast<size_t>(ik_joint_count));
                mcrt_vector<DirectX::XMFLOAT4X4> ik_joints_model_space(static_cast<size_t>(ik_joint_count));
                for (uint32_t ik_joint_index = 0U; ik_joint_index < ik_joint_count; ++ik_joint_index)
                {
                    uint32_t const animation_skeleton_joint_index = ik_joint_indices[ik_joint_index];
                    assert(animation_skeleton_joint_index < this->m_skeleton->get_animation_skeleton_joint_count());

                    ik_joints_model_space[ik_joint_index] = internal_calculate_transform_model_space(this->m_skeleton->get_animation_skeleton_joint_parent_indices(), animation_skeleton_animation_pose_local_space.data(), animation_skeleton_joint_index);

                    if ((0U == ik_joint_index) || (this->m_skeleton->get_animation_skeleton_joint_parent_indices()[animation_skeleton_joint_index] == ik_joint_indices[ik_joint_index - 1U]))
                    {
                        ik_joints_local_space[ik_joint_index] = animation_skeleton_animation_pose_local_space[animation_skeleton_joint_index];
                    }
                    else
                    {
                        assert(false);

                        DirectX::XMVECTOR unused_determinant;
                        DirectX::XMStoreFloat4x4(&ik_joints_local_space[ik_joint_index], DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&ik_joints_model_space[ik_joint_index]), DirectX::XMMatrixInverse(&unused_determinant, DirectX::XMLoadFloat4x4(&ik_joints_model_space[ik_joint_index - 1U]))));
                    }
                }

                DirectX::XMFLOAT4X4 end_effector_transform_local_space;
                {
                    uint32_t const end_effector_animation_skeleton_joint_index = animation_skeleton_joint_constraint.m_inverse_kinematics.m_ik_end_effector_index;

                    if (this->m_skeleton->get_animation_skeleton_joint_parent_indices()[end_effector_animation_skeleton_joint_index] == ik_joint_indices[ik_joint_count - 1U])
                    {
                        end_effector_transform_local_space = animation_skeleton_animation_pose_local_space[end_effector_animation_skeleton_joint_index];
                    }
                    else
                    {
                        assert(false);

                        DirectX::XMFLOAT4X4 const end_effector_transform_model_space = internal_calculate_transform_model_space(this->m_skeleton->get_animation_skeleton_joint_parent_indices(), animation_skeleton_animation_pose_local_space.data(), end_effector_animation_skeleton_joint_index);

                        DirectX::XMVECTOR unused_determinant;
                        DirectX::XMStoreFloat4x4(&end_effector_transform_local_space, DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&end_effector_transform_model_space), DirectX::XMMatrixInverse(&unused_determinant, DirectX::XMLoadFloat4x4(&ik_joints_model_space[ik_joint_count - 1U]))));
                    }
                }

                if (1U == ik_joint_count)
                {
                    ik_one_joint_solve(1.0F, target_position_model_space, end_effector_transform_local_space, ik_joints_local_space.data(), ik_joints_model_space.data());
                }
                else if (2U == ik_joint_count)
                {
                    ik_two_joints_solve(1.0F, animation_skeleton_joint_constraint.m_inverse_kinematics.m_ik_two_joints_hinge_joint_axis_local_space, animation_skeleton_joint_constraint.m_inverse_kinematics.m_cosine_max_ik_two_joints_hinge_joint_angle, animation_skeleton_joint_constraint.m_inverse_kinematics.m_cosine_min_ik_two_joints_hinge_joint_angle, 1.0F, target_position_model_space, end_effector_transform_local_space, ik_joints_local_space.data(), ik_joints_model_space.data());
                }
                else if (3U == ik_joint_count)
                {
                    // TODO: three joints IK
                    ik_ccd_solve(8U, 0.5F, target_position_model_space, end_effector_transform_local_space, ik_joint_count, ik_joints_local_space.data(), ik_joints_model_space.data());
                }
                else
                {

                    assert(ik_joint_count >= 4U);
                    ik_ccd_solve(8U, 0.5F, target_position_model_space, end_effector_transform_local_space, ik_joint_count, ik_joints_local_space.data(), ik_joints_model_space.data());
                }

                for (uint32_t ik_joint_index = 0U; ik_joint_index < ik_joint_count; ++ik_joint_index)
                {
                    uint32_t const animation_skeleton_joint_index = ik_joint_indices[ik_joint_index];
                    assert(animation_skeleton_joint_index < this->m_skeleton->get_animation_skeleton_joint_count());

                    if ((0U == ik_joint_index) || (this->m_skeleton->get_animation_skeleton_joint_parent_indices()[animation_skeleton_joint_index] == ik_joint_indices[ik_joint_index - 1U]))
                    {
                        animation_skeleton_animation_pose_local_space[animation_skeleton_joint_index] = ik_joints_local_space[ik_joint_index];
                    }
                    else
                    {
                        assert(false);

                        DirectX::XMVECTOR unused_determinant;
                        DirectX::XMStoreFloat4x4(&animation_skeleton_animation_pose_local_space[animation_skeleton_joint_index], DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&ik_joints_model_space[ik_joint_index]), DirectX::XMMatrixInverse(&unused_determinant, DirectX::XMLoadFloat4x4(&ik_joints_model_space[ik_joint_index - 1U]))));
                    }
                }
            }
        }
    }
}

inline void brx_motion_animation_skeleton_instance::ragdoll(mcrt_vector<DirectX::XMFLOAT4X4> &animation_skeleton_animation_pose_model_space, float physics_delta_time, uint32_t physics_max_substep_count, float physics_substep_delta_time)
{
    //
    uint32_t const animation_to_ragdoll_direct_mapping_count = this->m_skeleton->get_animation_to_ragdoll_direct_mapping_count();

    for (uint32_t animation_to_ragdoll_direct_mapping_index = 0U; animation_to_ragdoll_direct_mapping_index < animation_to_ragdoll_direct_mapping_count; ++animation_to_ragdoll_direct_mapping_index)
    {
        brx_animation_ragdoll_direct_mapping const &ragdoll_direct_mapping = this->m_skeleton->get_animation_to_ragdoll_direct_mappings()[animation_to_ragdoll_direct_mapping_index];

        DirectX::XMFLOAT4X4 const &animation_transform = animation_skeleton_animation_pose_model_space[ragdoll_direct_mapping.m_joint_index_a];

        DirectX::XMFLOAT4 ragdoll_rotation;
        DirectX::XMFLOAT3 ragdoll_translation;
        {
            DirectX::XMMATRIX ragdoll_transform = DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&ragdoll_direct_mapping.m_a_to_b_transform_model_space), DirectX::XMLoadFloat4x4(&animation_transform));

            DirectX::XMVECTOR simd_ragdoll_scale;
            DirectX::XMVECTOR simd_ragdoll_rotation;
            DirectX::XMVECTOR simd_ragdoll_translation;
            bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&simd_ragdoll_scale, &simd_ragdoll_rotation, &simd_ragdoll_translation, ragdoll_transform);
            assert(directx_xm_matrix_decompose);

            assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(simd_ragdoll_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));

            DirectX::XMStoreFloat4(&ragdoll_rotation, simd_ragdoll_rotation);

            DirectX::XMStoreFloat3(&ragdoll_translation, simd_ragdoll_translation);
        }

        brx_physics_rigid_body_set_transform(g_physics_context, this->m_physics_world, this->m_physics_rigid_bodies[ragdoll_direct_mapping.m_joint_index_b], &ragdoll_rotation.x, &ragdoll_translation.x);
    }

    //
    brx_physics_world_step(g_physics_context, this->m_physics_world, physics_delta_time, physics_max_substep_count, physics_substep_delta_time);

    //
    uint32_t const ragdoll_to_animation_direct_mapping_count = this->m_skeleton->get_ragdoll_to_animation_direct_mapping_count();

    for (uint32_t ragdoll_to_animation_direct_mapping_index = 0U; ragdoll_to_animation_direct_mapping_index < ragdoll_to_animation_direct_mapping_count; ++ragdoll_to_animation_direct_mapping_index)
    {
        brx_animation_ragdoll_direct_mapping const &ragdoll_direct_mapping = this->m_skeleton->get_ragdoll_to_animation_direct_mappings()[ragdoll_to_animation_direct_mapping_index];

        DirectX::XMFLOAT4 ragdoll_rotation;
        DirectX::XMFLOAT3 ragdoll_translation;
        brx_physics_rigid_body_get_transform(g_physics_context, this->m_physics_world, this->m_physics_rigid_bodies[ragdoll_direct_mapping.m_joint_index_a], &ragdoll_rotation.x, &ragdoll_translation.x);

        DirectX::XMFLOAT4X4 animation_transform;
        {
            DirectX::XMMATRIX ragdoll_transform = DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(DirectX::XMLoadFloat4(&ragdoll_rotation)), DirectX::XMMatrixTranslationFromVector(DirectX::XMLoadFloat3(&ragdoll_translation)));

            DirectX::XMStoreFloat4x4(&animation_transform, DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&ragdoll_direct_mapping.m_a_to_b_transform_model_space), ragdoll_transform));
        }

        animation_skeleton_animation_pose_model_space[ragdoll_direct_mapping.m_joint_index_b] = animation_transform;

        // TODO: check chain mapping
        // TODO: check unmapped
    }
}

uint32_t brx_motion_animation_skeleton_instance::get_skin_transform_count() const
{
    return this->m_skin_transforms.size();
}

brx_motion_rigid_transform const *brx_motion_animation_skeleton_instance::get_skin_transforms() const
{
    return this->m_skin_transforms.data();
}

static inline void internal_video_detector_pose_ik_one_joint_solve(uint32_t const in_ball_and_socket_joint_animation_skeleton_joint_index, uint32_t const in_end_effector_animation_skeleton_joint_index, DirectX::XMFLOAT3 const &in_target_displacement_model_space, uint32_t const *const in_animation_skeleton_joint_parent_indices, DirectX::XMFLOAT4X4 const *const in_animation_skeleton_animation_pose_local_space, DirectX::XMFLOAT4 *const out_ball_and_socket_joint_rotation_local_space)
{
    DirectX::XMFLOAT4X4 const ball_and_socket_joint_transform_model_space = internal_calculate_transform_model_space(in_animation_skeleton_joint_parent_indices, in_animation_skeleton_animation_pose_local_space, in_ball_and_socket_joint_animation_skeleton_joint_index);

    DirectX::XMFLOAT4X4 inout_joints_local_space[1];
    DirectX::XMFLOAT4X4 inout_joints_model_space[1];
    inout_joints_local_space[0] = in_animation_skeleton_animation_pose_local_space[in_ball_and_socket_joint_animation_skeleton_joint_index];
    inout_joints_model_space[0] = ball_and_socket_joint_transform_model_space;

    DirectX::XMFLOAT3 target_position_model_space;
    {
        DirectX::XMVECTOR ball_and_socket_joint_translation_model_space;
        {
            DirectX::XMVECTOR ball_and_socket_joint_scale_model_space;
            DirectX::XMVECTOR ball_and_socket_joint_rotation_model_space;
            bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&ball_and_socket_joint_scale_model_space, &ball_and_socket_joint_rotation_model_space, &ball_and_socket_joint_translation_model_space, DirectX::XMLoadFloat4x4(&ball_and_socket_joint_transform_model_space));
            assert(directx_xm_matrix_decompose);

            assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(ball_and_socket_joint_scale_model_space, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
        }

        // TODO: do we really need to scale to make sure the translation is high enough?
        DirectX::XMStoreFloat3(&target_position_model_space, DirectX::XMVectorAdd(ball_and_socket_joint_translation_model_space, DirectX::XMLoadFloat3(&in_target_displacement_model_space)));
    }

    DirectX::XMFLOAT4X4 end_effector_transform_local_space;
    if (in_animation_skeleton_joint_parent_indices[in_end_effector_animation_skeleton_joint_index] == in_ball_and_socket_joint_animation_skeleton_joint_index)
    {
        end_effector_transform_local_space = in_animation_skeleton_animation_pose_local_space[in_end_effector_animation_skeleton_joint_index];
    }
    else
    {
        assert(in_animation_skeleton_joint_parent_indices[in_animation_skeleton_joint_parent_indices[in_end_effector_animation_skeleton_joint_index]] == in_ball_and_socket_joint_animation_skeleton_joint_index);

        DirectX::XMFLOAT4X4 const end_effector_transform_model_space = internal_calculate_transform_model_space(in_animation_skeleton_joint_parent_indices, in_animation_skeleton_animation_pose_local_space, in_end_effector_animation_skeleton_joint_index);

        DirectX::XMVECTOR unused_determinant;
        DirectX::XMStoreFloat4x4(&end_effector_transform_local_space, DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&end_effector_transform_model_space), DirectX::XMMatrixInverse(&unused_determinant, DirectX::XMLoadFloat4x4(&ball_and_socket_joint_transform_model_space))));
    }

    ik_one_joint_solve(1.0F, target_position_model_space, end_effector_transform_local_space, inout_joints_local_space, inout_joints_model_space);

    DirectX::XMFLOAT4X4 const &ball_and_socket_joint_transform_local_space = inout_joints_local_space[0];

    DirectX::XMVECTOR ball_and_socket_joint_rotation_local_space;
    {
        DirectX::XMVECTOR ball_and_socket_joint_scale_local_space;
        DirectX::XMVECTOR ball_and_socket_joint_translation_local_space;
        bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&ball_and_socket_joint_scale_local_space, &ball_and_socket_joint_rotation_local_space, &ball_and_socket_joint_translation_local_space, DirectX::XMLoadFloat4x4(&ball_and_socket_joint_transform_local_space));
        assert(directx_xm_matrix_decompose);

        assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(ball_and_socket_joint_scale_local_space, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
    }

    DirectX::XMStoreFloat4(out_ball_and_socket_joint_rotation_local_space, ball_and_socket_joint_rotation_local_space);
}

static inline DirectX::XMFLOAT4X4 internal_calculate_transform_model_space(uint32_t const *const in_animation_skeleton_joint_parent_indices, DirectX::XMFLOAT4X4 const *const in_animation_skeleton_local_space, uint32_t const in_animation_skeleton_joint_index)
{
    uint32_t current_animation_skeleton_joint_index = in_animation_skeleton_joint_index;

    mcrt_vector<uint32_t> ancestors;
    while (BRX_MOTION_UINT32_INDEX_INVALID != current_animation_skeleton_joint_index)
    {
        ancestors.push_back(current_animation_skeleton_joint_index);
        current_animation_skeleton_joint_index = in_animation_skeleton_joint_parent_indices[current_animation_skeleton_joint_index];
    }

    assert(!ancestors.empty());
    DirectX::XMFLOAT4X4 model_space = in_animation_skeleton_local_space[ancestors.back()];
    ancestors.pop_back();

    while (!ancestors.empty())
    {
        DirectX::XMStoreFloat4x4(&model_space, DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&in_animation_skeleton_local_space[ancestors.back()]), DirectX::XMLoadFloat4x4(&model_space)));
        ancestors.pop_back();
    }

    return model_space;
}
