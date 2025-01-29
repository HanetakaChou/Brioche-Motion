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

#include "brx_animation_ik_ccd.h"
#include "brx_animation_ik_internal.inl"

static inline void internal_ik_ccd_solve_iteration(float const in_gain, DirectX::XMFLOAT3 const &in_target_position_model_space, DirectX::XMFLOAT4X4 const &in_end_effector_transform_local_space, DirectX::XMFLOAT4X4 const &in_base_parent_transform_model_space, uint32_t const in_joint_count, DirectX::XMFLOAT4X4 *const inout_joints_local_space, DirectX::XMFLOAT4X4 *const inout_joints_model_space);

extern void ik_ccd_solve(uint32_t const in_iteration, float const in_gain, DirectX::XMFLOAT3 const &in_target_position_model_space, DirectX::XMFLOAT4X4 const &in_end_effector_transform_local_space, uint32_t const in_joint_count, DirectX::XMFLOAT4X4 *const inout_joints_local_space, DirectX::XMFLOAT4X4 *const inout_joints_model_space)
{
    DirectX::XMFLOAT4X4 in_base_parent_transform_model_space;
    assert(in_joint_count >= 1U);
    {
        DirectX::XMVECTOR unused_determinant;
        DirectX::XMStoreFloat4x4(&in_base_parent_transform_model_space, DirectX::XMMatrixMultiply(DirectX::XMMatrixInverse(&unused_determinant, DirectX::XMLoadFloat4x4(&inout_joints_local_space[0])), DirectX::XMLoadFloat4x4(&inout_joints_model_space[0])));
    }

    for (uint32_t iteration_index = 0U; iteration_index < in_iteration; ++iteration_index)
    {
        internal_ik_ccd_solve_iteration(in_gain, in_target_position_model_space, in_end_effector_transform_local_space, in_base_parent_transform_model_space, in_joint_count, inout_joints_local_space, inout_joints_model_space);
    }
}

static inline void internal_ik_ccd_solve_iteration(float const in_gain, DirectX::XMFLOAT3 const &in_target_position_model_space, DirectX::XMFLOAT4X4 const &in_end_effector_transform_local_space, DirectX::XMFLOAT4X4 const &in_base_parent_transform_model_space, uint32_t const in_joint_count, DirectX::XMFLOAT4X4 *const inout_joints_local_space, DirectX::XMFLOAT4X4 *const inout_joints_model_space)
{
    constexpr float const INTERNAL_SCALE_EPSILON = 9E-5F;
    constexpr float const INTERNAL_TRANSLATION_EPSILON = 7E-5F;

    for (uint32_t current_joint_index_plus_1 = in_joint_count; current_joint_index_plus_1 >= 1U; --current_joint_index_plus_1)
    {
        uint32_t const current_joint_index = current_joint_index_plus_1 - 1U;

        DirectX::XMVECTOR current_joint_model_space_rotation;
        DirectX::XMVECTOR current_joint_model_space_translation;
        {
            DirectX::XMVECTOR current_joint_model_space_scale;
            bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&current_joint_model_space_scale, &current_joint_model_space_rotation, &current_joint_model_space_translation, DirectX::XMLoadFloat4x4(&inout_joints_model_space[current_joint_index]));
            assert(directx_xm_matrix_decompose);

            assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(current_joint_model_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
        }

        DirectX::XMVECTOR from_end_effector_to_target_model_space_rotation;
        {
            DirectX::XMVECTOR end_effector_model_space_translation;
            {
                uint32_t const end_effector_parent_joint_index = in_joint_count - 1U;

                DirectX::XMMATRIX end_effector_transform_model_space = DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&in_end_effector_transform_local_space), DirectX::XMLoadFloat4x4(&inout_joints_model_space[end_effector_parent_joint_index]));

                DirectX::XMVECTOR end_effector_model_space_scale;
                DirectX::XMVECTOR end_effector_model_space_rotation;
                bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&end_effector_model_space_scale, &end_effector_model_space_rotation, &end_effector_model_space_translation, end_effector_transform_model_space);
                assert(directx_xm_matrix_decompose);

                assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(end_effector_model_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
            }

            DirectX::XMVECTOR end_effector_model_space_direction = DirectX::XMVector3Normalize(DirectX::XMVectorSubtract(end_effector_model_space_translation, current_joint_model_space_translation));

            DirectX::XMVECTOR target_model_space_direction = DirectX::XMVector3Normalize(DirectX::XMVectorSubtract(DirectX::XMLoadFloat3(&in_target_position_model_space), current_joint_model_space_translation));

            assert((in_gain >= 0.0F) && (in_gain <= 1.0F));
            from_end_effector_to_target_model_space_rotation = internal_compute_shortest_rotation_damped(end_effector_model_space_direction, target_model_space_direction, in_gain);
        }

        DirectX::XMVECTOR updated_current_joint_model_space_rotation = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(current_joint_model_space_rotation, from_end_effector_to_target_model_space_rotation));

        DirectX::XMMATRIX updated_current_joint_model_space_transform = DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(updated_current_joint_model_space_rotation), DirectX::XMMatrixTranslationFromVector(current_joint_model_space_translation));

        DirectX::XMStoreFloat4x4(&inout_joints_model_space[current_joint_index], updated_current_joint_model_space_transform);

        DirectX::XMVECTOR current_joint_local_space_translation;
        {
            DirectX::XMVECTOR current_joint_local_space_scale;
            DirectX::XMVECTOR current_joint_local_space_rotation;
            bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&current_joint_local_space_scale, &current_joint_local_space_rotation, &current_joint_local_space_translation, DirectX::XMLoadFloat4x4(&inout_joints_local_space[current_joint_index]));
            assert(directx_xm_matrix_decompose);

            assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(current_joint_local_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
        }

        DirectX::XMVECTOR updated_current_joint_local_space_rotation;
        {
            DirectX::XMMATRIX current_parent_joint_model_space = (current_joint_index >= 1U) ? DirectX::XMLoadFloat4x4(&inout_joints_model_space[current_joint_index - 1U]) : DirectX::XMLoadFloat4x4(&in_base_parent_transform_model_space);

            DirectX::XMVECTOR unused_determinant;

            DirectX::XMMATRIX unused_updated_current_joint_local_space_transform = DirectX::XMMatrixMultiply(updated_current_joint_model_space_transform, DirectX::XMMatrixInverse(&unused_determinant, current_parent_joint_model_space));

            DirectX::XMVECTOR updated_current_joint_local_space_scale;
            DirectX::XMVECTOR updated_current_joint_local_space_translation;
            bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&updated_current_joint_local_space_scale, &updated_current_joint_local_space_rotation, &updated_current_joint_local_space_translation, unused_updated_current_joint_local_space_transform);
            assert(directx_xm_matrix_decompose);

            assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(updated_current_joint_local_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));

            assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(updated_current_joint_local_space_translation, current_joint_local_space_translation)), DirectX::XMVectorReplicate(INTERNAL_TRANSLATION_EPSILON))));
        }

        DirectX::XMMATRIX updated_current_joint_local_space_transform = DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(updated_current_joint_local_space_rotation), DirectX::XMMatrixTranslationFromVector(current_joint_local_space_translation));

        DirectX::XMStoreFloat4x4(&inout_joints_local_space[current_joint_index], updated_current_joint_local_space_transform);

        for (uint32_t child_joint_index_plus_1 = (current_joint_index_plus_1 + 1U); child_joint_index_plus_1 <= in_joint_count; ++child_joint_index_plus_1)
        {
            uint32_t const parent_joint_index = child_joint_index_plus_1 - 1U - 1U;
            uint32_t const child_joint_index = child_joint_index_plus_1 - 1U;
            DirectX::XMStoreFloat4x4(&inout_joints_model_space[child_joint_index], DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&inout_joints_local_space[child_joint_index]), DirectX::XMLoadFloat4x4(&inout_joints_model_space[parent_joint_index])));
        }
    }
}
