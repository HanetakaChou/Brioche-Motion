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

#include "brx_animation_ik_one_joint.h"
#include "brx_animation_ik_internal.inl"

extern void ik_one_joint_solve(float const in_ball_and_socket_joint_gain, DirectX::XMFLOAT3 const &in_target_position_model_space, DirectX::XMFLOAT4X4 const &in_end_effector_transform_local_space, DirectX::XMFLOAT4X4 *const inout_joint_local_space, DirectX::XMFLOAT4X4 *const inout_joint_model_space)
{
    constexpr uint32_t const ball_and_socket_joint_index = 0U;
    constexpr uint32_t const end_effector_parent_joint_index = 0U;

    constexpr float const INTERNAL_SCALE_EPSILON = 9E-5F;
    constexpr float const INTERNAL_TRANSLATION_EPSILON = 7E-5F;
    constexpr float const INTERNAL_LENGTH_EPSILON = 1E-6F;

    DirectX::XMMATRIX ball_and_socket_parent_transform_model_space;
    {
        DirectX::XMVECTOR unused_determinant;
        ball_and_socket_parent_transform_model_space = DirectX::XMMatrixMultiply(DirectX::XMMatrixInverse(&unused_determinant, DirectX::XMLoadFloat4x4(&inout_joint_local_space[ball_and_socket_joint_index])), DirectX::XMLoadFloat4x4(&inout_joint_model_space[ball_and_socket_joint_index]));
    }

    DirectX::XMVECTOR ball_and_socket_joint_model_space_translation;
    DirectX::XMVECTOR ball_and_socket_joint_model_space_rotation;
    {
        DirectX::XMMATRIX ball_and_socket_joint_transform_model_space = DirectX::XMLoadFloat4x4(&inout_joint_model_space[ball_and_socket_joint_index]);

        DirectX::XMVECTOR ball_and_socket_joint_model_space_scale;
        bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&ball_and_socket_joint_model_space_scale, &ball_and_socket_joint_model_space_rotation, &ball_and_socket_joint_model_space_translation, ball_and_socket_joint_transform_model_space);
        assert(directx_xm_matrix_decompose);

        assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(ball_and_socket_joint_model_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
    }

    DirectX::XMVECTOR from_end_effector_to_target_model_space_rotation;
    {
        DirectX::XMVECTOR end_effector_model_space_translation;
        {
            DirectX::XMMATRIX end_effector_transform_model_space = DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&in_end_effector_transform_local_space), DirectX::XMLoadFloat4x4(&inout_joint_model_space[end_effector_parent_joint_index]));

            DirectX::XMVECTOR end_effector_model_space_scale;
            DirectX::XMVECTOR end_effector_model_space_rotation;
            bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&end_effector_model_space_scale, &end_effector_model_space_rotation, &end_effector_model_space_translation, end_effector_transform_model_space);
            assert(directx_xm_matrix_decompose);

            assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(end_effector_model_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
        }

        DirectX::XMVECTOR end_effector_model_space_displacement = DirectX::XMVectorSubtract(end_effector_model_space_translation, ball_and_socket_joint_model_space_translation);

        DirectX::XMVECTOR target_model_space_displacement = DirectX::XMVectorSubtract(DirectX::XMLoadFloat3(&in_target_position_model_space), ball_and_socket_joint_model_space_translation);

        float end_effector_model_space_displacement_length = DirectX::XMVectorGetX(DirectX::XMVector3Length(end_effector_model_space_displacement));

        float target_model_space_displacement_length = DirectX::XMVectorGetX(DirectX::XMVector3Length(target_model_space_displacement));

        if ((end_effector_model_space_displacement_length > INTERNAL_LENGTH_EPSILON) && (target_model_space_displacement_length > INTERNAL_LENGTH_EPSILON))
        {
            assert((in_ball_and_socket_joint_gain >= 0.0F) && (in_ball_and_socket_joint_gain <= 1.0F));

            DirectX::XMVECTOR end_effector_model_space_direction = DirectX::XMVectorScale(end_effector_model_space_displacement, 1.0F / end_effector_model_space_displacement_length);

            DirectX::XMVECTOR target_model_space_direction = DirectX::XMVectorScale(target_model_space_displacement, 1.0F / target_model_space_displacement_length);

            from_end_effector_to_target_model_space_rotation = internal_compute_shortest_rotation_damped(end_effector_model_space_direction, target_model_space_direction, in_ball_and_socket_joint_gain);
        }
        else
        {
            assert(false);
            
            from_end_effector_to_target_model_space_rotation = DirectX::XMQuaternionIdentity();
        }
    }

    DirectX::XMVECTOR updated_ball_and_socket_joint_model_space_rotation = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(ball_and_socket_joint_model_space_rotation, from_end_effector_to_target_model_space_rotation));

    DirectX::XMMATRIX updated_ball_and_socket_joint_model_space_transform = DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(updated_ball_and_socket_joint_model_space_rotation), DirectX::XMMatrixTranslationFromVector(ball_and_socket_joint_model_space_translation));

    DirectX::XMStoreFloat4x4(&inout_joint_model_space[ball_and_socket_joint_index], updated_ball_and_socket_joint_model_space_transform);

    DirectX::XMVECTOR ball_and_socket_joint_local_space_translation;
    {
        DirectX::XMVECTOR ball_and_socket_joint_local_space_scale;
        DirectX::XMVECTOR ball_and_socket_joint_local_space_rotation;
        bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&ball_and_socket_joint_local_space_scale, &ball_and_socket_joint_local_space_rotation, &ball_and_socket_joint_local_space_translation, DirectX::XMLoadFloat4x4(&inout_joint_local_space[ball_and_socket_joint_index]));
        assert(directx_xm_matrix_decompose);

        assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(ball_and_socket_joint_local_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
    }

    DirectX::XMVECTOR updated_ball_and_socket_joint_local_space_rotation;
    {
        DirectX::XMVECTOR unused_determinant;
        DirectX::XMMATRIX unused_updated_ball_and_socket_joint_local_space_transform = DirectX::XMMatrixMultiply(updated_ball_and_socket_joint_model_space_transform, DirectX::XMMatrixInverse(&unused_determinant, ball_and_socket_parent_transform_model_space));

        DirectX::XMVECTOR updated_ball_and_socket_joint_local_space_scale;
        DirectX::XMVECTOR updated_ball_and_socket_joint_local_space_translation;
        bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&updated_ball_and_socket_joint_local_space_scale, &updated_ball_and_socket_joint_local_space_rotation, &updated_ball_and_socket_joint_local_space_translation, unused_updated_ball_and_socket_joint_local_space_transform);
        assert(directx_xm_matrix_decompose);

        assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(updated_ball_and_socket_joint_local_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));

        assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(updated_ball_and_socket_joint_local_space_translation, ball_and_socket_joint_local_space_translation)), DirectX::XMVectorReplicate(INTERNAL_TRANSLATION_EPSILON))));
    }

    DirectX::XMMATRIX updated_current_joint_local_space_transform = DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(updated_ball_and_socket_joint_local_space_rotation), DirectX::XMMatrixTranslationFromVector(ball_and_socket_joint_local_space_translation));

    DirectX::XMStoreFloat4x4(&inout_joint_local_space[ball_and_socket_joint_index], updated_current_joint_local_space_transform);
}
