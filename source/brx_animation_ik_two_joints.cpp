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

#include "brx_animation_ik_two_joints.h"
#include "brx_animation_ik_internal.inl"
#include <algorithm>

extern void ik_two_joints_solve(float const in_ball_and_socket_joint_gain, DirectX::XMFLOAT3 const &in_hinge_joint_axis_local_space, float const in_cosine_max_hinge_joint_angle, float const in_cosine_min_hinge_joint_angle, float const in_hinge_joint_gain, DirectX::XMFLOAT3 const &in_target_position_model_space, DirectX::XMFLOAT4X4 const &in_end_effector_transform_local_space, DirectX::XMFLOAT4X4 *const inout_joints_local_space, DirectX::XMFLOAT4X4 *const inout_joints_model_space)
{
    constexpr uint32_t const ball_and_socket_joint_index = 0U;
    constexpr uint32_t const hinge_joint_index = 1U;
    constexpr uint32_t const end_effector_parent_joint_index = 1U;

    constexpr float const INTERNAL_SCALE_EPSILON = 1E-4F;
    constexpr float const INTERNAL_TRANSLATION_EPSILON = 7E-5F;

    DirectX::XMMATRIX ball_and_socket_parent_transform_model_space;
    {
        DirectX::XMVECTOR unused_determinant;
        ball_and_socket_parent_transform_model_space = DirectX::XMMatrixMultiply(DirectX::XMMatrixInverse(&unused_determinant, DirectX::XMLoadFloat4x4(&inout_joints_local_space[ball_and_socket_joint_index])), DirectX::XMLoadFloat4x4(&inout_joints_model_space[ball_and_socket_joint_index]));
    }

    DirectX::XMVECTOR ball_and_socket_joint_model_space_translation;
    DirectX::XMVECTOR ball_and_socket_joint_model_space_rotation;
    {
        DirectX::XMMATRIX ball_and_socket_joint_transform_model_space = DirectX::XMLoadFloat4x4(&inout_joints_model_space[ball_and_socket_joint_index]);

        DirectX::XMVECTOR ball_and_socket_joint_model_space_scale;
        bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&ball_and_socket_joint_model_space_scale, &ball_and_socket_joint_model_space_rotation, &ball_and_socket_joint_model_space_translation, ball_and_socket_joint_transform_model_space);
        assert(directx_xm_matrix_decompose);

        assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(ball_and_socket_joint_model_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
    }

    DirectX::XMVECTOR from_ball_and_socket_joint_to_target_model_space_translation = DirectX::XMVectorSubtract(DirectX::XMLoadFloat3(&in_target_position_model_space), ball_and_socket_joint_model_space_translation);

    {
        DirectX::XMVECTOR hinge_joint_model_space_translation;
        DirectX::XMVECTOR hinge_joint_model_space_rotation;
        DirectX::XMVECTOR hinge_joint_axis_model_space;
        {
            DirectX::XMMATRIX hinge_joint_transform_model_space = DirectX::XMLoadFloat4x4(&inout_joints_model_space[hinge_joint_index]);

            DirectX::XMVECTOR hinge_joint_model_space_scale;
            bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&hinge_joint_model_space_scale, &hinge_joint_model_space_rotation, &hinge_joint_model_space_translation, hinge_joint_transform_model_space);
            assert(directx_xm_matrix_decompose);

            assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(hinge_joint_model_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));

            DirectX::XMVECTOR hinge_joint_axis_local_space = DirectX::XMLoadFloat3(&in_hinge_joint_axis_local_space);

            hinge_joint_axis_model_space = DirectX::XMVector3Normalize(DirectX::XMVector3TransformNormal(hinge_joint_axis_local_space, hinge_joint_transform_model_space));
        }

        DirectX::XMVECTOR different_hinge_joint_model_space_rotation;
        {
            DirectX::XMVECTOR end_effector_model_space_translation;
            {
                DirectX::XMMATRIX end_effector_transform_model_space = DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&in_end_effector_transform_local_space), DirectX::XMLoadFloat4x4(&inout_joints_model_space[end_effector_parent_joint_index]));

                DirectX::XMVECTOR end_effector_model_space_scale;
                DirectX::XMVECTOR end_effector_model_space_rotation;
                bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&end_effector_model_space_scale, &end_effector_model_space_rotation, &end_effector_model_space_translation, end_effector_transform_model_space);
                assert(directx_xm_matrix_decompose);

                assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(end_effector_model_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
            }

            DirectX::XMVECTOR v1 = DirectX::XMVectorSubtract(hinge_joint_model_space_translation, ball_and_socket_joint_model_space_translation);

            DirectX::XMVECTOR v1_in_normal = DirectX::XMVectorScale(hinge_joint_axis_model_space, DirectX::XMVectorGetX(DirectX::XMVector3Dot(v1, hinge_joint_axis_model_space)));

            DirectX::XMVECTOR v1_in_plane = DirectX::XMVectorSubtract(v1, v1_in_normal);

            float v1_in_plane_length_square = DirectX::XMVectorGetX(DirectX::XMVector3Dot(v1_in_plane, v1_in_plane));

            float v1_in_plane_length = internal_sqrt(v1_in_plane_length_square);

            DirectX::XMVECTOR v2 = DirectX::XMVectorSubtract(end_effector_model_space_translation, hinge_joint_model_space_translation);

            DirectX::XMVECTOR v2_in_normal = DirectX::XMVectorScale(hinge_joint_axis_model_space, DirectX::XMVectorGetX(DirectX::XMVector3Dot(v2, hinge_joint_axis_model_space)));

            DirectX::XMVECTOR v2_in_plane = DirectX::XMVectorSubtract(v2, v2_in_normal);

            float v2_in_plane_length_square = DirectX::XMVectorGetX(DirectX::XMVector3Dot(v2_in_plane, v2_in_plane));

            float v2_in_plane_length = internal_sqrt(v2_in_plane_length_square);

            DirectX::XMVECTOR h_in_normal = DirectX::XMVectorAdd(v1_in_normal, v2_in_normal);

            float h_in_normal_length_square = DirectX::XMVectorGetX(DirectX::XMVector3Dot(h_in_normal, h_in_normal));

            float d_length_square = DirectX::XMVectorGetX(DirectX::XMVector3Dot(from_ball_and_socket_joint_to_target_model_space_translation, from_ball_and_socket_joint_to_target_model_space_translation));

            float d_in_plane_length_square = d_length_square - h_in_normal_length_square;

            float cos_v1_v2_in_plane = (v1_in_plane_length_square + v2_in_plane_length_square - d_in_plane_length_square) * 0.5F / (v1_in_plane_length * v2_in_plane_length);

            assert(in_cosine_max_hinge_joint_angle < in_cosine_min_hinge_joint_angle);
            assert(in_cosine_max_hinge_joint_angle >= -1.0F);
            assert(in_cosine_max_hinge_joint_angle <= 1.0F);
            assert(in_cosine_min_hinge_joint_angle >= -1.0F);
            assert(in_cosine_min_hinge_joint_angle <= 1.0F);
            float cos_updated = std::min(std::max(in_cosine_max_hinge_joint_angle, cos_v1_v2_in_plane), in_cosine_min_hinge_joint_angle);

            float cos_current = DirectX::XMVectorGetX(DirectX::XMVector3Dot(DirectX::XMVectorNegate(v1_in_plane), v2_in_plane)) / (v1_in_plane_length * v2_in_plane_length);

            assert(DirectX::XMVectorGetX(DirectX::XMVector3Dot(hinge_joint_axis_model_space, DirectX::XMVector3Normalize(DirectX::XMVector3Cross(DirectX::XMVectorNegate(v1_in_plane), v2_in_plane)))) > (1.0F - 1E-6F));

            float cos_different;
            float sin_different;
            {
                float const cos_b = cos_updated;
                float const cos_a = cos_current;

                float sin_b_sqr = 1.0F - cos_b * cos_b;
                float sin_a_sqr = 1.0F - cos_a * cos_a;

                float sin_b = internal_sqrt(sin_b_sqr);
                float sin_a = internal_sqrt(sin_a_sqr);

                float cos_b_a = cos_b * cos_a + sin_b * sin_a;
                float sin_b_a = sin_b * cos_a - sin_a * cos_b;

                cos_different = cos_b_a;
                sin_different = sin_b_a;
            }

            assert((in_hinge_joint_gain >= 0.0F) && (in_hinge_joint_gain <= 1.0F));
            different_hinge_joint_model_space_rotation = internal_compute_rotation_axis_damped(hinge_joint_axis_model_space, cos_different, sin_different, in_hinge_joint_gain);
        }

        DirectX::XMVECTOR updated_hinge_joint_model_space_rotation = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(hinge_joint_model_space_rotation, different_hinge_joint_model_space_rotation));

        DirectX::XMMATRIX updated_hinge_joint_model_space_transform = DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(updated_hinge_joint_model_space_rotation), DirectX::XMMatrixTranslationFromVector(hinge_joint_model_space_translation));

        DirectX::XMStoreFloat4x4(&inout_joints_model_space[hinge_joint_index], updated_hinge_joint_model_space_transform);

        DirectX::XMVECTOR hinge_joint_local_space_translation;
        {
            DirectX::XMVECTOR hinge_joint_local_space_scale;
            DirectX::XMVECTOR hinge_joint_local_space_rotation;
            bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&hinge_joint_local_space_scale, &hinge_joint_local_space_rotation, &hinge_joint_local_space_translation, DirectX::XMLoadFloat4x4(&inout_joints_local_space[hinge_joint_index]));
            assert(directx_xm_matrix_decompose);

            assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(hinge_joint_local_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
        }

        DirectX::XMVECTOR updated_hinge_joint_local_space_rotation;
        {
            DirectX::XMMATRIX ball_and_socket_joint_transform_model_space = DirectX::XMLoadFloat4x4(&inout_joints_model_space[ball_and_socket_joint_index]);

            DirectX::XMVECTOR unused_determinant;
            DirectX::XMMATRIX unused_updated_hinge_joint_local_space_transform = DirectX::XMMatrixMultiply(updated_hinge_joint_model_space_transform, DirectX::XMMatrixInverse(&unused_determinant, ball_and_socket_joint_transform_model_space));

            DirectX::XMVECTOR updated_hinge_joint_local_space_scale;
            DirectX::XMVECTOR updated_hinge_joint_local_space_translation;
            bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&updated_hinge_joint_local_space_scale, &updated_hinge_joint_local_space_rotation, &updated_hinge_joint_local_space_translation, unused_updated_hinge_joint_local_space_transform);
            assert(directx_xm_matrix_decompose);

            assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(updated_hinge_joint_local_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));

            constexpr float const INTERNAL_TRANSLATION_EPSILON = 7E-5F;
            assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(updated_hinge_joint_local_space_translation, hinge_joint_local_space_translation)), DirectX::XMVectorReplicate(INTERNAL_TRANSLATION_EPSILON))));
        }

        DirectX::XMMATRIX updated_hinge_joint_local_space_transform = DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(updated_hinge_joint_local_space_rotation), DirectX::XMMatrixTranslationFromVector(hinge_joint_local_space_translation));

        DirectX::XMStoreFloat4x4(&inout_joints_local_space[hinge_joint_index], updated_hinge_joint_local_space_transform);
    }

    {
        DirectX::XMVECTOR from_end_effector_to_target_model_space_rotation;
        {
            DirectX::XMVECTOR end_effector_model_space_translation;
            {
                DirectX::XMMATRIX end_effector_transform_model_space = DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&in_end_effector_transform_local_space), DirectX::XMLoadFloat4x4(&inout_joints_model_space[end_effector_parent_joint_index]));

                DirectX::XMVECTOR end_effector_model_space_scale;
                DirectX::XMVECTOR end_effector_model_space_rotation;
                bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&end_effector_model_space_scale, &end_effector_model_space_rotation, &end_effector_model_space_translation, end_effector_transform_model_space);
                assert(directx_xm_matrix_decompose);

                assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(end_effector_model_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
            }

            DirectX::XMVECTOR end_effector_model_space_direction = DirectX::XMVector3Normalize(DirectX::XMVectorSubtract(end_effector_model_space_translation, ball_and_socket_joint_model_space_translation));

            DirectX::XMVECTOR target_model_space_direction = DirectX::XMVector3Normalize(from_ball_and_socket_joint_to_target_model_space_translation);

            assert((in_ball_and_socket_joint_gain >= 0.0F) && (in_ball_and_socket_joint_gain <= 1.0F));
            from_end_effector_to_target_model_space_rotation = internal_compute_shortest_rotation_damped(end_effector_model_space_direction, target_model_space_direction, in_ball_and_socket_joint_gain);
        }

        DirectX::XMVECTOR updated_ball_and_socket_joint_model_space_rotation = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(ball_and_socket_joint_model_space_rotation, from_end_effector_to_target_model_space_rotation));

        DirectX::XMMATRIX updated_ball_and_socket_joint_model_space_transform = DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(updated_ball_and_socket_joint_model_space_rotation), DirectX::XMMatrixTranslationFromVector(ball_and_socket_joint_model_space_translation));

        DirectX::XMStoreFloat4x4(&inout_joints_model_space[ball_and_socket_joint_index], updated_ball_and_socket_joint_model_space_transform);

        DirectX::XMVECTOR ball_and_socket_joint_local_space_translation;
        {
            DirectX::XMVECTOR ball_and_socket_joint_local_space_scale;
            DirectX::XMVECTOR ball_and_socket_joint_local_space_rotation;
            bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&ball_and_socket_joint_local_space_scale, &ball_and_socket_joint_local_space_rotation, &ball_and_socket_joint_local_space_translation, DirectX::XMLoadFloat4x4(&inout_joints_local_space[ball_and_socket_joint_index]));
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

        DirectX::XMStoreFloat4x4(&inout_joints_local_space[ball_and_socket_joint_index], updated_current_joint_local_space_transform);

        DirectX::XMStoreFloat4x4(&inout_joints_model_space[hinge_joint_index], DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&inout_joints_local_space[hinge_joint_index]), DirectX::XMLoadFloat4x4(&inout_joints_model_space[ball_and_socket_joint_index])));
    }
}
