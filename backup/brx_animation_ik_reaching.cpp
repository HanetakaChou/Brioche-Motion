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

#include "brx_animation_ik_reaching.h"
#include "brx_animation_ik_one_joint.h"
#include "brx_animation_ik_two_joints.h"
#include "brx_animation_ik_ccd.h"

extern void ik_reaching_solve(DirectX::XMFLOAT3 const &in_target_position_model_space, DirectX::XMFLOAT4X4 const &in_end_effector_transform_local_space, uint32_t const in_joint_count, DirectX::XMFLOAT4X4 *const inout_joints_local_space, DirectX::XMFLOAT4X4 *const inout_joints_model_space)
{
    constexpr float const INTERNAL_SCALE_EPSILON = 9E-5F;

    // NOTE: the model space of all children of the end effector should also be marked as invalid

    if (1U == in_joint_count)
    {
        ik_one_joint_solve(1.0F, in_target_position_model_space, in_end_effector_transform_local_space, inout_joints_local_space, inout_joints_model_space);
    }
    else if (2U == in_joint_count)
    {
        DirectX::XMFLOAT3 hinge_joint_axis_local_space;
        {
            DirectX::XMVECTOR ball_and_socket_joint_hinge_joint_local_space_translation;
            {
                constexpr uint32_t const ball_and_socket_joint_index = 0U;
                constexpr uint32_t const hinge_joint_index = 1U;

                DirectX::XMVECTOR unused_determinant;
                DirectX::XMMATRIX ball_and_socket_joint_hinge_joint_local_space_transform = DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&inout_joints_model_space[ball_and_socket_joint_index]), DirectX::XMMatrixInverse(&unused_determinant, DirectX::XMLoadFloat4x4(&inout_joints_model_space[hinge_joint_index])));

                DirectX::XMVECTOR ball_and_socket_joint_hinge_joint_local_space_scale;
                DirectX::XMVECTOR ball_and_socket_joint_hinge_joint_local_space_rotation;
                bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&ball_and_socket_joint_hinge_joint_local_space_scale, &ball_and_socket_joint_hinge_joint_local_space_rotation, &ball_and_socket_joint_hinge_joint_local_space_translation, ball_and_socket_joint_hinge_joint_local_space_transform);
                assert(directx_xm_matrix_decompose);

                assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(ball_and_socket_joint_hinge_joint_local_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
            }

            DirectX::XMVECTOR end_effector_hinge_joint_local_space_translation;
            {
                DirectX::XMVECTOR end_effector_hinge_joint_local_space_scale;
                DirectX::XMVECTOR end_effector_hinge_joint_local_space_rotation;
                bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&end_effector_hinge_joint_local_space_scale, &end_effector_hinge_joint_local_space_rotation, &end_effector_hinge_joint_local_space_translation, DirectX::XMLoadFloat4x4(&in_end_effector_transform_local_space));
                assert(directx_xm_matrix_decompose);

                assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(end_effector_hinge_joint_local_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
            }

            DirectX::XMStoreFloat3(&hinge_joint_axis_local_space, DirectX::XMVector3Normalize(DirectX::XMVector3Cross(ball_and_socket_joint_hinge_joint_local_space_translation, end_effector_hinge_joint_local_space_translation)));
        }

        ik_two_joints_solve(1.0F, hinge_joint_axis_local_space, -1.0F, 1.0F, 1.0F, in_target_position_model_space, in_end_effector_transform_local_space, inout_joints_local_space, inout_joints_model_space);
    }
    else if (3U == in_joint_count)
    {
        // TODO: three joints IK
        ik_ccd_solve(8U, 0.5F, in_target_position_model_space, in_end_effector_transform_local_space, in_joint_count, inout_joints_local_space, inout_joints_model_space);
    }
    else
    {
        assert(in_joint_count >= 4U);
        ik_ccd_solve(8U, 0.5F, in_target_position_model_space, in_end_effector_transform_local_space, in_joint_count, inout_joints_local_space, inout_joints_model_space);
    }
}
