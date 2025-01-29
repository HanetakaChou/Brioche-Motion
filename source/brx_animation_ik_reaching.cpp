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
#include "brx_animation_ik_ccd.h"

extern void ik_reaching_solve(DirectX::XMFLOAT3 const &in_target_position_model_space, DirectX::XMFLOAT4X4 const &in_end_effector_transform_local_space, uint32_t const in_chain_joint_count, DirectX::XMFLOAT4X4 *const inout_chain_local_space, DirectX::XMFLOAT4X4 *const inout_chain_model_space)
{
    // NOTE: the model space of all children of the end effector should also be marked as invalid
    if (in_chain_joint_count >= 4U)
    {
        ik_ccd_solve(8U, 0.5F, in_target_position_model_space, in_end_effector_transform_local_space, in_chain_joint_count, inout_chain_local_space, inout_chain_model_space);
    }
    else if (3U == in_chain_joint_count)
    {
        // TODO: three joints IK
        ik_ccd_solve(8U, 0.5F, in_target_position_model_space, in_end_effector_transform_local_space, in_chain_joint_count, inout_chain_local_space, inout_chain_model_space);
    }

    else if (2U == in_chain_joint_count)
    {
        // TODO: two joints IK
        ik_ccd_solve(8U, 0.5F, in_target_position_model_space, in_end_effector_transform_local_space, in_chain_joint_count, inout_chain_local_space, inout_chain_model_space);
    }
    else
    {
        // Trivial
        assert(1U == in_chain_joint_count);
        ik_ccd_solve(8U, 0.5F, in_target_position_model_space, in_end_effector_transform_local_space, in_chain_joint_count, inout_chain_local_space, inout_chain_model_space);
    }
}