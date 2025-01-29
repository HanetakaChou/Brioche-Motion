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

#ifndef _BRX_ANIMATION_IK_TWO_JOINTS_H_
#define _BRX_ANIMATION_IK_TWO_JOINTS_H_ 1

#include <cstdint>
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

// NOTE: the larger the angle, the smaller the cosine instead
// in_cosine_max_hinge_angle: -1 (180 degree)
// in_cosine_min_hinge_angle: 1 (0 degree)

extern void ik_two_joints_solve(float const in_ball_and_socket_joint_gain, DirectX::XMFLOAT3 const &in_hinge_joint_axis_local_space, float const in_cosine_max_hinge_joint_angle, float const in_cosine_min_hinge_joint_angle, float const in_hinge_joint_gain, DirectX::XMFLOAT3 const &in_target_position_model_space, DirectX::XMFLOAT4X4 const &in_end_effector_transform_local_space, DirectX::XMFLOAT4X4 *const inout_joints_local_space, DirectX::XMFLOAT4X4 *const inout_joints_model_space);

#endif
