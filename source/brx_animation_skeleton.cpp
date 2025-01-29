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

#include "brx_animation_skeleton.h"
#include <cassert>

brx_animation_skeleton::brx_animation_skeleton(mcrt_vector<mcrt_string> &&joint_names, mcrt_vector<uint32_t> &&joint_parent_indices, mcrt_vector<brx_motion_rigid_transform> &&bind_pose_local_space) : m_joint_names(std::move(joint_names)), m_joint_parent_indices(std::move(joint_parent_indices)), m_bind_pose_local_space(std::move(bind_pose_local_space))
{
}

uint32_t brx_animation_skeleton::get_joint_count() const
{
    assert(this->m_joint_names.size() == this->m_joint_parent_indices.size());
    assert(this->m_joint_names.size() == this->m_bind_pose_local_space.size());
    return static_cast<uint32_t>(this->m_joint_names.size());
}

mcrt_string const &brx_animation_skeleton::get_joint_name(uint32_t joint_index) const
{
    return this->m_joint_names[joint_index];
}

uint32_t const *brx_animation_skeleton::get_joint_parent_indices() const
{
    return this->m_joint_parent_indices.data();
}

uint32_t brx_animation_skeleton::get_joint_parent_index(uint32_t joint_index) const
{
    return this->m_joint_parent_indices[joint_index];
}

brx_motion_rigid_transform brx_animation_skeleton::get_bind_pose_joint_transform_local_space(uint32_t joint_index) const
{
    return this->m_bind_pose_local_space[joint_index];
}
