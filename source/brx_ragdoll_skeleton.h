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

#ifndef _BRX_RAGDOLL_SKELETON_H_
#define _BRX_RAGDOLL_SKELETON_H_ 1

#include "../include/brx_motion.h"
#include "../../McRT-Malloc/include/mcrt_vector.h"
#include "../../McRT-Malloc/include/mcrt_string.h"

class brx_ragdoll_skeleton
{
    mcrt_vector<uint32_t> m_joint_to_rigid_body_map;

public:
    brx_ragdoll_skeleton(mcrt_vector<uint32_t> &&joint_to_rigid_body_map);
    uint32_t get_joint_count() const;
    mcrt_string const &get_joint_name(uint32_t joint_index) const;
    uint32_t const *get_joint_parent_indices() const;
    uint32_t get_joint_parent_index(uint32_t joint_index) const;
    brx_motion_rigid_transform get_bind_pose_joint_transform_local_space(uint32_t joint_index) const;
};

#endif
