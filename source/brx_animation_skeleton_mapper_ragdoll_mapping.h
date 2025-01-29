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

#ifndef _BRX_ANIMATION_SKELETON_MAPPER_RAGDOLL_MAPPING_H_
#define _BRX_ANIMATION_SKELETON_MAPPER_RAGDOLL_MAPPING_H_ 1

#include "../include/brx_motion.h"
#include "../../McRT-Malloc/include/mcrt_vector.h"
#include "../../McRT-Malloc/include/mcrt_string.h"
#include "brx_animation_skeleton.h"

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

class brx_animation_skeleton_mapper_ragdoll_direct_mapping_data
{
    uint32_t m_joint_index_a;
    uint32_t m_joint_index_b;
    DirectX::XMFLOAT4X4 m_a_to_b_transform_model_space;

public:
    inline brx_animation_skeleton_mapper_ragdoll_direct_mapping_data(uint32_t joint_index_a, uint32_t joint_index_b, DirectX::XMFLOAT4X4 a_to_b_transform_model_space);
    inline uint32_t get_joint_index_a() const;
    inline uint32_t get_joint_index_b() const;
    inline DirectX::XMFLOAT4X4 const &get_a_to_b_transform_model_space() const;
};

class brx_animation_skeleton_mapper_ragdoll_chain_mapping_data
{
    mcrt_vector<uint32_t> m_joint_indices_b;

public:
    inline brx_animation_skeleton_mapper_ragdoll_chain_mapping_data(mcrt_vector<uint32_t> &&joint_indices_b);
    inline mcrt_vector<uint32_t> const &get_joint_indices_b() const;
};

class brx_animation_skeleton_mapper_ragdoll_unmapped_data
{
    uint32_t m_joint_index_b;

public:
    inline brx_animation_skeleton_mapper_ragdoll_unmapped_data(uint32_t joint_index_b);
    inline uint32_t get_joint_index_b() const;
};

class brx_animation_skeleton_mapper_ragdoll_locked_translation_data
{
    uint32_t m_joint_index_b;

public:
    inline brx_animation_skeleton_mapper_ragdoll_locked_translation_data(uint32_t joint_index_b);
    inline uint32_t get_joint_index_b() const;
};

class brx_animation_skeleton_mapper_ragdoll_mapping_data
{
    mcrt_vector<brx_animation_skeleton_mapper_ragdoll_direct_mapping_data> m_direct_mappings;
    mcrt_vector<brx_animation_skeleton_mapper_ragdoll_chain_mapping_data> m_chain_mappings;
    mcrt_vector<brx_animation_skeleton_mapper_ragdoll_unmapped_data> m_unmapped;
    mcrt_vector<brx_animation_skeleton_mapper_ragdoll_locked_translation_data> m_locked_translations;

public:
    inline void set_direct_mappings(mcrt_vector<brx_animation_skeleton_mapper_ragdoll_direct_mapping_data> &&direct_mappings);
    inline void set_chain_mappings(mcrt_vector<brx_animation_skeleton_mapper_ragdoll_chain_mapping_data> &&chain_mappings);
    inline void set_unmapped(mcrt_vector<brx_animation_skeleton_mapper_ragdoll_unmapped_data> &&unmapped);
    inline void set_locked_translations(mcrt_vector<brx_animation_skeleton_mapper_ragdoll_locked_translation_data> &&locked_translations);
    inline mcrt_vector<brx_animation_skeleton_mapper_ragdoll_direct_mapping_data> const &get_direct_mappings() const;
    inline mcrt_vector<brx_animation_skeleton_mapper_ragdoll_chain_mapping_data> const &get_chain_mappings() const;
    inline mcrt_vector<brx_animation_skeleton_mapper_ragdoll_unmapped_data> const &get_unmapped() const;
    inline mcrt_vector<brx_animation_skeleton_mapper_ragdoll_locked_translation_data> const &get_locked_translations() const;
};

extern void brx_animation_skeleton_mapper_create_ragdoll_mapping(brx_animation_skeleton const &in_skeleton_a, brx_animation_skeleton const &in_skeleton_b, uint32_t (*const in_user_callback_map_joint_b_to_a)(brx_animation_skeleton const &in_skeleton_a, brx_animation_skeleton const &in_skeleton_b, uint32_t const in_skeleton_b_joint_index, void *const in_user_data_map_joint_b_to_a), void *const in_user_data_map_joint_b_to_a, bool const in_lock_translations, brx_animation_skeleton_mapper_ragdoll_mapping_data &out_ragdoll_mapping_data);

extern void brx_animation_skeleton_mapper_map_pose_model_space(brx_animation_skeleton_mapper_ragdoll_mapping_data const &in_ragdoll_mapping_data, DirectX::XMFLOAT4X4 const *const in_pose_a_model_space, uint32_t const in_skeleton_b_joint_count, uint32_t const *const in_skeleton_b_joint_parent_indices, DirectX::XMFLOAT4X4 const *const in_pose_b_local_space, DirectX::XMFLOAT4X4 *const out_pose_b_model_space);

#endif
