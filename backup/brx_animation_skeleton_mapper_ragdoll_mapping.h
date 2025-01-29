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
    inline brx_animation_skeleton_mapper_ragdoll_direct_mapping_data(uint32_t joint_index_a, uint32_t joint_index_b, DirectX::XMFLOAT4X4 a_to_b_transform_model_space) : m_joint_index_a(joint_index_a), m_joint_index_b(joint_index_b), m_a_to_b_transform_model_space(a_to_b_transform_model_space)
    {
    }
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
    inline mcrt_vector<brx_animation_skeleton_mapper_ragdoll_direct_mapping_data> &access_direct_mappings()
    {
        return this->m_direct_mappings;
    }
    inline mcrt_vector<brx_animation_skeleton_mapper_ragdoll_chain_mapping_data> &access_chain_mappings()
    {
        return this->m_chain_mappings;
    }

    inline mcrt_vector<brx_animation_skeleton_mapper_ragdoll_unmapped_data> &access_unmapped()
    {
        return this->m_unmapped;
    }

    inline mcrt_vector<brx_animation_skeleton_mapper_ragdoll_locked_translation_data> &access_locked_translations()
    {
        return this->m_locked_translations;
    }

    inline mcrt_vector<brx_animation_skeleton_mapper_ragdoll_direct_mapping_data> const &get_direct_mappings() const
    {
        return this->m_direct_mappings;
    }

    inline mcrt_vector<brx_animation_skeleton_mapper_ragdoll_chain_mapping_data> const &get_chain_mappings() const
    {
        return this->m_chain_mappings;
    }
    inline mcrt_vector<brx_animation_skeleton_mapper_ragdoll_unmapped_data> const &get_unmapped() const;
    inline mcrt_vector<brx_animation_skeleton_mapper_ragdoll_locked_translation_data> const &get_locked_translations() const;
};

extern void brx_animation_skeleton_mapper_create_ragdoll_chain_mapping(uint32_t const in_skeleton_b_joint_count, uint32_t const *const in_skeleton_b_joint_parent_indices, mcrt_vector<brx_animation_skeleton_mapper_ragdoll_direct_mapping_data> const &in_direct_mappings, mcrt_vector<brx_animation_skeleton_mapper_ragdoll_chain_mapping_data> &out_chain_mappings);

extern void brx_animation_skeleton_mapper_create_ragdoll_unmapped(uint32_t const in_skeleton_b_joint_count, uint32_t const *const in_skeleton_b_joint_parent_indices, mcrt_vector<brx_animation_skeleton_mapper_ragdoll_direct_mapping_data> const &in_direct_mappings, mcrt_vector<brx_animation_skeleton_mapper_ragdoll_chain_mapping_data> const &in_chain_mappings, mcrt_vector<brx_animation_skeleton_mapper_ragdoll_unmapped_data> &out_unmapped);

extern void brx_animation_skeleton_mapper_create_ragdoll_locked_translation(uint32_t const in_skeleton_b_joint_count, uint32_t const *const in_skeleton_b_joint_parent_indices, mcrt_vector<brx_animation_skeleton_mapper_ragdoll_direct_mapping_data> const &in_direct_mappings, mcrt_vector<brx_animation_skeleton_mapper_ragdoll_locked_translation_data> &out_locked_translations);

extern void brx_animation_skeleton_mapper_map_pose_model_space(brx_animation_skeleton_mapper_ragdoll_mapping_data const &in_ragdoll_mapping_data, DirectX::XMFLOAT4X4 const *const in_pose_a_model_space, uint32_t const in_skeleton_b_joint_count, uint32_t const *const in_skeleton_b_joint_parent_indices, DirectX::XMFLOAT4X4 const *const in_pose_b_local_space, DirectX::XMFLOAT4X4 *const out_pose_b_model_space);

#endif
