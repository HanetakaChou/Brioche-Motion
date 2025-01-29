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

#include "brx_animation_skeleton_mapper_ragdoll_mapping.h"
#include "brx_animation_ik_reaching.h"
#include "../../McRT-Malloc/include/mcrt_unordered_set.h"
#include "../../McRT-Malloc/include/mcrt_unordered_map.h"

#define internal_stack_alloc(size) alloca((size))

inline uint32_t brx_animation_skeleton_mapper_ragdoll_direct_mapping_data::get_joint_index_a() const
{
    return this->m_joint_index_a;
}

inline uint32_t brx_animation_skeleton_mapper_ragdoll_direct_mapping_data::get_joint_index_b() const
{
    return this->m_joint_index_b;
}

inline DirectX::XMFLOAT4X4 const &brx_animation_skeleton_mapper_ragdoll_direct_mapping_data::get_a_to_b_transform_model_space() const
{
    return this->m_a_to_b_transform_model_space;
}

inline brx_animation_skeleton_mapper_ragdoll_chain_mapping_data::brx_animation_skeleton_mapper_ragdoll_chain_mapping_data(mcrt_vector<uint32_t> &&joint_indices_b) : m_joint_indices_b(joint_indices_b)
{
}

inline mcrt_vector<uint32_t> const &brx_animation_skeleton_mapper_ragdoll_chain_mapping_data::get_joint_indices_b() const
{
    return this->m_joint_indices_b;
}

inline brx_animation_skeleton_mapper_ragdoll_unmapped_data::brx_animation_skeleton_mapper_ragdoll_unmapped_data(uint32_t joint_index_b) : m_joint_index_b(joint_index_b)
{
}

inline uint32_t brx_animation_skeleton_mapper_ragdoll_unmapped_data::get_joint_index_b() const
{
    return this->m_joint_index_b;
}

inline brx_animation_skeleton_mapper_ragdoll_locked_translation_data::brx_animation_skeleton_mapper_ragdoll_locked_translation_data(uint32_t joint_index_b) : m_joint_index_b(joint_index_b)
{
}

inline uint32_t brx_animation_skeleton_mapper_ragdoll_locked_translation_data::get_joint_index_b() const
{
    return this->m_joint_index_b;
}

inline mcrt_vector<brx_animation_skeleton_mapper_ragdoll_unmapped_data> const &brx_animation_skeleton_mapper_ragdoll_mapping_data::get_unmapped() const
{
    return this->m_unmapped;
}

inline mcrt_vector<brx_animation_skeleton_mapper_ragdoll_locked_translation_data> const &brx_animation_skeleton_mapper_ragdoll_mapping_data::get_locked_translations() const
{
    return this->m_locked_translations;
}

extern void brx_animation_skeleton_mapper_create_ragdoll_chain_mapping(uint32_t const in_skeleton_b_joint_count, uint32_t const *const in_skeleton_b_joint_parent_indices, mcrt_vector<brx_animation_skeleton_mapper_ragdoll_direct_mapping_data> const &in_direct_mappings, mcrt_vector<brx_animation_skeleton_mapper_ragdoll_chain_mapping_data> &out_chain_mappings)
{
    mcrt_vector<bool> mapped_b(static_cast<size_t>(in_skeleton_b_joint_count), false);

    assert(!in_direct_mappings.empty());
    mcrt_unordered_map<uint32_t, uint32_t> map_joint_b_to_a;
    {
        for (brx_animation_skeleton_mapper_ragdoll_direct_mapping_data const &ragdoll_direct_mapping_data : in_direct_mappings)
        {
            uint32_t const joint_index_a = ragdoll_direct_mapping_data.get_joint_index_a();
            uint32_t const joint_index_b = ragdoll_direct_mapping_data.get_joint_index_b();

            assert(!mapped_b[joint_index_b]);
            mapped_b[joint_index_b] = true;

            mcrt_unordered_map<uint32_t, uint32_t>::const_iterator found = map_joint_b_to_a.find(joint_index_b);
            assert(map_joint_b_to_a.end() == found);
            map_joint_b_to_a.emplace_hint(found, joint_index_b, joint_index_a);
        }
    }

    assert(out_chain_mappings.empty());
    out_chain_mappings = {};
    {
        mcrt_vector<mcrt_vector<uint32_t>> skeleton_b_children_joint_indices(static_cast<size_t>(in_skeleton_b_joint_count));
        mcrt_vector<mcrt_unordered_set<uint32_t>> skeleton_b_children_joint_indices_set(static_cast<size_t>(in_skeleton_b_joint_count));
        mcrt_vector<uint32_t> skeleton_b_depth_first_search_stack;
        mcrt_vector<std::pair<uint32_t, uint32_t>> skeleton_b_depth_first_search_ancestor_map_reverse_skeleton_a;
        int skeleton_b_depth_first_search_chain_start_ancestor_map_index = -1;
        for (uint32_t skeleton_b_joint_index_plus_1 = in_skeleton_b_joint_count; skeleton_b_joint_index_plus_1 > 0U; --skeleton_b_joint_index_plus_1)
        {
            uint32_t skeleton_b_joint_index = skeleton_b_joint_index_plus_1 - 1U;
            uint32_t skeleton_b_parent_joint_index = in_skeleton_b_joint_parent_indices[skeleton_b_joint_index];
            if (BRX_MOTION_UINT32_INDEX_INVALID != skeleton_b_parent_joint_index)
            {
                skeleton_b_children_joint_indices[skeleton_b_parent_joint_index].push_back(skeleton_b_joint_index);
                assert(skeleton_b_children_joint_indices_set[skeleton_b_parent_joint_index].end() == skeleton_b_children_joint_indices_set[skeleton_b_parent_joint_index].find(skeleton_b_joint_index));
                skeleton_b_children_joint_indices_set[skeleton_b_parent_joint_index].emplace_hint(skeleton_b_children_joint_indices_set[skeleton_b_parent_joint_index].end(), skeleton_b_joint_index);
            }
            else
            {
                skeleton_b_depth_first_search_stack.push_back(skeleton_b_joint_index);
            }
        }
        assert(1U == skeleton_b_depth_first_search_stack.size());

        mcrt_vector<bool> skeleton_b_joint_visited_flags(static_cast<size_t>(in_skeleton_b_joint_count), false);
        mcrt_vector<bool> skeleton_b_joint_pushed_flags(static_cast<size_t>(in_skeleton_b_joint_count), false);
        while (!skeleton_b_depth_first_search_stack.empty())
        {
            uint32_t skeleton_b_current_joint_index = skeleton_b_depth_first_search_stack.back();
            skeleton_b_depth_first_search_stack.pop_back();

            assert(!skeleton_b_joint_visited_flags[skeleton_b_current_joint_index]);
            skeleton_b_joint_visited_flags[skeleton_b_current_joint_index] = true;

            // Back Tracking
            bool back_tracking = false;
            while ((!skeleton_b_depth_first_search_ancestor_map_reverse_skeleton_a.empty()) && (skeleton_b_children_joint_indices_set[skeleton_b_depth_first_search_ancestor_map_reverse_skeleton_a.back().first].end() == skeleton_b_children_joint_indices_set[skeleton_b_depth_first_search_ancestor_map_reverse_skeleton_a.back().first].find(skeleton_b_current_joint_index)))
            {
                skeleton_b_depth_first_search_ancestor_map_reverse_skeleton_a.pop_back();
                back_tracking = true;
            }

            if (back_tracking)
            {
                skeleton_b_depth_first_search_chain_start_ancestor_map_index = -1;
                for (int chain_ancestor_map_index = static_cast<int>(skeleton_b_depth_first_search_ancestor_map_reverse_skeleton_a.size()); chain_ancestor_map_index > 0; --chain_ancestor_map_index)
                {
                    if (-1 != skeleton_b_depth_first_search_ancestor_map_reverse_skeleton_a[chain_ancestor_map_index - 1].second)
                    {
                        skeleton_b_depth_first_search_chain_start_ancestor_map_index = (chain_ancestor_map_index - 1);
                        break;
                    }
                }
            }

            assert((-1 == skeleton_b_depth_first_search_chain_start_ancestor_map_index) || skeleton_b_depth_first_search_chain_start_ancestor_map_index < static_cast<int>(skeleton_b_depth_first_search_ancestor_map_reverse_skeleton_a.size()));

            uint32_t skeleton_a_current_joint_index;
            {
                mcrt_unordered_map<std::uint32_t, uint32_t>::const_iterator found_skeleton_a_joint = map_joint_b_to_a.find(skeleton_b_current_joint_index);
                if (map_joint_b_to_a.end() != found_skeleton_a_joint)
                {
                    skeleton_a_current_joint_index = found_skeleton_a_joint->second;
                }
                else
                {
                    skeleton_a_current_joint_index = BRX_MOTION_UINT32_INDEX_INVALID;
                }
            }
            skeleton_b_depth_first_search_ancestor_map_reverse_skeleton_a.push_back({skeleton_b_current_joint_index, skeleton_a_current_joint_index});

            if (BRX_MOTION_UINT32_INDEX_INVALID != skeleton_a_current_joint_index)
            {
                if ((-1 != skeleton_b_depth_first_search_chain_start_ancestor_map_index) && (static_cast<int>(skeleton_b_depth_first_search_ancestor_map_reverse_skeleton_a.size()) > (skeleton_b_depth_first_search_chain_start_ancestor_map_index + 2)))
                {
                    // It should have joints between the mapped joints
                    mcrt_vector<uint32_t> chain_b;
                    chain_b.reserve(static_cast<size_t>(static_cast<int>(skeleton_b_depth_first_search_ancestor_map_reverse_skeleton_a.size()) - skeleton_b_depth_first_search_chain_start_ancestor_map_index));
                    for (int chain_ancestor_map_index = skeleton_b_depth_first_search_chain_start_ancestor_map_index; chain_ancestor_map_index < int(skeleton_b_depth_first_search_ancestor_map_reverse_skeleton_a.size()); ++chain_ancestor_map_index)
                    {
                        chain_b.push_back(skeleton_b_depth_first_search_ancestor_map_reverse_skeleton_a[chain_ancestor_map_index].first);

                        // Mark elements mapped
                        assert((skeleton_b_depth_first_search_chain_start_ancestor_map_index == chain_ancestor_map_index) || ((static_cast<int>(skeleton_b_depth_first_search_ancestor_map_reverse_skeleton_a.size()) - 1) == chain_ancestor_map_index) || (!mapped_b[skeleton_b_depth_first_search_ancestor_map_reverse_skeleton_a[chain_ancestor_map_index].first]));
                        mapped_b[skeleton_b_depth_first_search_ancestor_map_reverse_skeleton_a[chain_ancestor_map_index].first] = true;
                    }

                    out_chain_mappings.emplace_back(std::move(chain_b));
                }

                skeleton_b_depth_first_search_chain_start_ancestor_map_index = static_cast<int>(skeleton_b_depth_first_search_ancestor_map_reverse_skeleton_a.size()) - 1;
            }

            for (uint32_t child_joint_index_index_plus_1 = static_cast<uint32_t>(skeleton_b_children_joint_indices[skeleton_b_current_joint_index].size()); child_joint_index_index_plus_1 > 0U; --child_joint_index_index_plus_1)
            {
                uint32_t const skeleton_b_child_joint_index = skeleton_b_children_joint_indices[skeleton_b_current_joint_index][child_joint_index_index_plus_1 - 1U];
                if ((!skeleton_b_joint_visited_flags[skeleton_b_child_joint_index]) && (!skeleton_b_joint_pushed_flags[skeleton_b_child_joint_index]))
                {
                    skeleton_b_joint_pushed_flags[skeleton_b_child_joint_index] = true;
                    skeleton_b_depth_first_search_stack.push_back(skeleton_b_child_joint_index);
                }
                else
                {
                    assert(false);
                }
            }
        }
    }
}

extern void brx_animation_skeleton_mapper_create_ragdoll_unmapped(uint32_t const in_skeleton_b_joint_count, uint32_t const *const in_skeleton_b_joint_parent_indices, mcrt_vector<brx_animation_skeleton_mapper_ragdoll_direct_mapping_data> const &in_direct_mappings, mcrt_vector<brx_animation_skeleton_mapper_ragdoll_chain_mapping_data> const &in_chain_mappings, mcrt_vector<brx_animation_skeleton_mapper_ragdoll_unmapped_data> &out_unmapped)
{
    mcrt_vector<bool> mapped_b(static_cast<size_t>(in_skeleton_b_joint_count), false);

    assert(!in_direct_mappings.empty());
    {
        for (brx_animation_skeleton_mapper_ragdoll_direct_mapping_data const &ragdoll_direct_mapping_data : in_direct_mappings)
        {
            uint32_t const joint_index_a = ragdoll_direct_mapping_data.get_joint_index_a();
            uint32_t const joint_index_b = ragdoll_direct_mapping_data.get_joint_index_b();

            assert(!mapped_b[joint_index_b]);
            mapped_b[joint_index_b] = true;
        }
    }

    {
        for (brx_animation_skeleton_mapper_ragdoll_chain_mapping_data const &ragdoll_chain_mapping_data : in_chain_mappings)
        {
            uint32_t const chain_joint_count_b_plus_1 = ragdoll_chain_mapping_data.get_joint_indices_b().size();
            assert(chain_joint_count_b_plus_1 >= 1U);

            for (uint32_t chain_index_b_plus_1 = 2U; chain_index_b_plus_1 < chain_joint_count_b_plus_1; ++chain_index_b_plus_1)
            {
                uint32_t const chain_index_b = chain_index_b_plus_1 - 1U;

                uint32_t const joint_index_b = ragdoll_chain_mapping_data.get_joint_indices_b()[chain_index_b];

                assert(!mapped_b[joint_index_b]);
                mapped_b[joint_index_b] = true;
            }
        }
    }

    assert(out_unmapped.empty());
    out_unmapped = {};
    for (uint32_t skeleton_b_joint_index = 0; skeleton_b_joint_index < in_skeleton_b_joint_count; ++skeleton_b_joint_index)
    {
        if (!mapped_b[skeleton_b_joint_index])
        {
            out_unmapped.emplace_back(skeleton_b_joint_index);
        }
    }
}

extern void brx_animation_skeleton_mapper_create_ragdoll_locked_translation(uint32_t const in_skeleton_b_joint_count, uint32_t const *const in_skeleton_b_joint_parent_indices, mcrt_vector<brx_animation_skeleton_mapper_ragdoll_direct_mapping_data> const &in_direct_mappings, mcrt_vector<brx_animation_skeleton_mapper_ragdoll_locked_translation_data> &out_locked_translations)
{
    mcrt_vector<bool> locked(static_cast<size_t>(in_skeleton_b_joint_count), false);

    uint32_t root_joint_index_b = BRX_MOTION_UINT32_INDEX_INVALID;

    assert(!in_direct_mappings.empty());
    if (!in_direct_mappings.empty())
    {
        root_joint_index_b = in_direct_mappings[0].get_joint_index_b();

#ifndef NDEBUG
        uint32_t root_joint_index_a = in_direct_mappings[0].get_joint_index_a();

        for (uint32_t ragdoll_direct_mapping_data_index = 1U; ragdoll_direct_mapping_data_index < in_direct_mappings.size(); ++ragdoll_direct_mapping_data_index)
        {
            uint32_t const joint_index_a = in_direct_mappings[ragdoll_direct_mapping_data_index].get_joint_index_a();
            uint32_t const joint_index_b = in_direct_mappings[ragdoll_direct_mapping_data_index].get_joint_index_b();
            assert(joint_index_a > root_joint_index_a);
            assert(joint_index_b > root_joint_index_b);
        }
#endif

        locked[root_joint_index_b] = true;
    }

    assert(out_locked_translations.empty());
    out_locked_translations = {};
    // "root_joint_index_b" NOT included
    for (uint32_t skeleton_b_joint_index = (root_joint_index_b + 1U); skeleton_b_joint_index < in_skeleton_b_joint_count; ++skeleton_b_joint_index)
    {
        uint32_t const skeleton_b_parent_joint_index = in_skeleton_b_joint_parent_indices[skeleton_b_joint_index];
        if (BRX_MOTION_UINT32_INDEX_INVALID != skeleton_b_parent_joint_index)
        {
            assert(skeleton_b_parent_joint_index < skeleton_b_joint_index);
            if (locked[skeleton_b_parent_joint_index])
            {
                locked[skeleton_b_joint_index] = true;
                out_locked_translations.emplace_back(skeleton_b_joint_index);
            }
        }
    }
}

extern void brx_animation_skeleton_mapper_map_pose_model_space(brx_animation_skeleton_mapper_ragdoll_mapping_data const &in_ragdoll_mapping_data, DirectX::XMFLOAT4X4 const *const in_pose_a_model_space, uint32_t const in_skeleton_b_joint_count, uint32_t const *const in_skeleton_b_joint_parent_indices, DirectX::XMFLOAT4X4 const *const in_pose_b_local_space, DirectX::XMFLOAT4X4 *const out_pose_b_model_space)
{
#ifndef NDEBUG
    bool *mapped_b = NULL;
    {
        size_t const size = sizeof(bool) * in_skeleton_b_joint_count;
        mapped_b = static_cast<bool *>(internal_stack_alloc(size));
        std::memset(mapped_b, 0, size);
    }
#endif

    for (brx_animation_skeleton_mapper_ragdoll_direct_mapping_data const &ragdoll_direct_mapping_data : in_ragdoll_mapping_data.get_direct_mappings())
    {
        uint32_t const joint_index_a = ragdoll_direct_mapping_data.get_joint_index_a();
        uint32_t const joint_index_b = ragdoll_direct_mapping_data.get_joint_index_b();
        DirectX::XMFLOAT4X4 const &a_to_b_transform_model_space = ragdoll_direct_mapping_data.get_a_to_b_transform_model_space();
#ifndef NDEBUG
        assert(!mapped_b[joint_index_b]);
        mapped_b[joint_index_b] = true;
#endif
        DirectX::XMStoreFloat4x4(&out_pose_b_model_space[joint_index_b], DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&a_to_b_transform_model_space), DirectX::XMLoadFloat4x4(&in_pose_a_model_space[joint_index_a])));
    }

    for (brx_animation_skeleton_mapper_ragdoll_chain_mapping_data const &ragdoll_chain_mapping_data : in_ragdoll_mapping_data.get_chain_mappings())
    {
        DirectX::XMFLOAT3 in_target_position_model_space;
        DirectX::XMFLOAT4X4 in_end_effector_transform_local_space;
        {
            uint32_t const end_joint_index_b = ragdoll_chain_mapping_data.get_joint_indices_b().back();
            assert(mapped_b[end_joint_index_b]);

            DirectX::XMVECTOR end_joint_b_model_space_scale;
            DirectX::XMVECTOR end_joint_b_model_space_rotation;
            DirectX::XMVECTOR end_joint_b_model_space_translation;
            bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&end_joint_b_model_space_scale, &end_joint_b_model_space_rotation, &end_joint_b_model_space_translation, DirectX::XMLoadFloat4x4(&out_pose_b_model_space[end_joint_index_b]));
            assert(directx_xm_matrix_decompose);

            DirectX::XMStoreFloat3(&in_target_position_model_space, end_joint_b_model_space_translation);

            constexpr float const INTERNAL_SCALE_EPSILON = 7E-5F;
            assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(end_joint_b_model_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));

            in_end_effector_transform_local_space = in_pose_b_local_space[end_joint_index_b];
        }

        uint32_t const chain_joint_count_b_plus_1 = ragdoll_chain_mapping_data.get_joint_indices_b().size();
        assert(chain_joint_count_b_plus_1 >= 1U);
        uint32_t const chain_joint_count_b = chain_joint_count_b_plus_1 - 1U;

        DirectX::XMFLOAT4X4 *inout_chain_b_local_space = NULL;
        DirectX::XMFLOAT4X4 *inout_chain_b_model_space = NULL;
        {
            {
                size_t const alignment = alignof(DirectX::XMFLOAT4X4);
                size_t const size = sizeof(DirectX::XMFLOAT4X4) * chain_joint_count_b;

                size_t space_local_space = size + (alignment - 1U);
                void *ptr_local_space = internal_stack_alloc(space_local_space);
                inout_chain_b_local_space = static_cast<DirectX::XMFLOAT4X4 *>(std::align(alignment, size, ptr_local_space, space_local_space));

                size_t space_model_space = size + (alignment - 1U);
                void *ptr_model_space = internal_stack_alloc(space_model_space);
                inout_chain_b_model_space = static_cast<DirectX::XMFLOAT4X4 *>(std::align(alignment, size, ptr_model_space, space_model_space));
            }

            uint32_t const front_joint_index_b = ragdoll_chain_mapping_data.get_joint_indices_b().front();

            DirectX::XMStoreFloat4x4(&inout_chain_b_local_space[0], DirectX::XMLoadFloat4x4(&in_pose_b_local_space[front_joint_index_b]));

            assert(mapped_b[front_joint_index_b]);
            DirectX::XMMATRIX chain_end = DirectX::XMLoadFloat4x4(&out_pose_b_model_space[front_joint_index_b]);

            DirectX::XMStoreFloat4x4(&inout_chain_b_model_space[0], chain_end);

            for (uint32_t chain_index_b_plus_1 = 2U; chain_index_b_plus_1 < chain_joint_count_b_plus_1; ++chain_index_b_plus_1)
            {
                uint32_t chain_index_b = chain_index_b_plus_1 - 1U;

                uint32_t const intermediate_joint_index_b = ragdoll_chain_mapping_data.get_joint_indices_b()[chain_index_b];

                DirectX::XMFLOAT4X4 const &intermediate_joint_joint_b_local_space = in_pose_b_local_space[intermediate_joint_index_b];

                inout_chain_b_local_space[chain_index_b] = intermediate_joint_joint_b_local_space;

                chain_end = DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&intermediate_joint_joint_b_local_space), chain_end);

                DirectX::XMStoreFloat4x4(&inout_chain_b_model_space[chain_index_b], chain_end);
            }
        }

        ik_reaching_solve(in_target_position_model_space, in_end_effector_transform_local_space, chain_joint_count_b, inout_chain_b_local_space, inout_chain_b_model_space);

        for (uint32_t chain_index_b_plus_1 = 1U; chain_index_b_plus_1 < chain_joint_count_b_plus_1; ++chain_index_b_plus_1)
        {
            uint32_t const chain_index_b = chain_index_b_plus_1 - 1U;

            uint32_t const intermediate_joint_index_b = ragdoll_chain_mapping_data.get_joint_indices_b()[chain_index_b];

#ifndef NDEBUG
            // When there are multiple children of this base joint, the current modification to (the rotation of) this base joint can impact the subsequent IK calculation
            assert((0U == chain_index_b) || (!mapped_b[intermediate_joint_index_b]));
            mapped_b[intermediate_joint_index_b] = true;
#endif
            out_pose_b_model_space[intermediate_joint_index_b] = inout_chain_b_model_space[chain_index_b];
        }
    }

    for (brx_animation_skeleton_mapper_ragdoll_unmapped_data const &ragdoll_unmapped_data : in_ragdoll_mapping_data.get_unmapped())
    {
        uint32_t const joint_index_b = ragdoll_unmapped_data.get_joint_index_b();
        uint32_t const parent_joint_index_b = in_skeleton_b_joint_parent_indices[joint_index_b];
        if (BRX_MOTION_UINT32_INDEX_INVALID != parent_joint_index_b)
        {
#ifndef NDEBUG
            assert(mapped_b[parent_joint_index_b]);
            assert(!mapped_b[joint_index_b]);
            mapped_b[joint_index_b] = true;
#endif
            DirectX::XMStoreFloat4x4(&out_pose_b_model_space[joint_index_b], DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&in_pose_b_local_space[joint_index_b]), DirectX::XMLoadFloat4x4(&out_pose_b_model_space[parent_joint_index_b])));
        }
        else
        {
#ifndef NDEBUG
            assert(!mapped_b[joint_index_b]);
            mapped_b[joint_index_b] = true;
#endif
            out_pose_b_model_space[joint_index_b] = in_pose_b_local_space[joint_index_b];
        }
    }

#ifndef NDEBUG
    for (uint32_t joint_index_b = 0; joint_index_b < in_skeleton_b_joint_count; ++joint_index_b)
    {
        assert(mapped_b[joint_index_b]);
    }
#endif

    for (brx_animation_skeleton_mapper_ragdoll_locked_translation_data const &ragdoll_locked_translation_data : in_ragdoll_mapping_data.get_locked_translations())
    {
        uint32_t const joint_index_b = ragdoll_locked_translation_data.get_joint_index_b();
        uint32_t const parent_joint_index_b = in_skeleton_b_joint_parent_indices[joint_index_b];

        assert(parent_joint_index_b < joint_index_b);

        // use the current animation pose instead of the bind pose as the source local space
        DirectX::XMVECTOR joint_index_b_locked_translation;
        {
            DirectX::XMVECTOR joint_index_b_locked_scale;
            DirectX::XMVECTOR joint_index_b_locked_rotation;
            bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&joint_index_b_locked_scale, &joint_index_b_locked_rotation, &joint_index_b_locked_translation, DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&in_pose_b_local_space[joint_index_b]), DirectX::XMLoadFloat4x4(&out_pose_b_model_space[parent_joint_index_b])));
            assert(directx_xm_matrix_decompose);

            constexpr float const INTERNAL_SCALE_EPSILON = 7E-5F;
            assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(joint_index_b_locked_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
        }

        DirectX::XMVECTOR joint_index_b_rotation;
        {
            DirectX::XMVECTOR joint_index_b_scale;
            DirectX::XMVECTOR joint_index_b_translation;
            bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&joint_index_b_scale, &joint_index_b_rotation, &joint_index_b_translation, DirectX::XMLoadFloat4x4(&out_pose_b_model_space[joint_index_b]));
            assert(directx_xm_matrix_decompose);

            constexpr float const INTERNAL_SCALE_EPSILON = 7E-5F;
            assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(joint_index_b_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
        }

        DirectX::XMStoreFloat4x4(&out_pose_b_model_space[joint_index_b], DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(joint_index_b_rotation), DirectX::XMMatrixTranslationFromVector(joint_index_b_locked_translation)));
    }
}
