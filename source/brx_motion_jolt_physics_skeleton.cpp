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

#include "brx_motion_jolt_physics_skeleton.h"
#include "../../McRT-Malloc/include/mcrt_malloc.h"
#include <cassert>
#include <new>

extern "C" brx_motion_skeleton *brx_motion_create_skeleton(int32_t joint_count, char const *const *joint_names, int32_t const *parent_indices, int32_t const vrm_joint_indices[BRX_MOTION_VRM_JOINT_NAME_COUNT])
{
    void *new_unwrapped_skeleton_base = mcrt_malloc(sizeof(brx_motion_jolt_physics_skeleton), alignof(brx_motion_jolt_physics_skeleton));
    assert(NULL != new_unwrapped_skeleton_base);

    brx_motion_jolt_physics_skeleton *new_unwrapped_skeleton = new (new_unwrapped_skeleton_base) brx_motion_jolt_physics_skeleton{};
    new_unwrapped_skeleton->init(joint_count, joint_names, parent_indices, vrm_joint_indices);
    return new_unwrapped_skeleton;
}

brx_motion_jolt_physics_skeleton::brx_motion_jolt_physics_skeleton()
{
}

brx_motion_jolt_physics_skeleton::~brx_motion_jolt_physics_skeleton()
{
}

void brx_motion_jolt_physics_skeleton::init(int32_t joint_count, char const *const *joint_names, int32_t const *parent_indices, int32_t const vrm_joint_indices[BRX_MOTION_VRM_JOINT_NAME_COUNT])
{
    for (int32_t joint_index = 0; joint_index < joint_count; ++joint_index)
    {
        this->m_skeleton.AddJoint(joint_names[joint_index], parent_indices[joint_index]);
    }

    assert(this->m_skeleton.AreJointsCorrectlyOrdered());
}

void brx_motion_jolt_physics_skeleton::uninit()
{
}

void brx_motion_jolt_physics_skeleton::brx_motion_skelton_get_pose(float (*out_joint_model_space_rotations)[4], float (*out_joint_model_space_translations)[3]) const
{
    JPH::SkeletonPose::Mat44Vector const &joint_model_space_transforms = this->m_pose.GetJointMatrices();
    for (int joint_index = 0; joint_index < this->m_pose.GetJointCount(); ++joint_index)
    {
        joint_model_space_transforms[joint_index].GetQuaternion().GetXYZW().StoreFloat4(reinterpret_cast<JPH::Float4 *>(&out_joint_model_space_rotations[joint_index]));
        joint_model_space_transforms[joint_index].GetTranslation().StoreFloat3(reinterpret_cast<JPH::Float3 *>(&out_joint_model_space_translations[joint_index]));
    }
}

void brx_motion_jolt_physics_skeleton::destroy()
{
    this->uninit();

    this->~brx_motion_jolt_physics_skeleton();
    mcrt_free(this);
}

extern "C" brx_motion_skeleton_animation *brx_motion_create_skeleton_animation(int32_t joint_count, char const *const *joint_names, uint32_t frame_count, float const (*joint_model_space_rotations)[4], float const (*joint_model_space_translations)[3])
{
    void *new_unwrapped_skeleton_animation_base = mcrt_malloc(sizeof(brx_motion_jolt_physics_skeleton_animation), alignof(brx_motion_jolt_physics_skeleton_animation));
    assert(NULL != new_unwrapped_skeleton_animation_base);

    brx_motion_jolt_physics_skeleton_animation *new_unwrapped_skeleton_animation = new (new_unwrapped_skeleton_animation_base) brx_motion_jolt_physics_skeleton_animation{};
    new_unwrapped_skeleton_animation->init(joint_count, joint_names, frame_count, joint_model_space_rotations, joint_model_space_translations);
    return new_unwrapped_skeleton_animation;
}

brx_motion_jolt_physics_skeleton_animation::brx_motion_jolt_physics_skeleton_animation()
{
}

brx_motion_jolt_physics_skeleton_animation::~brx_motion_jolt_physics_skeleton_animation()
{
}

void brx_motion_jolt_physics_skeleton_animation::init(int32_t joint_count, char const *const *joint_names, uint32_t frame_count, float const (*joint_model_space_rotations)[4], float const (*joint_model_space_translations)[3])
{
    this->m_joint_names.resize(joint_count);
    this->m_poses_model_space.resize(joint_count * frame_count);

    for (int32_t joint_index = 0; joint_index < joint_count; ++joint_index)
    {
        this->m_joint_names[joint_index] = joint_names[joint_index];
    }

    for (uint32_t frame_index = 0U; frame_index < frame_count; ++frame_index)
    {
        for (int32_t joint_index = 0; joint_index < joint_count; ++joint_index)
        {
            this->m_poses_model_space[joint_count * frame_index + joint_index].mRotation.Set(joint_model_space_rotations[joint_count * frame_index + joint_index][0], joint_model_space_rotations[joint_count * frame_index + joint_index][1], joint_model_space_rotations[joint_count * frame_index + joint_index][2], joint_model_space_rotations[joint_count * frame_index + joint_index][3]);
            this->m_poses_model_space[joint_count * frame_index + joint_index].mTranslation.Set(joint_model_space_translations[joint_count * frame_index + joint_index][0], joint_model_space_translations[joint_count * frame_index + joint_index][1], joint_model_space_translations[joint_count * frame_index + joint_index][2]);
        }
    }
}

void brx_motion_jolt_physics_skeleton_animation::uninit()
{
}

#include <Jolt/Core/Memory.h>

JPH_NAMESPACE_BEGIN

static void *AllocateHook(size_t inSize)
{
    return mcrt_malloc(inSize, 16U);
}

static void *ReallocateHook(void *inBlock, size_t inOldSize, size_t inNewSize)
{
    assert(0);
    return NULL;
}

static void FreeHook(void *inBlock)
{
    return mcrt_free(inBlock);
}

static void *AlignedAllocateHook(size_t inSize, size_t inAlignment)
{
    return mcrt_malloc(inSize, inAlignment);
}

static void AlignedFreeHook(void *inBlock)
{
    return mcrt_free(inBlock);
}

AllocateFunction Allocate = AllocateHook;
ReallocateFunction Reallocate = ReallocateHook;
FreeFunction Free = FreeHook;
AlignedAllocateFunction AlignedAllocate = AlignedAllocateHook;
AlignedFreeFunction AlignedFree = AlignedFreeHook;

JPH_NAMESPACE_END
