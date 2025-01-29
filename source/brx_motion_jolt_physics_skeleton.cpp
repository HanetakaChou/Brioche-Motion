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
// #define JPH_DEBUG_RENDERER
// #define JPH_PROFILE_ENABLED
#undef JPH_OBJECT_STREAM
#define JPH_USE_AVX2
#define JPH_USE_AVX
#define JPH_USE_SSE4_1
#define JPH_USE_SSE4_2
#define JPH_USE_LZCNT
#define JPH_USE_TZCNT
#define JPH_USE_F16C
#define JPH_USE_FMADD
#include <Jolt/Jolt.h>
#include <Jolt/Skeleton/Skeleton.h>
#include <Jolt/Skeleton/SkeletonPose.h>
#include <Jolt/Skeleton/SkeletonMapper.h>
#include <cassert>
#include <new>

extern "C" brx_motion_skeleton *brx_motion_create_skeleton(uint32_t skeleton_joint_count, char const *const *skeleton_joint_names, uint32_t const *skeleton_joint_parent_indices, brx_motion_skeleton_joint_transform const *skeleton_bind_pose_joint_transforms, uint32_t const vrm_skeleton_joint_indices[BRX_MOTION_VRM_SKELETON_JOINT_NAME_COUNT])
{
    void *new_unwrapped_skeleton_base = mcrt_malloc(sizeof(brx_motion_jolt_physics_skeleton), alignof(brx_motion_jolt_physics_skeleton));
    assert(NULL != new_unwrapped_skeleton_base);

    brx_motion_jolt_physics_skeleton *new_unwrapped_skeleton = new (new_unwrapped_skeleton_base) brx_motion_jolt_physics_skeleton{};
    new_unwrapped_skeleton->init(skeleton_joint_count, skeleton_joint_names, skeleton_joint_parent_indices, skeleton_bind_pose_joint_transforms, vrm_skeleton_joint_indices);
    return new_unwrapped_skeleton;
}

extern "C" void brx_motion_destory_skeleton(brx_motion_skeleton *wrapped_skeleton)
{
    assert(NULL != wrapped_skeleton);
    brx_motion_jolt_physics_skeleton *delete_unwrapped_skeleton = static_cast<brx_motion_jolt_physics_skeleton *>(wrapped_skeleton);

    delete_unwrapped_skeleton->uninit();

    delete_unwrapped_skeleton->~brx_motion_jolt_physics_skeleton();
    mcrt_free(delete_unwrapped_skeleton);
}

brx_motion_jolt_physics_skeleton::brx_motion_jolt_physics_skeleton()
{
}

brx_motion_jolt_physics_skeleton::~brx_motion_jolt_physics_skeleton()
{
}

void brx_motion_jolt_physics_skeleton::init(uint32_t skeleton_joint_count, char const *const *skeleton_joint_names, uint32_t const *skeleton_joint_parent_indices, brx_motion_skeleton_joint_transform const *skeleton_bind_pose_joint_transforms, uint32_t const vrm_skeleton_joint_indices[BRX_MOTION_VRM_SKELETON_JOINT_NAME_COUNT])
{
    assert(this->m_skeleton_joint_names.empty());
    this->m_skeleton_joint_names.resize(static_cast<size_t>(skeleton_joint_count));

    for (int32_t joint_index = 0; joint_index < skeleton_joint_count; ++joint_index)
    {
        this->m_skeleton_joint_names[joint_index] = skeleton_joint_names[joint_index];
    }

    assert(this->m_skeleton_joint_parent_indices.empty());
    this->m_skeleton_joint_parent_indices.resize(static_cast<size_t>(skeleton_joint_count));

    std::memcpy(&this->m_skeleton_joint_parent_indices[0], skeleton_joint_parent_indices, sizeof(int32_t) * static_cast<size_t>(skeleton_joint_count));

    assert(this->m_skeleton_bind_pose_joint_transforms.empty());
    this->m_skeleton_bind_pose_joint_transforms.resize(static_cast<size_t>(skeleton_joint_count));

    std::memcpy(&this->m_skeleton_bind_pose_joint_transforms[0], skeleton_bind_pose_joint_transforms, sizeof(brx_motion_skeleton_joint_transform) * static_cast<size_t>(skeleton_joint_count));
}

void brx_motion_jolt_physics_skeleton::uninit()
{
}

uint32_t brx_motion_jolt_physics_skeleton::get_skeleton_joint_count() const
{
    return this->m_skeleton_joint_names.size();
}

mcrt_vector<mcrt_string> const *brx_motion_jolt_physics_skeleton::get_skeleton_joint_names() const
{
    return &this->m_skeleton_joint_names;
}

int32_t const *brx_motion_jolt_physics_skeleton::get_skeleton_joint_parent_indices() const
{
    return &this->m_skeleton_joint_parent_indices[0];
}

brx_motion_skeleton_joint_transform const *brx_motion_jolt_physics_skeleton::get_skeleton_bind_pose_joint_transforms() const
{
    return &this->m_skeleton_bind_pose_joint_transforms[0];
}

extern "C" brx_motion_skeleton_animation *brx_motion_create_skeleton_animation(uint32_t skeleton_joint_count, char const *const *skeleton_joint_names, uint32_t animation_frame_rate, uint32_t animation_frame_count, brx_motion_skeleton_joint_transform const *skeleton_animation_joint_transforms)
{
    void *new_unwrapped_skeleton_animation_base = mcrt_malloc(sizeof(brx_motion_jolt_physics_skeleton_animation), alignof(brx_motion_jolt_physics_skeleton_animation));
    assert(NULL != new_unwrapped_skeleton_animation_base);

    brx_motion_jolt_physics_skeleton_animation *new_unwrapped_skeleton_animation = new (new_unwrapped_skeleton_animation_base) brx_motion_jolt_physics_skeleton_animation{};
    new_unwrapped_skeleton_animation->init(skeleton_joint_count, skeleton_joint_names, animation_frame_rate, animation_frame_count, skeleton_animation_joint_transforms);
    return new_unwrapped_skeleton_animation;
}

extern "C" void brx_motion_destory_skeleton_animation(brx_motion_skeleton_animation *wrapped_skeleton_animation)
{
    assert(NULL != wrapped_skeleton_animation);
    brx_motion_jolt_physics_skeleton_animation *delete_unwrapped_skeleton_animation = static_cast<brx_motion_jolt_physics_skeleton_animation *>(wrapped_skeleton_animation);

    delete_unwrapped_skeleton_animation->uninit();

    delete_unwrapped_skeleton_animation->~brx_motion_jolt_physics_skeleton_animation();
    mcrt_free(delete_unwrapped_skeleton_animation);
}

brx_motion_jolt_physics_skeleton_animation::brx_motion_jolt_physics_skeleton_animation()
{
}

brx_motion_jolt_physics_skeleton_animation::~brx_motion_jolt_physics_skeleton_animation()
{
}

void brx_motion_jolt_physics_skeleton_animation::init(uint32_t skeleton_joint_count, char const *const *skeleton_joint_names, uint32_t animation_frame_rate, uint32_t animation_frame_count, brx_motion_skeleton_joint_transform const *skeleton_animation_joint_transforms)
{
    assert(this->m_skeleton_joint_names.empty());
    this->m_skeleton_joint_names.resize(static_cast<size_t>(skeleton_joint_count));

    for (int32_t joint_index = 0; joint_index < skeleton_joint_count; ++joint_index)
    {
        this->m_skeleton_joint_names[joint_index] = skeleton_joint_names[joint_index];
    }

    assert(this->m_skeleton_animation_joint_transforms.empty());
    this->m_skeleton_animation_joint_transforms.resize(static_cast<size_t>(skeleton_joint_count) * static_cast<size_t>(animation_frame_count));

    std::memcpy(&this->m_skeleton_animation_joint_transforms[0], skeleton_animation_joint_transforms, sizeof(brx_motion_skeleton_joint_transform) * (static_cast<size_t>(skeleton_joint_count) * static_cast<size_t>(animation_frame_count)));

    this->m_animation_frame_rate = animation_frame_rate;
}

void brx_motion_jolt_physics_skeleton_animation::uninit()
{
}

uint32_t brx_motion_jolt_physics_skeleton_animation::get_skeleton_joint_count() const
{
    return static_cast<uint32_t>(this->m_skeleton_joint_names.size());
}

mcrt_vector<mcrt_string> const *brx_motion_jolt_physics_skeleton_animation::get_skeleton_joint_names() const
{
    return &this->m_skeleton_joint_names;
}

uint32_t brx_motion_jolt_physics_skeleton_animation::get_animation_frame_count() const
{
    int const skeleton_joint_count = this->get_skeleton_joint_count();
    assert(0 == (this->m_skeleton_animation_joint_transforms.size() % static_cast<size_t>(skeleton_joint_count)));
    uint32_t const animation_frame_count = this->m_skeleton_animation_joint_transforms.size() / static_cast<size_t>(skeleton_joint_count);
    return animation_frame_count;
}

brx_motion_skeleton_joint_transform const *brx_motion_jolt_physics_skeleton_animation::get_skeleton_animation_joint_transforms() const
{
    return &this->m_skeleton_animation_joint_transforms[0];
}

uint32_t brx_motion_jolt_physics_skeleton_animation::get_animation_frame_rate() const
{
    return this->m_animation_frame_rate;
}

extern "C" brx_motion_skeleton_animation_instance *brx_motion_create_skeleton_animation_instance(brx_motion_skeleton_animation *skeleton_animation, brx_motion_skeleton *skeleton)
{
    void *new_unwrapped_skeleton_animation_instance_base = mcrt_malloc(sizeof(brx_motion_jolt_physics_skeleton_animation_instance), alignof(brx_motion_jolt_physics_skeleton_animation_instance));
    assert(NULL != new_unwrapped_skeleton_animation_instance_base);

    brx_motion_jolt_physics_skeleton_animation_instance *new_unwrapped_skeleton_animation_instance = new (new_unwrapped_skeleton_animation_instance_base) brx_motion_jolt_physics_skeleton_animation_instance{};
    new_unwrapped_skeleton_animation_instance->init(skeleton_animation, skeleton);
    return new_unwrapped_skeleton_animation_instance;
}

extern "C" void brx_motion_destory_skeleton_animation_instance(brx_motion_skeleton_animation_instance *wrapped_skeleton_animation_instance)
{
    assert(NULL != wrapped_skeleton_animation_instance);
    brx_motion_jolt_physics_skeleton_animation_instance *delete_unwrapped_skeleton_animation_instance = static_cast<brx_motion_jolt_physics_skeleton_animation_instance *>(wrapped_skeleton_animation_instance);

    delete_unwrapped_skeleton_animation_instance->uninit();

    delete_unwrapped_skeleton_animation_instance->~brx_motion_jolt_physics_skeleton_animation_instance();
    mcrt_free(delete_unwrapped_skeleton_animation_instance);
}

brx_motion_jolt_physics_skeleton_animation_instance::brx_motion_jolt_physics_skeleton_animation_instance() : m_animation_frame_rate(0U), m_animation_frame_count(0U), m_skeleton_joint_count(0), m_animation_duration(0.0F), m_animation_time(0.0F)
{
}

brx_motion_jolt_physics_skeleton_animation_instance::~brx_motion_jolt_physics_skeleton_animation_instance()
{
}

void brx_motion_jolt_physics_skeleton_animation_instance::init(brx_motion_skeleton_animation *wrapped_skeleton_animation, brx_motion_skeleton *wrapped_skeleton)
{
    // Animation Retargeting

    brx_motion_jolt_physics_skeleton_animation const *const unwrapped_skeleton_animation = static_cast<brx_motion_jolt_physics_skeleton_animation *>(wrapped_skeleton_animation);

    mcrt_vector<mcrt_string> const &skeleton1_joint_names = *unwrapped_skeleton_animation->get_skeleton_joint_names();

    brx_motion_skeleton_joint_transform const *const skeleton1_animation_joint_transforms = unwrapped_skeleton_animation->get_skeleton_animation_joint_transforms();

    int const skeleton1_joint_count = unwrapped_skeleton_animation->get_skeleton_joint_count();

    brx_motion_jolt_physics_skeleton const *const unwrapped_skeleton = static_cast<brx_motion_jolt_physics_skeleton *>(wrapped_skeleton);

    mcrt_vector<mcrt_string> const &skeleton2_joint_names = *unwrapped_skeleton->get_skeleton_joint_names();

    int32_t const *const skeleton2_joint_parent_indices = unwrapped_skeleton->get_skeleton_joint_parent_indices();

    brx_motion_skeleton_joint_transform const *const skeleton_bind_pose_joint_transforms = unwrapped_skeleton->get_skeleton_bind_pose_joint_transforms();

    int const skeleton2_joint_count = unwrapped_skeleton->get_skeleton_joint_count();

    uint32_t const animation_frame_count = unwrapped_skeleton_animation->get_animation_frame_count();

    assert(this->m_skeleton_animation_joint_transforms.empty());
    this->m_skeleton_animation_joint_transforms.resize(static_cast<size_t>(skeleton2_joint_count) * static_cast<size_t>(animation_frame_count));

    {
        JPH::Ref<JPH::SkeletonMapper> skeleton_mapper = new JPH::SkeletonMapper;

        JPH::SkeletonPose skeleton2_animation_pose;

        {
            JPH::Ref<JPH::Skeleton> skeleton1 = new JPH::Skeleton;
            for (int skeleton1_joint_index = 0; skeleton1_joint_index < skeleton1_joint_count; ++skeleton1_joint_index)
            {
                skeleton1->AddJoint(skeleton1_joint_names[skeleton1_joint_index], -1);
            }

            JPH::SkeletonPose skeleton1_bind_pose;
            {
                skeleton1_bind_pose.SetSkeleton(skeleton1);
                for (int skeleton1_joint_index = 0; skeleton1_joint_index < skeleton1_joint_count; ++skeleton1_joint_index)
                {
                    // we assume frame 0 is binding pose
                    JPH::Quat skeleton1_bind_pose_rotation(
                        skeleton1_animation_joint_transforms[skeleton1_joint_count * 0U + skeleton1_joint_index].m_rotation[0],
                        skeleton1_animation_joint_transforms[skeleton1_joint_count * 0U + skeleton1_joint_index].m_rotation[1],
                        skeleton1_animation_joint_transforms[skeleton1_joint_count * 0U + skeleton1_joint_index].m_rotation[2],
                        skeleton1_animation_joint_transforms[skeleton1_joint_count * 0U + skeleton1_joint_index].m_rotation[3]);
                    JPH::Vec3 skeleton1_bind_pose_translation(
                        skeleton1_animation_joint_transforms[skeleton1_joint_count * 0U + skeleton1_joint_index].m_translation[0],
                        skeleton1_animation_joint_transforms[skeleton1_joint_count * 0U + skeleton1_joint_index].m_translation[1],
                        skeleton1_animation_joint_transforms[skeleton1_joint_count * 0U + skeleton1_joint_index].m_translation[2]);
                    skeleton1_bind_pose.GetJointMatrix(skeleton1_joint_index) = JPH::Mat44::sRotationTranslation(skeleton1_bind_pose_rotation, skeleton1_bind_pose_translation);
                }

                // sync from the model space to the local space
                skeleton1_bind_pose.CalculateJointStates();
            }

            JPH::Ref<JPH::Skeleton> skeleton2 = new JPH::Skeleton;
            {
                for (int skeleton2_joint_index = 0; skeleton2_joint_index < skeleton2_joint_count; ++skeleton2_joint_index)
                {
                    skeleton2->AddJoint(skeleton2_joint_names[skeleton2_joint_index], skeleton2_joint_parent_indices[skeleton2_joint_index]);
                }

                assert(skeleton2->AreJointsCorrectlyOrdered());
            }

            JPH::SkeletonPose skeleton2_bind_pose;
            {
                skeleton2_bind_pose.SetSkeleton(skeleton2);
                // set the model space
                for (int skeleton2_joint_index = 0; skeleton2_joint_index < skeleton2_joint_count; ++skeleton2_joint_index)
                {
                    JPH::Quat bind_pose_rotation(
                        skeleton_bind_pose_joint_transforms[skeleton2_joint_index].m_rotation[0],
                        skeleton_bind_pose_joint_transforms[skeleton2_joint_index].m_rotation[1],
                        skeleton_bind_pose_joint_transforms[skeleton2_joint_index].m_rotation[2],
                        skeleton_bind_pose_joint_transforms[skeleton2_joint_index].m_rotation[3]);
                    JPH::Vec3 bind_pose_translation(
                        skeleton_bind_pose_joint_transforms[skeleton2_joint_index].m_translation[0],
                        skeleton_bind_pose_joint_transforms[skeleton2_joint_index].m_translation[1],
                        skeleton_bind_pose_joint_transforms[skeleton2_joint_index].m_translation[2]);
                    skeleton2_bind_pose.GetJointMatrix(skeleton2_joint_index) = JPH::Mat44::sRotationTranslation(bind_pose_rotation, bind_pose_translation);
                }
                // sync from the model space to the local space
                skeleton2_bind_pose.CalculateJointStates();
            }

            skeleton_mapper->Initialize(skeleton1, skeleton1_bind_pose.GetJointMatrices().data(), skeleton2_bind_pose.GetSkeleton(), skeleton2_bind_pose.GetJointMatrices().data());

            // skeleton_mapper->LockAllTranslations(skeleton2_bind_pose.GetSkeleton(), skeleton2_bind_pose.GetJointMatrices().data());

            skeleton1 = NULL;

            skeleton2 = NULL;

            // use the bind pose as the initial state
            skeleton2_animation_pose = skeleton2_bind_pose;
        }

        for (uint32_t animation_frame_index = 0U; animation_frame_index < animation_frame_count; ++animation_frame_index)
        {
            JPH::Array<JPH::Mat44> skeleton1_model_space_pose(static_cast<size_t>(skeleton1_joint_count));
            for (int skeleton1_joint_index = 0; skeleton1_joint_index < skeleton1_joint_count; ++skeleton1_joint_index)
            {
                JPH::Quat animation_rotation(
                    skeleton1_animation_joint_transforms[skeleton1_joint_count * animation_frame_index + skeleton1_joint_index].m_rotation[0],
                    skeleton1_animation_joint_transforms[skeleton1_joint_count * animation_frame_index + skeleton1_joint_index].m_rotation[1],
                    skeleton1_animation_joint_transforms[skeleton1_joint_count * animation_frame_index + skeleton1_joint_index].m_rotation[2],
                    skeleton1_animation_joint_transforms[skeleton1_joint_count * animation_frame_index + skeleton1_joint_index].m_rotation[3]);
                JPH::Vec3 animation_translation(
                    skeleton1_animation_joint_transforms[skeleton1_joint_count * animation_frame_index + skeleton1_joint_index].m_translation[0],
                    skeleton1_animation_joint_transforms[skeleton1_joint_count * animation_frame_index + skeleton1_joint_index].m_translation[1],
                    skeleton1_animation_joint_transforms[skeleton1_joint_count * animation_frame_index + skeleton1_joint_index].m_translation[2]);
                skeleton1_model_space_pose[skeleton1_joint_index] = JPH::Mat44::sRotationTranslation(animation_rotation, animation_translation);
            }

            JPH::Array<JPH::Mat44> skeleton2_local_space_pose(static_cast<size_t>(skeleton2_animation_pose.GetJointCount()));
            skeleton2_animation_pose.CalculateLocalSpaceJointMatrices(skeleton2_local_space_pose.data());

            // write into the model space
            skeleton_mapper->Map(skeleton1_model_space_pose.data(), skeleton2_local_space_pose.data(), skeleton2_animation_pose.GetJointMatrices().data());

            // output the model space
            for (int skeleton2_joint_index = 0; skeleton2_joint_index < skeleton2_joint_count; ++skeleton2_joint_index)
            {
                JPH::Mat44 const &animation_transform = skeleton2_animation_pose.GetJointMatrix(skeleton2_joint_index);
                animation_transform.GetQuaternion().GetXYZW().StoreFloat4(reinterpret_cast<JPH::Float4 *>(&this->m_skeleton_animation_joint_transforms[skeleton2_joint_count * animation_frame_index + skeleton2_joint_index].m_rotation[0]));
                animation_transform.GetTranslation().StoreFloat3(reinterpret_cast<JPH::Float3 *>(&this->m_skeleton_animation_joint_transforms[skeleton2_joint_count * animation_frame_index + skeleton2_joint_index].m_translation[0]));
            }

            // sync from the model space into the local space (to be used as input for skeleton mapping in next frame)
            skeleton2_animation_pose.CalculateJointStates();
        }

        skeleton_mapper = NULL;
    }

    uint32_t const animation_frame_rate = unwrapped_skeleton_animation->get_animation_frame_rate();

    assert(0U == this->m_animation_frame_rate);
    this->m_animation_frame_rate = animation_frame_rate;

    assert(0U == this->m_animation_frame_count);
    this->m_animation_frame_count = animation_frame_count;

    assert(0 == this->m_skeleton_joint_count);
    this->m_skeleton_joint_count = skeleton2_joint_count;

    assert(0.0F == this->m_animation_duration);
    this->m_animation_duration = static_cast<float>(static_cast<double>(animation_frame_count) / static_cast<double>(animation_frame_rate));
}

void brx_motion_jolt_physics_skeleton_animation_instance::uninit()
{
}

void brx_motion_jolt_physics_skeleton_animation_instance::step(float delta_time)
{
    this->m_animation_time = this->m_animation_time + delta_time;

    if (this->m_animation_time > this->m_animation_duration)
    {
        this->m_animation_time -= this->m_animation_duration;
    }

    if (this->m_animation_time < 0.0)
    {
        this->m_animation_time = 0.0;
    }
}

brx_motion_skeleton_joint_transform const *brx_motion_jolt_physics_skeleton_animation_instance::get_skeleton_animation_joint_transforms() const
{
    uint32_t const animation_frame_index = static_cast<uint32_t>(std::max(static_cast<int64_t>(0), static_cast<int64_t>(static_cast<double>(this->m_animation_frame_rate) * static_cast<double>(this->m_animation_time)))) % this->m_animation_frame_count;

    return &this->m_skeleton_animation_joint_transforms[this->m_skeleton_joint_count * animation_frame_index];
}

#include <Jolt/Core/Memory.h>

JPH_NAMESPACE_BEGIN

static void *AllocateHook(size_t inSize)
{
    return mcrt_malloc(inSize, 16U);
}

static void *ReallocateHook(void *inOldBlock, size_t inOldSize, size_t inNewSize)
{
    if ((NULL != inOldBlock) && (0U < inNewSize))
    {
        void *inNewBlock = mcrt_malloc(inNewSize, 16U);
        if (NULL != inNewBlock)
        {
            std::memcpy(inNewBlock, inOldBlock, ((inOldSize < inNewSize) ? inOldSize : inNewSize));
            mcrt_free(inOldBlock);
        }
        return inNewBlock;
    }
    else if (0U < inNewSize)
    {
        return mcrt_malloc(inNewSize, 16U);
    }
    else if (NULL != inOldBlock)
    {
        mcrt_free(inOldBlock);
        return NULL;
    }
    else
    {
        return NULL;
    }
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
