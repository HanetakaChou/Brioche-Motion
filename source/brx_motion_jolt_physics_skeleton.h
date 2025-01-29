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

#ifndef _BRX_MOTION_JOLT_PHYSICS_SKELETON_H_
#define _BRX_MOTION_JOLT_PHYSICS_SKELETON_H_ 1

#include "../include/brx_motion.h"
#include "../../McRT-Malloc/include/mcrt_vector.h"
#include "../../McRT-Malloc/include/mcrt_string.h"

class brx_motion_jolt_physics_skeleton final : public brx_motion_skeleton
{
	mcrt_vector<mcrt_string> m_skeleton_joint_names;
	mcrt_vector<int32_t> m_skeleton_joint_parent_indices;
	mcrt_vector<brx_motion_skeleton_joint_transform> m_skeleton_bind_pose_joint_transforms;

public:
	brx_motion_jolt_physics_skeleton();
	~brx_motion_jolt_physics_skeleton();
	void init(uint32_t skeleton_joint_count, char const *const *skeleton_joint_names, uint32_t const *skeleton_joint_parent_indices, brx_motion_skeleton_joint_transform const *skeleton_bind_pose_joint_transforms, uint32_t const vrm_skeleton_joint_indices[BRX_MOTION_VRM_SKELETON_JOINT_NAME_COUNT]);
	void uninit();
	uint32_t get_skeleton_joint_count() const override;
	mcrt_vector<mcrt_string> const *get_skeleton_joint_names() const;
	int32_t const *get_skeleton_joint_parent_indices() const;
	brx_motion_skeleton_joint_transform const *get_skeleton_bind_pose_joint_transforms() const override;
};

class brx_motion_jolt_physics_skeleton_animation final : public brx_motion_skeleton_animation
{
	mcrt_vector<mcrt_string> m_skeleton_joint_names;
	// Model Space
	mcrt_vector<brx_motion_skeleton_joint_transform> m_skeleton_animation_joint_transforms;
	uint32_t m_animation_frame_rate;

public:
	brx_motion_jolt_physics_skeleton_animation();
	~brx_motion_jolt_physics_skeleton_animation();
	void init(uint32_t skeleton_joint_count, char const *const *skeleton_joint_names, uint32_t animation_frame_rate, uint32_t animation_frame_count, brx_motion_skeleton_joint_transform const *skeleton_animation_joint_transforms);
	void uninit();
	uint32_t get_skeleton_joint_count() const;
	mcrt_vector<mcrt_string> const *get_skeleton_joint_names() const;
	uint32_t get_animation_frame_count() const;
	brx_motion_skeleton_joint_transform const *get_skeleton_animation_joint_transforms() const;
	uint32_t get_animation_frame_rate() const;
};

class brx_motion_jolt_physics_skeleton_animation_instance final : public brx_motion_skeleton_animation_instance
{
	mcrt_vector<brx_motion_skeleton_joint_transform> m_skeleton_animation_joint_transforms;
	uint32_t m_animation_frame_rate;
	uint32_t m_animation_frame_count;
	uint32_t m_skeleton_joint_count;
	float m_animation_duration;
	float m_animation_time;

public:
	brx_motion_jolt_physics_skeleton_animation_instance();
	~brx_motion_jolt_physics_skeleton_animation_instance();
	void init(brx_motion_skeleton_animation *skeleton_animation, brx_motion_skeleton *skeleton);
	void uninit();

private:
	void step(float delta_time) override;
	brx_motion_skeleton_joint_transform const *get_skeleton_animation_joint_transforms() const override;
};

class brx_motion_humanoid_skeleton_pose
{

};


// [saba](https://github.com/benikabocha/saba)  

// GLMMDModel::UpdateAnimation // focus on
// GLMMDModel::Update (PMXModel::Update) // Morph & Skin Deformation on CPU (deprecated)  

// GLMMDModel::EvaluateAnimation // sample animation
// PMXModel::UpdateMorphAnimation 
// PMXModel::UpdateNodeAnimation (before Physics) // we can bake the result before ragdoll // we do not bake ragdoll // the motion input can be from the user (motion capture)  
// PMXModel::UpdatePhysicsAnimation // Ragdoll   
// PMXModel::UpdateNodeAnimation (after Physics) // we may ignore   


// PMXModel::UpdateNodeAnimation
//  PMXModel::UpdateLocalTransform  // s r t from EvaluateAnimation (and append) to matrix 
//	PMXModel::UpdateGlobalTransform // sync from local to model space // use recursive ??? // actually do not need if the nodes are sorted?
//  PMXMOdel::UpdateAppendTransform // Append Transform (MMD unique) // create dummy/shadow bone like blender mmd tools?
//             UpdateLocalTransform
//  PMXMOdel::UpdateGlobalTransform
//          Foot IK Solver
//  PMXMOdel::UpdateGlobalTransform

// KinematicMotionState
// DynamicMotionState
// DynamicAndBoneMergeMotionState

// PMXModel::UpdatePhysicsAnimation // Ragdoll
//   MMDRigidBody::SetActivation
//     btMotionState::getWorldTransform (callback) // sync from (btTransform m_transform) to physics (use skeleton mapper) // Skeleton Mapper from animation to Ragdoll // JoltPhysics // Ragdoll::DriveToPoseUsingKinematics // Ragdoll::DriveToPoseUsingMotors // We should only drive some of the rigidbodies // We can drive all rigidbodies (but when perform the skeleton mapper, only the Kinematic rigidbodies are mapped)
//   MMDPhysics::Update
//     btMotionState::setWorldTransform (callback) // we use // JoltPhysics // Ragdoll::GetPose
//   MMDPhysics::ReflectGlobalTransform // sync from physics to animation (model space) // m_invOffset // mJoint1To2 of "Skeleton Mapper"   // Kinematic (not affected) // Dynamic (affected) // DynamicAndBoneMerge (affected but remain the Z axis of the animation) // root offset (motion extraction?)  
//   MMDRigidBody::CalcLocalTransform // calculate the local space of the joints affected by physics (use SkeletonPose::CalculateJointStates)

// btMotionState::getWorldTransform 
// Kinematic (get from animation (consider offset)
// Dynamic or DynamicAndBone (do not affected by animation

// MMDRigidBody::SetActivation
// XXXMotionState::n_transform (btTransform ) // Ragdoll Pose (include all rigidbodies)
// m_offset // SkeletonMapper::Mapping::mJoint1To2 (only valid for Kinematic)
// SkeletonMapper AnimationToRagdoll // only map the Kinematic rigidbodies // modify the "CanMapJoint" function of SkeletonMapper::Initialize  

// Ragdoll::DriveToPoseUsingKinematics // it is safe to drive all rigidbodies // since only the kinematic are mapped from animation  

// MMDPhysics::Update
// physics simulation
// Ragdoll::GetPose 

// MMDNode::m_local // Animation Pose
// MMDNode::m_global

// MMDPhysics::ReflectGlobalTransform (write into the model space
// m_invOffset // SkeletonMapper::Mapping::mJoint1To2  (only valid for Dynamic or DynamicAndBoneMerge)
// SkeletonMapper RagdollToAnimation // only map the Dynamic or DynamicAndBoneMerge rigidbodies // modify the "CanMapJoint" function of SkeletonMapper::Initialize

// DynamicAndBoneMerge (affected but remain the Z axis of the animation) ??? // Perhaps related to the root offset (motion extract???)  

// MMDRigidBody::CalcLocalTransform (sync from model space to localspace) 
// use SkeletonPose::CalculateJointStates

#endif
