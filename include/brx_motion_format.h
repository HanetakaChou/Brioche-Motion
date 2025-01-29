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

#ifndef _BRX_MOTION_FORMAT_H_
#define _BRX_MOTION_FORMAT_H_ 1

#include <cstddef>
#include <cstdint>

// https://github.com/vrm-c/vrm-specification/blob/master/specification/0.0/README.md#predefined-expression-name
// https://github.com/vrm-c/vrm-specification/blob/master/specification/VRMC_vrm-1.0/expressions.md#preset-expressions
enum BRX_MOTION_VRM_MORPH_TARGET_NAME
{
    BRX_MOTION_VRM_MORPH_TARGET_NAME_NEUTRAL = 0,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_A = 1,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_I = 2,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_U = 3,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_E = 4,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_O = 5,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_BLINK = 6,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_BLINK_L = 7,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_BLINK_R = 8,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_FUN = 9,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_ANGRY = 10,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_SORROW = 11,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_JOY = 12,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_SURPRISED = 13,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_LOOK_UP = 14,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_LOOK_DOWN = 15,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_LOOK_LEFT = 16,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_LOOK_RIGHT = 17,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_COUNT = 18
};

// https://developer.apple.com/documentation/arkit/arfaceanchor/blendshapelocation
static constexpr uint32_t const BRX_MOTION_ARKIT_MORPH_TARGET_NAME_COUNT = 52U;

// https://github.com/vrm-c/vrm-specification/blob/master/specification/0.0/README.md#defined-bones
// https://github.com/vrm-c/vrm-specification/blob/master/specification/VRMC_vrm-1.0/humanoid.md#list-of-humanoid-bones
// https://github.com/saturday06/VRM-Addon-for-Blender/blob/main/src/io_scene_vrm/common/human_bone_mapper/mmd_mapping.py
// https://docs.unity3d.com/ScriptReference/HumanBodyBones.html
// https://developer.apple.com/documentation/arkit/validating-a-model-for-motion-capture
// https://developer.apple.com/documentation/arkit/arskeleton/jointname
// https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker
// https://cmu-perceptual-computing-lab.github.io/openpose/web/html/doc/md_doc_02_output.html
enum BRX_MOTION_VRM_SKELETON_JOINT_NAME
{
    BRX_MOTION_VRM_SKELETON_JOINT_NAME_HIPS = 0,
    BRX_MOTION_VRM_SKELETON_JOINT_NAME_COUNT = 55
};

static constexpr uint32_t BRX_MOTION_SKELETON_JOINT_INDEX_INVALID = static_cast<uint32_t>(~static_cast<uint32_t>(0U));

struct brx_motion_skeleton_joint_transform
{
    float m_rotation[4];
    float m_translation[3];
};

#endif
