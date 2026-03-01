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

#ifndef _BRX_MOTION_VMC_MOTION_RECEIVER_H_
#define _BRX_MOTION_VMC_MOTION_RECEIVER_H_ 1

#include "../include/brx_motion.h"
#include "brx_motion_motion_capture.h"

#if defined(__GNUC__)
// GCC or CLANG
#define USE_WSA 0
#elif defined(_MSC_VER)
// MSVC or CLANG-CL
#define USE_WSA 1
#define NOMINMAX 1
#define WIN32_LEAN_AND_MEAN 1
#include <sdkddkver.h>
#include <Windows.h>
#include <winsock2.h>
#else
#error Unknown Compiler
#endif

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

enum
{
    INTERNAL_VMC_MORPH_JOINT_NAME_RIGHT_EYE = 0,
    INTERNAL_VMC_MORPH_JOINT_NAME_LEFT_EYE = 1,
    INTERNAL_VMC_MORPH_JOINT_NAME_COUNT = 2
};

enum
{
    INTERNAL_VMC_SKELETON_JOINT_NAME_CENTER = 0,
    INTERNAL_VMC_SKELETON_JOINT_NAME_UPPER_BODY = 1,
    INTERNAL_VMC_SKELETON_JOINT_NAME_UPPER_BODY_2 = 2,
    INTERNAL_VMC_SKELETON_JOINT_NAME_UPPER_BODY_3 = 3,
    INTERNAL_VMC_SKELETON_JOINT_NAME_NECK = 4,
    INTERNAL_VMC_SKELETON_JOINT_NAME_HEAD = 5,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_EYE = 6,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_EYE = 7,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_LEG = 8,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_KNEE = 9,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_ANKLE = 10,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_TOE_TIP = 11,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_LEG = 12,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_KNEE = 13,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_ANKLE = 14,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_TOE_TIP = 15,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_SHOULDER = 16,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_ARM = 17,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_ELBOW = 18,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_WRIST = 19,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_SHOULDER = 20,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_ARM = 21,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_ELBOW = 22,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_WRIST = 23,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_THUMB_0 = 24,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_THUMB_1 = 25,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_THUMB_2 = 26,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_THUMB_TIP = 27,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_INDEX_FINGER_1 = 28,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_INDEX_FINGER_2 = 29,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_INDEX_FINGER_3 = 30,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_INDEX_FINGER_TIP = 31,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_MIDDLE_FINGER_1 = 32,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_MIDDLE_FINGER_2 = 33,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_MIDDLE_FINGER_3 = 34,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_MIDDLE_FINGER_TIP = 35,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_RING_FINGER_1 = 36,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_RING_FINGER_2 = 37,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_RING_FINGER_3 = 38,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_RING_FINGER_TIP = 39,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_LITTLE_FINGER_1 = 40,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_LITTLE_FINGER_2 = 41,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_LITTLE_FINGER_3 = 42,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_LITTLE_FINGER_TIP = 43,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_THUMB_0 = 44,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_THUMB_1 = 45,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_THUMB_2 = 46,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_THUMB_TIP = 47,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_INDEX_FINGER_1 = 48,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_INDEX_FINGER_2 = 49,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_INDEX_FINGER_3 = 50,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_INDEX_FINGER_TIP = 51,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_MIDDLE_FINGER_1 = 52,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_MIDDLE_FINGER_2 = 53,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_MIDDLE_FINGER_3 = 54,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_MIDDLE_FINGER_TIP = 55,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_RING_FINGER_1 = 56,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_RING_FINGER_2 = 57,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_RING_FINGER_3 = 58,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_RING_FINGER_TIP = 59,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_LITTLE_FINGER_1 = 60,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_LITTLE_FINGER_2 = 61,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_LITTLE_FINGER_3 = 62,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_LITTLE_FINGER_TIP = 63,
    INTERNAL_VMC_SKELETON_JOINT_NAME_COUNT = 64
};

struct brx_motion_vmc_rigid_transform
{
    DirectX::XMFLOAT4 m_rotation;
    DirectX::XMFLOAT3 m_translation;
};

class brx_motion_vmc_motion_receiver final : public internal_brx_motion_motion_capture, public brx_motion_motion_receiver
{
    uint32_t m_ref_count;
#if defined(USE_WSA)
#if USE_WSA
    SOCKET m_socket_descriptor;
#else
    int m_socket_descriptor;
#endif
#else
#error "0 or 1"
#endif
    // VRM 0.0: false
    // VRM 1.0: true
    bool m_vrmc_vrm;
    bool m_apply;
    brx_motion_vmc_rigid_transform m_model_transform;
    float m_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_COUNT];
    DirectX::XMFLOAT4 m_morph_joint_weights[INTERNAL_VMC_MORPH_JOINT_NAME_COUNT];
    brx_motion_vmc_rigid_transform m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_COUNT];
    brx_motion_vmc_rigid_transform m_skeleton_joint_transforms_model_space[INTERNAL_VMC_SKELETON_JOINT_NAME_COUNT];

public:
    brx_motion_vmc_motion_receiver();
    ~brx_motion_vmc_motion_receiver();
    bool init(uint16_t port);
    void uninit();

private:
    inline void internal_retain();
    inline uint32_t internal_release();

    brx_motion_video_detector *get_video_detector() override;
    brx_motion_motion_receiver *get_motion_receiver() override;
    brx_motion_motion_capture *get_motion_capture() override;

    uint32_t get_hand_count() const override;
    uint32_t get_face_count() const override;
    uint32_t get_pose_count() const override;
    void step() override;
    brx_motion_rigid_transform get_model_transform() const override;
    float get_morph_target_weight(uint32_t face_index, BRX_MOTION_MORPH_TARGET_NAME morph_target_name) const override;

    double get_delta_time() const override;
    DirectX::XMFLOAT4 get_face_morph_joint_weight(uint32_t face_index, BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name) const override;
    DirectX::XMFLOAT3 const *get_pose_skeleton_joint_translation(uint32_t pose_index, BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name) const override;
    DirectX::XMFLOAT4 const *get_face_skeleton_joint_rotation(uint32_t face_index, BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name) const override;
    DirectX::XMFLOAT3 const *get_hand_skeleton_joint_translation(uint32_t hand_index, BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name) const override;

    void retain() override;
    void release() override;
};

#endif
