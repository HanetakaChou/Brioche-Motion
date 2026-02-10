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

#include "brx_motion_media_pipe_video_detector.h"
#include "internal_tflite.h"
#include "brx_motion_opencv_video_capture.h"
#include "../../Brioche-Window-System-Integration/include/brx_wsi.h"
#include "../../McRT-Malloc/include/mcrt_tick_count.h"
#include "../../McRT-Malloc/include/mcrt_malloc.h"
#include <mediapipe/tasks/c/vision/hand_landmarker/hand_landmarker.h>
#include <mediapipe/tasks/c/vision/face_landmarker/face_landmarker.h>
#include <mediapipe/tasks/c/vision/pose_landmarker/pose_landmarker.h>
#include <cassert>
#include <cstring>
#include <algorithm>
#include <new>

#include "../models/hand_landmarker_task.h"
#include "../models/face_landmarker_task.h"
#include "../models/pose_landmarker_task.h"

static constexpr uint32_t const INTERNAL_MEIDA_PIPE_MAX_POSE_COUNT = 5U;
static constexpr uint32_t const INTERNAL_MEIDA_PIPE_MAX_FACE_COUNT = 5U;
static constexpr uint32_t const INTERNAL_MEIDA_PIPE_MAX_HAND_COUNT = 5U;

// https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker#models
enum
{
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_NOSE = 0,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_EYE_INNER = 1,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_EYE = 2,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_EYE_OUTER = 3,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_EYE_INNER = 4,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_EYE = 5,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_EYE_OUTER = 6,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_EAR = 7,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_EAR = 8,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_MOUTH_LEFT = 9,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_MOUTH_RIGHT = 10,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_SHOULDER = 11,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_SHOULDER = 12,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_ELBOW = 13,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_ELBOW = 14,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_WRIST = 15,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_WRIST = 16,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_PINKY = 17,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_PINKY = 18,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_INDEX = 19,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_INDEX = 20,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_THUMB = 21,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_THUMB = 22,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_HIP = 23,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_HIP = 24,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_KNEE = 25,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_KNEE = 26,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_ANKLE = 27,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_ANKLE = 28,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_HEEL = 29,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_HEEL = 30,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_FOOT = 31,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_FOOT = 32,
    INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_COUNT = 33
};

// https://ai.google.dev/edge/mediapipe/solutions/vision/face_landmarker#models
enum
{
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_NEUTRAL = 0,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_BROW_DOWN_LEFT = 1,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_BROW_DOWN_RIGHT = 2,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_BROW_INNER_UP = 3,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_BROW_OUTER_UP_LEFT = 4,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_BROW_OUTER_UP_RIGHT = 5,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_CHEEK_PUFF = 6,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_CHEEK_SQUINT_LEFT = 7,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_CHEEK_SQUINT_RIGHT = 8,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_BLINK_LEFT = 9,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_BLINK_RIGHT = 10,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_LOOK_DOWN_LEFT = 11,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_LOOK_DOWN_RIGHT = 12,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_LOOK_IN_LEFT = 13,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_LOOK_IN_RIGHT = 14,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_LOOK_OUT_LEFT = 15,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_LOOK_OUT_RIGHT = 16,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_LOOK_UP_LEFT = 17,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_LOOK_UP_RIGHT = 18,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_SQUINT_LEFT = 19,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_SQUINT_RIGHT = 20,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_WIDE_LEFT = 21,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_WIDE_RIGHT = 22,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_JAW_FORWARD = 23,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_JAW_LEFT = 24,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_JAW_OPEN = 25,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_JAW_RIGHT = 26,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_CLOSE = 27,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_DIMPLE_LEFT = 28,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_DIMPLE_RIGHT = 29,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_FROWN_LEFT = 30,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_FROWN_RIGHT = 31,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_FUNNEL = 32,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_LEFT = 33,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_LOWER_DOWN_LEFT = 34,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_LOWER_DOWN_RIGHT = 35,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_PRESS_LEFT = 36,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_PRESS_RIGHT = 37,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_PUCKER = 38,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_RIGHT = 39,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_ROLL_LOWER = 40,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_ROLL_UPPER = 41,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_SHRUG_LOWER = 42,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_SHRUG_UPPER = 43,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_SMILE_LEFT = 44,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_SMILE_RIGHT = 45,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_STRETCH_LEFT = 46,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_STRETCH_RIGHT = 47,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_UPPER_UP_LEFT = 48,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_UPPER_UP_RIGHT = 49,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_NOSE_SNEER_LEFT = 50,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_NOSE_SNEER_RIGHT = 51,
    INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_COUNT = 52
};

static constexpr char const *const internal_media_pipe_face_weight_name_strings[] = {
    "_neutral",
    "browDownLeft",
    "browDownRight",
    "browInnerUp",
    "browOuterUpLeft",
    "browOuterUpRight",
    "cheekPuff",
    "cheekSquintLeft",
    "cheekSquintRight",
    "eyeBlinkLeft",
    "eyeBlinkRight",
    "eyeLookDownLeft",
    "eyeLookDownRight",
    "eyeLookInLeft",
    "eyeLookInRight",
    "eyeLookOutLeft",
    "eyeLookOutRight",
    "eyeLookUpLeft",
    "eyeLookUpRight",
    "eyeSquintLeft",
    "eyeSquintRight",
    "eyeWideLeft",
    "eyeWideRight",
    "jawForward",
    "jawLeft",
    "jawOpen",
    "jawRight",
    "mouthClose",
    "mouthDimpleLeft",
    "mouthDimpleRight",
    "mouthFrownLeft",
    "mouthFrownRight",
    "mouthFunnel",
    "mouthLeft",
    "mouthLowerDownLeft",
    "mouthLowerDownRight",
    "mouthPressLeft",
    "mouthPressRight",
    "mouthPucker",
    "mouthRight",
    "mouthRollLower",
    "mouthRollUpper",
    "mouthShrugLower",
    "mouthShrugUpper",
    "mouthSmileLeft",
    "mouthSmileRight",
    "mouthStretchLeft",
    "mouthStretchRight",
    "mouthUpperUpLeft",
    "mouthUpperUpRight",
    "noseSneerLeft",
    "noseSneerRight"};

// https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker#models
enum
{
    INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_WRIST = 0,
    INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_THUMB_CMC = 1,
    INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_THUMB_MCP = 2,
    INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_THUMB_IP = 3,
    INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_THUMB_TIP = 4,
    INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_INDEX_FINGER_MCP = 5,
    INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_INDEX_FINGER_PIP = 6,
    INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_INDEX_FINGER_DIP = 7,
    INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_INDEX_FINGER_TIP = 8,
    INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_MIDDLE_FINGER_MCP = 9,
    INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_MIDDLE_FINGER_PIP = 10,
    INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_MIDDLE_FINGER_DIP = 11,
    INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_MIDDLE_FINGER_TIP = 12,
    INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_RING_FINGER_MCP = 13,
    INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_RING_FINGER_PIP = 14,
    INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_RING_FINGER_DIP = 15,
    INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_RING_FINGER_TIP = 16,
    INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_PINKY_MCP = 17,
    INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_PINKY_PIP = 18,
    INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_PINKY_DIP = 19,
    INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_PINKY_TIP = 20,
    INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_COUNT = 21
};

static inline uint32_t internal_get_face_morph_joint_index(BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name);

static inline uint32_t internal_get_pose_skeleton_joint_index(BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name);

static inline uint32_t internal_get_face_skeleton_joint_index(BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name);

static inline uint32_t internal_get_hand_skeleton_joint_index(BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name);

extern "C" brx_motion_video_detector *brx_motion_create_video_detector(uint32_t hand_count, uint32_t face_count, uint32_t pose_count, bool force_gpu, brx_motion_video_capture *video_capture)
{
    void *new_unwrapped_video_detector_base = mcrt_malloc(sizeof(brx_motion_media_pipe_video_detector), alignof(brx_motion_media_pipe_video_detector));
    assert(NULL != new_unwrapped_video_detector_base);

    brx_motion_media_pipe_video_detector *new_unwrapped_video_detector = new (new_unwrapped_video_detector_base) brx_motion_media_pipe_video_detector{};
    if (new_unwrapped_video_detector->init(hand_count, face_count, pose_count, force_gpu, video_capture))
    {
        return new_unwrapped_video_detector;
    }
    else
    {
        new_unwrapped_video_detector->~brx_motion_media_pipe_video_detector();
        mcrt_free(new_unwrapped_video_detector);
        return NULL;
    }
}

extern "C" void brx_motion_destroy_video_detector(brx_motion_video_detector *wrapped_video_detector)
{
    assert(NULL != wrapped_video_detector);
    internal_brx_motion_video_detector *const release_unwrapped_video_detector = static_cast<internal_brx_motion_video_detector *>(wrapped_video_detector);

    release_unwrapped_video_detector->release();
}

brx_motion_media_pipe_video_detector::brx_motion_media_pipe_video_detector() : m_ref_count(0U), m_pose_count(0U), m_face_count(0U), m_hand_count(0U), m_pose_landmarker(NULL), m_face_landmarker(NULL), m_hand_landmarker(NULL), m_delta_time(0.0), m_timestamp_ms(0), m_enable_debug_renderer(false), m_debug_renderer_window(NULL), m_input_video_capture(NULL)
{
}

brx_motion_media_pipe_video_detector::~brx_motion_media_pipe_video_detector()
{
    assert(0U == this->m_ref_count);
    assert(0U == this->m_pose_count);
    assert(0U == this->m_face_count);
    assert(0U == this->m_hand_count);
    assert(NULL == this->m_pose_landmarker);
    assert(NULL == this->m_face_landmarker);
    assert(NULL == this->m_hand_landmarker);
    assert(this->m_poses_skeleton_joint_translations_model_space_valid.empty());
    assert(this->m_poses_skeleton_joint_translations_model_space.empty());
    assert(this->m_faces_morph_target_weights.empty());
    assert(this->m_faces_morph_joint_weights.empty());
    assert(this->m_faces_skeleton_joint_rotations.empty());
    assert(this->m_hands_skeleton_joint_translations_model_space_valid.empty());
    assert(this->m_hands_skeleton_joint_translations_model_space.empty());
    assert(!this->m_enable_debug_renderer);
    assert(NULL == this->m_input_video_capture);
}

extern void internal_retain_video_capture(brx_motion_video_capture *wrapped_video_capture);

extern void internal_release_video_capture(brx_motion_video_capture *wrapped_video_capture);

bool brx_motion_media_pipe_video_detector::init(uint32_t hand_count, uint32_t face_count, uint32_t pose_count, bool force_gpu, brx_motion_video_capture const *video_capture)
{
    assert(0U == this->m_ref_count);
    this->m_ref_count = 1U;

    if (force_gpu && (internal_tflite_check_gpu_support()))
    {
        internal_tflite_set_env_force_gpu(true);
        this->m_enable_gpu = true;
    }
    else
    {
        internal_tflite_set_env_force_gpu(false);
        this->m_enable_gpu = false;
    }

    bool has_error = false;

    assert(0U == this->m_pose_count);
    this->m_pose_count = std::min(std::max(0U, pose_count), INTERNAL_MEIDA_PIPE_MAX_POSE_COUNT);

    if ((!has_error) && (this->m_pose_count > 0U))
    {
        {
            PoseLandmarkerOptions options;
            options.base_options.model_asset_buffer = reinterpret_cast<char const *>(brx_motion_mediapipe_model_asset_get_pose_landmarker_task_base());
            options.base_options.model_asset_buffer_count = static_cast<unsigned int>(brx_motion_mediapipe_model_asset_get_pose_landmarker_task_size());
            options.base_options.model_asset_path = NULL;
            options.running_mode = VIDEO;
            options.num_poses = static_cast<int>(this->m_pose_count);
            options.output_segmentation_masks = false;
            options.result_callback = NULL;

            assert(NULL == this->m_pose_landmarker);
            this->m_pose_landmarker = pose_landmarker_create(&options, NULL);
        }

        if (NULL != this->m_pose_landmarker)
        {
            assert(this->m_poses_skeleton_joint_translations_model_space_valid.empty());
            this->m_poses_skeleton_joint_translations_model_space_valid.resize(this->m_pose_count);
            for (uint32_t pose_index = 0U; pose_index < this->m_pose_count; ++pose_index)
            {
                std::array<bool, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_COUNT> &pose_skeleton_joint_translations_model_space_valid = this->m_poses_skeleton_joint_translations_model_space_valid[pose_index];
                for (uint32_t pose_skeleton_joint_name_index = 0U; pose_skeleton_joint_name_index < INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_COUNT; ++pose_skeleton_joint_name_index)
                {
                    pose_skeleton_joint_translations_model_space_valid[pose_skeleton_joint_name_index] = false;
                }
            }

            assert(this->m_poses_skeleton_joint_translations_model_space.empty());
            this->m_poses_skeleton_joint_translations_model_space.resize(this->m_pose_count);
        }
        else
        {
            assert(!has_error);
            has_error = true;
        }
    }

    assert(0U == this->m_face_count);
    this->m_face_count = std::min(std::max(0U, face_count), INTERNAL_MEIDA_PIPE_MAX_FACE_COUNT);

    if ((!has_error) && (this->m_face_count > 0U))
    {
        {
            FaceLandmarkerOptions options;
            options.base_options.model_asset_buffer = reinterpret_cast<char const *>(brx_motion_mediapipe_model_asset_get_face_landmarker_task_base());
            options.base_options.model_asset_buffer_count = static_cast<unsigned int>(brx_motion_mediapipe_model_asset_get_face_landmarker_task_size());
            options.base_options.model_asset_path = NULL;
            options.running_mode = VIDEO;
            options.num_faces = static_cast<int>(this->m_face_count);
            options.output_face_blendshapes = true;
            options.output_facial_transformation_matrixes = true;
            options.result_callback = NULL;

            assert(NULL == this->m_face_landmarker);
            this->m_face_landmarker = face_landmarker_create(&options, NULL);
        }

        if (NULL != this->m_face_landmarker)
        {
            assert(this->m_faces_morph_target_weights.empty());
            this->m_faces_morph_target_weights.resize(this->m_face_count);
            for (uint32_t face_index = 0U; face_index < this->m_face_count; ++face_index)
            {
                std::array<float, BRX_MOTION_MORPH_TARGET_NAME_MMD_COUNT> &face_morph_target_weights = this->m_faces_morph_target_weights[face_index];
                for (uint32_t face_morph_target_name_index = 0U; face_morph_target_name_index < BRX_MOTION_MORPH_TARGET_NAME_MMD_COUNT; ++face_morph_target_name_index)
                {
                    face_morph_target_weights[face_morph_target_name_index] = 0.0F;
                }
            }

            assert(this->m_faces_morph_joint_weights.empty());
            this->m_faces_morph_joint_weights.resize(this->m_face_count);
            for (uint32_t face_index = 0U; face_index < this->m_face_count; ++face_index)
            {
                std::array<DirectX::XMFLOAT4, INTERNAL_VIDEO_DETECTOR_FACE_MORPH_JOINT_NAME_COUNT> &face_morph_joint_weights = this->m_faces_morph_joint_weights[face_index];
                for (uint32_t face_morph_joint_name_index = 0U; face_morph_joint_name_index < INTERNAL_VIDEO_DETECTOR_FACE_MORPH_JOINT_NAME_COUNT; ++face_morph_joint_name_index)
                {
                    DirectX::XMStoreFloat4(&face_morph_joint_weights[face_morph_joint_name_index], DirectX::XMVectorZero());
                }
            }

            assert(this->m_faces_skeleton_joint_rotations.empty());
            this->m_faces_skeleton_joint_rotations.resize(this->m_face_count);
            for (uint32_t face_index = 0U; face_index < this->m_face_count; ++face_index)
            {
                std::array<DirectX::XMFLOAT4, INTERNAL_VIDEO_DETECTOR_FACE_SKELETON_JOINT_NAME_COUNT> &face_skeleton_joint_rotations = this->m_faces_skeleton_joint_rotations[face_index];
                for (uint32_t face_skeleton_joint_name_index = 0U; face_skeleton_joint_name_index < INTERNAL_VIDEO_DETECTOR_FACE_SKELETON_JOINT_NAME_COUNT; ++face_skeleton_joint_name_index)
                {
                    DirectX::XMStoreFloat4(&face_skeleton_joint_rotations[face_skeleton_joint_name_index], DirectX::XMQuaternionIdentity());
                }
            }
        }
        else
        {
            assert(!has_error);
            has_error = true;
        }
    }

    assert(0U == this->m_hand_count);
    this->m_hand_count = std::min(std::max(0U, hand_count), INTERNAL_MEIDA_PIPE_MAX_HAND_COUNT);

    if ((!has_error) && (this->m_hand_count > 0U))
    {
        {
            HandLandmarkerOptions options;
            options.base_options.model_asset_buffer = reinterpret_cast<char const *>(brx_motion_mediapipe_model_asset_get_hand_landmarker_task_base());
            options.base_options.model_asset_buffer_count = static_cast<unsigned int>(brx_motion_mediapipe_model_asset_get_hand_landmarker_task_size());
            options.base_options.model_asset_path = NULL;
            options.running_mode = VIDEO;
            options.num_hands = 2U * static_cast<int>(this->m_hand_count);
            options.result_callback = NULL;

            assert(NULL == this->m_hand_landmarker);
            this->m_hand_landmarker = hand_landmarker_create(&options, NULL);
        }

        if (NULL != this->m_hand_landmarker)
        {
            assert(this->m_hands_skeleton_joint_translations_model_space_valid.empty());
            this->m_hands_skeleton_joint_translations_model_space_valid.resize(this->m_hand_count);
            for (uint32_t hand_index = 0U; hand_index < this->m_hand_count; ++hand_index)
            {
                std::array<bool, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_COUNT> &hand_skeleton_joint_translations_model_space_valid = this->m_hands_skeleton_joint_translations_model_space_valid[hand_index];
                for (uint32_t hand_skeleton_joint_name_index = 0U; hand_skeleton_joint_name_index < INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_COUNT; ++hand_skeleton_joint_name_index)
                {
                    hand_skeleton_joint_translations_model_space_valid[hand_skeleton_joint_name_index] = false;
                }
            }

            assert(this->m_hands_skeleton_joint_translations_model_space.empty());
            this->m_hands_skeleton_joint_translations_model_space.resize(this->m_hand_count);
        }
        else
        {
            assert(!has_error);
            has_error = true;
        }
    }

    internal_tflite_unset_env_force_gpu();

    if (!has_error)
    {
        assert(NULL == this->m_input_video_capture);
        internal_retain_video_capture(const_cast<brx_motion_video_capture *>(video_capture));
        this->m_input_video_capture = video_capture;

        return true;
    }
    else
    {
        if (NULL != this->m_hand_landmarker)
        {
            assert(!this->m_hands_skeleton_joint_translations_model_space_valid.empty());
            assert(this->m_hands_skeleton_joint_translations_model_space_valid.size() == this->m_hand_count);
            this->m_hands_skeleton_joint_translations_model_space_valid.clear();

            assert(!this->m_hands_skeleton_joint_translations_model_space.empty());
            assert(this->m_hands_skeleton_joint_translations_model_space.size() == this->m_hand_count);
            this->m_hands_skeleton_joint_translations_model_space.clear();

            int status_hand_landmarker_close = hand_landmarker_close(this->m_hand_landmarker, NULL);
            assert(0 == status_hand_landmarker_close);
            this->m_hand_landmarker = NULL;

            assert(this->m_hand_count > 0U);
            this->m_hand_count = 0U;
        }
        else
        {
            assert(this->m_hands_skeleton_joint_translations_model_space_valid.empty());
            assert(this->m_hands_skeleton_joint_translations_model_space.empty());
            assert(NULL == this->m_hand_landmarker);

            this->m_hand_count = 0U;
        }

        if (NULL != this->m_face_landmarker)
        {
            assert(!this->m_faces_morph_target_weights.empty());
            assert(this->m_faces_morph_target_weights.size() == this->m_face_count);
            this->m_faces_morph_target_weights.clear();

            assert(!this->m_faces_morph_joint_weights.empty());
            assert(this->m_faces_morph_joint_weights.size() == this->m_face_count);
            this->m_faces_morph_joint_weights.clear();

            assert(!this->m_faces_skeleton_joint_rotations.empty());
            assert(this->m_faces_skeleton_joint_rotations.size() == this->m_face_count);
            this->m_faces_skeleton_joint_rotations.clear();

            int status_face_landmarker_close = face_landmarker_close(this->m_face_landmarker, NULL);
            assert(0 == status_face_landmarker_close);
            this->m_face_landmarker = NULL;

            assert(this->m_face_count > 0U);
            this->m_face_count = 0U;
        }
        else
        {
            assert(this->m_faces_morph_target_weights.empty());
            assert(this->m_faces_morph_joint_weights.empty());
            assert(this->m_faces_skeleton_joint_rotations.empty());
            assert(NULL == this->m_face_landmarker);

            this->m_face_count = 0U;
        }

        if (NULL != this->m_pose_landmarker)
        {
            assert(!this->m_poses_skeleton_joint_translations_model_space_valid.empty());
            assert(this->m_poses_skeleton_joint_translations_model_space_valid.size() == this->m_pose_count);
            this->m_poses_skeleton_joint_translations_model_space_valid.clear();

            assert(!this->m_poses_skeleton_joint_translations_model_space.empty());
            assert(this->m_poses_skeleton_joint_translations_model_space.size() == this->m_pose_count);
            this->m_poses_skeleton_joint_translations_model_space.clear();

            int status_pose_landmarker_close = pose_landmarker_close(this->m_pose_landmarker, NULL);
            assert(0 == status_pose_landmarker_close);
            this->m_pose_landmarker = NULL;

            assert(this->m_pose_count > 0U);
            this->m_pose_count = 0U;
        }
        else
        {
            assert(this->m_poses_skeleton_joint_translations_model_space_valid.empty());
            assert(this->m_poses_skeleton_joint_translations_model_space.empty());
            assert(NULL == this->m_pose_landmarker);

            this->m_pose_count = 0U;
        }

        assert(1U == this->m_ref_count);
        this->m_ref_count = 0U;

        return false;
    }
}

void brx_motion_media_pipe_video_detector::uninit()
{
    assert(0U == this->m_ref_count);

    assert(NULL != this->m_input_video_capture);
    internal_release_video_capture(const_cast<brx_motion_video_capture *>(this->m_input_video_capture));
    this->m_input_video_capture = NULL;

    if (this->m_enable_debug_renderer)
    {
        assert(NULL != this->m_debug_renderer_window);
        brx_wsi_destroy_image_window(this->m_debug_renderer_window);
        this->m_debug_renderer_window = NULL;

        this->m_enable_debug_renderer = false;
    }
    else
    {
        assert(!this->m_enable_debug_renderer);
    }

    if (this->m_hand_count > 0U)
    {
        assert(!this->m_hands_skeleton_joint_translations_model_space.empty());
        assert(this->m_hands_skeleton_joint_translations_model_space.size() == this->m_hand_count);
        this->m_hands_skeleton_joint_translations_model_space.clear();

        assert(!this->m_hands_skeleton_joint_translations_model_space_valid.empty());
        assert(this->m_hands_skeleton_joint_translations_model_space_valid.size() == this->m_hand_count);
        this->m_hands_skeleton_joint_translations_model_space_valid.clear();

        assert(NULL != this->m_hand_landmarker);
        int status_hand_landmarker_close = hand_landmarker_close(this->m_hand_landmarker, NULL);
        assert(0 == status_hand_landmarker_close);
        this->m_hand_landmarker = NULL;

        this->m_hand_count = 0U;
    }
    else
    {
        assert(this->m_hands_skeleton_joint_translations_model_space.empty());
        assert(this->m_hands_skeleton_joint_translations_model_space_valid.empty());

        assert(NULL == this->m_hand_landmarker);
    }

    if (this->m_face_count > 0U)
    {
        assert(!this->m_faces_morph_target_weights.empty());
        assert(this->m_faces_morph_target_weights.size() == this->m_face_count);
        this->m_faces_morph_target_weights.clear();

        assert(!this->m_faces_morph_joint_weights.empty());
        assert(this->m_faces_morph_joint_weights.size() == this->m_face_count);
        this->m_faces_morph_joint_weights.clear();

        assert(!this->m_faces_skeleton_joint_rotations.empty());
        assert(this->m_faces_skeleton_joint_rotations.size() == this->m_face_count);
        this->m_faces_skeleton_joint_rotations.clear();

        assert(NULL != this->m_face_landmarker);
        int status_face_landmarker_close = face_landmarker_close(this->m_face_landmarker, NULL);
        assert(0 == status_face_landmarker_close);
        this->m_face_landmarker = NULL;

        this->m_face_count = 0U;
    }
    else
    {
        assert(this->m_faces_morph_target_weights.empty());
        assert(this->m_faces_morph_joint_weights.empty());
        assert(this->m_faces_skeleton_joint_rotations.empty());
        assert(NULL == this->m_face_landmarker);
    }

    if (this->m_pose_count > 0U)
    {
        assert(!this->m_poses_skeleton_joint_translations_model_space.empty());
        assert(this->m_poses_skeleton_joint_translations_model_space.size() == this->m_pose_count);
        this->m_poses_skeleton_joint_translations_model_space.clear();

        assert(!this->m_poses_skeleton_joint_translations_model_space_valid.empty());
        assert(this->m_poses_skeleton_joint_translations_model_space_valid.size() == this->m_pose_count);
        this->m_poses_skeleton_joint_translations_model_space_valid.clear();

        assert(NULL != this->m_pose_landmarker);
        int status_pose_landmarker_close = pose_landmarker_close(this->m_pose_landmarker, NULL);
        assert(0 == status_pose_landmarker_close);
        this->m_pose_landmarker = NULL;

        this->m_pose_count = 0U;
    }
    else
    {
        assert(this->m_poses_skeleton_joint_translations_model_space.empty());
        assert(this->m_poses_skeleton_joint_translations_model_space_valid.empty());

        assert(NULL == this->m_pose_landmarker);
    }
}

inline void brx_motion_media_pipe_video_detector::internal_retain()
{
    assert(this->m_ref_count > 0U);
    assert(this->m_ref_count < static_cast<uint32_t>(UINT32_MAX));
    ++this->m_ref_count;
}

inline uint32_t brx_motion_media_pipe_video_detector::internal_release()
{
    assert(this->m_ref_count > 0U);
    --this->m_ref_count;
    return this->m_ref_count;
}

uint32_t brx_motion_media_pipe_video_detector::get_hand_count() const
{
    return this->m_hand_count;
}

uint32_t brx_motion_media_pipe_video_detector::get_face_count() const
{
    return this->m_face_count;
}

uint32_t brx_motion_media_pipe_video_detector::get_pose_count() const
{
    return this->m_pose_count;
}

bool brx_motion_media_pipe_video_detector::get_enable_gpu() const
{
    return this->m_enable_gpu;
}

brx_motion_video_capture *brx_motion_media_pipe_video_detector::get_input() const
{
    return const_cast<brx_motion_video_capture *>(this->m_input_video_capture);
}

void brx_motion_media_pipe_video_detector::set_enable_debug_renderer(bool enable_debug_renderer, char const *in_debug_renderer_window_name)
{
    if (enable_debug_renderer != this->m_enable_debug_renderer)
    {
        if (enable_debug_renderer)
        {
            mcrt_string debug_renderer_window_name;
            debug_renderer_window_name = " Brioche Motion Video Detector [";
            debug_renderer_window_name += in_debug_renderer_window_name;
            debug_renderer_window_name += "]";

            assert(NULL == this->m_debug_renderer_window);
            this->m_debug_renderer_window = brx_wsi_create_image_window(debug_renderer_window_name.c_str());
        }
        else
        {
            assert(NULL != this->m_debug_renderer_window);
            brx_wsi_destroy_image_window(this->m_debug_renderer_window);
            this->m_debug_renderer_window = NULL;
        }

        this->m_enable_debug_renderer = enable_debug_renderer;
    }
}

bool brx_motion_media_pipe_video_detector::get_enable_debug_renderer() const
{
    return this->m_enable_debug_renderer;
}

void brx_motion_media_pipe_video_detector::step()
{
    this->m_delta_time = static_cast<brx_motion_opencv_video_capture const *>(this->m_input_video_capture)->get_delta_time();

    this->m_timestamp_ms += static_cast<int64_t>((static_cast<double>(this->m_delta_time) * 1000.0));

    cv::Mat const *const video_frame = static_cast<brx_motion_opencv_video_capture const *>(this->m_input_video_capture)->get_video_frame();

    if (!video_frame->empty())
    {
        PoseLandmarkerResult pose_landmarker_result = {};
        int status_pose_landmarker_detect_for_video = -1;
        FaceLandmarkerResult face_landmarker_result = {};
        int status_face_landmarker_detect_for_video = -1;
        HandLandmarkerResult hand_landmarker_result = {};
        int status_hand_landmarker_detect_for_video = -1;
        int input_image_width = -1;
        int input_image_height = -1;
        cv::Mat debug_renderer_output_image;
        {
            cv::Mat input_image;
            {
                cv::Mat input_rgb_image;

                cv::cvtColor((*video_frame), input_rgb_image, cv::COLOR_BGR2RGB);

                if ((0U != (static_cast<uint32_t>(input_rgb_image.cols) & (16U - 1U))) || (0U != (static_cast<uint32_t>(input_rgb_image.rows) & (16U - 1U))))
                {
                    int const width = static_cast<int>(static_cast<uint32_t>(input_rgb_image.cols) & (~(16U - 1U)));
                    int const height = static_cast<int>(static_cast<uint32_t>(input_rgb_image.rows) & (~(16U - 1U)));
                    assert(width <= input_rgb_image.cols);
                    assert(height <= input_rgb_image.rows);

                    if (0 == width || 0 == height)
                    {
                        assert(false);
                        return;
                    }

                    cv::resize(input_rgb_image, input_image, cv::Size(static_cast<int>(width), static_cast<int>(height)), 0.0, 0.0, cv::INTER_AREA);
                }
                else
                {
                    input_image = std::move(input_rgb_image);
                }
            }

            MpImage mediapipe_input_image;
            {
                // mediapipe/examples/desktopdemo_run_graph_main.cc
                // mediapipe/framework/formats/image_frame_opencv.h
                // mediapipe/framework/formats/image_frame_opencv.cc

                constexpr ImageFormat const k_format = SRGB;
                constexpr int const k_number_of_channels_for_format = 3;
                constexpr int const k_channel_size_for_format = sizeof(uint8_t);
                constexpr int const k_mat_type_for_format = CV_8U;

                constexpr uint32_t const k_default_alignment_boundary = 16U;

                mediapipe_input_image.type = MpImage::IMAGE_FRAME;
                mediapipe_input_image.image_frame.format = k_format;
                mediapipe_input_image.image_frame.width = input_image.cols;
                mediapipe_input_image.image_frame.height = input_image.rows;

                int const type = CV_MAKETYPE(k_mat_type_for_format, k_number_of_channels_for_format);
                int const width_step = (((mediapipe_input_image.image_frame.width * k_number_of_channels_for_format * k_channel_size_for_format) - 1) | (k_default_alignment_boundary - 1)) + 1;
                assert(type == input_image.type());
                assert(width_step == input_image.step[0]);
                mediapipe_input_image.image_frame.image_buffer = static_cast<uint8_t *>(input_image.data);
                assert(0U == (reinterpret_cast<uintptr_t>(mediapipe_input_image.image_frame.image_buffer) & (k_default_alignment_boundary - 1)));
                assert(input_image.isContinuous());
            }

            if (this->m_pose_count > 0U)
            {
                assert(NULL != this->m_pose_landmarker);
                status_pose_landmarker_detect_for_video = pose_landmarker_detect_for_video(this->m_pose_landmarker, &mediapipe_input_image, this->m_timestamp_ms, &pose_landmarker_result, NULL);
            }
            else
            {
                assert(NULL == this->m_pose_landmarker);
            }

            if (this->m_face_count > 0U)
            {
                assert(NULL != this->m_face_landmarker);
                status_face_landmarker_detect_for_video = face_landmarker_detect_for_video(this->m_face_landmarker, &mediapipe_input_image, this->m_timestamp_ms, &face_landmarker_result, NULL);
            }
            else
            {
                assert(NULL == this->m_face_landmarker);
            }

            if (this->m_hand_count > 0U)
            {
                assert(NULL != this->m_hand_landmarker);
                status_hand_landmarker_detect_for_video = hand_landmarker_detect_for_video(this->m_hand_landmarker, &mediapipe_input_image, this->m_timestamp_ms, &hand_landmarker_result, NULL);
            }
            else
            {
                assert(NULL == this->m_hand_landmarker);
            }

            input_image_width = input_image.cols;
            input_image_height = input_image.rows;

            if (this->m_enable_debug_renderer)
            {
                // we do NOT need the input image any more
                debug_renderer_output_image = std::move(input_image);
            }
        }

        {
            // Pose Landmarker

            struct wrist_2d_pose_landmark_t
            {
                float m_x;
                float m_y;
                bool m_right;
            };

            mcrt_vector<wrist_2d_pose_landmark_t> wrist_2d_pose_landmarks;

            for (uint32_t pose_index = 0U; pose_index < this->m_pose_count; ++pose_index)
            {
                std::array<bool, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_COUNT> &pose_skeleton_joint_translations_model_space_valid = this->m_poses_skeleton_joint_translations_model_space_valid[pose_index];
                for (uint32_t pose_skeleton_joint_name_index = 0U; pose_skeleton_joint_name_index < INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_COUNT; ++pose_skeleton_joint_name_index)
                {
                    pose_skeleton_joint_translations_model_space_valid[pose_skeleton_joint_name_index] = false;
                }
            }

            if ((this->m_pose_count > 0U) && (0 == status_pose_landmarker_detect_for_video))
            {
                assert(pose_landmarker_result.pose_world_landmarks_count <= this->m_pose_count);

                for (uint32_t pose_index = 0U; pose_index < this->m_pose_count && pose_index < pose_landmarker_result.pose_world_landmarks_count; ++pose_index)
                {
                    Landmarks const &pose_world_landmark = pose_landmarker_result.pose_world_landmarks[pose_index];

                    DirectX::XMFLOAT3 media_pipe_positions[INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_COUNT];
                    // initialize "false"
                    bool media_pipe_positions_valid[INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_COUNT] = {};
#ifndef NDEBUG
                    for (uint32_t pose_position_name_index = 0U; pose_position_name_index < INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_COUNT; ++pose_position_name_index)
                    {
                        assert(!media_pipe_positions_valid[pose_position_name_index]);
                    }
#endif

                    assert(INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_COUNT == pose_world_landmark.landmarks_count);

                    for (uint32_t world_landmark_index = 0U; world_landmark_index < pose_world_landmark.landmarks_count; ++world_landmark_index)
                    {
                        Landmark const &landmark = pose_world_landmark.landmarks[world_landmark_index];

                        if (((!landmark.has_visibility) || (landmark.visibility > 0.5F)) && ((!landmark.has_presence) || (landmark.presence > 0.5F)))
                        {
                            media_pipe_positions[world_landmark_index] = DirectX::XMFLOAT3(landmark.x, landmark.y, landmark.z);
                            media_pipe_positions_valid[world_landmark_index] = true;
                        }
                    }

                    assert(pose_index < this->m_poses_skeleton_joint_translations_model_space.size());
                    assert(this->m_pose_count == this->m_poses_skeleton_joint_translations_model_space.size());
                    assert(pose_index < this->m_poses_skeleton_joint_translations_model_space_valid.size());
                    assert(this->m_pose_count == this->m_poses_skeleton_joint_translations_model_space_valid.size());
                    std::array<DirectX::XMFLOAT3, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_COUNT> &pose_skeleton_joint_translations_model_space = this->m_poses_skeleton_joint_translations_model_space[pose_index];
                    std::array<bool, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_COUNT> &pose_skeleton_joint_translations_model_space_valid = this->m_poses_skeleton_joint_translations_model_space_valid[pose_index];

                    // MediaPipe Pose
                    // RH
                    // Right -X
                    // Up -Y
                    // Forward -Z

                    // glTF
                    // RH
                    // Right -X
                    // Up +Y
                    // Forward +Z

                    constexpr DirectX::XMFLOAT4X4 const media_pipe_to_gltf(
                        1.0F, 0.0F, 0.0F, 0.0F,
                        0.0F, -1.0F, 0.0F, 0.0F,
                        0.0F, 0.0F, -1.0F, 0.0F,
                        0.0F, 0.0F, 0.0F, 1.0F);

                    DirectX::XMMATRIX simd_media_pipe_to_gltf = DirectX::XMLoadFloat4x4(&media_pipe_to_gltf);

                    constexpr uint32_t const internal_media_pipe_mappings[][2] = {
                        {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_SHOULDER, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_RIGHT_ARM},
                        {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_ELBOW, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_RIGHT_ELBOW},
                        {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_WRIST, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_RIGHT_WRIST},
                        {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_HIP, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_RIGHT_LEG},
                        {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_KNEE, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_RIGHT_KNEE},
                        {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_ANKLE, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_RIGHT_ANKLE},
                        {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_FOOT, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_RIGHT_TOE_TIP},
                        {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_SHOULDER, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LEFT_ARM},
                        {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_ELBOW, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LEFT_ELBOW},
                        {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_WRIST, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LEFT_WRIST},
                        {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_HIP, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LEFT_LEG},
                        {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_KNEE, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LEFT_KNEE},
                        {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_ANKLE, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LEFT_ANKLE},
                        {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_FOOT, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LEFT_TOE_TIP}};

                    constexpr uint32_t const mapping_count = sizeof(internal_media_pipe_mappings) / sizeof(internal_media_pipe_mappings[0]);

                    static_assert(INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_COUNT == (mapping_count + 2), "");

                    for (uint32_t mapping_index = 0U; mapping_index < mapping_count; ++mapping_index)
                    {
                        uint32_t const position_index = internal_media_pipe_mappings[mapping_index][0];
                        uint32_t const skeleton_joint_index = internal_media_pipe_mappings[mapping_index][1];

                        DirectX::XMStoreFloat3(&pose_skeleton_joint_translations_model_space[skeleton_joint_index], DirectX::XMVector3TransformCoord(DirectX::XMLoadFloat3(&media_pipe_positions[position_index]), simd_media_pipe_to_gltf));
                        pose_skeleton_joint_translations_model_space_valid[skeleton_joint_index] = media_pipe_positions_valid[position_index];
                    }

                    if (media_pipe_positions_valid[INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_HIP] && media_pipe_positions_valid[INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_HIP])
                    {
                        DirectX::XMStoreFloat3(&pose_skeleton_joint_translations_model_space[INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LOWER_BODY], DirectX::XMVectorScale(DirectX::XMVectorAdd(DirectX::XMVector3TransformCoord(DirectX::XMLoadFloat3(&media_pipe_positions[INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_HIP]), simd_media_pipe_to_gltf), DirectX::XMVector3TransformCoord(DirectX::XMLoadFloat3(&media_pipe_positions[INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_HIP]), simd_media_pipe_to_gltf)), 0.5F));

                        pose_skeleton_joint_translations_model_space_valid[INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LOWER_BODY] = true;
                    }
                    else
                    {
                        pose_skeleton_joint_translations_model_space_valid[INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LOWER_BODY] = false;
                    }

                    if (pose_skeleton_joint_translations_model_space_valid[INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LOWER_BODY] && pose_skeleton_joint_translations_model_space_valid[INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_RIGHT_ARM] && pose_skeleton_joint_translations_model_space_valid[INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LEFT_ARM])
                    {
                        float const golden_ratio = std::sqrt(5.0F) * 0.5F - 0.5F;

                        DirectX::XMStoreFloat3(&pose_skeleton_joint_translations_model_space[INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_UPPER_BODY_2], DirectX::XMVectorLerp(DirectX::XMVectorScale(DirectX::XMVectorAdd(DirectX::XMLoadFloat3(&pose_skeleton_joint_translations_model_space[INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_RIGHT_ARM]), DirectX::XMLoadFloat3(&pose_skeleton_joint_translations_model_space[INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LEFT_ARM])), 0.5F), DirectX::XMLoadFloat3(&pose_skeleton_joint_translations_model_space[INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LOWER_BODY]), golden_ratio));

                        pose_skeleton_joint_translations_model_space_valid[INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_UPPER_BODY_2] = true;
                    }
                    else
                    {
                        pose_skeleton_joint_translations_model_space_valid[INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_UPPER_BODY_2] = false;
                    }

                    // wrist 2D
                    {
                        NormalizedLandmarks const &pose_normalized_landmark = pose_landmarker_result.pose_landmarks[pose_index];

                        if (INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_WRIST < pose_normalized_landmark.landmarks_count)
                        {
                            NormalizedLandmark const &right_wrist_normalized_landmark = pose_normalized_landmark.landmarks[INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_WRIST];

                            if (((!right_wrist_normalized_landmark.has_visibility) || (right_wrist_normalized_landmark.visibility > 0.5F)) && ((!right_wrist_normalized_landmark.has_presence) || (right_wrist_normalized_landmark.presence > 0.5F)))
                            {
                                wrist_2d_pose_landmarks.push_back(wrist_2d_pose_landmark_t{right_wrist_normalized_landmark.x * static_cast<float>(input_image_width), right_wrist_normalized_landmark.y * static_cast<float>(input_image_height), true});
                            }
                        }

                        if (INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_WRIST < pose_normalized_landmark.landmarks_count)
                        {
                            NormalizedLandmark const &left_wrist_normalized_landmark = pose_normalized_landmark.landmarks[INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_WRIST];

                            if (((!left_wrist_normalized_landmark.has_visibility) || (left_wrist_normalized_landmark.visibility > 0.5F)) && ((!left_wrist_normalized_landmark.has_presence) || (left_wrist_normalized_landmark.presence > 0.5F)))
                            {
                                wrist_2d_pose_landmarks.push_back(wrist_2d_pose_landmark_t{left_wrist_normalized_landmark.x * static_cast<float>(input_image_width), left_wrist_normalized_landmark.y * static_cast<float>(input_image_height), false});
                            }
                        }
                    }
                }

                if (this->m_enable_debug_renderer)
                {
                    cv::Scalar const debug_renderer_pose_color(255, 0, 0);

                    for (uint32_t pose_index = 0U; pose_index < this->m_pose_count && pose_index < pose_landmarker_result.pose_landmarks_count; ++pose_index)
                    {
                        NormalizedLandmarks const &pose_normalized_landmark = pose_landmarker_result.pose_landmarks[pose_index];

                        assert(INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_COUNT == pose_normalized_landmark.landmarks_count);

                        constexpr uint32_t const internal_media_pipe_bones[][2] = {
                            //
                            {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_SHOULDER, INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_SHOULDER},
                            //
                            {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_SHOULDER, INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_ELBOW},
                            {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_ELBOW, INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_WRIST},
                            {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_WRIST, INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_THUMB},
                            {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_WRIST, INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_INDEX},
                            {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_WRIST, INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_PINKY},
                            //
                            {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_SHOULDER, INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_ELBOW},
                            {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_ELBOW, INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_WRIST},
                            {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_WRIST, INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_THUMB},
                            {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_WRIST, INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_INDEX},
                            {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_WRIST, INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_PINKY},
                            //
                            {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_HIP, INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_HIP},
                            //
                            {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_HIP, INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_KNEE},
                            {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_KNEE, INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_ANKLE},
                            {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_ANKLE, INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_RIGHT_FOOT},
                            //
                            {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_HIP, INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_KNEE},
                            {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_KNEE, INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_ANKLE},
                            {INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_ANKLE, INTERNAL_MEIDA_PIPE_POSE_POSITION_NAME_LEFT_FOOT}};

                        constexpr uint32_t const bone_count = sizeof(internal_media_pipe_bones) / sizeof(internal_media_pipe_bones[0]);

                        for (uint32_t bone_index = 0U; bone_index < bone_count; ++bone_index)
                        {
                            uint32_t const begin_joint_landmark_index = internal_media_pipe_bones[bone_index][0];
                            uint32_t const end_joint_landmark_index = internal_media_pipe_bones[bone_index][1];

                            if ((begin_joint_landmark_index < pose_normalized_landmark.landmarks_count) && (end_joint_landmark_index < pose_normalized_landmark.landmarks_count))
                            {
                                NormalizedLandmark const &begin_normalized_landmark = pose_normalized_landmark.landmarks[begin_joint_landmark_index];

                                NormalizedLandmark const &end_normalized_landmark = pose_normalized_landmark.landmarks[end_joint_landmark_index];

                                if (((!begin_normalized_landmark.has_visibility) || (begin_normalized_landmark.visibility > 0.5F)) && ((!begin_normalized_landmark.has_presence) || (begin_normalized_landmark.presence > 0.5F)) && ((!end_normalized_landmark.has_visibility) || (end_normalized_landmark.visibility > 0.5F)) && ((!end_normalized_landmark.has_presence) || (end_normalized_landmark.presence > 0.5F)))
                                {
                                    cv::Point const parent_point(static_cast<int>(begin_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(begin_normalized_landmark.y * debug_renderer_output_image.rows));

                                    cv::Point const end_point(static_cast<int>(end_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(end_normalized_landmark.y * debug_renderer_output_image.rows));

                                    cv::line(debug_renderer_output_image, parent_point, end_point, debug_renderer_pose_color);
                                }
                            }
                        }
                    }
                }
            }
            else
            {
                assert(0 == this->m_pose_count);
            }

            assert(wrist_2d_pose_landmarks.size() <= (2U * this->m_pose_count));

            // Face Landmarker

            if ((this->m_face_count > 0U) && (0 == status_face_landmarker_detect_for_video))
            {
                assert(face_landmarker_result.face_blendshapes_count <= this->m_face_count);

                for (uint32_t face_index = 0U; face_index < face_landmarker_result.face_blendshapes_count && face_index < this->m_face_count; ++face_index)
                {
                    Categories const &face_blendshape = face_landmarker_result.face_blendshapes[face_index];

                    static_assert(INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_COUNT == (sizeof(internal_media_pipe_face_weight_name_strings) / sizeof(internal_media_pipe_face_weight_name_strings[0])), "");

                    // initialize "0.0F"
                    float media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_COUNT] = {};

                    assert(INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_COUNT == face_blendshape.categories_count);

                    for (uint32_t blend_shape_index = 0U; blend_shape_index < face_blendshape.categories_count; ++blend_shape_index)
                    {
                        Category const &category = face_blendshape.categories[blend_shape_index];

                        assert(0 == std::strcmp(category.category_name, internal_media_pipe_face_weight_name_strings[blend_shape_index]));

                        media_pipe_weights[blend_shape_index] = category.score;
                    }

                    assert(face_index < this->m_faces_morph_target_weights.size());
                    assert(this->m_face_count == this->m_faces_morph_target_weights.size());
                    std::array<float, BRX_MOTION_MORPH_TARGET_NAME_MMD_COUNT> &face_morph_target_weights = this->m_faces_morph_target_weights[face_index];

                    for (uint32_t morph_target_name_index = 0U; morph_target_name_index < BRX_MOTION_MORPH_TARGET_NAME_MMD_COUNT; ++morph_target_name_index)
                    {
                        face_morph_target_weights[morph_target_name_index] = 0.0F;
                    }

                    // TODO: optimize the mapping from medie pipe to mmd

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_ANGRY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_BROW_DOWN_LEFT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_ANGRY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_BROW_DOWN_LEFT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_ANGRY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_BROW_DOWN_LEFT] * 0.5F);

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_ANGRY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_BROW_DOWN_RIGHT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_ANGRY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_BROW_DOWN_RIGHT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_ANGRY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_BROW_DOWN_RIGHT] * 0.5F);

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_CHEEK_SQUINT_LEFT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_CHEEK_SQUINT_LEFT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_CHEEK_SQUINT_LEFT] * 0.5F);

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_CHEEK_SQUINT_RIGHT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_CHEEK_SQUINT_RIGHT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_CHEEK_SQUINT_RIGHT] * 0.5F);

                    // TODO: optimize the remapping
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_BLINK_L] += std::max(0.0F, media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_BLINK_LEFT] - 0.125F) * (1.0F / (1.0F - 0.125F));
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_BLINK_R] += std::max(0.0F, media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_BLINK_RIGHT] - 0.125F) * (1.0F / (1.0F - 0.125F));

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_SQUINT_LEFT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_SQUINT_LEFT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_SQUINT_LEFT] * 0.5F);

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_SQUINT_RIGHT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_SQUINT_RIGHT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_SQUINT_RIGHT] * 0.5F);

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_SURPRISED] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_WIDE_LEFT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_SURPRISED] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_WIDE_LEFT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_SURPRISED] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_WIDE_LEFT] * 0.5F);

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_SURPRISED] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_WIDE_RIGHT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_SURPRISED] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_WIDE_RIGHT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_SURPRISED] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_WIDE_RIGHT] * 0.5F);

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_A] += media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_JAW_OPEN];

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_O] += media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_FUNNEL];

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_U] += media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_PUCKER];

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_SMILE_LEFT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_SMILE_LEFT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_SMILE_LEFT] * 0.5F);

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_SMILE_RIGHT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_SMILE_RIGHT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_SMILE_RIGHT] * 0.5F);

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_I] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_STRETCH_LEFT] * 0.5F);

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_I] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_STRETCH_RIGHT] * 0.5F);

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_E] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_UPPER_UP_LEFT] * 0.5F);

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_E] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_UPPER_UP_RIGHT] * 0.5F);

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_ANGRY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_NOSE_SNEER_LEFT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_ANGRY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_NOSE_SNEER_LEFT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_ANGRY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_NOSE_SNEER_LEFT] * 0.5F);

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_ANGRY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_NOSE_SNEER_RIGHT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_ANGRY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_NOSE_SNEER_RIGHT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_ANGRY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_MOUTH_NOSE_SNEER_RIGHT] * 0.5F);

                    for (uint32_t morph_target_name_index = 0U; morph_target_name_index < BRX_MOTION_MORPH_TARGET_NAME_MMD_COUNT; ++morph_target_name_index)
                    {
                        assert(face_morph_target_weights[morph_target_name_index] >= 0.0F);
                        face_morph_target_weights[morph_target_name_index] = std::min(face_morph_target_weights[morph_target_name_index], 1.0F);
                    }

                    assert(face_index < this->m_faces_morph_joint_weights.size());
                    assert(this->m_face_count == this->m_faces_morph_joint_weights.size());
                    std::array<DirectX::XMFLOAT4, INTERNAL_VIDEO_DETECTOR_FACE_MORPH_JOINT_NAME_COUNT> &face_morph_joint_weights = this->m_faces_morph_joint_weights[face_index];

                    face_morph_joint_weights[INTERNAL_VIDEO_DETECTOR_FACE_MORPH_JOINT_NAME_RIGHT_EYE] = DirectX::XMFLOAT4(
                        // up
                        media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_LOOK_UP_RIGHT],
                        // down
                        media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_LOOK_DOWN_RIGHT],
                        // left
                        media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_LOOK_IN_RIGHT],
                        // right
                        media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_LOOK_OUT_RIGHT]);

                    face_morph_joint_weights[INTERNAL_VIDEO_DETECTOR_FACE_MORPH_JOINT_NAME_LEFT_EYE] = DirectX::XMFLOAT4(
                        // up
                        media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_LOOK_UP_LEFT],
                        // down
                        media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_LOOK_DOWN_LEFT],
                        // left
                        media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_LOOK_OUT_LEFT],
                        // right
                        media_pipe_weights[INTERNAL_MEDIA_PIPE_FACE_WEIGHT_NAME_EYE_LOOK_IN_LEFT]);
                }

                assert(face_landmarker_result.facial_transformation_matrixes_count <= this->m_face_count);

                for (uint32_t face_index = 0U; face_index < face_landmarker_result.facial_transformation_matrixes_count && face_index < this->m_face_count; ++face_index)
                {
                    Matrix const &facial_transformation_matrix = face_landmarker_result.facial_transformation_matrixes[face_index];

                    assert(face_index < this->m_faces_skeleton_joint_rotations.size());
                    assert(this->m_face_count == this->m_faces_skeleton_joint_rotations.size());
                    std::array<DirectX::XMFLOAT4, INTERNAL_VIDEO_DETECTOR_FACE_SKELETON_JOINT_NAME_COUNT> &face_skeleton_joint_rotations = this->m_faces_skeleton_joint_rotations[face_index];

                    DirectX::XMFLOAT4X4 media_pipe_head_transform;
                    {
                        DirectX::XMStoreFloat4x4(&media_pipe_head_transform, DirectX::XMMatrixIdentity());

                        uint32_t const row_count = facial_transformation_matrix.cols;
                        uint32_t const column_count = facial_transformation_matrix.rows;

                        assert(4U == row_count);
                        assert(4U == column_count);

                        for (uint32_t row_index = 0U; row_index < row_count; ++row_index)
                        {
                            for (uint32_t column_index = 0U; column_index < column_count; ++column_index)
                            {
                                media_pipe_head_transform.m[row_index][column_index] = facial_transformation_matrix.data[column_count * row_index + column_index];
                            }
                        }
                    }

                    DirectX::XMFLOAT4 gltf_head_rotation;
                    {
                        // MediaPipe Face
                        // RH
                        // Right -X
                        // Up +Y
                        // Forward +Z

                        // glTF
                        // RH
                        // Right -X
                        // Up +Y
                        // Forward +Z

                        constexpr DirectX::XMFLOAT4X4 const gltf_to_media_pipe(
                            1.0F, 0.0F, 0.0F, 0.0F,
                            0.0F, 1.0F, 0.0F, 0.0F,
                            0.0F, 0.0F, 1.0F, 0.0F,
                            0.0F, 0.0F, 0.0F, 1.0F);

                        constexpr DirectX::XMFLOAT4X4 const media_pipe_to_gltf(
                            1.0F, 0.0F, 0.0F, 0.0F,
                            0.0F, 1.0F, 0.0F, 0.0F,
                            0.0F, 0.0F, 1.0F, 0.0F,
                            0.0F, 0.0F, 0.0F, 1.0F);

                        DirectX::XMMATRIX gltf_head_transform = DirectX::XMMatrixMultiply(DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&gltf_to_media_pipe), DirectX::XMLoadFloat4x4(&media_pipe_head_transform)), DirectX::XMLoadFloat4x4(&media_pipe_to_gltf));

                        constexpr float const INTERNAL_SCALE_EPSILON = 1E-4F;

                        DirectX::XMVECTOR simd_gltf_head_scale;
                        DirectX::XMVECTOR simd_gltf_head_rotation;
                        DirectX::XMVECTOR simd_gltf_head_translation;
                        bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&simd_gltf_head_scale, &simd_gltf_head_rotation, &simd_gltf_head_translation, gltf_head_transform);
                        assert(directx_xm_matrix_decompose);

                        assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(simd_gltf_head_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));

                        DirectX::XMStoreFloat4(&gltf_head_rotation, simd_gltf_head_rotation);
                    }

                    face_skeleton_joint_rotations[INTERNAL_VIDEO_DETECTOR_FACE_SKELETON_JOINT_NAME_HEAD] = gltf_head_rotation;
                }

                if (this->m_enable_debug_renderer)
                {
                    for (uint32_t face_index = 0U; face_index < this->m_face_count && face_index < face_landmarker_result.face_landmarks_count; ++face_index)
                    {
                        NormalizedLandmarks const &face_landmark = face_landmarker_result.face_landmarks[face_index];

                        for (uint32_t landmarks_index = 0U; landmarks_index < face_landmark.landmarks_count; ++landmarks_index)
                        {
                            NormalizedLandmark const &normalized_landmark = face_landmark.landmarks[landmarks_index];

                            if (((!normalized_landmark.has_visibility) || (normalized_landmark.visibility > 0.5F)) && ((!normalized_landmark.has_presence) || (normalized_landmark.presence > 0.5F)))
                            {
                                cv::Point point(static_cast<int>(normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(normalized_landmark.y * debug_renderer_output_image.rows));
                                cv::circle(debug_renderer_output_image, point, 1, cv::Scalar(0, 255, 0));
                            }
                        }
                    }
                }
            }
            else
            {
                assert(0 == this->m_face_count);
            }

            // Hand Landmarker

            for (uint32_t hand_index = 0U; hand_index < this->m_hand_count; ++hand_index)
            {
                std::array<bool, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_COUNT> &hand_skeleton_joint_translations_model_space_valid = this->m_hands_skeleton_joint_translations_model_space_valid[hand_index];
                for (uint32_t hand_skeleton_joint_name_index = 0U; hand_skeleton_joint_name_index < INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_COUNT; ++hand_skeleton_joint_name_index)
                {
                    hand_skeleton_joint_translations_model_space_valid[hand_skeleton_joint_name_index] = false;
                }
            }

            if ((this->m_hand_count > 0U) && (0 == status_hand_landmarker_detect_for_video))
            {
                {
                    assert(hand_landmarker_result.hand_landmarks_count <= (2U * this->m_hand_count));
                    assert(hand_landmarker_result.hand_landmarks_count == hand_landmarker_result.hand_world_landmarks_count);
                    assert(hand_landmarker_result.handedness_count <= (2U * this->m_hand_count));
                    assert(hand_landmarker_result.handedness_count == hand_landmarker_result.hand_world_landmarks_count);

                    mcrt_vector<bool> handedness_right_marks;
                    {
                        struct wrist_2d_hand_landmark_t
                        {
                            float m_x;
                            float m_y;
                            uint32_t m_both_hand_index;
                        };

                        mcrt_vector<wrist_2d_hand_landmark_t> wrist_2d_hand_landmarks;

                        for (uint32_t both_hand_index = 0U; both_hand_index < hand_landmarker_result.hand_landmarks_count; ++both_hand_index)
                        {
                            NormalizedLandmarks const &hand_normalized_landmark = hand_landmarker_result.hand_landmarks[both_hand_index];

                            if (INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_WRIST < hand_normalized_landmark.landmarks_count)
                            {
                                NormalizedLandmark const &wrist_normalized_landmark = hand_normalized_landmark.landmarks[INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_WRIST];

                                if (((!wrist_normalized_landmark.has_visibility) || (wrist_normalized_landmark.visibility > 0.5F)) && ((!wrist_normalized_landmark.has_presence) || (wrist_normalized_landmark.presence > 0.5F)))
                                {
                                    wrist_2d_hand_landmarks.push_back(wrist_2d_hand_landmark_t{wrist_normalized_landmark.x * static_cast<float>(input_image_width), wrist_normalized_landmark.y * static_cast<float>(input_image_height), both_hand_index});
                                }
                            }
                        }

                        if (wrist_2d_hand_landmarks.size() <= wrist_2d_pose_landmarks.size())
                        {
                            // Greedy

                            uint32_t const wrist_2d_hand_landmark_count = static_cast<uint32_t>(wrist_2d_hand_landmarks.size());

                            uint32_t const wrist_2d_pose_landmark_count = static_cast<uint32_t>(wrist_2d_pose_landmarks.size());

                            struct wrist_2d_edge_t
                            {
                                float m_cost;
                                uint32_t m_hand_landmark_index;
                                uint32_t m_pose_landmark_index;
                            };

                            mcrt_vector<wrist_2d_edge_t> wrist_2d_edges(static_cast<size_t>(static_cast<size_t>(wrist_2d_pose_landmark_count) * static_cast<size_t>(wrist_2d_hand_landmark_count)));

                            for (uint32_t wrist_2d_hand_landmark_index = 0U; wrist_2d_hand_landmark_index < wrist_2d_hand_landmark_count; ++wrist_2d_hand_landmark_index)
                            {
                                for (uint32_t wrist_2d_pose_landmark_index = 0U; wrist_2d_pose_landmark_index < wrist_2d_pose_landmark_count; ++wrist_2d_pose_landmark_index)
                                {
                                    float length_square;
                                    {
                                        DirectX::XMFLOAT2 const wrist_2d_hand_landmark_xy(wrist_2d_hand_landmarks[wrist_2d_hand_landmark_index].m_x, wrist_2d_hand_landmarks[wrist_2d_hand_landmark_index].m_y);
                                        DirectX::XMFLOAT2 const wrist_2d_pose_landmark_xy(wrist_2d_pose_landmarks[wrist_2d_pose_landmark_index].m_x, wrist_2d_pose_landmarks[wrist_2d_pose_landmark_index].m_y);
                                        length_square = DirectX::XMVectorGetX(DirectX::XMVector2LengthSq(DirectX::XMVectorSubtract(DirectX::XMLoadFloat2(&wrist_2d_hand_landmark_xy), DirectX::XMLoadFloat2(&wrist_2d_pose_landmark_xy))));
                                    }
                                    wrist_2d_edges[wrist_2d_pose_landmark_count * wrist_2d_hand_landmark_index + wrist_2d_pose_landmark_index] = wrist_2d_edge_t{length_square, wrist_2d_hand_landmark_index, wrist_2d_pose_landmark_index};
                                };
                            }

                            std::stable_sort(
                                wrist_2d_edges.begin(),
                                wrist_2d_edges.end(),
                                [](wrist_2d_edge_t const &x, wrist_2d_edge_t const &y) -> bool
                                {
                                    return x.m_cost < y.m_cost;
                                });

                            mcrt_vector<uint32_t> wrist_2d_hand_to_pose(static_cast<size_t>(wrist_2d_hand_landmark_count), BRX_MOTION_UINT32_INDEX_INVALID);
                            mcrt_vector<uint32_t> wrist_2d_pose_to_hand(static_cast<size_t>(wrist_2d_pose_landmark_count), BRX_MOTION_UINT32_INDEX_INVALID);
                            {
                                uint32_t wrist_2d_matched_hand_count = 0U;

                                for (wrist_2d_edge_t const &wrist_2d_edge : wrist_2d_edges)
                                {
                                    if ((BRX_MOTION_UINT32_INDEX_INVALID == wrist_2d_hand_to_pose[wrist_2d_edge.m_hand_landmark_index]) && (BRX_MOTION_UINT32_INDEX_INVALID == wrist_2d_pose_to_hand[wrist_2d_edge.m_pose_landmark_index]))
                                    {
                                        wrist_2d_hand_to_pose[wrist_2d_edge.m_hand_landmark_index] = wrist_2d_edge.m_pose_landmark_index;
                                        wrist_2d_pose_to_hand[wrist_2d_edge.m_pose_landmark_index] = wrist_2d_edge.m_hand_landmark_index;

                                        ++wrist_2d_matched_hand_count;

                                        if (wrist_2d_matched_hand_count >= wrist_2d_hand_landmark_count)
                                        {
                                            break;
                                        }
                                    }
                                }
#ifndef NDEBUG
                                for (uint32_t pose_landmark_index : wrist_2d_hand_to_pose)
                                {
                                    assert(BRX_MOTION_UINT32_INDEX_INVALID != pose_landmark_index);
                                }
#endif
                            }

                            assert(wrist_2d_hand_landmark_count <= hand_landmarker_result.hand_landmarks_count);

                            assert(handedness_right_marks.empty());
                            handedness_right_marks.resize(hand_landmarker_result.hand_landmarks_count, false);

                            for (uint32_t wrist_2d_hand_landmark_index = 0U; wrist_2d_hand_landmark_index < wrist_2d_hand_landmark_count; ++wrist_2d_hand_landmark_index)
                            {
                                uint32_t const both_hand_index = wrist_2d_hand_landmarks[wrist_2d_hand_landmark_index].m_both_hand_index;

                                uint32_t const wrist_2d_pose_landmark_index = wrist_2d_hand_to_pose[wrist_2d_hand_landmark_index];

                                bool const wrist_2d_right = wrist_2d_pose_landmarks[wrist_2d_pose_landmark_index].m_right;

                                handedness_right_marks[both_hand_index] = wrist_2d_right;
                            }
                        }
                        else
                        {
                            // assert(false);

                            mcrt_vector<std::pair<float, uint32_t>> handedness_right_scores_and_indices;
                            mcrt_vector<std::pair<float, uint32_t>> handedness_left_scores_and_indices;
                            for (uint32_t both_hand_index = 0U; both_hand_index < hand_landmarker_result.handedness_count; ++both_hand_index)
                            {
                                if (hand_landmarker_result.handedness[both_hand_index].categories_count > 0U)
                                {
                                    assert(1U == hand_landmarker_result.handedness[both_hand_index].categories_count);
                                    if (0 == std::strcmp("Right", hand_landmarker_result.handedness[both_hand_index].categories[0].category_name))
                                    {
                                        handedness_right_scores_and_indices.emplace_back(hand_landmarker_result.handedness[both_hand_index].categories[0].score, both_hand_index);
                                    }
                                    else if (0 == std::strcmp("Left", hand_landmarker_result.handedness[both_hand_index].categories[0].category_name))
                                    {
                                        handedness_left_scores_and_indices.emplace_back(hand_landmarker_result.handedness[both_hand_index].categories[0].score, both_hand_index);
                                    }
                                    else
                                    {
                                        assert(false);
                                    }
                                }
                                else
                                {
                                    assert(false);
                                }
                            }
                            assert(hand_landmarker_result.handedness_count == (handedness_right_scores_and_indices.size() + handedness_left_scores_and_indices.size()));

                            assert(handedness_right_marks.empty());
                            handedness_right_marks.resize(hand_landmarker_result.handedness_count, false);

                            if (handedness_right_scores_and_indices.size() > this->m_hand_count)
                            {
                                std::stable_sort(
                                    handedness_right_scores_and_indices.begin(),
                                    handedness_right_scores_and_indices.end(),
                                    [](std::pair<float, uint32_t> const &x, std::pair<float, uint32_t> const &y)
                                    {
                                        return x.first > y.first;
                                    });

                                for (uint32_t both_hand_index = 0U; both_hand_index < hand_landmarker_result.handedness_count; ++both_hand_index)
                                {
                                    handedness_right_marks[both_hand_index] = false;
                                }

                                for (uint32_t right_hand_index = 0U; right_hand_index < this->m_hand_count; ++right_hand_index)
                                {
                                    uint32_t right_both_hand_index = handedness_right_scores_and_indices[right_hand_index].second;
                                    assert(!handedness_right_marks[right_both_hand_index]);
                                    handedness_right_marks[right_both_hand_index] = true;
                                }
                            }
                            else if (handedness_left_scores_and_indices.size() > this->m_hand_count)
                            {
                                std::stable_sort(
                                    handedness_left_scores_and_indices.begin(),
                                    handedness_left_scores_and_indices.end(),
                                    [](std::pair<float, uint32_t> const &x, std::pair<float, uint32_t> const &y)
                                    {
                                        return x.first > y.first;
                                    });

                                for (uint32_t both_hand_index = 0U; both_hand_index < hand_landmarker_result.handedness_count; ++both_hand_index)
                                {
                                    handedness_right_marks[both_hand_index] = true;
                                }

                                for (uint32_t left_hand_index = 0U; left_hand_index < this->m_hand_count; ++left_hand_index)
                                {
                                    uint32_t left_both_hand_index = handedness_left_scores_and_indices[left_hand_index].second;
                                    assert(handedness_right_marks[left_both_hand_index]);
                                    handedness_right_marks[left_both_hand_index] = false;
                                }
                            }
                            else
                            {
                                assert(handedness_right_scores_and_indices.size() <= this->m_hand_count);
                                assert(handedness_left_scores_and_indices.size() <= this->m_hand_count);

                                for (std::pair<float, uint32_t> const &handedness_right_score_and_index : handedness_right_scores_and_indices)
                                {
                                    uint32_t right_both_hand_index = handedness_right_score_and_index.second;
                                    handedness_right_marks[right_both_hand_index] = true;
                                }

                                for (std::pair<float, uint32_t> const &handedness_left_score_and_index : handedness_left_scores_and_indices)
                                {
                                    uint32_t left_both_hand_index = handedness_left_score_and_index.second;
                                    handedness_right_marks[left_both_hand_index] = false;
                                }
                            }
                        }

                        assert(hand_landmarker_result.hand_world_landmarks_count <= (2U * this->m_hand_count));
                    }

                    uint32_t right_hand_index = 0U;
                    uint32_t left_hand_index = 0U;

                    for (uint32_t both_hand_index = 0U; both_hand_index < (2U * this->m_hand_count) && both_hand_index < handedness_right_marks.size() && both_hand_index < hand_landmarker_result.hand_world_landmarks_count; ++both_hand_index)
                    {
                        bool const handedness_right = handedness_right_marks[both_hand_index];

                        uint32_t hand_index;
                        if (handedness_right)
                        {
                            hand_index = right_hand_index;
                            ++right_hand_index;
                        }
                        else
                        {
                            hand_index = left_hand_index;
                            ++left_hand_index;
                        }

                        if (hand_index < this->m_hand_count)
                        {
                            Landmarks const &hand_world_landmark = hand_landmarker_result.hand_world_landmarks[both_hand_index];

                            DirectX::XMFLOAT3 media_pipe_positions[INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_COUNT];
                            // initialize "false"
                            bool media_pipe_positions_valid[INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_COUNT] = {};
#ifndef NDEBUG
                            for (uint32_t hand_position_name_index = 0U; hand_position_name_index < INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_COUNT; ++hand_position_name_index)
                            {
                                assert(!media_pipe_positions_valid[hand_position_name_index]);
                            }
#endif

                            assert(INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_COUNT == hand_world_landmark.landmarks_count);

                            for (uint32_t world_landmark_index = 0U; world_landmark_index < hand_world_landmark.landmarks_count; ++world_landmark_index)
                            {
                                Landmark const &landmark = hand_world_landmark.landmarks[world_landmark_index];

                                if (((!landmark.has_visibility) || (landmark.visibility > 0.5F)) && ((!landmark.has_presence) || (landmark.presence > 0.5F)))
                                {
                                    media_pipe_positions[world_landmark_index] = DirectX::XMFLOAT3(landmark.x, landmark.y, landmark.z);
                                    media_pipe_positions_valid[world_landmark_index] = true;
                                }
                            }

                            assert(hand_index < this->m_hands_skeleton_joint_translations_model_space.size());
                            assert(this->m_hand_count == this->m_hands_skeleton_joint_translations_model_space.size());
                            assert(hand_index < this->m_hands_skeleton_joint_translations_model_space_valid.size());
                            assert(this->m_hand_count == this->m_hands_skeleton_joint_translations_model_space_valid.size());
                            std::array<DirectX::XMFLOAT3, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_COUNT> &hand_skeleton_joint_translations_model_space = this->m_hands_skeleton_joint_translations_model_space[hand_index];
                            std::array<bool, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_COUNT> &hand_skeleton_joint_translations_model_space_valid = this->m_hands_skeleton_joint_translations_model_space_valid[hand_index];

                            // MediaPipe hand
                            // RH
                            // Right -X
                            // Up -Y
                            // Forward -Z

                            // glTF
                            // RH
                            // Right -X
                            // Up +Y
                            // Forward +Z

                            constexpr DirectX::XMFLOAT4X4 const media_pipe_to_gltf(
                                1.0F, 0.0F, 0.0F, 0.0F,
                                0.0F, -1.0F, 0.0F, 0.0F,
                                0.0F, 0.0F, -1.0F, 0.0F,
                                0.0F, 0.0F, 0.0F, 1.0F);

                            DirectX::XMMATRIX simd_media_pipe_to_gltf = DirectX::XMLoadFloat4x4(&media_pipe_to_gltf);

                            constexpr uint32_t const internal_media_pipe_right_mappings[][2] = {
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_WRIST, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_WRIST},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_THUMB_CMC, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_THUMB_CMC},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_THUMB_MCP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_THUMB_MCP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_THUMB_IP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_THUMB_IP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_THUMB_TIP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_THUMB_TIP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_INDEX_FINGER_MCP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_INDEX_FINGER_MCP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_INDEX_FINGER_PIP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_INDEX_FINGER_PIP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_INDEX_FINGER_DIP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_INDEX_FINGER_DIP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_INDEX_FINGER_TIP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_INDEX_FINGER_TIP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_MIDDLE_FINGER_MCP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_MIDDLE_FINGER_MCP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_MIDDLE_FINGER_PIP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_MIDDLE_FINGER_PIP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_MIDDLE_FINGER_DIP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_MIDDLE_FINGER_DIP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_MIDDLE_FINGER_TIP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_MIDDLE_FINGER_TIP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_RING_FINGER_MCP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_RING_FINGER_MCP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_RING_FINGER_PIP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_RING_FINGER_PIP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_RING_FINGER_DIP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_RING_FINGER_DIP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_RING_FINGER_TIP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_RING_FINGER_TIP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_PINKY_MCP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_PINKY_MCP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_PINKY_PIP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_PINKY_PIP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_PINKY_DIP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_PINKY_DIP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_PINKY_TIP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_PINKY_TIP}};

                            constexpr uint32_t const internal_media_pipe_left_mappings[][2] = {
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_WRIST, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_WRIST},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_THUMB_CMC, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_THUMB_CMC},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_THUMB_MCP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_THUMB_MCP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_THUMB_IP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_THUMB_IP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_THUMB_TIP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_THUMB_TIP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_INDEX_FINGER_MCP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_INDEX_FINGER_MCP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_INDEX_FINGER_PIP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_INDEX_FINGER_PIP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_INDEX_FINGER_DIP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_INDEX_FINGER_DIP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_INDEX_FINGER_TIP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_INDEX_FINGER_TIP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_MIDDLE_FINGER_MCP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_MIDDLE_FINGER_MCP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_MIDDLE_FINGER_PIP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_MIDDLE_FINGER_PIP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_MIDDLE_FINGER_DIP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_MIDDLE_FINGER_DIP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_MIDDLE_FINGER_TIP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_MIDDLE_FINGER_TIP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_RING_FINGER_MCP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_RING_FINGER_MCP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_RING_FINGER_PIP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_RING_FINGER_PIP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_RING_FINGER_DIP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_RING_FINGER_DIP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_RING_FINGER_TIP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_RING_FINGER_TIP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_PINKY_MCP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_PINKY_MCP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_PINKY_PIP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_PINKY_PIP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_PINKY_DIP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_PINKY_DIP},
                                {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_PINKY_TIP, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_PINKY_TIP}};

                            constexpr uint32_t const mapping_count = sizeof(internal_media_pipe_right_mappings) / sizeof(internal_media_pipe_right_mappings[0]);
                            static_assert((sizeof(internal_media_pipe_left_mappings) / sizeof(internal_media_pipe_left_mappings[0])) == mapping_count, "");

                            uint32_t const(*const internal_media_pipe_mappings)[2] = handedness_right ? internal_media_pipe_right_mappings : internal_media_pipe_left_mappings;

                            static_assert(INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_COUNT == (2U * mapping_count), "");

                            for (uint32_t mapping_index = 0U; mapping_index < mapping_count; ++mapping_index)
                            {
                                uint32_t const position_index = internal_media_pipe_mappings[mapping_index][0];
                                uint32_t const skeleton_joint_index = internal_media_pipe_mappings[mapping_index][1];

                                DirectX::XMStoreFloat3(&hand_skeleton_joint_translations_model_space[skeleton_joint_index], DirectX::XMVector3TransformCoord(DirectX::XMLoadFloat3(&media_pipe_positions[position_index]), simd_media_pipe_to_gltf));
                                hand_skeleton_joint_translations_model_space_valid[skeleton_joint_index] = media_pipe_positions_valid[position_index];
                            }
                        }
                        else
                        {
                            assert(false);
                        }
                    }

                    assert(hand_landmarker_result.hand_landmarks_count == (right_hand_index + left_hand_index));
                    assert(hand_landmarker_result.hand_world_landmarks_count == (right_hand_index + left_hand_index));
                    assert((right_hand_index + left_hand_index) <= (2U * this->m_hand_count));
                }

                if (this->m_enable_debug_renderer)
                {
                    cv::Scalar const debug_renderer_hand_color(0, 0, 255);

                    for (uint32_t both_hand_index = 0U; both_hand_index < (2U * this->m_hand_count) && both_hand_index < hand_landmarker_result.hand_landmarks_count; ++both_hand_index)
                    {
                        NormalizedLandmarks const &hand_normalized_landmark = hand_landmarker_result.hand_landmarks[both_hand_index];

                        assert(INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_COUNT == hand_normalized_landmark.landmarks_count);

                        constexpr uint32_t const internal_media_pipe_bones[][2] = {
                            //
                            {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_WRIST, INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_THUMB_CMC},
                            {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_THUMB_CMC, INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_THUMB_MCP},
                            {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_THUMB_MCP, INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_THUMB_IP},
                            {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_THUMB_IP, INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_THUMB_TIP},
                            //
                            {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_WRIST, INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_INDEX_FINGER_MCP},
                            {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_INDEX_FINGER_MCP, INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_INDEX_FINGER_PIP},
                            {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_INDEX_FINGER_PIP, INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_INDEX_FINGER_DIP},
                            {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_INDEX_FINGER_DIP, INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_INDEX_FINGER_TIP},
                            //
                            {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_WRIST, INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_MIDDLE_FINGER_MCP},
                            {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_MIDDLE_FINGER_MCP, INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_MIDDLE_FINGER_PIP},
                            {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_MIDDLE_FINGER_PIP, INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_MIDDLE_FINGER_DIP},
                            {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_MIDDLE_FINGER_DIP, INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_MIDDLE_FINGER_TIP},
                            //
                            {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_WRIST, INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_RING_FINGER_MCP},
                            {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_RING_FINGER_MCP, INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_RING_FINGER_PIP},
                            {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_RING_FINGER_PIP, INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_RING_FINGER_DIP},
                            {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_RING_FINGER_DIP, INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_RING_FINGER_TIP},
                            //
                            {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_WRIST, INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_PINKY_MCP},
                            {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_PINKY_MCP, INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_PINKY_PIP},
                            {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_PINKY_PIP, INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_PINKY_DIP},
                            {INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_PINKY_DIP, INTERNAL_MEIDA_PIPE_HAND_POSITION_NAME_PINKY_TIP}};

                        constexpr uint32_t const bone_count = sizeof(internal_media_pipe_bones) / sizeof(internal_media_pipe_bones[0]);

                        for (uint32_t bone_index = 0U; bone_index < bone_count; ++bone_index)
                        {
                            uint32_t const begin_joint_landmark_index = internal_media_pipe_bones[bone_index][0];
                            uint32_t const end_joint_landmark_index = internal_media_pipe_bones[bone_index][1];

                            if ((begin_joint_landmark_index < hand_normalized_landmark.landmarks_count) && (end_joint_landmark_index < hand_normalized_landmark.landmarks_count))
                            {
                                NormalizedLandmark const &begin_normalized_landmark = hand_normalized_landmark.landmarks[begin_joint_landmark_index];

                                NormalizedLandmark const &end_normalized_landmark = hand_normalized_landmark.landmarks[end_joint_landmark_index];

                                if (((!begin_normalized_landmark.has_visibility) || (begin_normalized_landmark.visibility > 0.5F)) && ((!begin_normalized_landmark.has_presence) || (begin_normalized_landmark.presence > 0.5F)) && ((!end_normalized_landmark.has_visibility) || (end_normalized_landmark.visibility > 0.5F)) && ((!end_normalized_landmark.has_presence) || (end_normalized_landmark.presence > 0.5F)))
                                {
                                    cv::Point const parent_point(static_cast<int>(begin_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(begin_normalized_landmark.y * debug_renderer_output_image.rows));

                                    cv::Point const end_point(static_cast<int>(end_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(end_normalized_landmark.y * debug_renderer_output_image.rows));

                                    cv::line(debug_renderer_output_image, parent_point, end_point, debug_renderer_hand_color);
                                }
                            }
                        }
                    }
                }
            }
            else
            {
                assert(0 == this->m_hand_count);
            }

            // Present

            if (this->m_enable_debug_renderer)
            {
                // Left <-> Right
                // cv::flip(debug_renderer_output_image, debug_renderer_output_image, 1);

                cv::Mat debug_renderer_raw_output_image;
                cv::cvtColor(debug_renderer_output_image, debug_renderer_raw_output_image, cv::COLOR_RGB2BGRA);
                debug_renderer_output_image.release();

                void *image_buffer;
                int32_t image_width;
                int32_t image_height;
                {
                    // mediapipe/examples/desktopdemo_run_graph_main.cc
                    // mediapipe/framework/formats/image_frame_opencv.h
                    // mediapipe/framework/formats/image_frame_opencv.cc

                    constexpr int const k_number_of_channels_for_format = 4;
                    constexpr int const k_channel_size_for_format = sizeof(uint8_t);
                    constexpr int const k_mat_type_for_format = CV_8U;

                    constexpr uint32_t const k_default_alignment_boundary = 16U;

                    image_width = debug_renderer_raw_output_image.cols;
                    image_height = debug_renderer_raw_output_image.rows;

                    int const type = CV_MAKETYPE(k_mat_type_for_format, k_number_of_channels_for_format);
                    int const width_step = (((image_width * k_number_of_channels_for_format * k_channel_size_for_format) - 1) | (k_default_alignment_boundary - 1)) + 1;
                    assert(type == debug_renderer_raw_output_image.type());
                    assert(width_step == debug_renderer_raw_output_image.step[0]);
                    image_buffer = static_cast<void *>(debug_renderer_raw_output_image.data);
                    assert(0U == (reinterpret_cast<uintptr_t>(image_buffer) & (k_default_alignment_boundary - 1)));
                    assert(debug_renderer_raw_output_image.isContinuous());
                }

                assert(NULL != this->m_debug_renderer_window);
                brx_wsi_present_image_window(this->m_debug_renderer_window, image_buffer, image_width, image_height);
            }
        }

        // should we still close the result when fail?
        if (this->m_pose_count > 0U)
        {
            pose_landmarker_close_result(&pose_landmarker_result);
        }

        if (this->m_face_count > 0U)
        {
            face_landmarker_close_result(&face_landmarker_result);
        }

        if (this->m_hand_count > 0U)
        {
            hand_landmarker_close_result(&hand_landmarker_result);
        }
    }
    else
    {
        // use the last successful result when fail
    }
}

double brx_motion_media_pipe_video_detector::get_delta_time() const
{
    return this->m_delta_time;
}

float brx_motion_media_pipe_video_detector::get_morph_target_weight(uint32_t face_index, BRX_MOTION_MORPH_TARGET_NAME morph_target_name) const
{
    if (face_index < this->m_face_count)
    {
        assert(face_index < this->m_faces_morph_target_weights.size());
        assert(this->m_face_count == this->m_faces_morph_target_weights.size());
        assert(morph_target_name < BRX_MOTION_MORPH_TARGET_NAME_MMD_COUNT);
        return this->m_faces_morph_target_weights[face_index][morph_target_name];
    }
    else
    {
        return 0.0F;
    }
}

DirectX::XMFLOAT4 const brx_motion_media_pipe_video_detector::get_face_morph_joint_weight(uint32_t face_index, BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name) const
{
    if (face_index < this->m_face_count)
    {
        assert(face_index < this->m_faces_morph_joint_weights.size());
        assert(this->m_face_count == this->m_faces_morph_joint_weights.size());
        std::array<DirectX::XMFLOAT4, INTERNAL_VIDEO_DETECTOR_FACE_MORPH_JOINT_NAME_COUNT> const &face_morph_joint_weights = this->m_faces_morph_joint_weights[face_index];

        assert((BRX_MOTION_SKELETON_JOINT_NAME_INVALID == skeleton_joint_name) || (skeleton_joint_name < BRX_MOTION_SKELETON_JOINT_NAME_MMD_COUNT));
        uint32_t const face_morph_joint_index = internal_get_face_morph_joint_index(skeleton_joint_name);

        if (BRX_MOTION_UINT32_INDEX_INVALID != face_morph_joint_index)
        {
            assert(face_morph_joint_index < INTERNAL_VIDEO_DETECTOR_FACE_MORPH_JOINT_NAME_COUNT);
            return face_morph_joint_weights[face_morph_joint_index];
        }
        else
        {
            return DirectX::XMFLOAT4(0.0, 0.0, 0.0, 0.0);
        }
    }
    else
    {
        return DirectX::XMFLOAT4(0.0, 0.0, 0.0, 0.0);
    }
}

DirectX::XMFLOAT3 const *brx_motion_media_pipe_video_detector::get_pose_skeleton_joint_translation(uint32_t pose_index, BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name) const
{
    if (pose_index < this->m_pose_count)
    {
        assert(pose_index < this->m_poses_skeleton_joint_translations_model_space.size());
        assert(this->m_pose_count == this->m_poses_skeleton_joint_translations_model_space.size());
        assert(pose_index < this->m_poses_skeleton_joint_translations_model_space_valid.size());
        assert(this->m_pose_count == this->m_poses_skeleton_joint_translations_model_space_valid.size());
        std::array<DirectX::XMFLOAT3, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_COUNT> const &pose_skeleton_joint_translations_model_space = this->m_poses_skeleton_joint_translations_model_space[pose_index];
        std::array<bool, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_COUNT> const &pose_skeleton_joint_translations_model_space_valid = this->m_poses_skeleton_joint_translations_model_space_valid[pose_index];

        assert((BRX_MOTION_SKELETON_JOINT_NAME_INVALID == skeleton_joint_name) || (skeleton_joint_name < BRX_MOTION_SKELETON_JOINT_NAME_MMD_COUNT));
        uint32_t const pose_skeleton_joint_index = internal_get_pose_skeleton_joint_index(skeleton_joint_name);

        if (BRX_MOTION_UINT32_INDEX_INVALID != pose_skeleton_joint_index)
        {
            assert(pose_skeleton_joint_index < INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_COUNT);
            if (pose_skeleton_joint_translations_model_space_valid[pose_skeleton_joint_index])
            {
                return &pose_skeleton_joint_translations_model_space[pose_skeleton_joint_index];
            }
            else
            {
                return NULL;
            }
        }
        else
        {
            return NULL;
        }
    }
    else
    {
        assert(false);
        return NULL;
    }
}

DirectX::XMFLOAT4 const *brx_motion_media_pipe_video_detector::get_face_skeleton_joint_rotation(uint32_t face_index, BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name) const
{
    if (face_index < this->m_face_count)
    {
        assert(face_index < this->m_faces_skeleton_joint_rotations.size());
        assert(this->m_face_count == this->m_faces_skeleton_joint_rotations.size());
        std::array<DirectX::XMFLOAT4, INTERNAL_VIDEO_DETECTOR_FACE_SKELETON_JOINT_NAME_COUNT> const &face_skeleton_joint_rotations = this->m_faces_skeleton_joint_rotations[face_index];

        assert((BRX_MOTION_SKELETON_JOINT_NAME_INVALID == skeleton_joint_name) || (skeleton_joint_name < BRX_MOTION_SKELETON_JOINT_NAME_MMD_COUNT));
        uint32_t const face_skeleton_joint_index = internal_get_face_skeleton_joint_index(skeleton_joint_name);

        if (BRX_MOTION_UINT32_INDEX_INVALID != face_skeleton_joint_index)
        {
            assert(face_skeleton_joint_index < INTERNAL_VIDEO_DETECTOR_FACE_SKELETON_JOINT_NAME_COUNT);
            return &face_skeleton_joint_rotations[face_skeleton_joint_index];
        }
        else
        {
            return NULL;
        }
    }
    else
    {
        assert(false);
        return NULL;
    }
}

DirectX::XMFLOAT3 const *brx_motion_media_pipe_video_detector::get_hand_skeleton_joint_translation(uint32_t hand_index, BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name) const
{
    if (hand_index < this->m_hand_count)
    {
        assert(hand_index < this->m_hands_skeleton_joint_translations_model_space.size());
        assert(this->m_hand_count == this->m_hands_skeleton_joint_translations_model_space.size());
        assert(hand_index < this->m_hands_skeleton_joint_translations_model_space_valid.size());
        assert(this->m_hand_count == this->m_hands_skeleton_joint_translations_model_space_valid.size());
        std::array<DirectX::XMFLOAT3, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_COUNT> const &hand_skeleton_joint_translations_model_space = this->m_hands_skeleton_joint_translations_model_space[hand_index];
        std::array<bool, INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_COUNT> const &hand_skeleton_joint_translations_model_space_valid = this->m_hands_skeleton_joint_translations_model_space_valid[hand_index];

        assert((BRX_MOTION_SKELETON_JOINT_NAME_INVALID == skeleton_joint_name) || (skeleton_joint_name < BRX_MOTION_SKELETON_JOINT_NAME_MMD_COUNT));
        uint32_t const hand_skeleton_joint_index = internal_get_hand_skeleton_joint_index(skeleton_joint_name);

        if (BRX_MOTION_UINT32_INDEX_INVALID != hand_skeleton_joint_index)
        {
            assert(hand_skeleton_joint_index < INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_COUNT);
            if (hand_skeleton_joint_translations_model_space_valid[hand_skeleton_joint_index])
            {
                return &hand_skeleton_joint_translations_model_space[hand_skeleton_joint_index];
            }
            else
            {
                return NULL;
            }
        }
        else
        {
            return NULL;
        }
    }
    else
    {
        assert(false);
        return NULL;
    }
}

void brx_motion_media_pipe_video_detector::retain()
{
    brx_motion_media_pipe_video_detector *const retain_unwrapped_video_detector = this;

    retain_unwrapped_video_detector->internal_retain();
}

void brx_motion_media_pipe_video_detector::release()
{
    brx_motion_media_pipe_video_detector *const release_unwrapped_video_detector = this;

    if (0U == release_unwrapped_video_detector->internal_release())
    {
        brx_motion_media_pipe_video_detector *delete_unwrapped_video_detector = release_unwrapped_video_detector;

        delete_unwrapped_video_detector->uninit();

        delete_unwrapped_video_detector->~brx_motion_media_pipe_video_detector();
        mcrt_free(delete_unwrapped_video_detector);
    }
}

static inline uint32_t internal_get_face_morph_joint_index(BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name)
{
    uint32_t face_skeleton_joint_index;
    switch (skeleton_joint_name)
    {
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_EYE:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_EYE == skeleton_joint_name);
        face_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_FACE_MORPH_JOINT_NAME_RIGHT_EYE;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_EYE:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_EYE == skeleton_joint_name);
        face_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_FACE_MORPH_JOINT_NAME_LEFT_EYE;
    }
    break;
    default:
    {
        assert(false);
        face_skeleton_joint_index = BRX_MOTION_UINT32_INDEX_INVALID;
    }
    }
    return face_skeleton_joint_index;
}

static inline uint32_t internal_get_pose_skeleton_joint_index(BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name)
{
    uint32_t pose_skeleton_joint_index;
    switch (skeleton_joint_name)
    {
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_UPPER_BODY_2:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_UPPER_BODY_2 == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_UPPER_BODY_2;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_ARM:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_ARM == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_RIGHT_ARM;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_ELBOW:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_ELBOW == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_RIGHT_ELBOW;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_WRIST:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_WRIST == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_RIGHT_WRIST;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_ARM:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_ARM == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LEFT_ARM;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_ELBOW:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_ELBOW == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LEFT_ELBOW;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_WRIST:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_WRIST == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LEFT_WRIST;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LOWER_BODY:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LOWER_BODY == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LOWER_BODY;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_LEG:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_LEG == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_RIGHT_LEG;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_KNEE:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_KNEE == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_RIGHT_KNEE;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_ANKLE:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_ANKLE == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_RIGHT_ANKLE;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_TOE_TIP:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_TOE_TIP == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_RIGHT_TOE_TIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_LEG:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_LEG == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LEFT_LEG;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_KNEE:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_KNEE == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LEFT_KNEE;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_ANKLE:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_ANKLE == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LEFT_ANKLE;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_TOE_TIP:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_TOE_TIP == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LEFT_TOE_TIP;
    }
    break;
    default:
    {
        assert(false);
        pose_skeleton_joint_index = BRX_MOTION_UINT32_INDEX_INVALID;
    }
    }
    return pose_skeleton_joint_index;
}

static inline uint32_t internal_get_face_skeleton_joint_index(BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name)
{
    uint32_t face_skeleton_joint_index;
    switch (skeleton_joint_name)
    {
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_HEAD:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_HEAD == skeleton_joint_name);
        face_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_FACE_SKELETON_JOINT_NAME_HEAD;
    }
    break;
    default:
    {
        assert(false);
        face_skeleton_joint_index = BRX_MOTION_UINT32_INDEX_INVALID;
    }
    }
    return face_skeleton_joint_index;
}

static inline uint32_t internal_get_hand_skeleton_joint_index(BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name)
{
    uint32_t hand_skeleton_joint_index;
    switch (skeleton_joint_name)
    {
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_WRIST:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_WRIST == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_WRIST;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_THUMB_0:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_THUMB_0 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_THUMB_CMC;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_THUMB_1:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_THUMB_1 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_THUMB_MCP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_THUMB_2:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_THUMB_2 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_THUMB_IP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_THUMB_TIP:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_THUMB_TIP == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_THUMB_TIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_INDEX_FINGER_1:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_INDEX_FINGER_1 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_INDEX_FINGER_MCP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_INDEX_FINGER_2:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_INDEX_FINGER_2 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_INDEX_FINGER_PIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_INDEX_FINGER_3:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_INDEX_FINGER_3 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_INDEX_FINGER_DIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_INDEX_FINGER_TIP:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_INDEX_FINGER_TIP == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_INDEX_FINGER_TIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_MIDDLE_FINGER_1:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_MIDDLE_FINGER_1 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_MIDDLE_FINGER_MCP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_MIDDLE_FINGER_2:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_MIDDLE_FINGER_2 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_MIDDLE_FINGER_PIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_MIDDLE_FINGER_3:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_MIDDLE_FINGER_3 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_MIDDLE_FINGER_DIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_MIDDLE_FINGER_TIP:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_MIDDLE_FINGER_TIP == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_MIDDLE_FINGER_TIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_RING_FINGER_1:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_RING_FINGER_1 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_RING_FINGER_MCP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_RING_FINGER_2:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_RING_FINGER_2 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_RING_FINGER_PIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_RING_FINGER_3:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_RING_FINGER_3 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_RING_FINGER_DIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_RING_FINGER_TIP:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_RING_FINGER_TIP == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_RING_FINGER_TIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_LITTLE_FINGER_1:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_LITTLE_FINGER_1 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_PINKY_MCP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_LITTLE_FINGER_2:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_LITTLE_FINGER_2 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_PINKY_PIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_LITTLE_FINGER_3:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_LITTLE_FINGER_3 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_PINKY_DIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_LITTLE_FINGER_TIP:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_LITTLE_FINGER_TIP == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_RIGHT_PINKY_TIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_WRIST:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_WRIST == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_WRIST;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_THUMB_0:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_THUMB_0 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_THUMB_CMC;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_THUMB_1:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_THUMB_1 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_THUMB_MCP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_THUMB_2:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_THUMB_2 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_THUMB_IP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_THUMB_TIP:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_THUMB_TIP == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_THUMB_TIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_INDEX_FINGER_1:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_INDEX_FINGER_1 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_INDEX_FINGER_MCP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_INDEX_FINGER_2:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_INDEX_FINGER_2 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_INDEX_FINGER_PIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_INDEX_FINGER_3:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_INDEX_FINGER_3 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_INDEX_FINGER_DIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_INDEX_FINGER_TIP:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_INDEX_FINGER_TIP == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_INDEX_FINGER_TIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_MIDDLE_FINGER_1:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_MIDDLE_FINGER_1 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_MIDDLE_FINGER_MCP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_MIDDLE_FINGER_2:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_MIDDLE_FINGER_2 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_MIDDLE_FINGER_PIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_MIDDLE_FINGER_3:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_MIDDLE_FINGER_3 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_MIDDLE_FINGER_DIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_MIDDLE_FINGER_TIP:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_MIDDLE_FINGER_TIP == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_MIDDLE_FINGER_TIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_RING_FINGER_1:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_RING_FINGER_1 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_RING_FINGER_MCP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_RING_FINGER_2:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_RING_FINGER_2 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_RING_FINGER_PIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_RING_FINGER_3:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_RING_FINGER_3 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_RING_FINGER_DIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_RING_FINGER_TIP:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_RING_FINGER_TIP == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_RING_FINGER_TIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_LITTLE_FINGER_1:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_LITTLE_FINGER_1 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_PINKY_MCP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_LITTLE_FINGER_2:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_LITTLE_FINGER_2 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_PINKY_PIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_LITTLE_FINGER_3:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_LITTLE_FINGER_3 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_PINKY_DIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_LITTLE_FINGER_TIP:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_LITTLE_FINGER_TIP == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_HAND_SKELETON_JOINT_NAME_LEFT_PINKY_TIP;
    }
    break;
    default:
    {
        assert(false);
        hand_skeleton_joint_index = BRX_MOTION_UINT32_INDEX_INVALID;
    }
    }
    return hand_skeleton_joint_index;
}
