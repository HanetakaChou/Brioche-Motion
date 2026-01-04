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
#include <mediapipe/tasks/c/vision/face_landmarker/face_landmarker.h>
#include <mediapipe/tasks/c/vision/pose_landmarker/pose_landmarker.h>
#include <cassert>
#include <cstring>
#include <algorithm>
#include <new>

#include "../models/face_landmarker_task.h"
#include "../models/pose_landmarker_task.h"

static constexpr uint32_t const INTERNAL_MEIDA_PIPE_MAX_FACE_COUNT = 2U;
static constexpr uint32_t const INTERNAL_MEIDA_PIPE_MAX_POSE_COUNT = 5U;

// https://ai.google.dev/edge/mediapipe/solutions/vision/face_landmarker
// https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker
// https://developer.apple.com/documentation/arkit/arfaceanchor/blendshapelocation
// https://developer.apple.com/documentation/arkit/arskeleton/jointname

enum
{
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_NEUTRAL = 0,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_BROW_DOWN_LEFT = 1,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_BROW_DOWN_RIGHT = 2,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_BROW_INNER_UP = 3,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_BROW_OUTER_UP_LEFT = 4,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_BROW_OUTER_UP_RIGHT = 5,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_CHEEK_PUFF = 6,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_CHEEK_SQUINT_LEFT = 7,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_CHEEK_SQUINT_RIGHT = 8,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_BLINK_LEFT = 9,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_BLINK_RIGHT = 10,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_DOWN_LEFT = 11,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_DOWN_RIGHT = 12,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_IN_LEFT = 13,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_IN_RIGHT = 14,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_OUT_LEFT = 15,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_OUT_RIGHT = 16,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_UP_LEFT = 17,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_UP_RIGHT = 18,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_SQUINT_LEFT = 19,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_SQUINT_RIGHT = 20,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_WIDE_LEFT = 21,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_WIDE_RIGHT = 22,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_JAW_FORWARD = 23,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_JAW_LEFT = 24,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_JAW_OPEN = 25,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_JAW_RIGHT = 26,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_CLOSE = 27,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_DIMPLE_LEFT = 28,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_DIMPLE_RIGHT = 29,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_FROWN_LEFT = 30,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_FROWN_RIGHT = 31,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_FUNNEL = 32,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_LEFT = 33,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_LOWER_DOWN_LEFT = 34,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_LOWER_DOWN_RIGHT = 35,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_PRESS_LEFT = 36,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_PRESS_RIGHT = 37,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_PUCKER = 38,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_RIGHT = 39,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_ROLL_LOWER = 40,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_ROLL_UPPER = 41,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_SHRUG_LOWER = 42,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_SHRUG_UPPER = 43,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_SMILE_LEFT = 44,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_SMILE_RIGHT = 45,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_STRETCH_LEFT = 46,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_STRETCH_RIGHT = 47,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_UPPER_UP_LEFT = 48,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_UPPER_UP_RIGHT = 49,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_NOSE_SNEER_LEFT = 50,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_NOSE_SNEER_RIGHT = 51,
    INTERNAL_MEDIA_PIPE_WEIGHT_NAME_COUNT = 52
};

static constexpr char const *const internal_media_pipe_weight_name_strings[] =
    {
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

//  nose
//      left_eye_inner
//          left_eye
//              left_eye_outer
//                  left_ear
//
//      right_eye_inner
//          right_eye
//              right_eye_outer
//                  right_ear
//
//  mouth_left
//
//  mouth_right
//
//
//  right_shoulder
//      right_elbow
//          right_wrist
//              right_thumb
//              right_index
//              right_pinky
//
//  right_hip
//      right_knee
//          right_ankle
//              right_foot
//              right_heel
//
//  left_shoulder
//      left_elbow
//          left_wrist
//              left_thumb
//              left_index
//              left_pinky
//
//  left_hip
//      left_knee
//          left_ankle
//              left_foot
//              left_heel
//

enum
{
    INTERNAL_MEIDA_PIPE_POSITION_NAME_NOSE = 0,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_EYE_INNER = 1,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_EYE = 2,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_EYE_OUTER = 3,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_EYE_INNER = 4,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_EYE = 5,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_EYE_OUTER = 6,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_EAR = 7,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_EAR = 8,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_MOUTH_LEFT = 9,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_MOUTH_RIGHT = 10,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_SHOULDER = 11,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_SHOULDER = 12,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_ELBOW = 13,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_ELBOW = 14,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_WRIST = 15,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_WRIST = 16,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_PINKY = 17,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_PINKY = 18,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_INDEX = 19,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_INDEX = 20,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_THUMB = 21,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_THUMB = 22,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_HIP = 23,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_HIP = 24,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_KNEE = 25,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_KNEE = 26,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_ANKLE = 27,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_ANKLE = 28,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_HEEL = 29,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_HEEL = 30,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_FOOT = 31,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_FOOT = 32,
    INTERNAL_MEIDA_PIPE_POSITION_NAME_COUNT = 33
};

static inline uint32_t internal_get_face_skeleton_joint_index(BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name);

static inline uint32_t internal_get_pose_skeleton_joint_index(BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name);

extern "C" brx_motion_video_detector *brx_motion_create_video_detector(uint32_t face_count, uint32_t pose_count, bool force_gpu, brx_motion_video_capture *video_capture)
{
    void *new_unwrapped_video_detector_base = mcrt_malloc(sizeof(brx_motion_media_pipe_video_detector), alignof(brx_motion_media_pipe_video_detector));
    assert(NULL != new_unwrapped_video_detector_base);

    brx_motion_media_pipe_video_detector *new_unwrapped_video_detector = new (new_unwrapped_video_detector_base) brx_motion_media_pipe_video_detector{};
    if (new_unwrapped_video_detector->init(face_count, pose_count, force_gpu, video_capture))
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

extern void internal_retain_video_detector(brx_motion_video_detector *wrapped_video_detector)
{
    assert(NULL != wrapped_video_detector);
    brx_motion_media_pipe_video_detector *const retain_unwrapped_video_detector = static_cast<brx_motion_media_pipe_video_detector *>(wrapped_video_detector);

    retain_unwrapped_video_detector->retain();
}

extern void internal_release_video_detector(brx_motion_video_detector *wrapped_video_detector)
{
    assert(NULL != wrapped_video_detector);
    brx_motion_media_pipe_video_detector *const release_unwrapped_video_detector = static_cast<brx_motion_media_pipe_video_detector *>(wrapped_video_detector);

    if (0U == release_unwrapped_video_detector->internal_release())
    {
        brx_motion_media_pipe_video_detector *delete_unwrapped_video_detector = release_unwrapped_video_detector;

        delete_unwrapped_video_detector->uninit();

        delete_unwrapped_video_detector->~brx_motion_media_pipe_video_detector();
        mcrt_free(delete_unwrapped_video_detector);
    }
}

extern "C" void brx_motion_destroy_video_detector(brx_motion_video_detector *wrapped_video_detector)
{
    internal_release_video_detector(wrapped_video_detector);
}

brx_motion_media_pipe_video_detector::brx_motion_media_pipe_video_detector() : m_ref_count(0U), m_face_count(0U), m_pose_count(0U), m_face_landmarker(NULL), m_pose_landmarker(NULL), m_timestamp_ms(0), m_enable_debug_renderer(false), m_debug_renderer_window(NULL), m_input_video_capture(NULL)
{
}

brx_motion_media_pipe_video_detector::~brx_motion_media_pipe_video_detector()
{
    assert(0U == this->m_ref_count);
    assert(0U == this->m_face_count);
    assert(0U == this->m_pose_count);
    assert(NULL == this->m_face_landmarker);
    assert(NULL == this->m_pose_landmarker);
    assert(this->m_faces_morph_target_weights.empty());
    assert(this->m_poses_skeleton_joint_translations_model_space.empty());
    assert(this->m_poses_skeleton_joint_translations_model_space_valid.empty());
    assert(this->m_debug_renderer_window_name.empty());
    assert(!this->m_enable_debug_renderer);
    assert(NULL == this->m_input_video_capture);
}

extern void internal_retain_video_capture(brx_motion_video_capture *wrapped_video_capture);

extern void internal_release_video_capture(brx_motion_video_capture *wrapped_video_capture);

bool brx_motion_media_pipe_video_detector::init(uint32_t face_count, uint32_t pose_count, bool force_gpu, brx_motion_video_capture const *video_capture)
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

    assert(0U == this->m_face_count);
    this->m_face_count = std::min(std::max(0U, face_count), INTERNAL_MEIDA_PIPE_MAX_FACE_COUNT);

    if (this->m_face_count > 0U)
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
        if (NULL == this->m_face_landmarker)
        {
            this->m_face_count = 0U;

            internal_tflite_unset_env_force_gpu();

            return false;
        }

        assert(this->m_faces_morph_target_weights.empty());
        // initialize "0.0F"
        this->m_faces_morph_target_weights.resize(this->m_face_count);

        assert(this->m_faces_skeleton_joint_rotations.empty());
        this->m_faces_skeleton_joint_rotations.resize(this->m_face_count);

        // initialize identity
        for (uint32_t face_index = 0U; face_index < this->m_face_count; ++face_index)
        {
            std::array<DirectX::XMFLOAT4, INTERNAL_VIDEO_DETECTOR_FACE_SKELETON_JOINT_NAME_COUNT> &face_skeleton_joint_rotations = this->m_faces_skeleton_joint_rotations[face_index];
            for (uint32_t face_skeleton_joint_name_index = 0U; face_skeleton_joint_name_index < INTERNAL_VIDEO_DETECTOR_FACE_SKELETON_JOINT_NAME_COUNT; ++face_skeleton_joint_name_index)
            {
                DirectX::XMStoreFloat4(&face_skeleton_joint_rotations[face_skeleton_joint_name_index], DirectX::XMQuaternionIdentity());
            }
        }
    }

    assert(0U == this->m_pose_count);
    this->m_pose_count = std::min(std::max(0U, pose_count), INTERNAL_MEIDA_PIPE_MAX_POSE_COUNT);

    if (this->m_pose_count > 0U)
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
        if (NULL == this->m_pose_landmarker)
        {
            this->m_pose_count = 0U;

            if (this->m_face_count > 0U)
            {
                assert(!this->m_faces_morph_target_weights.empty());
                assert(this->m_faces_morph_target_weights.size() == this->m_face_count);
                this->m_faces_morph_target_weights.clear();

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
                assert(this->m_faces_skeleton_joint_rotations.empty());
                assert(NULL == this->m_face_landmarker);
            }

            internal_tflite_unset_env_force_gpu();

            return false;
        }

        assert(this->m_poses_skeleton_joint_translations_model_space.empty());
        this->m_poses_skeleton_joint_translations_model_space.resize(this->m_pose_count);

        assert(this->m_poses_skeleton_joint_translations_model_space_valid.empty());
        // initialize "false"
        this->m_poses_skeleton_joint_translations_model_space_valid.resize(this->m_pose_count);
    }

    internal_tflite_unset_env_force_gpu();

    assert(NULL == this->m_input_video_capture);
    internal_retain_video_capture(const_cast<brx_motion_video_capture *>(video_capture));
    this->m_input_video_capture = video_capture;

    return true;
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

        assert(!this->m_debug_renderer_window_name.empty());
        this->m_debug_renderer_window_name.clear();
    }
    else
    {
        assert(!this->m_enable_debug_renderer);
        assert(this->m_debug_renderer_window_name.empty());
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

    if (this->m_face_count > 0U)
    {
        assert(!this->m_faces_morph_target_weights.empty());
        assert(this->m_faces_morph_target_weights.size() == this->m_face_count);
        this->m_faces_morph_target_weights.clear();

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
        assert(this->m_faces_skeleton_joint_rotations.empty());
        assert(NULL == this->m_face_landmarker);
    }
}

inline void brx_motion_media_pipe_video_detector::retain()
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

void brx_motion_media_pipe_video_detector::set_enable_debug_renderer(bool enable_debug_renderer, char const *debug_renderer_window_name)
{
    if (enable_debug_renderer != this->m_enable_debug_renderer)
    {
        if (enable_debug_renderer)
        {
            {
                char text_timestamp[] = {"18446744073709551615"};
                std::snprintf(text_timestamp, sizeof(text_timestamp) / sizeof(text_timestamp[0]), "%llu", static_cast<long long unsigned>(mcrt_tick_count_now()));
                text_timestamp[(sizeof(text_timestamp) / sizeof(text_timestamp[0])) - 1] = '\0';

                assert(this->m_debug_renderer_window_name.empty());
                this->m_debug_renderer_window_name = debug_renderer_window_name;
                this->m_debug_renderer_window_name += " Brioche Motion Tick Count [";
                this->m_debug_renderer_window_name += text_timestamp;
                this->m_debug_renderer_window_name += "]";
            }

            assert(NULL == this->m_debug_renderer_window);
            this->m_debug_renderer_window = brx_wsi_create_image_window(this->m_debug_renderer_window_name.c_str());
        }
        else
        {
            assert(NULL != this->m_debug_renderer_window);
            brx_wsi_destroy_image_window(this->m_debug_renderer_window);
            this->m_debug_renderer_window = NULL;

            {
                assert(!this->m_debug_renderer_window_name.empty());
                this->m_debug_renderer_window_name.clear();
            }
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
    float const delta_time = static_cast<brx_motion_opencv_video_capture const *>(this->m_input_video_capture)->get_delta_time();

    this->m_timestamp_ms += static_cast<int64_t>((static_cast<double>(delta_time) * 1000.0));

    cv::Mat const *const video_frame = static_cast<brx_motion_opencv_video_capture const *>(this->m_input_video_capture)->get_video_frame();

    if (!video_frame->empty())
    {
        FaceLandmarkerResult face_landmarker_result = {};
        int status_face_landmarker_detect_for_video = -1;
        PoseLandmarkerResult pose_landmarker_result = {};
        int status_pose_landmarker_detect_for_video = -1;
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

            if (this->m_face_count > 0U)
            {
                assert(NULL != this->m_face_landmarker);
                status_face_landmarker_detect_for_video = face_landmarker_detect_for_video(this->m_face_landmarker, &mediapipe_input_image, this->m_timestamp_ms, &face_landmarker_result, NULL);
            }
            else
            {
                assert(NULL == this->m_face_landmarker);
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

            if (this->m_enable_debug_renderer)
            {
                // we do NOT need the input image any more
                debug_renderer_output_image = std::move(input_image);
            }
        }

        {
            if ((this->m_face_count > 0U) && (0 == status_face_landmarker_detect_for_video))
            {
                assert(face_landmarker_result.face_blendshapes_count <= this->m_face_count);

                for (uint32_t face_index = 0U; face_index < face_landmarker_result.face_blendshapes_count && face_index < this->m_face_count; ++face_index)
                {
                    Categories const &face_blendshape = face_landmarker_result.face_blendshapes[face_index];

                    static_assert(INTERNAL_MEDIA_PIPE_WEIGHT_NAME_COUNT == (sizeof(internal_media_pipe_weight_name_strings) / sizeof(internal_media_pipe_weight_name_strings[0])), "");

                    // initialize "0.0F"
                    float media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_COUNT] = {};

                    assert(INTERNAL_MEDIA_PIPE_WEIGHT_NAME_COUNT == face_blendshape.categories_count);

                    for (uint32_t blend_shape_index = 0U; blend_shape_index < face_blendshape.categories_count; ++blend_shape_index)
                    {
                        Category const &category = face_blendshape.categories[blend_shape_index];

                        assert(0 == std::strcmp(category.category_name, internal_media_pipe_weight_name_strings[blend_shape_index]));

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

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_ANGRY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_BROW_DOWN_LEFT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_ANGRY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_BROW_DOWN_LEFT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_ANGRY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_BROW_DOWN_LEFT] * 0.5F);

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_ANGRY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_BROW_DOWN_RIGHT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_ANGRY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_BROW_DOWN_RIGHT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_ANGRY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_BROW_DOWN_RIGHT] * 0.5F);

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_CHEEK_SQUINT_LEFT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_CHEEK_SQUINT_LEFT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_CHEEK_SQUINT_LEFT] * 0.5F);

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_CHEEK_SQUINT_RIGHT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_CHEEK_SQUINT_RIGHT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_CHEEK_SQUINT_RIGHT] * 0.5F);

                    // TODO: optimize the remapping
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_BLINK_L] += std::max(0.0F, media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_BLINK_LEFT] - 0.125F) * (1.0F / (1.0F - 0.125F));
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_BLINK_R] += std::max(0.0F, media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_BLINK_RIGHT] - 0.125F) * (1.0F / (1.0F - 0.125F));

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_SQUINT_LEFT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_SQUINT_LEFT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_SQUINT_LEFT] * 0.5F);

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_SQUINT_RIGHT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_SQUINT_RIGHT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_SQUINT_RIGHT] * 0.5F);

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_SURPRISED] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_WIDE_LEFT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_SURPRISED] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_WIDE_LEFT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_SURPRISED] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_WIDE_LEFT] * 0.5F);

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_SURPRISED] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_WIDE_RIGHT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_SURPRISED] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_WIDE_RIGHT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_SURPRISED] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_WIDE_RIGHT] * 0.5F);

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_A] += media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_JAW_OPEN];

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_O] += media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_FUNNEL];

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_U] += media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_PUCKER];

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_SMILE_LEFT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_SMILE_LEFT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_SMILE_LEFT] * 0.5F);

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_SMILE_RIGHT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_SMILE_RIGHT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_HAPPY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_SMILE_RIGHT] * 0.5F);

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_I] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_STRETCH_LEFT] * 0.5F);

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_I] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_STRETCH_RIGHT] * 0.5F);

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_E] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_UPPER_UP_LEFT] * 0.5F);

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_E] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_UPPER_UP_RIGHT] * 0.5F);

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_ANGRY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_NOSE_SNEER_LEFT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_ANGRY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_NOSE_SNEER_LEFT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_ANGRY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_NOSE_SNEER_LEFT] * 0.5F);

                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_ANGRY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_NOSE_SNEER_RIGHT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_ANGRY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_NOSE_SNEER_RIGHT] * 0.5F);
                    face_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_ANGRY] += (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_MOUTH_NOSE_SNEER_RIGHT] * 0.5F);

                    for (uint32_t morph_target_name_index = 0U; morph_target_name_index < BRX_MOTION_MORPH_TARGET_NAME_MMD_COUNT; ++morph_target_name_index)
                    {
                        assert(face_morph_target_weights[morph_target_name_index] >= 0.0F);
                        face_morph_target_weights[morph_target_name_index] = std::min(face_morph_target_weights[morph_target_name_index], 1.0F);
                    }

                    assert(face_index < this->m_faces_skeleton_joint_rotations.size());
                    assert(this->m_face_count == this->m_faces_skeleton_joint_rotations.size());
                    std::array<DirectX::XMFLOAT4, INTERNAL_VIDEO_DETECTOR_FACE_SKELETON_JOINT_NAME_COUNT> &face_skeleton_joint_rotations = this->m_faces_skeleton_joint_rotations[face_index];

                    DirectX::XMFLOAT4 right_eye_rotation;
                    DirectX::XMFLOAT4 left_eye_rotation;
                    {
                        constexpr float const INTERNAL_EYE_ANGLE_MAXIMUM = 0.25F;
                        constexpr float const INTERNAL_EYE_WEIGHT_SCALE = 0.5F;
                        constexpr float const INTERNAL_EYE_WEIGHT_MAXIMUM = 0.5F;

                        // glTF
                        // RH
                        // Right -X
                        // Up +Y
                        // Forward +Z

                        constexpr DirectX::XMFLOAT3 const axis_right(-1.0F, 0.0F, 0.0F);

                        DirectX::XMVECTOR const eye_rotation_up = DirectX::XMQuaternionRotationAxis(DirectX::XMLoadFloat3(&axis_right), DirectX::XM_PI * INTERNAL_EYE_ANGLE_MAXIMUM);

                        DirectX::XMVECTOR const eye_rotation_down = DirectX::XMQuaternionRotationAxis(DirectX::XMLoadFloat3(&axis_right), -DirectX::XM_PI * INTERNAL_EYE_ANGLE_MAXIMUM);

                        constexpr DirectX::XMFLOAT3 const axis_up(0.0F, 1.0F, 0.0F);

                        DirectX::XMVECTOR const eye_rotation_right = DirectX::XMQuaternionRotationAxis(DirectX::XMLoadFloat3(&axis_up), -DirectX::XM_PI * INTERNAL_EYE_ANGLE_MAXIMUM);

                        DirectX::XMVECTOR const eye_rotation_left = DirectX::XMQuaternionRotationAxis(DirectX::XMLoadFloat3(&axis_up), DirectX::XM_PI * INTERNAL_EYE_ANGLE_MAXIMUM);

                        // right eye
                        {
                            DirectX::XMVECTOR right_eye_vertical_rotation;
                            if (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_UP_RIGHT] > media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_DOWN_RIGHT])
                            {
                                float const weight = (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_UP_RIGHT] - media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_DOWN_RIGHT]);
                                assert(weight >= 0.0F);

                                right_eye_vertical_rotation = DirectX::XMQuaternionSlerp(DirectX::XMQuaternionIdentity(), eye_rotation_up, std::min(weight * INTERNAL_EYE_WEIGHT_SCALE, INTERNAL_EYE_WEIGHT_MAXIMUM));
                            }
                            else
                            {
                                float const weight = (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_DOWN_RIGHT] - media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_UP_RIGHT]);
                                assert(weight >= 0.0F);

                                right_eye_vertical_rotation = DirectX::XMQuaternionSlerp(DirectX::XMQuaternionIdentity(), eye_rotation_down, std::min(weight * INTERNAL_EYE_WEIGHT_SCALE, INTERNAL_EYE_WEIGHT_MAXIMUM));
                            }

                            DirectX::XMVECTOR right_eye_horizontal_rotation;
                            if (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_IN_RIGHT] > media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_OUT_RIGHT])
                            {
                                float const weight = (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_IN_RIGHT] - media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_OUT_RIGHT]);
                                assert(weight >= 0.0F);

                                right_eye_horizontal_rotation = DirectX::XMQuaternionSlerp(DirectX::XMQuaternionIdentity(), eye_rotation_left, std::min(weight * INTERNAL_EYE_WEIGHT_SCALE, INTERNAL_EYE_WEIGHT_MAXIMUM));
                            }
                            else
                            {
                                float const weight = (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_OUT_RIGHT] - media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_IN_RIGHT]);
                                assert(weight >= 0.0F);

                                right_eye_horizontal_rotation = DirectX::XMQuaternionSlerp(DirectX::XMQuaternionIdentity(), eye_rotation_right, std::min(weight * INTERNAL_EYE_WEIGHT_SCALE, INTERNAL_EYE_WEIGHT_MAXIMUM));
                            }

                            DirectX::XMStoreFloat4(&right_eye_rotation, DirectX::XMQuaternionSlerp(right_eye_vertical_rotation, right_eye_horizontal_rotation, 0.5F));
                        }

                        // left eye
                        {
                            DirectX::XMVECTOR left_eye_vertical_rotation;
                            if (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_UP_LEFT] > media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_DOWN_LEFT])
                            {
                                float const weight = (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_UP_LEFT] - media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_DOWN_LEFT]);
                                assert(weight >= 0.0F);

                                left_eye_vertical_rotation = DirectX::XMQuaternionSlerp(DirectX::XMQuaternionIdentity(), eye_rotation_up, std::min(weight * INTERNAL_EYE_WEIGHT_SCALE, INTERNAL_EYE_WEIGHT_MAXIMUM));
                            }
                            else
                            {
                                float const weight = (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_DOWN_LEFT] - media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_UP_LEFT]);
                                assert(weight >= 0.0F);

                                left_eye_vertical_rotation = DirectX::XMQuaternionSlerp(DirectX::XMQuaternionIdentity(), eye_rotation_down, std::min(weight * INTERNAL_EYE_WEIGHT_SCALE, INTERNAL_EYE_WEIGHT_MAXIMUM));
                            }

                            DirectX::XMVECTOR left_eye_horizontal_rotation;
                            if (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_IN_LEFT] > media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_OUT_LEFT])
                            {
                                float const weight = (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_IN_LEFT] - media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_OUT_LEFT]);
                                assert(weight >= 0.0F);

                                left_eye_horizontal_rotation = DirectX::XMQuaternionSlerp(DirectX::XMQuaternionIdentity(), eye_rotation_right, std::min(weight * INTERNAL_EYE_WEIGHT_SCALE, INTERNAL_EYE_WEIGHT_MAXIMUM));
                            }
                            else
                            {
                                float const weight = (media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_OUT_LEFT] - media_pipe_weights[INTERNAL_MEDIA_PIPE_WEIGHT_NAME_EYE_LOOK_IN_LEFT]);
                                assert(weight >= 0.0F);

                                left_eye_horizontal_rotation = DirectX::XMQuaternionSlerp(DirectX::XMQuaternionIdentity(), eye_rotation_left, std::min(weight * INTERNAL_EYE_WEIGHT_SCALE, INTERNAL_EYE_WEIGHT_MAXIMUM));
                            }

                            DirectX::XMStoreFloat4(&left_eye_rotation, DirectX::XMQuaternionSlerp(left_eye_vertical_rotation, left_eye_horizontal_rotation, 0.5F));
                        }
                    }

                    face_skeleton_joint_rotations[INTERNAL_VIDEO_DETECTOR_FACE_SKELETON_JOINT_NAME_RIGHT_EYE] = right_eye_rotation;
                    face_skeleton_joint_rotations[INTERNAL_VIDEO_DETECTOR_FACE_SKELETON_JOINT_NAME_LEFT_EYE] = left_eye_rotation;
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

            if ((this->m_pose_count > 0U) && (0 == status_pose_landmarker_detect_for_video))
            {
                for (uint32_t pose_index = 0U; pose_index < this->m_pose_count && pose_index < pose_landmarker_result.pose_world_landmarks_count; ++pose_index)
                {
                    Landmarks const &pose_world_landmark = pose_landmarker_result.pose_world_landmarks[pose_index];

                    DirectX::XMFLOAT3 media_pipe_positions[INTERNAL_MEIDA_PIPE_POSITION_NAME_COUNT];
                    // initialize "false"
                    bool media_pipe_positions_valid[INTERNAL_MEIDA_PIPE_POSITION_NAME_COUNT] = {};

                    assert(INTERNAL_MEIDA_PIPE_POSITION_NAME_COUNT == pose_world_landmark.landmarks_count);

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
                        {INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_SHOULDER, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_RIGHT_ARM},
                        {INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_ELBOW, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_RIGHT_ELBOW},
                        {INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_WRIST, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_RIGHT_WRIST},
                        {INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_INDEX, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_RIGHT_INDEX},
                        {INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_HIP, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_RIGHT_LEG},
                        {INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_KNEE, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_RIGHT_KNEE},
                        {INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_ANKLE, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_RIGHT_ANKLE},
                        {INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_FOOT, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_RIGHT_TOE_TIP},
                        {INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_SHOULDER, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LEFT_ARM},
                        {INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_ELBOW, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LEFT_ELBOW},
                        {INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_WRIST, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LEFT_WRIST},
                        {INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_INDEX, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LEFT_INDEX},
                        {INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_HIP, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LEFT_LEG},
                        {INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_KNEE, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LEFT_KNEE},
                        {INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_ANKLE, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LEFT_ANKLE},
                        {INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_FOOT, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LEFT_TOE_TIP}};

                    constexpr uint32_t const mapping_count = sizeof(internal_media_pipe_mappings) / sizeof(internal_media_pipe_mappings[0]);

                    for (uint32_t mapping_index = 0U; mapping_index < mapping_count; ++mapping_index)
                    {
                        uint32_t const position_index = internal_media_pipe_mappings[mapping_index][0];
                        uint32_t const skeleton_joint_index = internal_media_pipe_mappings[mapping_index][1];

                        DirectX::XMStoreFloat3(&pose_skeleton_joint_translations_model_space[skeleton_joint_index], DirectX::XMVector3TransformCoord(DirectX::XMLoadFloat3(&media_pipe_positions[position_index]), simd_media_pipe_to_gltf));
                        pose_skeleton_joint_translations_model_space_valid[skeleton_joint_index] = media_pipe_positions_valid[position_index];
                    }

                    static_assert(INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_COUNT == (mapping_count + 2), "");

                    if (media_pipe_positions_valid[INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_HIP] && media_pipe_positions_valid[INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_HIP])
                    {
                        DirectX::XMStoreFloat3(&pose_skeleton_joint_translations_model_space[INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LOWER_BODY], DirectX::XMVectorScale(DirectX::XMVectorAdd(DirectX::XMVector3TransformCoord(DirectX::XMLoadFloat3(&media_pipe_positions[INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_HIP]), simd_media_pipe_to_gltf), DirectX::XMVector3TransformCoord(DirectX::XMLoadFloat3(&media_pipe_positions[INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_HIP]), simd_media_pipe_to_gltf)), 0.5F));

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
                }

                if (this->m_enable_debug_renderer)
                {
                    cv::Scalar const debug_renderer_pose_color(255, 0, 0);

                    for (uint32_t pose_index = 0U; pose_index < this->m_pose_count && pose_index < pose_landmarker_result.pose_landmarks_count; ++pose_index)
                    {
                        NormalizedLandmarks const &pose_landmark = pose_landmarker_result.pose_landmarks[pose_index];

                        assert(INTERNAL_MEIDA_PIPE_POSITION_NAME_COUNT == pose_landmark.landmarks_count);

                        constexpr uint32_t const internal_media_pipe_bones[][2] = {
                            //
                            {INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_SHOULDER, INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_SHOULDER},
                            //
                            {INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_SHOULDER, INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_ELBOW},
                            {INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_ELBOW, INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_WRIST},
                            {INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_WRIST, INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_THUMB},
                            {INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_WRIST, INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_INDEX},
                            {INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_WRIST, INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_PINKY},
                            //
                            {INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_SHOULDER, INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_ELBOW},
                            {INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_ELBOW, INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_WRIST},
                            {INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_WRIST, INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_THUMB},
                            {INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_WRIST, INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_INDEX},
                            {INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_WRIST, INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_PINKY},
                            //
                            {INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_HIP, INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_HIP},
                            //
                            {INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_HIP, INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_KNEE},
                            {INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_KNEE, INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_ANKLE},
                            {INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_ANKLE, INTERNAL_MEIDA_PIPE_POSITION_NAME_RIGHT_FOOT},
                            //
                            {INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_HIP, INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_KNEE},
                            {INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_KNEE, INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_ANKLE},
                            {INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_ANKLE, INTERNAL_MEIDA_PIPE_POSITION_NAME_LEFT_FOOT}};

                        constexpr uint32_t const bone_count = sizeof(internal_media_pipe_bones) / sizeof(internal_media_pipe_bones[0]);

                        for (uint32_t bone_index = 0U; bone_index < bone_count; ++bone_index)
                        {
                            uint32_t const begin_joint_landmark_index = internal_media_pipe_bones[bone_index][0];
                            uint32_t const end_joint_landmark_index = internal_media_pipe_bones[bone_index][1];

                            if ((begin_joint_landmark_index < pose_landmark.landmarks_count) && (end_joint_landmark_index < pose_landmark.landmarks_count))
                            {
                                NormalizedLandmark const &begin_normalized_landmark = pose_landmark.landmarks[begin_joint_landmark_index];

                                NormalizedLandmark const &end_normalized_landmark = pose_landmark.landmarks[end_joint_landmark_index];

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
        if (this->m_face_count > 0U)
        {
            face_landmarker_close_result(&face_landmarker_result);
        }

        if (this->m_pose_count > 0U)
        {
            pose_landmarker_close_result(&pose_landmarker_result);
        }
    }
    else
    {
        // use the last successful result when fail
    }
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
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_EYE:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_EYE == skeleton_joint_name);
        face_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_FACE_SKELETON_JOINT_NAME_RIGHT_EYE;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_EYE:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_EYE == skeleton_joint_name);
        face_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_FACE_SKELETON_JOINT_NAME_LEFT_EYE;
    }
    break;
    default:
    {
        // no assert here
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
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_INDEX_FINGER_1:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_INDEX_FINGER_1 == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_RIGHT_INDEX;
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
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_INDEX_FINGER_1:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_INDEX_FINGER_1 == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_LEFT_INDEX;
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
