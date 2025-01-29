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
#include "../../McRT-Malloc/include/mcrt_malloc.h"
#include <cassert>
#include <cstring>
#include <algorithm>
#include <new>

#include "../thirdparty/mediapipe/include/mediapipe/tasks/c/vision/face_landmarker/face_landmarker.h"
#include "../thirdparty/mediapipe/include/mediapipe/tasks/c/vision/pose_landmarker/pose_landmarker.h"

#define CV_IGNORE_DEBUG_BUILD_GUARD 1

#if defined(__GNUC__)
// CLANG or GCC
#include <opencv2/opencv.hpp>
#elif defined(_MSC_VER)
#if defined(__clang__)
// CLANG-CL
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-noreturn"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wdeprecated-anon-enum-enum-conversion"
#pragma GCC diagnostic ignored "-Wreturn-type-c-linkage"
#include <opencv2/opencv.hpp>
#pragma GCC diagnostic pop
#else
// MSVC
#include <opencv2/opencv.hpp>
#endif
#else
#error Unknown Compiler
#endif

static inline cv::Mat const *unwrap(brx_motion_video_frame const *wrapped_video_frame)
{
    return reinterpret_cast<cv::Mat const *>(wrapped_video_frame);
}

extern uint8_t const *const face_landmarker_task_base;
extern size_t const face_landmarker_task_size;

extern uint8_t const *const pose_landmarker_task_base;
extern size_t const pose_landmarker_task_size;

static constexpr uint32_t const BRX_MOTION_MEIDA_PIPE_MAX_FACE_COUNT = 2U;
static constexpr uint32_t const BRX_MOTION_MEIDA_PIPE_MAX_POSE_COUNT = 5U;

extern "C" brx_motion_video_detector *brx_motion_create_video_detector(uint32_t face_count, uint32_t pose_count, bool force_gpu)
{
    void *new_unwrapped_video_detector_base = mcrt_malloc(sizeof(brx_motion_media_pipe_video_detector), alignof(brx_motion_media_pipe_video_detector));
    assert(NULL != new_unwrapped_video_detector_base);

    brx_motion_media_pipe_video_detector *new_unwrapped_video_detector = new (new_unwrapped_video_detector_base) brx_motion_media_pipe_video_detector{};
    if (new_unwrapped_video_detector->init(face_count, pose_count, force_gpu))
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

extern "C" void brx_motion_destory_video_detector(brx_motion_video_detector *wrapped_video_detector)
{
    assert(NULL != wrapped_video_detector);
    brx_motion_media_pipe_video_detector *delete_unwrapped_video_detector = static_cast<brx_motion_media_pipe_video_detector *>(wrapped_video_detector);

    delete_unwrapped_video_detector->uninit();

    delete_unwrapped_video_detector->~brx_motion_media_pipe_video_detector();
    mcrt_free(delete_unwrapped_video_detector);
}

brx_motion_media_pipe_video_detector::brx_motion_media_pipe_video_detector() : m_face_count(0U), m_pose_count(0U), m_face_landmarker(NULL), m_pose_landmarker(NULL), m_timestamp_ms(0), m_enable_debug_renderer(false)
{
}

brx_motion_media_pipe_video_detector::~brx_motion_media_pipe_video_detector()
{
    assert(0U == this->m_face_count);
    assert(0U == this->m_pose_count);
    assert(NULL == this->m_face_landmarker);
    assert(NULL == this->m_pose_landmarker);
    assert(this->m_morph_target_weights.empty());
    assert(this->m_joint_reaching_ik_target_world_positions.empty());
    assert(this->m_debug_renderer_window_name.empty());
    assert(!this->m_enable_debug_renderer);
}

bool brx_motion_media_pipe_video_detector::init(uint32_t face_count, uint32_t pose_count, bool force_gpu)
{
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
    this->m_face_count = std::min(std::max(0U, face_count), BRX_MOTION_MEIDA_PIPE_MAX_FACE_COUNT);

    if (this->m_face_count > 0U)
    {
        FaceLandmarkerOptions options;
        options.base_options.model_asset_buffer = reinterpret_cast<char const *>(face_landmarker_task_base);
        options.base_options.model_asset_buffer_count = static_cast<unsigned int>(face_landmarker_task_size);
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

        assert(this->m_morph_target_weights.empty());
        this->m_morph_target_weights.resize(this->m_face_count);
    }

    assert(0U == this->m_pose_count);
    this->m_pose_count = std::min(std::max(0U, pose_count), BRX_MOTION_MEIDA_PIPE_MAX_POSE_COUNT);

    if (this->m_pose_count > 0U)
    {
        PoseLandmarkerOptions options;
        options.base_options.model_asset_buffer = reinterpret_cast<char const *>(pose_landmarker_task_base);
        options.base_options.model_asset_buffer_count = static_cast<unsigned int>(pose_landmarker_task_size);
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
                assert(!this->m_morph_target_weights.empty());
                assert(this->m_morph_target_weights.size() == this->m_face_count);
                this->m_morph_target_weights.clear();

                assert(NULL != this->m_face_landmarker);
                int status_face_landmarker_close = face_landmarker_close(this->m_face_landmarker, NULL);
                assert(0 == status_face_landmarker_close);
                this->m_face_landmarker = NULL;

                this->m_face_count = 0U;
            }
            else
            {
                assert(this->m_morph_target_weights.empty());
                assert(NULL == this->m_face_landmarker);
            }

            internal_tflite_unset_env_force_gpu();

            return false;
        }

        assert(this->m_joint_reaching_ik_target_world_positions.empty());
        this->m_joint_reaching_ik_target_world_positions.resize(this->m_pose_count);
    }

    internal_tflite_unset_env_force_gpu();

    return true;
}

void brx_motion_media_pipe_video_detector::uninit()
{
    if (this->m_enable_debug_renderer)
    {
        cv::destroyWindow(this->m_debug_renderer_window_name.c_str());
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
        assert(!this->m_joint_reaching_ik_target_world_positions.empty());
        assert(this->m_joint_reaching_ik_target_world_positions.size() == this->m_pose_count);
        this->m_joint_reaching_ik_target_world_positions.clear();

        assert(NULL != this->m_pose_landmarker);
        int status_pose_landmarker_close = pose_landmarker_close(this->m_pose_landmarker, NULL);
        assert(0 == status_pose_landmarker_close);
        this->m_pose_landmarker = NULL;

        this->m_pose_count = 0U;
    }
    else
    {
        assert(this->m_joint_reaching_ik_target_world_positions.empty());

        assert(NULL == this->m_pose_landmarker);
    }

    if (this->m_face_count > 0U)
    {
        assert(!this->m_morph_target_weights.empty());
        assert(this->m_morph_target_weights.size() == this->m_face_count);
        this->m_morph_target_weights.clear();

        assert(NULL != this->m_face_landmarker);
        int status_face_landmarker_close = face_landmarker_close(this->m_face_landmarker, NULL);
        assert(0 == status_face_landmarker_close);
        this->m_face_landmarker = NULL;

        this->m_face_count = 0U;
    }
    else
    {
        assert(this->m_morph_target_weights.empty());

        assert(NULL == this->m_face_landmarker);
    }
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

void brx_motion_media_pipe_video_detector::set_enable_debug_renderer(bool enable_debug_renderer, char const *debug_renderer_window_name)
{
    if (enable_debug_renderer != this->m_enable_debug_renderer)
    {
        if (enable_debug_renderer)
        {
            {
                char text_timestamp[] = {"18446744073709551615"};
                std::snprintf(text_timestamp, sizeof(text_timestamp) / sizeof(text_timestamp[0]), "%llu", static_cast<long long unsigned>(cv::getTickCount()));
                text_timestamp[(sizeof(text_timestamp) / sizeof(text_timestamp[0])) - 1] = '\0';

                assert(this->m_debug_renderer_window_name.empty());
                this->m_debug_renderer_window_name = debug_renderer_window_name;
                this->m_debug_renderer_window_name += " OpenCV Tick Count [";
                this->m_debug_renderer_window_name += text_timestamp;
                this->m_debug_renderer_window_name += "]";
            }

            cv::namedWindow(this->m_debug_renderer_window_name.c_str());
        }
        else
        {
            cv::destroyWindow(this->m_debug_renderer_window_name.c_str());

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

void brx_motion_media_pipe_video_detector::step(brx_motion_video_frame const *video_frame, float delta_time)
{
    this->m_timestamp_ms += static_cast<int64_t>((static_cast<double>(delta_time) * 1000.0));

    if (!unwrap(video_frame)->empty())
    {
        cv::Mat input_image;
        {
            cv::cvtColor((*unwrap(video_frame)), input_image, cv::COLOR_BGR2RGB);
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

        FaceLandmarkerResult face_landmarker_result = {};
        int status_face_landmarker_detect_for_video = -1;
        if (this->m_face_count > 0U)
        {
            assert(NULL != this->m_face_landmarker);
            status_face_landmarker_detect_for_video = face_landmarker_detect_for_video(this->m_face_landmarker, &mediapipe_input_image, this->m_timestamp_ms, &face_landmarker_result, NULL);
        }
        else
        {
            assert(NULL == this->m_face_landmarker);
        }

        PoseLandmarkerResult pose_landmarker_result = {};
        int status_pose_landmarker_detect_for_video = -1;
        if (this->m_pose_count > 0U)
        {
            assert(NULL != this->m_pose_landmarker);
            status_pose_landmarker_detect_for_video = pose_landmarker_detect_for_video(this->m_pose_landmarker, &mediapipe_input_image, this->m_timestamp_ms, &pose_landmarker_result, NULL);
        }
        else
        {
            assert(NULL == this->m_pose_landmarker);
        }

        {
            cv::Mat debug_renderer_output_image;

            if (this->m_enable_debug_renderer)
            {
                // we do NOT need the input image any more
                debug_renderer_output_image = std::move(input_image);
            }

            if ((this->m_face_count > 0U) && (0 == status_face_landmarker_detect_for_video))
            {
                for (uint32_t face_index = 0U; face_index < this->m_face_count && face_index < face_landmarker_result.face_blendshapes_count; ++face_index)
                {
                    Categories const &face_blendshape = face_landmarker_result.face_blendshapes[face_index];

                    assert(BRX_MOTION_MEIDA_PIPE_MORPH_TARGET_NAME_COUNT == face_blendshape.categories_count);

                    assert(face_index < this->m_morph_target_weights.size());

                    for (uint32_t blend_shape_index = 0U; blend_shape_index < face_blendshape.categories_count; ++blend_shape_index)
                    {
                        Category const &category = face_blendshape.categories[blend_shape_index];

                        // TODO: mcrt_unordered_map<mcrt_string, size_t>
                        // this->m_morph_target_name[category.category_name]
                        this->m_morph_target_weights[face_index][blend_shape_index] = category.score;
                    }
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
                                cv::circle(debug_renderer_output_image, point, 1, cv::Scalar(0, 255, 0), -1);
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

                    for (uint32_t world_landmark_index = 0U; world_landmark_index < pose_world_landmark.landmarks_count; ++world_landmark_index)
                    {
                        Landmark const &landmark = pose_world_landmark.landmarks[world_landmark_index];

                        if (((!landmark.has_visibility) || (landmark.visibility > 0.5F)) && ((!landmark.has_presence) || (landmark.presence > 0.5F)))
                        {
                            this->m_joint_reaching_ik_target_world_positions[pose_index][world_landmark_index][0] = landmark.x;
                            this->m_joint_reaching_ik_target_world_positions[pose_index][world_landmark_index][1] = landmark.y;
                            this->m_joint_reaching_ik_target_world_positions[pose_index][world_landmark_index][2] = landmark.z;
                        }
                    }
                }

                if (this->m_enable_debug_renderer)
                {
                    cv::Scalar const debug_renderer_pose_color(255, 0, 0);

                    for (uint32_t pose_index = 0U; pose_index < this->m_pose_count && pose_index < pose_landmarker_result.pose_landmarks_count; ++pose_index)
                    {
                        NormalizedLandmarks const &pose_landmark = pose_landmarker_result.pose_landmarks[pose_index];

                        assert(BRX_MOTION_MEIDA_PIPE_JOINT_NAME_COUNT == pose_landmark.landmarks_count);

                        if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_NOSE < pose_landmark.landmarks_count)
                        {
                            NormalizedLandmark const &nose_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_NOSE];

                            if (((!nose_normalized_landmark.has_visibility) || (nose_normalized_landmark.visibility > 0.5F)) && ((!nose_normalized_landmark.has_presence) || (nose_normalized_landmark.presence > 0.5F)))
                            {
                                cv::Point nose_point(static_cast<int>(nose_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(nose_normalized_landmark.y * debug_renderer_output_image.rows));

                                if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_EYE_INNER < pose_landmark.landmarks_count)
                                {
                                    NormalizedLandmark const &left_eye_inner_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_EYE_INNER];

                                    if (((!left_eye_inner_normalized_landmark.has_visibility) || (left_eye_inner_normalized_landmark.visibility > 0.5F)) && ((!left_eye_inner_normalized_landmark.has_presence) || (left_eye_inner_normalized_landmark.presence > 0.5F)))
                                    {
                                        cv::Point left_eye_inner_point(static_cast<int>(left_eye_inner_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(left_eye_inner_normalized_landmark.y * debug_renderer_output_image.rows));

                                        cv::line(debug_renderer_output_image, nose_point, left_eye_inner_point, debug_renderer_pose_color, 1);

                                        if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_EYE < pose_landmark.landmarks_count)
                                        {
                                            NormalizedLandmark const &left_eye_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_EYE];

                                            if (((!left_eye_normalized_landmark.has_visibility) || (left_eye_normalized_landmark.visibility > 0.5F)) && ((!left_eye_normalized_landmark.has_presence) || (left_eye_normalized_landmark.presence > 0.5F)))
                                            {
                                                cv::Point left_eye_point(static_cast<int>(left_eye_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(left_eye_normalized_landmark.y * debug_renderer_output_image.rows));

                                                cv::line(debug_renderer_output_image, left_eye_inner_point, left_eye_point, debug_renderer_pose_color, 1);

                                                if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_EYE_OUTER < pose_landmark.landmarks_count)
                                                {
                                                    NormalizedLandmark const &left_eye_outer_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_EYE_OUTER];

                                                    if (((!left_eye_outer_normalized_landmark.has_visibility) || (left_eye_outer_normalized_landmark.visibility > 0.5F)) && ((!left_eye_outer_normalized_landmark.has_presence) || (left_eye_outer_normalized_landmark.presence > 0.5F)))
                                                    {
                                                        cv::Point left_eye_outer_point(static_cast<int>(left_eye_outer_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(left_eye_outer_normalized_landmark.y * debug_renderer_output_image.rows));

                                                        cv::line(debug_renderer_output_image, left_eye_point, left_eye_outer_point, debug_renderer_pose_color, 1);

                                                        if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_EAR < pose_landmark.landmarks_count)
                                                        {
                                                            NormalizedLandmark const &left_ear_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_EAR];

                                                            if (((!left_ear_normalized_landmark.has_visibility) || (left_ear_normalized_landmark.visibility > 0.5F)) && ((!left_ear_normalized_landmark.has_presence) || (left_ear_normalized_landmark.presence > 0.5F)))
                                                            {
                                                                cv::Point left_ear_point(static_cast<int>(left_ear_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(left_ear_normalized_landmark.y * debug_renderer_output_image.rows));

                                                                cv::line(debug_renderer_output_image, left_eye_outer_point, left_ear_point, debug_renderer_pose_color, 1);
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }

                                if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_EYE_INNER < pose_landmark.landmarks_count)
                                {
                                    NormalizedLandmark const &right_eye_inner_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_EYE_INNER];

                                    if (((!right_eye_inner_normalized_landmark.has_visibility) || (right_eye_inner_normalized_landmark.visibility > 0.5F)) && ((!right_eye_inner_normalized_landmark.has_presence) || (right_eye_inner_normalized_landmark.presence > 0.5F)))
                                    {
                                        cv::Point right_eye_inner_point(static_cast<int>(right_eye_inner_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(right_eye_inner_normalized_landmark.y * debug_renderer_output_image.rows));

                                        cv::line(debug_renderer_output_image, nose_point, right_eye_inner_point, debug_renderer_pose_color, 1);

                                        if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_EYE < pose_landmark.landmarks_count)
                                        {
                                            NormalizedLandmark const &right_eye_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_EYE];

                                            if (((!right_eye_normalized_landmark.has_visibility) || (right_eye_normalized_landmark.visibility > 0.5F)) && ((!right_eye_normalized_landmark.has_presence) || (right_eye_normalized_landmark.presence > 0.5F)))
                                            {
                                                cv::Point right_eye_point(static_cast<int>(right_eye_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(right_eye_normalized_landmark.y * debug_renderer_output_image.rows));

                                                cv::line(debug_renderer_output_image, right_eye_inner_point, right_eye_point, debug_renderer_pose_color, 1);

                                                if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_EYE_OUTER < pose_landmark.landmarks_count)
                                                {
                                                    NormalizedLandmark const &right_eye_outer_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_EYE_OUTER];

                                                    if (((!right_eye_outer_normalized_landmark.has_visibility) || (right_eye_outer_normalized_landmark.visibility > 0.5F)) && ((!right_eye_outer_normalized_landmark.has_presence) || (right_eye_outer_normalized_landmark.presence > 0.5F)))
                                                    {
                                                        cv::Point right_eye_outer_point(static_cast<int>(right_eye_outer_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(right_eye_outer_normalized_landmark.y * debug_renderer_output_image.rows));

                                                        cv::line(debug_renderer_output_image, right_eye_point, right_eye_outer_point, debug_renderer_pose_color, 1);

                                                        if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_EAR < pose_landmark.landmarks_count)
                                                        {
                                                            NormalizedLandmark const &right_ear_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_EAR];

                                                            if (((!right_ear_normalized_landmark.has_visibility) || (right_ear_normalized_landmark.visibility > 0.5F)) && ((!right_ear_normalized_landmark.has_presence) || (right_ear_normalized_landmark.presence > 0.5F)))
                                                            {
                                                                cv::Point right_ear_point(static_cast<int>(right_ear_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(right_ear_normalized_landmark.y * debug_renderer_output_image.rows));

                                                                cv::line(debug_renderer_output_image, right_eye_outer_point, right_ear_point, debug_renderer_pose_color, 1);
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        {
                            cv::Point mouth_left_point;
                            if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_MOUTH_LEFT < pose_landmark.landmarks_count)
                            {
                                NormalizedLandmark const &mouth_left_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_MOUTH_LEFT];

                                if (((!mouth_left_normalized_landmark.has_visibility) || (mouth_left_normalized_landmark.visibility > 0.5F)) && ((!mouth_left_normalized_landmark.has_presence) || (mouth_left_normalized_landmark.presence > 0.5F)))
                                {
                                    mouth_left_point = cv::Point(static_cast<int>(mouth_left_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(mouth_left_normalized_landmark.y * debug_renderer_output_image.rows));
                                }
                            }

                            cv::Point mouth_right_point;
                            if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_MOUTH_RIGHT < pose_landmark.landmarks_count)
                            {
                                NormalizedLandmark const &mouth_right_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_MOUTH_RIGHT];

                                if (((!mouth_right_normalized_landmark.has_visibility) || (mouth_right_normalized_landmark.visibility > 0.5F)) && ((!mouth_right_normalized_landmark.has_presence) || (mouth_right_normalized_landmark.presence > 0.5F)))
                                {
                                    mouth_right_point = cv::Point(static_cast<int>(mouth_right_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(mouth_right_normalized_landmark.y * debug_renderer_output_image.rows));
                                }
                            }

                            if ((BRX_MOTION_MEIDA_PIPE_JOINT_NAME_MOUTH_LEFT < pose_landmark.landmarks_count) && (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_MOUTH_RIGHT < pose_landmark.landmarks_count))
                            {
                                cv::line(debug_renderer_output_image, mouth_left_point, mouth_right_point, debug_renderer_pose_color, 1);
                            }
                        }

                        {
                            cv::Point left_shoulder_point;
                            if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_SHOULDER < pose_landmark.landmarks_count)
                            {
                                NormalizedLandmark const &left_shoulder_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_SHOULDER];

                                if (((!left_shoulder_normalized_landmark.has_visibility) || (left_shoulder_normalized_landmark.visibility > 0.5F)) && ((!left_shoulder_normalized_landmark.has_presence) || (left_shoulder_normalized_landmark.presence > 0.5F)))
                                {
                                    left_shoulder_point = cv::Point(static_cast<int>(left_shoulder_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(left_shoulder_normalized_landmark.y * debug_renderer_output_image.rows));

                                    if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_ELBOW < pose_landmark.landmarks_count)
                                    {
                                        NormalizedLandmark const &left_elbow_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_ELBOW];

                                        if (((!left_elbow_normalized_landmark.has_visibility) || (left_elbow_normalized_landmark.visibility > 0.5F)) && ((!left_elbow_normalized_landmark.has_presence) || (left_elbow_normalized_landmark.presence > 0.5F)))
                                        {
                                            cv::Point left_elbow_point(static_cast<int>(left_elbow_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(left_elbow_normalized_landmark.y * debug_renderer_output_image.rows));

                                            cv::line(debug_renderer_output_image, left_shoulder_point, left_elbow_point, debug_renderer_pose_color, 1);

                                            if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_WRIST < pose_landmark.landmarks_count)
                                            {
                                                NormalizedLandmark const &left_wrist_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_WRIST];

                                                if (((!left_wrist_normalized_landmark.has_visibility) || (left_wrist_normalized_landmark.visibility > 0.5F)) && ((!left_wrist_normalized_landmark.has_presence) || (left_wrist_normalized_landmark.presence > 0.5F)))
                                                {
                                                    cv::Point left_wrist_point(static_cast<int>(left_wrist_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(left_wrist_normalized_landmark.y * debug_renderer_output_image.rows));

                                                    cv::line(debug_renderer_output_image, left_elbow_point, left_wrist_point, debug_renderer_pose_color, 1);

                                                    if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_THUMB < pose_landmark.landmarks_count)
                                                    {
                                                        NormalizedLandmark const &left_thumb_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_THUMB];

                                                        if (((!left_thumb_normalized_landmark.has_visibility) || (left_thumb_normalized_landmark.visibility > 0.5F)) && ((!left_thumb_normalized_landmark.has_presence) || (left_thumb_normalized_landmark.presence > 0.5F)))
                                                        {
                                                            cv::Point left_thumb_point(static_cast<int>(left_thumb_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(left_thumb_normalized_landmark.y * debug_renderer_output_image.rows));

                                                            cv::line(debug_renderer_output_image, left_wrist_point, left_thumb_point, debug_renderer_pose_color, 1);
                                                        }
                                                    }

                                                    if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_INDEX < pose_landmark.landmarks_count)
                                                    {
                                                        NormalizedLandmark const &left_index_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_INDEX];

                                                        if (((!left_index_normalized_landmark.has_visibility) || (left_index_normalized_landmark.visibility > 0.5F)) && ((!left_index_normalized_landmark.has_presence) || (left_index_normalized_landmark.presence > 0.5F)))
                                                        {
                                                            cv::Point left_index_point(static_cast<int>(left_index_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(left_index_normalized_landmark.y * debug_renderer_output_image.rows));

                                                            cv::line(debug_renderer_output_image, left_wrist_point, left_index_point, debug_renderer_pose_color, 1);
                                                        }
                                                    }

                                                    if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_PRINKY < pose_landmark.landmarks_count)
                                                    {
                                                        NormalizedLandmark const &left_pinky_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_PRINKY];

                                                        if (((!left_pinky_normalized_landmark.has_visibility) || (left_pinky_normalized_landmark.visibility > 0.5F)) && ((!left_pinky_normalized_landmark.has_presence) || (left_pinky_normalized_landmark.presence > 0.5F)))
                                                        {
                                                            cv::Point left_pinky_point(static_cast<int>(left_pinky_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(left_pinky_normalized_landmark.y * debug_renderer_output_image.rows));

                                                            cv::line(debug_renderer_output_image, left_wrist_point, left_pinky_point, debug_renderer_pose_color, 1);
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }

                            cv::Point right_shoulder_point;
                            if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_SHOULDER < pose_landmark.landmarks_count)
                            {
                                NormalizedLandmark const &right_shoulder_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_SHOULDER];

                                if (((!right_shoulder_normalized_landmark.has_visibility) || (right_shoulder_normalized_landmark.visibility > 0.5F)) && ((!right_shoulder_normalized_landmark.has_presence) || (right_shoulder_normalized_landmark.presence > 0.5F)))
                                {
                                    right_shoulder_point = cv::Point(static_cast<int>(right_shoulder_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(right_shoulder_normalized_landmark.y * debug_renderer_output_image.rows));

                                    if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_ELBOW < pose_landmark.landmarks_count)
                                    {
                                        NormalizedLandmark const &right_elbow_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_ELBOW];

                                        if (((!right_elbow_normalized_landmark.has_visibility) || (right_elbow_normalized_landmark.visibility > 0.5F)) && ((!right_elbow_normalized_landmark.has_presence) || (right_elbow_normalized_landmark.presence > 0.5F)))
                                        {
                                            cv::Point right_elbow_point(static_cast<int>(right_elbow_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(right_elbow_normalized_landmark.y * debug_renderer_output_image.rows));

                                            cv::line(debug_renderer_output_image, right_shoulder_point, right_elbow_point, debug_renderer_pose_color, 1);

                                            if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_WRIST < pose_landmark.landmarks_count)
                                            {
                                                NormalizedLandmark const &right_wrist_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_WRIST];

                                                if (((!right_wrist_normalized_landmark.has_visibility) || (right_wrist_normalized_landmark.visibility > 0.5F)) && ((!right_wrist_normalized_landmark.has_presence) || (right_wrist_normalized_landmark.presence > 0.5F)))
                                                {
                                                    cv::Point right_wrist_point(static_cast<int>(right_wrist_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(right_wrist_normalized_landmark.y * debug_renderer_output_image.rows));

                                                    cv::line(debug_renderer_output_image, right_elbow_point, right_wrist_point, debug_renderer_pose_color, 1);

                                                    if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_THUMB < pose_landmark.landmarks_count)
                                                    {
                                                        NormalizedLandmark const &right_thumb_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_THUMB];

                                                        if (((!right_thumb_normalized_landmark.has_visibility) || (right_thumb_normalized_landmark.visibility > 0.5F)) && ((!right_thumb_normalized_landmark.has_presence) || (right_thumb_normalized_landmark.presence > 0.5F)))
                                                        {
                                                            cv::Point right_thumb_point(static_cast<int>(right_thumb_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(right_thumb_normalized_landmark.y * debug_renderer_output_image.rows));

                                                            cv::line(debug_renderer_output_image, right_wrist_point, right_thumb_point, debug_renderer_pose_color, 1);
                                                        }
                                                    }

                                                    if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_INDEX < pose_landmark.landmarks_count)
                                                    {
                                                        NormalizedLandmark const &right_index_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_INDEX];

                                                        if (((!right_index_normalized_landmark.has_visibility) || (right_index_normalized_landmark.visibility > 0.5F)) && ((!right_index_normalized_landmark.has_presence) || (right_index_normalized_landmark.presence > 0.5F)))
                                                        {
                                                            cv::Point right_index_point(static_cast<int>(right_index_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(right_index_normalized_landmark.y * debug_renderer_output_image.rows));

                                                            cv::line(debug_renderer_output_image, right_wrist_point, right_index_point, debug_renderer_pose_color, 1);
                                                        }
                                                    }

                                                    if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_PRINKY < pose_landmark.landmarks_count)
                                                    {
                                                        NormalizedLandmark const &right_pinky_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_PRINKY];

                                                        if (((!right_pinky_normalized_landmark.has_visibility) || (right_pinky_normalized_landmark.visibility > 0.5F)) && ((!right_pinky_normalized_landmark.has_presence) || (right_pinky_normalized_landmark.presence > 0.5F)))
                                                        {
                                                            cv::Point right_pinky_point(static_cast<int>(right_pinky_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(right_pinky_normalized_landmark.y * debug_renderer_output_image.rows));

                                                            cv::line(debug_renderer_output_image, right_wrist_point, right_pinky_point, debug_renderer_pose_color, 1);
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }

                            if ((BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_SHOULDER < pose_landmark.landmarks_count) && (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_SHOULDER < pose_landmark.landmarks_count))
                            {
                                cv::line(debug_renderer_output_image, left_shoulder_point, right_shoulder_point, debug_renderer_pose_color, 1);
                            }
                        }

                        {
                            cv::Point left_hip_point;
                            if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_HIP < pose_landmark.landmarks_count)
                            {
                                NormalizedLandmark const &left_hip_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_HIP];

                                if (((!left_hip_normalized_landmark.has_visibility) || (left_hip_normalized_landmark.visibility > 0.5F)) && ((!left_hip_normalized_landmark.has_presence) || (left_hip_normalized_landmark.presence > 0.5F)))
                                {
                                    left_hip_point = cv::Point(static_cast<int>(left_hip_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(left_hip_normalized_landmark.y * debug_renderer_output_image.rows));
                                }

                                if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_KNEE < pose_landmark.landmarks_count)
                                {
                                    NormalizedLandmark const &left_knee_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_KNEE];

                                    if (((!left_knee_normalized_landmark.has_visibility) || (left_knee_normalized_landmark.visibility > 0.5F)) && ((!left_knee_normalized_landmark.has_presence) || (left_knee_normalized_landmark.presence > 0.5F)))
                                    {
                                        cv::Point left_knee_point(static_cast<int>(left_knee_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(left_knee_normalized_landmark.y * debug_renderer_output_image.rows));

                                        cv::line(debug_renderer_output_image, left_hip_point, left_knee_point, debug_renderer_pose_color, 1);

                                        if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_ANKLE < pose_landmark.landmarks_count)
                                        {
                                            NormalizedLandmark const &left_ankle_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_ANKLE];

                                            if (((!left_ankle_normalized_landmark.has_visibility) || (left_ankle_normalized_landmark.visibility > 0.5F)) && ((!left_ankle_normalized_landmark.has_presence) || (left_ankle_normalized_landmark.presence > 0.5F)))
                                            {
                                                cv::Point left_ankle_point(static_cast<int>(left_ankle_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(left_ankle_normalized_landmark.y * debug_renderer_output_image.rows));

                                                cv::line(debug_renderer_output_image, left_knee_point, left_ankle_point, debug_renderer_pose_color, 1);

                                                if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_FOOT < pose_landmark.landmarks_count)
                                                {
                                                    NormalizedLandmark const &left_foot_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_FOOT];

                                                    if (((!left_foot_normalized_landmark.has_visibility) || (left_foot_normalized_landmark.visibility > 0.5F)) && ((!left_foot_normalized_landmark.has_presence) || (left_foot_normalized_landmark.presence > 0.5F)))
                                                    {
                                                        cv::Point left_foot_point(static_cast<int>(left_foot_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(left_foot_normalized_landmark.y * debug_renderer_output_image.rows));

                                                        cv::line(debug_renderer_output_image, left_ankle_point, left_foot_point, debug_renderer_pose_color, 1);
                                                    }
                                                }

                                                if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_HEEL < pose_landmark.landmarks_count)
                                                {
                                                    NormalizedLandmark const &left_heel_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_HEEL];

                                                    if (((!left_heel_normalized_landmark.has_visibility) || (left_heel_normalized_landmark.visibility > 0.5F)) && ((!left_heel_normalized_landmark.has_presence) || (left_heel_normalized_landmark.presence > 0.5F)))
                                                    {
                                                        cv::Point left_heel_point(static_cast<int>(left_heel_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(left_heel_normalized_landmark.y * debug_renderer_output_image.rows));

                                                        cv::line(debug_renderer_output_image, left_ankle_point, left_heel_point, debug_renderer_pose_color, 1);
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }

                            cv::Point right_hip_point;
                            if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_HIP < pose_landmark.landmarks_count)
                            {
                                NormalizedLandmark const &right_hip_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_HIP];

                                if (((!right_hip_normalized_landmark.has_visibility) || (right_hip_normalized_landmark.visibility > 0.5F)) && ((!right_hip_normalized_landmark.has_presence) || (right_hip_normalized_landmark.presence > 0.5F)))
                                {
                                    right_hip_point = cv::Point(static_cast<int>(right_hip_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(right_hip_normalized_landmark.y * debug_renderer_output_image.rows));

                                    if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_KNEE < pose_landmark.landmarks_count)
                                    {
                                        NormalizedLandmark const &right_knee_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_KNEE];

                                        if (((!right_knee_normalized_landmark.has_visibility) || (right_knee_normalized_landmark.visibility > 0.5F)) && ((!right_knee_normalized_landmark.has_presence) || (right_knee_normalized_landmark.presence > 0.5F)))
                                        {
                                            cv::Point right_knee_point(static_cast<int>(right_knee_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(right_knee_normalized_landmark.y * debug_renderer_output_image.rows));

                                            cv::line(debug_renderer_output_image, right_hip_point, right_knee_point, debug_renderer_pose_color, 1);

                                            if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_ANKLE < pose_landmark.landmarks_count)
                                            {
                                                NormalizedLandmark const &right_ankle_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_ANKLE];

                                                if (((!right_ankle_normalized_landmark.has_visibility) || (right_ankle_normalized_landmark.visibility > 0.5F)) && ((!right_ankle_normalized_landmark.has_presence) || (right_ankle_normalized_landmark.presence > 0.5F)))
                                                {
                                                    cv::Point right_ankle_point(static_cast<int>(right_ankle_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(right_ankle_normalized_landmark.y * debug_renderer_output_image.rows));

                                                    cv::line(debug_renderer_output_image, right_knee_point, right_ankle_point, debug_renderer_pose_color, 1);

                                                    if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_FOOT < pose_landmark.landmarks_count)
                                                    {
                                                        NormalizedLandmark const &right_foot_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_FOOT];

                                                        if (((!right_foot_normalized_landmark.has_visibility) || (right_foot_normalized_landmark.visibility > 0.5F)) && ((!right_foot_normalized_landmark.has_presence) || (right_foot_normalized_landmark.presence > 0.5F)))
                                                        {
                                                            cv::Point right_foot_point(static_cast<int>(right_foot_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(right_foot_normalized_landmark.y * debug_renderer_output_image.rows));

                                                            cv::line(debug_renderer_output_image, right_ankle_point, right_foot_point, debug_renderer_pose_color, 1);
                                                        }
                                                    }

                                                    if (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_HEEL < pose_landmark.landmarks_count)
                                                    {
                                                        NormalizedLandmark const &right_heel_normalized_landmark = pose_landmark.landmarks[BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_HEEL];

                                                        if (((!right_heel_normalized_landmark.has_visibility) || (right_heel_normalized_landmark.visibility > 0.5F)) && ((!right_heel_normalized_landmark.has_presence) || (right_heel_normalized_landmark.presence > 0.5F)))
                                                        {
                                                            cv::Point right_heel_point(static_cast<int>(right_heel_normalized_landmark.x * debug_renderer_output_image.cols), static_cast<int>(right_heel_normalized_landmark.y * debug_renderer_output_image.rows));

                                                            cv::line(debug_renderer_output_image, right_ankle_point, right_heel_point, debug_renderer_pose_color, 1);
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }

                            if ((BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_HIP < pose_landmark.landmarks_count) && (BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_HIP < pose_landmark.landmarks_count))
                            {
                                cv::line(debug_renderer_output_image, left_hip_point, right_hip_point, debug_renderer_pose_color, 1);
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
                cv::cvtColor(debug_renderer_output_image, debug_renderer_raw_output_image, cv::COLOR_RGB2BGR);
                debug_renderer_output_image.release();

                cv::imshow(this->m_debug_renderer_window_name.c_str(), debug_renderer_raw_output_image);
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

void brx_motion_media_pipe_video_detector::get_vrm_morph_target_weights(float(out_morph_target_weights)[BRX_MOTION_VRM_MORPH_TARGET_NAME_COUNT]) const
{
    // https://developer.apple.com/documentation/arkit/arskeleton/jointname
    // https://ai.google.dev/edge/mediapipe/solutions/vision/face_landmarker
}

void brx_motion_media_pipe_video_detector::get_arkit_morph_target_weights(float(out_morph_target_weights)[BRX_MOTION_ARKIT_MORPH_TARGET_NAME_COUNT]) const
{
    // https://developer.apple.com/documentation/arkit/arskeleton/jointname
    // https://ai.google.dev/edge/mediapipe/solutions/vision/face_landmarker
}

void brx_motion_media_pipe_video_detector::map_skeleton(brx_motion_skeleton *skeleton) const
{
    // IK
    //
    // Reaching IK (position-based IK): target position (end effector joint)
    // two joints: hinge Axis (second joint)
    // three joints: hinge Axis (first joint)
    // more joints: CCD or FABRIK
    // FABRIK: http://www.andreasaristidou.com/FABRIK.html
    //
    // Look At IK (rotation-based IK) target position (not really need to reach)
    //
    // Foot IK
    // use ground detection to find the correct foot placement
    // the problem can be converted to Reaching IK

    // Skeleton Map
    //
    // Chain Map
    // similiar to Reaching IK
}
