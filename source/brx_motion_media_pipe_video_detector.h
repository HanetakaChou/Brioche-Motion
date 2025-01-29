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

#ifndef _BRX_MOTION_MEDIA_PIPE_VIDEO_DETECTOR_H_
#define _BRX_MOTION_MEDIA_PIPE_VIDEO_DETECTOR_H_ 1

#include "../include/brx_motion.h"
#include "../../McRT-Malloc/include/mcrt_string.h"
#include "../../McRT-Malloc/include/mcrt_unordered_map.h"
#include "../../McRT-Malloc/include/mcrt_vector.h"
#include <array>

enum BRX_MOTION_MEIDA_PIPE_MORPH_TARGET_NAME
{
	BRX_MOTION_MEIDA_PIPE_MORPH_TARGET_NAME_COUNT = 52
};

// https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker
enum BRX_MOTION_MEIDA_PIPE_JOINT_NAME
{
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_NOSE = 0,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_EYE_INNER = 1,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_EYE = 2,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_EYE_OUTER = 3,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_EYE_INNER = 4,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_EYE = 5,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_EYE_OUTER = 6,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_EAR = 7,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_EAR = 8,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_MOUTH_LEFT = 9,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_MOUTH_RIGHT = 10,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_SHOULDER = 11,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_SHOULDER = 12,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_ELBOW = 13,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_ELBOW = 14,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_WRIST = 15,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_WRIST = 16,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_PRINKY = 17,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_PRINKY = 18,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_INDEX = 19,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_INDEX = 20,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_THUMB = 21,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_THUMB = 22,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_HIP = 23,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_HIP = 24,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_KNEE = 25,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_KNEE = 26,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_ANKLE = 27,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_ANKLE = 28,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_HEEL = 29,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_HEEL = 30,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_LEFT_FOOT = 31,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_RIGHT_FOOT = 32,
	BRX_MOTION_MEIDA_PIPE_JOINT_NAME_COUNT = 33
};

// nose
//  |
//  - left_eye_inner - left_eye - left_eye_outer - left_ear
//  |
//  - right_eye_inner - right_eye - right_eye_outer - right_ear

// N/A
//  |
//  - mouth_left
//  |
//  - mouth_right

// N/A
//  |
//  - left_shoulder - left_elbow - left_wrist
//  |                               |
//  |                               - left_thumb
//  |                               |
//  |                               - left_index
//  |                               |
//  |                               - left_pinky
//  |
//  - lright_shoulder - right_elbow - right_wrist
//                                     |
//                                     - right_thumb
//                                     |
//                                     - right_index
//                                     |
//                                     - right_pinky

// N/A
// |
// - left_hip - left_knee - left_ankle
// |                         |
// |                         - left_foot
// |                         |
// |                         - left_heel
// |
// - right_hip - right_knee - right_ankle
// |                           |
// |                           - right_foot
// |                           |
// |                           - right_heel

class brx_motion_media_pipe_video_detector final : public brx_motion_video_detector
{
	uint32_t m_face_count;
	uint32_t m_pose_count;
	void *m_face_landmarker;
	void *m_pose_landmarker;
	mcrt_vector<std::array<float, BRX_MOTION_MEIDA_PIPE_MORPH_TARGET_NAME_COUNT>> m_morph_target_weights;
	// neck_to_head_rotation
	mcrt_vector<std::array<std::array<float, 3>, BRX_MOTION_MEIDA_PIPE_JOINT_NAME_COUNT>> m_joint_reaching_ik_target_world_positions;
	int64_t m_timestamp_ms;
	bool m_enable_debug_renderer;
	mcrt_string m_debug_renderer_window_name;
	bool m_enable_gpu;

public:
	brx_motion_media_pipe_video_detector();
	~brx_motion_media_pipe_video_detector();
	bool init(uint32_t face_count, uint32_t pose_count, bool force_gpu);
	void uninit();

private:
	uint32_t get_face_count() const override;
	uint32_t get_pose_count() const override;
	bool get_enable_gpu() const override;
	void set_enable_debug_renderer(bool enable_debug_renderer, char const *debug_renderer_window_name) override;
	bool get_enable_debug_renderer() const override;
	void step(brx_motion_video_frame const *video_frame, float delta_time) override;
	void get_vrm_morph_target_weights(float(out_morph_target_weights)[BRX_MOTION_VRM_MORPH_TARGET_NAME_COUNT]) const override;
	void get_arkit_morph_target_weights(float(out_morph_target_weights)[BRX_MOTION_ARKIT_MORPH_TARGET_NAME_COUNT]) const override;
	void map_skeleton(brx_motion_skeleton *skeleton) const override;
};

#endif
