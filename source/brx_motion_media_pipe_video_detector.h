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
#include "brx_motion_video_detector.h"
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
#include <array>

class brx_motion_media_pipe_video_detector final : public brx_motion_video_detector
{
	uint32_t m_ref_count;
	uint32_t m_face_count;
	uint32_t m_pose_count;
	void *m_face_landmarker;
	void *m_pose_landmarker;
	mcrt_vector<std::array<float, BRX_MOTION_MORPH_TARGET_NAME_MMD_COUNT>> m_faces_morph_target_weights;
	mcrt_vector<std::array<DirectX::XMFLOAT4, INTERNAL_VIDEO_DETECTOR_FACE_SKELETON_JOINT_NAME_COUNT>> m_faces_skeleton_joint_rotations;
	mcrt_vector<std::array<bool, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_COUNT>> m_poses_skeleton_joint_translations_model_space_valid;
	mcrt_vector<std::array<DirectX::XMFLOAT3, INTERNAL_VIDEO_DETECTOR_POSE_SKELETON_JOINT_NAME_COUNT>> m_poses_skeleton_joint_translations_model_space;
	int64_t m_timestamp_ms;
	bool m_enable_debug_renderer;
	mcrt_string m_debug_renderer_window_name;
	void *m_debug_renderer_window;
	bool m_enable_gpu;
	brx_motion_video_capture const *m_input_video_capture;

public:
	brx_motion_media_pipe_video_detector();
	~brx_motion_media_pipe_video_detector();
	bool init(uint32_t face_count, uint32_t pose_count, bool force_gpu, brx_motion_video_capture const *video_capture);
	void uninit();
	inline void retain();
	inline uint32_t internal_release();

private:
	uint32_t get_face_count() const override;
	uint32_t get_pose_count() const override;
	bool get_enable_gpu() const override;
	brx_motion_video_capture *get_input() const override;
	void set_enable_debug_renderer(bool enable_debug_renderer, char const *debug_renderer_window_name) override;
	bool get_enable_debug_renderer() const override;
	void step() override;
	float get_morph_target_weight(uint32_t face_index, BRX_MOTION_MORPH_TARGET_NAME morph_target_name) const override;

public:
	DirectX::XMFLOAT4 const *get_face_skeleton_joint_rotation(uint32_t face_index, BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name) const;
	DirectX::XMFLOAT3 const *get_pose_skeleton_joint_translation(uint32_t pose_index, BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name) const;
};

#endif
