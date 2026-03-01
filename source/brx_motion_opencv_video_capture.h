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

#ifndef _BRX_MOTION_OPENCV_VIDEO_CAPTURE_H_
#define _BRX_MOTION_OPENCV_VIDEO_CAPTURE_H_ 1

#include "../include/brx_motion.h"
#include "brx_motion_video_capture.h"
#include "../../McRT-Malloc/include/mcrt_string.h"

class brx_motion_opencv_video_capture final : public internal_brx_motion_video_capture
{
	uint32_t m_ref_count;
	BRX_MOTION_VIDEO_CAPTURE_TYPE m_type;
	cv::VideoCapture m_video_capture;
	uint64_t m_tick_count_per_second;
	uint64_t m_tick_count_per_frame;
	uint64_t m_tick_count_previous_frame;
	cv::Mat m_video_frame;
	uint32_t m_width;
	uint32_t m_height;
	uint32_t m_fps;
	mcrt_string m_backend_name;
	bool m_enable_debug_renderer;
	void *m_debug_renderer_window;

public:
	brx_motion_opencv_video_capture();
	~brx_motion_opencv_video_capture();
	bool init(char const *video_url);
	void uninit();
private:
	inline void internal_retain();
	inline uint32_t internal_release();

	char const *get_backend_name() const override;
	void set_resolution(uint32_t width, uint32_t height) override;
	void set_fps(uint32_t fps) override;
	BRX_MOTION_VIDEO_CAPTURE_TYPE get_type() const override;
	uint32_t get_width() const override;
	uint32_t get_height() const override;
	uint32_t get_fps() const override;
	void set_enable_debug_renderer(bool enable_debug_renderer, char const *debug_renderer_window_name) override;
	bool get_enable_debug_renderer() const override;
	void step() override;

	cv::Mat const *get_video_frame() const override;
	double get_delta_time() const override;

	void retain() override;
	void release() override;
};

#endif
