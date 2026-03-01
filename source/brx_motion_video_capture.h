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

#ifndef _BRX_MOTION_VIDEO_CAPTURE_H_
#define _BRX_MOTION_VIDEO_CAPTURE_H_ 1

#include "../include/brx_motion.h"
// #define CV_IGNORE_DEBUG_BUILD_GUARD 1
#if defined(__GNUC__)
// CLANG or GCC
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#elif defined(_MSC_VER)
#if defined(__clang__)
// CLANG-CL
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-noreturn"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wdeprecated-anon-enum-enum-conversion"
#pragma GCC diagnostic ignored "-Wreturn-type-c-linkage"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#pragma GCC diagnostic pop
#else
// MSVC
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#endif
#else
#error Unknown Compiler
#endif

class internal_brx_motion_video_capture : public brx_motion_video_capture
{
public:
    virtual cv::Mat const *get_video_frame() const = 0;
    virtual double get_delta_time() const = 0;
    virtual void retain() = 0;
    virtual void release() = 0;
};

#endif
