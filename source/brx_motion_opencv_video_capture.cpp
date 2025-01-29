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

#include "../../McRT-Malloc/include/mcrt_malloc.h"
#include "../../McRT-Malloc/include/mcrt_tick_count.h"
#include "brx_motion_opencv_video_capture.h"
#include <cassert>
#include <cstring>
#include <new>

static inline void _internal_pause();

extern "C" brx_motion_video_capture *brx_motion_create_video_capture(char const *video_url)
{
    void *new_unwrapped_video_capture_base = mcrt_malloc(sizeof(brx_motion_opencv_video_capture), alignof(brx_motion_opencv_video_capture));
    assert(NULL != new_unwrapped_video_capture_base);

    brx_motion_opencv_video_capture *new_unwrapped_video_capture = new (new_unwrapped_video_capture_base) brx_motion_opencv_video_capture{};
    if (new_unwrapped_video_capture->init(video_url))
    {
        return new_unwrapped_video_capture;
    }
    else
    {
        new_unwrapped_video_capture->~brx_motion_opencv_video_capture();
        mcrt_free(new_unwrapped_video_capture);
        return NULL;
    }
}

extern void internal_retain_video_capture(brx_motion_video_capture *wrapped_video_capture)
{
    assert(NULL != wrapped_video_capture);
    brx_motion_opencv_video_capture *const retain_unwrapped_video_capture = static_cast<brx_motion_opencv_video_capture *>(wrapped_video_capture);

    retain_unwrapped_video_capture->retain();
}

extern void internal_release_video_capture(brx_motion_video_capture *wrapped_video_capture)
{
    assert(NULL != wrapped_video_capture);
    brx_motion_opencv_video_capture *const release_unwrapped_video_capture = static_cast<brx_motion_opencv_video_capture *>(wrapped_video_capture);

    if (0U == release_unwrapped_video_capture->internal_release())
    {
        brx_motion_opencv_video_capture *delete_unwrapped_video_capture = release_unwrapped_video_capture;

        delete_unwrapped_video_capture->uninit();

        delete_unwrapped_video_capture->~brx_motion_opencv_video_capture();
        mcrt_free(delete_unwrapped_video_capture);
    }
}

extern "C" void brx_motion_destroy_video_capture(brx_motion_video_capture *wrapped_video_capture)
{
    internal_release_video_capture(wrapped_video_capture);
}

brx_motion_opencv_video_capture::brx_motion_opencv_video_capture() : m_ref_count(0U), m_type(BRX_MOTION_VIDEO_CAPTURE_TYPE_UNKNOWN), m_tick_count_per_second(0U), m_tick_count_per_frame(0U), m_tick_count_previous_frame(0U), m_width(0U), m_height(0U), m_fps(0U)
{
}

brx_motion_opencv_video_capture::~brx_motion_opencv_video_capture()
{
    assert(this->m_backend_name.empty());
    assert(!this->m_video_capture.isOpened());
}

bool brx_motion_opencv_video_capture::init(char const *video_url)
{
    assert(0U == this->m_ref_count);
    this->m_ref_count = 1U;

    assert(!this->m_video_capture.isOpened());
    bool status_open_video_capture;
    if (0 == std::strncmp(video_url, "camera://", 9U))
    {
        int camera_index = 0;
        bool error_camera_index = false;
        for (size_t char_index = 9U; video_url[char_index] != '\0'; ++char_index)
        {
            // INT32_MAX 2147483647
            if (!(char_index < (9U + 10U)))
            {
                error_camera_index = true;
                break;
            }

            camera_index *= 10;

            switch (video_url[char_index])
            {
            case '0':
            {
            }
            break;
            case '1':
            {
                camera_index += 1;
            }
            break;
            case '2':
            {
                camera_index += 2;
            }
            break;
            case '3':
            {
                camera_index += 3;
            }
            break;
            case '4':
            {
                camera_index += 4;
            }
            break;
            case '5':
            {
                camera_index += 5;
            }
            break;
            case '6':
            {
                camera_index += 6;
            }
            break;
            case '7':
            {
                camera_index += 7;
            }
            break;
            case '8':
            {
                camera_index += 8;
            }
            break;
            case '9':
            {
                camera_index += 9;
            }
            break;
            default:
            {
                error_camera_index = true;
            }
            }

            if (error_camera_index)
            {
                break;
            }
        }

        if (!error_camera_index)
        {
            status_open_video_capture = this->m_video_capture.open(camera_index);

            if (status_open_video_capture)
            {
                this->m_type = BRX_MOTION_VIDEO_CAPTURE_TYPE_CAMERA;
            }
        }
        else
        {
            status_open_video_capture = false;
        }
    }
    else if (0 == std::strncmp(video_url, "file://", 7U))
    {
        status_open_video_capture = this->m_video_capture.open(video_url + 7U);

        if (status_open_video_capture)
        {
            this->m_type = BRX_MOTION_VIDEO_CAPTURE_TYPE_FILE;
        }
    }
    else
    {
        status_open_video_capture = false;
    }

    if (!status_open_video_capture)
    {
        assert(!this->m_video_capture.isOpened());
        return false;
    }

    assert(this->m_backend_name.empty());
    this->m_backend_name = this->m_video_capture.getBackendName().c_str();

    assert(0U == this->m_width);
    this->m_width = static_cast<uint32_t>(this->m_video_capture.get(cv::CAP_PROP_FRAME_WIDTH));

    assert(0U == this->m_height);
    this->m_height = static_cast<uint32_t>(this->m_video_capture.get(cv::CAP_PROP_FRAME_HEIGHT));

    assert(0U == this->m_fps);
    double const _double_fps = this->m_video_capture.get(cv::CAP_PROP_FPS);
    this->m_fps = static_cast<uint32_t>(_double_fps);

    if (BRX_MOTION_VIDEO_CAPTURE_TYPE_FILE == this->m_type)
    {
        assert(0U == this->m_tick_count_per_second);
        this->m_tick_count_per_second = mcrt_tick_count_per_second();

        assert(0U == this->m_tick_count_per_frame);
        this->m_tick_count_per_frame = static_cast<uint64_t>(static_cast<double>(mcrt_tick_count_per_second()) / _double_fps);

        assert(0U == this->m_tick_count_previous_frame);
        this->m_tick_count_previous_frame = mcrt_tick_count_now();
    }

    return true;
}

void brx_motion_opencv_video_capture::uninit()
{
    assert(0U == this->m_ref_count);

    assert(this->m_video_capture.isOpened());
    this->m_video_capture.release();

    assert(BRX_MOTION_VIDEO_CAPTURE_TYPE_CAMERA == this->m_type || BRX_MOTION_VIDEO_CAPTURE_TYPE_FILE == this->m_type);
    this->m_type = BRX_MOTION_VIDEO_CAPTURE_TYPE_UNKNOWN;

    assert(!this->m_backend_name.empty());
    this->m_backend_name.clear();
}

inline void brx_motion_opencv_video_capture::retain()
{
    assert(this->m_ref_count > 0U);
    assert(this->m_ref_count < static_cast<uint32_t>(UINT32_MAX));
    ++this->m_ref_count;
}

inline uint32_t brx_motion_opencv_video_capture::internal_release()
{
    assert(this->m_ref_count > 0U);
    --this->m_ref_count;
    return this->m_ref_count;
}

char const *brx_motion_opencv_video_capture::get_backend_name() const
{
    return this->m_backend_name.c_str();
}

void brx_motion_opencv_video_capture::set_resolution(uint32_t width, uint32_t height)
{
    if (width != this->m_width || height != this->m_height)
    {
        // TODO: we are sometimes asked to set width first while we are sometimes asked to set height first
        this->m_video_capture.set(cv::CAP_PROP_FRAME_WIDTH, width);
        this->m_video_capture.set(cv::CAP_PROP_FRAME_HEIGHT, height);
        this->m_video_capture.set(cv::CAP_PROP_FRAME_WIDTH, width);

        this->m_width = static_cast<uint32_t>(this->m_video_capture.get(cv::CAP_PROP_FRAME_WIDTH));
        this->m_height = static_cast<uint32_t>(this->m_video_capture.get(cv::CAP_PROP_FRAME_HEIGHT));
    }
}

void brx_motion_opencv_video_capture::set_fps(uint32_t fps)
{
    if (fps != this->m_fps)
    {
        this->m_video_capture.set(cv::CAP_PROP_FPS, fps);
        this->m_fps = static_cast<uint32_t>(this->m_video_capture.get(cv::CAP_PROP_FPS));
    }
}

BRX_MOTION_VIDEO_CAPTURE_TYPE brx_motion_opencv_video_capture::get_type() const
{
    return this->m_type;
}

uint32_t brx_motion_opencv_video_capture::get_width() const
{
    return this->m_width;
}

uint32_t brx_motion_opencv_video_capture::get_height() const
{
    return this->m_height;
}

uint32_t brx_motion_opencv_video_capture::get_fps() const
{
    return this->m_fps;
}

void brx_motion_opencv_video_capture::step()
{
    if (BRX_MOTION_VIDEO_CAPTURE_TYPE_FILE == this->m_type)
    {
        uint64_t const tick_count_next_second = this->m_tick_count_previous_frame + this->m_tick_count_per_second;
        uint64_t const tick_count_next_frame = this->m_tick_count_previous_frame + this->m_tick_count_per_frame;

        if (mcrt_tick_count_now() < tick_count_next_second)
        {
            while (mcrt_tick_count_now() < tick_count_next_frame)
            {
                _internal_pause();
            }
            this->m_tick_count_previous_frame = tick_count_next_frame;
        }
        else
        {
            this->m_tick_count_previous_frame = mcrt_tick_count_now();
        }
    }
    else
    {
        assert(BRX_MOTION_VIDEO_CAPTURE_TYPE_CAMERA == this->m_type);
        assert(0U == this->m_tick_count_per_frame);
        assert(0U == this->m_tick_count_previous_frame);
    }

    if (this->m_video_capture.read(this->m_video_frame))
    {
        assert(!this->m_video_frame.empty());
    }
    else
    {
        if (BRX_MOTION_VIDEO_CAPTURE_TYPE_FILE == this->m_type)
        {
            // Loop Playback
            this->m_video_capture.set(cv::CAP_PROP_POS_FRAMES, 0);
            this->m_tick_count_previous_frame = mcrt_tick_count_now();
        }
        else
        {
            assert(BRX_MOTION_VIDEO_CAPTURE_TYPE_CAMERA == this->m_type);
            assert(false);
        }
    }

    // use the last successful result
    // this->m_video_frame
}

cv::Mat const *brx_motion_opencv_video_capture::get_video_frame() const
{
    return &this->m_video_frame;
}

double brx_motion_opencv_video_capture::get_delta_time() const
{
    return 1.0 / static_cast<double>(this->m_fps);
}

#if defined(__GNUC__)
#if defined(__x86_64__) || defined(__i386__)
#include <immintrin.h>
#elif defined(__aarch64__) || defined(__arm__)
#include <arm_acle.h>
#else
#error Unknown Architecture
#endif
#elif defined(_MSC_VER)
#if defined(_M_X64) || defined(_M_IX86)
#include <immintrin.h>
#elif defined(_M_ARM64) || defined(_M_ARM)
#include <intrin.h>
#else
#error Unknown Architecture
#endif
#else
#error Unknown Compiler
#endif

static inline void _internal_pause()
{
#if defined(__GNUC__)
#if defined(__x86_64__) || defined(__i386__)
    _mm_pause();
#elif defined(__aarch64__) || defined(__arm__)
    __yield();
#else
#error Unknown Architecture
#endif
#elif defined(_MSC_VER)
#if defined(_M_X64) || defined(_M_IX86)
    _mm_pause();
#elif defined(_M_ARM64) || defined(_M_ARM)
    __yield();
#else
#error Unknown Architecture
#endif
#else
#error Unknown Compiler
#endif
}
