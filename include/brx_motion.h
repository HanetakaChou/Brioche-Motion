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

#ifndef _BRX_MOTION_H_
#define _BRX_MOTION_H_ 1

#include "brx_motion_format.h"
#include <cstddef>
#include <cstdint>

enum BRX_MOTION_VIDEO_CAPTURE_TYPE
{
    BRX_MOTION_VIDEO_CAPTURE_TYPE_UNKNOWN = 0,
    BRX_MOTION_VIDEO_CAPTURE_TYPE_CAMERA = 1,
    BRX_MOTION_VIDEO_CAPTURE_TYPE_FILE = 2
};

class brx_motion_video_capture;
class brx_motion_video_frame;
class brx_motion_video_detector;
class brx_motion_skeleton;
class brx_motion_skeleton_animation;
class brx_motion_skeleton_animation_instance;

// camera://0
// file://a.mp4
extern "C" brx_motion_video_capture *brx_motion_create_video_capture(char const *video_url);

extern "C" void brx_motion_destory_video_capture(brx_motion_video_capture *video_capture);

class brx_motion_video_capture
{
public:
    virtual char const *get_backend_name() const = 0;

    virtual void set_resolution(uint32_t width, uint32_t height) = 0;
    virtual void set_fps(uint32_t fps) = 0;

    virtual BRX_MOTION_VIDEO_CAPTURE_TYPE get_type() const = 0;
    virtual uint32_t get_width() const = 0;
    virtual uint32_t get_height() const = 0;
    virtual uint32_t get_fps() const = 0;

    virtual brx_motion_video_frame const *read_video_frame(uint64_t frame_index) = 0;
};

extern "C" brx_motion_video_detector *brx_motion_create_video_detector(uint32_t face_count, uint32_t pose_count, bool force_gpu);

extern "C" void brx_motion_destory_video_detector(brx_motion_video_detector *video_detector);

class brx_motion_video_detector
{
public:
    virtual uint32_t get_face_count() const = 0;
    virtual uint32_t get_pose_count() const = 0;

    virtual bool get_enable_gpu() const = 0;

    virtual void set_enable_debug_renderer(bool enable_debug_renderer, char const *debug_renderer_window_name) = 0;

    virtual bool get_enable_debug_renderer() const = 0;

    virtual void step(brx_motion_video_frame const *video_frame, float delta_time) = 0;

    virtual void get_vrm_morph_target_weights(float(out_morph_target_weights)[BRX_MOTION_VRM_MORPH_TARGET_NAME_COUNT]) const = 0;
    virtual void get_arkit_morph_target_weights(float(out_morph_target_weights)[BRX_MOTION_ARKIT_MORPH_TARGET_NAME_COUNT]) const = 0;

    // consider openpose example?
    //

    // neck - head rotation

    // extern "C" void brx_motion_video_capture_instance_get_head_from_neck_rotation(uint32_t face_index, uint32_t pose_index, float (*out_head_ik_rotation)[4]);

    // face index defines the neck-head transform
    virtual void map_skeleton(brx_motion_skeleton *skeleton) const = 0;
};

extern "C" brx_motion_skeleton *brx_motion_create_skeleton(uint32_t skeleton_joint_count, char const *const *skeleton_joint_names, uint32_t const *skeleton_joint_parent_indices, brx_motion_skeleton_joint_transform const *skeleton_bind_pose_joint_transforms, uint32_t const vrm_skeleton_joint_indices[BRX_MOTION_VRM_SKELETON_JOINT_NAME_COUNT]);

extern "C" void brx_motion_destory_skeleton(brx_motion_skeleton *skeleton);

class brx_motion_skeleton
{
public:
    virtual uint32_t get_skeleton_joint_count() const = 0;
    virtual brx_motion_skeleton_joint_transform const* get_skeleton_bind_pose_joint_transforms() const = 0;
};

// When performing animation retargeting, only the model space transforms matter
//
// Interleaved
// Frame 0
//  Joint 0
//  Joint 1
//  Joint 2
// Frame 1
//  Joint 0
//  Joint 1
//  Joint 2
//
extern "C" brx_motion_skeleton_animation *brx_motion_create_skeleton_animation(uint32_t skeleton_joint_count, char const *const *skeleton_joint_names, uint32_t animation_frame_rate, uint32_t animation_frame_count, brx_motion_skeleton_joint_transform const *skeleton_animation_joint_transforms);

extern "C" void brx_motion_destory_skeleton_animation(brx_motion_skeleton_animation *skeleton_animation);

class brx_motion_skeleton_animation
{
};

extern "C" brx_motion_skeleton_animation_instance *brx_motion_create_skeleton_animation_instance(brx_motion_skeleton_animation *skeleton_animation, brx_motion_skeleton *skeleton);

extern "C" void brx_motion_destory_skeleton_animation_instance(brx_motion_skeleton_animation_instance* skeleton_animation_instance);

class brx_motion_skeleton_animation_instance
{
public:
    virtual void step(float delta_time) = 0;
    virtual brx_motion_skeleton_joint_transform const *get_skeleton_animation_joint_transforms() const = 0;
};

#endif
