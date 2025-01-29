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

#include <stddef.h>
#include <stdint.h>

enum BRX_MOTION_VIDEO_CAPTURE_TYPE
{
    BRX_MOTION_VIDEO_CAPTURE_TYPE_UNKNOWN = 0,
    BRX_MOTION_VIDEO_CAPTURE_TYPE_CAMERA = 1,
    BRX_MOTION_VIDEO_CAPTURE_TYPE_FILE = 2
};

// https://github.com/vrm-c/vrm-specification/blob/master/specification/0.0/README.md#predefined-expression-name
// https://github.com/vrm-c/vrm-specification/blob/master/specification/VRMC_vrm-1.0/expressions.md#preset-expressions
enum BRX_MOTION_VRM_MORPH_TARGET_NAME
{
    BRX_MOTION_VRM_MORPH_TARGET_NAME_NEUTRAL = 0,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_A = 1,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_I = 2,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_U = 3,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_E = 4,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_O = 5,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_BLINK = 6,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_BLINK_L = 7,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_BLINK_R = 8,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_FUN = 9,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_ANGRY = 10,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_SORROW = 11,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_JOY = 12,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_SURPRISED = 13,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_LOOK_UP = 14,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_LOOK_DOWN = 15,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_LOOK_LEFT = 16,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_LOOK_RIGHT = 17,
    BRX_MOTION_VRM_MORPH_TARGET_NAME_COUNT = 18
};

// https://developer.apple.com/documentation/arkit/arfaceanchor/blendshapelocation
static constexpr uint32_t const BRX_MOTION_ARKIT_MORPH_TARGET_NAME_COUNT = 52U;

// https://github.com/vrm-c/vrm-specification/blob/master/specification/0.0/README.md#defined-bones
// https://github.com/vrm-c/vrm-specification/blob/master/specification/VRMC_vrm-1.0/humanoid.md#list-of-humanoid-bones
// https://github.com/saturday06/VRM-Addon-for-Blender/blob/main/src/io_scene_vrm/common/human_bone_mapper/mmd_mapping.py
// https://docs.unity3d.com/ScriptReference/HumanBodyBones.html
// https://developer.apple.com/documentation/arkit/validating-a-model-for-motion-capture
// https://developer.apple.com/documentation/arkit/arskeleton/jointname
// https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker
// https://cmu-perceptual-computing-lab.github.io/openpose/web/html/doc/md_doc_02_output.html
enum BRX_MOTION_VRM_JOINT_NAME
{
    BRX_MOTION_VRM_JOINT_NAME_HIPS = 0,
    BRX_MOTION_VRM_JOINT_NAME_COUNT = 55,
};

class brx_motion_video_capture;
class brx_motion_video_frame;
class brx_motion_video_detector;
class brx_motion_skeleton;
class brx_motion_skeleton_animation;

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

extern "C" brx_motion_skeleton *brx_motion_create_skeleton(int32_t joint_count, char const *const *joint_names, int32_t const *parent_indices, int32_t const vrm_joint_indices[BRX_MOTION_VRM_JOINT_NAME_COUNT]);

class brx_motion_skeleton
{
public:
    virtual void brx_motion_skelton_get_pose(float (*out_joint_model_space_rotations)[4], float (*out_joint_model_space_translations)[3]) const = 0;
    virtual void destroy() = 0;
};

// When performing animation retargeting, only the model space transforms matter
//
// Interleaved Uncompressed Animation
// Frame 0
//  Joint 0
//  Joint 1
//  Joint 2
// Frame 1
//  Joint 0
//  Joint 1
//  Joint 2
extern "C" brx_motion_skeleton_animation *brx_motion_create_skeleton_animation(int32_t joint_count, char const *const *joint_names, uint32_t frame_count, float const (*joint_model_space_rotations)[4], float const (*joint_model_space_translations)[3]);

class brx_motion_skeleton_animation
{
public:
    // virtual void step(float delta_time) = 0;
    // virtual void map_skeleton(brx_motion_skeleton *skeleton) const = 0;
};

#endif
