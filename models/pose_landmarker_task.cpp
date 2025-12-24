#include "pose_landmarker_task.h"

static constexpr uint8_t const pose_landmarker_task[] = {
#include "bin2h/_internal_pose_landmarker_full_task.inl"
};

extern "C" void const * brx_motion_mediapipe_model_asset_get_pose_landmarker_task_base()
{
    return pose_landmarker_task;
}

extern "C" uint32_t brx_motion_mediapipe_model_asset_get_pose_landmarker_task_size()
{
    return sizeof(pose_landmarker_task);
} 
