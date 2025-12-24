#include "face_landmarker_task.h"

static constexpr uint8_t const face_landmarker_task[] = {
#include "bin2h/_internal_face_landmarker_task.inl"
};

extern "C" void const *brx_motion_mediapipe_model_asset_get_face_landmarker_task_base()
{
    return face_landmarker_task;
}

extern "C" uint32_t brx_motion_mediapipe_model_asset_get_face_landmarker_task_size()
{
    return sizeof(face_landmarker_task);
}
