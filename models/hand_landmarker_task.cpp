#include "hand_landmarker_task.h"

static constexpr uint8_t const hand_landmarker_task[] = {
#include "bin2h/_internal_hand_landmarker_task.inl"
};

extern "C" void const *brx_motion_mediapipe_model_asset_get_hand_landmarker_task_base()
{
    return hand_landmarker_task;
}

extern "C" uint32_t brx_motion_mediapipe_model_asset_get_hand_landmarker_task_size()
{
    return sizeof(hand_landmarker_task);
}
