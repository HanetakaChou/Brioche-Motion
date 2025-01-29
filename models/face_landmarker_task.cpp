#include <cstddef>
#include <cstdint>

static constexpr uint8_t const face_landmarker_task[] = {
#include "bin2h/_internal_face_landmarker_task.inl"
};

extern uint8_t const *const face_landmarker_task_base = face_landmarker_task;

extern size_t const face_landmarker_task_size = sizeof(face_landmarker_task);
