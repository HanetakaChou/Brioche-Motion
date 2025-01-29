#ifndef _INTERNAL_TFLITE_H_
#define _INTERNAL_TFLITE_H_ 1

extern bool internal_tflite_check_gpu_support();

extern void internal_tflite_set_env_force_gpu(bool force_gpu);

extern void internal_tflite_unset_env_force_gpu();

#endif