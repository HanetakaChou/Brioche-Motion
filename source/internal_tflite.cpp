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

#include "internal_tflite.h"
#include <CL/cl.h>
#include "../../McRT-Malloc/include/mcrt_vector.h"
#include <cassert>

#if defined(__GNUC__)
#include <dlfcn.h>
#elif defined(_MSC_VER)
#define NOMINMAX 1
#define WIN32_LEAN_AND_MEAN 1
#include <sdkddkver.h>
#include <Windows.h>
#else
#error Unknown Compiler
#endif

extern bool internal_tflite_check_gpu_support()
{
    // tensorflow\lite/delegates/gpu/cl/opencl_wrapper.cc
    // LoadOpenCLFunctions
    // LoadFunctionExtension
    //
    // tensorflow\lite/delegates/gpu/cl/cl_device.cc
    // CreateDefaultGPUDevice
    // LoadOpenCLFunctionExtensions

#if defined(__GNUC__)
    constexpr char const *const dynamic_library_name = "libOpenCL.so";

    void *dynamic_library = dlopen(dynamic_library_name, RTLD_NOW | RTLD_LOCAL);
#elif defined(_MSC_VER)
    constexpr wchar_t const *const dynamic_library_name = L"OpenCL.dll";

    HMODULE dynamic_library = GetModuleHandleW(dynamic_library_name);
    if (NULL == dynamic_library)
    {
        assert(ERROR_MOD_NOT_FOUND == GetLastError());

        dynamic_library = LoadLibraryW(dynamic_library_name);
        if (NULL == dynamic_library)
        {
            assert(ERROR_MOD_NOT_FOUND == GetLastError());
            return false;
        }
    }
#else
#error Unknown Compiler
#endif

    decltype(clGetPlatformIDs) *pfn_get_platform_ids = NULL;
    decltype(clGetDeviceIDs) *pfn_get_device_ids = NULL;
    decltype(clGetDeviceInfo) *pfn_get_device_info = NULL;
    decltype(clCreateContext) *pfn_create_context = NULL;
    decltype(clReleaseContext) *pfn_release_context = NULL;
    {
        constexpr char const *const function_names[] = {
            "clGetPlatformInfo",
            "clCreateSubDevices",
            "clRetainDevice",
            "clReleaseDevice",
            "clCreateContextFromType",
            "clRetainContext",
            "clGetContextInfo",
            "clCreateCommandQueueWithProperties",
            "clRetainCommandQueue",
            "clReleaseCommandQueue",
            "clGetCommandQueueInfo",
            "clCreateBuffer",
            "clCreateSubBuffer",
            "clCreateImage",
            "clCreatePipe",
            "clRetainMemObject",
            "clReleaseMemObject",
            "clGetSupportedImageFormats",
            "clGetMemObjectInfo",
            "clGetImageInfo",
            "clGetPipeInfo",
            "clSetMemObjectDestructorCallback",
            "clSVMAlloc",
            "clSVMFree",
            "clCreateSamplerWithProperties",
            "clRetainSampler",
            "clReleaseSampler",
            "clGetSamplerInfo",
            "clCreateProgramWithSource",
            "clCreateProgramWithBinary",
            "clCreateProgramWithBuiltInKernels",
            "clRetainProgram",
            "clReleaseProgram",
            "clBuildProgram",
            "clCompileProgram",
            "clLinkProgram",
            "clUnloadPlatformCompiler",
            "clGetProgramInfo",
            "clGetProgramBuildInfo",
            "clCreateKernel",
            "clCreateKernelsInProgram",
            "clRetainKernel",
            "clReleaseKernel",
            "clSetKernelArg",
            "clSetKernelArgSVMPointer",
            "clSetKernelExecInfo",
            "clGetKernelInfo",
            "clGetKernelArgInfo",
            "clGetKernelWorkGroupInfo",
            "clWaitForEvents",
            "clGetEventInfo",
            "clCreateUserEvent",
            "clRetainEvent",
            "clReleaseEvent",
            "clSetUserEventStatus",
            "clSetEventCallback",
            "clGetEventProfilingInfo",
            "clFlush",
            "clFinish",
            "clEnqueueReadBuffer",
            "clEnqueueReadBufferRect",
            "clEnqueueWriteBuffer",
            "clEnqueueWriteBufferRect",
            "clEnqueueFillBuffer",
            "clEnqueueCopyBuffer",
            "clEnqueueCopyBufferRect",
            "clEnqueueReadImage",
            "clEnqueueWriteImage",
            "clEnqueueFillImage",
            "clEnqueueCopyImage",
            "clEnqueueCopyImageToBuffer",
            "clEnqueueCopyBufferToImage",
            "clEnqueueMapBuffer",
            "clEnqueueMapImage",
            "clEnqueueUnmapMemObject",
            "clEnqueueMigrateMemObjects",
            "clEnqueueNDRangeKernel",
            "clEnqueueNativeKernel",
            "clEnqueueMarkerWithWaitList",
            "clEnqueueBarrierWithWaitList",
            "clEnqueueSVMFree",
            "clEnqueueSVMMemcpy",
            "clEnqueueSVMMemFill",
            "clEnqueueSVMMap",
            "clEnqueueSVMUnmap",
            "clGetExtensionFunctionAddressForPlatform",
            "clCreateImage2D",
            "clCreateImage3D",
            "clEnqueueMarker",
            "clEnqueueWaitForEvents",
            "clEnqueueBarrier",
            "clUnloadCompiler",
            "clGetExtensionFunctionAddress",
            "clCreateCommandQueue",
            "clCreateSampler",
            "clEnqueueTask",
        };

        for (size_t function_index = 0U; function_index < (sizeof(function_names) / sizeof(function_names[0])); ++function_index)
        {
            if (
#if defined(__GNUC__)
                NULL == dlsym(dynamic_library, function_names[function_index])
#elif defined(_MSC_VER)
                NULL == GetProcAddress(dynamic_library, function_names[function_index])
#else
#error Unknown Compiler
#endif
            )
            {
                return false;
            }
        }

        if (
#if defined(__GNUC__)
            NULL == (pfn_get_platform_ids = reinterpret_cast<decltype(clGetPlatformIDs) *>(dlsym(dynamic_library, "clGetPlatformIDs")))
#elif defined(_MSC_VER)
            NULL == (pfn_get_platform_ids = reinterpret_cast<decltype(clGetPlatformIDs) *>(GetProcAddress(dynamic_library, "clGetPlatformIDs")))
#else
#error Unknown Compiler
#endif
        )
        {
            return false;
        }

        if (
#if defined(__GNUC__)
            NULL == (pfn_get_device_ids = reinterpret_cast<decltype(clGetDeviceIDs) *>(dlsym(dynamic_library, "clGetDeviceIDs")))
#elif defined(_MSC_VER)
            NULL == (pfn_get_device_ids = reinterpret_cast<decltype(clGetDeviceIDs) *>(GetProcAddress(dynamic_library, "clGetDeviceIDs")))
#else
#error Unknown Compiler
#endif
        )
        {
            return false;
        }

        if (
#if defined(__GNUC__)
            NULL == (pfn_get_device_info = reinterpret_cast<decltype(clGetDeviceInfo) *>(dlsym(dynamic_library, "clGetDeviceInfo")))
#elif defined(_MSC_VER)
            NULL == (pfn_get_device_info = reinterpret_cast<decltype(clGetDeviceInfo) *>(GetProcAddress(dynamic_library, "clGetDeviceInfo")))
#else
#error Unknown Compiler
#endif
        )
        {
            return false;
        }

        if (
#if defined(__GNUC__)
            NULL == (pfn_create_context = reinterpret_cast<decltype(clCreateContext) *>(dlsym(dynamic_library, "clCreateContext")))
#elif defined(_MSC_VER)
            NULL == (pfn_create_context = reinterpret_cast<decltype(clCreateContext) *>(GetProcAddress(dynamic_library, "clCreateContext")))
#else
#error Unknown Compiler
#endif
        )
        {
            return false;
        }

        if (
#if defined(__GNUC__)
            NULL == (pfn_release_context = reinterpret_cast<decltype(clReleaseContext) *>(dlsym(dynamic_library, "clReleaseContext")))
#elif defined(_MSC_VER)
            NULL == (pfn_release_context = reinterpret_cast<decltype(clReleaseContext) *>(GetProcAddress(dynamic_library, "clReleaseContext")))
#else
#error Unknown Compiler
#endif
        )
        {
            return false;
        }
    }

    cl_platform_id selected_platform_id = NULL;
    cl_device_id selected_device_id = NULL;
    {
        cl_uint num_platforms;
        cl_int status_get_platform_id_1 = pfn_get_platform_ids(0, nullptr, &num_platforms);
        if (CL_SUCCESS != status_get_platform_id_1)
        {
            return false;
        }

        if (!(num_platforms > 0))
        {
            return false;
        }

        mcrt_vector<cl_platform_id> platforms(num_platforms);
        cl_int status_get_platform_id_2 = pfn_get_platform_ids(num_platforms, platforms.data(), nullptr);
        if (CL_SUCCESS != status_get_platform_id_2)
        {
            return false;
        }

        bool device_found = false;
        for (cl_uint platform_index = 0; platform_index < num_platforms; ++platform_index)
        {
            cl_platform_id platform_id = platforms[platform_index];

            cl_uint num_devices;
            cl_int status_get_device_id_1 = pfn_get_device_ids(platform_id, CL_DEVICE_TYPE_GPU, 0, nullptr, &num_devices);
            if (CL_SUCCESS == status_get_device_id_1)
            {
                if (num_devices > 0)
                {
                    std::vector<cl_device_id> devices(num_devices);
                    cl_int status_get_device_id_2 = pfn_get_device_ids(platform_id, CL_DEVICE_TYPE_GPU, num_devices, devices.data(), nullptr);
                    if (CL_SUCCESS == status_get_device_id_2)
                    {
                        for (cl_uint device_index = 0; device_index < num_devices; ++device_index)
                        {
                            cl_device_id device_id = devices[device_index];

                            cl_bool host_unified_memory;
                            cl_int status_get_device_host_unified_memory = pfn_get_device_info(device_id, CL_DEVICE_HOST_UNIFIED_MEMORY, sizeof(host_unified_memory), &host_unified_memory, nullptr);
                            if (CL_SUCCESS == status_get_device_host_unified_memory)
                            {
                                if (CL_FALSE == host_unified_memory)
                                {
                                    selected_platform_id = platform_id;
                                    selected_device_id = device_id;
                                    device_found = true;
                                    break;
                                }
                                else if (!device_found)
                                {
                                    selected_platform_id = platform_id;
                                    selected_device_id = device_id;
                                    device_found = true;
                                }
                            }
                            else
                            {
                                // WARNING: status_get_device_host_unified_memory
                            }
                        }
                    }
                    else
                    {
                        // WANRING: status_get_device_id_2
                    }
                }
                else
                {
                    // WARNING: No GPU on platform (platform_id)
                }
            }
            else
            {
                // WANRING: status_get_device_id_1
            }
        }

        if (!device_found)
        {
            return false;
        }
    }

    {
        cl_context_properties properties[] =
            {
                CL_CONTEXT_PLATFORM,
                reinterpret_cast<cl_context_properties>(selected_platform_id),
                0};

        cl_context context = NULL;
        cl_int status_create_context = CL_DEVICE_NOT_FOUND;
        context = pfn_create_context(properties, 1U, &selected_device_id, NULL, NULL, &status_create_context);
        if ((NULL == context) || (CL_SUCCESS != status_create_context))
        {
            return false;
        }

        cl_int status_release_context = pfn_release_context(context);
        assert(CL_SUCCESS == status_release_context);
    }

    return true;
}

extern void internal_tflite_set_env_force_gpu(bool force_gpu)
{
    if (force_gpu)
    {
#if defined(__GNUC__)

#if defined(__linux__)
        int status_set_env = setenv("TFLITE_FORCE_GPU", "1", 1);
        assert(0 == status_set_env);
#else
#error Unknown Platform
#endif

#elif defined(_MSC_VER)
        long status_set_env = SetEnvironmentVariableW(L"TFLITE_FORCE_GPU", L"1");
        assert(0 != status_set_env);
#else
#error Unknown Compiler
#endif
    }
    else
    {
#if defined(__GNUC__)

#if defined(__linux__)
        int status_set_env = setenv("TFLITE_FORCE_GPU", "0", 1);
        assert(0 == status_set_env);
#else
#error Unknown Platform
#endif

#elif defined(_MSC_VER)
        long status_set_env = SetEnvironmentVariableW(L"TFLITE_FORCE_GPU", L"0");
        assert(0 != status_set_env);
#else
#error Unknown Compiler
#endif
    }
}

extern void internal_tflite_unset_env_force_gpu()
{
#if defined(__GNUC__)

#if defined(__linux__)
    int status_unset_env = unsetenv("TFLITE_FORCE_GPU");
    assert(0 == status_unset_env);
#else
#error Unknown Platform
#endif

#elif defined(_MSC_VER)
    long status_unset_env = SetEnvironmentVariableW(L"TFLITE_FORCE_GPU", NULL);
    assert(0 != status_unset_env);
#else
#error Unknown Compiler
#endif
}
