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

#include "brx_motion_vmc_motion_receiver.h"
#if defined(USE_WSA)
#if USE_WSA
#include <mswsock.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#endif
#else
#error "0 or 1"
#endif
#include "../../McRT-Malloc/include/mcrt_malloc.h"
#include "../../McRT-Malloc/include/mcrt_vector.h"
#include <algorithm>
#include <bit>
#include <cstring>
#include <cassert>

// https://github.com/saturday06/VRM-Addon-for-Blender/blob/main/src/io_scene_vrm/common/human_bone_mapper/mmd_mapping.py
// https://github.com/vrm-c/vrm-specification/blob/master/specification/VRMC_vrm-1.0/humanoid.md#humanoid-bone-parent-child-relationship
constexpr uint32_t const internal_vmc_skeleton_joint_parent_indices[] = {
    BRX_MOTION_UINT32_INDEX_INVALID,
    INTERNAL_VMC_SKELETON_JOINT_NAME_CENTER,
    INTERNAL_VMC_SKELETON_JOINT_NAME_UPPER_BODY,
    INTERNAL_VMC_SKELETON_JOINT_NAME_UPPER_BODY_2,
    INTERNAL_VMC_SKELETON_JOINT_NAME_UPPER_BODY_3,
    INTERNAL_VMC_SKELETON_JOINT_NAME_NECK,
    INTERNAL_VMC_SKELETON_JOINT_NAME_HEAD,
    INTERNAL_VMC_SKELETON_JOINT_NAME_HEAD,
    INTERNAL_VMC_SKELETON_JOINT_NAME_CENTER,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_LEG,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_KNEE,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_ANKLE,
    INTERNAL_VMC_SKELETON_JOINT_NAME_CENTER,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_LEG,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_KNEE,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_ANKLE,
    INTERNAL_VMC_SKELETON_JOINT_NAME_UPPER_BODY_3,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_SHOULDER,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_ARM,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_ELBOW,
    INTERNAL_VMC_SKELETON_JOINT_NAME_UPPER_BODY_3,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_SHOULDER,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_ARM,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_ELBOW,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_WRIST,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_THUMB_0,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_THUMB_1,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_THUMB_2,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_WRIST,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_INDEX_FINGER_1,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_INDEX_FINGER_2,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_INDEX_FINGER_3,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_WRIST,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_MIDDLE_FINGER_1,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_MIDDLE_FINGER_2,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_MIDDLE_FINGER_3,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_WRIST,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_RING_FINGER_1,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_RING_FINGER_2,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_RING_FINGER_3,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_WRIST,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_LITTLE_FINGER_1,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_LITTLE_FINGER_2,
    INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_LITTLE_FINGER_3,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_WRIST,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_THUMB_0,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_THUMB_1,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_THUMB_2,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_WRIST,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_INDEX_FINGER_1,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_INDEX_FINGER_2,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_INDEX_FINGER_3,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_WRIST,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_MIDDLE_FINGER_1,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_MIDDLE_FINGER_2,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_MIDDLE_FINGER_3,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_WRIST,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_RING_FINGER_1,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_RING_FINGER_2,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_RING_FINGER_3,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_WRIST,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_LITTLE_FINGER_1,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_LITTLE_FINGER_2,
    INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_LITTLE_FINGER_3};

static_assert(INTERNAL_VMC_SKELETON_JOINT_NAME_COUNT == (sizeof(internal_vmc_skeleton_joint_parent_indices) / sizeof(internal_vmc_skeleton_joint_parent_indices[0])), "");

static constexpr float const INTERNAL_SCALE_EPSILON = 1E-4F;

#if defined(__GNUC__)
// GCC or CLANG
#define internal_unlikely(x) __builtin_expect(!!(x), 0)
#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__) && __ORDER_LITTLE_ENDIAN__ == __BYTE_ORDER__
static inline uint16_t internal_hton16(uint16_t x) { return __builtin_bswap16(x); }
static inline uint32_t internal_hton32(uint32_t x) { return __builtin_bswap32(x); }
static inline uint16_t internal_ntoh16(uint16_t x) { return __builtin_bswap16(x); }
static inline uint32_t internal_ntoh32(uint32_t x) { return __builtin_bswap32(x); }
#elif defined(__BYTE_ORDER__) && defined(__ORDER_BIG_ENDIAN__) && __ORDER_BIG_ENDIAN__ == __BYTE_ORDER__
static inline uint16_t internal_hton16(uint16_t x) { return x; }
static inline uint32_t internal_hton32(uint32_t x) { return x; }
static inline uint16_t internal_ntoh16(uint16_t x) { return x; }
static inline uint32_t internal_ntoh32(uint32_t x) { return x; }
#else
#error Unknown Byte Order
#endif
#elif defined(_MSC_VER)
#if defined(__clang__)
// CLANG-CL
#define internal_unlikely(x) __builtin_expect(!!(x), 0)
#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__) && __ORDER_LITTLE_ENDIAN__ == __BYTE_ORDER__
static inline uint16_t internal_hton16(uint16_t x) { return __builtin_bswap16(x); }
static inline uint32_t internal_hton32(uint32_t x) { return __builtin_bswap32(x); }
static inline uint16_t internal_ntoh16(uint16_t x) { return __builtin_bswap16(x); }
static inline uint32_t internal_ntoh32(uint32_t x) { return __builtin_bswap32(x); }
#elif defined(__BYTE_ORDER__) && defined(__ORDER_BIG_ENDIAN__) && __ORDER_BIG_ENDIAN__ == __BYTE_ORDER__
static inline uint16_t internal_hton16(uint16_t x) { return x; }
static inline uint32_t internal_hton32(uint32_t x) { return x; }
static inline uint16_t internal_ntoh16(uint16_t x) { return x; }
static inline uint32_t internal_ntoh32(uint32_t x) { return x; }
#else
#error Unknown Byte Order
#endif
#else
// MSVC
#define internal_unlikely(x) (!!(x))
// Assume Little Endian
static inline uint16_t internal_hton16(uint16_t x) { return _byteswap_ushort(x); }
static inline uint32_t internal_hton32(uint32_t x) { return _byteswap_ulong(x); }
static inline uint16_t internal_ntoh16(uint16_t x) { return _byteswap_ushort(x); }
static inline uint32_t internal_ntoh32(uint32_t x) { return _byteswap_ulong(x); }
#endif
#else
#error Unknown Compiler
#endif

constexpr uint32_t const INTERNAL_UDP_MAX_MSG_SIZE = 65507U;

static inline bool internal_data_read_osc_uint32(void const *data_base, uint32_t data_length, uint32_t &inout_data_offset, uint32_t *out_uint32);

static inline bool internal_data_read_osc_float(void const *data_base, uint32_t data_length, uint32_t &inout_data_offset, float *out_float);

static inline bool internal_data_read_osc_string(void const *data_base, uint32_t data_length, uint32_t &inout_data_offset, char const **out_string_base, uint32_t *out_string_length);

static inline DirectX::XMFLOAT3 internal_transform_translation(DirectX::XMFLOAT3 const &v);

static inline DirectX::XMFLOAT4 internal_transform_rotation(DirectX::XMFLOAT4 const &q);

static inline uint32_t internal_get_face_morph_joint_index(BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name);

static inline uint32_t internal_get_pose_skeleton_joint_index(BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name);

static inline uint32_t internal_get_face_skeleton_joint_index(BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name);

static inline uint32_t internal_get_hand_skeleton_joint_index(BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name);

extern "C" brx_motion_motion_receiver *brx_motion_create_motion_receiver(uint16_t port)
{
    void *new_unwrapped_motion_receiver_base = mcrt_malloc(sizeof(brx_motion_vmc_motion_receiver), alignof(brx_motion_vmc_motion_receiver));
    assert(NULL != new_unwrapped_motion_receiver_base);

    brx_motion_vmc_motion_receiver *new_unwrapped_motion_receiver = new (new_unwrapped_motion_receiver_base) brx_motion_vmc_motion_receiver{};
    if (new_unwrapped_motion_receiver->init(port))
    {
        return new_unwrapped_motion_receiver;
    }
    else
    {
        new_unwrapped_motion_receiver->~brx_motion_vmc_motion_receiver();
        mcrt_free(new_unwrapped_motion_receiver);
        return NULL;
    }
}

extern "C" void brx_motion_destroy_motion_receiver(brx_motion_motion_receiver *wrapped_motion_receiver)
{
    assert(NULL != wrapped_motion_receiver);
    internal_brx_motion_motion_capture *const release_unwrapped_motion_receiver = static_cast<internal_brx_motion_motion_capture *>(static_cast<brx_motion_vmc_motion_receiver *>(wrapped_motion_receiver));

    release_unwrapped_motion_receiver->release();
}

brx_motion_vmc_motion_receiver::brx_motion_vmc_motion_receiver()
    : m_ref_count(0U),
#if defined(USE_WSA)
#if USE_WSA
      m_socket_descriptor(INVALID_SOCKET)
#else
      m_socket_descriptor(-1)
#endif
#else
#error "0 or 1"
#endif
      ,
      m_vrmc_vrm(false),
      m_apply(false),
      m_morph_target_weights{},
      m_morph_joint_weights{},
      m_skeleton_joint_transforms_local_space{},
      m_skeleton_joint_transforms_model_space{}
{
}

brx_motion_vmc_motion_receiver::~brx_motion_vmc_motion_receiver()
{
#if defined(USE_WSA)
#if USE_WSA
    assert(INVALID_SOCKET == this->m_socket_descriptor);
#else
    assert(-1 == this->m_socket_descriptor);
#endif
#else
#error "0 or 1"
#endif
}

bool brx_motion_vmc_motion_receiver::init(uint16_t port)
{
    assert(0U == this->m_ref_count);
    this->m_ref_count = 1U;

    bool has_error = false;

#if defined(USE_WSA)
#if USE_WSA
    if (!has_error)
    {
        assert(INVALID_SOCKET == this->m_socket_descriptor);
        this->m_socket_descriptor = WSASocketW(AF_INET, SOCK_DGRAM, IPPROTO_UDP, NULL, 0U, 0U);
        if (INVALID_SOCKET != this->m_socket_descriptor)
        {
            // Do Nothing
        }
        else
        {
            assert(!has_error);
            has_error = true;
        }
    }

    if (!has_error)
    {
        u_long mode = 1U;
        DWORD bytes_returned = 0U;
        int res_ioctl = WSAIoctl(this->m_socket_descriptor, FIONBIO, &mode, sizeof(mode), NULL, 0U, &bytes_returned, NULL, NULL);
        if (0 == res_ioctl)
        {
            assert(0U == bytes_returned);
        }
        else
        {
            assert(!has_error);
            has_error = true;
        }
    }

    if (!has_error)
    {
        BOOL behavior = FALSE;
        DWORD bytes_returned = 0U;
        int res_ioctl = WSAIoctl(this->m_socket_descriptor, SIO_UDP_CONNRESET, &behavior, sizeof(behavior), NULL, 0U, &bytes_returned, NULL, NULL);
        assert(0 == res_ioctl);
        assert(0U == bytes_returned);

        if (0 == res_ioctl)
        {
            assert(0U == bytes_returned);
        }
        else
        {
            assert(!has_error);
            has_error = true;
        }
    }
#else
    if (!has_error)
    {
        assert(-1 == this->m_socket_descriptor);
        this->m_socket_descriptor = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (-1 != this->m_socket_descriptor)
        {
            // Do Nothing
        }
        else
        {
            assert(!has_error);
            has_error = true;
        }
    }

    int socket_descriptor_old_flags = -1;
    if (!has_error)
    {
        socket_descriptor_old_flags = fcntl(this->m_socket_descriptor, F_GETFL, 0);
        if (-1 != socket_descriptor_old_flags)
        {
            // Do Nothing
        }
        else
        {
            assert(!has_error);
            has_error = true;
        }
    }

    if (!has_error)
    {
        int res_set_flags = fcntl(this->m_socket_descriptor, F_SETFL, socket_descriptor_old_flags | O_NONBLOCK);
        if (0 == res_set_flags)
        {
            // Do Nothing
        }
        else
        {
            assert(!has_error);
            has_error = true;
        }
    }
#endif
#else
#error "0 or 1"
#endif

    if (!has_error)
    {
        // TODO: GetAddrInfoW/getaddrinfo

        struct sockaddr_in name;
        std::memset(&name, 0, sizeof(struct sockaddr_in));
        name.sin_family = AF_INET;
        name.sin_port = internal_hton16(port);
        name.sin_addr.s_addr = INADDR_ANY;

        int res_bind = bind(this->m_socket_descriptor, reinterpret_cast<struct sockaddr *>(&name), sizeof(struct sockaddr_in));
        if (0 == res_bind)
        {
            // Do Nothing
        }
        else
        {
            assert(!has_error);
            has_error = true;
        }
    }

    // https://en.wikipedia.org/wiki/User_Datagram_Protocol#UDP_datagram_structure
#ifndef NDEBUG
#if defined(USE_WSA)
#if USE_WSA
    {
        DWORD max_msg_size = 0U;
        char *opt_var = reinterpret_cast<char *>(&max_msg_size);
        int opt_len = sizeof(DWORD);
        int res_get_sock_opt = getsockopt(this->m_socket_descriptor, SOL_SOCKET, SO_MAX_MSG_SIZE, opt_var, &opt_len);
        assert(0 == res_get_sock_opt);
        assert(INTERNAL_UDP_MAX_MSG_SIZE == max_msg_size);
    }
#else
// Do Nothing
#endif
#else
#error "0 or 1"
#endif
#endif

    if (!has_error)
    {
        {
            DirectX::XMStoreFloat4(&this->m_model_transform.m_rotation, DirectX::XMQuaternionIdentity());
            DirectX::XMStoreFloat3(&this->m_model_transform.m_translation, DirectX::XMVectorZero());
        }

        for (uint32_t morph_target_name_index = 0U; morph_target_name_index < BRX_MOTION_MORPH_TARGET_NAME_MMD_COUNT; ++morph_target_name_index)
        {
            this->m_morph_target_weights[morph_target_name_index] = 0.0F;
        }

        for (uint32_t morph_joint_name_index = 0U; morph_joint_name_index < INTERNAL_VMC_MORPH_JOINT_NAME_COUNT; ++morph_joint_name_index)
        {
            DirectX::XMStoreFloat4(&this->m_morph_joint_weights[morph_joint_name_index], DirectX::XMVectorZero());
        }

        for (uint32_t skeleton_joint_name_index = 0U; skeleton_joint_name_index < INTERNAL_VMC_SKELETON_JOINT_NAME_COUNT; ++skeleton_joint_name_index)
        {
            DirectX::XMStoreFloat4(&this->m_skeleton_joint_transforms_local_space[skeleton_joint_name_index].m_rotation, DirectX::XMQuaternionIdentity());
            DirectX::XMStoreFloat3(&this->m_skeleton_joint_transforms_local_space[skeleton_joint_name_index].m_translation, DirectX::XMVectorZero());
        }

        for (uint32_t skeleton_joint_name_index = 0U; skeleton_joint_name_index < INTERNAL_VMC_SKELETON_JOINT_NAME_COUNT; ++skeleton_joint_name_index)
        {
            DirectX::XMStoreFloat4(&this->m_skeleton_joint_transforms_model_space[skeleton_joint_name_index].m_rotation, DirectX::XMQuaternionIdentity());
            DirectX::XMStoreFloat3(&this->m_skeleton_joint_transforms_model_space[skeleton_joint_name_index].m_translation, DirectX::XMVectorZero());
        }

        return true;
    }
    else
    {
#if defined(USE_WSA)
#if USE_WSA
        if (INVALID_SOCKET != this->m_socket_descriptor)
        {
            int res_close_socket = closesocket(this->m_socket_descriptor);
            assert(0 == res_close_socket);
            this->m_socket_descriptor = INVALID_SOCKET;
        }
#else
        if (-1 != this->m_socket_descriptor)
        {
            int res_close = close(this->m_socket_descriptor);
            assert(0 == res_close);
            this->m_socket_descriptor = -1;
        }
#endif
#else
#error "0 or 1"
#endif

        assert(1U == this->m_ref_count);
        this->m_ref_count = 0U;

        return false;
    }
}

void brx_motion_vmc_motion_receiver::uninit()
{
    assert(0U == this->m_ref_count);

#if defined(USE_WSA)
#if USE_WSA
    assert(INVALID_SOCKET != this->m_socket_descriptor);
    int res_close_socket = closesocket(this->m_socket_descriptor);
    assert(0 == res_close_socket);
    this->m_socket_descriptor = INVALID_SOCKET;
#else
    assert(-1 != this->m_socket_descriptor);
    int res_close = close(this->m_socket_descriptor);
    assert(0 == res_close);
    this->m_socket_descriptor = -1;
#endif
#else
#error "0 or 1"
#endif
}

inline void brx_motion_vmc_motion_receiver::internal_retain()
{
    assert(this->m_ref_count > 0U);
    assert(this->m_ref_count < static_cast<uint32_t>(UINT32_MAX));
    ++this->m_ref_count;
}

inline uint32_t brx_motion_vmc_motion_receiver::internal_release()
{
    assert(this->m_ref_count > 0U);
    --this->m_ref_count;
    return this->m_ref_count;
}

void brx_motion_vmc_motion_receiver::retain()
{
    brx_motion_vmc_motion_receiver *const retain_unwrapped_motion_receiver = this;

    retain_unwrapped_motion_receiver->internal_retain();
}

void brx_motion_vmc_motion_receiver::release()
{
    brx_motion_vmc_motion_receiver *const release_unwrapped_motion_receiver = this;

    if (0U == release_unwrapped_motion_receiver->internal_release())
    {
        brx_motion_vmc_motion_receiver *delete_unwrapped_motion_receiver = release_unwrapped_motion_receiver;

        delete_unwrapped_motion_receiver->uninit();

        delete_unwrapped_motion_receiver->~brx_motion_vmc_motion_receiver();
        mcrt_free(delete_unwrapped_motion_receiver);
    }
}

brx_motion_video_detector *brx_motion_vmc_motion_receiver::get_video_detector()
{
    return NULL;
}

brx_motion_motion_receiver *brx_motion_vmc_motion_receiver::get_motion_receiver()
{
    return static_cast<brx_motion_motion_receiver *>(this);
}

brx_motion_motion_capture *brx_motion_vmc_motion_receiver::get_motion_capture()
{
    return static_cast<internal_brx_motion_motion_capture *>(this);
}

uint32_t brx_motion_vmc_motion_receiver::get_hand_count() const
{
    return 1U;
}

uint32_t brx_motion_vmc_motion_receiver::get_face_count() const
{
    return 1U;
}

uint32_t brx_motion_vmc_motion_receiver::get_pose_count() const
{
    return 1U;
}

void brx_motion_vmc_motion_receiver::step()
{
    // "PeekMessage" Style
    {
        mcrt_vector<uint8_t> osc_packet_data(static_cast<size_t>(INTERNAL_UDP_MAX_MSG_SIZE));
#if defined(USE_WSA)
#if USE_WSA
        WSABUF unused_buffers[1] = {{static_cast<ULONG>(sizeof(uint8_t) * osc_packet_data.size()), reinterpret_cast<CHAR *>(osc_packet_data.data())}};
        DWORD unused_number_of_bytes_recvd = 0U;
        DWORD unused_flags = 0U;
        int unused_res_recv;
        while (0 == (unused_res_recv = WSARecv(this->m_socket_descriptor, unused_buffers, 1U, &unused_number_of_bytes_recvd, &unused_flags, NULL, NULL)))
        {
            assert(unused_number_of_bytes_recvd <= INTERNAL_UDP_MAX_MSG_SIZE);
            uint32_t osc_packet_length = std::min(unused_number_of_bytes_recvd, static_cast<DWORD>(INTERNAL_UDP_MAX_MSG_SIZE));
            unused_number_of_bytes_recvd = 0U;
            unused_flags = 0U;
#else
        ssize_t unused_res_recv;
        while (-1 != (unused_res_recv = recv(this->m_socket_descriptor, reinterpret_cast<char *>(osc_packet_data.data()), sizeof(uint8_t) * osc_packet_data.size(), 0)))
        {
            assert(unused_res_recv <= INTERNAL_UDP_MAX_MSG_SIZE);
            uint32_t osc_packet_length = std::min(unused_res_recv, static_cast<ssize_t>(INTERNAL_UDP_MAX_MSG_SIZE));
#endif
#else
#error "0 or 1"
#endif

            struct osc_packet_data_range
            {
                void const *const m_data_base;
                uint32_t const m_data_length;
            };

            mcrt_vector<osc_packet_data_range> osc_packet_depth_first_search_stack;
            osc_packet_depth_first_search_stack.push_back(osc_packet_data_range{osc_packet_data.data(), osc_packet_length});

            while (!osc_packet_depth_first_search_stack.empty())
            {
                osc_packet_data_range osc_packet_data_range_current = osc_packet_depth_first_search_stack.back();
                osc_packet_depth_first_search_stack.pop_back();

                if ((NULL != osc_packet_data_range_current.m_data_base) && (8U <= osc_packet_data_range_current.m_data_length) && (0 == std::memcmp("#bundle", osc_packet_data_range_current.m_data_base, 8U)))
                {
                    // OSC Bundle

                    // #bundle\0"   8
                    // timetag      8
                    uint32_t osc_packet_data_offset_current = 8U + 8U;

                    mcrt_vector<osc_packet_data_range> osc_packet_children;

                    while (osc_packet_data_offset_current < osc_packet_data_range_current.m_data_length)
                    {
                        uint32_t bundle_element_size;
                        if (internal_data_read_osc_uint32(osc_packet_data_range_current.m_data_base, osc_packet_data_range_current.m_data_length, osc_packet_data_offset_current, &bundle_element_size))
                        {
                            if (bundle_element_size > 0U)
                            {
                                if ((osc_packet_data_offset_current + bundle_element_size) <= osc_packet_data_range_current.m_data_length)
                                {
                                    osc_packet_children.push_back(osc_packet_data_range{reinterpret_cast<void const *>(reinterpret_cast<uintptr_t>(osc_packet_data_range_current.m_data_base) + osc_packet_data_offset_current), bundle_element_size});
                                    osc_packet_data_offset_current += bundle_element_size;
                                }
                                else
                                {
                                    assert(false);
                                    break;
                                }
                            }
                            else
                            {
                                // zero bundle element size would infinite loop.
                                assert(false);
                                break;
                            }
                        }
                        else
                        {
                            assert(false);
                            break;
                        }
                    }

                    assert(osc_packet_data_range_current.m_data_length == osc_packet_data_offset_current);

                    for (uint32_t osc_packet_child_index_plus_1 = static_cast<uint32_t>(osc_packet_children.size()); osc_packet_child_index_plus_1 > 0U; --osc_packet_child_index_plus_1)
                    {
                        uint32_t const osc_packet_child_index = osc_packet_child_index_plus_1 - 1U;
                        osc_packet_depth_first_search_stack.push_back(osc_packet_children[osc_packet_child_index]);
                    }
                }
                else if ((NULL != osc_packet_data_range_current.m_data_base) && (1U <= osc_packet_data_range_current.m_data_length) && ('/' == reinterpret_cast<char const *>(osc_packet_data_range_current.m_data_base)[0]))
                {
                    // OSC Message

                    void const *const osc_message_data_base = osc_packet_data_range_current.m_data_base;
                    uint32_t const osc_message_data_length = osc_packet_data_range_current.m_data_length;

                    uint32_t osc_message_data_offset_current = 0U;

                    char const *address_pattern_string_base = NULL;
                    uint32_t address_pattern_string_length = 0U;
                    if (internal_unlikely(!internal_data_read_osc_string(osc_message_data_base, osc_message_data_length, osc_message_data_offset_current, &address_pattern_string_base, &address_pattern_string_length)))
                    {
                        assert(false);
                        return;
                    }

                    char const *type_tag_string_base = NULL;
                    uint32_t type_tag_string_length = 0U;
                    if (internal_unlikely(!internal_data_read_osc_string(osc_message_data_base, osc_message_data_length, osc_message_data_offset_current, &type_tag_string_base, &type_tag_string_length)))
                    {
                        assert(false);
                        return;
                    }

                    // https://protocol.vmc.info/marionette-spec

                    if ((17U == address_pattern_string_length) && (0 == std::memcmp("/VMC/Ext/Root/Pos", address_pattern_string_base, address_pattern_string_length)))
                    {
                        if ((9U == type_tag_string_length) && (0 == std::memcmp(",sfffffff", type_tag_string_base, type_tag_string_length)))
                        {
                            // 2.0

                            char const *name_string_base = NULL;
                            uint32_t name_string_length = 0U;
                            if (internal_unlikely(!internal_data_read_osc_string(osc_message_data_base, osc_message_data_length, osc_message_data_offset_current, &name_string_base, &name_string_length)))
                            {
                                assert(false);
                                return;
                            }

                            float p_x = 0.0F;
                            if (internal_unlikely(!internal_data_read_osc_float(osc_message_data_base, osc_message_data_length, osc_message_data_offset_current, &p_x)))
                            {
                                assert(false);
                                return;
                            }

                            float p_y = 0.0F;
                            if (internal_unlikely(!internal_data_read_osc_float(osc_message_data_base, osc_message_data_length, osc_message_data_offset_current, &p_y)))
                            {
                                assert(false);
                                return;
                            }

                            float p_z = 0.0F;
                            if (internal_unlikely(!internal_data_read_osc_float(osc_message_data_base, osc_message_data_length, osc_message_data_offset_current, &p_z)))
                            {
                                assert(false);
                                return;
                            }

                            float q_x = 0.0F;
                            if (internal_unlikely(!internal_data_read_osc_float(osc_message_data_base, osc_message_data_length, osc_message_data_offset_current, &q_x)))
                            {
                                assert(false);
                                return;
                            }

                            float q_y = 0.0F;
                            if (internal_unlikely(!internal_data_read_osc_float(osc_message_data_base, osc_message_data_length, osc_message_data_offset_current, &q_y)))
                            {
                                assert(false);
                                return;
                            }

                            float q_z = 0.0F;
                            if (internal_unlikely(!internal_data_read_osc_float(osc_message_data_base, osc_message_data_length, osc_message_data_offset_current, &q_z)))
                            {
                                assert(false);
                                return;
                            }

                            float q_w = 1.0F;
                            if (internal_unlikely(!internal_data_read_osc_float(osc_message_data_base, osc_message_data_length, osc_message_data_offset_current, &q_w)))
                            {
                                assert(false);
                                return;
                            }

                            DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                            DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                            DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                            DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                            this->m_model_transform = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                        }
                        else if ((15U == type_tag_string_length) && (0 == std::memcmp(",sfffffffffffff", type_tag_string_base, type_tag_string_length)))
                        {
                            // 2.1

                            char const *name_string_base = NULL;
                            uint32_t name_string_length = 0U;
                            if (internal_unlikely(!internal_data_read_osc_string(osc_message_data_base, osc_message_data_length, osc_message_data_offset_current, &name_string_base, &name_string_length)))
                            {
                                assert(false);
                                return;
                            }

                            float p_x = 0.0F;
                            if (internal_unlikely(!internal_data_read_osc_float(osc_message_data_base, osc_message_data_length, osc_message_data_offset_current, &p_x)))
                            {
                                assert(false);
                                return;
                            }

                            float p_y = 0.0F;
                            if (internal_unlikely(!internal_data_read_osc_float(osc_message_data_base, osc_message_data_length, osc_message_data_offset_current, &p_y)))
                            {
                                assert(false);
                                return;
                            }

                            float p_z = 0.0F;
                            if (internal_unlikely(!internal_data_read_osc_float(osc_message_data_base, osc_message_data_length, osc_message_data_offset_current, &p_z)))
                            {
                                assert(false);
                                return;
                            }

                            float q_x = 0.0F;
                            if (internal_unlikely(!internal_data_read_osc_float(osc_message_data_base, osc_message_data_length, osc_message_data_offset_current, &q_x)))
                            {
                                assert(false);
                                return;
                            }

                            float q_y = 0.0F;
                            if (internal_unlikely(!internal_data_read_osc_float(osc_message_data_base, osc_message_data_length, osc_message_data_offset_current, &q_y)))
                            {
                                assert(false);
                                return;
                            }

                            float q_z = 0.0F;
                            if (internal_unlikely(!internal_data_read_osc_float(osc_message_data_base, osc_message_data_length, osc_message_data_offset_current, &q_z)))
                            {
                                assert(false);
                                return;
                            }

                            float q_w = 1.0F;
                            if (internal_unlikely(!internal_data_read_osc_float(osc_message_data_base, osc_message_data_length, osc_message_data_offset_current, &q_w)))
                            {
                                assert(false);
                                return;
                            }

                            DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                            DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                            DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                            DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                            this->m_model_transform = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                        }
                        else
                        {
                            assert(false);
                        }
                    }
                    else if ((17U == address_pattern_string_length) && (0 == std::memcmp("/VMC/Ext/Bone/Pos", address_pattern_string_base, address_pattern_string_length)))
                    {
                        // NOTE: UDP not reliable
                        this->m_apply = true;

                        if ((9U == type_tag_string_length) && (0 == std::memcmp(",sfffffff", type_tag_string_base, type_tag_string_length)))
                        {
                            char const *name_string_base = NULL;
                            uint32_t name_string_length = 0U;
                            if (internal_unlikely(!internal_data_read_osc_string(osc_message_data_base, osc_message_data_length, osc_message_data_offset_current, &name_string_base, &name_string_length)))
                            {
                                assert(false);
                                return;
                            }

                            float p_x = 0.0F;
                            if (internal_unlikely(!internal_data_read_osc_float(osc_message_data_base, osc_message_data_length, osc_message_data_offset_current, &p_x)))
                            {
                                assert(false);
                                return;
                            }

                            float p_y = 0.0F;
                            if (internal_unlikely(!internal_data_read_osc_float(osc_message_data_base, osc_message_data_length, osc_message_data_offset_current, &p_y)))
                            {
                                assert(false);
                                return;
                            }

                            float p_z = 0.0F;
                            if (internal_unlikely(!internal_data_read_osc_float(osc_message_data_base, osc_message_data_length, osc_message_data_offset_current, &p_z)))
                            {
                                assert(false);
                                return;
                            }

                            float q_x = 0.0F;
                            if (internal_unlikely(!internal_data_read_osc_float(osc_message_data_base, osc_message_data_length, osc_message_data_offset_current, &q_x)))
                            {
                                assert(false);
                                return;
                            }

                            float q_y = 0.0F;
                            if (internal_unlikely(!internal_data_read_osc_float(osc_message_data_base, osc_message_data_length, osc_message_data_offset_current, &q_y)))
                            {
                                assert(false);
                                return;
                            }

                            float q_z = 0.0F;
                            if (internal_unlikely(!internal_data_read_osc_float(osc_message_data_base, osc_message_data_length, osc_message_data_offset_current, &q_z)))
                            {
                                assert(false);
                                return;
                            }

                            float q_w = 1.0F;
                            if (internal_unlikely(!internal_data_read_osc_float(osc_message_data_base, osc_message_data_length, osc_message_data_offset_current, &q_w)))
                            {
                                assert(false);
                                return;
                            }

                            // https://github.com/saturday06/VRM-Addon-for-Blender/blob/main/src/io_scene_vrm/common/human_bone_mapper/mmd_mapping.py
                            // https://github.com/vrm-c/vrm-specification/blob/master/specification/VRMC_vrm-1.0/humanoid.md#humanoid-bone-parent-child-relationship

                            if ((4U == name_string_length) && (0 == std::memcmp("Hips", name_string_base, name_string_length) || (0 == std::memcmp("hips", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_CENTER] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((5U == name_string_length) && (0 == std::memcmp("Spine", name_string_base, name_string_length) || (0 == std::memcmp("spine", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_UPPER_BODY] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((5U == name_string_length) && (0 == std::memcmp("Chest", name_string_base, name_string_length) || (0 == std::memcmp("chest", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_UPPER_BODY_2] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((10U == name_string_length) && (0 == std::memcmp("UpperChest", name_string_base, name_string_length) || (0 == std::memcmp("upperChest", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_UPPER_BODY_3] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((4U == name_string_length) && (0 == std::memcmp("Neck", name_string_base, name_string_length) || (0 == std::memcmp("neck", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_NECK] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((4U == name_string_length) && (0 == std::memcmp("Head", name_string_base, name_string_length) || (0 == std::memcmp("head", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_HEAD] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((7U == name_string_length) && (0 == std::memcmp("LeftEye", name_string_base, name_string_length) || (0 == std::memcmp("leftEye", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_EYE] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((8U == name_string_length) && (0 == std::memcmp("RightEye", name_string_base, name_string_length) || (0 == std::memcmp("rightEye", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_EYE] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((3U == name_string_length) && (0 == std::memcmp("Jaw", name_string_base, name_string_length) || (0 == std::memcmp("jaw", name_string_base, name_string_length))))
                            {
                                // Do Nothing
                            }
                            else if ((12U == name_string_length) && (0 == std::memcmp("LeftUpperLeg", name_string_base, name_string_length) || (0 == std::memcmp("leftUpperLeg", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_LEG] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((12U == name_string_length) && (0 == std::memcmp("LeftLowerLeg", name_string_base, name_string_length) || (0 == std::memcmp("leftLowerLeg", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_KNEE] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((8U == name_string_length) && (0 == std::memcmp("LeftFoot", name_string_base, name_string_length) || (0 == std::memcmp("leftFoot", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_ANKLE] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((8U == name_string_length) && (0 == std::memcmp("LeftToes", name_string_base, name_string_length) || (0 == std::memcmp("leftToes", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_TOE_TIP] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((13U == name_string_length) && (0 == std::memcmp("RightUpperLeg", name_string_base, name_string_length) || (0 == std::memcmp("rightUpperLeg", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_LEG] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((13U == name_string_length) && (0 == std::memcmp("RightLowerLeg", name_string_base, name_string_length) || (0 == std::memcmp("rightLowerLeg", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_KNEE] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((9U == name_string_length) && (0 == std::memcmp("RightFoot", name_string_base, name_string_length) || (0 == std::memcmp("rightFoot", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_ANKLE] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((9U == name_string_length) && (0 == std::memcmp("RightToes", name_string_base, name_string_length) || (0 == std::memcmp("rightToes", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_TOE_TIP] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((12U == name_string_length) && (0 == std::memcmp("LeftShoulder", name_string_base, name_string_length) || (0 == std::memcmp("leftShoulder", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_SHOULDER] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((12U == name_string_length) && (0 == std::memcmp("LeftUpperArm", name_string_base, name_string_length) || (0 == std::memcmp("leftUpperArm", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_ARM] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((12U == name_string_length) && (0 == std::memcmp("LeftLowerArm", name_string_base, name_string_length) || (0 == std::memcmp("leftLowerArm", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_ELBOW] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((8U == name_string_length) && (0 == std::memcmp("LeftHand", name_string_base, name_string_length) || (0 == std::memcmp("leftHand", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_WRIST] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((13U == name_string_length) && (0 == std::memcmp("RightShoulder", name_string_base, name_string_length) || (0 == std::memcmp("rightShoulder", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_SHOULDER] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((13U == name_string_length) && (0 == std::memcmp("RightUpperArm", name_string_base, name_string_length) || (0 == std::memcmp("rightUpperArm", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_ARM] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((13U == name_string_length) && (0 == std::memcmp("RightLowerArm", name_string_base, name_string_length) || (0 == std::memcmp("rightLowerArm", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_ELBOW] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((9U == name_string_length) && (0 == std::memcmp("RightHand", name_string_base, name_string_length) || (0 == std::memcmp("rightHand", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_WRIST] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((19U == name_string_length) && (0 == std::memcmp("LeftThumbMetacarpal", name_string_base, name_string_length) || (0 == std::memcmp("leftThumbMetacarpal", name_string_base, name_string_length))))
                            {
                                if (this->m_vrmc_vrm)
                                {
                                    DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                    DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                    DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                    DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                    this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_THUMB_0] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                                }
                                else
                                {
                                    assert(false);
                                }
                            }
                            else if ((17U == name_string_length) && (0 == std::memcmp("LeftThumbProximal", name_string_base, name_string_length) || (0 == std::memcmp("leftThumbProximal", name_string_base, name_string_length))))
                            {
                                if (this->m_vrmc_vrm)
                                {
                                    DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                    DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                    DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                    DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                    this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_THUMB_1] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                                }
                                else
                                {
                                    DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                    DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                    DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                    DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                    this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_THUMB_0] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                                }
                            }
                            else if ((21U == name_string_length) && (0 == std::memcmp("LeftThumbIntermediate", name_string_base, name_string_length) || (0 == std::memcmp("leftThumbIntermediate", name_string_base, name_string_length))))
                            {
                                if (this->m_vrmc_vrm)
                                {
                                    assert(false);
                                }
                                else
                                {
                                    DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                    DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                    DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                    DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                    this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_THUMB_1] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                                }
                            }
                            else if ((15U == name_string_length) && (0 == std::memcmp("LeftThumbDistal", name_string_base, name_string_length) || (0 == std::memcmp("leftThumbDistal", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_THUMB_2] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((17U == name_string_length) && (0 == std::memcmp("LeftIndexProximal", name_string_base, name_string_length) || (0 == std::memcmp("leftIndexProximal", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_INDEX_FINGER_1] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((21U == name_string_length) && (0 == std::memcmp("LeftIndexIntermediate", name_string_base, name_string_length) || (0 == std::memcmp("leftIndexIntermediate", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_INDEX_FINGER_2] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((15U == name_string_length) && (0 == std::memcmp("LeftIndexDistal", name_string_base, name_string_length) || (0 == std::memcmp("leftIndexDistal", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_INDEX_FINGER_3] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((18U == name_string_length) && (0 == std::memcmp("LeftMiddleProximal", name_string_base, name_string_length) || (0 == std::memcmp("leftMiddleProximal", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_MIDDLE_FINGER_1] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((22U == name_string_length) && (0 == std::memcmp("LeftMiddleIntermediate", name_string_base, name_string_length) || (0 == std::memcmp("leftMiddleIntermediate", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_MIDDLE_FINGER_2] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((16U == name_string_length) && (0 == std::memcmp("LeftMiddleDistal", name_string_base, name_string_length) || (0 == std::memcmp("leftMiddleDistal", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_MIDDLE_FINGER_3] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((16U == name_string_length) && (0 == std::memcmp("LeftRingProximal", name_string_base, name_string_length) || (0 == std::memcmp("leftRingProximal", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_RING_FINGER_1] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((20U == name_string_length) && (0 == std::memcmp("LeftRingIntermediate", name_string_base, name_string_length) || (0 == std::memcmp("leftRingIntermediate", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_RING_FINGER_2] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((14U == name_string_length) && (0 == std::memcmp("LeftRingDistal", name_string_base, name_string_length) || (0 == std::memcmp("leftRingDistal", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_RING_FINGER_3] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if (((18U == name_string_length) && (0 == std::memcmp("LeftLittleProximal", name_string_base, name_string_length) || (0 == std::memcmp("leftLittleProximal", name_string_base, name_string_length)))) || ((17U == name_string_length) && ((0 == std::memcmp("LeftPinkyProximal", name_string_base, name_string_length)) || (0 == std::memcmp("leftPinkyProximal", name_string_base, name_string_length)))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_LITTLE_FINGER_1] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if (((22U == name_string_length) && (0 == std::memcmp("LeftLittleIntermediate", name_string_base, name_string_length) || (0 == std::memcmp("leftLittleIntermediate", name_string_base, name_string_length)))) || ((21U == name_string_length) && ((0 == std::memcmp("LeftPinkyIntermediate", name_string_base, name_string_length)) || (0 == std::memcmp("leftPinkyIntermediate", name_string_base, name_string_length)))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_LITTLE_FINGER_2] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if (((16U == name_string_length) && ((0 == std::memcmp("LeftLittleDistal", name_string_base, name_string_length)) || (0 == std::memcmp("leftLittleDistal", name_string_base, name_string_length)))) || ((15U == name_string_length) && ((0 == std::memcmp("LeftPinkyDistal", name_string_base, name_string_length)) || (0 == std::memcmp("leftPinkyDistal", name_string_base, name_string_length)))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_LITTLE_FINGER_3] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((20U == name_string_length) && (0 == std::memcmp("RightThumbMetacarpal", name_string_base, name_string_length) || (0 == std::memcmp("rightThumbMetacarpal", name_string_base, name_string_length))))
                            {
                                if (this->m_vrmc_vrm)
                                {
                                    DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                    DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                    DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                    DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                    this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_THUMB_0] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                                }
                                else
                                {
                                    assert(false);
                                }
                            }
                            else if ((18U == name_string_length) && ((0 == std::memcmp("RightThumbProximal", name_string_base, name_string_length)) || (0 == std::memcmp("rightThumbProximal", name_string_base, name_string_length))))
                            {
                                if (this->m_vrmc_vrm)
                                {
                                    DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                    DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                    DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                    DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                    this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_THUMB_1] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                                }
                                else
                                {
                                    DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                    DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                    DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                    DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                    this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_THUMB_0] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                                }
                            }
                            else if ((22U == name_string_length) && ((0 == std::memcmp("RightThumbIntermediate", name_string_base, name_string_length)) || (0 == std::memcmp("rightThumbIntermediate", name_string_base, name_string_length))))
                            {
                                if (this->m_vrmc_vrm)
                                {
                                    assert(false);
                                }
                                else
                                {
                                    DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                    DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                    DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                    DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                    this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_THUMB_1] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                                }
                            }
                            else if ((16U == name_string_length) && ((0 == std::memcmp("RightThumbDistal", name_string_base, name_string_length)) || (0 == std::memcmp("rightThumbDistal", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_THUMB_2] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((18U == name_string_length) && ((0 == std::memcmp("RightIndexProximal", name_string_base, name_string_length)) || (0 == std::memcmp("rightIndexProximal", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_INDEX_FINGER_1] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((22U == name_string_length) && ((0 == std::memcmp("RightIndexIntermediate", name_string_base, name_string_length)) || (0 == std::memcmp("rightIndexIntermediate", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_INDEX_FINGER_2] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((16U == name_string_length) && ((0 == std::memcmp("RightIndexDistal", name_string_base, name_string_length)) || (0 == std::memcmp("rightIndexDistal", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_INDEX_FINGER_3] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((19U == name_string_length) && ((0 == std::memcmp("RightMiddleProximal", name_string_base, name_string_length)) || (0 == std::memcmp("rightMiddleProximal", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_MIDDLE_FINGER_1] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((23U == name_string_length) && ((0 == std::memcmp("RightMiddleIntermediate", name_string_base, name_string_length)) || (0 == std::memcmp("rightMiddleIntermediate", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_MIDDLE_FINGER_2] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((17U == name_string_length) && ((0 == std::memcmp("RightMiddleDistal", name_string_base, name_string_length)) || (0 == std::memcmp("rightMiddleDistal", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_MIDDLE_FINGER_3] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((17U == name_string_length) && ((0 == std::memcmp("RightRingProximal", name_string_base, name_string_length)) || (0 == std::memcmp("rightRingProximal", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_RING_FINGER_1] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((21U == name_string_length) && ((0 == std::memcmp("RightRingIntermediate", name_string_base, name_string_length)) || (0 == std::memcmp("rightRingIntermediate", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_RING_FINGER_2] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if ((15U == name_string_length) && ((0 == std::memcmp("RightRingDistal", name_string_base, name_string_length)) || (0 == std::memcmp("rightRingDistal", name_string_base, name_string_length))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_RING_FINGER_3] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if (((19U == name_string_length) && ((0 == std::memcmp("RightLittleProximal", name_string_base, name_string_length)) || (0 == std::memcmp("rightLittleProximal", name_string_base, name_string_length)))) || ((18U == name_string_length) && ((0 == std::memcmp("RightPinkyProximal", name_string_base, name_string_length)) || (0 == std::memcmp("rightPinkyProximal", name_string_base, name_string_length)))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_LITTLE_FINGER_1] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if (((23U == name_string_length) && ((0 == std::memcmp("RightLittleIntermediate", name_string_base, name_string_length)) || (0 == std::memcmp("rightLittleIntermediate", name_string_base, name_string_length)))) || ((22U == name_string_length) && ((0 == std::memcmp("RightPinkyIntermediate", name_string_base, name_string_length)) || (0 == std::memcmp("rightPinkyIntermediate", name_string_base, name_string_length)))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_LITTLE_FINGER_2] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else if (((17U == name_string_length) && ((0 == std::memcmp("RightLittleDistal", name_string_base, name_string_length)) || (0 == std::memcmp("rightLittleDistal", name_string_base, name_string_length)))) || ((16U == name_string_length) && ((0 == std::memcmp("RightPinkyDistal", name_string_base, name_string_length)) || (0 == std::memcmp("rightPinkyDistal", name_string_base, name_string_length)))))
                            {
                                DirectX::XMFLOAT3 const p(p_x, p_y, p_z);
                                DirectX::XMFLOAT4 const q(q_x, q_y, q_z, q_w);
                                DirectX::XMFLOAT3 const p_transformed = internal_transform_translation(p);
                                DirectX::XMFLOAT4 const q_transformed = internal_transform_rotation(q);
                                this->m_skeleton_joint_transforms_local_space[INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_LITTLE_FINGER_3] = brx_motion_vmc_rigid_transform{q_transformed, p_transformed};
                            }
                            else
                            {
                                assert(false);
                            }
                        }
                        else
                        {
                            assert(false);
                        }
                    }
                    else if ((18U == address_pattern_string_length) && (0 == std::memcmp("/VMC/Ext/Blend/Val", address_pattern_string_base, address_pattern_string_length)))
                    {
                        if ((3U == type_tag_string_length) && (0 == std::memcmp(",sf", type_tag_string_base, type_tag_string_length)))
                        {
                            char const *name_string_base = NULL;
                            uint32_t name_string_length = 0U;
                            if (internal_unlikely(!internal_data_read_osc_string(osc_message_data_base, osc_message_data_length, osc_message_data_offset_current, &name_string_base, &name_string_length)))
                            {
                                assert(false);
                                return;
                            }

                            float value = 0.0F;
                            if (internal_unlikely(!internal_data_read_osc_float(osc_message_data_base, osc_message_data_length, osc_message_data_offset_current, &value)))
                            {
                                assert(false);
                                return;
                            }

                            if (((1U == name_string_length) && (0 == std::memcmp("A", name_string_base, name_string_length))) || ((2U == name_string_length) && (0 == std::memcmp("aa", name_string_base, name_string_length))))
                            {
                                this->m_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_A] = value;
                            }
                            else if (((1U == name_string_length) && (0 == std::memcmp("I", name_string_base, name_string_length))) || ((2U == name_string_length) && (0 == std::memcmp("ih", name_string_base, name_string_length))))
                            {
                                this->m_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_I] = value;
                            }
                            else if (((1U == name_string_length) && (0 == std::memcmp("U", name_string_base, name_string_length))) || ((2U == name_string_length) && (0 == std::memcmp("ou", name_string_base, name_string_length))))
                            {
                                this->m_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_U] = value;
                            }
                            else if (((1U == name_string_length) && (0 == std::memcmp("E", name_string_base, name_string_length))) || ((2U == name_string_length) && (0 == std::memcmp("ee", name_string_base, name_string_length))))
                            {
                                this->m_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_E] = value;
                            }
                            else if (((1U == name_string_length) && (0 == std::memcmp("O", name_string_base, name_string_length))) || ((2U == name_string_length) && (0 == std::memcmp("oh", name_string_base, name_string_length))))
                            {
                                this->m_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_O] = value;
                            }
                            else if ((5U == name_string_length) && ((0 == std::memcmp("Blink", name_string_base, name_string_length)) || (0 == std::memcmp("blink", name_string_base, name_string_length))))
                            {
                                this->m_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_BLINK] = value;
                            }
                            else if (((7U == name_string_length) && (0 == std::memcmp("Blink_L", name_string_base, name_string_length))) || ((9U == name_string_length) && (0 == std::memcmp("blinkLeft", name_string_base, name_string_length))))
                            {
                                this->m_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_BLINK_L] = value;
                            }
                            else if (((7U == name_string_length) && (0 == std::memcmp("Blink_R", name_string_base, name_string_length))) || ((10U == name_string_length) && (0 == std::memcmp("blinkRight", name_string_base, name_string_length))))
                            {
                                this->m_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_BLINK_R] = value;
                            }
                            else if (((3U == name_string_length) && (0 == std::memcmp("Joy", name_string_base, name_string_length))) || ((5U == name_string_length) && (0 == std::memcmp("happy", name_string_base, name_string_length))))
                            {
                                this->m_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_HAPPY] = value;
                                this->m_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_HAPPY] = value;
                                this->m_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_HAPPY] = value;
                            }
                            else if ((5U == name_string_length) && ((0 == std::memcmp("Angry", name_string_base, name_string_length)) || (0 == std::memcmp("angry", name_string_base, name_string_length))))
                            {
                                this->m_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_ANGRY] = value;
                                this->m_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_ANGRY] = value;
                                this->m_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_ANGRY] = value;
                            }
                            else if (((6U == name_string_length) && (0 == std::memcmp("Sorrow", name_string_base, name_string_length))) || ((3U == name_string_length) && (0 == std::memcmp("sad", name_string_base, name_string_length))))
                            {
                                this->m_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_SAD] = value;
                                this->m_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_SAD] = value;
                                this->m_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_SAD] = value;
                            }
                            else if (((3U == name_string_length) && (0 == std::memcmp("Fun", name_string_base, name_string_length))) || ((7U == name_string_length) && (0 == std::memcmp("relaxed", name_string_base, name_string_length))))
                            {
                                // Do Nothing
                            }
                            else if ((9U == name_string_length) && (0 == std::memcmp("surprised", name_string_base, name_string_length)))
                            {
                                this->m_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_SURPRISED] = value;
                                this->m_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_EYE_SURPRISED] = value;
                                this->m_morph_target_weights[BRX_MOTION_MORPH_TARGET_NAME_MMD_MOUTH_SURPRISED] = value;
                            }
                            else if ((7U == name_string_length) && ((0 == std::memcmp("Neutral", name_string_base, name_string_length)) || (0 == std::memcmp("neutral", name_string_base, name_string_length))))
                            {
                                // Do Nothing
                            }
                            else if ((6U == name_string_length) && ((0 == std::memcmp("LookUp", name_string_base, name_string_length)) || (0 == std::memcmp("lookUp", name_string_base, name_string_length))))
                            {
                                this->m_morph_joint_weights[INTERNAL_VMC_MORPH_JOINT_NAME_LEFT_EYE].x = value;
                                this->m_morph_joint_weights[INTERNAL_VMC_MORPH_JOINT_NAME_RIGHT_EYE].x = value;
                            }
                            else if ((8U == name_string_length) && ((0 == std::memcmp("LookDown", name_string_base, name_string_length)) || (0 == std::memcmp("lookDown", name_string_base, name_string_length))))
                            {
                                this->m_morph_joint_weights[INTERNAL_VMC_MORPH_JOINT_NAME_LEFT_EYE].y = value;
                                this->m_morph_joint_weights[INTERNAL_VMC_MORPH_JOINT_NAME_RIGHT_EYE].y = value;
                            }
                            else if ((8U == name_string_length) && ((0 == std::memcmp("LookLeft", name_string_base, name_string_length)) || (0 == std::memcmp("lookLeft", name_string_base, name_string_length))))
                            {
                                this->m_morph_joint_weights[INTERNAL_VMC_MORPH_JOINT_NAME_LEFT_EYE].z = value;
                                this->m_morph_joint_weights[INTERNAL_VMC_MORPH_JOINT_NAME_RIGHT_EYE].z = value;
                            }
                            else if ((9U == name_string_length) && ((0 == std::memcmp("LookRight", name_string_base, name_string_length)) || (0 == std::memcmp("lookRight", name_string_base, name_string_length))))
                            {
                                this->m_morph_joint_weights[INTERNAL_VMC_MORPH_JOINT_NAME_LEFT_EYE].w = value;
                                this->m_morph_joint_weights[INTERNAL_VMC_MORPH_JOINT_NAME_RIGHT_EYE].w = value;
                            }
                            else
                            {
                                assert(false);
                            }
                        }
                        else
                        {
                            assert(false);
                        }
                    }
                    if ((20U == address_pattern_string_length) && (0 == std::memcmp("/VMC/Ext/Blend/Apply", address_pattern_string_base, address_pattern_string_length)))
                    {
                        if ((1U == type_tag_string_length) && (0 == std::memcmp(",", type_tag_string_base, type_tag_string_length)))
                        {
                            // NOTE: UDP not reliable
                            // this->m_apply = true;
                        }
                        else
                        {
                            assert(false);
                        }
                    }
                    else
                    {
                        // Do Nothing
                    }
                }
                else
                {
                    // Unknown OSC Packet Type
                    assert(false);
                }
            }

#if defined(USE_WSA)
#if USE_WSA
        }
        assert((0 == unused_res_recv) || WSAEWOULDBLOCK == WSAGetLastError());
#else
        }
        assert((-1 != unused_res_recv) || ((EAGAIN == errno) || (EWOULDBLOCK == errno)));
#endif
#else
#error "0 or 1"
#endif
    }

    // sync from local space to model space
    {
        mcrt_vector<DirectX::XMFLOAT4X4> animation_skeleton_animation_pose_model_space(static_cast<size_t>(INTERNAL_VMC_SKELETON_JOINT_NAME_COUNT));

        for (uint32_t current_animation_skeleton_joint_index = 0; current_animation_skeleton_joint_index < static_cast<uint32_t>(INTERNAL_VMC_SKELETON_JOINT_NAME_COUNT); ++current_animation_skeleton_joint_index)
        {
            DirectX::XMFLOAT4 const &animation_skeleton_animation_pose_rotation_local_space = this->m_skeleton_joint_transforms_local_space[current_animation_skeleton_joint_index].m_rotation;

            DirectX::XMFLOAT3 const &animation_skeleton_animation_pose_translation_local_space = this->m_skeleton_joint_transforms_local_space[current_animation_skeleton_joint_index].m_translation;

            DirectX::XMFLOAT4X4 animation_skeleton_animation_pose_local_space;
            DirectX::XMStoreFloat4x4(&animation_skeleton_animation_pose_local_space, DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(DirectX::XMLoadFloat4(&animation_skeleton_animation_pose_rotation_local_space)), DirectX::XMMatrixTranslationFromVector(DirectX::XMLoadFloat3(&animation_skeleton_animation_pose_translation_local_space))));

            uint32_t const parent_animation_skeleton_joint_index = internal_vmc_skeleton_joint_parent_indices[current_animation_skeleton_joint_index];
            if (BRX_MOTION_UINT32_INDEX_INVALID != parent_animation_skeleton_joint_index)
            {
                assert(parent_animation_skeleton_joint_index < current_animation_skeleton_joint_index);

                DirectX::XMStoreFloat4x4(&animation_skeleton_animation_pose_model_space[current_animation_skeleton_joint_index], DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&animation_skeleton_animation_pose_local_space), DirectX::XMLoadFloat4x4(&animation_skeleton_animation_pose_model_space[parent_animation_skeleton_joint_index])));
            }
            else
            {
                animation_skeleton_animation_pose_model_space[current_animation_skeleton_joint_index] = animation_skeleton_animation_pose_local_space;
            }
        }

        for (uint32_t current_animation_skeleton_joint_index = 0; current_animation_skeleton_joint_index < static_cast<uint32_t>(INTERNAL_VMC_SKELETON_JOINT_NAME_COUNT); ++current_animation_skeleton_joint_index)
        {
            DirectX::XMFLOAT4 animation_skeleton_animation_pose_rotation_model_space;
            DirectX::XMFLOAT3 animation_skeleton_animation_pose_translation_model_space;
            {
                DirectX::XMVECTOR simd_animation_skeleton_animation_pose_scale_model_space;
                DirectX::XMVECTOR simd_animation_skeleton_animation_pose_rotation_model_space;
                DirectX::XMVECTOR simd_animation_skeleton_animation_pose_translation_model_space;
                bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&simd_animation_skeleton_animation_pose_scale_model_space, &simd_animation_skeleton_animation_pose_rotation_model_space, &simd_animation_skeleton_animation_pose_translation_model_space, DirectX::XMLoadFloat4x4(&animation_skeleton_animation_pose_model_space[current_animation_skeleton_joint_index]));
                assert(directx_xm_matrix_decompose);

                assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(simd_animation_skeleton_animation_pose_scale_model_space, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));

                DirectX::XMStoreFloat4(&animation_skeleton_animation_pose_rotation_model_space, simd_animation_skeleton_animation_pose_rotation_model_space);
                DirectX::XMStoreFloat3(&animation_skeleton_animation_pose_translation_model_space, simd_animation_skeleton_animation_pose_translation_model_space);
            }

            this->m_skeleton_joint_transforms_model_space[current_animation_skeleton_joint_index] = brx_motion_vmc_rigid_transform{animation_skeleton_animation_pose_rotation_model_space, animation_skeleton_animation_pose_translation_model_space};
        }
    }
}

brx_motion_rigid_transform brx_motion_vmc_motion_receiver::get_model_transform() const
{
    return brx_motion_rigid_transform{{this->m_model_transform.m_rotation.x, this->m_model_transform.m_rotation.y, this->m_model_transform.m_rotation.z, this->m_model_transform.m_rotation.w}, {this->m_model_transform.m_translation.x, this->m_model_transform.m_translation.y, this->m_model_transform.m_translation.z}};
}

float brx_motion_vmc_motion_receiver::get_morph_target_weight(uint32_t face_index, BRX_MOTION_MORPH_TARGET_NAME morph_target_name) const
{
    assert(0U == face_index);
    return this->m_morph_target_weights[morph_target_name];
}

double brx_motion_vmc_motion_receiver::get_delta_time() const
{
    // TODO:
    return (1.0 / 30.0);
}

DirectX::XMFLOAT4 brx_motion_vmc_motion_receiver::get_face_morph_joint_weight(uint32_t face_index, BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name) const
{
    assert(0U == face_index);

    assert((BRX_MOTION_SKELETON_JOINT_NAME_INVALID == skeleton_joint_name) || (skeleton_joint_name < BRX_MOTION_SKELETON_JOINT_NAME_MMD_COUNT));
    uint32_t const face_morph_joint_index = internal_get_face_morph_joint_index(skeleton_joint_name);

    if (BRX_MOTION_UINT32_INDEX_INVALID != face_morph_joint_index)
    {
        assert(face_morph_joint_index < INTERNAL_VMC_MORPH_JOINT_NAME_COUNT);
        return this->m_morph_joint_weights[face_morph_joint_index];
    }
    else
    {
        assert(false);
        return DirectX::XMFLOAT4(0.0, 0.0, 0.0, 0.0);
    }
}

DirectX::XMFLOAT3 const *brx_motion_vmc_motion_receiver::get_pose_skeleton_joint_translation(uint32_t pose_index, BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name) const
{
    assert(0U == pose_index);

    assert((BRX_MOTION_SKELETON_JOINT_NAME_INVALID == skeleton_joint_name) || (skeleton_joint_name < BRX_MOTION_SKELETON_JOINT_NAME_MMD_COUNT));
    uint32_t const pose_skeleton_joint_index = internal_get_pose_skeleton_joint_index(skeleton_joint_name);

    if (BRX_MOTION_UINT32_INDEX_INVALID != pose_skeleton_joint_index)
    {
        assert(pose_skeleton_joint_index < INTERNAL_VMC_SKELETON_JOINT_NAME_COUNT);
        if (this->m_apply)
        {
            return &this->m_skeleton_joint_transforms_model_space[pose_skeleton_joint_index].m_translation;
        }
        else
        {
            return NULL;
        }
    }
    else
    {
        assert(false);
        return NULL;
    }
}

DirectX::XMFLOAT4 const *brx_motion_vmc_motion_receiver::get_face_skeleton_joint_rotation(uint32_t face_index, BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name) const
{
    assert(0U == face_index);

    assert((BRX_MOTION_SKELETON_JOINT_NAME_INVALID == skeleton_joint_name) || (skeleton_joint_name < BRX_MOTION_SKELETON_JOINT_NAME_MMD_COUNT));
    uint32_t const face_skeleton_joint_index = internal_get_face_skeleton_joint_index(skeleton_joint_name);

    if (BRX_MOTION_UINT32_INDEX_INVALID != face_skeleton_joint_index)
    {
        assert(face_skeleton_joint_index < INTERNAL_VMC_SKELETON_JOINT_NAME_COUNT);
        if (this->m_apply)
        {
            return &this->m_skeleton_joint_transforms_model_space[face_skeleton_joint_index].m_rotation;
        }
        else
        {
            return NULL;
        }
    }
    else
    {
        return NULL;
    }
}

DirectX::XMFLOAT3 const *brx_motion_vmc_motion_receiver::get_hand_skeleton_joint_translation(uint32_t hand_index, BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name) const
{
    assert(0U == hand_index);

    assert((BRX_MOTION_SKELETON_JOINT_NAME_INVALID == skeleton_joint_name) || (skeleton_joint_name < BRX_MOTION_SKELETON_JOINT_NAME_MMD_COUNT));
    uint32_t const hand_skeleton_joint_index = internal_get_hand_skeleton_joint_index(skeleton_joint_name);

    if (BRX_MOTION_UINT32_INDEX_INVALID != hand_skeleton_joint_index)
    {
        assert(hand_skeleton_joint_index < INTERNAL_VMC_SKELETON_JOINT_NAME_COUNT);
        if (this->m_apply)
        {
            return &this->m_skeleton_joint_transforms_model_space[hand_skeleton_joint_index].m_translation;
        }
        else
        {
            return NULL;
        }
    }
    else
    {
        assert(false);
        return NULL;
    }
}

static inline uint32_t internal_align_up(uint32_t value, uint32_t alignment);

static inline bool internal_data_read_osc_uint32(void const *data_base, uint32_t data_length, uint32_t &inout_data_offset, uint32_t *out_uint32)
{
    if ((inout_data_offset + (sizeof(uint8_t) * 4U)) <= data_length)
    {
        uint8_t const *const not_aligned_network_uint32_base = reinterpret_cast<uint8_t const *>(reinterpret_cast<uintptr_t>(data_base) + inout_data_offset);
        uint8_t const not_aligned_network_uint32[4] = {not_aligned_network_uint32_base[0], not_aligned_network_uint32_base[1], not_aligned_network_uint32_base[2], not_aligned_network_uint32_base[3]};
        inout_data_offset += (sizeof(uint8_t) * 4U);
        uint32_t const aligned_network_uint32 = std::bit_cast<uint32_t>(not_aligned_network_uint32);
        (*out_uint32) = internal_ntoh32(aligned_network_uint32);
        return true;
    }
    else
    {
        return false;
    }
}

static inline bool internal_data_read_osc_float(void const *data_base, uint32_t data_length, uint32_t &inout_data_offset, float *out_float)
{
    if ((inout_data_offset + (sizeof(uint8_t) * 4U)) <= data_length)
    {
        uint8_t const *const not_aligned_network_uint32_base = reinterpret_cast<uint8_t const *>(reinterpret_cast<uintptr_t>(data_base) + inout_data_offset);
        uint8_t const not_aligned_network_uint32[4] = {not_aligned_network_uint32_base[0], not_aligned_network_uint32_base[1], not_aligned_network_uint32_base[2], not_aligned_network_uint32_base[3]};
        inout_data_offset += (sizeof(uint8_t) * 4U);
        uint32_t const aligned_network_uint32 = std::bit_cast<uint32_t>(not_aligned_network_uint32);
        uint32_t const aligned_host_uint32 = internal_ntoh32(aligned_network_uint32);
        (*out_float) = std::bit_cast<float>(aligned_host_uint32);
        return true;
    }
    else
    {
        return false;
    }
}

static inline bool internal_data_read_osc_string(void const *data_base, uint32_t data_length, uint32_t &inout_data_offset, char const **out_string_base, uint32_t *out_string_length)
{
    uint32_t string_length = 0U;
    bool found_null_terminator = false;
    while ((inout_data_offset + string_length) < data_length)
    {
        if ('\0' == reinterpret_cast<char const *>(reinterpret_cast<uintptr_t>(data_base) + inout_data_offset)[string_length])
        {
            found_null_terminator = true;
            break;
        }
        ++string_length;
    }

    if (found_null_terminator)
    {
        (*out_string_base) = reinterpret_cast<char const *>(reinterpret_cast<uintptr_t>(data_base) + inout_data_offset);
        (*out_string_length) = string_length;
        inout_data_offset += (string_length + 1U);
        inout_data_offset = internal_align_up(inout_data_offset, 4U);
        return true;
    }
    else
    {
        return false;
    }
}

static inline uint32_t internal_align_up(uint32_t value, uint32_t alignment)
{
    //
    //  Copyright (c) 2005-2019 Intel Corporation
    //
    //  Licensed under the Apache License, Version 2.0 (the "License");
    //  you may not use this file except in compliance with the License.
    //  You may obtain a copy of the License at
    //
    //      http://www.apache.org/licenses/LICENSE-2.0
    //
    //  Unless required by applicable law or agreed to in writing, software
    //  distributed under the License is distributed on an "AS IS" BASIS,
    //  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    //  See the License for the specific language governing permissions and
    //  limitations under the License.
    //

    // [alignUp](https://github.com/oneapi-src/oneTBB/blob/tbb_2019/src/tbbmalloc/shared_utils.h#L42)

    assert(alignment != static_cast<uint32_t>(0U));

    // power-of-2 alignment
    assert((alignment & (alignment - static_cast<uint32_t>(1U))) == static_cast<uint32_t>(0U));

    return (((value - static_cast<uint32_t>(1U)) | (alignment - static_cast<uint32_t>(1U))) + static_cast<uint32_t>(1U));
}

// glTF
// RH
// Up +Y
// Forward Z
// Right -X

// VRM
// RH
// Up +Y
// Forward -Z
// Right +X

// VRMC_vrm
// RH
// Up +Y
// Forward Z
// Right -X

// Unity
// LH
// Up +Y
// Forward Z
// Right X

static inline DirectX::XMFLOAT3 internal_transform_translation(DirectX::XMFLOAT3 const &v)
{
    // from Unity to glTF

    return DirectX::XMFLOAT3{-v.x, v.y, v.z};
}

static inline DirectX::XMFLOAT4 internal_transform_rotation(DirectX::XMFLOAT4 const &q)
{
    // from Unity to glTF

    DirectX::XMFLOAT4 out_q;
    {
        DirectX::XMMATRIX r = DirectX::XMMatrixRotationQuaternion(DirectX::XMLoadFloat4(&q));

        DirectX::XMFLOAT4X4 x = {
            -1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0};
        DirectX::XMMATRIX simd_x = DirectX::XMLoadFloat4x4(&x);

        DirectX::XMStoreFloat4(&out_q, DirectX::XMQuaternionNormalize(DirectX::XMQuaternionRotationMatrix(DirectX::XMMatrixMultiply(DirectX::XMMatrixMultiply(simd_x, r), simd_x))));
    }

    return out_q;
}

static inline uint32_t internal_get_face_morph_joint_index(BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name)
{
    uint32_t face_morph_joint_index;
    switch (skeleton_joint_name)
    {
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_EYE:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_EYE == skeleton_joint_name);
        face_morph_joint_index = INTERNAL_VMC_MORPH_JOINT_NAME_RIGHT_EYE;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_EYE:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_EYE == skeleton_joint_name);
        face_morph_joint_index = INTERNAL_VMC_MORPH_JOINT_NAME_LEFT_EYE;
    }
    break;
    default:
    {
        assert(false);
        face_morph_joint_index = BRX_MOTION_UINT32_INDEX_INVALID;
    }
    }
    return face_morph_joint_index;
}

static inline uint32_t internal_get_pose_skeleton_joint_index(BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name)
{
    uint32_t pose_skeleton_joint_index;
    switch (skeleton_joint_name)
    {
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_CENTER:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_CENTER == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_CENTER;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_UPPER_BODY_2:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_UPPER_BODY_2 == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_UPPER_BODY_2;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_ARM:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_ARM == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_ARM;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_ELBOW:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_ELBOW == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_ELBOW;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_WRIST:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_WRIST == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_WRIST;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_ARM:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_ARM == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_ARM;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_ELBOW:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_ELBOW == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_ELBOW;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_WRIST:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_WRIST == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_WRIST;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_LEG:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_LEG == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_LEG;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_KNEE:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_KNEE == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_KNEE;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_ANKLE:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_ANKLE == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_ANKLE;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_TOE_TIP:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_TOE_TIP == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_TOE_TIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_LEG:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_LEG == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_LEG;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_KNEE:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_KNEE == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_KNEE;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_ANKLE:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_ANKLE == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_ANKLE;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_TOE_TIP:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_TOE_TIP == skeleton_joint_name);
        pose_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_TOE_TIP;
    }
    break;
    default:
    {
        assert(false);
        pose_skeleton_joint_index = BRX_MOTION_UINT32_INDEX_INVALID;
    }
    }
    return pose_skeleton_joint_index;
}

static inline uint32_t internal_get_face_skeleton_joint_index(BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name)
{
    uint32_t face_skeleton_joint_index;
    switch (skeleton_joint_name)
    {
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_HEAD:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_HEAD == skeleton_joint_name);
        face_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_HEAD;
    }
    break;
    default:
    {
        assert(false);
        face_skeleton_joint_index = BRX_MOTION_UINT32_INDEX_INVALID;
    }
    }
    return face_skeleton_joint_index;
}

static inline uint32_t internal_get_hand_skeleton_joint_index(BRX_MOTION_SKELETON_JOINT_NAME skeleton_joint_name)
{
    uint32_t hand_skeleton_joint_index;
    switch (skeleton_joint_name)
    {
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_WRIST:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_WRIST == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_WRIST;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_THUMB_0:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_THUMB_0 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_THUMB_0;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_THUMB_1:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_THUMB_1 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_THUMB_1;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_THUMB_2:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_THUMB_2 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_THUMB_2;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_THUMB_TIP:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_THUMB_TIP == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_THUMB_TIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_INDEX_FINGER_1:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_INDEX_FINGER_1 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_INDEX_FINGER_1;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_INDEX_FINGER_2:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_INDEX_FINGER_2 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_INDEX_FINGER_2;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_INDEX_FINGER_3:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_INDEX_FINGER_3 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_INDEX_FINGER_3;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_INDEX_FINGER_TIP:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_INDEX_FINGER_TIP == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_INDEX_FINGER_TIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_MIDDLE_FINGER_1:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_MIDDLE_FINGER_1 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_MIDDLE_FINGER_1;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_MIDDLE_FINGER_2:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_MIDDLE_FINGER_2 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_MIDDLE_FINGER_2;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_MIDDLE_FINGER_3:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_MIDDLE_FINGER_3 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_MIDDLE_FINGER_3;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_MIDDLE_FINGER_TIP:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_MIDDLE_FINGER_TIP == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_MIDDLE_FINGER_TIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_RING_FINGER_1:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_RING_FINGER_1 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_RING_FINGER_1;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_RING_FINGER_2:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_RING_FINGER_2 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_RING_FINGER_2;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_RING_FINGER_3:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_RING_FINGER_3 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_RING_FINGER_3;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_RING_FINGER_TIP:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_RING_FINGER_TIP == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_RING_FINGER_TIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_LITTLE_FINGER_1:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_LITTLE_FINGER_1 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_LITTLE_FINGER_1;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_LITTLE_FINGER_2:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_LITTLE_FINGER_2 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_LITTLE_FINGER_2;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_LITTLE_FINGER_3:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_LITTLE_FINGER_3 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_LITTLE_FINGER_3;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_LITTLE_FINGER_TIP:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_RIGHT_LITTLE_FINGER_TIP == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_RIGHT_LITTLE_FINGER_TIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_WRIST:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_WRIST == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_WRIST;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_THUMB_0:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_THUMB_0 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_THUMB_0;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_THUMB_1:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_THUMB_1 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_THUMB_1;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_THUMB_2:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_THUMB_2 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_THUMB_2;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_THUMB_TIP:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_THUMB_TIP == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_THUMB_TIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_INDEX_FINGER_1:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_INDEX_FINGER_1 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_INDEX_FINGER_1;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_INDEX_FINGER_2:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_INDEX_FINGER_2 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_INDEX_FINGER_2;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_INDEX_FINGER_3:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_INDEX_FINGER_3 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_INDEX_FINGER_3;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_INDEX_FINGER_TIP:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_INDEX_FINGER_TIP == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_INDEX_FINGER_TIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_MIDDLE_FINGER_1:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_MIDDLE_FINGER_1 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_MIDDLE_FINGER_1;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_MIDDLE_FINGER_2:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_MIDDLE_FINGER_2 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_MIDDLE_FINGER_2;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_MIDDLE_FINGER_3:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_MIDDLE_FINGER_3 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_MIDDLE_FINGER_3;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_MIDDLE_FINGER_TIP:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_MIDDLE_FINGER_TIP == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_MIDDLE_FINGER_TIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_RING_FINGER_1:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_RING_FINGER_1 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_RING_FINGER_1;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_RING_FINGER_2:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_RING_FINGER_2 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_RING_FINGER_2;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_RING_FINGER_3:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_RING_FINGER_3 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_RING_FINGER_3;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_RING_FINGER_TIP:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_RING_FINGER_TIP == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_RING_FINGER_TIP;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_LITTLE_FINGER_1:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_LITTLE_FINGER_1 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_LITTLE_FINGER_1;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_LITTLE_FINGER_2:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_LITTLE_FINGER_2 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_LITTLE_FINGER_2;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_LITTLE_FINGER_3:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_LITTLE_FINGER_3 == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_LITTLE_FINGER_3;
    }
    break;
    case BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_LITTLE_FINGER_TIP:
    {
        assert(BRX_MOTION_SKELETON_JOINT_NAME_MMD_LEFT_LITTLE_FINGER_TIP == skeleton_joint_name);
        hand_skeleton_joint_index = INTERNAL_VMC_SKELETON_JOINT_NAME_LEFT_LITTLE_FINGER_TIP;
    }
    break;
    default:
    {
        assert(false);
        hand_skeleton_joint_index = BRX_MOTION_UINT32_INDEX_INVALID;
    }
    }
    return hand_skeleton_joint_index;
}

#if defined(USE_WSA)
#if USE_WSA
class internal_wsa_scope
{
public:
    inline internal_wsa_scope()
    {
        WSADATA wsa_data;
        int res_wsa_start_up = WSAStartup(MAKEWORD(2, 2), &wsa_data);
        assert(0 == res_wsa_start_up);

        assert(MAKEWORD(2, 2) == wsa_data.wVersion);
    }

    inline ~internal_wsa_scope()
    {
        int res_wsa_clean_up = WSACleanup();
        assert(0 == res_wsa_clean_up);
    }
};

static internal_wsa_scope internal_wsa_scope_instance;
#else
// Do Nothing
#endif
#else
#error "0 or 1"
#endif
