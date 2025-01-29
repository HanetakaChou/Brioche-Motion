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

#ifndef _BRX_ANIMATION_IK_INTERNAL_INL_
#define _BRX_ANIMATION_IK_INTERNAL_INL_ 1

#if defined(__GNUC__)
// GCC or CLANG
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunknown-pragmas"
#include <DirectXMath.h>
#pragma GCC diagnostic pop
#elif defined(_MSC_VER)
// MSVC or CLANG-CL
#include <DirectXMath.h>
#else
#error Unknown Compiler
#endif
#include <cmath>

static inline float internal_sqrt(float x)
{
    constexpr float const epsilon = 1E-6F;
    return ((x > epsilon) ? std::sqrt(x) : 0.0F);
}

static inline DirectX::XMVECTOR XM_CALLCONV internal_compute_rotation_axis_damped(DirectX::XMVECTOR axis, float cos_angle, float sin_angle, float gain)
{
    constexpr float const epsilon = 1E-6F;
    assert(std::abs(DirectX::XMVectorGetX(DirectX::XMVector3Dot(axis, axis)) - 1.0F) < epsilon);

    constexpr float const one = 1.0F;
    constexpr float const half = 0.5F;
    constexpr float const zero = 0.0F;
    constexpr float const nearly_one = one - epsilon;

    float const damped_dot = one - gain + gain * cos_angle;

    float const cos_angle_div_2_square = (damped_dot + one) * half;

    if (cos_angle_div_2_square > zero && cos_angle >= (-nearly_one) && cos_angle <= nearly_one)
    {
        // cos(angle/2) = sqrt((1+cos(angle))/2)
        float cos_angle_div_2 = std::sqrt(cos_angle_div_2_square);

        // sin(angle/2) = sin(angle)/(2*cos(angle/2))
        float sin_angle_div_2 = sin_angle * ((gain * half) / cos_angle_div_2);

        // "cos_angle >= (-nearly_one) && cos_angle <= nearly_one" to avoid zero vector which can NOT be normalized
        return DirectX::XMQuaternionNormalize(DirectX::XMVectorSetW(DirectX::XMVectorScale(axis, sin_angle_div_2), cos_angle_div_2));
    }
    else if (cos_angle_div_2_square > zero && cos_angle > nearly_one)
    {
        return DirectX::XMQuaternionIdentity();
    }
    else
    {
        return DirectX::XMVectorSetW(axis, 0.0F);
    }
}

static inline DirectX::XMVECTOR XM_CALLCONV internal_calculate_perpendicular_vector(DirectX::XMVECTOR simd_in_v)
{
    int min = 0;
    int ok1 = 1;
    int ok2 = 2;

    float in_v[3];
    DirectX::XMStoreFloat3(reinterpret_cast<DirectX::XMFLOAT3 *>(&in_v[0]), simd_in_v);

    float a0 = in_v[0];
    float a1 = in_v[1];
    float a2 = in_v[2];

    if (a1 < a0)
    {
        ok1 = 0;
        min = 1;
        a0 = a1;
    }

    if (a2 < a0)
    {
        ok2 = min;
        min = 2;
    }

    float out_v[3] = {0.0F, 0.0F, 0.0F};
    out_v[ok1] = in_v[ok2];
    out_v[ok2] = -in_v[ok1];
    return DirectX::XMLoadFloat3(reinterpret_cast<DirectX::XMFLOAT3 *>(&out_v[0]));
}

static inline DirectX::XMVECTOR XM_CALLCONV internal_compute_shortest_rotation_damped(DirectX::XMVECTOR from, DirectX::XMVECTOR to, float gain)
{
    constexpr float const epsilon = 1E-6F;
    assert(std::abs(DirectX::XMVectorGetX(DirectX::XMVector3Dot(from, from)) - 1.0F) < epsilon);
    assert(std::abs(DirectX::XMVectorGetX(DirectX::XMVector3Dot(to, to)) - 1.0F) < epsilon);

    constexpr float const one = 1.0F;
    constexpr float const half = 0.5F;
    constexpr float const zero = 0.0F;
    constexpr float const nearly_one = one - epsilon;

    // cos(theta)
    float const cos_theta = DirectX::XMVectorGetX(DirectX::XMVector3Dot(from, to));

    float const damped_dot = one - gain + gain * cos_theta;

    float const cos_theta_div_2_square = (damped_dot + one) * half;

    if (cos_theta_div_2_square > zero && cos_theta >= (-nearly_one) && cos_theta <= nearly_one)
    {
        // cos(theta/2) = sqrt((1+cos(theta))/2)
        float cos_theta_div_2 = std::sqrt(cos_theta_div_2_square);

        // sin(theta)
        DirectX::XMVECTOR cross = DirectX::XMVector3Cross(from, to);

        // sin(theta/2) = sin(theta)/(2*cos(theta/2))
        DirectX::XMVECTOR sin_theta_div_2 = DirectX::XMVectorScale(cross, ((gain * half) / cos_theta_div_2));

        // "cos_theta >= (-nearly_one) && cos_theta <= nearly_one" to avoid zero vector which can NOT be normalized
        return DirectX::XMQuaternionNormalize(DirectX::XMVectorSetW(sin_theta_div_2, cos_theta_div_2));
    }
    else if (cos_theta_div_2_square > zero && cos_theta > nearly_one)
    {
        return DirectX::XMQuaternionIdentity();
    }
    else
    {
        return DirectX::XMVectorSetW(DirectX::XMVector3Normalize(internal_calculate_perpendicular_vector(from)), 0.0F);
    }
}

#else
#error include inl multiple times
#endif
