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

#include "internal_highgui.h"
#include "../../McRT-Malloc/include/mcrt_string.h"
#include <cassert>

#if defined(__GNUC__)
// GCC or CLANG
#elif defined(_MSC_VER)
// MSVC or CLANG-CL
#define NOMINMAX 1
#define WIN32_LEAN_AND_MEAN 1
#include <sdkddkver.h>
#include <Windows.h>

#include "../../libiconv/include/iconv.h"

#include "../build-windows/resource.h"

struct internal_brx_window
{
    HWND window;
    HDC device_context;
    HDC memory_device_context;
    HBITMAP bitmap;
};

extern "C" IMAGE_DOS_HEADER __ImageBase;

static ATOM s_window_class = 0;

static constexpr DWORD const s_dw_style = WS_CLIPSIBLINGS | WS_CLIPCHILDREN | WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX;
static constexpr DWORD const s_dw_ex_style = WS_EX_APPWINDOW;

static LRESULT CALLBACK s_wnd_proc(HWND hWnd, UINT Msg, WPARAM wParam, LPARAM lParam);

extern void *internal_brx_named_window(char const *const window_name)
{
    internal_brx_window *unwrapped_window = new (mcrt_malloc(sizeof(internal_brx_window), alignof(internal_brx_window))) internal_brx_window{NULL, NULL, NULL, NULL};
    assert(NULL != unwrapped_window);

    HINSTANCE hInstance = reinterpret_cast<HINSTANCE>(&__ImageBase);

    if (0 == s_window_class)
    {
        WNDCLASSEXW const window_class_create_info = {
            sizeof(WNDCLASSEX),
            CS_HREDRAW | CS_VREDRAW | CS_OWNDC | CS_NOCLOSE,
            s_wnd_proc,
            0,
            0,
            hInstance,
            LoadIconW(hInstance, MAKEINTRESOURCEW(IDI_ICON_OPENCV)),
            LoadCursorW(hInstance, IDC_ARROW),
            (HBRUSH)GetStockObject(NULL_BRUSH),
            NULL,
            L"Brioche Motion",
            LoadIconW(hInstance, MAKEINTRESOURCEW(IDI_ICON_OPENCV)),
        };
        s_window_class = RegisterClassExW(&window_class_create_info);
        assert(0 != s_window_class);
    }

    constexpr int32_t const window_width = 256;
    constexpr int32_t const window_height = 256;

    {
        mcrt_wstring window_name_utf16;
        {
            assert(window_name_utf16.empty());

            mcrt_string src_utf8 = window_name;
            mcrt_wstring &dst_utf16 = window_name_utf16;

            assert(dst_utf16.empty());

            if (!src_utf8.empty())
            {
                // Allocate the same number of UTF-16 code units as UTF-8 code units. Encoding
                // as UTF-16 should always require the same amount or less code units than the
                // UTF-8 encoding.  Allocate one extra byte for the null terminator though,
                // so that someone calling DstUTF16.data() gets a null terminated string.
                // We resize down later so we don't have to worry that this over allocates.
                dst_utf16.resize(src_utf8.size() + 1U);

                size_t in_bytes_left = sizeof(src_utf8[0]) * src_utf8.size();
                size_t out_bytes_left = sizeof(dst_utf16[0]) * dst_utf16.size();
                char *in_buf = src_utf8.data();
                char *out_buf = reinterpret_cast<char *>(dst_utf16.data());

                iconv_t conversion_descriptor = iconv_open("UTF-16LE", "UTF-8");
                assert(((iconv_t)(-1)) != conversion_descriptor);

                size_t conversion_result = iconv(conversion_descriptor, &in_buf, &in_bytes_left, &out_buf, &out_bytes_left);
                assert(((size_t)(-1)) != conversion_result);

                int result = iconv_close(conversion_descriptor);
                assert(-1 != result);

                dst_utf16.resize(reinterpret_cast<decltype(&dst_utf16[0])>(out_buf) - dst_utf16.data());
            }
        }

        HWND const desktop_window = GetDesktopWindow();

        RECT rect;
        {
            HMONITOR const monitor = MonitorFromWindow(desktop_window, MONITOR_DEFAULTTONEAREST);

            MONITORINFOEXW monitor_info;
            monitor_info.cbSize = sizeof(MONITORINFOEXW);
            BOOL res_get_monitor_info = GetMonitorInfoW(monitor, &monitor_info);
            assert(FALSE != res_get_monitor_info);

            rect = RECT{(monitor_info.rcWork.left + monitor_info.rcWork.right) / 2 - window_width / 2,
                        (monitor_info.rcWork.bottom + monitor_info.rcWork.top) / 2 - window_height / 2,
                        (monitor_info.rcWork.left + monitor_info.rcWork.right) / 2 + window_width / 2,
                        (monitor_info.rcWork.bottom + monitor_info.rcWork.top) / 2 + window_height / 2};

            BOOL const res_adjust_window_rest = AdjustWindowRectEx(&rect, s_dw_style, FALSE, s_dw_ex_style);
            assert(FALSE != res_adjust_window_rest);
        }

        assert(NULL == unwrapped_window->window);
        unwrapped_window->window = CreateWindowExW(s_dw_ex_style, MAKEINTATOM(s_window_class), window_name_utf16.c_str(), s_dw_style, rect.left, rect.top, rect.right - rect.left, rect.bottom - rect.top, desktop_window, NULL, hInstance, NULL);
        assert(NULL != unwrapped_window->window);
    }

    assert(NULL == unwrapped_window->device_context);
    unwrapped_window->device_context = GetDC(unwrapped_window->window);
    assert(NULL != unwrapped_window->device_context);

    assert(NULL == unwrapped_window->memory_device_context);
    unwrapped_window->memory_device_context = CreateCompatibleDC(unwrapped_window->device_context);
    assert(NULL != unwrapped_window->memory_device_context);

    assert(NULL == unwrapped_window->bitmap);
    unwrapped_window->bitmap = CreateCompatibleBitmap(unwrapped_window->device_context, window_width, window_height);
    assert(NULL != unwrapped_window->bitmap);

    ShowWindow(unwrapped_window->window, SW_SHOWDEFAULT);

    return static_cast<void *>(unwrapped_window);
}

static LRESULT CALLBACK s_wnd_proc(HWND hWnd, UINT Msg, WPARAM wParam, LPARAM lParam)
{
    LRESULT result;
    switch (Msg)
    {
    case WM_ERASEBKGND:
    {
        assert(WM_ERASEBKGND == Msg);
        result = 1;
    }
    break;
    case WM_CLOSE:
    {
        assert(WM_CLOSE == Msg);
        result = 0;
    }
    break;
    default:
    {
        assert((WM_ERASEBKGND != Msg) && (WM_CLOSE != Msg));
        result = DefWindowProcW(hWnd, Msg, wParam, lParam);
    }
    }
    return result;
}

extern void internal_brx_destroy_window(void *const wrapped_window)
{
    internal_brx_window *unwrapped_window = static_cast<internal_brx_window *>(wrapped_window);
    assert(NULL != unwrapped_window);

    assert(NULL != unwrapped_window->bitmap);
    BOOL result_delete_object = DeleteObject(unwrapped_window->bitmap);
    assert(FALSE != result_delete_object);
    unwrapped_window->bitmap = NULL;

    assert(NULL != unwrapped_window->memory_device_context);
    BOOL result_delete_dc = DeleteDC(unwrapped_window->memory_device_context);
    assert(FALSE != result_delete_dc);
    unwrapped_window->memory_device_context = NULL;

    assert(NULL != unwrapped_window->device_context);
    BOOL result_release_dc = ReleaseDC(unwrapped_window->window, unwrapped_window->device_context);
    assert(FALSE != result_release_dc);
    unwrapped_window->device_context = NULL;

    assert(NULL != unwrapped_window->window);
    BOOL res_destroy_window = DestroyWindow(unwrapped_window->window);
    assert(FALSE != res_destroy_window);
    unwrapped_window->window = NULL;

    unwrapped_window->~internal_brx_window();
    mcrt_free(unwrapped_window);
}

extern void internal_brx_image_show(void *const wrapped_window, void const *const image_buffer, int const image_width, int const image_height)
{
    internal_brx_window *unwrapped_window = static_cast<internal_brx_window *>(wrapped_window);
    assert(NULL != unwrapped_window);

    assert(NULL != unwrapped_window->window);
    assert(NULL != unwrapped_window->device_context);

    int window_width;
    int window_height;
    {
        RECT rect;
        BOOL res_get_client_rect = GetClientRect(unwrapped_window->window, &rect);
        assert(FALSE != res_get_client_rect);

        window_width = rect.right - rect.left;
        window_height = rect.bottom - rect.top;
    }

    if ((image_width != window_width) || (image_height != window_height))
    {
        {
            RECT rect;
            {
                rect = RECT{0, 0, image_width, image_height};

                BOOL const res_adjust_window_rest = AdjustWindowRectEx(&rect, s_dw_style, FALSE, s_dw_ex_style);
                assert(FALSE != res_adjust_window_rest);
            }

            BOOL res_set_window_pos = SetWindowPos(unwrapped_window->window, nullptr, 0, 0, rect.right - rect.left, rect.bottom - rect.top, SWP_NOMOVE | SWP_NOZORDER | SWP_NOACTIVATE);
            assert(FALSE != res_set_window_pos);
        }

        assert(NULL != unwrapped_window->bitmap);
        BOOL result_delete_object = DeleteObject(unwrapped_window->bitmap);
        assert(FALSE != result_delete_object);
        unwrapped_window->bitmap = NULL;

        assert(NULL != unwrapped_window->memory_device_context);
        BOOL result_delete_dc = DeleteDC(unwrapped_window->memory_device_context);
        assert(FALSE != result_delete_dc);
        unwrapped_window->memory_device_context = NULL;

        assert(NULL == unwrapped_window->memory_device_context);
        unwrapped_window->memory_device_context = CreateCompatibleDC(unwrapped_window->device_context);
        assert(NULL != unwrapped_window->memory_device_context);

        assert(NULL == unwrapped_window->bitmap);
        unwrapped_window->bitmap = CreateCompatibleBitmap(unwrapped_window->device_context, image_width, image_height);
        assert(NULL != unwrapped_window->bitmap);
    }

    {
        // write "texture" into "back buffer"
        BITMAPINFO bmi;
        ZeroMemory(&bmi, sizeof(bmi));
        bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
        bmi.bmiHeader.biWidth = image_width;
        bmi.bmiHeader.biHeight = -image_height;
        bmi.bmiHeader.biPlanes = 1;
        bmi.bmiHeader.biBitCount = 32;
        bmi.bmiHeader.biCompression = BI_RGB;

        int result_set_dib_bits = SetDIBits(unwrapped_window->device_context, unwrapped_window->bitmap, 0, image_height, image_buffer, &bmi, DIB_RGB_COLORS);
        assert(result_set_dib_bits > 0);
    }

    {
        HBITMAP old_bitmap = reinterpret_cast<HBITMAP>(SelectObject(unwrapped_window->memory_device_context, unwrapped_window->bitmap));
        assert(NULL != old_bitmap && HGDI_ERROR != old_bitmap);

        // copy from "back-buffer" into "front buffer"
        int result_bit_blt = BitBlt(unwrapped_window->device_context, 0, 0, image_width, image_height, unwrapped_window->memory_device_context, 0, 0, SRCCOPY);
        assert(0 != result_bit_blt);

        HGDIOBJ new_bitmap = reinterpret_cast<HBITMAP>(SelectObject(unwrapped_window->memory_device_context, old_bitmap));
        assert(new_bitmap == unwrapped_window->bitmap);
    }
}

#else
#error Unknown Compiler
#endif
