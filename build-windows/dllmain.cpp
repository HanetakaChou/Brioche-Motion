#define WIN32_LEAN_AND_MEAN 1
#include <sdkddkver.h>
#include <Windows.h>
#include <assert.h>

BOOL APIENTRY DllMain(HMODULE hModule, DWORD ul_reason_for_call, LPVOID)
{
    switch (ul_reason_for_call)
    {
    case DLL_PROCESS_ATTACH:
    {
        DisableThreadLibraryCalls(hModule);
    }
    break;
    case DLL_PROCESS_DETACH:
    {
    }
    break;
    case DLL_THREAD_ATTACH:
    case DLL_THREAD_DETACH:
    default:
    {
        // we have disabled the "Thread Library Calls"
        // assert(false);
    }
    }
    return TRUE;
}
