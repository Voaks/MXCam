// proxy.cpp
//   The opengl32.dll proxy itself:
//     1. defines one 16-byte jump stub per opengl32 export and exports it under the real name,
//     2. loads the genuine opengl32.dll from the system directory,
//     3. points every stub at the genuine function - or at our hook for the few we intercept.
//
//   Why runtime-patched jump stubs instead of a hand-written wrapper per function?
//   opengl32 exports ~360 functions. A bare `jmp` forwards the call with the caller's stack
//   and registers untouched, so we never need a signature or calling convention for anything
//   we don't hook, and the same code works on x86 and x64 (where MSVC has no inline asm or
//   naked functions). Hooked functions are ordinary, fully typed C++ (see hooks.cpp).

#include <windows.h>
#include <cstdlib>
#include <cstring>
#include <cwchar>

#include "config.h"
#include "hooks.h"
#include "log.h"

// ===========================================================================
// 1. Export stubs
// ===========================================================================
// Every stub lives in a dedicated read+execute section. It starts out as int3 bytes (crash
// loudly if it were ever reached before initialisation). InitializeProxy() briefly makes the
// section writable, rewrites each stub into "jmp [absolute target]", then restores read+execute.

#pragma section(".pxystub", read, execute)

#define MXCAM_INT3_X16 { 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, \
                         0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC }

// Export "glFoo" from the stub symbol "Proxy_glFoo". x86 C symbols carry a leading underscore.
#if defined(_M_IX86)
#define MXCAM_EXPORT_DIRECTIVE(name) __pragma(comment(linker, "/EXPORT:" #name "=_Proxy_" #name))
#elif defined(_M_X64)
#define MXCAM_EXPORT_DIRECTIVE(name) __pragma(comment(linker, "/EXPORT:" #name "=Proxy_" #name))
#else
#error "MXCam supports x86 and x64 builds only."
#endif

#define PXY_EXPORT(name)                                                        \
    MXCAM_EXPORT_DIRECTIVE(name)                                                \
    extern "C" __declspec(allocate(".pxystub")) __declspec(align(16))           \
        unsigned char Proxy_##name[16] = MXCAM_INT3_X16;
#include "gl_exports.inl"
#undef PXY_EXPORT

namespace mxcam {
namespace {

struct ExportStub {
    const char*    name;
    unsigned char* code;
};

#define PXY_EXPORT(name) { #name, Proxy_##name },
const ExportStub kExports[] = {
#include "gl_exports.inl"
};
#undef PXY_EXPORT

HMODULE g_realOpenGL = nullptr;

// Section names are 8 bytes, not NUL-terminated when they use all 8.
constexpr char kStubSection[8] = { '.', 'p', 'x', 'y', 's', 't', 'u', 'b' };

// Rewrites a stub as an indirect jump through an address stored inside the stub itself.
void WriteJump(unsigned char* code, const void* target)
{
#if defined(_M_X64)
    // FF 25 00 00 00 00    jmp qword ptr [rip+0]
    // <8-byte target>
    code[0] = 0xFF;
    code[1] = 0x25;
    memset(code + 2, 0, 4);
    memcpy(code + 6, &target, sizeof target);
#else
    // FF 25 <abs32>        jmp dword ptr [code+6]
    // <4-byte target>
    const void* slot = code + 6;
    code[0] = 0xFF;
    code[1] = 0x25;
    memcpy(code + 2, &slot, sizeof slot);
    memcpy(code + 6, &target, sizeof target);
#endif
}

// Target for exports missing from the system DLL (doesn't happen on any supported Windows).
// Every export has a different signature, so the only safe response is to stop cleanly.
void __cdecl UnresolvedExport()
{
    Log("fatal: the game called an opengl32 export that the system DLL does not provide");
    MessageBoxW(nullptr, L"The game called an OpenGL function that the system opengl32.dll "
                         L"does not provide. See mxcam.log.", L"MXCam", MB_ICONERROR);
    ExitProcess(1);
}

// Locates the stub section in our own loaded image.
bool FindStubSection(HMODULE self, BYTE*& start, SIZE_T& size)
{
    auto* base = reinterpret_cast<BYTE*>(self);
    auto* nt   = reinterpret_cast<IMAGE_NT_HEADERS*>(base + reinterpret_cast<IMAGE_DOS_HEADER*>(base)->e_lfanew);

    IMAGE_SECTION_HEADER* section = IMAGE_FIRST_SECTION(nt);
    for (WORD i = 0; i < nt->FileHeader.NumberOfSections; ++i, ++section) {
        if (memcmp(section->Name, kStubSection, sizeof kStubSection) == 0) {
            start = base + section->VirtualAddress;
            size  = section->Misc.VirtualSize;
            return true;
        }
    }
    return false;
}

// "C:\Games\MX Simulator\" for "C:\Games\MX Simulator\opengl32.dll".
bool ModuleDirectory(HMODULE module, wchar_t* out, DWORD size)
{
    const DWORD length = GetModuleFileNameW(module, out, size);
    if (length == 0 || length >= size)
        return false;
    wchar_t* slash = wcsrchr(out, L'\\');
    if (!slash)
        return false;
    slash[1] = L'\0';
    return true;
}

bool InitializeProxy(HMODULE self)
{
    wchar_t dir[1024];
    wchar_t path[1024];
    if (ModuleDirectory(self, dir, _countof(dir))) {
        swprintf_s(path, L"%smxcam.log", dir);
        LogOpen(path);
        Log("MXCam opengl32 proxy loaded (%s build)", sizeof(void*) == 8 ? "x64" : "x86");
        swprintf_s(path, L"%smxcam.ini", dir);
        LoadConfig(path);
    }

    // Load the genuine DLL by full path - a bare "opengl32.dll" would resolve back to us.
    // GetSystemDirectory returns System32 for 32-bit processes too; WOW64 file-system
    // redirection then transparently maps that to SysWOW64.
    wchar_t systemDir[MAX_PATH];
    const UINT systemDirLength = GetSystemDirectoryW(systemDir, MAX_PATH);
    if (systemDirLength == 0 || systemDirLength >= MAX_PATH) {
        Log("fatal: GetSystemDirectoryW failed (%lu)", GetLastError());
        return false;
    }
    swprintf_s(path, L"%s\\opengl32.dll", systemDir);

    // LoadLibrary inside DllMain is normally discouraged (it runs under the loader lock), but it
    // is the standard technique for proxy DLLs and is safe here: opengl32 only depends on core
    // system DLLs that are already loaded.
    const HMODULE realGL = LoadLibraryW(path);
    if (!realGL || realGL == self) {
        Log("fatal: could not load the system OpenGL library %ls (error %lu)", path, GetLastError());
        return false;
    }
    g_realOpenGL = realGL;

    BYTE*  stubs = nullptr;
    SIZE_T stubsSize = 0;
    DWORD  oldProtect = 0;
    if (!FindStubSection(self, stubs, stubsSize) ||
        !VirtualProtect(stubs, stubsSize, PAGE_EXECUTE_READWRITE, &oldProtect)) {
        Log("fatal: could not unprotect the export stubs (error %lu)", GetLastError());
        return false;
    }

    size_t hookCount = 0;
    const HookEntry* hooks = GetHookTable(hookCount);

    int hooked = 0;
    int missing = 0;
    for (const ExportStub& entry : kExports) {
        const FARPROC real = GetProcAddress(realGL, entry.name);
        const void* target = reinterpret_cast<const void*>(real);

        for (size_t i = 0; i < hookCount; ++i) {
            if (strcmp(hooks[i].name, entry.name) != 0)
                continue;
            *hooks[i].realSlot = reinterpret_cast<void*>(real);
            if (real) {
                target = hooks[i].hook;
                ++hooked;
            }
        }

        if (!target) {
            Log("warning: %s is missing from the system opengl32.dll", entry.name);
            target = reinterpret_cast<const void*>(&UnresolvedExport);
            ++missing;
        }
        WriteJump(entry.code, target);
    }
    VirtualProtect(stubs, stubsSize, PAGE_EXECUTE_READ, &oldProtect);
    FlushInstructionCache(GetCurrentProcess(), stubs, stubsSize);

    Log("forwarding to %ls: %d exports, %d hooked, %d missing",
        path, static_cast<int>(_countof(kExports)), hooked, missing);
    return true;
}

}  // namespace

HMODULE RealOpenGLModule() { return g_realOpenGL; }

}  // namespace mxcam

BOOL WINAPI DllMain(HINSTANCE instance, DWORD reason, LPVOID /*reserved*/)
{
    if (reason == DLL_PROCESS_ATTACH) {
        DisableThreadLibraryCalls(instance);
        return mxcam::InitializeProxy(instance) ? TRUE : FALSE;
    }
    if (reason == DLL_PROCESS_DETACH)
        mxcam::LogClose();
    // The system opengl32.dll is deliberately never freed: driver threads may still be running
    // inside it while the process shuts down.
    return TRUE;
}
