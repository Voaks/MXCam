// hooks.h
//   Interface between the generic proxy (proxy.cpp) and the intercepted GL functions (hooks.cpp).
#pragma once

#include <windows.h>
#include <cstddef>

namespace mxcam {

// The genuine system opengl32.dll (loaded by proxy.cpp).
HMODULE RealOpenGLModule();

// A driver entry point that bypasses all of our hooks: opengl32 exports first, then the real
// wglGetProcAddress. Needs a current GL context for anything past OpenGL 1.1.
void* GetRealGLFunction(const char* name);

// True when the game window is the foreground window (hotkeys are ignored otherwise).
bool GameHasFocus();

struct HookEntry {
    const char* name;      // export name, e.g. "glLoadMatrixf"
    void*       hook;      // our replacement, which the export stub will jump to
    void**      realSlot;  // receives the genuine function from the system opengl32.dll
};

// The table of exports hooks.cpp wants to intercept. The proxy fills every realSlot before
// pointing the export at the hook, so a hook can always call its real function.
const HookEntry* GetHookTable(size_t& count);

// Frame dump, shared with the extension wrappers in proc_hooks.cpp.
bool IsDumping();
void DumpCall(const char* format, ...);

// Called for every function the game fetches through wglGetProcAddress (OpenGL 2.0+ and
// extensions). Returns our wrapper for the ones we intercept, otherwise the real pointer.
void* WrapProc(const char* name, void* real);

// Called once per frame by the swap hook. Returns true if the shader path found the camera
// this frame.
bool EndShaderFrame();

}  // namespace mxcam
