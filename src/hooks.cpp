// hooks.cpp
//   The intercepted OpenGL entry points.
//
//   How the camera is found
//   -----------------------
//   We mirror just enough of the fixed-function matrix state to know, at every modelview
//   load, whether a perspective projection is active (3D scene) or not (HUD / menus / 2D).
//   Each frame, the modelview loads made during perspective passes are numbered 0, 1, 2...
//   Load number ViewMatrixIndex (mxcam.ini) is taken to be the camera's view matrix. Use the
//   frame dump (F10) to see what the game actually does and pick the right index.
//
//   How it is modified
//   ------------------
//   camera::Update() turns that raw view into an eye-space correction C. Every later
//   perspective-pass modelview load that frame is replaced with C * M - correct whether the
//   engine loads the bare view (then glMultMatrix's objects onto it) or loads a full
//   View*Model per object, because C * View * Model == NewView * Model.
//
//   A "load" is glLoadMatrixf/d, or glMultMatrixf/d onto a freshly loaded identity (which
//   is equivalent, and is how glLoadIdentity(); glMultMatrixf(view); engines set a camera).

#include <windows.h>
#include <GL/gl.h>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>

#include "camera.h"
#include "config.h"
#include "hooks.h"
#include "log.h"
#include "overlay.h"

namespace mxcam {
namespace {

// ===========================================================================
// Real driver entry points, filled in by the proxy loader before any game code runs.
// decltype(&::glFoo) takes the exact signature and calling convention from <GL/gl.h>
// without importing the symbol.
// ===========================================================================
using PFN_wglSwapBuffers = BOOL(WINAPI*)(HDC);  // exported by opengl32 but not declared in the SDK

decltype(&::glMatrixMode)        real_glMatrixMode        = nullptr;
decltype(&::glLoadIdentity)      real_glLoadIdentity      = nullptr;
decltype(&::glLoadMatrixf)       real_glLoadMatrixf       = nullptr;
decltype(&::glLoadMatrixd)       real_glLoadMatrixd       = nullptr;
decltype(&::glMultMatrixf)       real_glMultMatrixf       = nullptr;
decltype(&::glMultMatrixd)       real_glMultMatrixd       = nullptr;
decltype(&::glPushMatrix)        real_glPushMatrix        = nullptr;
decltype(&::glPopMatrix)         real_glPopMatrix         = nullptr;
decltype(&::glFrustum)           real_glFrustum           = nullptr;
decltype(&::glOrtho)             real_glOrtho             = nullptr;
decltype(&::glRotatef)           real_glRotatef           = nullptr;
decltype(&::glRotated)           real_glRotated           = nullptr;
decltype(&::glTranslatef)        real_glTranslatef        = nullptr;
decltype(&::glTranslated)        real_glTranslated        = nullptr;
decltype(&::glScalef)            real_glScalef            = nullptr;
decltype(&::glScaled)            real_glScaled            = nullptr;
decltype(&::glNewList)           real_glNewList           = nullptr;
decltype(&::glEndList)           real_glEndList           = nullptr;
decltype(&::glClear)             real_glClear             = nullptr;
PFN_wglSwapBuffers               real_wglSwapBuffers      = nullptr;
decltype(&::wglSwapLayerBuffers) real_wglSwapLayerBuffers = nullptr;
decltype(&::wglGetProcAddress)   real_wglGetProcAddress   = nullptr;

// ===========================================================================
// Mirrored GL state
// ===========================================================================
constexpr int kStackDepth   = 64;    // >= any driver's modelview / projection stack depth
constexpr int kMaxDumpLines = 4000;  // per frame dump

struct MatrixState {
    GLenum mode     = GL_MODELVIEW;
    GLenum listMode = 0;             // GL_COMPILE / GL_COMPILE_AND_EXECUTE inside glNewList..glEndList
    bool   projIsPerspective[kStackDepth] = {};
    int    projTop = 0;
    bool   mvIsIdentity[kStackDepth] = {};  // "top of modelview is a fresh glLoadIdentity"
    int    mvTop = 0;
};

struct FrameState {
    int    sceneLoadIndex = 0;       // modelview loads seen this frame during perspective passes
    bool   captured       = false;   // the camera load was reached this frame
    bool   haveCorrection = false;   // ...and camera::Update() accepted it
    double correction[16] = {};
    double lastOutput[16] = {};      // the last matrix we substituted (see MatchesLastOutput)
    bool   haveLastOutput = false;
    bool   dumping        = false;
    int    dumpLines      = 0;
};

MatrixState g_gl;
FrameState  g_frame;
bool        g_dumpRequested       = false;
int         g_framesWithoutCamera = 0;
ULONGLONG   g_lastSwapTick        = 0;

// GL_COMPILE records commands into a display list without executing them, so they must not
// change our mirror of the live state.
bool Executes() { return g_gl.listMode != GL_COMPILE; }
bool& ProjIsPerspective() { return g_gl.projIsPerspective[g_gl.projTop]; }
bool& MvIsIdentity() { return g_gl.mvIsIdentity[g_gl.mvTop]; }

// A perspective projection makes clip-space w depend on z (m[11] != 0) with no constant
// w term (m[15] == 0); orthographic and identity projections have m[15] == 1.
template <class T>
bool IsPerspective(const T* m)
{
    return std::fabs(double(m[15])) < 1e-6 && std::fabs(double(m[11])) > 1e-6;
}

void LoadReal(const GLfloat* m)  { real_glLoadMatrixf(m); }
void LoadReal(const GLdouble* m) { real_glLoadMatrixd(m); }
void MultReal(const GLfloat* m)  { real_glMultMatrixf(m); }
void MultReal(const GLdouble* m) { real_glMultMatrixd(m); }

// ===========================================================================
// Frame dump (diagnostics): logs every matrix call of one frame
// ===========================================================================
const char* ModeName(GLenum mode)
{
    switch (mode) {
    case GL_MODELVIEW:  return "MODELVIEW";
    case GL_PROJECTION: return "PROJECTION";
    case GL_TEXTURE:    return "TEXTURE";
    default:            return "other";
    }
}

}  // namespace

bool IsDumping() { return g_frame.dumping && g_frame.dumpLines < kMaxDumpLines; }

bool GameHasFocus()
{
    DWORD pid = 0;
    GetWindowThreadProcessId(GetForegroundWindow(), &pid);
    return pid == GetCurrentProcessId();
}

void* GetRealGLFunction(const char* name)
{
    if (void* exported = reinterpret_cast<void*>(GetProcAddress(RealOpenGLModule(), name)))
        return exported;
    if (!real_wglGetProcAddress)
        return nullptr;
    // Some drivers return small sentinel values instead of NULL for unsupported functions.
    const auto address = reinterpret_cast<INT_PTR>(real_wglGetProcAddress(name));
    return (address >= -1 && address <= 3) ? nullptr : reinterpret_cast<void*>(address);
}

void DumpCall(const char* format, ...)
{
    if (!g_frame.dumping || g_frame.dumpLines++ >= kMaxDumpLines)
        return;
    char text[512];
    va_list args;
    va_start(args, format);
    vsnprintf(text, sizeof text, format, args);
    va_end(args);
    Log("  %-10s %-5s %s", ModeName(g_gl.mode), ProjIsPerspective() ? "3D" : "2D", text);
}

namespace {

template <class T>
void DumpMatrix(const char* call, const T* m)
{
    // Printed row by row; the 4th column is the translation.
    DumpCall("%s [% .3f % .3f % .3f % .3f | % .3f % .3f % .3f % .3f | % .3f % .3f % .3f % .3f | % .3f % .3f % .3f % .3f]",
             call, m[0], m[4], m[8], m[12], m[1], m[5], m[9], m[13],
             m[2], m[6], m[10], m[14], m[3], m[7], m[11], m[15]);
}

// ===========================================================================
// Camera capture & correction
// ===========================================================================

// Some engines read the modelview back (glGetFloatv) and load it again later. If that
// matrix is one we produced, it is already corrected and must not be corrected twice.
template <class T>
bool MatchesLastOutput(const T* m)
{
    if (!g_frame.haveLastOutput)
        return false;
    for (int i = 0; i < 16; ++i) {
        const double expected = g_frame.lastOutput[i];
        if (std::fabs(double(m[i]) - expected) > 1e-5 * (1.0 + std::fabs(expected)))
            return false;
    }
    return true;
}

// The engine is about to replace the whole modelview matrix with m.
template <class T>
void SetModelview(const T* m)
{
    const Config& cfg = GetConfig();
    if (!cfg.enabled || !ProjIsPerspective() || g_gl.listMode != 0) {
        LoadReal(m);
        return;
    }

    const int index = g_frame.sceneLoadIndex++;
    if (index == cfg.viewMatrixIndex) {
        double view[16];
        for (int i = 0; i < 16; ++i)
            view[i] = double(m[i]);
        g_frame.captured = true;
        g_frame.haveCorrection = camera::Update(view, g_frame.correction);
        if (g_frame.haveCorrection) {
            const camera::Telemetry& t = camera::LastTelemetry();
            DumpCall("^ scene load #%d = CAMERA  dt=%.1fms lag=%.2fdeg roll=%.2fdeg rollCorr=%.2fdeg "
                     "accelUp=%.2fg neck=%.1fmm eyeHeight%+.1fmm",
                     index, t.dtMs, t.lagDeg, t.rawRollDeg, t.rollCorrectionDeg, t.verticalAccelG,
                     t.neckOffset * 1000.0, t.verticalOffset * 1000.0);
        } else {
            DumpCall("^ scene load #%d = CAMERA, rejected (not a rigid view matrix)", index);
        }
    } else {
        DumpCall("^ scene load #%d", index);
    }

    if (!g_frame.haveCorrection || MatchesLastOutput(m)) {
        LoadReal(m);
        return;
    }

    // out = C * m, accumulated in double precision.
    T out[16];
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
            double sum = 0;
            for (int k = 0; k < 4; ++k)
                sum += g_frame.correction[k * 4 + r] * double(m[c * 4 + k]);
            out[c * 4 + r] = T(sum);
            g_frame.lastOutput[c * 4 + r] = double(out[c * 4 + r]);
        }
    }
    g_frame.haveLastOutput = true;
    LoadReal(out);
}

// ===========================================================================
// Frame boundary & hotkeys
// ===========================================================================
bool Pressed(int vk, bool& wasDown)
{
    const bool down = vk > 0 && (GetAsyncKeyState(vk) & 0x8000) != 0;
    const bool pressed = down && !wasDown;
    wasDown = down;
    return pressed;
}

void PollHotkeys()
{
    static bool toggleDown = false, reloadDown = false, dumpDown = false, traceDown = false;
    const Config& cfg = GetConfig();
    // Always sample all keys so the edge detection stays in sync while unfocused.
    const bool toggle = Pressed(cfg.toggleKey, toggleDown);
    const bool reload = Pressed(cfg.reloadKey, reloadDown);
    const bool dump   = Pressed(cfg.dumpKey, dumpDown);
    const bool trace  = Pressed(cfg.traceKey, traceDown);
    if (!GameHasFocus())
        return;

    if (toggle) {
        SetEnabled(!cfg.enabled);
        camera::Reset();
        Log("hotkey: helmet cam %s", cfg.enabled ? "ON" : "OFF");
    }
    if (reload) {
        ReloadConfig();
        camera::Reset();
    }
    if (dump)
        g_dumpRequested = true;
    if (trace)
        StartTrace(10.0);
}

void EndFrame()
{
    const Config& cfg = GetConfig();
    const bool shaderCaptured = EndShaderFrame();
    if (g_frame.dumping)
        Log("==== end of frame dump: %d scene modelview loads ====", g_frame.sceneLoadIndex);

    if (cfg.enabled && !g_frame.captured && !shaderCaptured) {
        if (++g_framesWithoutCamera == 600) {
            Log("note: no camera matrix found for 600 frames (menu, replay, or detection settings?). "
                "Press the dump key in first person to inspect a frame.");
        }
    } else {
        g_framesWithoutCamera = 0;
    }

    g_frame = FrameState{};
    PollHotkeys();
    if (g_dumpRequested) {
        g_dumpRequested = false;
        g_frame.dumping = true;
        Log("==== frame dump begin (ViewMatrixIndex=%d) ====", cfg.viewMatrixIndex);
    }
}

// ===========================================================================
// Hooks
// ===========================================================================
void APIENTRY Hook_glMatrixMode(GLenum mode)
{
    if (Executes())
        g_gl.mode = mode;
    DumpCall("glMatrixMode(%s)", ModeName(mode));
    real_glMatrixMode(mode);
}

void APIENTRY Hook_glLoadIdentity()
{
    if (Executes()) {
        if (g_gl.mode == GL_MODELVIEW)
            MvIsIdentity() = true;
        else if (g_gl.mode == GL_PROJECTION)
            ProjIsPerspective() = false;
    }
    DumpCall("glLoadIdentity()");
    real_glLoadIdentity();
}

template <class T>
void OnLoadMatrix(const T* m, const char* name)
{
    if (Executes()) {
        if (g_gl.mode == GL_PROJECTION)
            ProjIsPerspective() = IsPerspective(m);
        else if (g_gl.mode == GL_MODELVIEW)
            MvIsIdentity() = false;
    }
    DumpMatrix(name, m);
    if (g_gl.mode == GL_MODELVIEW && g_gl.listMode == 0)
        SetModelview(m);
    else
        LoadReal(m);
}

template <class T>
void OnMultMatrix(const T* m, const char* name)
{
    bool ontoIdentity = false;
    if (Executes()) {
        if (g_gl.mode == GL_PROJECTION) {
            if (IsPerspective(m))
                ProjIsPerspective() = true;
        } else if (g_gl.mode == GL_MODELVIEW) {
            ontoIdentity = MvIsIdentity();
            MvIsIdentity() = false;
        }
    }
    DumpMatrix(name, m);
    // Identity * M == M: a multiply onto a fresh identity is really a load.
    if (ontoIdentity && g_gl.listMode == 0)
        SetModelview(m);
    else
        MultReal(m);
}

void APIENTRY Hook_glLoadMatrixf(const GLfloat* m)  { OnLoadMatrix(m, "glLoadMatrixf"); }
void APIENTRY Hook_glLoadMatrixd(const GLdouble* m) { OnLoadMatrix(m, "glLoadMatrixd"); }
void APIENTRY Hook_glMultMatrixf(const GLfloat* m)  { OnMultMatrix(m, "glMultMatrixf"); }
void APIENTRY Hook_glMultMatrixd(const GLdouble* m) { OnMultMatrix(m, "glMultMatrixd"); }

void APIENTRY Hook_glPushMatrix()
{
    if (Executes()) {
        if (g_gl.mode == GL_PROJECTION && g_gl.projTop + 1 < kStackDepth) {
            g_gl.projIsPerspective[g_gl.projTop + 1] = ProjIsPerspective();
            ++g_gl.projTop;
        } else if (g_gl.mode == GL_MODELVIEW && g_gl.mvTop + 1 < kStackDepth) {
            g_gl.mvIsIdentity[g_gl.mvTop + 1] = MvIsIdentity();
            ++g_gl.mvTop;
        }
    }
    DumpCall("glPushMatrix()");
    real_glPushMatrix();
}

void APIENTRY Hook_glPopMatrix()
{
    if (Executes()) {
        if (g_gl.mode == GL_PROJECTION && g_gl.projTop > 0)
            --g_gl.projTop;
        else if (g_gl.mode == GL_MODELVIEW && g_gl.mvTop > 0)
            --g_gl.mvTop;
    }
    DumpCall("glPopMatrix()");
    real_glPopMatrix();
}

void APIENTRY Hook_glFrustum(GLdouble l, GLdouble r, GLdouble b, GLdouble t, GLdouble n, GLdouble f)
{
    if (Executes() && g_gl.mode == GL_PROJECTION)
        ProjIsPerspective() = true;
    DumpCall("glFrustum(%g, %g, %g, %g, %g, %g)", l, r, b, t, n, f);
    real_glFrustum(l, r, b, t, n, f);
}

void APIENTRY Hook_glOrtho(GLdouble l, GLdouble r, GLdouble b, GLdouble t, GLdouble n, GLdouble f)
{
    DumpCall("glOrtho(%g, %g, %g, %g, %g, %g)", l, r, b, t, n, f);
    real_glOrtho(l, r, b, t, n, f);
}

// Rotate / translate / scale multiply onto the stack, so its top is no longer a plain identity.
void NoteModelviewMultiplied()
{
    if (Executes() && g_gl.mode == GL_MODELVIEW)
        MvIsIdentity() = false;
}

void APIENTRY Hook_glRotatef(GLfloat a, GLfloat x, GLfloat y, GLfloat z)
{
    NoteModelviewMultiplied();
    DumpCall("glRotatef(%g, %g, %g, %g)", a, x, y, z);
    real_glRotatef(a, x, y, z);
}

void APIENTRY Hook_glRotated(GLdouble a, GLdouble x, GLdouble y, GLdouble z)
{
    NoteModelviewMultiplied();
    DumpCall("glRotated(%g, %g, %g, %g)", a, x, y, z);
    real_glRotated(a, x, y, z);
}

void APIENTRY Hook_glTranslatef(GLfloat x, GLfloat y, GLfloat z)
{
    NoteModelviewMultiplied();
    DumpCall("glTranslatef(%g, %g, %g)", x, y, z);
    real_glTranslatef(x, y, z);
}

void APIENTRY Hook_glTranslated(GLdouble x, GLdouble y, GLdouble z)
{
    NoteModelviewMultiplied();
    DumpCall("glTranslated(%g, %g, %g)", x, y, z);
    real_glTranslated(x, y, z);
}

void APIENTRY Hook_glScalef(GLfloat x, GLfloat y, GLfloat z)
{
    NoteModelviewMultiplied();
    DumpCall("glScalef(%g, %g, %g)", x, y, z);
    real_glScalef(x, y, z);
}

void APIENTRY Hook_glScaled(GLdouble x, GLdouble y, GLdouble z)
{
    NoteModelviewMultiplied();
    DumpCall("glScaled(%g, %g, %g)", x, y, z);
    real_glScaled(x, y, z);
}

// Matrix commands recorded into a display list are left untouched: a corrected matrix baked
// into a list would replay this frame's correction forever.
void APIENTRY Hook_glNewList(GLuint list, GLenum mode)
{
    g_gl.listMode = mode;
    DumpCall("glNewList(%u, 0x%X)", list, mode);
    real_glNewList(list, mode);
}

void APIENTRY Hook_glEndList()
{
    g_gl.listMode = 0;
    DumpCall("glEndList()");
    real_glEndList();
}

// Fallback frame boundary for games whose buffer swap bypasses opengl32!wglSwapBuffers:
// if no swap has been seen for a second, a colour clear after the camera starts a new frame.
void APIENTRY Hook_glClear(GLbitfield mask)
{
    if ((mask & GL_COLOR_BUFFER_BIT) && g_frame.captured && GetTickCount64() - g_lastSwapTick > 1000)
        EndFrame();
    DumpCall("glClear(0x%X)", mask);
    real_glClear(mask);
}

// The settings panel is drawn into the finished frame, right before it is presented.
BOOL WINAPI Hook_wglSwapBuffers(HDC dc)
{
    OverlayFrame(dc);
    g_lastSwapTick = GetTickCount64();
    EndFrame();
    return real_wglSwapBuffers(dc);
}

BOOL WINAPI Hook_wglSwapLayerBuffers(HDC dc, UINT planes)
{
    if (planes & WGL_SWAP_MAIN_PLANE) {
        OverlayFrame(dc);
        g_lastSwapTick = GetTickCount64();
        EndFrame();
    }
    return real_wglSwapLayerBuffers(dc, planes);
}

// Everything past OpenGL 1.1 (shaders, uniforms, buffers) is fetched through here.
PROC WINAPI Hook_wglGetProcAddress(LPCSTR name)
{
    const PROC real = real_wglGetProcAddress(name);
    return reinterpret_cast<PROC>(WrapProc(name, reinterpret_cast<void*>(real)));
}

}  // namespace

#define MXCAM_HOOK(fn) { #fn, reinterpret_cast<void*>(&Hook_##fn), reinterpret_cast<void**>(&real_##fn) }

const HookEntry* GetHookTable(size_t& count)
{
    static const HookEntry kHooks[] = {
        MXCAM_HOOK(glMatrixMode),   MXCAM_HOOK(glLoadIdentity),
        MXCAM_HOOK(glLoadMatrixf),  MXCAM_HOOK(glLoadMatrixd),
        MXCAM_HOOK(glMultMatrixf),  MXCAM_HOOK(glMultMatrixd),
        MXCAM_HOOK(glPushMatrix),   MXCAM_HOOK(glPopMatrix),
        MXCAM_HOOK(glFrustum),      MXCAM_HOOK(glOrtho),
        MXCAM_HOOK(glRotatef),      MXCAM_HOOK(glRotated),
        MXCAM_HOOK(glTranslatef),   MXCAM_HOOK(glTranslated),
        MXCAM_HOOK(glScalef),       MXCAM_HOOK(glScaled),
        MXCAM_HOOK(glNewList),      MXCAM_HOOK(glEndList),
        MXCAM_HOOK(glClear),
        MXCAM_HOOK(wglSwapBuffers), MXCAM_HOOK(wglSwapLayerBuffers),
        MXCAM_HOOK(wglGetProcAddress),
    };
    count = _countof(kHooks);
    return kHooks;
}

}  // namespace mxcam
