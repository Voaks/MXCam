// proc_hooks.cpp
//   Wrappers for OpenGL 2.0+ functions. Games don't import these from opengl32.dll; they fetch
//   them at startup through wglGetProcAddress, which hooks.cpp routes through WrapProc().
//
//   This is the path MX Simulator actually renders through. Every object is drawn with a shader
//   whose 'u_modelview' uniform (Config::modelviewUniform) receives View * Model, computed on the
//   CPU and uploaded with glUniformMatrix4fv. Per frame:
//     - the terrain shader (the program that has Config::cameraProgramUniform, 'u_heights') is
//       drawn with Model = identity, so its first u_modelview upload is the camera view matrix;
//       it feeds camera::Update(), which yields the eye-space correction C;
//     - every rigid u_modelview upload is replaced with C * M - this covers the world and the
//       rider's own bike;
//     - the sky is drawn before the terrain with a rotation-only view. That rotation is enough
//       to preview this frame's view orientation (camera::PreviewRotation), and the sky gets
//       only the rotation part of C, so eye offsets never move it. Other rotation-only draws
//       can come first (tens of degrees off the camera), so the one closest to the last camera
//       rotation is taken as the sky; a preview never changes the filter state;
//     - uploads that aren't rigid (HUD elements are scaled down or flattened) or are exactly
//       identity (2D screens) are left alone.
//     - lighting is computed in eye space, so the light direction ('u_light_position',
//       Config::lightUniform) arrives already turned by the game's view. It gets the rotation
//       part of C too, or turning the view would turn the lighting with it;
//     - reflections look up an environment map through 'u_envmat' (eye -> world rotation, the
//       transpose of the view's), which becomes u_envmat * C^T so reflections stay put.
//   TraceKey (F11) records a per-frame trace (mxcam_trace.csv) for diagnosing jerks.
//   Uniform names are logged as the game looks them up, and every upload goes into the F10 dump.

#include <windows.h>
#include <GL/gl.h>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <new>
#include <share.h>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "bike.h"
#include "camera.h"
#include "config.h"
#include "hooks.h"
#include "log.h"

namespace mxcam {
namespace {

using GLchar = char;
using PFN_UseProgram              = void (APIENTRY*)(GLuint program);
using PFN_GetUniformLocation      = GLint (APIENTRY*)(GLuint program, const GLchar* name);
using PFN_UniformMatrix4fv        = void (APIENTRY*)(GLint location, GLsizei count, GLboolean transpose, const GLfloat* value);
using PFN_ProgramUniformMatrix4fv = void (APIENTRY*)(GLuint program, GLint location, GLsizei count, GLboolean transpose, const GLfloat* value);
using PFN_Uniform4fv              = void (APIENTRY*)(GLint location, GLsizei count, const GLfloat* value);
using PFN_Uniform3fv              = PFN_Uniform4fv;
using PFN_UniformMatrix3fv        = PFN_UniformMatrix4fv;

// Real driver functions, filled in by WrapProc() when the game first asks for them.
// The ARB variants have the same signatures (GLhandleARB is an unsigned int on Windows).
PFN_UseProgram              real_glUseProgram              = nullptr;
PFN_UseProgram              real_glUseProgramObjectARB     = nullptr;
PFN_GetUniformLocation      real_glGetUniformLocation      = nullptr;
PFN_GetUniformLocation      real_glGetUniformLocationARB   = nullptr;
PFN_UniformMatrix4fv        real_glUniformMatrix4fv        = nullptr;
PFN_UniformMatrix4fv        real_glUniformMatrix4fvARB     = nullptr;
PFN_ProgramUniformMatrix4fv real_glProgramUniformMatrix4fv = nullptr;
PFN_Uniform4fv              real_glUniform4fv              = nullptr;
PFN_Uniform4fv              real_glUniform4fvARB           = nullptr;
PFN_Uniform3fv              real_glUniform3fv              = nullptr;
PFN_Uniform3fv              real_glUniform3fvARB           = nullptr;
PFN_UniformMatrix3fv        real_glUniformMatrix3fv        = nullptr;
PFN_UniformMatrix3fv        real_glUniformMatrix3fvARB     = nullptr;

constexpr int kMaxLookupsLogged = 500;

SRWLOCK g_lock = SRWLOCK_INIT;
std::unordered_set<std::string> g_fetched;                       // names already logged
std::unordered_map<unsigned long long, std::string> g_uniforms;  // (program << 32 | location) -> name
std::unordered_set<unsigned long long> g_modelviewUniforms;      // (program, location) of each u_modelview
std::unordered_set<unsigned long long> g_lightUniforms;          // (program, location) of each light direction
std::unordered_set<unsigned long long> g_envUniforms;            // (program, location) of each environment-map matrix
std::unordered_set<GLuint> g_cameraPrograms;                     // programs that draw the terrain
int     g_lookupsLogged = 0;
GLuint  g_program = 0;                                           // program bound by glUseProgram

struct ShaderCamera {
    bool   haveCorrection    = false;  // the camera was found (this frame, or last frame until it is)
    double correction[16]    = {};     // full eye-space correction, for everything with a position
    double rotationOnly[16]  = {};     // its rotation alone, for the sky (this frame's preview once there is one)
    bool   capturedThisFrame = false;
    bool   havePreview       = false;  // rotationOnly is this frame's preview
    double skyAngle          = 0;      // how far the chosen sky is from the last camera rotation
    int    previews          = 0;      // rotation-only draws taken as the sky this frame
    int    otherRotations    = 0;      // rotation-only draws that were not the camera
    int    correctedThisFrame = 0;
};
ShaderCamera g_camera;

// Rotation-only draws further than this from the last camera rotation are not the sky.
constexpr double kMaxSkyAngleDeg = 45.0;

// Per-frame trace (TraceKey). Every frame goes into a ring buffer, so the key saves the seconds
// *before* it was pressed (the moment something looked wrong) and then keeps recording a while.
constexpr int    kTraceRing    = 4096;  // frames; covers kTraceHistory at up to ~270 fps
constexpr double kTraceHistory = 15.0;  // seconds kept before the key press
struct TraceLine {
    double time = 0;
    char   text[512];
};
TraceLine* g_traceRing   = nullptr;     // allocated on first use
int    g_traceHead       = 0;
int    g_traceCount      = 0;
FILE*  g_trace           = nullptr;
int    g_traceFrames     = 0;
double g_traceStart      = -1;          // content time of the key press (-1: at the next frame)
double g_traceEnd        = 0;

unsigned long long UniformKey(GLuint program, GLint location)
{
    return (static_cast<unsigned long long>(program) << 32) | static_cast<unsigned int>(location);
}

void NoteUseProgram(GLuint program) { g_program = program; }

void NoteUniformLocation(GLuint program, const GLchar* name, GLint location)
{
    if (!name || location < 0)
        return;
    const Config& cfg = GetConfig();
    const unsigned long long key = UniformKey(program, location);

    AcquireSRWLockExclusive(&g_lock);
    const bool added = g_uniforms.emplace(key, name).second;
    if (strcmp(name, cfg.modelviewUniform) == 0)
        g_modelviewUniforms.insert(key);
    if (strcmp(name, cfg.lightUniform) == 0)
        g_lightUniforms.insert(key);
    if (strcmp(name, cfg.envUniform) == 0)
        g_envUniforms.insert(key);
    if (strcmp(name, cfg.cameraProgramUniform) == 0)
        g_cameraPrograms.insert(program);
    const bool log = added && g_lookupsLogged < kMaxLookupsLogged;
    if (log)
        ++g_lookupsLogged;
    ReleaseSRWLockExclusive(&g_lock);

    if (log)
        Log("uniform: program %u '%s' = location %d", program, name, location);
}

std::string UniformName(GLuint program, GLint location)
{
    AcquireSRWLockShared(&g_lock);
    const auto found = g_uniforms.find(UniformKey(program, location));
    std::string name = found != g_uniforms.end() ? found->second : "?";
    ReleaseSRWLockShared(&g_lock);
    return name;
}

// Logs the first few vec4s / the first matrix of an upload, as stored in memory
// (column-major unless transpose is set).
void DumpUniform(const char* call, GLuint program, GLint location, GLsizei count, int transpose,
                 const GLfloat* value, int floatsPerElement)
{
    if (!IsDumping() || !value || count <= 0)
        return;
    const int floats = count * floatsPerElement < 16 ? count * floatsPerElement : 16;
    char numbers[256] = "";
    int length = 0;
    for (int i = 0; i < floats && length < static_cast<int>(sizeof numbers) - 16; ++i)
        length += snprintf(numbers + length, sizeof numbers - length, "%s% .3f", i && i % (floatsPerElement % 3 == 0 ? 3 : 4) == 0 ? " |" : "", value[i]);
    DumpCall("%s prog=%u loc=%d '%s' count=%d%s [%s]", call, program, location,
             UniformName(program, location).c_str(), count, transpose ? " TRANSPOSED" : "", numbers);
}

// ---- Camera correction on u_modelview uploads -----------------------------------------------

// A rotation + translation with unit scale: world objects, the bike, the sky. HUD elements are
// scaled to fractions of a unit (or flattened), so they fail this test and stay screen-fixed.
bool IsRigid(const GLfloat* m)
{
    if (std::fabs(m[3]) > 1e-4f || std::fabs(m[7]) > 1e-4f || std::fabs(m[11]) > 1e-4f || std::fabs(m[15] - 1.0f) > 1e-4f)
        return false;
    for (int a = 0; a < 3; ++a) {
        for (int b = a; b < 3; ++b) {
            const float dot = m[a * 4] * m[b * 4] + m[a * 4 + 1] * m[b * 4 + 1] + m[a * 4 + 2] * m[b * 4 + 2];
            if (std::fabs(dot - (a == b ? 1.0f : 0.0f)) > 0.02f)
                return false;
        }
    }
    return true;
}

bool IsIdentity(const GLfloat* m)
{
    for (int i = 0; i < 16; ++i) {
        if (m[i] != ((i % 5 == 0) ? 1.0f : 0.0f))
            return false;
    }
    return true;
}

bool HasTranslation(const GLfloat* m)
{
    return std::fabs(m[12]) + std::fabs(m[13]) + std::fabs(m[14]) > 1e-3f;
}

// A rotation-only view drawn before the camera is known this frame: if it is the closest yet to
// the last camera rotation (the sky), preview this frame's orientation from it. Anything further
// off - clouds, a sun billboard, a reflection pass - takes whatever correction is current.
void ConsiderSkyRotation(const double view[16])
{
    if (g_camera.capturedThisFrame || !g_camera.haveCorrection)
        return;
    const double angle = camera::AngleToLastCameraDeg(view);
    if (g_camera.havePreview && std::fabs(angle - g_camera.skyAngle) < 0.01)
        return;  // the same rotation again (u_envmat, then the sky): already previewed
    if (angle < 0 || angle > kMaxSkyAngleDeg || (g_camera.havePreview && angle >= g_camera.skyAngle)) {
        ++g_camera.otherRotations;
        DumpCall("^ rotation only, %.1f deg from the camera: not taken as the sky", angle);
        return;
    }
    if (camera::PreviewRotation(view, g_camera.rotationOnly)) {
        g_camera.havePreview = true;
        g_camera.skyAngle = angle;
        ++g_camera.previews;
        DumpCall("^ SKY (%.2f deg from last frame's camera): orientation previewed for this frame", angle);
    }
}

// Returns the matrix to actually upload: value itself, or scratch holding C * value.
const GLfloat* CorrectModelview(GLuint program, GLint location, GLsizei count, GLboolean transpose,
                                const GLfloat* value, GLfloat* scratch)
{
    const Config& cfg = GetConfig();
    if (!cfg.enabled || !value || count != 1 || transpose)
        return value;

    AcquireSRWLockShared(&g_lock);
    const bool isModelview = g_modelviewUniforms.count(UniformKey(program, location)) != 0;
    const bool knownCamera = !g_cameraPrograms.empty();
    const bool isCamera    = g_cameraPrograms.count(program) != 0;
    ReleaseSRWLockShared(&g_lock);

    if (!isModelview || !IsRigid(value) || IsIdentity(value))
        return value;

    // The terrain program's first upload is the bare view matrix. If the configured terrain
    // uniform was never seen, fall back to the first rigid upload that has a translation.
    if (!g_camera.capturedThisFrame && (knownCamera ? isCamera : HasTranslation(value))) {
        double view[16];
        for (int i = 0; i < 16; ++i)
            view[i] = value[i];
        g_camera.capturedThisFrame = true;
        bike::BeginFrame(view);
        g_camera.haveCorrection = camera::Update(view, g_camera.correction) &&
                                  camera::RotationCorrection(g_camera.rotationOnly);
        if (g_camera.haveCorrection) {
            const camera::Telemetry& t = camera::LastTelemetry();
            DumpCall("^ CAMERA  dt=%.1fms lag=%.2fdeg roll=%.2fdeg rollCorr=%.2fdeg pitchExcess=%.1fdeg "
                     "viewPitch=%.1fdeg lookYaw=%.0fdeg airYaw=%.1fdeg air=%.2f bike=%s %s accelUp=%.2fg "
                     "neck=%.1fmm eyeHeight%+.1fmm",
                     t.dtMs, t.lagDeg, t.rawRollDeg, t.rollCorrectionDeg, t.pitchExcessDeg, t.viewPitchDeg,
                     t.lookYawDeg, t.airYawDeg, t.airborne, t.bikeTracked ? "yes" : "no",
                     t.customActive ? "CUSTOM" : "GAME", t.verticalAccelG, t.neckOffset * 1000.0,
                     t.verticalOffset * 1000.0);
        } else {
            DumpCall("^ CAMERA rejected (not a rigid view matrix)");
        }
    } else if (!HasTranslation(value)) {
        // The sky: a rotation-only view, drawn before the terrain. Its rotation is this frame's
        // raw camera rotation, which is all the orientation half of the camera pipeline needs,
        // so the sky turns in step with this frame's world instead of last frame's. Only the
        // rotation is applied: the sky is infinitely far away, so eye offsets (neck, head
        // position) must not move it.
        double view[16];
        for (int i = 0; i < 16; ++i)
            view[i] = value[i];
        ConsiderSkyRotation(view);
    } else if (g_camera.capturedThisFrame) {
        bike::Observe(program, value);  // raw View * Model: one of the candidates for the bike
    }
    if (!g_camera.haveCorrection)
        return value;

    // scratch = C * value, accumulated in double precision.
    const double* correction = HasTranslation(value) ? g_camera.correction : g_camera.rotationOnly;
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
            double sum = 0;
            for (int k = 0; k < 4; ++k)
                sum += correction[k * 4 + r] * value[c * 4 + k];
            scratch[c * 4 + r] = static_cast<GLfloat>(sum);
        }
    }
    ++g_camera.correctedThisFrame;
    return scratch;
}

// ---- Wrappers ----------------------------------------------------------------------------
void APIENTRY Wrap_glUseProgram(GLuint program)
{
    NoteUseProgram(program);
    DumpCall("glUseProgram(%u)", program);
    real_glUseProgram(program);
}

void APIENTRY Wrap_glUseProgramObjectARB(GLuint program)
{
    NoteUseProgram(program);
    DumpCall("glUseProgramObjectARB(%u)", program);
    real_glUseProgramObjectARB(program);
}

GLint APIENTRY Wrap_glGetUniformLocation(GLuint program, const GLchar* name)
{
    const GLint location = real_glGetUniformLocation(program, name);
    NoteUniformLocation(program, name, location);
    return location;
}

GLint APIENTRY Wrap_glGetUniformLocationARB(GLuint program, const GLchar* name)
{
    const GLint location = real_glGetUniformLocationARB(program, name);
    NoteUniformLocation(program, name, location);
    return location;
}

void APIENTRY Wrap_glUniformMatrix4fv(GLint location, GLsizei count, GLboolean transpose, const GLfloat* value)
{
    DumpUniform("glUniformMatrix4fv", g_program, location, count, transpose, value, 16);
    GLfloat scratch[16];
    real_glUniformMatrix4fv(location, count, transpose, CorrectModelview(g_program, location, count, transpose, value, scratch));
}

void APIENTRY Wrap_glUniformMatrix4fvARB(GLint location, GLsizei count, GLboolean transpose, const GLfloat* value)
{
    DumpUniform("glUniformMatrix4fvARB", g_program, location, count, transpose, value, 16);
    GLfloat scratch[16];
    real_glUniformMatrix4fvARB(location, count, transpose, CorrectModelview(g_program, location, count, transpose, value, scratch));
}

void APIENTRY Wrap_glProgramUniformMatrix4fv(GLuint program, GLint location, GLsizei count, GLboolean transpose, const GLfloat* value)
{
    DumpUniform("glProgramUniformMatrix4fv", program, location, count, transpose, value, 16);
    GLfloat scratch[16];
    real_glProgramUniformMatrix4fv(program, location, count, transpose, CorrectModelview(program, location, count, transpose, value, scratch));
}

// Light directions (and positions) arrive in the game's eye space; returns them turned into
// ours, in scratch, or value itself if this isn't a light uniform. Directions (vec3, or vec4 with
// w = 0) get the rotation only; a vec4 position with w != 0 gets the full correction.
constexpr GLsizei kMaxLights = 16;
const GLfloat* CorrectLight(GLint location, GLsizei count, const GLfloat* value, int components, GLfloat* scratch)
{
    if (!GetConfig().enabled || !value || count <= 0 || count > kMaxLights || !g_camera.haveCorrection)
        return value;
    AcquireSRWLockShared(&g_lock);
    const bool isLight = g_lightUniforms.count(UniformKey(g_program, location)) != 0;
    ReleaseSRWLockShared(&g_lock);
    if (!isLight)
        return value;

    for (GLsizei i = 0; i < count; ++i) {
        const GLfloat* v = value + i * components;
        GLfloat* out = scratch + i * components;
        const double w = components == 4 ? v[3] : 0.0;
        const double* m = w != 0 ? g_camera.correction : g_camera.rotationOnly;
        for (int r = 0; r < 3; ++r)
            out[r] = static_cast<GLfloat>(m[r] * v[0] + m[4 + r] * v[1] + m[8 + r] * v[2] + m[12 + r] * w);
        if (components == 4)
            out[3] = v[3];
    }
    return scratch;
}

// The environment-map matrix turns eye-space reflection vectors into world space: it is the
// transpose of the view rotation. It's uploaded before the sky, so it can also supply this
// frame's camera rotation. Ours: (C R)^T = R^T C^T = u_envmat * C^T.
const GLfloat* CorrectEnvMatrix(GLint location, GLsizei count, GLboolean transpose, const GLfloat* value, GLfloat* scratch)
{
    if (!GetConfig().enabled || !value || count != 1 || transpose)
        return value;
    AcquireSRWLockShared(&g_lock);
    const bool isEnv = g_envUniforms.count(UniformKey(g_program, location)) != 0;
    ReleaseSRWLockShared(&g_lock);
    if (!isEnv)
        return value;

    double view[16] = {};  // R = u_envmat^T as a rotation-only view: R(r, c) = value[r * 3 + c]
    for (int c = 0; c < 3; ++c)
        for (int r = 0; r < 3; ++r)
            view[c * 4 + r] = value[r * 3 + c];
    view[15] = 1;
    ConsiderSkyRotation(view);
    if (!g_camera.haveCorrection)
        return value;
    const double* m = g_camera.capturedThisFrame ? g_camera.correction : g_camera.rotationOnly;
    for (int c = 0; c < 3; ++c) {
        for (int r = 0; r < 3; ++r) {
            double sum = 0;  // (E C^T)(r, c) = sum_k E(r, k) C(c, k)
            for (int k = 0; k < 3; ++k)
                sum += value[k * 3 + r] * m[k * 4 + c];
            scratch[c * 3 + r] = static_cast<GLfloat>(sum);
        }
    }
    return scratch;
}

// Some engines upload matrices as arrays of vec4.
void APIENTRY Wrap_glUniform4fv(GLint location, GLsizei count, const GLfloat* value)
{
    DumpUniform("glUniform4fv", g_program, location, count, 0, value, 4);
    GLfloat scratch[kMaxLights * 4];
    real_glUniform4fv(location, count, CorrectLight(location, count, value, 4, scratch));
}

void APIENTRY Wrap_glUniform4fvARB(GLint location, GLsizei count, const GLfloat* value)
{
    DumpUniform("glUniform4fvARB", g_program, location, count, 0, value, 4);
    GLfloat scratch[kMaxLights * 4];
    real_glUniform4fvARB(location, count, CorrectLight(location, count, value, 4, scratch));
}

void APIENTRY Wrap_glUniform3fv(GLint location, GLsizei count, const GLfloat* value)
{
    DumpUniform("glUniform3fv", g_program, location, count, 0, value, 3);
    GLfloat scratch[kMaxLights * 3];
    real_glUniform3fv(location, count, CorrectLight(location, count, value, 3, scratch));
}

void APIENTRY Wrap_glUniform3fvARB(GLint location, GLsizei count, const GLfloat* value)
{
    DumpUniform("glUniform3fvARB", g_program, location, count, 0, value, 3);
    GLfloat scratch[kMaxLights * 3];
    real_glUniform3fvARB(location, count, CorrectLight(location, count, value, 3, scratch));
}

void APIENTRY Wrap_glUniformMatrix3fv(GLint location, GLsizei count, GLboolean transpose, const GLfloat* value)
{
    DumpUniform("glUniformMatrix3fv", g_program, location, count, transpose, value, 9);
    GLfloat scratch[9];
    real_glUniformMatrix3fv(location, count, transpose, CorrectEnvMatrix(location, count, transpose, value, scratch));
}

void APIENTRY Wrap_glUniformMatrix3fvARB(GLint location, GLsizei count, GLboolean transpose, const GLfloat* value)
{
    DumpUniform("glUniformMatrix3fvARB", g_program, location, count, transpose, value, 9);
    GLfloat scratch[9];
    real_glUniformMatrix3fvARB(location, count, transpose, CorrectEnvMatrix(location, count, transpose, value, scratch));
}

struct ProcWrapper {
    const char* name;
    void*       wrapper;
    void**      realSlot;
};

#define MXCAM_WRAP(fn) { #fn, reinterpret_cast<void*>(&Wrap_##fn), reinterpret_cast<void**>(&real_##fn) }

const ProcWrapper kWrappers[] = {
    MXCAM_WRAP(glUseProgram),              MXCAM_WRAP(glUseProgramObjectARB),
    MXCAM_WRAP(glGetUniformLocation),      MXCAM_WRAP(glGetUniformLocationARB),
    MXCAM_WRAP(glUniformMatrix4fv),        MXCAM_WRAP(glUniformMatrix4fvARB),
    MXCAM_WRAP(glProgramUniformMatrix4fv),
    MXCAM_WRAP(glUniform4fv),              MXCAM_WRAP(glUniform4fvARB),
    MXCAM_WRAP(glUniform3fv),              MXCAM_WRAP(glUniform3fvARB),
    MXCAM_WRAP(glUniformMatrix3fv),        MXCAM_WRAP(glUniformMatrix3fvARB),
};

}  // namespace

void* WrapProc(const char* name, void* real)
{
    // wglGetProcAddress only takes names, but guard against an ordinal-style pointer anyway.
    if (reinterpret_cast<ULONG_PTR>(name) <= 0xFFFF)
        return real;

    const ProcWrapper* match = nullptr;
    for (const ProcWrapper& w : kWrappers) {
        if (strcmp(w.name, name) == 0) {
            match = &w;
            break;
        }
    }

    AcquireSRWLockExclusive(&g_lock);
    const bool firstTime = g_fetched.insert(name).second;
    ReleaseSRWLockExclusive(&g_lock);
    if (firstTime)
        Log("proc: %s%s", name, !real ? " (not available)" : match ? " (wrapped)" : "");

    if (!match || !real)
        return real;
    *match->realSlot = real;
    return match->wrapper;
}

void StartTrace(double seconds)
{
    if (g_trace)
        return;
    wchar_t path[1024];
    swprintf_s(path, L"%smxcam_trace.csv", LogDirectory());
    g_trace = _wfsopen(path, L"w", _SH_DENYWR);
    if (!g_trace) {
        Log("trace: could not create %ls", path);
        return;
    }
    fprintf(g_trace, "time_ms,interval_ms,step_ms,sky_previews,other_rotations,cut,reset,bike,bike_fresh,custom,"
                     "raw_yaw,raw_pitch,raw_roll,bike_yaw,bike_pitch,bike_roll,view_yaw,view_pitch,view_roll,"
                     "lag_deg,look_yaw,air_yaw,air,fall_g,accel_up_g,neck_mm,"
                     "cam_x,cam_y,cam_z,eye_dx,eye_dy,eye_dz,bike_x,bike_y,bike_z\n");
    // Times are relative to the key press: the history before it is negative.
    const double now = camera::ContentTime();
    g_traceFrames = 0;
    for (int i = g_traceCount; i > 0; --i) {
        const TraceLine& line = g_traceRing[(g_traceHead - i + kTraceRing) % kTraceRing];
        if (now - line.time > kTraceHistory)
            continue;
        fprintf(g_trace, "%.1f,%s", (line.time - now) * 1000.0, line.text);
        ++g_traceFrames;
    }
    g_traceStart = now;
    g_traceEnd   = seconds;
    Log("trace: saved the last %.0f s, recording %.0f s more to %ls", kTraceHistory, seconds, path);
}

namespace {

// One line per frame into the ring buffer, and into the file while a trace is recording.
void TraceFrame(bool captured)
{
    if (!g_traceRing) {
        g_traceRing = new (std::nothrow) TraceLine[kTraceRing];
        if (!g_traceRing)
            return;
    }
    const double now = camera::ContentTime();
    TraceLine& line = g_traceRing[g_traceHead];
    line.time = now;
    g_traceHead = (g_traceHead + 1) % kTraceRing;
    if (g_traceCount < kTraceRing)
        ++g_traceCount;

    const camera::Telemetry& t = camera::LastTelemetry();
    const bike::Pose& pose = bike::Current();
    if (!captured) {
        snprintf(line.text, sizeof line.text, "no camera\n");
    } else {
        snprintf(line.text, sizeof line.text,
            "%.2f,%.2f,%d,%d,%d,%d,%d,%d,%.2f,"
            "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
            "%.3f,%.1f,%.2f,%.2f,%.2f,%.2f,%.1f,"
            "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
            t.rawIntervalMs, t.dtMs, g_camera.previews, g_camera.otherRotations,
            t.cut ? 1 : 0, t.reset ? 1 : 0, t.bikeTracked ? 1 : 0, t.bikeFresh ? 1 : 0, t.customWeight,
            t.rawYaw, t.rawPitch, t.rawRoll, t.bikeYaw, t.bikePitch, t.bikeRoll, t.outYaw, t.outPitch, t.outRoll,
            t.lagDeg, t.lookYawDeg, t.airYawDeg, t.airborne, t.fallG, t.verticalAccelG, t.neckOffset * 1000.0,
            t.rawPos[0], t.rawPos[1], t.rawPos[2], t.eyeOffset[0], t.eyeOffset[1], t.eyeOffset[2],
            pose.worldPosition[0], pose.worldPosition[1], pose.worldPosition[2]);
    }

    if (!g_trace)
        return;
    if (now - g_traceStart > g_traceEnd) {
        fclose(g_trace);
        g_trace = nullptr;
        Log("trace: %d frames written to mxcam_trace.csv", g_traceFrames);
        return;
    }
    fprintf(g_trace, "%.1f,%s", (now - g_traceStart) * 1000.0, line.text);
    ++g_traceFrames;
}

}  // namespace

bool EndShaderFrame()
{
    const bool captured = g_camera.capturedThisFrame;
    if (captured)
        bike::EndFrame(camera::ContentTime(), camera::FrameInterval());
    if (IsDumping())
        Log("  shader path: camera %s, %d '%s' uploads corrected, %d sky preview(s), %d other rotation-only draw(s)",
            captured ? "found" : "NOT found", g_camera.correctedThisFrame, GetConfig().modelviewUniform,
            g_camera.previews, g_camera.otherRotations);
    TraceFrame(captured);
    camera::FrameTick();

    // No camera this frame (menu, replay, effect off): don't carry a stale correction into the
    // next frame's sky.
    if (!captured)
        g_camera.haveCorrection = false;
    g_camera.capturedThisFrame = false;
    g_camera.havePreview = false;
    g_camera.skyAngle = 0;
    g_camera.previews = 0;
    g_camera.otherRotations = 0;
    g_camera.correctedThisFrame = 0;
    return captured;
}

}  // namespace mxcam
