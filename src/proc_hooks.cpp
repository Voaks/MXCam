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
//     - every rigid u_modelview upload is replaced with C * M - this covers the world, the
//       rider's own bike, and the sky (drawn before the terrain, so it reuses the previous
//       frame's C);
//     - uploads that aren't rigid (HUD elements are scaled down or flattened) or are exactly
//       identity (2D screens) are left alone.
//   Uniform names are logged as the game looks them up, and every upload goes into the F10 dump.

#include <windows.h>
#include <GL/gl.h>
#include <cmath>
#include <cstdio>
#include <cstring>
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

constexpr int kMaxLookupsLogged = 500;

SRWLOCK g_lock = SRWLOCK_INIT;
std::unordered_set<std::string> g_fetched;                       // names already logged
std::unordered_map<unsigned long long, std::string> g_uniforms;  // (program << 32 | location) -> name
std::unordered_set<unsigned long long> g_modelviewUniforms;      // (program, location) of each u_modelview
std::unordered_set<GLuint> g_cameraPrograms;                     // programs that draw the terrain
int     g_lookupsLogged = 0;
GLuint  g_program = 0;                                           // program bound by glUseProgram

struct ShaderCamera {
    bool   haveCorrection    = false;  // kept into the next frame for the sky, which comes first
    double correction[16]    = {};
    bool   capturedThisFrame = false;
    int    correctedThisFrame = 0;
};
ShaderCamera g_camera;

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
        length += snprintf(numbers + length, sizeof numbers - length, "%s% .3f", i && i % 4 == 0 ? " |" : "", value[i]);
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
        g_camera.haveCorrection = camera::Update(view, g_camera.correction);
        if (g_camera.haveCorrection) {
            const camera::Telemetry& t = camera::LastTelemetry();
            DumpCall("^ CAMERA  dt=%.1fms lag=%.2fdeg roll=%.2fdeg rollCorr=%.2fdeg pitchExcess=%.1fdeg "
                     "viewPitch=%.1fdeg lookYaw=%.0fdeg whip=%.1fdeg air=%.2f bike=%s %s accelUp=%.2fg "
                     "neck=%.1fmm eyeHeight%+.1fmm",
                     t.dtMs, t.lagDeg, t.rawRollDeg, t.rollCorrectionDeg, t.pitchExcessDeg, t.viewPitchDeg,
                     t.lookYawDeg, t.whipDeg, t.airborne, t.bikeTracked ? "yes" : "no",
                     t.customActive ? "CUSTOM" : "GAME", t.verticalAccelG, t.neckOffset * 1000.0,
                     t.verticalOffset * 1000.0);
        } else {
            DumpCall("^ CAMERA rejected (not a rigid view matrix)");
        }
    } else if (g_camera.capturedThisFrame) {
        bike::Observe(program, value);  // raw View * Model: one of the candidates for the bike
    }
    if (!g_camera.haveCorrection)
        return value;

    // scratch = C * value, accumulated in double precision.
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
            double sum = 0;
            for (int k = 0; k < 4; ++k)
                sum += g_camera.correction[k * 4 + r] * value[c * 4 + k];
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

// Some engines upload matrices as arrays of vec4.
void APIENTRY Wrap_glUniform4fv(GLint location, GLsizei count, const GLfloat* value)
{
    DumpUniform("glUniform4fv", g_program, location, count, 0, value, 4);
    real_glUniform4fv(location, count, value);
}

void APIENTRY Wrap_glUniform4fvARB(GLint location, GLsizei count, const GLfloat* value)
{
    DumpUniform("glUniform4fvARB", g_program, location, count, 0, value, 4);
    real_glUniform4fvARB(location, count, value);
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

bool EndShaderFrame()
{
    const bool captured = g_camera.capturedThisFrame;
    if (captured)
        bike::EndFrame();
    if (IsDumping())
        Log("  shader path: camera %s, %d '%s' uploads corrected", captured ? "found" : "NOT found",
            g_camera.correctedThisFrame, GetConfig().modelviewUniform);

    // No camera this frame (menu, replay, effect off): don't carry a stale correction into the
    // next frame's sky.
    if (!captured)
        g_camera.haveCorrection = false;
    g_camera.capturedThisFrame = false;
    g_camera.correctedThisFrame = 0;
    return captured;
}

}  // namespace mxcam
