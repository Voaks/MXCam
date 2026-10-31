// smoke_test.cpp
//   Loads a built MXCam opengl32.dll explicitly, creates a real GL context through it and
//   checks the camera pipeline end to end by reading the modelview matrix back from the driver.
//   It assumes the default settings (no mxcam.ini, or the shipped one, next to the DLL).
//   The settings panel is forced open (MXCAM_TEST_OPEN_MENU) so every swap also draws it, and
//   the last section checks that drawing it leaves the game's GL state untouched.
//
//   Build from a Developer Command Prompt matching the DLL's bitness:
//     cl /nologo /EHsc /std:c++17 tests\smoke_test.cpp user32.lib gdi32.lib
//   Run (a .bmp path saves a screenshot of the panel; --custom expects an mxcam.ini next to the
//   DLL with CustomCamera=1 and HeadHeight=0.1 and checks the custom camera instead):
//     smoke_test.exe bin\x64\Release\opengl32.dll [--custom] [panel.bmp]

#include <windows.h>
#include <GL/gl.h>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <share.h>
#include <vector>

namespace {

#define GL_FUNCTIONS(X) \
    X(glMatrixMode) X(glLoadIdentity) X(glLoadMatrixf) X(glMultMatrixf) X(glGetFloatv) \
    X(glFrustum) X(glOrtho) X(glPushMatrix) X(glPopMatrix) X(wglCreateContext)         \
    X(wglMakeCurrent) X(wglDeleteContext) X(wglGetProcAddress) X(glGetError)           \
    X(glGetIntegerv) X(glIsEnabled) X(glEnable) X(glDisable) X(glViewport)             \
    X(glClearColor) X(glClear) X(glReadBuffer) X(glReadPixels) X(glPixelStorei) X(glFinish) \
    X(glDrawElements) X(glBlendFunc)

#define DECLARE(name) decltype(&::name) p_##name = nullptr;
GL_FUNCTIONS(DECLARE)
#undef DECLARE
BOOL(WINAPI* p_wglSwapBuffers)(HDC) = nullptr;

HDC g_dc = nullptr;
int g_failures = 0;
double g_headHeight = 0;  // HeadHeight in the DLL's mxcam.ini: the eye sits this far up the camera's up axis

void Check(bool ok, const char* what)
{
    printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok)
        ++g_failures;
}

struct V3 { double x, y, z; };

double EyeShift(V3 a, V3 b) { return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z)); }
V3 Cross(V3 a, V3 b) { return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }
double Dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

// World matrix (column-major) of an object at pos with yaw/pitch/roll in degrees. Y is up.
void MakeWorld(V3 pos, double yawDeg, double pitchDeg, double rollDeg, float out[16])
{
    const double y = yawDeg * 3.14159265358979 / 180, p = pitchDeg * 3.14159265358979 / 180,
                 r = rollDeg * 3.14159265358979 / 180;
    // Camera-to-world rotation = Ry(yaw) * Rx(pitch) * Rz(roll); its columns are the camera axes.
    const double cy = cos(y), sy = sin(y), cp = cos(p), sp = sin(p), cr = cos(r), sr = sin(r);
    const double Ry[3][3] = { { cy, 0, sy }, { 0, 1, 0 }, { -sy, 0, cy } };
    const double Rx[3][3] = { { 1, 0, 0 }, { 0, cp, -sp }, { 0, sp, cp } };
    const double Rz[3][3] = { { cr, -sr, 0 }, { sr, cr, 0 }, { 0, 0, 1 } };
    double t[3][3] = {}, m[3][3] = {};
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) for (int k = 0; k < 3; ++k) t[i][j] += Ry[i][k] * Rx[k][j];
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) for (int k = 0; k < 3; ++k) m[i][j] += t[i][k] * Rz[k][j];
    for (int c = 0; c < 3; ++c) {
        for (int row = 0; row < 3; ++row)
            out[c * 4 + row] = float(m[row][c]);
        out[c * 4 + 3] = 0;
    }
    out[12] = float(pos.x);
    out[13] = float(pos.y);
    out[14] = float(pos.z);
    out[15] = 1;
}

// Inverse of a rotation + translation: [ R^T | -R^T t ].
void RigidInverse(const float m[16], float out[16])
{
    for (int c = 0; c < 3; ++c) {
        for (int r = 0; r < 3; ++r)
            out[c * 4 + r] = m[r * 4 + c];
        out[c * 4 + 3] = 0;
    }
    for (int r = 0; r < 3; ++r)
        out[12 + r] = -(out[r] * m[12] + out[4 + r] * m[13] + out[8 + r] * m[14]);
    out[15] = 1;
}

// View matrix (column-major) for a camera at pos with yaw/pitch/roll in degrees.
void MakeView(V3 pos, double yawDeg, double pitchDeg, double rollDeg, float out[16])
{
    float world[16];
    MakeWorld(pos, yawDeg, pitchDeg, rollDeg, world);
    RigidInverse(world, out);
}

// Pitch (degrees, + = up) of a view matrix's forward axis (forward = -back, back.y = m[6]).
double ViewPitch(const float m[16])
{
    const double length = sqrt(double(m[2]) * m[2] + double(m[6]) * m[6] + double(m[10]) * m[10]);
    return asin(-m[6] / length) * 180 / 3.14159265358979;
}

// out = a * b (column-major)
void Mul(const float a[16], const float b[16], float out[16])
{
    for (int c = 0; c < 4; ++c)
        for (int r = 0; r < 4; ++r) {
            double s = 0;
            for (int k = 0; k < 4; ++k) s += double(a[k * 4 + r]) * b[c * 4 + k];
            out[c * 4 + r] = float(s);
        }
}

bool Near(const float a[16], const float b[16], double tol = 1e-4)
{
    for (int i = 0; i < 16; ++i)
        if (fabs(double(a[i]) - b[i]) > tol) return false;
    return true;
}

// Eye position and roll (degrees, relative to a level horizon with Y up) of a view matrix.
void Decompose(const float m[16], V3& eye, double& rollDeg)
{
    const V3 right{ m[0], m[4], m[8] }, up{ m[1], m[5], m[9] }, back{ m[2], m[6], m[10] };
    eye = { -(right.x * m[12] + up.x * m[13] + back.x * m[14]),
            -(right.y * m[12] + up.y * m[13] + back.y * m[14]),
            -(right.z * m[12] + up.z * m[13] + back.z * m[14]) };
    const V3 fwd{ -back.x, -back.y, -back.z };
    V3 level = Cross(fwd, { 0, 1, 0 });
    const double len = sqrt(Dot(level, level));
    level = { level.x / len, level.y / len, level.z / len };
    rollDeg = atan2(Dot(Cross(level, right), fwd), Dot(level, right)) * 180 / 3.14159265358979;
}

void ReadModelview(float out[16]) { p_glGetFloatv(GL_MODELVIEW_MATRIX, out); }

void SetPerspective()
{
    p_glMatrixMode(GL_PROJECTION);
    p_glLoadIdentity();
    p_glFrustum(-0.1, 0.1, -0.075, 0.075, 0.1, 1000);
    p_glMatrixMode(GL_MODELVIEW);
}

// Ends the frame, and optionally waits long enough for the camera filters to reset (> 0.25 s).
void Swap(bool resetFilters = false)
{
    p_wglSwapBuffers(g_dc);
    if (resetFilters)
        Sleep(350);
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc < 2) {
        printf("usage: smoke_test <path to proxy opengl32.dll> [--custom] [panel.bmp]\n");
        return 2;
    }
    // --custom: the DLL's mxcam.ini sets CustomCamera=1 and HeadHeight=0.1 (otherwise defaults).
    bool custom = false;
    const char* screenshot = nullptr;
    for (int i = 2; i < argc; ++i) {
        if (strcmp(argv[i], "--custom") == 0)
            custom = true;
        else
            screenshot = argv[i];
    }
    g_headHeight = custom ? 0.1 : 0.0;

    SetEnvironmentVariableA("MXCAM_TEST_OPEN_MENU", "1");
    HMODULE dll = LoadLibraryA(argv[1]);
    if (!dll) {
        printf("could not load %s (error %lu)\n", argv[1], GetLastError());
        return 1;
    }
#define LOAD(name) p_##name = reinterpret_cast<decltype(p_##name)>(GetProcAddress(dll, #name)); \
    if (!p_##name) { printf("missing export %s\n", #name); return 1; }
    GL_FUNCTIONS(LOAD)
    LOAD(wglSwapBuffers)
#undef LOAD

    // 1600x900 window (shown without taking focus, so the panel has a real framebuffer to draw
    // into) + pixel format + context, all through the proxy's WGL exports.
    WNDCLASSA wc = {};
    wc.style = CS_OWNDC;
    wc.lpfnWndProc = DefWindowProcA;
    wc.hInstance = GetModuleHandleA(nullptr);
    wc.lpszClassName = "MXCamSmokeTest";
    RegisterClassA(&wc);
    RECT frame = { 0, 0, 1600, 900 };
    AdjustWindowRect(&frame, WS_OVERLAPPEDWINDOW, FALSE);
    HWND window = CreateWindowA(wc.lpszClassName, "MXCam smoke test", WS_OVERLAPPEDWINDOW, 0, 0, frame.right - frame.left,
                                frame.bottom - frame.top, nullptr, nullptr, wc.hInstance, nullptr);
    ShowWindow(window, SW_SHOWNOACTIVATE);
    g_dc = GetDC(window);
    PIXELFORMATDESCRIPTOR pfd = { sizeof pfd, 1, PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER, PFD_TYPE_RGBA, 32 };
    pfd.cDepthBits = 24;
    SetPixelFormat(g_dc, ChoosePixelFormat(g_dc, &pfd), &pfd);
    HGLRC rc = p_wglCreateContext(g_dc);
    if (!rc || !p_wglMakeCurrent(g_dc, rc)) {
        printf("could not create a GL context through the proxy\n");
        return 1;
    }
    printf("GL context created through the proxy.\n");

    float view[16], readback[16];

    printf("2D pass is left untouched\n");
    {
        p_glMatrixMode(GL_PROJECTION);
        p_glLoadIdentity();
        p_glOrtho(0, 640, 480, 0, -1, 1);
        p_glMatrixMode(GL_MODELVIEW);
        MakeView({ 1, 2, 3 }, 10, 5, 25, view);
        p_glLoadMatrixf(view);
        ReadModelview(readback);
        Check(Near(view, readback, 1e-6), "modelview under glOrtho is passed through unchanged");
        Swap(true);
    }

    printf("Lean compensation on the camera load\n");
    float corrected[16];
    {
        SetPerspective();
        MakeView({ 10, 2, -5 }, 30, 0, 20, view);
        p_glLoadMatrixf(view);
        ReadModelview(corrected);
        V3 eye; double roll;
        Decompose(corrected, eye, roll);
        printf("    raw roll 20.0 deg -> corrected roll %.2f deg, eye (%.3f, %.3f, %.3f)\n", fabs(roll), eye.x, eye.y, eye.z);
        Check(fabs(fabs(roll) - 8.0) < 0.05, "roll reduced by LeanCompensation=0.6 (20 -> 8 deg)");
        Check(fabs(EyeShift(eye, { 10, 2, -5 }) - g_headHeight) < 1e-3, "eye position unchanged on the first frame (apart from HeadHeight)");

        // A later object load (View * Model) gets the same eye-space correction.
        float model[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 3, 0.5f, -7, 1 }, viewModel[16], expected[16];
        Mul(view, model, viewModel);
        p_glLoadMatrixf(viewModel);
        ReadModelview(readback);
        Mul(corrected, model, expected);
        Check(Near(readback, expected), "object load: C * (View * Model) == CorrectedView * Model");

        // A HUD drawn under a pushed ortho projection is untouched; after the pop, 3D loads are corrected again.
        p_glMatrixMode(GL_PROJECTION);
        p_glPushMatrix();
        p_glLoadIdentity();
        p_glOrtho(0, 640, 480, 0, -1, 1);
        p_glMatrixMode(GL_MODELVIEW);
        p_glLoadMatrixf(viewModel);
        ReadModelview(readback);
        Check(Near(readback, viewModel, 1e-6), "HUD load under pushed ortho projection is untouched");
        p_glMatrixMode(GL_PROJECTION);
        p_glPopMatrix();
        p_glMatrixMode(GL_MODELVIEW);
        p_glLoadMatrixf(viewModel);
        ReadModelview(readback);
        Check(Near(readback, expected), "after glPopMatrix the 3D pass is corrected again");
        Swap(true);
    }

    printf("glLoadIdentity + glMultMatrixf camera idiom\n");
    {
        SetPerspective();
        MakeView({ 10, 2, -5 }, 30, 0, 20, view);
        p_glLoadIdentity();
        p_glMultMatrixf(view);
        ReadModelview(readback);
        Check(Near(readback, corrected), "multiply onto identity is corrected exactly like a load");
        Swap(true);
    }

    printf("Neck compression on a hard landing\n");
    {
        // The camera falls at 5 units/s, then stops dead at height 1.0 (a landing).
        LARGE_INTEGER freq, start, now;
        QueryPerformanceFrequency(&freq);
        QueryPerformanceCounter(&start);
        double minHeight = 1e9, finalHeight = 0, landedAt = -1;
        for (;;) {
            QueryPerformanceCounter(&now);
            const double t = double(now.QuadPart - start.QuadPart) / freq.QuadPart;
            if (t > 1.6) break;
            const double height = t < 0.4 ? 3.0 - 5.0 * t : 1.0;
            SetPerspective();
            MakeView({ 0, height, 0 }, 0, 0, 0, view);
            p_glLoadMatrixf(view);
            ReadModelview(readback);
            V3 eye; double roll;
            Decompose(readback, eye, roll);
            if (t >= 0.4) {
                if (landedAt < 0) landedAt = t;
                if (eye.y < minHeight) minHeight = eye.y;
                finalHeight = eye.y;
            }
            Swap();
            Sleep(5);
        }
        printf("    after landing: lowest eye height %.4f (ground truth 1.0), settled at %.4f\n", minHeight, finalHeight);
        Check(minHeight < 0.99 + g_headHeight, "the eye dips below the head on impact (neck compression)");
        Check(minHeight > 1.0 + g_headHeight - 0.07 - 0.051, "the dip stays within MaxCompression + MaxVerticalLag");
        Check(fabs(finalHeight - 1.0 - g_headHeight) < 0.005, "the neck recovers to the head height");
    }

    printf("Shader path: u_modelview uniform uploads (how MX Simulator draws)\n");
    {
        using PFN_CreateShader       = GLuint(APIENTRY*)(GLenum);
        using PFN_ShaderSource       = void(APIENTRY*)(GLuint, GLsizei, const char* const*, const GLint*);
        using PFN_CompileShader      = void(APIENTRY*)(GLuint);
        using PFN_CreateProgram      = GLuint(APIENTRY*)();
        using PFN_AttachShader       = void(APIENTRY*)(GLuint, GLuint);
        using PFN_LinkProgram        = void(APIENTRY*)(GLuint);
        using PFN_GetProgramiv       = void(APIENTRY*)(GLuint, GLenum, GLint*);
        using PFN_UseProgram         = void(APIENTRY*)(GLuint);
        using PFN_GetUniformLocation = GLint(APIENTRY*)(GLuint, const char*);
        using PFN_UniformMatrix4fv   = void(APIENTRY*)(GLint, GLsizei, GLboolean, const GLfloat*);
        using PFN_GetUniformfv       = void(APIENTRY*)(GLuint, GLint, GLfloat*);
        using PFN_Uniform3fv         = void(APIENTRY*)(GLint, GLsizei, const GLfloat*);
#define PROC(type, name) auto name = reinterpret_cast<type>(p_wglGetProcAddress(#name))
        PROC(PFN_CreateShader, glCreateShader);
        PROC(PFN_ShaderSource, glShaderSource);
        PROC(PFN_CompileShader, glCompileShader);
        PROC(PFN_CreateProgram, glCreateProgram);
        PROC(PFN_AttachShader, glAttachShader);
        PROC(PFN_LinkProgram, glLinkProgram);
        PROC(PFN_GetProgramiv, glGetProgramiv);
        PROC(PFN_UseProgram, glUseProgram);
        PROC(PFN_GetUniformLocation, glGetUniformLocation);
        PROC(PFN_UniformMatrix4fv, glUniformMatrix4fv);
        PROC(PFN_GetUniformfv, glGetUniformfv);
        PROC(PFN_Uniform3fv, glUniform3fv);
#undef PROC
        if (!glCreateShader || !glUseProgram || !glUniformMatrix4fv || !glGetUniformfv) {
            printf("    (driver has no OpenGL 2.0 here; skipped)\n");
        } else {
            const auto build = [&](const char* vertexSource) -> GLuint {
                const char* fragmentSource = "void main() { gl_FragColor = vec4(1.0); }";
                const GLuint vs = glCreateShader(0x8B31 /* GL_VERTEX_SHADER */);
                const GLuint fs = glCreateShader(0x8B30 /* GL_FRAGMENT_SHADER */);
                glShaderSource(vs, 1, &vertexSource, nullptr);
                glShaderSource(fs, 1, &fragmentSource, nullptr);
                glCompileShader(vs);
                glCompileShader(fs);
                const GLuint program = glCreateProgram();
                glAttachShader(program, vs);
                glAttachShader(program, fs);
                glLinkProgram(program);
                GLint linked = 0;
                glGetProgramiv(program, 0x8B82 /* GL_LINK_STATUS */, &linked);
                return linked ? program : 0;
            };
            // Like MX Simulator: the terrain shader is the only one with u_heights.
            const GLuint terrain = build(
                "uniform mat4 u_modelview; uniform mat4 u_projection; uniform float u_heights;\n"
                "void main() { gl_Position = u_projection * u_modelview * gl_Vertex + vec4(u_heights); }");
            const GLuint object = build(
                "uniform mat4 u_modelview; uniform mat4 u_projection; uniform vec3 u_light_position;\n"
                "void main() { gl_Position = u_projection * u_modelview * gl_Vertex + vec4(u_light_position, 0.0); }");
            Check(terrain && object, "test shaders compile and link");

            const GLint terrainMv = glGetUniformLocation(terrain, "u_modelview");
            glGetUniformLocation(terrain, "u_heights");
            const GLint objectMv = glGetUniformLocation(object, "u_modelview");
            const GLint objectLight = glGetUniformLocation(object, "u_light_position");
            Swap(true);

            // Frame 1: the terrain upload is the camera.
            MakeView({ 10, 2, -5 }, 30, 0, 20, view);
            glUseProgram(terrain);
            glUniformMatrix4fv(terrainMv, 1, GL_FALSE, view);
            float shaderCorrected[16];
            glGetUniformfv(terrain, terrainMv, shaderCorrected);
            V3 eye; double roll;
            Decompose(shaderCorrected, eye, roll);
            printf("    terrain u_modelview: raw roll 20.0 deg -> %.2f deg\n", fabs(roll));
            Check(fabs(fabs(roll) - 8.0) < 0.05, "camera captured from the terrain program and leveled");
            Check(fabs(EyeShift(eye, { 10, 2, -5 }) - g_headHeight) < 1e-3, "eye position unchanged on the first frame (apart from HeadHeight)");

            // Objects drawn with other programs get the same correction.
            float model[16] = { 0, 0, -1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 4, 0.5f, -9, 1 }, viewModel[16], expected[16];
            Mul(view, model, viewModel);
            glUseProgram(object);
            glUniformMatrix4fv(objectMv, 1, GL_FALSE, viewModel);
            glGetUniformfv(object, objectMv, readback);
            Mul(shaderCorrected, model, expected);
            Check(Near(readback, expected), "object upload: C * (View * Model) == CorrectedView * Model");

            // HUD elements (scaled) and 2D screens (identity) are left alone.
            const float hud[16] = { 0.198f, 0, 0, 0, 0, 0.198f, 0, 0, 0, 0, 0.198f, 0, 1.415f, -0.65f, -1, 1 };
            const float identity[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
            glUniformMatrix4fv(objectMv, 1, GL_FALSE, hud);
            glGetUniformfv(object, objectMv, readback);
            Check(Near(readback, hud, 1e-7), "scaled HUD upload is untouched");
            glUniformMatrix4fv(objectMv, 1, GL_FALSE, identity);
            glGetUniformfv(object, objectMv, readback);
            Check(Near(readback, identity, 1e-7), "identity (2D screen) upload is untouched");

            // Lighting is done in eye space: the light direction (here the raw camera's right
            // axis) must turn with the view, by the rotation part of C = Corrected * Raw^-1.
            {
                float rawInverse[16], correction[16], light[3];
                RigidInverse(view, rawInverse);
                Mul(shaderCorrected, rawInverse, correction);
                const float rawLight[3] = { 1, 0, 0 };
                glUniform3fv(objectLight, 1, rawLight);
                glGetUniformfv(object, objectLight, light);
                double error = 0;
                for (int r = 0; r < 3; ++r)
                    error = fmax(error, fabs(light[r] - correction[r]));  // C * (1,0,0) = column 0
                printf("    light direction (1,0,0) -> (%.3f, %.3f, %.3f)\n", light[0], light[1], light[2]);
                Check(error < 1e-4 && fabs(light[1]) > 0.1, "the light direction turns with the view (lighting stays put)");
            }
            Swap();

            // Frame 2: the sky (rotation-only view) is drawn before the terrain and must still
            // be corrected: its rotation lets this frame's orientation be worked out early.
            float sky[16];
            for (int i = 0; i < 16; ++i)
                sky[i] = (i >= 12 && i < 15) ? 0.0f : view[i];
            glUniformMatrix4fv(objectMv, 1, GL_FALSE, sky);
            glGetUniformfv(object, objectMv, readback);
            Decompose(readback, eye, roll);
            Check(fabs(fabs(roll) - 8.0) < 0.1, "sky drawn before the camera is corrected too");
            glUseProgram(terrain);
            glUniformMatrix4fv(terrainMv, 1, GL_FALSE, view);
            Swap();

            // Riding: the bike moves along -Z at 10 units/s with the game camera mounted on it
            // (1.2 up, 0.3 back, looking 10 degrees down) on a spring: for the first 2 s it
            // bounces a few centimetres against the bike, like MX Simulator's camera. Also drawn:
            // the steering fork (drawn first), scenery passing by and a spinning wheel, none of
            // which may be taken for the bike, and the sky before the terrain. After 2.5 s of level
            // riding the bike pulls a 30 degree wheelie for 0.8 s.
            printf("Bike tracking, sky and pitch lock (%s camera)\n", custom ? "custom" : "game");
            {
                LARGE_INTEGER freq, start, now;
                QueryPerformanceFrequency(&freq);
                QueryPerformanceCounter(&start);
                float mount[16], bikeWorld[16], camWorld[16], bikeView[16], scenery[16], wheel[16], spin[16], vm[16];
                float fork[16], steer[16], skyView[16], skyOut[16];
                double levelPitch = 0, wheeliePitch = 0, levelEyeLift = 0, maxYaw = 0, maxSkyDiff = 0, maxSkyShift = 0, maxWobbleRoll = 0;
                // The custom camera must ignore the game camera's 10 degree look-down. The game camera keeps it,
                // and learns its resting pitch over a few seconds; earlier sections held it level, so stay close
                // to level (not exactly: a perfectly level sky is an identity matrix, which is left alone).
                const double mountPitch = custom ? -10.0 : -1.0;
                int rideFrame = 0;
                for (;; ++rideFrame) {
                    QueryPerformanceCounter(&now);
                    const double t = double(now.QuadPart - start.QuadPart) / freq.QuadPart;
                    if (t > 3.3) break;
                    const double bounce = t < 2.0 ? 1.0 : 0.0;
                    const double wobble = t < 2.4 ? 1.0 : 0.0;  // the sprung camera also rolls +-8 degrees against the bike, fast
                    MakeWorld({ 0, 1.2 + bounce * 0.06 * sin(t * 9), 0.3 + bounce * 0.04 * sin(t * 6) }, 0, mountPitch, wobble * 8 * sin(t * 44), mount);
                    const double bikePitch = t < 2.5 ? 0.0 : 30.0;
                    MakeWorld({ 0, 0.5, 100 - 10 * t }, 0, bikePitch, 0, bikeWorld);
                    Mul(bikeWorld, mount, camWorld);
                    RigidInverse(camWorld, bikeView);

                    // The sky first: rotation only. Every third frame something else rotation-only
                    // (like a sun billboard, 90 degrees off the camera) is drawn before it: it must
                    // neither be taken for the camera nor disturb the filters.
                    for (int i = 0; i < 16; ++i)
                        skyView[i] = (i >= 12 && i < 15) ? 0.0f : bikeView[i];
                    glUseProgram(object);
                    if (rideFrame % 3 == 0) {
                        float turn[16], billboard[16];
                        MakeWorld({ 0, 0, 0 }, 90, 0, 0, turn);
                        Mul(skyView, turn, billboard);
                        glUniformMatrix4fv(objectMv, 1, GL_FALSE, billboard);
                    }
                    glUniformMatrix4fv(objectMv, 1, GL_FALSE, skyView);
                    glGetUniformfv(object, objectMv, skyOut);

                    glUseProgram(terrain);
                    glUniformMatrix4fv(terrainMv, 1, GL_FALSE, bikeView);
                    glGetUniformfv(terrain, terrainMv, readback);
                    const double pitch = ViewPitch(readback);
                    V3 viewEye, rawEye; double unusedRoll;
                    Decompose(readback, viewEye, unusedRoll);
                    Decompose(bikeView, rawEye, unusedRoll);
                    if (t < 2.5) {
                        levelPitch = pitch;
                        levelEyeLift = viewEye.y - rawEye.y;
                    } else {
                        wheeliePitch = pitch;
                    }
                    if (t > 2.0 && t < 2.4) {  // the camera still wobbles here; the custom view must not
                        V3 unusedEye; double viewRoll;
                        Decompose(readback, unusedEye, viewRoll);
                        maxWobbleRoll = fmax(maxWobbleRoll, fabs(viewRoll));
                    }
                    if (t > 2.0 && t < 2.5)  // forward = -back; yaw away from -Z
                        maxYaw = fmax(maxYaw, fabs(atan2(-double(readback[2]), double(readback[10])) * 180 / 3.14159265358979));
                    for (int c = 0; c < 3; ++c)
                        for (int r = 0; r < 3; ++r)
                            maxSkyDiff = fmax(maxSkyDiff, fabs(double(skyOut[c * 4 + r]) - readback[c * 4 + r]));
                    maxSkyShift = fmax(maxSkyShift, fabs(skyOut[12]) + fabs(skyOut[13]) + fabs(skyOut[14]));

                    glUseProgram(object);
                    MakeWorld({ 0, 0.9, -0.6 }, 10 * sin(t * 2), 0, 0, steer);  // the fork, steering +-10 degrees
                    Mul(bikeWorld, steer, fork);
                    Mul(bikeView, fork, vm);
                    glUniformMatrix4fv(objectMv, 1, GL_FALSE, vm);
                    Mul(bikeView, bikeWorld, vm);  // the bike: View * Model
                    glUniformMatrix4fv(objectMv, 1, GL_FALSE, vm);
                    MakeWorld({ 1.5, 0.5, 100 - 4 * floor(t * 2.5) }, 20, 0, 0, scenery);  // a post every 4 units
                    Mul(bikeView, scenery, vm);
                    glUniformMatrix4fv(objectMv, 1, GL_FALSE, vm);
                    MakeWorld({ 0, 0.33, -0.75 }, 0, -t * 900, 0, spin);  // the front wheel
                    Mul(bikeWorld, spin, wheel);
                    Mul(bikeView, wheel, vm);
                    glUniformMatrix4fv(objectMv, 1, GL_FALSE, vm);
                    Swap();
                    Sleep(1 + (rideFrame * 7) % 11);  // uneven frame times, 1..11 ms + the frame itself
                }
                printf("    level: view pitch %.2f deg (game camera %.0f), eye lifted %.3f, yaw wobble %.2f deg; "
                       "wheelie: %.2f deg (game camera %+.0f)\n", levelPitch, mountPitch, levelEyeLift, maxYaw,
                       wheeliePitch, mountPitch + 30);
                printf("    sky vs world rotation: max difference %.2g; sky shifted by %.2g; view roll during camera wobble %.2f deg\n",
                       maxSkyDiff, maxSkyShift, maxWobbleRoll);
                Check(maxSkyDiff < 1e-4, "the sky turns exactly with this frame's world (no one-frame lag)");
                Check(maxSkyShift < 1e-6, "the sky is never moved by eye offsets");
                if (custom) {
                    // How often the tracker locked and lost the bike, from the DLL's log.
                    char logPath[MAX_PATH];
                    GetModuleFileNameA(dll, logPath, MAX_PATH);
                    strcpy_s(strrchr(logPath, '\\') + 1, MAX_PATH - (strrchr(logPath, '\\') + 1 - logPath), "mxcam.log");
                    int locks = 0, losses = 0, cuts = 0;
                    FILE* log = _fsopen(logPath, "r", _SH_DENYNO);  // the DLL still has it open for writing
                    if (log) {
                        char line[1024];
                        while (fgets(line, sizeof line, log)) {
                            locks += strstr(line, "bike: tracking") != nullptr;
                            losses += strstr(line, "bike: lost track") != nullptr;
                            cuts += strstr(line, "deg turn") != nullptr;  // rotation cuts (position cuts between sections are expected)
                        }
                        fclose(log);
                    }
                    printf("    tracker: locked %d time(s), lost %d time(s), camera cuts %d\n", locks, losses, cuts);
                    Check(cuts == 0, "other rotation-only draws and uneven frame times never reset the filters");
                    Check(locks == 1 && losses == 0, "the bike is found once and never lost (camera bounce tolerated)");
                    Check(maxYaw < 0.5, "the view ignores the steering fork");
                    Check(maxWobbleRoll < 0.6, "the game camera's roll wobble against the bike does not reach the view");

                    Check(fabs(levelPitch + 8.0) < 0.5, "custom camera found the bike and uses its own 8 degree look-down");
                    Check(fabs(levelEyeLift - 0.1) < 0.005, "HeadHeight lifts the eye along the bike's up axis");
                    Check(fabs(wheeliePitch - 7.0) < 1.5, "pitch lock halves the wheelie (+30 -> +15 - 8 = +7 deg)");
                } else {
                    Check(fabs(levelPitch - mountPitch) < 0.6, "game camera untouched on flat ground");
                    Check(fabs(wheeliePitch - (mountPitch + 15.0)) < 1.5, "pitch lock halves the wheelie (30 degrees -> 15)");
                }
            }
            Swap(true);

            // A replay: the game's replay camera looks 26.6 degrees down from 5 up and 10 back of
            // the rider, slowly orbiting round it (20 degrees/s; it is beside the rider, looking
            // across its path, when the ground is checked), while the rider rides along +X at
            // 10 units/s. The rider's skinned mesh is drawn after the terrain with its bones' world
            // positions in u_bone_t.
            printf("Replay (%s)\n", custom ? "3rd person camera, ChaseCam=1" : "game camera");
            {
                const GLuint skinned = build(
                    "uniform mat4 u_modelview; uniform mat4 u_projection; uniform vec3 u_bone_t[9];\n"
                    "void main() { gl_Position = u_projection * u_modelview * gl_Vertex + vec4(u_bone_t[0] + u_bone_t[8], 0.0); }");
                const GLint skinnedMv = glGetUniformLocation(skinned, "u_modelview");
                const GLint bones = glGetUniformLocation(skinned, "u_bone_t");
                Check(skinned && bones >= 0, "skinned test shader compiles and links");

                // The ground, like MX Simulator's: a flat 2048 x 2048 map drawn as a quadtree of
                // patches, one unit square mesh placed by u_meshoffset / u_meshscale, skipping the
                // patches entirely behind the plane through the game camera. Every fragment adds
                // 0.25 to red, so a pixel covered twice (overlapping patches) shows up as 0.5.
                using PFN_BindAttribLocation = void(APIENTRY*)(GLuint, GLuint, const char*);
                using PFN_GenObjects         = void(APIENTRY*)(GLsizei, GLuint*);
                using PFN_BindObject         = void(APIENTRY*)(GLuint);
                using PFN_BindBuffer         = void(APIENTRY*)(GLenum, GLuint);
                using PFN_BufferData         = void(APIENTRY*)(GLenum, ptrdiff_t, const void*, GLenum);
                using PFN_VertexAttribPointer = void(APIENTRY*)(GLuint, GLint, GLenum, GLboolean, GLsizei, const void*);
                using PFN_Uniform2fv         = void(APIENTRY*)(GLint, GLsizei, const GLfloat*);
                using PFN_Uniform1f          = void(APIENTRY*)(GLint, GLfloat);
#define PROC(type, name) auto name = reinterpret_cast<type>(p_wglGetProcAddress(#name))
                PROC(PFN_BindAttribLocation, glBindAttribLocation);
                PROC(PFN_GenObjects, glGenVertexArrays);
                PROC(PFN_BindObject, glBindVertexArray);
                PROC(PFN_GenObjects, glGenBuffers);
                PROC(PFN_BindBuffer, glBindBuffer);
                PROC(PFN_BufferData, glBufferData);
                PROC(PFN_VertexAttribPointer, glVertexAttribPointer);
                PROC(PFN_BindObject, glEnableVertexAttribArray);
                PROC(PFN_Uniform2fv, glUniform2fv);
                PROC(PFN_Uniform1f, glUniform1f);
#undef PROC
                const bool groundTest = glBindAttribLocation && glGenVertexArrays && glBindVertexArray && glGenBuffers &&
                                        glBindBuffer && glBufferData && glVertexAttribPointer && glEnableVertexAttribArray &&
                                        glUniform2fv && glUniform1f;
                GLuint ground = 0, groundArrays[16] = {};
                GLint groundMv = -1, groundProjection = -1, meshOffset = -1, meshScale = -1, texOffset = -1, texScale = -1;
                if (groundTest) {
                    const char* vertexSource =
                        "attribute vec2 a_corner; varying vec2 v_tex;\n"
                        "uniform mat4 u_modelview; uniform mat4 u_projection; uniform float u_heights;\n"
                        "uniform vec2 u_meshoffset; uniform float u_meshscale; uniform vec2 u_texoffset; uniform float u_texscale;\n"
                        "void main() {\n"
                        "    vec2 p = u_meshoffset + a_corner * u_meshscale;\n"
                        "    v_tex = (p + u_texoffset) * u_texscale;\n"
                        "    gl_Position = u_projection * u_modelview * vec4(p.x, u_heights, p.y, 1.0);\n"
                        "}";
                    const char* fragmentSource = "varying vec2 v_tex; void main() { gl_FragColor = vec4(0.25, v_tex * 0.01, 1.0); }";
                    const GLuint vs = glCreateShader(0x8B31 /* GL_VERTEX_SHADER */);
                    const GLuint fs = glCreateShader(0x8B30 /* GL_FRAGMENT_SHADER */);
                    glShaderSource(vs, 1, &vertexSource, nullptr);
                    glShaderSource(fs, 1, &fragmentSource, nullptr);
                    glCompileShader(vs);
                    glCompileShader(fs);
                    ground = glCreateProgram();
                    glAttachShader(ground, vs);
                    glAttachShader(ground, fs);
                    glBindAttribLocation(ground, 0, "a_corner");
                    glLinkProgram(ground);
                    GLint linked = 0;
                    glGetProgramiv(ground, 0x8B82 /* GL_LINK_STATUS */, &linked);
                    Check(linked != 0, "ground test shader compiles and links");
                    groundMv = glGetUniformLocation(ground, "u_modelview");
                    groundProjection = glGetUniformLocation(ground, "u_projection");
                    glGetUniformLocation(ground, "u_heights");
                    meshOffset = glGetUniformLocation(ground, "u_meshoffset");
                    meshScale = glGetUniformLocation(ground, "u_meshscale");
                    texOffset = glGetUniformLocation(ground, "u_texoffset");
                    texScale = glGetUniformLocation(ground, "u_texscale");

                    // One vertex array per combination of stitched edges, like the game's (the
                    // single-quad mesh here looks the same stitched or not).
                    glGenVertexArrays(16, groundArrays);
                    for (GLuint vertexArray : groundArrays) {
                        GLuint buffers[2] = {};
                        glBindVertexArray(vertexArray);
                        glGenBuffers(2, buffers);
                        const float corners[8] = { 0, 0, 1, 0, 0, 1, 1, 1 };
                        const unsigned short indices[6] = { 0, 1, 2, 2, 1, 3 };
                        glBindBuffer(0x8892 /* GL_ARRAY_BUFFER */, buffers[0]);
                        glBufferData(0x8892, sizeof corners, corners, 0x88E4 /* GL_STATIC_DRAW */);
                        glBindBuffer(0x8893 /* GL_ELEMENT_ARRAY_BUFFER */, buffers[1]);
                        glBufferData(0x8893, sizeof indices, indices, 0x88E4);
                        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
                        glEnableVertexAttribArray(0);
                    }
                    glBindVertexArray(0);
                } else {
                    printf("    (no vertex arrays here; the ground fill isn't tested)\n");
                }
                // 90 degrees vertical field of view, 16:9, near 0.5, far 4000.
                const float n = 0.5f, f = 4000.0f;
                const float projection[16] = { 0.5625f, 0, 0, 0, 0, 1, 0, 0, 0, 0, (f + n) / (n - f), -1, 0, 0, 2 * f * n / (n - f), 0 };

                // The game's patches: split while the game camera is within 2 patch sizes, down to
                // 32 (none bigger than 512), skipped when entirely behind the game camera.
                struct GameCamera { V3 eye, forward; } gameCamera{};
                int gamePatches = 0;
                GLuint lastArray = 0;
                std::vector<int> cells{ 0, 0, 2048 };
                const auto splits = [&](int x, int z, int size) {
                    const double dx = fmax(0.0, fmax(x - gameCamera.eye.x, gameCamera.eye.x - (x + size)));
                    const double dz = fmax(0.0, fmax(z - gameCamera.eye.z, gameCamera.eye.z - (z + size)));
                    return size > 512 || (size > 32 && sqrt(dx * dx + dz * dz) < 2.0 * size);
                };
                // The size of the patch (drawn or culled) holding a point; 0 off the map.
                const auto leafSize = [&](int px, int pz) {
                    if (px < 0 || pz < 0 || px >= 2048 || pz >= 2048)
                        return 0;
                    int x = 0, z = 0, size = 2048;
                    while (splits(x, z, size)) {
                        size /= 2;
                        x += px >= x + size ? size : 0;
                        z += pz >= z + size ? size : 0;
                    }
                    return size;
                };
                const auto drawGameTerrain = [&]() {
                    gamePatches = 0;
                    cells.assign({ 0, 0, 2048 });
                    while (!cells.empty()) {
                        const int size = cells.back(), z = cells[cells.size() - 2], x = cells[cells.size() - 3];
                        cells.resize(cells.size() - 3);
                        double ahead = -1e9;
                        for (int c = 0; c < 4; ++c)
                            ahead = fmax(ahead, Dot({ x + (c & 1 ? size : 0) - gameCamera.eye.x, -gameCamera.eye.y,
                                                      z + (c & 2 ? size : 0) - gameCamera.eye.z }, gameCamera.forward));
                        if (ahead < 0)
                            continue;
                        if (splits(x, z, size)) {
                            const int half = size / 2;
                            for (int c = 0; c < 4; ++c)
                                cells.insert(cells.end(), { x + (c & 1 ? half : 0), z + (c & 2 ? half : 0), half });
                            continue;
                        }
                        const int stitched = (leafSize(x - 1, z) > size ? 1 : 0) | (leafSize(x + size, z) > size ? 2 : 0) |
                                             (leafSize(x, z - 1) > size ? 4 : 0) | (leafSize(x, z + size) > size ? 8 : 0);
                        if (groundArrays[stitched] != lastArray) {
                            lastArray = groundArrays[stitched];
                            glBindVertexArray(lastArray);
                        }
                        const float offset[2] = { float(x), float(z) }, none[2] = {};
                        glUniform2fv(meshOffset, 1, offset);
                        glUniform1f(meshScale, float(size));
                        glUniform2fv(texOffset, 1, none);
                        glUniform1f(texScale, 0);
                        p_glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, nullptr);
                        ++gamePatches;
                    }
                };
                // Pixels from the bottom of the screen to just below the horizon (all of it ground
                // inside the map): returns the fractions covered once, more than once and not at all.
                const int rows = 535;
                std::vector<unsigned char> red(1600 * rows);
                const auto coverage = [&](double& once, double& twice, double& none) {
                    p_glReadBuffer(GL_BACK);
                    p_glPixelStorei(GL_PACK_ALIGNMENT, 1);
                    p_glReadPixels(0, 0, 1600, rows, GL_RED, GL_UNSIGNED_BYTE, red.data());
                    int counts[3] = {};
                    for (unsigned char value : red)
                        ++counts[value < 20 ? 2 : value <= 80 ? 0 : 1];
                    once = double(counts[0]) / red.size();
                    twice = double(counts[1]) / red.size();
                    none = double(counts[2]) / red.size();
                };
                double gameOnly[3] = { 0, 0, 1 }, filled[3] = { 0, 0, 1 };
                bool groundChecked = false, arrayRestored = true;
                int groundPatches = 0;

                LARGE_INTEGER freq, start, now;
                QueryPerformanceFrequency(&freq);
                QueryPerformanceCounter(&start);
                double maxPassThrough = 0, chaseHeading = 0, chasePitch = 0, chaseEyeError = 1e9;
                for (;;) {
                    QueryPerformanceCounter(&now);
                    const double t = double(now.QuadPart - start.QuadPart) / freq.QuadPart;
                    if (t > 2.6) break;
                    // In the middle of the map, so the ground stretches all round.
                    const V3 rider{ 1000 + 10 * t, 1, 1000 };
                    float replayView[16];
                    const double orbit = 20 * t - 44, orbitRad = orbit * 3.14159265358979 / 180;
                    const V3 replayEye{ rider.x + 10 * sin(orbitRad), rider.y + 5, rider.z + 10 * cos(orbitRad) };
                    MakeView(replayEye, orbit, -26.565, 0, replayView);
                    glUseProgram(terrain);
                    glUniformMatrix4fv(terrainMv, 1, GL_FALSE, replayView);
                    glGetUniformfv(terrain, terrainMv, readback);
                    if (t > 0.6) {
                        double difference = 0;
                        for (int i = 0; i < 16; ++i)
                            difference = fmax(difference, fabs(double(readback[i]) - replayView[i]));
                        maxPassThrough = fmax(maxPassThrough, difference);
                    }
                    if (t > 2.2) {
                        const V3 back{ readback[2], readback[6], readback[10] };
                        chaseHeading = atan2(-back.z, -back.x) * 180 / 3.14159265358979;  // 0 = looking along +X
                        chasePitch = ViewPitch(readback);
                        V3 chaseEye; double unusedRoll;
                        Decompose(readback, chaseEye, unusedRoll);
                        // The default placement: 4.5 behind and 1.55 above the rider.
                        const V3 wanted{ rider.x - 4.5, rider.y + 1.55, rider.z };
                        chaseEyeError = EyeShift(chaseEye, wanted);
                    }

                    // The ground, once the chase camera has settled: read back what the game's
                    // patches cover, then (after the switch to the next program) with the fill.
                    if (ground) {
                        const bool measure = t > 2.2 && !groundChecked;
                        gameCamera.eye = replayEye;
                        gameCamera.forward = { -replayView[2], -replayView[6], -replayView[10] };
                        p_glViewport(0, 0, 1600, 900);
                        p_glClearColor(0, 0, 0, 1);
                        p_glClear(GL_COLOR_BUFFER_BIT);
                        p_glDisable(GL_DEPTH_TEST);
                        p_glDisable(GL_CULL_FACE);
                        p_glEnable(GL_BLEND);
                        p_glBlendFunc(GL_ONE, GL_ONE);
                        glUseProgram(ground);
                        lastArray = 0;
                        glBindVertexArray(0);
                        glUniformMatrix4fv(groundMv, 1, GL_FALSE, replayView);
                        glUniformMatrix4fv(groundProjection, 1, GL_FALSE, projection);
                        drawGameTerrain();
                        if (measure) {
                            coverage(gameOnly[0], gameOnly[1], gameOnly[2]);
                            groundPatches = gamePatches;
                        }
                    }

                    // The bones: spread around the rider, centred on it.
                    float boneData[27];
                    for (int b = 0; b < 9; ++b) {
                        boneData[b * 3]     = float(rider.x + 0.3 * (b % 3 - 1));
                        boneData[b * 3 + 1] = float(rider.y + 0.4 * (b / 3 - 1));
                        boneData[b * 3 + 2] = float(rider.z + 0.2 * ((b * 7) % 3 - 1));
                    }
                    glUseProgram(skinned);
                    if (ground) {
                        GLint bound = 0;
                        p_glGetIntegerv(0x85B5 /* GL_VERTEX_ARRAY_BINDING */, &bound);
                        arrayRestored = arrayRestored && GLuint(bound) == lastArray;
                        if (t > 2.2 && !groundChecked) {
                            coverage(filled[0], filled[1], filled[2]);
                            groundChecked = true;
                        }
                        p_glDisable(GL_BLEND);
                    }
                    glUniform3fv(bones, 9, boneData);
                    glUniformMatrix4fv(skinnedMv, 1, GL_FALSE, replayView);
                    Swap();
                    Sleep(5);
                }
                if (ground) {
                    glBindVertexArray(0);
                    printf("    ground: %d game patches cover %.1f%% (%.2f%% twice), with the fill %.2f%% once, %.2f%% twice, %.2f%% missing\n",
                           groundPatches, 100 * (gameOnly[0] + gameOnly[1]), 100 * gameOnly[1], 100 * filled[0], 100 * filled[1], 100 * filled[2]);
                    Check(groundChecked && gameOnly[1] < 0.001, "the simulated game's patches never overlap");
                    Check(arrayRestored, "the game's vertex array is bound again after the ground pass");
                }
                if (custom) {
                    printf("    3rd person: heading %.2f deg, pitch %.2f deg, eye %.3f from where it belongs\n",
                           chaseHeading, chasePitch, chaseEyeError);
                    Check(fabs(chaseHeading) < 1.0, "the camera turns to look along the rider's path");
                    Check(fabs(chasePitch + 12.0) < 0.5, "it looks 12 degrees down at the rider");
                    Check(chaseEyeError < 0.05, "it sits behind and above the rider, locked to it");
                    if (ground) {
                        Check(gameOnly[2] > 0.01, "the game's culling leaves holes in the 3rd person view (test is meaningful)");
                        Check(filled[2] < 0.002, "the fill covers the ground the game culled");
                        Check(filled[1] < 0.002, "filled cells never overlap the game's patches");
                    }
                } else {
                    printf("    largest change to the replay camera once recognised: %.2g\n", maxPassThrough);
                    Check(maxPassThrough < 1e-6, "the game's replay camera is left untouched");
                    if (ground)
                        Check(filled[0] == gameOnly[0] && filled[1] == gameOnly[1], "nothing is filled in for the game's own camera");
                }
            }
            Swap(true);

            printf("Settings panel (forced open) leaves the game's GL state alone\n");
            using PFN_ActiveTexture = void(APIENTRY*)(GLenum);
            auto glActiveTexture = reinterpret_cast<PFN_ActiveTexture>(p_wglGetProcAddress("glActiveTexture"));
            glUseProgram(object);
            glActiveTexture(0x84C0 + 3);  // GL_TEXTURE3
            p_glDisable(GL_BLEND);
            p_glEnable(GL_DEPTH_TEST);
            p_glEnable(GL_CULL_FACE);
            p_glViewport(7, 9, 300, 200);
            Swap();
            GLint program = 0, activeTexture = 0, viewport[4] = {};
            p_glGetIntegerv(0x8B8D /* GL_CURRENT_PROGRAM */, &program);
            p_glGetIntegerv(0x84E0 /* GL_ACTIVE_TEXTURE */, &activeTexture);
            p_glGetIntegerv(GL_VIEWPORT, viewport);
            Check(static_cast<GLuint>(program) == object, "current program restored");
            Check(activeTexture == 0x84C0 + 3, "active texture unit restored");
            Check(!p_glIsEnabled(GL_BLEND) && p_glIsEnabled(GL_DEPTH_TEST) && p_glIsEnabled(GL_CULL_FACE),
                  "blend / depth test / culling restored");
            Check(viewport[0] == 7 && viewport[1] == 9 && viewport[2] == 300 && viewport[3] == 200, "viewport restored");
            Check(p_glGetError() == GL_NO_ERROR, "drawing the panel raised no GL errors");
            glUseProgram(0);
            p_glDisable(GL_DEPTH_TEST);
            p_glDisable(GL_CULL_FACE);
            p_glViewport(0, 0, 1600, 900);
        }
    }

    if (screenshot) {
        // A frame with a sky-ish background, then read the presented image back.
        p_glClearColor(0.42f, 0.55f, 0.68f, 1.0f);
        p_glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        Swap();
        p_glFinish();
        const int w = 1600, h = 900, stride = (w * 3 + 3) & ~3;
        std::vector<unsigned char> pixels(static_cast<size_t>(stride) * h);
        p_glReadBuffer(GL_FRONT);
        p_glPixelStorei(GL_PACK_ALIGNMENT, 4);
        p_glReadPixels(0, 0, w, h, 0x80E0 /* GL_BGR */, GL_UNSIGNED_BYTE, pixels.data());
        BITMAPFILEHEADER file = {};
        BITMAPINFOHEADER info = {};
        info.biSize = sizeof info;
        info.biWidth = w;
        info.biHeight = h;  // bottom-up, like glReadPixels
        info.biPlanes = 1;
        info.biBitCount = 24;
        file.bfType = 0x4D42;
        file.bfOffBits = sizeof file + sizeof info;
        file.bfSize = file.bfOffBits + static_cast<DWORD>(pixels.size());
        FILE* out = nullptr;
        if (fopen_s(&out, screenshot, "wb") == 0 && out) {
            fwrite(&file, sizeof file, 1, out);
            fwrite(&info, sizeof info, 1, out);
            fwrite(pixels.data(), 1, pixels.size(), out);
            fclose(out);
            printf("Panel screenshot written to %s\n", screenshot);
        }
    }

    p_wglMakeCurrent(nullptr, nullptr);
    p_wglDeleteContext(rc);
    printf(g_failures ? "\n%d check(s) FAILED\n" : "\nAll checks passed\n", g_failures);
    return g_failures ? 1 : 0;
}
