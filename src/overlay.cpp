// overlay.cpp
//   In-game settings panel. Config::menuKey (E by default) opens and closes it.
//
//   Rendering: the panel is drawn into the game's back buffer right before each swap, as one
//   batch of textured triangles through a tiny shader of our own (GLSL 1.50, 1.30 or 1.20,
//   whichever the context accepts). The texture is an atlas built once with GDI: Segoe UI
//   glyphs, an anti-aliased disc (slider knobs, the toggle, rounded corners) and a solid white
//   block (plain rectangles). Every piece of GL state touched is saved and restored, and all GL
//   calls go straight to the driver, never through our own hooks.
//
//   Input: keys and the mouse are polled each frame. While the panel is open the game window is
//   subclassed so clicks and key presses stop reaching the game (releases still pass, so nothing
//   gets stuck), and the system cursor is replaced by a drawn one.

#include <windows.h>
#include <GL/gl.h>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "bike.h"
#include "camera.h"
#include "config.h"
#include "gamepad.h"
#include "hooks.h"
#include "log.h"
#include "mathlib.h"
#include "overlay.h"
#include "replay.h"

namespace mxcam {
namespace {

// ===========================================================================
// OpenGL entry points (the driver's own) and constants past OpenGL 1.1
// ===========================================================================
constexpr GLenum kArrayBuffer             = 0x8892;
constexpr GLenum kArrayBufferBinding      = 0x8894;
constexpr GLenum kStreamDraw              = 0x88E0;
constexpr GLenum kVertexShader            = 0x8B31;
constexpr GLenum kFragmentShader          = 0x8B30;
constexpr GLenum kCompileStatus           = 0x8B81;
constexpr GLenum kLinkStatus              = 0x8B82;
constexpr GLenum kCurrentProgram          = 0x8B8D;
constexpr GLenum kTexture0                = 0x84C0;
constexpr GLenum kActiveTexture           = 0x84E0;
constexpr GLenum kVertexArrayBinding      = 0x85B5;
constexpr GLenum kDrawFramebuffer         = 0x8CA9;
constexpr GLenum kDrawFramebufferBinding  = 0x8CA6;
constexpr GLenum kSamplerBinding          = 0x8919;
constexpr GLenum kBlendDstRgb             = 0x80C8;
constexpr GLenum kBlendSrcRgb             = 0x80C9;
constexpr GLenum kBlendDstAlpha           = 0x80CA;
constexpr GLenum kBlendSrcAlpha           = 0x80CB;
constexpr GLenum kBlendEquationRgb        = 0x8009;
constexpr GLenum kBlendEquationAlpha      = 0x883D;
constexpr GLenum kFuncAdd                 = 0x8006;
constexpr GLenum kClampToEdge             = 0x812F;
constexpr GLenum kPixelUnpackBuffer       = 0x88EC;
constexpr GLenum kPixelUnpackBufferBinding = 0x88EF;
constexpr GLenum kFramebufferSrgb         = 0x8DB9;

struct GLApi {
    decltype(&::glGetIntegerv)        GetIntegerv;
    decltype(&::glGetBooleanv)        GetBooleanv;
    decltype(&::glIsEnabled)          IsEnabled;
    decltype(&::glEnable)             Enable;
    decltype(&::glDisable)            Disable;
    decltype(&::glViewport)           Viewport;
    decltype(&::glColorMask)          ColorMask;
    decltype(&::glGenTextures)        GenTextures;
    decltype(&::glDeleteTextures)     DeleteTextures;
    decltype(&::glBindTexture)        BindTexture;
    decltype(&::glTexParameteri)      TexParameteri;
    decltype(&::glTexImage2D)         TexImage2D;
    decltype(&::glPixelStorei)        PixelStorei;
    decltype(&::glDrawArrays)         DrawArrays;
    decltype(&::wglGetCurrentContext) GetCurrentContext;
    GLuint (APIENTRY* CreateShader)(GLenum);
    void   (APIENTRY* ShaderSource)(GLuint, GLsizei, const char* const*, const GLint*);
    void   (APIENTRY* CompileShader)(GLuint);
    void   (APIENTRY* GetShaderiv)(GLuint, GLenum, GLint*);
    void   (APIENTRY* DeleteShader)(GLuint);
    GLuint (APIENTRY* CreateProgram)();
    void   (APIENTRY* DeleteProgram)(GLuint);
    void   (APIENTRY* AttachShader)(GLuint, GLuint);
    void   (APIENTRY* BindAttribLocation)(GLuint, GLuint, const char*);
    void   (APIENTRY* LinkProgram)(GLuint);
    void   (APIENTRY* GetProgramiv)(GLuint, GLenum, GLint*);
    void   (APIENTRY* UseProgram)(GLuint);
    GLint  (APIENTRY* GetUniformLocation)(GLuint, const char*);
    void   (APIENTRY* Uniform2f)(GLint, GLfloat, GLfloat);
    void   (APIENTRY* Uniform1i)(GLint, GLint);
    void   (APIENTRY* GenBuffers)(GLsizei, GLuint*);
    void   (APIENTRY* BindBuffer)(GLenum, GLuint);
    void   (APIENTRY* BufferData)(GLenum, ptrdiff_t, const void*, GLenum);
    void   (APIENTRY* VertexAttribPointer)(GLuint, GLint, GLenum, GLboolean, GLsizei, const void*);
    void   (APIENTRY* EnableVertexAttribArray)(GLuint);
    void   (APIENTRY* ActiveTexture)(GLenum);
    void   (APIENTRY* BlendFuncSeparate)(GLenum, GLenum, GLenum, GLenum);
    void   (APIENTRY* BlendEquation)(GLenum);
    void   (APIENTRY* BlendEquationSeparate)(GLenum, GLenum);
    void   (APIENTRY* GenVertexArrays)(GLsizei, GLuint*);
    void   (APIENTRY* BindVertexArray)(GLuint);
    void   (APIENTRY* BindFramebuffer)(GLenum, GLuint);  // optional (GL 3.0)
    void   (APIENTRY* BindSampler)(GLuint, GLuint);      // optional (GL 3.3)
};

GLApi g_gl{};
bool  g_glLoaded    = false;
bool  g_glAvailable = false;

template <class T>
bool LoadFunction(T& slot, const char* name)
{
    slot = reinterpret_cast<T>(GetRealGLFunction(name));
    return slot != nullptr;
}

bool LoadGL()
{
    bool ok = true;
    ok = LoadFunction(g_gl.GetIntegerv, "glGetIntegerv") && ok;
    ok = LoadFunction(g_gl.GetBooleanv, "glGetBooleanv") && ok;
    ok = LoadFunction(g_gl.IsEnabled, "glIsEnabled") && ok;
    ok = LoadFunction(g_gl.Enable, "glEnable") && ok;
    ok = LoadFunction(g_gl.Disable, "glDisable") && ok;
    ok = LoadFunction(g_gl.Viewport, "glViewport") && ok;
    ok = LoadFunction(g_gl.ColorMask, "glColorMask") && ok;
    ok = LoadFunction(g_gl.GenTextures, "glGenTextures") && ok;
    ok = LoadFunction(g_gl.DeleteTextures, "glDeleteTextures") && ok;
    ok = LoadFunction(g_gl.BindTexture, "glBindTexture") && ok;
    ok = LoadFunction(g_gl.TexParameteri, "glTexParameteri") && ok;
    ok = LoadFunction(g_gl.TexImage2D, "glTexImage2D") && ok;
    ok = LoadFunction(g_gl.PixelStorei, "glPixelStorei") && ok;
    ok = LoadFunction(g_gl.DrawArrays, "glDrawArrays") && ok;
    ok = LoadFunction(g_gl.GetCurrentContext, "wglGetCurrentContext") && ok;
    ok = LoadFunction(g_gl.CreateShader, "glCreateShader") && ok;
    ok = LoadFunction(g_gl.ShaderSource, "glShaderSource") && ok;
    ok = LoadFunction(g_gl.CompileShader, "glCompileShader") && ok;
    ok = LoadFunction(g_gl.GetShaderiv, "glGetShaderiv") && ok;
    ok = LoadFunction(g_gl.DeleteShader, "glDeleteShader") && ok;
    ok = LoadFunction(g_gl.CreateProgram, "glCreateProgram") && ok;
    ok = LoadFunction(g_gl.DeleteProgram, "glDeleteProgram") && ok;
    ok = LoadFunction(g_gl.AttachShader, "glAttachShader") && ok;
    ok = LoadFunction(g_gl.BindAttribLocation, "glBindAttribLocation") && ok;
    ok = LoadFunction(g_gl.LinkProgram, "glLinkProgram") && ok;
    ok = LoadFunction(g_gl.GetProgramiv, "glGetProgramiv") && ok;
    ok = LoadFunction(g_gl.UseProgram, "glUseProgram") && ok;
    ok = LoadFunction(g_gl.GetUniformLocation, "glGetUniformLocation") && ok;
    ok = LoadFunction(g_gl.Uniform2f, "glUniform2f") && ok;
    ok = LoadFunction(g_gl.Uniform1i, "glUniform1i") && ok;
    ok = LoadFunction(g_gl.GenBuffers, "glGenBuffers") && ok;
    ok = LoadFunction(g_gl.BindBuffer, "glBindBuffer") && ok;
    ok = LoadFunction(g_gl.BufferData, "glBufferData") && ok;
    ok = LoadFunction(g_gl.VertexAttribPointer, "glVertexAttribPointer") && ok;
    ok = LoadFunction(g_gl.EnableVertexAttribArray, "glEnableVertexAttribArray") && ok;
    ok = LoadFunction(g_gl.ActiveTexture, "glActiveTexture") && ok;
    ok = LoadFunction(g_gl.BlendFuncSeparate, "glBlendFuncSeparate") && ok;
    ok = LoadFunction(g_gl.BlendEquation, "glBlendEquation") && ok;
    ok = LoadFunction(g_gl.BlendEquationSeparate, "glBlendEquationSeparate") && ok;
    ok = LoadFunction(g_gl.GenVertexArrays, "glGenVertexArrays") && ok;
    ok = LoadFunction(g_gl.BindVertexArray, "glBindVertexArray") && ok;
    LoadFunction(g_gl.BindFramebuffer, "glBindFramebuffer");
    LoadFunction(g_gl.BindSampler, "glBindSampler");
    return ok;
}

// ===========================================================================
// Saved GL state
// ===========================================================================
struct SavedState {
    GLint     program = 0, vao = 0, arrayBuffer = 0, activeTexture = 0, texture = 0, sampler = 0, drawFramebuffer = 0;
    GLint     viewport[4] = {};
    GLint     blendSrcRgb = 0, blendDstRgb = 0, blendSrcAlpha = 0, blendDstAlpha = 0, blendEqRgb = 0, blendEqAlpha = 0;
    GLboolean colorMask[4] = {};
    bool      blend = false, depthTest = false, cullFace = false, scissorTest = false, stencilTest = false, srgb = false;
};

void SaveState(SavedState& s)
{
    g_gl.GetIntegerv(kCurrentProgram, &s.program);
    g_gl.GetIntegerv(kVertexArrayBinding, &s.vao);
    g_gl.GetIntegerv(kArrayBufferBinding, &s.arrayBuffer);
    g_gl.GetIntegerv(kActiveTexture, &s.activeTexture);
    g_gl.ActiveTexture(kTexture0);
    g_gl.GetIntegerv(GL_TEXTURE_BINDING_2D, &s.texture);
    if (g_gl.BindSampler)
        g_gl.GetIntegerv(kSamplerBinding, &s.sampler);
    if (g_gl.BindFramebuffer)
        g_gl.GetIntegerv(kDrawFramebufferBinding, &s.drawFramebuffer);
    g_gl.GetIntegerv(GL_VIEWPORT, s.viewport);
    g_gl.GetIntegerv(kBlendSrcRgb, &s.blendSrcRgb);
    g_gl.GetIntegerv(kBlendDstRgb, &s.blendDstRgb);
    g_gl.GetIntegerv(kBlendSrcAlpha, &s.blendSrcAlpha);
    g_gl.GetIntegerv(kBlendDstAlpha, &s.blendDstAlpha);
    g_gl.GetIntegerv(kBlendEquationRgb, &s.blendEqRgb);
    g_gl.GetIntegerv(kBlendEquationAlpha, &s.blendEqAlpha);
    g_gl.GetBooleanv(GL_COLOR_WRITEMASK, s.colorMask);
    s.blend       = g_gl.IsEnabled(GL_BLEND) != GL_FALSE;
    s.depthTest   = g_gl.IsEnabled(GL_DEPTH_TEST) != GL_FALSE;
    s.cullFace    = g_gl.IsEnabled(GL_CULL_FACE) != GL_FALSE;
    s.scissorTest = g_gl.IsEnabled(GL_SCISSOR_TEST) != GL_FALSE;
    s.stencilTest = g_gl.IsEnabled(GL_STENCIL_TEST) != GL_FALSE;
    s.srgb        = g_gl.BindFramebuffer && g_gl.IsEnabled(kFramebufferSrgb) != GL_FALSE;  // GL 3.0+
}

void SetCapability(GLenum capability, bool enabled)
{
    if (enabled)
        g_gl.Enable(capability);
    else
        g_gl.Disable(capability);
}

void RestoreState(const SavedState& s)
{
    g_gl.BindVertexArray(s.vao);
    g_gl.BindBuffer(kArrayBuffer, s.arrayBuffer);
    g_gl.UseProgram(s.program);
    g_gl.BindTexture(GL_TEXTURE_2D, s.texture);  // texture unit 0 is still active here
    if (g_gl.BindSampler)
        g_gl.BindSampler(0, s.sampler);
    g_gl.ActiveTexture(s.activeTexture);
    if (g_gl.BindFramebuffer)
        g_gl.BindFramebuffer(kDrawFramebuffer, s.drawFramebuffer);
    g_gl.Viewport(s.viewport[0], s.viewport[1], s.viewport[2], s.viewport[3]);
    g_gl.BlendFuncSeparate(s.blendSrcRgb, s.blendDstRgb, s.blendSrcAlpha, s.blendDstAlpha);
    g_gl.BlendEquationSeparate(s.blendEqRgb, s.blendEqAlpha);
    g_gl.ColorMask(s.colorMask[0], s.colorMask[1], s.colorMask[2], s.colorMask[3]);
    SetCapability(GL_BLEND, s.blend);
    SetCapability(GL_DEPTH_TEST, s.depthTest);
    SetCapability(GL_CULL_FACE, s.cullFace);
    SetCapability(GL_SCISSOR_TEST, s.scissorTest);
    SetCapability(GL_STENCIL_TEST, s.stencilTest);
    if (g_gl.BindFramebuffer)
        SetCapability(kFramebufferSrgb, s.srgb);
}

// ===========================================================================
// Atlas: solid block + anti-aliased disc + glyphs
// ===========================================================================
constexpr int kAtlasSize = 1024;
constexpr int kDiscX = 16, kDiscY = 4, kDiscSize = 64;
constexpr int kGlyphTop = kDiscY + kDiscSize + 8;

enum FontId { kFontTitle, kFontBody, kFontValue, kFontSmall, kFontCount };

struct Glyph {
    float u0 = 0, v0 = 0, u1 = 0, v1 = 0;
    float width = 0, height = 0, bearing = 0, advance = 0;
    bool  valid = false;
};

struct Font {
    Glyph glyphs[256];
    float lineHeight = 0;
};

Font  g_fonts[kFontCount];
float g_atlasScale = 0;

bool BuildAtlas(GLuint texture, float scale)
{
    std::vector<uint8_t> coverage(kAtlasSize * kAtlasSize, 0);

    // Solid block for plain rectangles (sampled at its centre, away from the edges).
    for (int y = 0; y < 8; ++y)
        for (int x = 0; x < 8; ++x)
            coverage[y * kAtlasSize + x] = 255;

    // Anti-aliased disc: knobs, the toggle and rounded corners are cut from it.
    const float radius = kDiscSize * 0.5f - 1.0f;
    for (int y = 0; y < kDiscSize; ++y) {
        for (int x = 0; x < kDiscSize; ++x) {
            const float dx = x + 0.5f - kDiscSize * 0.5f, dy = y + 0.5f - kDiscSize * 0.5f;
            const float alpha = Clamp(radius - std::sqrt(dx * dx + dy * dy) + 0.5f, 0.0f, 1.0f);
            coverage[(kDiscY + y) * kAtlasSize + kDiscX + x] = static_cast<uint8_t>(alpha * 255.0f + 0.5f);
        }
    }

    // Glyphs, rendered white-on-black by GDI with grayscale anti-aliasing.
    BITMAPINFO info = {};
    info.bmiHeader.biSize        = sizeof info.bmiHeader;
    info.bmiHeader.biWidth       = kAtlasSize;
    info.bmiHeader.biHeight      = -kAtlasSize;  // top-down
    info.bmiHeader.biPlanes      = 1;
    info.bmiHeader.biBitCount    = 32;
    info.bmiHeader.biCompression = BI_RGB;
    void* bits = nullptr;
    HDC dc = CreateCompatibleDC(nullptr);
    HBITMAP bitmap = CreateDIBSection(dc, &info, DIB_RGB_COLORS, &bits, nullptr, 0);
    if (!dc || !bitmap || !bits) {
        if (bitmap) DeleteObject(bitmap);
        if (dc) DeleteDC(dc);
        return false;
    }
    HGDIOBJ previousBitmap = SelectObject(dc, bitmap);
    memset(bits, 0, kAtlasSize * kAtlasSize * 4);
    SetTextColor(dc, RGB(255, 255, 255));
    SetBkMode(dc, TRANSPARENT);

    struct FontSpec { int pixels; int weight; };
    const FontSpec specs[kFontCount] = {
        { 22, FW_BOLD },      // kFontTitle
        { 15, FW_NORMAL },    // kFontBody
        { 15, FW_SEMIBOLD },  // kFontValue
        { 12, FW_SEMIBOLD },  // kFontSmall (smaller gets crunchy once GDI hinting kicks in)
    };
    const int pad = 2;
    int penX = 0, penY = kGlyphTop, rowHeight = 0;
    bool fits = true;

    for (int f = 0; f < kFontCount && fits; ++f) {
        HFONT font = CreateFontW(-static_cast<int>(std::lround(specs[f].pixels * scale)), 0, 0, 0, specs[f].weight,
                                 FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_TT_PRECIS, CLIP_DEFAULT_PRECIS,
                                 ANTIALIASED_QUALITY, DEFAULT_PITCH | FF_SWISS, L"Segoe UI");
        HGDIOBJ previousFont = SelectObject(dc, font);
        TEXTMETRICW metrics = {};
        GetTextMetricsW(dc, &metrics);
        Font& out = g_fonts[f];
        out = Font{};
        out.lineHeight = static_cast<float>(metrics.tmHeight);

        // Printable ASCII plus the degree sign and middle dot.
        for (int code = 32; code <= 0xB7 && fits; ++code) {
            if (code > 126 && code != 0xB0 && code != 0xB7)
                continue;
            const wchar_t ch = static_cast<wchar_t>(code);
            ABC abc = {};
            if (!GetCharABCWidthsW(dc, ch, ch, &abc)) {
                SIZE size = {};
                GetTextExtentPoint32W(dc, &ch, 1, &size);
                abc.abcA = 0;
                abc.abcB = static_cast<UINT>(size.cx);
                abc.abcC = 0;
            }
            const int cellWidth = static_cast<int>(abc.abcB > 0 ? abc.abcB : 1) + 2 * pad;
            const int cellHeight = metrics.tmHeight;
            if (penX + cellWidth > kAtlasSize) {
                penX = 0;
                penY += rowHeight + pad;
                rowHeight = 0;
            }
            if (penY + cellHeight > kAtlasSize) {
                fits = false;
                break;
            }
            TextOutW(dc, penX + pad - abc.abcA, penY, &ch, 1);

            Glyph& g  = out.glyphs[code];
            g.u0      = static_cast<float>(penX) / kAtlasSize;
            g.v0      = static_cast<float>(penY) / kAtlasSize;
            g.u1      = static_cast<float>(penX + cellWidth) / kAtlasSize;
            g.v1      = static_cast<float>(penY + cellHeight) / kAtlasSize;
            g.width   = static_cast<float>(cellWidth);
            g.height  = static_cast<float>(cellHeight);
            g.bearing = static_cast<float>(abc.abcA - pad);
            g.advance = static_cast<float>(abc.abcA + static_cast<int>(abc.abcB) + abc.abcC);
            g.valid   = true;

            penX += cellWidth + pad;
            rowHeight = rowHeight > cellHeight ? rowHeight : cellHeight;
        }
        penX = 0;
        penY += rowHeight + pad;
        rowHeight = 0;
        SelectObject(dc, previousFont);
        DeleteObject(font);
    }
    GdiFlush();

    const uint8_t* pixels = static_cast<const uint8_t*>(bits);
    for (int i = kGlyphTop * kAtlasSize; i < kAtlasSize * kAtlasSize; ++i)
        coverage[i] = pixels[i * 4 + 2];  // grayscale anti-aliasing: any channel will do (BGRA)
    SelectObject(dc, previousBitmap);
    DeleteObject(bitmap);
    DeleteDC(dc);
    if (!fits)
        return false;

    // RGBA, white everywhere: the shader multiplies the vertex colour by the alpha.
    std::vector<uint8_t> rgba(kAtlasSize * kAtlasSize * 4, 255);
    for (int i = 0; i < kAtlasSize * kAtlasSize; ++i)
        rgba[i * 4 + 3] = coverage[i];

    GLint alignment = 4, rowLength = 0, unpackBuffer = 0;
    g_gl.GetIntegerv(GL_UNPACK_ALIGNMENT, &alignment);
    g_gl.GetIntegerv(GL_UNPACK_ROW_LENGTH, &rowLength);
    g_gl.GetIntegerv(kPixelUnpackBufferBinding, &unpackBuffer);
    g_gl.PixelStorei(GL_UNPACK_ALIGNMENT, 4);
    g_gl.PixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    g_gl.BindBuffer(kPixelUnpackBuffer, 0);

    g_gl.BindTexture(GL_TEXTURE_2D, texture);
    g_gl.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    g_gl.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    g_gl.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, kClampToEdge);
    g_gl.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, kClampToEdge);
    g_gl.TexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, kAtlasSize, kAtlasSize, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());

    g_gl.BindBuffer(kPixelUnpackBuffer, unpackBuffer);
    g_gl.PixelStorei(GL_UNPACK_ROW_LENGTH, rowLength);
    g_gl.PixelStorei(GL_UNPACK_ALIGNMENT, alignment);
    g_atlasScale = scale;
    return true;
}

// ===========================================================================
// GPU resources (per GL context)
// ===========================================================================
const char* const kVertexBody =
    "IN vec2 a_pos;\n"
    "IN vec2 a_uv;\n"
    "IN vec4 a_color;\n"
    "OUT vec2 v_uv;\n"
    "OUT vec4 v_color;\n"
    "uniform vec2 u_screen;\n"
    "void main() {\n"
    "    v_uv = a_uv;\n"
    "    v_color = a_color;\n"
    "    gl_Position = vec4(a_pos.x / u_screen.x * 2.0 - 1.0, 1.0 - a_pos.y / u_screen.y * 2.0, 0.0, 1.0);\n"
    "}\n";

const char* const kFragmentBody =
    "IN vec2 v_uv;\n"
    "IN vec4 v_color;\n"
    "uniform sampler2D u_atlas;\n"
    "void main() { FRAG_COLOR = vec4(v_color.rgb, v_color.a * TEX(u_atlas, v_uv).a); }\n";

// The same shader in the three GLSL dialects a game context might accept (core 3.2+, 3.0/3.1,
// and 2.x compatibility); the first one that compiles wins.
struct ShaderDialect {
    const char* vertexHeader;
    const char* fragmentHeader;
};
const ShaderDialect kDialects[] = {
    { "#version 150\n#define IN in\n#define OUT out\n",
      "#version 150\n#define IN in\n#define TEX texture\nout vec4 o_color;\n#define FRAG_COLOR o_color\n" },
    { "#version 130\n#define IN in\n#define OUT out\n",
      "#version 130\n#define IN in\n#define TEX texture\nout vec4 o_color;\n#define FRAG_COLOR o_color\n" },
    { "#version 120\n#define IN attribute\n#define OUT varying\n",
      "#version 120\n#define IN varying\n#define TEX texture2D\n#define FRAG_COLOR gl_FragColor\n" },
};

struct Vertex {
    float    x, y, u, v;
    uint32_t color;  // RGBA bytes
};

struct GpuResources {
    HGLRC  context = nullptr;
    bool   failed  = false;
    GLuint program = 0, vao = 0, vbo = 0, texture = 0;
    GLint  screenUniform = -1, atlasUniform = -1;
};
GpuResources g_gpu;

bool Compiled(GLuint shader)
{
    GLint status = 0;
    g_gl.GetShaderiv(shader, kCompileStatus, &status);
    return status != 0;
}

GLuint CompileProgram()
{
    for (const ShaderDialect& dialect : kDialects) {
        const char* vertexSource[] = { dialect.vertexHeader, kVertexBody };
        const char* fragmentSource[] = { dialect.fragmentHeader, kFragmentBody };
        const GLuint vs = g_gl.CreateShader(kVertexShader);
        const GLuint fs = g_gl.CreateShader(kFragmentShader);
        g_gl.ShaderSource(vs, 2, vertexSource, nullptr);
        g_gl.ShaderSource(fs, 2, fragmentSource, nullptr);
        g_gl.CompileShader(vs);
        g_gl.CompileShader(fs);

        GLuint program = 0;
        if (Compiled(vs) && Compiled(fs)) {
            program = g_gl.CreateProgram();
            g_gl.AttachShader(program, vs);
            g_gl.AttachShader(program, fs);
            g_gl.BindAttribLocation(program, 0, "a_pos");
            g_gl.BindAttribLocation(program, 1, "a_uv");
            g_gl.BindAttribLocation(program, 2, "a_color");
            g_gl.LinkProgram(program);
            GLint linked = 0;
            g_gl.GetProgramiv(program, kLinkStatus, &linked);
            if (!linked) {
                g_gl.DeleteProgram(program);
                program = 0;
            }
        }
        g_gl.DeleteShader(vs);  // freed together with the program
        g_gl.DeleteShader(fs);
        if (program) {
            Log("overlay: ready (GLSL %.11s)", dialect.vertexHeader + 1);
            return program;
        }
    }
    return 0;
}

// Creates (or recreates, after a context change) everything the panel draws with.
bool EnsureResources(float scale)
{
    const HGLRC context = g_gl.GetCurrentContext();
    if (context != g_gpu.context) {
        // New context: the old objects belonged to the old one.
        g_gpu = GpuResources{};
        g_gpu.context = context;
        g_atlasScale = 0;
    }
    if (g_gpu.failed)
        return false;

    if (!g_gpu.program) {
        g_gpu.program = CompileProgram();
        if (!g_gpu.program) {
            Log("overlay: could not compile the panel shader; settings panel disabled");
            g_gpu.failed = true;
            return false;
        }
        g_gpu.screenUniform = g_gl.GetUniformLocation(g_gpu.program, "u_screen");
        g_gpu.atlasUniform  = g_gl.GetUniformLocation(g_gpu.program, "u_atlas");
    }
    if (!g_gpu.vao) {
        g_gl.GenVertexArrays(1, &g_gpu.vao);
        g_gl.GenBuffers(1, &g_gpu.vbo);
        g_gl.BindVertexArray(g_gpu.vao);
        g_gl.BindBuffer(kArrayBuffer, g_gpu.vbo);
        g_gl.VertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<const void*>(offsetof(Vertex, x)));
        g_gl.VertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<const void*>(offsetof(Vertex, u)));
        g_gl.VertexAttribPointer(2, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(Vertex), reinterpret_cast<const void*>(offsetof(Vertex, color)));
        g_gl.EnableVertexAttribArray(0);
        g_gl.EnableVertexAttribArray(1);
        g_gl.EnableVertexAttribArray(2);
    }
    if (!g_gpu.texture)
        g_gl.GenTextures(1, &g_gpu.texture);
    if (std::fabs(scale - g_atlasScale) > 0.01f && !BuildAtlas(g_gpu.texture, scale)) {
        Log("overlay: could not build the font atlas; settings panel disabled");
        g_gpu.failed = true;
        return false;
    }
    return true;
}

// ===========================================================================
// Drawing primitives (pixel coordinates, top-left origin)
// ===========================================================================
std::vector<Vertex> g_batch;

uint32_t Rgba(int r, int g, int b, float a)
{
    const uint32_t alpha = static_cast<uint32_t>(Clamp(a, 0.0f, 1.0f) * 255.0f + 0.5f);
    return static_cast<uint32_t>(r) | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b) << 16 | alpha << 24;
}
uint32_t White(float a) { return Rgba(255, 255, 255, a); }
uint32_t Gray(int level, float a = 1.0f) { return Rgba(level, level, level, a); }
const uint32_t kPanelColor = Rgba(12, 12, 12, 0.94f);
const uint32_t kPanelSolid = Rgba(12, 12, 12, 1.0f);

void Quad(float x0, float y0, float x1, float y1, float u0, float v0, float u1, float v1, uint32_t color)
{
    const Vertex a{ x0, y0, u0, v0, color }, b{ x1, y0, u1, v0, color };
    const Vertex c{ x1, y1, u1, v1, color }, d{ x0, y1, u0, v1, color };
    g_batch.insert(g_batch.end(), { a, b, c, a, c, d });
}

void Rect(float x, float y, float w, float h, uint32_t color)
{
    if (w <= 0 || h <= 0)
        return;
    const float u = 4.0f / kAtlasSize;  // middle of the solid block
    Quad(std::round(x), std::round(y), std::round(x + w), std::round(y + h), u, u, u, u, color);
}

constexpr float kDiscU0 = static_cast<float>(kDiscX) / kAtlasSize;
constexpr float kDiscV0 = static_cast<float>(kDiscY) / kAtlasSize;
constexpr float kDiscU1 = static_cast<float>(kDiscX + kDiscSize) / kAtlasSize;
constexpr float kDiscV1 = static_cast<float>(kDiscY + kDiscSize) / kAtlasSize;
constexpr float kDiscUM = (kDiscU0 + kDiscU1) * 0.5f;
constexpr float kDiscVM = (kDiscV0 + kDiscV1) * 0.5f;

void Disc(float cx, float cy, float r, uint32_t color)
{
    Quad(cx - r, cy - r, cx + r, cy + r, kDiscU0, kDiscV0, kDiscU1, kDiscV1, color);
}

// Rectangle with anti-aliased rounded corners cut from the disc.
void RoundedRect(float x, float y, float w, float h, float r, uint32_t color)
{
    r = std::floor(std::fmin(r, std::fmin(w, h) * 0.5f));
    Quad(x, y, x + r, y + r, kDiscU0, kDiscV0, kDiscUM, kDiscVM, color);
    Quad(x + w - r, y, x + w, y + r, kDiscUM, kDiscV0, kDiscU1, kDiscVM, color);
    Quad(x, y + h - r, x + r, y + h, kDiscU0, kDiscVM, kDiscUM, kDiscV1, color);
    Quad(x + w - r, y + h - r, x + w, y + h, kDiscUM, kDiscVM, kDiscU1, kDiscV1, color);
    Rect(x + r, y, w - 2 * r, h, color);
    Rect(x, y + r, r, h - 2 * r, color);
    Rect(x + w - r, y + r, r, h - 2 * r, color);
}

float LineHeight(FontId font) { return g_fonts[font].lineHeight; }

float TextWidth(FontId font, const char* text, float tracking)
{
    float width = 0;
    int count = 0;
    for (const unsigned char* p = reinterpret_cast<const unsigned char*>(text); *p; ++p, ++count)
        width += g_fonts[font].glyphs[*p].advance;
    return count ? width + tracking * (count - 1) : 0;
}

constexpr int kMaxWrapLines = 6;
constexpr int kWrapLineSize = 160;

// Splits text into lines no wider than width, breaking at spaces. Returns the line count.
int WrapText(FontId font, const char* text, float width, char (&lines)[kMaxWrapLines][kWrapLineSize])
{
    int count = 0;
    while (*text && count < kMaxWrapLines) {
        char* line = lines[count++];
        size_t length = 0;
        while (*text) {
            const char* end = text;
            while (*end && *end != ' ')
                ++end;
            const size_t word = static_cast<size_t>(end - text);
            const size_t joined = length + (length ? 1 : 0) + word;
            if (joined >= kWrapLineSize)
                break;
            char candidate[kWrapLineSize];
            snprintf(candidate, sizeof candidate, "%.*s%s%.*s", static_cast<int>(length), line, length ? " " : "",
                     static_cast<int>(word), text);
            if (length && TextWidth(font, candidate, 0) > width)
                break;  // the word starts the next line
            memcpy(line, candidate, joined + 1);
            length = joined;
            text = *end ? end + 1 : end;
        }
        line[length] = '\0';
    }
    return count;
}

void Text(FontId font, float x, float y, const char* text, uint32_t color, float tracking)
{
    float pen = x;
    y = std::round(y);
    for (const unsigned char* p = reinterpret_cast<const unsigned char*>(text); *p; ++p) {
        const Glyph& g = g_fonts[font].glyphs[*p];
        if (g.valid && *p != ' ') {
            const float gx = std::round(pen + g.bearing);
            Quad(gx, y, gx + g.width, y + g.height, g.u0, g.v0, g.u1, g.v1, color);
        }
        pen += g.advance + tracking;
    }
}

// ===========================================================================
// Panel state, input and window subclass
// ===========================================================================
enum class RowKind { Slider, Toggle, Choice, Key };

struct RowDef {
    int              tab;
    const char*      section;   // starts a new section when set
    const char*      label;
    RowKind          kind;
    double Config::* number;    // Slider
    bool Config::*   flag;      // Toggle / Choice
    double           min, max, step;
    double           display;   // multiplier for the shown value (e.g. fraction -> %)
    int              decimals;
    const char*      unit;
    const char*      zeroText;  // shown instead of the minimum, e.g. "Off"
    const char*      offText;   // Choice: the two options
    const char*      onText;
    bool             customOnly;  // only does something with the custom camera
    int Config::*    key = nullptr;  // Key: a virtual-key code, picked by pressing the key
    const char*      help = nullptr; // shown beside the panel while the row is selected / under the mouse
    bool             locked = false; // work in progress: drawn dimmed and can't be changed

    RowDef Help(const char* text) const
    {
        RowDef def = *this;
        def.help = text;
        return def;
    }
    RowDef Locked() const
    {
        RowDef def = *this;
        def.locked = true;
        return def;
    }
};

RowDef Slider(int tab, const char* section, const char* label, double Config::* field, double min, double max,
              double step, double display, int decimals, const char* unit, const char* zeroText = nullptr,
              bool customOnly = false)
{
    return { tab, section, label, RowKind::Slider, field, nullptr, min, max, step, display, decimals, unit,
             zeroText, nullptr, nullptr, customOnly };
}

RowDef Toggle(int tab, const char* section, const char* label, bool Config::* flag, bool customOnly = false)
{
    return { tab, section, label, RowKind::Toggle, nullptr, flag, 0, 1, 1, 1, 0, "", nullptr, nullptr, nullptr, customOnly };
}

RowDef Choice(int tab, const char* section, const char* label, bool Config::* flag, const char* offText, const char* onText)
{
    return { tab, section, label, RowKind::Choice, nullptr, flag, 0, 1, 1, 1, 0, "", nullptr, offText, onText, false };
}

RowDef KeyRow(int tab, const char* section, const char* label, int Config::* key)
{
    return { tab, section, label, RowKind::Key, nullptr, nullptr, 0, 1, 1, 1, 0, "", nullptr, nullptr, nullptr, false, key };
}

const char* const kTabs[] = { "VIEW", "AIR & LOOK", "NECK FX", "REPLAY" };
constexpr int kTabCount = static_cast<int>(sizeof kTabs / sizeof kTabs[0]);

const RowDef kRows[] = {
    // ---- VIEW ----
    Choice(0, "CAMERA",         "Camera",              &Config::customCamera, "GAME", "CUSTOM")
        .Help("GAME adjusts the game's own first-person camera. CUSTOM builds the view from your bike "
              "instead (ride a few metres after loading so it can find the bike)."),
    Toggle(0, nullptr,          "Keep game angle",     &Config::keepGameAngle, true)
        .Help("Custom camera: use the game's 1st Person Angle instead of the look-down angle below."),
    Slider(0, nullptr,          "Look-down angle",     &Config::cameraAngleDeg,   -10.0, 30.0, 0.5,  1.0,   1, "\xB0", nullptr, true)
        .Help("Custom camera: how far the view tilts down. Negative tilts it up."),
    Slider(0, "STABILIZATION",  "Horizon lock",        &Config::horizonLock,      0.0,   1.0,  0.01, 100.0, 0, "%", "Off")
        .Help("How much of the bike's lean is removed from the view. 0% leans with the bike, 100% keeps "
              "the horizon level."),
    Slider(0, nullptr,          "Pitch lock",          &Config::pitchLock,        0.0,   1.0,  0.01, 100.0, 0, "%", "Off")
        .Help("How much of the bike's nose up / down tilt (wheelies, jump faces) is removed. 0% tilts "
              "with the bike, 100% keeps the view steady."),
    Slider(0, "HEAD POSITION",  "Head forward / back", &Config::headForward,      -3.0,  3.0,  0.01, 1.0,   2, " m")
        .Help("Moves your eyes forward (+) or back (-) along the bike."),
    Slider(0, nullptr,          "Head height",         &Config::headHeight,       -0.6,  0.6,  0.01, 1.0,   2, " m")
        .Help("Moves your eyes up (+) or down (-)."),
    Toggle(0, nullptr,          "Hide torso & head",   &Config::hideOwnBody)
        .Help("Hides your own rider's torso and head (helmet included), so a high FOV doesn't show "
              "them from the inside. Arms and legs stay. Only yours: other riders keep theirs."),
    KeyRow(0, "CONTROLS",       "Menu key",            &Config::menuKey)
        .Help("The key that opens this panel. Click it, then press the new key. Esc cancels."),
    // ---- AIR & LOOK ----
    Slider(1, "IN THE AIR",     "Looseness",           &Config::airLooseness,     0.0,   1.0,  0.01, 100.0, 0, "%", "Off")
        .Help("In the air only: the view keeps pointing forward while the bike swings left / right "
              "(scrubs, whips), by up to 15\xB0 x this. Off follows the bike tightly."),
    Slider(1, nullptr,          "Whip lock",           &Config::whipLock,         0.0,   1.0,  0.01, 100.0, 0, "%", "Off")
        .Help("In the air only: holds the view on your direction of travel, horizon level, at your "
              "normal look-down angle, while the bike swings, leans and tilts under it. 100% keeps "
              "it completely straight; it eases back onto the bike after landing."),
    Slider(1, "CORNERS",        "Look to apex",        &Config::apexLook,         0.0,   1.0,  0.01, 100.0, 0, "%", "Off")
        .Help("In corners the head turns toward the apex, the way the bike is turning: more the "
              "tighter the corner. Higher looks further into the turn (100% looks about a second "
              "down your line, at most 45\xB0). Off at walking pace and in the air."),
    Slider(1, nullptr,          "Apex turn speed",     &Config::apexSpeed,        0.5,   5.0,  0.5, 1.0,   1, "")
        .Help("How quickly the head turns toward the apex and back to straight ahead after the "
              "corner. It eases in and out gently either way; lower is more gradual: 2.5 takes "
              "about a second to settle, 1 about three. Too low and it lags behind short corners."),
    Slider(1, nullptr,          "Anticipation",        &Config::apexAnticipation, 0.0,   0.5,  0.05, 1.0,  2, " s",
           "Work in progress")
        .Help("Work in progress, off for now. It will set how early the head turns: reading where the "
              "corner is going, so it turns in sooner as a corner tightens and comes back to straight "
              "ahead sooner as it opens up on the exit.")
        .Locked(),
    Toggle(1, "LOOKING AROUND", "D-pad look",          &Config::controllerLook)
        .Help("Look around with a controller's d-pad: left / right to look sideways, down to look "
              "behind. Release to look ahead again."),
    Slider(1, nullptr,          "Side look angle",     &Config::lookSideDeg,      30.0,   90.0, 1.0, 1.0,   0, "\xB0")
        .Help("How far the head turns when looking left / right with the d-pad."),
    Slider(1, nullptr,          "Turn speed",          &Config::lookTurnSpeed,    2.0,   40.0, 1.0, 1.0,   0, "")
        .Help("How quickly the head turns to look left / right / back and ahead again. Higher is "
              "faster: 12 turns 90\xB0 in about a quarter of a second, 40 is nearly instant."),
    Slider(1, nullptr,          "Sideways: forward",   &Config::lookSideForward,  -1.0,  1.0,  0.01, 1.0,   2, " m", nullptr, true)
        .Help("Custom camera: moves your eyes forward (+) or back (-) while looking left / right."),
    Slider(1, nullptr,          "Sideways: height",    &Config::lookSideHeight,   -1.0,  1.0,  0.01, 1.0,   2, " m", nullptr, true)
        .Help("Custom camera: moves your eyes up (+) or down (-) while looking left / right."),
    Slider(1, nullptr,          "Back: forward",       &Config::lookBackForward,  -1.0,  1.0,  0.01, 1.0,   2, " m", nullptr, true)
        .Help("Custom camera: moves your eyes forward (+) or back (-) while looking back."),
    Slider(1, nullptr,          "Back: height",        &Config::lookBackHeight,   -1.0,  1.0,  0.01, 1.0,   2, " m", nullptr, true)
        .Help("Custom camera: moves your eyes up (+) or down (-) while looking back."),
    // ---- NECK ----
    Slider(2, "SMOOTHING",      "Rotation response",   &Config::rotationResponse, 0.0,   40.0,  0.5,    1.0,    1, " /s", "Off")
        .Help("How fast the view's rotation catches up to the bike. Higher is sharper, lower is "
              "floatier. Works on the ground and in the air."),
    Slider(2, nullptr,          "Max lag",             &Config::maxLagDeg,        0.0,   90.0,  0.5,    1.0,    1, "\xB0")
        .Help("The furthest the smoothed view can fall behind the bike. Past this it turns with the "
              "bike instead of trailing further."),
    Slider(2, "NECK & IMPACTS", "Landing compression", &Config::compressionPerG,  0.0,   0.05,  0.0005, 1000.0, 1, " mm/g", "Off")
        .Help("How much your head sinks on landings and big hits (and lifts in the air)."),
    Slider(2, nullptr,          "Max compression",     &Config::maxCompression,   0.0,   0.2,   0.001,  1000.0, 0, " mm")
        .Help("The most your head can sink on a landing."),
    Slider(2, nullptr,          "Neck stiffness",      &Config::springHz,         0.5,   10.0,  0.1,    1.0,    1, " Hz")
        .Help("How quickly your head springs back after a hit. Higher is a tighter neck."),
    Slider(2, nullptr,          "Neck damping",        &Config::dampingRatio,     0.05,  2.0,   0.05,   1.0,    2, "")
        .Help("How much your head bounces after a hit. Lower is bouncier, 1.00 settles with no bounce."),
    Slider(2, nullptr,          "Head nod",            &Config::nodDegPerMeter,   0.0,   150.0, 1.0,    1.0,    0, "\xB0" "/m", "Off")
        .Help("How far your head nods forward as your neck compresses on a landing."),
    Slider(2, nullptr,          "Height smoothing",    &Config::verticalResponse, 0.0,   60.0,  1.0,    1.0,    0, " /s", "Off")
        .Help("Smooths small up / down head jitter. Higher follows the bike's height more closely."),
    // ---- REPLAY ----
    Choice(3, "REPLAY CAMERA",  "Camera",              &Config::replayChaseCam, "GAME", "3RD PERSON")
        .Help("GAME is the replay's own camera, which faces one way while the rider swivels in front "
              "of it. 3RD PERSON locks the camera behind the rider, like the 3rd person view when riding."),
    Slider(3, "3RD PERSON CAMERA", "Distance behind",  &Config::replayDistance,   0.5,   12.0,  0.1,    1.0,    1, " m")
        .Help("3rd person replays: how far behind the rider the camera sits. Lower moves it forward, "
              "closer to the rider."),
    Slider(3, nullptr,          "Height",              &Config::replayHeight,     -0.5,  5.0,   0.05,   1.0,    2, " m")
        .Help("3rd person replays: how high above the rider the camera sits."),
    Slider(3, nullptr,          "Pitch",               &Config::replayPitchDeg,   -10.0, 45.0,  0.5,    1.0,    1, "\xB0")
        .Help("3rd person replays: how far the camera looks down. Negative looks up."),
};
constexpr int kRowCount = static_cast<int>(sizeof kRows / sizeof kRows[0]);

// Keyboard focus order: the tab bar, the on/off switch, the current tab's rows, then the buttons.
constexpr int kTabBarItem = 0;
constexpr int kToggleItem = 1;
constexpr int kFirstRowItem = 2;

struct PanelState {
    bool   open         = false;
    int    tab          = 0;
    int    selected     = kFirstRowItem;
    int    dragging     = -1;  // index into kRows of the slider being dragged with the mouse
    int    pressedItem  = -1;  // button the mouse went down on
    int    listening    = -1;  // index into kRows of the key row waiting for a key press
    bool   helpForMouse = false;  // the help box follows the mouse (true) or the selection
    float  mouseX = -1, mouseY = -1;
    bool   dirty        = false;
    bool   closeRequest = false;
    char   status[96]   = "";
    double statusUntil  = 0;
};
PanelState g_panel;

struct Input {
    float mx = -1, my = -1;
    bool  inside = false, down = false, pressed = false, released = false;
    int   nav = 0;   // -1 up, +1 down
    int   step = 0;  // -1 left, +1 right
    bool  activate = false, coarse = false;

    bool Inside(float x, float y, float w, float h) const
    {
        return inside && mx >= x && mx < x + w && my >= y && my < y + h;
    }
};

struct KeyState {
    bool   down = false;
    double nextRepeat = 0;
};
struct Keys {
    KeyState menu, escape, up, downArrow, left, right, enter, space, mouse;
};
Keys g_keys;

// Edge-triggered key poll; with repeat, a held key fires again after 0.35 s, then every 45 ms.
bool PollKey(KeyState& key, int vk, bool repeat, double now)
{
    const bool down = vk > 0 && (GetAsyncKeyState(vk) & 0x8000) != 0;
    bool fire = false;
    if (down && !key.down) {
        fire = true;
        key.nextRepeat = now + 0.35;
    } else if (down && repeat && now >= key.nextRepeat) {
        fire = true;
        key.nextRepeat = now + 0.045;
    }
    key.down = down;
    return fire;
}

double Seconds()
{
    LARGE_INTEGER now, frequency;
    QueryPerformanceCounter(&now);
    QueryPerformanceFrequency(&frequency);
    return static_cast<double>(now.QuadPart) / static_cast<double>(frequency.QuadPart);
}

std::atomic<bool> g_menuOpen{ false };  // read by the window procedure (the window's thread)
HWND    g_hookedWindow   = nullptr;
WNDPROC g_previousProc   = nullptr;
bool    g_windowUnicode  = true;

LRESULT CALLBACK PanelWindowProc(HWND window, UINT message, WPARAM wParam, LPARAM lParam)
{
    if (g_menuOpen.load(std::memory_order_relaxed)) {
        switch (message) {
        case WM_SETCURSOR:
            if (LOWORD(lParam) == HTCLIENT) {
                SetCursor(nullptr);  // the panel draws its own cursor
                return TRUE;
            }
            break;
        // Presses go to the panel only. Releases (WM_KEYUP, WM_*BUTTONUP) still reach the game so
        // nothing it saw go down can get stuck; Alt combinations (WM_SYSKEYDOWN) pass too.
        case WM_MOUSEMOVE:
        case WM_LBUTTONDOWN:
        case WM_LBUTTONDBLCLK:
        case WM_RBUTTONDOWN:
        case WM_RBUTTONDBLCLK:
        case WM_MBUTTONDOWN:
        case WM_MBUTTONDBLCLK:
        case WM_MOUSEWHEEL:
        case WM_MOUSEHWHEEL:
        case WM_KEYDOWN:
        case WM_CHAR:
            return 0;
        default:
            break;
        }
    }
    return g_windowUnicode ? CallWindowProcW(g_previousProc, window, message, wParam, lParam)
                           : CallWindowProcA(g_previousProc, window, message, wParam, lParam);
}

void SubclassWindow(HWND window)
{
    if (window == g_hookedWindow)
        return;
    // A recreated window (e.g. a fullscreen switch): give the old one its procedure back.
    if (g_hookedWindow && IsWindow(g_hookedWindow) &&
        GetWindowLongPtrW(g_hookedWindow, GWLP_WNDPROC) == reinterpret_cast<LONG_PTR>(&PanelWindowProc)) {
        SetWindowLongPtrW(g_hookedWindow, GWLP_WNDPROC, reinterpret_cast<LONG_PTR>(g_previousProc));
    }
    g_windowUnicode = IsWindowUnicode(window) != FALSE;
    // Read the old procedure first so a message arriving mid-swap can always be forwarded.
    g_previousProc = reinterpret_cast<WNDPROC>(g_windowUnicode ? GetWindowLongPtrW(window, GWLP_WNDPROC)
                                                               : GetWindowLongPtrA(window, GWLP_WNDPROC));
    if (g_windowUnicode)
        SetWindowLongPtrW(window, GWLP_WNDPROC, reinterpret_cast<LONG_PTR>(&PanelWindowProc));
    else
        SetWindowLongPtrA(window, GWLP_WNDPROC, reinterpret_cast<LONG_PTR>(&PanelWindowProc));
    g_hookedWindow = window;
}

void OpenPanel(HWND window)
{
    SubclassWindow(window);
    g_panel.open = true;
    g_panel.dragging = -1;
    g_panel.pressedItem = -1;
    g_panel.listening = -1;
    g_panel.closeRequest = false;
    g_menuOpen = true;
    Log("panel: opened");
}

void ClosePanel()
{
    g_panel.open = false;
    g_panel.dragging = -1;
    g_panel.listening = -1;
    g_menuOpen = false;
    Log("panel: closed%s", g_panel.dirty ? " (unsaved changes kept for this session)" : "");
}

Input ReadInput(HWND window, bool focus, int width, int height, double now)
{
    Input in;
    const bool up    = PollKey(g_keys.up, VK_UP, true, now);
    const bool down  = PollKey(g_keys.downArrow, VK_DOWN, true, now);
    const bool left  = PollKey(g_keys.left, VK_LEFT, true, now);
    const bool right = PollKey(g_keys.right, VK_RIGHT, true, now);
    const bool enter = PollKey(g_keys.enter, VK_RETURN, false, now);
    const bool space = PollKey(g_keys.space, VK_SPACE, false, now);
    const bool wasDown = g_keys.mouse.down;
    PollKey(g_keys.mouse, VK_LBUTTON, false, now);
    if (!focus)
        return in;

    POINT cursor;
    if (GetCursorPos(&cursor) && ScreenToClient(window, &cursor)) {
        in.mx = static_cast<float>(cursor.x);
        in.my = static_cast<float>(cursor.y);
        in.inside = cursor.x >= 0 && cursor.y >= 0 && cursor.x < width && cursor.y < height;
    }
    in.down     = g_keys.mouse.down;
    in.pressed  = in.down && !wasDown && in.inside;
    in.released = !in.down && wasDown;
    in.nav      = (down ? 1 : 0) - (up ? 1 : 0);
    in.step     = (right ? 1 : 0) - (left ? 1 : 0);
    in.activate = enter || space;
    in.coarse   = (GetAsyncKeyState(VK_SHIFT) & 0x8000) != 0;
    return in;
}

const char* KeyName(int vk)
{
    static char name[32];
    const UINT scanCode = MapVirtualKeyA(static_cast<UINT>(vk), MAPVK_VK_TO_VSC);
    if (!scanCode || GetKeyNameTextA(static_cast<LONG>(scanCode << 16), name, sizeof name) <= 0)
        snprintf(name, sizeof name, "KEY %02X", vk);
    for (char* c = name; *c; ++c)
        *c = static_cast<char>(toupper(static_cast<unsigned char>(*c)));
    return name;
}

void SetStatus(const char* text, double now)
{
    snprintf(g_panel.status, sizeof g_panel.status, "%s", text);
    g_panel.statusUntil = now + 2.5;
}

// The first keyboard key to go down since the last call, or 0. Mouse buttons don't count.
int PollNewKey()
{
    static bool wasDown[256];
    int pressed = 0;
    for (int vk = VK_BACK; vk < 0xFF; ++vk) {
        if (vk >= VK_LSHIFT && vk <= VK_RMENU)  // reported as plain Shift / Ctrl / Alt as well
            continue;
        const bool down = (GetAsyncKeyState(vk) & 0x8000) != 0;
        if (down && !wasDown[vk] && !pressed)
            pressed = vk;
        wasDown[vk] = down;
    }
    return pressed;
}

void StartListening(int row)
{
    g_panel.listening = row;
    PollNewKey();  // keys already held (the Enter that got here) don't count as the answer
}

// Keys the panel steers with, or that another hotkey has, can't open the panel too.
const char* KeyConflict(int vk, const Config& cfg)
{
    switch (vk) {
    case VK_UP: case VK_DOWN: case VK_LEFT: case VK_RIGHT: case VK_RETURN: case VK_SPACE: case VK_SHIFT:
        return "USED BY THE PANEL";
    default:
        break;
    }
    if (vk == cfg.toggleKey || vk == cfg.reloadKey || vk == cfg.dumpKey || vk == cfg.traceKey)
        return "ALREADY AN MXCAM HOTKEY";
    return nullptr;
}

// A key row is waiting: Esc cancels, any other key is taken (unless it clashes).
void ListenForKey(double now)
{
    if (kRows[g_panel.listening].tab != g_panel.tab) {  // switched tabs with the mouse
        g_panel.listening = -1;
        return;
    }
    const int vk = PollNewKey();
    if (!vk)
        return;
    const RowDef& def = kRows[g_panel.listening];
    g_panel.listening = -1;
    if (vk == VK_ESCAPE)
        return;

    Config& cfg = EditConfig();
    char text[96];
    if (const char* conflict = KeyConflict(vk, cfg)) {
        snprintf(text, sizeof text, "%s IS %s", KeyName(vk), conflict);
        SetStatus(text, now);
        return;
    }
    int& key = cfg.*def.key;
    if (key != vk) {
        key = vk;
        g_panel.dirty = true;
    }
    g_keys.menu.down = true;  // the press that picked the key must not also close the panel
    snprintf(text, sizeof text, "MENU KEY: %s - NOT SAVED YET", KeyName(vk));
    SetStatus(text, now);
}

// ===========================================================================
// The panel (immediate mode: layout, input and drawing in one pass)
// ===========================================================================
void FormatValue(const RowDef& def, double value, char* out, size_t size)
{
    if (def.zeroText && value <= def.min + def.step * 0.5)
        snprintf(out, size, "%s", def.zeroText);
    else if (def.min < 0)
        snprintf(out, size, "%+.*f%s", def.decimals, value * def.display, def.unit);
    else
        snprintf(out, size, "%.*f%s", def.decimals, value * def.display, def.unit);
}

void SetSlider(const RowDef& def, double& value, double wanted)
{
    const double snapped = def.min + std::round((wanted - def.min) / def.step) * def.step;
    const double clamped = Clamp(snapped, def.min, def.max);
    if (clamped != value) {
        value = clamped;
        g_panel.dirty = true;
    }
}

void SetFlag(bool& flag, bool wanted)
{
    if (flag != wanted) {
        flag = wanted;
        g_panel.dirty = true;
    }
}

// Rows that currently have no effect are drawn dimmed (they can still be adjusted).
bool RowInactive(const RowDef& def, const Config& cfg)
{
    if (def.locked || (def.customOnly && !cfg.customCamera))
        return true;
    if (def.number == &Config::cameraAngleDeg && cfg.keepGameAngle)
        return true;
    if (def.tab == 3 && def.number && !cfg.replayChaseCam)  // replay camera placement, game camera chosen
        return true;
    return false;
}

// Height of one tab's rows, so the panel can be sized for the tallest tab and not jump around.
float TabHeight(int tab, float sectionH, float rowH)
{
    float height = 0;
    for (const RowDef& def : kRows) {
        if (def.tab == tab)
            height += (def.section ? sectionH : 0) + rowH;
    }
    return height;
}

void BuildPanel(int screenHeight, const Input& in, double now)
{
    Config& cfg = EditConfig();
    const float s = g_atlasScale;
    const auto S = [s](float v) { return std::round(v * s); };

    int rowsInTab = 0;
    for (const RowDef& def : kRows)
        rowsInTab += def.tab == g_panel.tab ? 1 : 0;
    const int saveItem = kFirstRowItem + rowsInTab, defaultsItem = saveItem + 1, closeItem = saveItem + 2;
    const int itemCount = closeItem + 1;
    if (g_panel.selected >= itemCount)
        g_panel.selected = itemCount - 1;
    if (in.nav)
        g_panel.selected = (g_panel.selected + in.nav + itemCount) % itemCount;

    // Help follows whichever was used last: the mouse (what's under it) or the keys (the selection).
    if (in.inside && (in.mx != g_panel.mouseX || in.my != g_panel.mouseY || in.pressed))
        g_panel.helpForMouse = true;
    if (in.nav || in.step || in.activate)
        g_panel.helpForMouse = false;
    g_panel.mouseX = in.mx;
    g_panel.mouseY = in.my;
    struct HelpSpot { const char* text = nullptr; float y = 0; };
    HelpSpot hoveredHelp, selectedHelp;
    const auto OfferHelp = [&](const char* text, float top, bool hot, bool selected) {
        if (hot)
            hoveredHelp = { text, top };
        if (selected)
            selectedHelp = { text, top };
    };

    const float pad = S(24), panelW = S(384), innerW = panelW - 2 * pad;
    const float headerH = S(46), tabsH = S(40), sectionH = S(36), rowH = S(46), buttonH = S(36);
    float contentH = 0;
    for (int tab = 0; tab < kTabCount; ++tab)
        contentH = std::fmax(contentH, TabHeight(tab, sectionH, rowH));
    const float panelH = pad + headerH + tabsH + contentH + S(24) + S(18) + S(18) + buttonH + S(16) + S(14) + S(18) + S(22);
    const float px = S(32);
    const float py = std::fmax(S(16), std::round((screenHeight - panelH) * 0.5f));

    // Keyboard-focus marker: a short white bar on the panel's left edge.
    const auto Marker = [&](float top, float height) { Rect(px, top, S(3), height, White(1.0f)); };

    // Panel with a 1 px hairline border.
    RoundedRect(px - 1, py - 1, panelW + 2, panelH + 2, S(12) + 1, White(0.16f));
    RoundedRect(px, py, panelW, panelH, S(12), kPanelColor);

    // ---- Header: title and the on/off switch ----
    const float x = px + pad;
    float y = py + pad;
    Text(kFontTitle, x, y + S(2), "MXCam", White(1.0f), S(2));

    // A pill switch; returns its outer width.
    const auto Switch = [&](float rightEdge, float centerY, float h, bool on, bool hot) {
        const float w = std::round(h * 1.8f), r = h * 0.5f, sx = rightEdge - w, sy = std::round(centerY - r);
        if (on) {
            RoundedRect(sx, sy, w, h, r, White(1.0f));
            Disc(sx + w - r, sy + r, r - S(4), kPanelSolid);
        } else {
            RoundedRect(sx, sy, w, h, r, White(hot ? 0.55f : 0.3f));
            RoundedRect(sx + 1, sy + 1, w - 2, h - 2, r - 1, kPanelSolid);
            Disc(sx + r, sy + r, r - S(5), Gray(hot ? 210 : 150));
        }
        return w;
    };

    const float th = S(24), ty = y + S(6);
    const bool toggleHot = in.Inside(x + innerW - S(116), ty - S(8), S(116), th + S(16));
    if ((toggleHot && in.pressed) || (g_panel.selected == kToggleItem && (in.step || in.activate))) {
        cfg.enabled = !cfg.enabled;
        camera::Reset();
        g_panel.dirty = true;
        g_panel.selected = kToggleItem;
    }
    const float tw = Switch(x + innerW, ty + th * 0.5f, th, cfg.enabled, toggleHot);
    const char* state = cfg.enabled ? "ON" : "OFF";
    Text(kFontSmall, x + innerW - tw - S(12) - TextWidth(kFontSmall, state, S(2)),
         ty + std::round((th - LineHeight(kFontSmall)) * 0.5f), state, cfg.enabled ? White(1.0f) : Gray(130), S(2));
    if (g_panel.selected == kToggleItem)
        Marker(ty - S(2), th + S(4));
    OfferHelp("Turns the whole mod on or off. Off gives you the game's normal camera.", py, toggleHot,
              g_panel.selected == kToggleItem);
    y += headerH;

    // ---- Tabs ----
    {
        if (g_panel.selected == kTabBarItem && in.step)
            g_panel.tab = (g_panel.tab + in.step + kTabCount) % kTabCount;
        // Each tab is its label plus an equal share of the spare width, so long labels still fit.
        float labelW[kTabCount], spare = innerW;
        for (int tab = 0; tab < kTabCount; ++tab) {
            labelW[tab] = TextWidth(kFontSmall, kTabs[tab], S(2));
            spare -= labelW[tab];
        }
        float tx = x;
        for (int tab = 0; tab < kTabCount; ++tab) {
            const float tabW = labelW[tab] + spare / kTabCount;
            const bool hot = in.Inside(tx, y, tabW, tabsH - S(8));
            if (hot && in.pressed) {
                g_panel.tab = tab;
                g_panel.selected = kTabBarItem;
            }
            const bool current = tab == g_panel.tab;
            const float lx = std::round(tx + (tabW - labelW[tab]) * 0.5f);
            Text(kFontSmall, lx, y + S(6), kTabs[tab], current ? White(1.0f) : Gray(hot ? 190 : 120), S(2));
            Rect(tx + S(6), y + S(26), tabW - S(12), current ? S(2) : 1, current ? White(1.0f) : White(0.10f));
            tx += tabW;
        }
        if (g_panel.selected == kTabBarItem)
            Marker(y + S(2), S(24));
        y += tabsH;
    }
    const float rowsTop = y;

    // ---- Rows of the current tab ----
    int item = kFirstRowItem;
    for (int i = 0; i < kRowCount; ++i) {
        const RowDef& def = kRows[i];
        if (def.tab != g_panel.tab)
            continue;
        const int thisItem = item++;
        if (def.section) {
            if (thisItem != kFirstRowItem)  // the tab underline already separates the first section
                Rect(x, y + S(8), innerW, 1, White(0.10f));
            Text(kFontSmall, x, y + S(18), def.section, Gray(120), S(2));
            y += sectionH;
        }

        const bool inactive = RowInactive(def, cfg);
        const uint32_t labelColor = inactive ? Gray(110) : Gray(215);
        const uint32_t valueColor = inactive ? Gray(120) : White(1.0f);
        const bool hot = in.Inside(px, y, panelW, rowH);
        const bool focused = g_panel.selected == thisItem;
        if (hot && in.pressed)
            g_panel.selected = thisItem;
        const bool active = g_panel.dragging == i;
        if (hot || active)
            Rect(px + 1, y, panelW - 2, rowH, White(0.035f));
        if (focused)
            Marker(y + S(9), rowH - S(18));
        OfferHelp(def.help, y, hot, focused);
        const float labelY = y + std::round((rowH - LineHeight(kFontBody)) * 0.5f);

        if (def.kind == RowKind::Slider) {
            double& value = cfg.*def.number;
            if (hot && in.pressed && !def.locked)
                g_panel.dragging = i;
            if (active && in.down)
                SetSlider(def, value, def.min + Clamp((in.mx - x) / innerW, 0.0f, 1.0f) * (def.max - def.min));
            if (focused && in.step && !def.locked)
                SetSlider(def, value, value + in.step * def.step * (in.coarse ? 10 : 1));

            char text[32];
            FormatValue(def, value, text, sizeof text);
            Text(kFontBody, x, y + S(4), def.label, labelColor, 0);
            Text(kFontValue, x + innerW - TextWidth(kFontValue, text, 0), y + S(4), text, valueColor, 0);

            const float trackY = y + S(33);
            const float t = static_cast<float>((value - def.min) / (def.max - def.min));
            const float knobX = std::round(x + t * innerW);
            Rect(x, trackY, innerW, S(2), White(0.16f));
            if (def.min < 0) {  // signed values fill from the centre
                const float zeroX = std::round(x + static_cast<float>(-def.min / (def.max - def.min)) * innerW);
                Rect(std::fmin(zeroX, knobX), trackY, std::fabs(knobX - zeroX), S(2), White(inactive ? 0.4f : 0.92f));
                Rect(zeroX, trackY - S(3), 1, S(8), White(0.3f));
            } else {
                Rect(x, trackY, knobX - x, S(2), White(inactive ? 0.4f : 0.92f));
            }
            Disc(knobX, trackY + S(1), active ? S(8) : (hot ? S(7) : S(6)), inactive ? Gray(150) : White(1.0f));
        } else if (def.kind == RowKind::Toggle) {
            bool& flag = cfg.*def.flag;
            if ((hot && in.pressed) || (focused && (in.step || in.activate)))
                SetFlag(flag, !flag);
            Text(kFontBody, x, labelY, def.label, labelColor, 0);
            const float sw = Switch(x + innerW, y + rowH * 0.5f, S(20), flag, hot);
            const char* onOff = flag ? "ON" : "OFF";
            Text(kFontSmall, x + innerW - sw - S(10) - TextWidth(kFontSmall, onOff, S(2)),
                 y + std::round((rowH - LineHeight(kFontSmall)) * 0.5f), onOff, flag ? valueColor : Gray(130), S(2));
        } else if (def.kind == RowKind::Key) {  // the key's name in a box; click it, then press a key
            const bool listening = g_panel.listening == i;
            if (listening && in.pressed)  // a second click, here or anywhere else, cancels
                g_panel.listening = -1;
            else if ((hot && in.pressed) || (focused && in.activate))
                StartListening(i);
            const bool waiting = g_panel.listening == i;

            const char* text = waiting ? "PRESS A KEY" : KeyName(cfg.*def.key);
            const float boxH = S(26), boxPad = S(12), boxY = std::round(y + (rowH - boxH) * 0.5f);
            const float boxW = TextWidth(kFontSmall, text, S(2)) + 2 * boxPad, bx = x + innerW - boxW;
            if (waiting) {
                RoundedRect(bx, boxY, boxW, boxH, boxH * 0.5f, White(1.0f));
            } else {
                RoundedRect(bx - 1, boxY - 1, boxW + 2, boxH + 2, boxH * 0.5f + 1, White(hot ? 0.5f : 0.22f));
                RoundedRect(bx, boxY, boxW, boxH, boxH * 0.5f, kPanelSolid);
            }
            Text(kFontSmall, bx + boxPad, boxY + std::round((boxH - LineHeight(kFontSmall)) * 0.5f), text,
                 waiting ? kPanelSolid : valueColor, S(2));
            Text(kFontBody, x, labelY, def.label, labelColor, 0);
        } else {  // Choice: two segments, e.g. GAME | CUSTOM
            bool& flag = cfg.*def.flag;
            const char* options[2] = { def.offText, def.onText };
            const float segH = S(26), segPad = S(12), segY = std::round(y + (rowH - segH) * 0.5f);
            float segW[2], total = 0;
            for (int o = 0; o < 2; ++o) {
                segW[o] = TextWidth(kFontSmall, options[o], S(2)) + 2 * segPad;
                total += segW[o];
            }
            float sx = x + innerW - total;
            RoundedRect(sx - 1, segY - 1, total + 2, segH + 2, segH * 0.5f + 1, White(0.22f));
            RoundedRect(sx, segY, total, segH, segH * 0.5f, kPanelSolid);
            for (int o = 0; o < 2; ++o) {
                const bool selected = flag == (o == 1);
                const bool segHot = in.Inside(sx, segY, segW[o], segH);
                if (segHot && in.pressed)
                    SetFlag(flag, o == 1);
                if (selected)
                    RoundedRect(sx, segY, segW[o], segH, segH * 0.5f, White(1.0f));
                Text(kFontSmall, sx + segPad, segY + std::round((segH - LineHeight(kFontSmall)) * 0.5f), options[o],
                     selected ? kPanelSolid : Gray(segHot ? 220 : 140), S(2));
                sx += segW[o];
            }
            if (focused && in.step)
                SetFlag(flag, in.step > 0);
            if (focused && in.activate)
                SetFlag(flag, !flag);
            Text(kFontBody, x, labelY, def.label, labelColor, 0);
        }
        y += rowH;
    }
    y = rowsTop + contentH;

    // ---- Live readout for the current tab ----
    Rect(x, y + S(10), innerW, 1, White(0.10f));
    y += S(24);
    char readout[128];
    const camera::Telemetry& t = camera::LastTelemetry();
    if (!cfg.enabled) {
        snprintf(readout, sizeof readout, "HELMET CAM IS OFF");
    } else if (g_panel.tab == 0) {
        if (cfg.customCamera && !t.bikeTracked)
            snprintf(readout, sizeof readout, "%s - RIDE A FEW METRES", bike::StatusText());
        else
            snprintf(readout, sizeof readout, "%s   PITCH %+.0f\xB0   LEAN %.0f\xB0",
                     t.customActive ? "CUSTOM CAM" : (t.bikeTracked ? "BIKE FOUND" : "GAME CAM"),
                     t.pitchExcessDeg, std::fabs(t.rawRollDeg));
    } else if (g_panel.tab == 1) {
        snprintf(readout, sizeof readout, "%s  SWING %+.0f\xB0  LOOK %+.0f\xB0  %s",
                 t.airborne > 0.5 ? "AIR" : "GROUND", t.airYawDeg, t.lookYawDeg,
                 cfg.controllerLook ? gamepad::StatusText() : "D-PAD OFF");
    } else if (g_panel.tab == 2) {
        snprintf(readout, sizeof readout, "LAG %.1f\xB0   %.1f G   NECK %+.0f MM", t.lagDeg,
                 1.0 + t.verticalAccelG, t.neckOffset * 1000.0);
    } else {
        snprintf(readout, sizeof readout, "%s", replay::StatusText());
    }
    Text(kFontSmall, x, y, readout, Gray(150), S(1));
    y += S(18) + S(18);

    // ---- Buttons ----
    const auto Button = [&](int buttonItem, float bx, float bw, const char* label, bool primary, const char* help) {
        const bool hot = in.Inside(bx, y, bw, buttonH);
        OfferHelp(help, y, hot, g_panel.selected == buttonItem);
        if (hot && in.pressed) {
            g_panel.pressedItem = buttonItem;
            g_panel.selected = buttonItem;
        }
        const bool clicked = (hot && in.released && g_panel.pressedItem == buttonItem) ||
                             (g_panel.selected == buttonItem && in.activate);
        const bool focused = hot || g_panel.selected == buttonItem;
        const float r = S(6);
        uint32_t textColor;
        if (primary || (hot && in.down)) {
            RoundedRect(bx, y, bw, buttonH, r, White(focused ? 1.0f : 0.9f));
            textColor = kPanelSolid;
        } else {
            RoundedRect(bx, y, bw, buttonH, r, White(focused ? 0.8f : 0.28f));
            RoundedRect(bx + 1, y + 1, bw - 2, buttonH - 2, r - 1, kPanelSolid);
            textColor = White(focused ? 1.0f : 0.85f);
        }
        const float labelW = TextWidth(kFontSmall, label, S(2));
        Text(kFontSmall, std::round(bx + (bw - labelW) * 0.5f), y + std::round((buttonH - LineHeight(kFontSmall)) * 0.5f),
             label, textColor, S(2));
        return clicked;
    };

    const float gap = S(10);
    const float bw = std::floor((innerW - 2 * gap) / 3);
    if (Button(saveItem, x, bw, g_panel.dirty ? "SAVE" : "SAVED", g_panel.dirty,
               "Writes your settings to mxcam.ini so they load next time.")) {
        if (SaveConfig()) {
            g_panel.dirty = false;
            SetStatus("SAVED TO MXCAM.INI", now);
        } else {
            SetStatus("COULD NOT WRITE MXCAM.INI", now);
        }
    }
    if (Button(defaultsItem, x + bw + gap, bw, "DEFAULTS", false,
               "Puts every camera setting back to its default (not the menu key). Press SAVE to keep it.")) {
        ResetTunables();
        camera::Reset();
        g_panel.dirty = true;
        SetStatus("DEFAULTS RESTORED - NOT SAVED YET", now);
    }
    if (Button(closeItem, x + 2 * (bw + gap), innerW - 2 * (bw + gap), "CLOSE", false,
               "Closes the panel. Unsaved changes stay until you quit the game."))
        g_panel.closeRequest = true;
    y += buttonH + S(16);

    // ---- Hint / status line ----
    if (g_panel.listening >= 0) {
        Text(kFontSmall, x, y, "PRESS THE NEW MENU KEY   \xB7   ESC CANCELS", White(1.0f), S(1));
    } else if (now < g_panel.statusUntil) {
        Text(kFontSmall, x, y, g_panel.status, White(1.0f), S(1));
    } else {
        char hint[128];
        snprintf(hint, sizeof hint, "%s TO CLOSE   \xB7   ARROW KEYS ADJUST", KeyName(cfg.menuKey));
        Text(kFontSmall, x, y, hint, Gray(105), S(1));
    }

    // ---- Credit, bottom right ----
    {
        y += S(18);
        const char* author = "By: Voaks";
        Text(kFontSmall, x + innerW - TextWidth(kFontSmall, author, S(1)), y, author, Gray(130), S(1));
    }

    if (in.released || !in.down) {
        g_panel.dragging = -1;
        if (in.released)
            g_panel.pressedItem = -1;
    }

    // ---- Help box beside the panel, level with its row ----
    const HelpSpot& help = g_panel.helpForMouse ? hoveredHelp : selectedHelp;
    if (help.text) {
        const float boxW = S(300), padX = S(14), padY = S(10), lineH = LineHeight(kFontBody);
        char lines[kMaxWrapLines][kWrapLineSize];
        const int count = WrapText(kFontBody, help.text, boxW - 2 * padX, lines);
        const float boxH = 2 * padY + count * lineH;
        const float bx = px + panelW + S(12);
        const float by = std::fmax(S(16), std::fmin(help.y, screenHeight - boxH - S(16)));
        RoundedRect(bx - 1, by - 1, boxW + 2, boxH + 2, S(8) + 1, White(0.16f));
        RoundedRect(bx, by, boxW, boxH, S(8), kPanelColor);
        for (int i = 0; i < count; ++i)
            Text(kFontBody, bx + padX, by + padY + i * lineH, lines[i], Gray(205), 0);
    }

    // Our own cursor (the system one is hidden over the game window while the panel is open).
    if (in.inside) {
        Disc(in.mx, in.my, S(6) + 1.5f, Rgba(0, 0, 0, 0.55f));
        Disc(in.mx, in.my, S(5), White(1.0f));
    }
}

void DrawBatch(int width, int height)
{
    if (g_batch.empty())
        return;
    if (g_gl.BindFramebuffer)
        g_gl.BindFramebuffer(kDrawFramebuffer, 0);
    g_gl.Viewport(0, 0, width, height);
    g_gl.Disable(GL_DEPTH_TEST);
    g_gl.Disable(GL_CULL_FACE);
    g_gl.Disable(GL_SCISSOR_TEST);
    g_gl.Disable(GL_STENCIL_TEST);
    if (g_gl.BindFramebuffer)
        g_gl.Disable(kFramebufferSrgb);
    g_gl.Enable(GL_BLEND);
    g_gl.BlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
    g_gl.BlendEquation(kFuncAdd);
    g_gl.ColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

    g_gl.UseProgram(g_gpu.program);
    g_gl.Uniform2f(g_gpu.screenUniform, static_cast<GLfloat>(width), static_cast<GLfloat>(height));
    g_gl.Uniform1i(g_gpu.atlasUniform, 0);
    g_gl.BindTexture(GL_TEXTURE_2D, g_gpu.texture);  // unit 0 (made active by SaveState)
    if (g_gl.BindSampler)
        g_gl.BindSampler(0, 0);

    g_gl.BindVertexArray(g_gpu.vao);
    g_gl.BindBuffer(kArrayBuffer, g_gpu.vbo);
    g_gl.BufferData(kArrayBuffer, static_cast<ptrdiff_t>(g_batch.size() * sizeof(Vertex)), g_batch.data(), kStreamDraw);
    g_gl.DrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(g_batch.size()));
}

}  // namespace

void OverlayFrame(HDC dc)
{
    if (!g_glLoaded) {
        g_glLoaded = true;
        g_glAvailable = LoadGL();
        if (!g_glAvailable)
            Log("overlay: required OpenGL functions are missing; settings panel disabled");
    }

    const double now = Seconds();
    const Config& cfg = GetConfig();
    const bool focus = GameHasFocus();
    const HWND window = WindowFromDC(dc);

    // Polled every frame, even while closed, so a key held during the toggle doesn't re-fire.
    const bool menuKey = PollKey(g_keys.menu, cfg.menuKey, false, now);
    const bool escape  = PollKey(g_keys.escape, VK_ESCAPE, false, now);

    // For automated tests: open the panel without a key press.
    static bool testOpenChecked = false;
    bool testOpen = false;
    if (!testOpenChecked) {
        testOpenChecked = true;
        testOpen = GetEnvironmentVariableA("MXCAM_TEST_OPEN_MENU", nullptr, 0) > 0;
    }

    if (!g_panel.open) {
        if (!window || !g_glAvailable || !((menuKey && focus) || testOpen))
            return;
        OpenPanel(window);
    } else if (g_panel.closeRequest || (focus && g_panel.listening < 0 && (menuKey || escape))) {
        ClosePanel();
        return;
    }
    if (!window)
        return;

    RECT client;
    if (!GetClientRect(window, &client) || client.right <= 0 || client.bottom <= 0)
        return;
    const int width = client.right, height = client.bottom;
    Input in = ReadInput(window, focus, width, height, now);
    if (g_panel.listening >= 0) {
        // The next key press is the answer, not navigation.
        if (focus)
            ListenForKey(now);
        else
            g_panel.listening = -1;
        in.nav = in.step = 0;
        in.activate = false;
    }

    // UI scale follows the window height (1080p = 1.0), in 5% steps so the atlas isn't rebuilt
    // for every pixel of a window resize.
    const float scale = std::round(Clamp(height / 1080.0f, 0.75f, 2.0f) * 20.0f) / 20.0f;

    SavedState saved;
    SaveState(saved);
    if (EnsureResources(scale)) {
        g_batch.clear();
        BuildPanel(height, in, now);
        DrawBatch(width, height);
    }
    RestoreState(saved);
}

}  // namespace mxcam
