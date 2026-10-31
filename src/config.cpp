// config.cpp
#include <windows.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cwchar>

#include "config.h"
#include "log.h"
#include "mathlib.h"

namespace mxcam {
namespace {

Config  g_config;
wchar_t g_path[1024] = L"";

bool ReadRaw(const wchar_t* section, const wchar_t* key, wchar_t* buffer, DWORD size)
{
    GetPrivateProfileStringW(section, key, L"", buffer, size, g_path);
    return buffer[0] != L'\0';
}

double ReadDouble(const wchar_t* section, const wchar_t* key, double fallback)
{
    wchar_t buffer[64];
    if (!ReadRaw(section, key, buffer, _countof(buffer)))
        return fallback;
    wchar_t* end = nullptr;
    const double value = wcstod(buffer, &end);
    return end != buffer ? value : fallback;
}

int ReadInt(const wchar_t* section, const wchar_t* key, int fallback)
{
    wchar_t buffer[64];
    if (!ReadRaw(section, key, buffer, _countof(buffer)))
        return fallback;
    wchar_t* end = nullptr;
    const long value = wcstol(buffer, &end, 0);  // base 0: accepts 120 or 0x78
    return end != buffer ? static_cast<int>(value) : fallback;
}

void ReadString(const wchar_t* section, const wchar_t* key, char (&value)[64])
{
    wchar_t buffer[64];
    if (!ReadRaw(section, key, buffer, _countof(buffer)))
        return;
    // Keep only the first word, so a trailing "; comment" can't sneak into the name.
    wchar_t* end = buffer;
    while (*end && *end != L' ' && *end != L'\t' && *end != L';')
        ++end;
    *end = L'\0';
    if (!WideCharToMultiByte(CP_UTF8, 0, buffer, -1, value, sizeof value, nullptr, nullptr))
        value[0] = '\0';
}

void ReadVector(const wchar_t* section, const wchar_t* key, double (&value)[3])
{
    wchar_t buffer[128];
    double x, y, z;
    if (!ReadRaw(section, key, buffer, _countof(buffer)) ||
        swscanf_s(buffer, L"%lf , %lf , %lf", &x, &y, &z) != 3)
        return;
    const double length = std::sqrt(x * x + y * y + z * z);
    if (length < 1e-9)
        return;
    value[0] = x / length;
    value[1] = y / length;
    value[2] = z / length;
}

}  // namespace

const Config& GetConfig() { return g_config; }

Config& EditConfig() { return g_config; }

void SetEnabled(bool enabled) { g_config.enabled = enabled; }

bool SaveConfig()
{
    if (!g_path[0])
        return false;

    struct Value {
        const wchar_t* section;
        const wchar_t* key;
        double         value;
    };
    const Value values[] = {
        { L"Camera",    L"CustomCamera",         g_config.customCamera ? 1.0 : 0.0 },
        { L"Camera",    L"KeepGameAngle",        g_config.keepGameAngle ? 1.0 : 0.0 },
        { L"Camera",    L"CameraAngleDeg",       g_config.cameraAngleDeg },
        { L"Camera",    L"HeadForward",          g_config.headForward },
        { L"Camera",    L"HeadHeight",           g_config.headHeight },
        { L"Smoothing", L"RotationResponse",     g_config.rotationResponse },
        { L"Smoothing", L"MaxLagDeg",            g_config.maxLagDeg },
        { L"Leveling",  L"HorizonLock",          g_config.horizonLock },
        { L"Leveling",  L"PitchLock",            g_config.pitchLock },
        { L"Leveling",  L"MaxRollCorrectionDeg", g_config.maxRollCorrectionDeg },
        { L"Air",       L"AirLooseness",         g_config.airLooseness },
        { L"Air",       L"WhipLock",             g_config.whipLock },
        { L"Look",      L"LookSideForward",      g_config.lookSideForward },
        { L"Look",      L"LookSideHeight",       g_config.lookSideHeight },
        { L"Look",      L"LookBackForward",      g_config.lookBackForward },
        { L"Look",      L"LookBackHeight",       g_config.lookBackHeight },
        { L"Neck",      L"VerticalResponse",     g_config.verticalResponse },
        { L"Neck",      L"MaxVerticalLag",       g_config.maxVerticalLag },
        { L"Neck",      L"CompressionPerG",      g_config.compressionPerG },
        { L"Neck",      L"SpringHz",             g_config.springHz },
        { L"Neck",      L"DampingRatio",         g_config.dampingRatio },
        { L"Neck",      L"MaxCompression",       g_config.maxCompression },
        { L"Neck",      L"MaxExtension",         g_config.maxExtension },
        { L"Neck",      L"NodDegPerMeter",       g_config.nodDegPerMeter },
        { L"Neck",      L"AccelFilterHz",        g_config.accelFilterHz },
        { L"Replay",    L"ChaseCam",             g_config.replayChaseCam ? 1.0 : 0.0 },
        { L"Replay",    L"Distance",             g_config.replayDistance },
        { L"Replay",    L"Height",               g_config.replayHeight },
        { L"Replay",    L"PitchDeg",             g_config.replayPitchDeg },
    };

    bool ok = WritePrivateProfileStringW(L"General", L"Enabled", g_config.enabled ? L"1" : L"0", g_path) != FALSE;
    wchar_t menuKey[16];
    swprintf_s(menuKey, L"0x%02X", g_config.menuKey);
    ok = WritePrivateProfileStringW(L"General", L"MenuKey", menuKey, g_path) != FALSE && ok;
    for (const Value& v : values) {
        wchar_t text[32];
        swprintf_s(text, L"%.6g", v.value);
        ok = WritePrivateProfileStringW(v.section, v.key, text, g_path) != FALSE && ok;
    }
    // Renamed to HorizonLock; drop the old key so the file can't disagree with itself.
    WritePrivateProfileStringW(L"Leveling", L"LeanCompensation", nullptr, g_path);
    Log("config: %s %ls", ok ? "saved" : "FAILED to save", g_path);
    return ok;
}

void ResetTunables()
{
    const Config defaults;
    g_config.customCamera         = defaults.customCamera;
    g_config.keepGameAngle        = defaults.keepGameAngle;
    g_config.cameraAngleDeg       = defaults.cameraAngleDeg;
    g_config.headForward          = defaults.headForward;
    g_config.headHeight           = defaults.headHeight;
    g_config.rotationResponse     = defaults.rotationResponse;
    g_config.maxLagDeg            = defaults.maxLagDeg;
    g_config.horizonLock          = defaults.horizonLock;
    g_config.pitchLock            = defaults.pitchLock;
    g_config.maxRollCorrectionDeg = defaults.maxRollCorrectionDeg;
    g_config.airLooseness         = defaults.airLooseness;
    g_config.whipLock             = defaults.whipLock;
    g_config.lookSideForward      = defaults.lookSideForward;
    g_config.lookSideHeight       = defaults.lookSideHeight;
    g_config.lookBackForward      = defaults.lookBackForward;
    g_config.lookBackHeight       = defaults.lookBackHeight;
    g_config.verticalResponse     = defaults.verticalResponse;
    g_config.maxVerticalLag       = defaults.maxVerticalLag;
    g_config.compressionPerG      = defaults.compressionPerG;
    g_config.springHz             = defaults.springHz;
    g_config.dampingRatio         = defaults.dampingRatio;
    g_config.maxCompression       = defaults.maxCompression;
    g_config.maxExtension         = defaults.maxExtension;
    g_config.nodDegPerMeter       = defaults.nodDegPerMeter;
    g_config.accelFilterHz        = defaults.accelFilterHz;
    g_config.replayChaseCam       = defaults.replayChaseCam;
    g_config.replayDistance       = defaults.replayDistance;
    g_config.replayHeight         = defaults.replayHeight;
    g_config.replayPitchDeg       = defaults.replayPitchDeg;
}

void LoadConfig(const wchar_t* iniPath)
{
    wcscpy_s(g_path, iniPath);
    ReloadConfig();
}

void ReloadConfig()
{
    Config c;  // start from defaults so keys missing from the file keep sane values

    if (GetFileAttributesW(g_path) == INVALID_FILE_ATTRIBUTES) {
        Log("config: %ls not found, using built-in defaults", g_path);
        g_config = c;
        return;
    }

    c.enabled   = ReadInt(L"General", L"Enabled", c.enabled) != 0;
    c.toggleKey = ReadInt(L"General", L"ToggleKey", c.toggleKey);
    c.reloadKey = ReadInt(L"General", L"ReloadKey", c.reloadKey);
    c.dumpKey   = ReadInt(L"General", L"DumpKey", c.dumpKey);
    c.menuKey   = ReadInt(L"General", L"MenuKey", c.menuKey);
    c.traceKey  = ReadInt(L"General", L"TraceKey", c.traceKey);

    ReadString(L"Detection", L"ModelviewUniform", c.modelviewUniform);
    ReadString(L"Detection", L"CameraProgramUniform", c.cameraProgramUniform);
    ReadString(L"Detection", L"LightUniform", c.lightUniform);
    ReadString(L"Detection", L"EnvMatrixUniform", c.envUniform);
    c.viewMatrixIndex  = ReadInt(L"Detection", L"ViewMatrixIndex", c.viewMatrixIndex);
    ReadVector(L"Detection", L"WorldUp", c.worldUp);
    c.snapAngleDeg     = ReadDouble(L"Detection", L"SnapAngleDeg", c.snapAngleDeg);
    c.teleportDistance = ReadDouble(L"Detection", L"TeleportDistance", c.teleportDistance);
    c.bikeSearchRadius = ReadDouble(L"Detection", L"BikeSearchRadius", c.bikeSearchRadius);

    c.customCamera   = ReadInt(L"Camera", L"CustomCamera", c.customCamera) != 0;
    c.keepGameAngle  = ReadInt(L"Camera", L"KeepGameAngle", c.keepGameAngle) != 0;
    c.cameraAngleDeg = ReadDouble(L"Camera", L"CameraAngleDeg", c.cameraAngleDeg);
    c.headForward    = ReadDouble(L"Camera", L"HeadForward", c.headForward);
    c.headHeight     = ReadDouble(L"Camera", L"HeadHeight", c.headHeight);

    c.rotationResponse = ReadDouble(L"Smoothing", L"RotationResponse", c.rotationResponse);
    c.maxLagDeg        = ReadDouble(L"Smoothing", L"MaxLagDeg", c.maxLagDeg);

    // HorizonLock was called LeanCompensation in earlier versions.
    c.horizonLock          = ReadDouble(L"Leveling", L"LeanCompensation", c.horizonLock);
    c.horizonLock          = ReadDouble(L"Leveling", L"HorizonLock", c.horizonLock);
    c.pitchLock            = ReadDouble(L"Leveling", L"PitchLock", c.pitchLock);
    c.maxRollCorrectionDeg = ReadDouble(L"Leveling", L"MaxRollCorrectionDeg", c.maxRollCorrectionDeg);

    c.airLooseness     = ReadDouble(L"Air", L"AirLooseness", c.airLooseness);
    c.whipLock         = ReadDouble(L"Air", L"WhipLock", c.whipLock);

    c.lookSideForward = ReadDouble(L"Look", L"LookSideForward", c.lookSideForward);
    c.lookSideHeight  = ReadDouble(L"Look", L"LookSideHeight", c.lookSideHeight);
    c.lookBackForward = ReadDouble(L"Look", L"LookBackForward", c.lookBackForward);
    c.lookBackHeight  = ReadDouble(L"Look", L"LookBackHeight", c.lookBackHeight);

    c.verticalResponse = ReadDouble(L"Neck", L"VerticalResponse", c.verticalResponse);
    c.maxVerticalLag   = ReadDouble(L"Neck", L"MaxVerticalLag", c.maxVerticalLag);
    c.compressionPerG  = ReadDouble(L"Neck", L"CompressionPerG", c.compressionPerG);
    c.springHz         = ReadDouble(L"Neck", L"SpringHz", c.springHz);
    c.dampingRatio     = ReadDouble(L"Neck", L"DampingRatio", c.dampingRatio);
    c.maxCompression   = ReadDouble(L"Neck", L"MaxCompression", c.maxCompression);
    c.maxExtension     = ReadDouble(L"Neck", L"MaxExtension", c.maxExtension);
    c.nodDegPerMeter   = ReadDouble(L"Neck", L"NodDegPerMeter", c.nodDegPerMeter);
    c.accelFilterHz    = ReadDouble(L"Neck", L"AccelFilterHz", c.accelFilterHz);

    c.replayChaseCam = ReadInt(L"Replay", L"ChaseCam", c.replayChaseCam) != 0;
    c.replayDistance = ReadDouble(L"Replay", L"Distance", c.replayDistance);
    c.replayHeight   = ReadDouble(L"Replay", L"Height", c.replayHeight);
    c.replayPitchDeg = ReadDouble(L"Replay", L"PitchDeg", c.replayPitchDeg);

    // Keep every value inside a range the physics can't blow up with.
    c.viewMatrixIndex      = Clamp(c.viewMatrixIndex, 0, 10000);
    c.snapAngleDeg         = Clamp(c.snapAngleDeg, 1.0, 180.0);
    c.teleportDistance     = Clamp(c.teleportDistance, 0.01, 1e6);
    c.rotationResponse     = Clamp(c.rotationResponse, 0.0, 1000.0);
    c.maxLagDeg            = Clamp(c.maxLagDeg, 0.0, 90.0);
    c.bikeSearchRadius     = Clamp(c.bikeSearchRadius, 0.5, 100.0);
    c.cameraAngleDeg       = Clamp(c.cameraAngleDeg, -45.0, 60.0);
    c.headForward          = Clamp(c.headForward, -5.0, 5.0);
    c.headHeight           = Clamp(c.headHeight, -5.0, 5.0);
    c.horizonLock          = Clamp(c.horizonLock, 0.0, 1.0);
    c.pitchLock            = Clamp(c.pitchLock, 0.0, 1.0);
    c.maxRollCorrectionDeg = Clamp(c.maxRollCorrectionDeg, 0.0, 180.0);
    c.airLooseness         = Clamp(c.airLooseness, 0.0, 1.0);
    c.whipLock             = Clamp(c.whipLock, 0.0, 1.0);
    c.lookSideForward      = Clamp(c.lookSideForward, -5.0, 5.0);
    c.lookSideHeight       = Clamp(c.lookSideHeight, -5.0, 5.0);
    c.lookBackForward      = Clamp(c.lookBackForward, -5.0, 5.0);
    c.lookBackHeight       = Clamp(c.lookBackHeight, -5.0, 5.0);
    c.verticalResponse     = Clamp(c.verticalResponse, 0.0, 1000.0);
    c.maxVerticalLag       = Clamp(c.maxVerticalLag, 0.0, 10.0);
    c.compressionPerG      = Clamp(c.compressionPerG, 0.0, 1.0);
    c.springHz             = Clamp(c.springHz, 0.0, 50.0);
    c.dampingRatio         = Clamp(c.dampingRatio, 0.0, 10.0);
    c.maxCompression       = Clamp(c.maxCompression, 0.0, 10.0);
    c.maxExtension         = Clamp(c.maxExtension, 0.0, 10.0);
    c.nodDegPerMeter       = Clamp(c.nodDegPerMeter, -1000.0, 1000.0);
    c.accelFilterHz        = Clamp(c.accelFilterHz, 0.0, 200.0);
    c.replayDistance       = Clamp(c.replayDistance, 0.5, 30.0);
    c.replayHeight         = Clamp(c.replayHeight, -2.0, 15.0);
    c.replayPitchDeg       = Clamp(c.replayPitchDeg, -45.0, 80.0);

    g_config = c;
    Log("config: loaded %ls (enabled=%d, %s camera, modelview='%s', camera program has '%s', up=%.2f,%.2f,%.2f)",
        g_path, c.enabled ? 1 : 0, c.customCamera ? "custom" : "game", c.modelviewUniform, c.cameraProgramUniform,
        c.worldUp[0], c.worldUp[1], c.worldUp[2]);
}

}  // namespace mxcam
