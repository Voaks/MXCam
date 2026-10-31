// config.h
//   Tunables, loaded from mxcam.ini next to the DLL. The defaults below match the shipped
//   mxcam.ini; see that file for what each value does in practice.
#pragma once

namespace mxcam {

struct Config {
    // [General]
    bool   enabled   = true;
    int    toggleKey = 0x77;  // VK_F8
    int    reloadKey = 0x78;  // VK_F9
    int    dumpKey   = 0x79;  // VK_F10
    int    menuKey   = 0x45;  // 'E': in-game settings panel
    int    traceKey  = 0x7A;  // VK_F11: save the last 15 s (+5 s) of per-frame trace

    // [Detection]
    // Shader path (MX Simulator): matrices arrive as glUniformMatrix4fv uploads. Both names are
    // read once at startup, because uniform roles are assigned as the game looks them up.
    char   modelviewUniform[64]     = "u_modelview";  // per-object View * Model uniform
    char   cameraProgramUniform[64] = "u_heights";    // a uniform only the terrain shader has
    char   lightUniform[64]         = "u_light_position";  // light direction, sent in eye space
    char   envUniform[64]           = "u_envmat";    // reflection lookup rotation (eye -> world)
    int    viewMatrixIndex  = 0;          // fixed-function path: which perspective-pass modelview load is the camera
    double worldUp[3]       = { 0, 1, 0 };
    double snapAngleDeg     = 60.0;       // bigger jumps are camera cuts: reset the filters
    double teleportDistance = 5.0;
    double bikeSearchRadius = 6.0;        // how close to the camera the rider's bike is drawn

    // [Camera]
    bool   customCamera   = false;        // build the view from the bike, ignoring the game's 1st-person settings
    bool   keepGameAngle  = false;        // custom camera: keep the game's look-down angle
    double cameraAngleDeg = 8.0;          // custom camera: look-down angle (positive = down)
    double headForward    = 0.0;          // eye offset along the bike, units (+ = forward)
    double headHeight     = 0.0;          // eye offset along the bike's up axis (+ = up)

    // [Smoothing]
    double rotationResponse = 14.0;       // 1/s, 0 = off
    double maxLagDeg        = 12.0;

    // [Leveling]
    double horizonLock          = 0.6;    // 0..1 fraction of roll removed
    double pitchLock            = 0.5;    // 0..1 fraction of the bike's pitch removed (wheelies, jump faces)
    double maxRollCorrectionDeg = 90.0;

    // [Air]
    double airLooseness     = 0.6;        // 0..1 how much of the nose's left / right swing in the air the view lets pass
    double whipLock         = 0.0;        // 0..1 in the air: hold the view on the direction of travel, level

    // [Look] the controller's d-pad looks left / right / back (the game has no look keys)
    bool   controllerLook  = true;
    double lookSideDeg     = 90.0;        // how far left / right looks turn the head
    // extra eye offsets while looking sideways / back (custom camera)
    double lookSideForward = 0.0;
    double lookSideHeight  = 0.0;
    double lookBackForward = 0.0;
    double lookBackHeight  = 0.0;

    // [Neck]
    double verticalResponse = 20.0;       // 1/s, 0 = off
    double maxVerticalLag   = 0.05;
    double compressionPerG  = 0.012;      // units of sag per g of vertical acceleration
    double springHz         = 3.0;
    double dampingRatio     = 0.5;
    double maxCompression   = 0.07;
    double maxExtension     = 0.04;
    double nodDegPerMeter   = 40.0;
    double accelFilterHz    = 12.0;

    // [Replay]
    bool   replayChaseCam   = false;      // film replays from behind the rider instead of the game's replay camera
    double replayDistance   = 4.5;        // 3rd person camera: m behind the rider
    double replayHeight     = 1.55;       // m above the rider (the middle of its body)
    double replayPitchDeg   = 12.0;       // look-down angle (positive = down)
};

const Config& GetConfig();
Config& EditConfig();  // for the settings panel; changes apply from the next frame

// Remembers the path and loads it. A missing file just leaves the defaults in place.
void LoadConfig(const wchar_t* iniPath);
void ReloadConfig();
void SetEnabled(bool enabled);

// Writes Enabled, MenuKey and the camera tunables back to mxcam.ini (comments and other keys are kept).
bool SaveConfig();
// Puts the camera tunables back to their defaults; keys and detection settings are untouched.
void ResetTunables();

}  // namespace mxcam
