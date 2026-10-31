// replay.cpp
//   Recognising a replay (see replay.h), and the chase camera.
//
//   The chase camera needs to know where the rider is this frame, but the terrain (whose upload
//   is the camera) is drawn before the rider. The game's replay camera is aimed at the rider, so
//   the rider is placed relative to it: the rider's offset from the replay camera is measured on
//   the bones each frame in the replay camera's own axes (it orbits the rider, so in world space
//   that offset keeps turning), smoothed, and applied to the next frame's replay camera.
//   Everything that sets the chase camera's orientation (heading, slope, ease-in) is worked out
//   once a frame is drawn, so the next frame's orientation is already known when its sky is.

#include <windows.h>
#include <cmath>
#include <cstdio>

#include "config.h"
#include "log.h"
#include "mathlib.h"
#include "replay.h"

namespace mxcam {
namespace replay {
namespace {

// Detection. The replay camera looks down at exactly atan(5 / 10) with a perfectly level
// horizon, whichever way it faces as it orbits the rider. Held to a twentieth of a degree for
// kEnterSeconds, that can't be a ride. Once in a replay the camera may rise and look down more
// steeply (it seems to lift over the terrain) but stays level; it ends when the horizon tilts,
// or when the camera jumps somewhere it no longer looks the replay way.
constexpr double kReplayPitchDeg = 26.565;
constexpr double kReplayDistance = 11.18;   // from the rider: sqrt(5^2 + 10^2)
constexpr double kAngleTolerance = 0.05;    // degrees
constexpr double kEnterSeconds   = 0.3;
constexpr double kJumpDistance   = 8.0;     // m in one frame

// Chase camera.
constexpr double kSlopeFollow    = 0.5;     // fraction of the slope ridden that tilts the camera with it
constexpr double kMaxSlopeDeg    = 25.0;
constexpr double kTurnRate       = 4.0;     // 1/s: how fast it swings round behind the rider
constexpr double kOffsetRate     = 2.0;     // 1/s: smoothing of the rider's offset from the replay camera
constexpr double kEaseSeconds    = 0.8;     // from the game's replay view to the chase view
constexpr double kMinSpeed       = 1.5;     // m/s: slower than this the heading is held

constexpr int kMaxRiders = 64;
constexpr int kMaxBones  = 256;

struct Detection {
    bool   haveLast      = false;
    Vec3   lastPos;
    double lastTime      = 0;
    double signatureTime = 0;  // how long the camera has looked exactly the replay way
    bool   active        = false;
};

// This frame's replay camera.
struct ReplayCamera {
    Quat rot;
    Vec3 pos;
    Vec3 right, up, back;
};

struct Chase {
    bool   haveRider     = false;
    Vec3   offset;               // rider minus the replay camera, in its axes (right, up, back), smoothed
    bool   haveLastRider = false;
    Vec3   lastRider;
    double lastTime      = -1;
    Vec3   velocity;
    bool   haveHeading   = false;
    bool   snapHeading   = false;  // take the next heading as it is (after a seek or a change of rider)
    Vec3   heading;                // horizontal, unit
    double slope         = 0;      // radians, + = climbing
    double ease          = 0;      // 0..1, linear; the view blends by its smoothstep
    // Prepared for the next frame.
    Quat   rot;
    Vec3   eyeOffset;              // eye minus the rider, world space
};

Detection    g_detect;
ReplayCamera g_camera;
Chase        g_chase;
Mode         g_mode = Mode::Off;
Vec3         g_riders[kMaxRiders];  // the middle of each skinned mesh's bones this frame
int          g_riderCount = 0;

Vec3 WorldUp()
{
    const Config& cfg = GetConfig();
    return Normalize(Vec3{ cfg.worldUp[0], cfg.worldUp[1], cfg.worldUp[2] });
}

double LowPassAlpha(double ratePerSecond, double dt) { return 1.0 - std::exp(-ratePerSecond * dt); }

double SmoothStep(double edge0, double edge1, double x)
{
    const double t = Clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

double Blend() { return SmoothStep(0.0, 1.0, g_chase.ease); }

// Camera axes and position of a rigid view matrix; false if it isn't one.
bool Decompose(const double view[16], Vec3& right, Vec3& up, Vec3& back, Vec3& pos)
{
    right = { view[0], view[4], view[8] };
    up    = { view[1], view[5], view[9] };
    back  = { view[2], view[6], view[10] };
    if (std::fabs(Length(right) - 1.0) > 0.01 || std::fabs(Length(up) - 1.0) > 0.01 ||
        std::fabs(Length(back) - 1.0) > 0.01 || Dot(Cross(right, up), back) < 0.9)
        return false;
    back  = Normalize(back);
    right = Normalize(Cross(up, back));
    up    = Cross(back, right);
    const Vec3 t{ view[12], view[13], view[14] };
    pos = -(right * t.x + up * t.y + back * t.z);
    return true;
}

// From the replay camera's axes to world space and back.
Vec3 ToWorld(Vec3 local) { return g_camera.right * local.x + g_camera.up * local.y + g_camera.back * local.z; }
Vec3 ToCamera(Vec3 world) { return { Dot(world, g_camera.right), Dot(world, g_camera.up), Dot(world, g_camera.back) }; }

// out(r, c) = dot(new axis r, raw axis c): the rotation taking the raw view to rot.
void RotationCorrection(Quat rot, Vec3 rawRight, Vec3 rawUp, Vec3 rawBack, double out[16])
{
    const Vec3 newAxes[3] = { Rotate(rot, { 1, 0, 0 }), Rotate(rot, { 0, 1, 0 }), Rotate(rot, { 0, 0, 1 }) };
    const Vec3 rawAxes[3] = { rawRight, rawUp, rawBack };
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r)
            out[c * 4 + r] = (r < 3 && c < 3) ? Dot(newAxes[r], rawAxes[c]) : (r == c ? 1.0 : 0.0);
    }
}

void Stop(const char* why)
{
    if (g_detect.active)
        Log("replay: over (%s), back to the helmet cam", why);
    g_detect.active = false;
    g_detect.signatureTime = 0;
    g_mode = Mode::Off;
}

// Which skinned mesh is the rider the replay camera follows: the one nearest where it was last
// frame, or else nearest the point the camera looks at. Returns the rider's offset from the
// camera, in the camera's axes.
Vec3 MeasureOffset()
{
    const Chase& c = g_chase;
    const Vec3 lookAt{ 0, 0, -kReplayDistance };
    const auto Nearest = [](Vec3 around, double within) {
        int best = -1;
        for (int i = 0; i < g_riderCount; ++i) {
            const double distance = Length(g_riders[i] - around);
            if (distance < within) {
                within = distance;
                best = i;
            }
        }
        return best;
    };
    int rider = c.haveRider ? Nearest(g_camera.pos + ToWorld(c.offset), 3.0) : -1;
    if (rider < 0)
        rider = Nearest(g_camera.pos + ToWorld(lookAt), 8.0);
    if (rider >= 0)
        return ToCamera(g_riders[rider] - g_camera.pos);
    // No bones this frame (or none near): keep the last offset, or aim where the camera looks.
    return c.haveRider ? c.offset : lookAt;
}

}  // namespace

void ObserveBones(const float* positions, int count)
{
    if (!g_detect.active || !positions || count <= 0 || count > kMaxBones || g_riderCount >= kMaxRiders)
        return;
    Vec3 sum;
    for (int i = 0; i < count; ++i)
        sum = sum + Vec3{ positions[i * 3], positions[i * 3 + 1], positions[i * 3 + 2] };
    g_riders[g_riderCount++] = sum / count;
}

Mode Update(const double view[16], double contentTime, double correction[16], double rotationOnly[16])
{
    Detection& d = g_detect;
    Vec3 right, up, back, pos;
    if (!Decompose(view, right, up, back, pos)) {
        Stop("not a camera");
        d.haveLast = false;
        return g_mode;
    }
    const Vec3 worldUp = WorldUp();
    const double dt = d.haveLast ? Clamp(contentTime - d.lastTime, 0.0, 0.1) : 0.0;
    const double tolerance = std::sin(kAngleTolerance * kDegToRad);
    const bool level = std::fabs(Dot(right, worldUp)) < tolerance;
    const bool replayAngle = level && std::fabs(Dot(back, worldUp) - std::sin(kReplayPitchDeg * kDegToRad)) < tolerance;
    const bool jumped = d.haveLast && Length(pos - d.lastPos) > kJumpDistance;
    if (d.active && !level)
        Stop("the horizon tilted");
    else if (d.active && jumped && !replayAngle)
        Stop("the camera jumped");
    d.signatureTime = replayAngle ? d.signatureTime + dt : 0.0;
    d.haveLast = true;
    d.lastPos  = pos;
    d.lastTime = contentTime;
    g_camera = { QuatFromBasis(right, up, back), pos, right, up, back };

    if (!d.active && d.signatureTime >= kEnterSeconds) {
        d.active = true;
        g_chase = Chase{};
        Log("replay: detected, showing the %s camera", GetConfig().replayChaseCam ? "3rd person" : "game's replay");
    }
    if (!d.active)
        return g_mode = Mode::Off;
    if (!GetConfig().replayChaseCam || g_chase.ease <= 0)
        return g_mode = Mode::Game;

    // Eases from the game's view to the chase view: rotation and position together.
    const Chase& c = g_chase;
    const double w = Blend();
    const Vec3 chaseEye = pos + ToWorld(c.offset) + c.eyeOffset;
    const Vec3 eye = pos + (chaseEye - pos) * w;
    const Quat q = Slerp(g_camera.rot, c.rot, w);
    const Vec3 R = Rotate(q, { 1, 0, 0 }), U = Rotate(q, { 0, 1, 0 }), B = Rotate(q, { 0, 0, 1 });
    const double newView[16] = {
        R.x, U.x, B.x, 0,
        R.y, U.y, B.y, 0,
        R.z, U.z, B.z, 0,
        -Dot(R, eye), -Dot(U, eye), -Dot(B, eye), 1,
    };
    double rawInverse[16];
    if (!AffineInverse(view, rawInverse))
        return g_mode = Mode::Game;
    MatMul(newView, rawInverse, correction);
    RotationCorrection(q, right, up, back, rotationOnly);
    return g_mode = Mode::Chase;
}

void EndFrame(double contentTime, bool captured)
{
    if (!captured) {
        Stop("no camera");
        g_detect.haveLast = false;
    }
    if (!g_detect.active) {
        g_riderCount = 0;
        return;
    }

    const Config& cfg = GetConfig();
    Chase& c = g_chase;
    const Vec3 worldUp = WorldUp();
    const double dt = c.lastTime >= 0 ? Clamp(contentTime - c.lastTime, 0.0, 0.1) : 0.0;
    c.lastTime = contentTime;

    // ---- Where the rider is, relative to the replay camera -------------------------------
    const Vec3 measured = MeasureOffset();
    g_riderCount = 0;
    if (!c.haveRider || Length(measured - c.offset) > 3.0)
        c.offset = measured;  // first frame, or the replay camera moved on to another rider
    else
        c.offset = c.offset + (measured - c.offset) * LowPassAlpha(kOffsetRate, dt);
    c.haveRider = true;

    // ---- How the rider moves ---------------------------------------------------------------
    const Vec3 rider = g_camera.pos + ToWorld(c.offset);
    if (!c.haveLastRider || Length(rider - c.lastRider) > 5.0) {  // first frame, a seek or another rider
        c.velocity = Vec3{};
        c.snapHeading = true;
    } else if (dt > 0) {
        c.velocity = c.velocity + ((rider - c.lastRider) / dt - c.velocity) * LowPassAlpha(6.0, dt);
    }
    c.lastRider = rider;
    c.haveLastRider = true;

    Vec3 velocity = c.velocity;
    Vec3 flat = velocity - worldUp * Dot(velocity, worldUp);
    const double speed = Length(flat);
    if (speed > kMinSpeed) {
        // A bike can't turn round on the spot: the replay is being played backwards.
        if (c.haveHeading && !c.snapHeading && Dot(flat, c.heading) < -0.5 * speed) {
            velocity = -velocity;
            flat = -flat;
        }
        const Vec3 direction = flat / speed;
        const double slope = Clamp(std::atan2(Dot(velocity, worldUp), speed), -kMaxSlopeDeg * kDegToRad, kMaxSlopeDeg * kDegToRad);
        if (!c.haveHeading || c.snapHeading) {
            c.heading = direction;
            c.slope = slope;
            c.haveHeading = true;
            c.snapHeading = false;
        } else {
            const double weight = SmoothStep(kMinSpeed, 4.0, speed);
            c.heading = Normalize(c.heading + (direction - c.heading) * (LowPassAlpha(kTurnRate, dt) * weight));
            c.slope += (slope - c.slope) * (LowPassAlpha(2.0, dt) * weight);
        }
    }

    // ---- Ease in, and the next frame's view ------------------------------------------------
    // Until the rider has moved (a replay starting at the gate) the game's view stays.
    c.ease = (cfg.replayChaseCam && c.haveHeading) ? std::fmin(1.0, c.ease + dt / kEaseSeconds) : 0.0;
    if (c.haveHeading) {
        // Behind and above the rider, tilted with part of the slope being ridden: downhill the
        // ground behind is higher, so the camera rises with it (and pitches down to match).
        const double tilt = kSlopeFollow * c.slope;
        const Vec3 along  = c.heading * std::cos(tilt) + worldUp * std::sin(tilt);
        const Vec3 tiltUp = worldUp * std::cos(tilt) - c.heading * std::sin(tilt);
        c.eyeOffset = tiltUp * cfg.replayHeight - along * cfg.replayDistance;
        const double pitch = tilt - cfg.replayPitchDeg * kDegToRad;
        const Vec3 forward = c.heading * std::cos(pitch) + worldUp * std::sin(pitch);
        const Vec3 right = Normalize(Cross(forward, worldUp));
        c.rot = QuatFromBasis(right, Cross(right, forward), -forward);
    }
}

Mode Current() { return g_mode; }

double AngleToCameraDeg(const double view[16])
{
    Vec3 right, up, back, pos;
    if (!Decompose(view, right, up, back, pos))
        return -1;
    return AngleBetween(g_camera.rot, QuatFromBasis(right, up, back)) * kRadToDeg;
}

bool PreviewRotation(const double view[16], double out[16])
{
    Vec3 right, up, back, pos;
    if (g_mode != Mode::Chase || !Decompose(view, right, up, back, pos))
        return false;
    RotationCorrection(Slerp(QuatFromBasis(right, up, back), g_chase.rot, Blend()), right, up, back, out);
    return true;
}

const char* StatusText()
{
    if (!g_detect.active)
        return "NOT IN A REPLAY";
    if (!GetConfig().replayChaseCam)
        return "REPLAY - GAME CAMERA";
    return g_chase.haveHeading ? "REPLAY - 3RD PERSON CAMERA" : "REPLAY - WAITING FOR THE RIDER TO MOVE";
}

void Describe(char* text, size_t size)
{
    const Chase& c = g_chase;
    const Vec3 rider = g_camera.pos + ToWorld(c.offset);
    snprintf(text, size, "replay,%s,ease %.2f,camera %.2f %.2f %.2f,rider %.2f %.2f %.2f,heading %.1f,speed %.1f,slope %.1f",
             g_mode == Mode::Chase ? "chase" : "game", c.ease, g_camera.pos.x, g_camera.pos.y, g_camera.pos.z,
             rider.x, rider.y, rider.z, std::atan2(c.heading.x, -c.heading.z) * kRadToDeg, Length(c.velocity),
             c.slope * kRadToDeg);
}

}  // namespace replay
}  // namespace mxcam
