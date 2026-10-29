// camera.cpp
//   Per-frame pipeline (all filters are frame-rate independent), in two phases:
//   OrientFrame - everything about where the view points. It only needs the raw camera rotation,
//                 so it can run as early as the sky, which is drawn before the camera position
//                 is known.
//     1. timing          - QueryPerformanceCounter delta since the last update
//     2. decompose       - raw view matrix -> head orientation (quaternion)
//     3. cut detection   - camera switches / look-back snaps reset every filter
//     5. attitude        - heading / pitch / roll of the game camera and, when the tracker has
//                          found it, of the rider's bike (bike.cpp)
//     6. target view     - game camera:   the game's view with pitch lock and horizon lock
//                          custom camera: a view built from the bike alone (the game's 1st
//                                         person angle and gimbal play no part), with the
//                                         locks, our own look-down angle and the game's
//                                         look-left / right / back passed through;
//                          both: whip centering while airborne, head position offsets
//     7. smoothing       - exponential slerp toward the target, with a lag limit; head nod
//   FinishFrame - where the eye is.
//     4. motion          - filtered velocity and acceleration of the eye; airborne detection
//     8. neck physics    - vertical low-pass + spring-damper driven by vertical acceleration
//     9. rebuild         - new view matrix, and correction C = NewView * inverse(RawView)

#include <windows.h>
#include <cmath>

#include "bike.h"
#include "camera.h"
#include "config.h"
#include "log.h"
#include "mathlib.h"

namespace mxcam {
namespace camera {
namespace {

// Learns the angle something rests at (the bike's pitch on flat ground, the game camera's
// look-down angle) as the peak of a histogram that forgets over ~30 s. Wheelies, jump faces
// and landings are brief, so they barely move the peak.
class RestEstimator {
public:
    void Add(double degrees, double dt)
    {
        if (!seeded_) {
            value_ = degrees;
            seeded_ = true;
        }
        if (dt <= 0)
            return;
        const double decay = std::exp(-dt / 30.0);
        for (double& bin : bins_)
            bin *= decay;
        const double position = Clamp(degrees, -kRange, kRange) + kRange;
        const int    i = position >= kBins - 1 ? kBins - 2 : static_cast<int>(position);
        const double f = position - i;
        bins_[i]     += dt * (1.0 - f);
        bins_[i + 1] += dt * f;

        int peak = 0;
        for (int b = 1; b < kBins; ++b) {
            if (bins_[b] > bins_[peak])
                peak = b;
        }
        double offset = 0;  // parabolic refinement between the 1-degree bins
        if (peak > 0 && peak < kBins - 1) {
            const double a = bins_[peak - 1], b = bins_[peak], c = bins_[peak + 1];
            const double denominator = a - 2.0 * b + c;
            if (std::fabs(denominator) > 1e-12)
                offset = Clamp(0.5 * (a - c) / denominator, -0.5, 0.5);
        }
        const double mode = peak + offset - kRange;
        value_ += (mode - value_) * (1.0 - std::exp(-0.7 * dt));
    }
    double Degrees() const { return value_; }

private:
    static constexpr int    kBins  = 121;  // -60..+60 degrees
    static constexpr double kRange = 60.0;
    double bins_[kBins] = {};
    double value_  = 0;
    bool   seeded_ = false;
};

struct State {
    bool   valid        = false;  // false: the next Update() seeds every filter from the raw camera
    bool   haveVelocity = false;
    bool   havePos      = false;  // false: FinishFrame() seeds the position filters
    Quat   smoothRot;             // low-passed view orientation
    Quat   prevRawRot;
    Vec3   prevPos;
    Vec3   velocity;              // filtered world-space velocity (units/s)
    Vec3   accel;                 // filtered world-space acceleration (units/s^2)
    double airborne     = 0;      // 0 on the ground .. 1 in free fall
    double lookSide     = 0;      // 0..1 how much the rider is looking sideways / back
    double lookBack     = 0;
    double smoothHeight = 0;      // low-passed eye height along WorldUp
    double neckOffset   = 0;      // spring displacement along WorldUp (negative = compressed)
    double neckVelocity = 0;
};

State         g_state;
Telemetry     g_telemetry;
LARGE_INTEGER g_lastTick{};      // kept outside State so a filter reset doesn't break timing
int           g_cutsLogged = 0;
// Learned over many seconds, so they survive filter resets.
RestEstimator g_cameraPitchRest;  // game camera pitch on flat ground (includes the game's angle)
RestEstimator g_bikePitchRest;    // bike model pitch on flat ground
RestEstimator g_gameAngleRest;    // game camera pitch relative to the bike: the game's 1st-person angle
Vec3          g_lastHeading{ 0, 0, -1 };
double        g_customWeight = 0; // 0 = game camera .. 1 = custom camera, eased on switches
bike::Pose    g_lastPose;         // last pose the tracker reported, used while easing out
bool          g_haveLastPose = false;

double SecondsSinceLastUpdate()
{
    LARGE_INTEGER now, frequency;
    QueryPerformanceCounter(&now);
    QueryPerformanceFrequency(&frequency);
    const double seconds = g_lastTick.QuadPart
        ? double(now.QuadPart - g_lastTick.QuadPart) / double(frequency.QuadPart)
        : 0.0;
    g_lastTick = now;
    return seconds;
}

// Blend factor of a first-order low-pass with the given rate (1/s) over dt seconds.
// Frame-rate independent: two 8 ms steps give the same result as one 16 ms step.
// A rate of 0 means "no filtering" (factor 1).
double LowPassAlpha(double ratePerSecond, double dt)
{
    return ratePerSecond > 0 ? 1.0 - std::exp(-ratePerSecond * dt) : 1.0;
}

double SmoothStep(double edge0, double edge1, double x)
{
    const double t = Clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

// Neck as a damped spring along WorldUp. The rider's body is accelerated by the bike; the
// head lags behind. Under a steady vertical acceleration a the neck settles at
//   offset = -CompressionPerG * a / g
// so a landing (large upward acceleration) compresses it and airtime (-1 g) extends it.
void StepNeck(double verticalAccel, double dt, const Config& cfg)
{
    const double omega = 2.0 * kPi * cfg.springHz;
    if (omega <= 0) {
        g_state.neckOffset = g_state.neckVelocity = 0;
        return;
    }
    const double stiffness = omega * omega;
    const double damping   = 2.0 * cfg.dampingRatio * omega;
    const double target    = -cfg.compressionPerG * verticalAccel / kGravity;

    // 1 ms semi-implicit Euler substeps keep a stiff spring stable at any frame rate.
    const int    steps = static_cast<int>(std::ceil(dt * 1000.0));
    const double h     = dt / steps;
    double& x = g_state.neckOffset;
    double& v = g_state.neckVelocity;
    for (int i = 0; i < steps; ++i) {
        v += (stiffness * (target - x) - damping * v) * h;
        x += v * h;
    }

    // End stops: the neck can't compress or stretch indefinitely.
    if (x < -cfg.maxCompression) { x = -cfg.maxCompression; if (v < 0) v = 0; }
    if (x >  cfg.maxExtension)   { x =  cfg.maxExtension;   if (v > 0) v = 0; }
}

// Heading (horizontal unit vector), pitch (radians, + = nose up) and roll (radians, + = right
// side up) of a forward / up pair relative to the horizon.
struct Attitude {
    Vec3   heading;
    double pitch = 0;
    double roll  = 0;
    double level = 1;  // ~cos(pitch); heading and roll are meaningless near 0
};

Attitude Measure(Vec3 forward, Vec3 up, Vec3 worldUp)
{
    Attitude a;
    const double sinPitch = Clamp(Dot(forward, worldUp), -1.0, 1.0);
    a.pitch = std::asin(sinPitch);
    const Vec3 horizontal = forward - worldUp * sinPitch;
    a.level = Length(horizontal);
    a.heading = a.level > 0.05 ? horizontal / a.level : g_lastHeading;
    const Vec3 levelRight = Normalize(Cross(a.heading, worldUp));  // a level head's right axis
    const Vec3 levelUp    = Cross(levelRight, forward);
    const Vec3 right      = Cross(forward, up);
    a.roll = std::atan2(Dot(right, levelUp), Dot(right, levelRight));
    return a;
}

// Orientation (camera-local -> world) from heading, pitch and roll; the inverse of Measure.
Quat Build(Vec3 heading, double pitch, double roll, Vec3 worldUp, double lookYaw = 0)
{
    const Vec3 levelRight = Normalize(Cross(heading, worldUp));
    Vec3 forward = heading * std::cos(pitch) + worldUp * std::sin(pitch);
    const Vec3 levelUp = Cross(levelRight, forward);
    Vec3 right = levelRight * std::cos(roll) + levelUp * std::sin(roll);
    const Vec3 up = Cross(right, forward);
    if (lookYaw != 0) {  // turn the head about its own up axis (+ = look right)
        const Vec3 f = forward * std::cos(lookYaw) + right * std::sin(lookYaw);
        right = right * std::cos(lookYaw) - forward * std::sin(lookYaw);
        forward = f;
    }
    return QuatFromBasis(right, up, -forward);
}

// Rotates a horizontal heading about WorldUp (+ = turn right).
Vec3 Turn(Vec3 heading, double radians, Vec3 worldUp)
{
    const Vec3 right = Normalize(Cross(heading, worldUp));
    return heading * std::cos(radians) + right * std::sin(radians);
}

// Roll after removing `lock` of it, capped at MaxRollCorrectionDeg and faded out near vertical.
double LockedRoll(const Attitude& a, double lock, const Config& cfg)
{
    const double fade    = Clamp((a.level - 0.1) / 0.3, 0.0, 1.0);
    const double maxCorr = cfg.maxRollCorrectionDeg * kDegToRad;
    return a.roll + Clamp(-a.roll * lock * fade, -maxCorr, maxCorr);
}

// Camera axes (unit, right-handed) and uniform scale of a view matrix; false if it isn't rigid.
bool DecomposeView(const double view[16], Vec3& right, Vec3& up, Vec3& back, double& scale)
{
    // A view matrix is [ s*R | t ] with bottom row 0,0,0,1. The rows of R are the camera's
    // right / up / back axes expressed in world space; the camera looks down -back.
    if (std::fabs(view[3]) > 1e-4 || std::fabs(view[7]) > 1e-4 || std::fabs(view[11]) > 1e-4 ||
        std::fabs(view[15] - 1.0) > 1e-4)
        return false;
    right = { view[0], view[4], view[8] };
    up    = { view[1], view[5], view[9] };
    back  = { view[2], view[6], view[10] };
    scale = Length(right);  // engines sometimes bake a uniform world scale in
    if (scale < 1e-9 || std::fabs(Length(up) / scale - 1.0) > 0.01 || std::fabs(Length(back) / scale - 1.0) > 0.01)
        return false;  // non-uniform scale: not a camera
    right = right / scale;
    up    = up / scale;
    back  = back / scale;
    if (Dot(Cross(right, up), back) < 0.9)
        return false;  // mirrored or sheared: e.g. a reflection pass
    // Re-orthonormalise to remove float drift before converting to a quaternion.
    back  = Normalize(back);
    right = Normalize(Cross(up, back));
    up    = Cross(back, right);
    return true;
}

// Carried from OrientFrame() to FinishFrame() within one frame.
struct Frame {
    bool   oriented = false;            // OrientFrame() ran and FinishFrame() has not yet
    bool   haveRotation = false;        // rot / raw axes hold a valid orientation (for RotationCorrection)
    double dt       = 0;
    bool   reset    = false;
    Vec3   rawRight, rawUp, rawBack;    // raw camera axes the orientation was computed from
    Quat   rot;                         // final view orientation
    Vec3   offsetFwd, offsetUp;         // head offset axes and amounts
    double forwardOffset = 0, heightOffset = 0;
};
Frame g_frame;

}  // namespace

void Reset() { g_state.valid = false; }

const Telemetry& LastTelemetry() { return g_telemetry; }

void AbandonFrame() { g_frame.oriented = false; }

bool OrientFrame(const double view[16])
{
    const Config& cfg = GetConfig();
    const Vec3 worldUp{ cfg.worldUp[0], cfg.worldUp[1], cfg.worldUp[2] };
    g_frame.oriented = false;
    g_frame.haveRotation = false;

    // ---- 1. Timing --------------------------------------------------------------------
    double dt = SecondsSinceLastUpdate();
    if (dt > 0.25)
        g_state.valid = false;  // paused, alt-tabbed or loading: resync rather than integrate a huge step
    dt = Clamp(dt, 0.0, 0.05);

    // ---- 2. Decompose the raw view matrix ---------------------------------------------
    Vec3 right, up, back;
    double scale;
    if (!DecomposeView(view, right, up, back, scale))
        return false;
    const Quat rawRot = QuatFromBasis(right, up, back);
    const Vec3 rawFwd = -back;

    // ---- 3. Camera cut detection (rotation; position jumps are handled in FinishFrame) ----
    // Compared raw to raw: the corrected view may sit tens of degrees from the raw one.
    if (g_state.valid) {
        const double jumpDeg = AngleBetween(g_state.prevRawRot, rawRot) * kRadToDeg;
        if (jumpDeg > cfg.snapAngleDeg) {
            if (g_cutsLogged++ < 20)
                Log("camera: cut detected (%.1f deg turn), resetting filters", jumpDeg);
            g_state.valid = false;
        }
    }
    const bool reset = !g_state.valid;
    if (reset) {
        g_state = State{};
        g_state.valid = true;
        dt = 0;
    }
    g_state.prevRawRot = rawRot;

    // Motion state is updated in FinishFrame, so these are one frame old: fine for slow signals.
    const double verticalAccel = Dot(g_state.accel, worldUp);
    const bool   onGround      = g_state.airborne < 0.3;

    // ---- 5. Attitude of the game camera and of the bike -----------------------------------
    const Attitude cameraAtt = Measure(rawFwd, up, worldUp);
    if (onGround && cameraAtt.level > 0.5)
        g_cameraPitchRest.Add(cameraAtt.pitch * kRadToDeg, dt);

    // The custom camera eases in when the bike is found and out when it is lost; while easing
    // out it keeps using the last known bike pose (the bike barely moves against the camera).
    const bike::Pose& pose = bike::Current();
    if (pose.tracked) {
        g_lastPose = pose;
        g_haveLastPose = true;
    }
    g_customWeight += (((cfg.customCamera && pose.tracked) ? 1.0 : 0.0) - g_customWeight) * LowPassAlpha(3.0, dt);
    if (!g_haveLastPose)
        g_customWeight = 0;

    bool     haveBike = false;
    Vec3     bikeFwd, bikeUp;
    Attitude bikeAtt;
    double   lookYaw = 0;
    if (g_haveLastPose && (pose.tracked || g_customWeight > 1e-3)) {
        // The bike's axes in camera space (its model axes through View * Model), then in world
        // space through the camera axes.
        const double* m = g_lastPose.cameraSpace;
        const auto ToCamera = [m](const double a[3]) {
            return Vec3{ m[0] * a[0] + m[4] * a[1] + m[8] * a[2],
                         m[1] * a[0] + m[5] * a[1] + m[9] * a[2],
                         m[2] * a[0] + m[6] * a[1] + m[10] * a[2] };
        };
        const auto ToWorld = [&](Vec3 v) { return right * v.x + up * v.y + back * v.z; };
        bikeFwd = Normalize(ToWorld(ToCamera(g_lastPose.forward)));
        const Vec3 u = ToWorld(ToCamera(g_lastPose.up));
        bikeUp = Normalize(u - bikeFwd * Dot(u, bikeFwd));
        haveBike = Length(Cross(bikeFwd, bikeUp)) > 0.5;
    }
    if (haveBike) {
        bikeAtt = Measure(bikeFwd, bikeUp, worldUp);
        if (onGround && bikeAtt.level > 0.5 && pose.tracked)
            g_bikePitchRest.Add(bikeAtt.pitch * kRadToDeg, dt);

        // Where the game camera points relative to the bike: its look-down angle, and the
        // look-left / right / back keys.
        const Vec3 bikeRight = Cross(bikeFwd, bikeUp);
        const double x = Dot(rawFwd, bikeRight), y = Dot(rawFwd, bikeUp), z = Dot(rawFwd, bikeFwd);
        lookYaw = std::atan2(x, z);
        if (std::fabs(lookYaw) < 20.0 * kDegToRad && pose.tracked)
            g_gameAngleRest.Add(std::atan2(y, std::sqrt(x * x + z * z)) * kRadToDeg, dt);
    } else {
        g_customWeight = 0;
    }
    const double absYawDeg = std::fabs(lookYaw) * kRadToDeg;

    // Pitch lock works on the pitch *away from rest*, so riding on flat ground looks the same
    // at any setting and only wheelies, stoppies and jump faces are evened out.
    const double cameraPitchExcess = cameraAtt.pitch - g_cameraPitchRest.Degrees() * kDegToRad;
    const double bikePitchExcess   = haveBike ? bikeAtt.pitch - g_bikePitchRest.Degrees() * kDegToRad : 0.0;

    // ---- 6. Target view -------------------------------------------------------------------
    // Whip centering: while airborne, turn the view from the bike's nose toward the direction of
    // travel. Inside WhipFreeAngle the view stays on the travel direction; past it the view
    // follows the extra swing. WhipFollow blends back toward the nose.
    const auto WhipCorrection = [&](Vec3 nose) {
        Vec3 travel = g_state.velocity - worldUp * Dot(g_state.velocity, worldUp);
        const double speed = Length(travel);
        if (!cfg.whipCentering || speed < 1e-3)
            return 0.0;
        travel = travel / speed;
        const double w       = std::atan2(Dot(nose, Normalize(Cross(travel, worldUp))), Dot(nose, travel));
        const double free    = cfg.whipFreeAngleDeg * kDegToRad;
        const double excess  = w > 0 ? std::fmax(0.0, w - free) : std::fmin(0.0, w + free);
        const double wanted  = excess + cfg.whipFollow * (w - excess);
        // Only in the air, at speed, and never for a sideways look or a bike going backwards.
        const double weight  = g_state.airborne * SmoothStep(3.0, 6.0, speed) *
                               (1.0 - SmoothStep(60.0 * kDegToRad, 80.0 * kDegToRad, std::fabs(w)));
        return (wanted - w) * weight;
    };

    // Game camera: the game's view with the locks applied. (It is fixed to the bike, so its
    // own pitch away from rest is the bike's.)
    double whip = WhipCorrection(cameraAtt.heading);
    const double gamePitch = cameraAtt.pitch - cfg.pitchLock * cameraPitchExcess;
    const double gameRoll  = LockedRoll(cameraAtt, cfg.horizonLock, cfg);
    Quat target = Build(Turn(cameraAtt.heading, whip, worldUp), gamePitch, gameRoll, worldUp);

    // Custom camera: built from the bike only.
    if (g_customWeight > 1e-3) {
        const double angleDown = cfg.keepGameAngle ? -g_gameAngleRest.Degrees() : cfg.cameraAngleDeg;
        const double pitch = (1.0 - cfg.pitchLock) * bikePitchExcess - angleDown * kDegToRad;
        const double roll  = LockedRoll(bikeAtt, cfg.horizonLock, cfg);
        // The game's look keys turn the camera 90 / 180 degrees; small offsets (the game's own
        // whip or gimbal) are ignored.
        const double look = lookYaw * SmoothStep(20.0, 45.0, absYawDeg);
        const double customWhip = WhipCorrection(bikeAtt.heading);
        const Quat custom = Build(Turn(bikeAtt.heading, customWhip, worldUp), pitch, roll, worldUp, look);
        target = Slerp(target, custom, g_customWeight);
        whip += (customWhip - whip) * g_customWeight;
    }
    g_lastHeading = cameraAtt.heading;

    // Head position: along the bike (custom) or the game camera's own axes.
    const double lookAlpha = LowPassAlpha(10.0, dt);
    const double sideTarget = haveBike ? SmoothStep(30.0, 70.0, absYawDeg) * (1.0 - SmoothStep(110.0, 150.0, absYawDeg)) : 0.0;
    const double backTarget = haveBike ? SmoothStep(110.0, 150.0, absYawDeg) : 0.0;
    g_state.lookSide += (sideTarget - g_state.lookSide) * (reset ? 1.0 : lookAlpha);
    g_state.lookBack += (backTarget - g_state.lookBack) * (reset ? 1.0 : lookAlpha);
    const double side = g_state.lookSide * g_customWeight, rear = g_state.lookBack * g_customWeight;
    g_frame.forwardOffset = cfg.headForward + side * cfg.lookSideForward + rear * cfg.lookBackForward;
    g_frame.heightOffset  = cfg.headHeight + side * cfg.lookSideHeight + rear * cfg.lookBackHeight;
    g_frame.offsetFwd = rawFwd;
    g_frame.offsetUp  = up;
    if (haveBike) {
        g_frame.offsetFwd = g_frame.offsetFwd + (bikeFwd - g_frame.offsetFwd) * g_customWeight;
        g_frame.offsetUp  = g_frame.offsetUp + (bikeUp - g_frame.offsetUp) * g_customWeight;
    }

    // ---- 7. Inertial smoothing (pitch / yaw / roll) ---------------------------------------
    // Exponential slerp toward the target, filtering out high-frequency spikes. MaxLagDeg caps
    // how far behind the view can fall during sustained fast motion.
    Quat& smooth = g_state.smoothRot;
    if (reset)
        smooth = target;
    smooth = Slerp(smooth, target, LowPassAlpha(cfg.rotationResponse, dt));
    const double lag    = AngleBetween(smooth, target);
    const double maxLag = cfg.maxLagDeg * kDegToRad;
    if (lag > maxLag)
        smooth = Slerp(target, smooth, maxLag / lag);
    Quat rot = smooth;

    // Neck compression also nods the head forward a little (rotation about the camera's right
    // axis; a negative angle pitches down). Uses last frame's neck, so the orientation is
    // complete here - the sky, drawn before the camera position is known, gets all of it.
    if (cfg.nodDegPerMeter != 0) {
        const Vec3 pitchAxis = Rotate(rot, { 1, 0, 0 });
        rot = AxisAngle(pitchAxis, g_state.neckOffset * cfg.nodDegPerMeter * kDegToRad) * rot;
    }

    g_frame.rot      = Normalize(rot);
    g_frame.dt       = dt;
    g_frame.reset    = reset;
    g_frame.rawRight = right;
    g_frame.rawUp    = up;
    g_frame.rawBack  = back;
    g_frame.oriented = true;
    g_frame.haveRotation = true;

    const Vec3 B = Rotate(g_frame.rot, { 0, 0, 1 });
    const Attitude finalAtt = Measure(-B, Rotate(g_frame.rot, { 0, 1, 0 }), worldUp);
    g_telemetry.dtMs              = dt * 1000.0;
    g_telemetry.lagDeg            = AngleBetween(g_state.smoothRot, target) * kRadToDeg;
    g_telemetry.rawRollDeg        = (haveBike ? bikeAtt.roll : cameraAtt.roll) * kRadToDeg;
    g_telemetry.rollCorrectionDeg = (finalAtt.roll - cameraAtt.roll) * kRadToDeg;
    g_telemetry.pitchExcessDeg    = (g_customWeight > 0.5 ? bikePitchExcess : cameraPitchExcess) * kRadToDeg;
    g_telemetry.viewPitchDeg      = finalAtt.pitch * kRadToDeg;
    g_telemetry.lookYawDeg        = lookYaw * kRadToDeg;
    g_telemetry.whipDeg           = whip * kRadToDeg;
    g_telemetry.airborne          = g_state.airborne;
    g_telemetry.bikeTracked       = pose.tracked;
    g_telemetry.customActive      = g_customWeight > 0.5;
    g_telemetry.verticalAccelG    = verticalAccel / kGravity;
    return true;
}

bool RotationCorrection(double out[16])
{
    if (!g_frame.haveRotation)
        return false;
    // C = NewRotation * RawRotation^T; element (r, c) = dot(new axis r, raw axis c).
    const Vec3 newAxes[3] = { Rotate(g_frame.rot, { 1, 0, 0 }), Rotate(g_frame.rot, { 0, 1, 0 }),
                              Rotate(g_frame.rot, { 0, 0, 1 }) };
    const Vec3 rawAxes[3] = { g_frame.rawRight, g_frame.rawUp, g_frame.rawBack };
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r)
            out[c * 4 + r] = (r < 3 && c < 3) ? Dot(newAxes[r], rawAxes[c]) : (r == c ? 1.0 : 0.0);
    }
    return true;
}

bool FinishFrame(const double view[16], double correction[16])
{
    const Config& cfg = GetConfig();
    const Vec3 worldUp{ cfg.worldUp[0], cfg.worldUp[1], cfg.worldUp[2] };
    if (!g_frame.oriented)
        return false;
    g_frame.oriented = false;

    Vec3 right, up, back;
    double scale;
    if (!DecomposeView(view, right, up, back, scale))
        return false;
    const Vec3 t{ view[12], view[13], view[14] };
    const Vec3 rawPos = -(right * t.x + up * t.y + back * t.z) / scale;  // eye = -R^T t / s
    double dt = g_frame.dt;

    // A position jump (respawn) restarts the position filters only.
    if (g_state.havePos && Length(rawPos - g_state.prevPos) > cfg.teleportDistance) {
        if (g_cutsLogged++ < 20)
            Log("camera: cut detected (%.2f units), resetting the position filters", Length(rawPos - g_state.prevPos));
        g_state.havePos = false;
    }
    if (!g_state.havePos) {
        g_state.havePos      = true;
        g_state.haveVelocity = false;
        g_state.velocity = g_state.accel = Vec3{};
        g_state.neckOffset = g_state.neckVelocity = 0;
        g_state.prevPos      = rawPos;
        g_state.smoothHeight = Dot(rawPos, worldUp);
        dt = 0;
    }

    // ---- 4. Motion: velocity, acceleration, airborne ------------------------------------
    if (dt > 1e-5) {
        // Velocity and acceleration by finite differences, each low-passed to tame frame jitter.
        const Vec3 instantVelocity = (rawPos - g_state.prevPos) / dt;
        if (!g_state.haveVelocity) {
            // First sample after a reset: seed velocity so the "acceleration" from 0 to current
            // speed isn't mistaken for a huge impact.
            g_state.velocity = instantVelocity;
            g_state.haveVelocity = true;
        } else {
            const double alpha = LowPassAlpha(2.0 * kPi * cfg.accelFilterHz, dt);
            const Vec3 newVelocity = g_state.velocity + (instantVelocity - g_state.velocity) * alpha;
            Vec3 instantAccel = (newVelocity - g_state.velocity) / dt;
            const double maxAccel = 30.0 * kGravity;  // reject glitches (e.g. a missed cut)
            if (Length(instantAccel) > maxAccel)
                instantAccel = Normalize(instantAccel) * maxAccel;
            g_state.velocity = newVelocity;
            g_state.accel = g_state.accel + (instantAccel - g_state.accel) * alpha;
        }
    }
    const double verticalAccel = Dot(g_state.accel, worldUp);
    // In the air the eye is in free fall (-1 g); on the ground it hovers around 0 g.
    g_state.airborne += ((verticalAccel < -0.6 * kGravity ? 1.0 : 0.0) - g_state.airborne) * LowPassAlpha(6.0, dt);

    // ---- 8. Vertical damping & G-force neck compression ---------------------------------
    const double rawHeight = Dot(rawPos, worldUp);
    if (dt > 1e-5) {
        StepNeck(verticalAccel, dt, cfg);
        // Low-pass the eye height itself to iron out head-bob jitter from the animation.
        g_state.smoothHeight += (rawHeight - g_state.smoothHeight) * LowPassAlpha(cfg.verticalResponse, dt);
    }
    g_state.smoothHeight = Clamp(g_state.smoothHeight, rawHeight - cfg.maxVerticalLag, rawHeight + cfg.maxVerticalLag);
    g_state.prevPos = rawPos;

    const double verticalOffset = (g_state.smoothHeight - rawHeight) + g_state.neckOffset;
    const Vec3 headOffset = g_frame.offsetFwd * g_frame.forwardOffset + g_frame.offsetUp * g_frame.heightOffset;
    const Vec3 eye = rawPos + headOffset + worldUp * verticalOffset;

    // ---- 9. Rebuild the view matrix and derive the eye-space correction -----------------
    const Vec3 R = Rotate(g_frame.rot, { 1, 0, 0 });
    const Vec3 U = Rotate(g_frame.rot, { 0, 1, 0 });
    const Vec3 B = Rotate(g_frame.rot, { 0, 0, 1 });
    const double s = scale;
    const double newView[16] = {
        s * R.x, s * U.x, s * B.x, 0,
        s * R.y, s * U.y, s * B.y, 0,
        s * R.z, s * U.z, s * B.z, 0,
        -s * Dot(R, eye), -s * Dot(U, eye), -s * Dot(B, eye), 1,
    };
    double rawInverse[16];
    if (!AffineInverse(view, rawInverse))
        return false;
    MatMul(newView, rawInverse, correction);

    g_telemetry.verticalAccelG = verticalAccel / kGravity;
    g_telemetry.airborne       = g_state.airborne;
    g_telemetry.neckOffset     = g_state.neckOffset;
    g_telemetry.verticalOffset = verticalOffset;
    return true;
}

bool Update(const double view[16], double correction[16])
{
    // Reuse the orientation if it was already worked out this frame (from the sky) from the same
    // raw camera rotation; otherwise work it out now.
    bool sameRotation = false;
    if (g_frame.oriented) {
        Vec3 right, up, back;
        double scale;
        if (DecomposeView(view, right, up, back, scale)) {
            const double c = Dot(right, g_frame.rawRight) + Dot(up, g_frame.rawUp) + Dot(back, g_frame.rawBack);
            sameRotation = c > 3.0 - 1e-5;  // within ~0.15 degrees
        }
        if (!sameRotation && g_cutsLogged++ < 20)
            Log("camera: the sky's rotation differs from the camera's this frame; orientation recomputed");
    }
    if (!sameRotation && !OrientFrame(view))
        return false;
    return FinishFrame(view, correction);
}

}  // namespace camera
}  // namespace mxcam
