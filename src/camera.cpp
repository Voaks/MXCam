// camera.cpp
//   Per-frame pipeline. Every filter steps by the smoothed frame interval (FrameTick), so a
//   frame that happens to be measured a millisecond long or short doesn't make the view judder.
//   Orient - everything about where the view points. It only needs the raw camera rotation,
//            so it can also be *previewed* from the sky, which is drawn before the camera
//            position is known. A preview works on a copy of the filter state and changes
//            nothing; only Update() (the terrain, once per frame) advances the filters. So an
//            odd rotation-only draw can't step or reset them, and the sky always gets exactly
//            the rotation the rest of the frame gets.
//     1. decompose       - raw view matrix -> head orientation (quaternion)
//     2. cut detection   - camera switches / look-key snaps skip the smoothing for one frame
//     3. attitude        - heading / pitch / roll of the game camera and, when the tracker has
//                          found it, of the rider's bike (bike.cpp)
//     4. target view     - game camera:   the game's view with pitch lock and horizon lock
//                          custom camera: a view built from the bike alone (the game's 1st
//                                         person angle and gimbal play no part), with the
//                                         locks, our own look-down angle and the game's
//                                         look-left / right / back passed through;
//                          both: air looseness (the nose's swing filtered in the air),
//                                head position offsets
//     5. smoothing       - exponential slerp toward the target, with a lag limit; head nod
//   FinishFrame - where the eye is.
//     6. motion          - velocity and acceleration of the eye (least-squares fit, motion.h);
//                          airborne detection
//     7. neck physics    - vertical low-pass + spring-damper driven by vertical acceleration
//     8. rebuild         - new view matrix, and correction C = NewView * inverse(RawView)

#include <windows.h>
#include <cmath>
#include <cstdarg>
#include <cstdio>

#include "bike.h"
#include "camera.h"
#include "config.h"
#include "log.h"
#include "mathlib.h"
#include "motion.h"

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

// Landing (see FinishFrame): an impact, vertical acceleration this many g above resting...
constexpr double kImpactG  = 0.8;
// ...or the eye alone out of free fall for longer than a whip swings the head (s).
constexpr double kSwingTime = 0.5;
// In the air the direction of travel can't change. Turning faster than this (deg/s) is a
// corner, whatever the vertical acceleration says: a bike laid over in a rut drops the eye
// and bike as it leans in, which reads as free fall (85 deg of lean, 100 deg/s, seen in a
// trace). Real jumps in the same trace stayed under 26 deg/s, whips included.
constexpr double kAirTurnRateDeg = 40.0;
// Whip lock is stricter, since it takes the view off the bike: the bike's path must be nearly
// straight (a downhill corner read as 0.25 s of free fall while turning 63 deg/s, but only
// crossed kAirTurnRateDeg after the lock had snatched the view; jumps stayed under 6 deg/s).
// Measured from the eye, a whip's head swing alone can read 26 deg/s, so the air limit then.
constexpr double kWhipTurnRateDeg = 20.0;
// ...and free fall must read at least this fraction of what it reads in the air. MX Simulator's
// airtime reads about -3 g in this mod's units; a bump in a slight turn read -0.9 g for 65 ms
// and snatched the view, while every real takeoff passed -1.5 g within 10 ms.
constexpr double kWhipFallFraction = 0.5;
// ...and only for a real jump. Whoops and rollers leave the ground too, for 0.2-0.6 s, reading
// as deep as airtime; locking the view there swings it from the bike's nose to its path (they
// differ by the slide through the section) and back on landing. The size of a jump is set at
// the lip: the airtime its launch speed carries on flat ground. In traces whoops launched for
// at most 0.5 s of that, real jumps for 0.8 s or more. A jump that launches flat and drops away
// (a step-down) still gets the lock once it has really been in the air this long.
constexpr double kWhipAirtimeMin  = 0.6, kWhipAirtimeFull  = 0.8;  // s, launch airtime
constexpr double kWhipFallTimeMin = 0.8, kWhipFallTimeFull = 1.2;  // s, time in the air

// Whip lock takes the direction of travel as it was over this long before free fall is detected:
// the path's own direction, averaged, not a filtered one. A filter lags a turn, and a double
// taken while turning out of the landing of the one before (20 deg in 0.3 s) was held 18 deg
// off its path, along the direction from before the turn, and swung back on landing.
constexpr double kTravelHoldBack = 0.25;  // s
constexpr int    kTravelSamples  = 128;   // frames of history: covers it up to ~500 fps
// In the air the path seen from above is a straight line, so the held direction then settles
// onto the line from where the fall began, once it has run far enough that the head's sway
// (the eye's path) can't turn it.
constexpr double kPathRunMin = 3.0, kPathRunFull = 10.0;  // units
constexpr double kPathResponse = 4.0;                     // 1/s
struct TravelSample {
    Vec3   direction;  // the direction of travel on that frame
    double rise = 0;  // upward speed (units/s)
    double dt   = 0;
};

struct State {
    bool   valid        = false;  // false: the next Update() seeds every filter from the raw camera
    bool   havePos      = false;  // false: FinishFrame() seeds the position filters
    Quat   smoothRot;             // low-passed view orientation
    Quat   prevRawRot;
    Vec3   prevPos;
    Vec3   velocity;              // world-space velocity of the eye (units/s)
    Vec3   accel;                 // world-space acceleration of the eye (units/s^2)
    Vec3   airNose;               // slowly filtered nose heading in the air: bike, game camera
    Vec3   airNoseCamera;
    Vec3   travel;                // horizontal direction of travel, filtered (whip lock)
    TravelSample travelHistory[kTravelSamples];  // on the ground since the last fall
    int    travelNext   = 0;
    int    travelCount  = 0;
    bool   wasFalling   = false;
    Vec3   takeoffPos;            // where this fall began: the bike's origin, or the eye (whip lock)
    bool   takeoffFromBike = false;
    double launchRise   = 0;      // upward speed at the lip of this fall (units/s, whip lock)
    double outOfFallTime = 0;    // in the air: how long the eye has read out of free fall
    Vec3   motionHeading;         // horizontal direction of the eye's velocity, last frame
    double turnRate     = 0;      // how fast that direction turns (rad/s, filtered)
    bool   turnFromBike = false;  // turnRate is measured from the bike's path (else the eye's)
    bool   deepFall     = false;  // this fall has read as deep as real airtime (whip lock)
    double whip         = 0;      // 0..1 how far whip lock has taken over (eased)
    double airborne     = 0;      // 0 on the ground .. 1 in the air (eased)
    bool   inAir        = false;
    double fallTime     = 0;      // seconds of continuous free fall
    double lookSide     = 0;      // 0..1 how much the rider is looking sideways / back
    double lookBack     = 0;
    double headTurn     = 0;      // d-pad look: the head's turn about WorldUp (rad, + = right, eased)
    double smoothHeight = 0;      // low-passed eye height along WorldUp
    double neckOffset   = 0;      // spring displacement along WorldUp (negative = compressed)
    double neckVelocity = 0;
};

// Everything the orientation half carries from frame to frame. A preview runs on a copy.
struct Model {
    State         state;
    RestEstimator cameraPitchRest;  // game camera pitch on flat ground (includes the game's angle)
    RestEstimator bikePitchRest;    // bike model pitch on flat ground
    RestEstimator gameAngleRest;    // game camera pitch relative to the bike: the game's 1st-person angle
    Vec3          lastHeading{ 0, 0, -1 };
    double        customWeight = 0; // 0 = game camera .. 1 = custom camera, eased on switches
    double        airG = -3.0;      // what free fall reads well into a jump (g), learned; MX Simulator's to start
    bike::Pose    lastPose;         // last pose the tracker reported, used while easing out
    bool          haveLastPose = false;
};

// One frame's orientation, carried from Orient() to FinishFrame().
struct Frame {
    bool   valid    = false;
    double dt       = 0;
    bool   reset    = false;
    Vec3   rawRight, rawUp, rawBack;    // raw camera axes the orientation was computed from
    Quat   rot;                         // final view orientation
    Vec3   offsetFwd, offsetUp;         // head offset axes and amounts
    double forwardOffset = 0, heightOffset = 0;
    Telemetry telemetry;
};

// Rate-limited log for events that can repeat every frame: the first few are logged, then a
// count every ten seconds, so a problem that keeps happening stays visible in the log.
struct EventLog {
    int    logged     = 0;
    int    suppressed = 0;
    double since      = 0;
};

Model     g_model;
Frame     g_frame;
Telemetry g_telemetry;
MotionFit g_eyeFit;
EventLog  g_cutLog, g_teleportLog;

// D-pad look (SetLook). The head turns at Config::lookTurnSpeed.
Look g_look = Look::Ahead;

// Frame timing (FrameTick).
double g_contentTime     = 0;      // the swap that started the frame being drawn, seconds
double g_frameInterval   = 1.0 / 60.0;
bool   g_haveInterval    = false;
double g_lastCommitTime  = -1;     // content time of the last Update()
double g_lastCommitWallTime = 0;   // when it ran, to notice pauses (loading, alt-tab)
double g_rawIntervalMs   = 0;

// The game's own clock (FinishFrame), which the motion fits are stamped with. The game steps its
// physics in fixed increments and now and then shows two (or three) steps in one frame - one
// frame in 40 at 125 fps in a trace - which the swap times can't see. Stamped with the swap
// time, such a frame read as a 6 g jolt and then a false landing, worst falling fast at the end
// of a big jump: whip lock let go and grabbed again, and the neck bounced.
double g_gameTime        = 0;
double g_lastMotionTime  = -1;     // content time of the last FinishFrame()
constexpr double kMinStepTravel = 0.03;  // units per frame below which steps aren't counted

double NowSeconds()
{
    LARGE_INTEGER now, frequency;
    QueryPerformanceCounter(&now);
    QueryPerformanceFrequency(&frequency);
    return double(now.QuadPart) / double(frequency.QuadPart);
}

void LogEvent(EventLog& e, const char* format, ...)
{
    if (e.logged >= 10) {
        ++e.suppressed;
        return;
    }
    ++e.logged;
    char text[512];
    va_list args;
    va_start(args, format);
    vsnprintf(text, sizeof text, format, args);
    va_end(args);
    Log("%s", text);
}

void SummarizeEvents(EventLog& e, const char* what)
{
    if (e.logged < 10 || g_contentTime - e.since < 10.0)
        return;
    if (e.suppressed)
        Log("camera: %d more %s in the last %.0f s", e.suppressed, what, g_contentTime - e.since);
    e.suppressed = 0;
    e.since = g_contentTime;
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
void StepNeck(State& s, double verticalAccel, double dt, const Config& cfg)
{
    const double omega = 2.0 * kPi * cfg.springHz;
    if (omega <= 0) {
        s.neckOffset = s.neckVelocity = 0;
        return;
    }
    const double stiffness = omega * omega;
    const double damping   = 2.0 * cfg.dampingRatio * omega;
    const double target    = -cfg.compressionPerG * verticalAccel / kGravity;

    // 1 ms semi-implicit Euler substeps keep a stiff spring stable at any frame rate.
    const int    steps = static_cast<int>(std::ceil(dt * 1000.0));
    const double h     = dt / steps;
    double& x = s.neckOffset;
    double& v = s.neckVelocity;
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

Attitude Measure(Vec3 forward, Vec3 up, Vec3 worldUp, Vec3 fallbackHeading)
{
    Attitude a;
    const double sinPitch = Clamp(Dot(forward, worldUp), -1.0, 1.0);
    a.pitch = std::asin(sinPitch);
    const Vec3 horizontal = forward - worldUp * sinPitch;
    a.level = Length(horizontal);
    a.heading = a.level > 0.05 ? horizontal / a.level : fallbackHeading;
    const Vec3 levelRight = Normalize(Cross(a.heading, worldUp));  // a level head's right axis
    const Vec3 levelUp    = Cross(levelRight, forward);
    const Vec3 right      = Cross(forward, up);
    a.roll = std::atan2(Dot(right, levelUp), Dot(right, levelRight));
    return a;
}

// Heading as a compass angle in degrees, for the trace.
double YawDeg(Vec3 heading, Vec3 worldUp)
{
    const Vec3 a = std::fabs(worldUp.z) < 0.9 ? Vec3{ 0, 0, -1 } : Vec3{ 0, 1, 0 };
    const Vec3 north = Normalize(a - worldUp * Dot(a, worldUp));
    const Vec3 east  = Cross(north, worldUp);
    return std::atan2(Dot(heading, east), Dot(heading, north)) * kRadToDeg;
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

// The step the next Update() will take: whole frames since the last one, at the smoothed
// frame interval. gap: too long since the last Update() to carry the filters over.
double StepDt(bool& gap)
{
    gap = g_lastCommitTime < 0 || NowSeconds() - g_lastCommitWallTime > 0.25;
    if (gap)
        return 0;
    const double elapsed = g_contentTime - g_lastCommitTime;
    const double frames = std::fmax(1.0, std::floor(elapsed / g_frameInterval + 0.5));
    return Clamp(frames * g_frameInterval, 0.0, 0.05);
}

bool Orient(Model& m, const double view[16], double dt, bool commit, Frame& f)
{
    const Config& cfg = GetConfig();
    const Vec3 worldUp{ cfg.worldUp[0], cfg.worldUp[1], cfg.worldUp[2] };
    State& s = m.state;
    Telemetry& tel = f.telemetry;

    // ---- 1. Decompose the raw view matrix ---------------------------------------------
    Vec3 right, up, back;
    double scale;
    if (!DecomposeView(view, right, up, back, scale))
        return false;
    const Quat rawRot = QuatFromBasis(right, up, back);
    const Vec3 rawFwd = -back;

    // ---- 2. Camera cut detection (rotation; position jumps are handled in FinishFrame) ----
    // Compared raw to raw: the corrected view may sit tens of degrees from the raw one. A cut
    // (the look keys snap the camera 90 / 180 degrees) only skips the smoothing for a frame;
    // the motion filters, airborne state and neck carry on.
    bool cut = false;
    if (s.valid) {
        const double jumpDeg = AngleBetween(s.prevRawRot, rawRot) * kRadToDeg;
        if (jumpDeg > cfg.snapAngleDeg) {
            cut = true;
            if (commit)
                LogEvent(g_cutLog, "camera: cut detected (%.1f deg turn), view snapped", jumpDeg);
        }
    }
    const bool reset = !s.valid;
    if (reset) {
        s = State{};
        s.valid = true;
        dt = 0;
    }
    s.prevRawRot = rawRot;
    const bool snap = reset || cut;

    // D-pad look: the head turns toward where it's asked to look. Back is reached over the
    // shoulder it's already turned toward (the right one from straight ahead).
    {
        const double side = cfg.lookSideDeg * kDegToRad;
        const double aim = g_look == Look::Left  ? -side
                         : g_look == Look::Right ? side
                         : g_look == Look::Back  ? (s.headTurn < 0 ? -kPi : kPi)
                         : 0.0;
        s.headTurn += (aim - s.headTurn) * (snap ? 1.0 : LowPassAlpha(cfg.lookTurnSpeed, dt));
    }

    const bool onGround = s.airborne < 0.3;

    // ---- 3. Attitude of the game camera and of the bike -----------------------------------
    const Attitude cameraAtt = Measure(rawFwd, up, worldUp, m.lastHeading);
    if (onGround && cameraAtt.level > 0.5)
        m.cameraPitchRest.Add(cameraAtt.pitch * kRadToDeg, dt);

    // The custom camera eases in when the bike is found and out when it is lost; while easing
    // out it keeps using the last known bike pose (the bike barely moves against the camera).
    const bike::Pose& pose = bike::Current();
    if (pose.tracked) {
        m.lastPose = pose;
        m.haveLastPose = true;
    }
    m.customWeight += (((cfg.customCamera && pose.tracked) ? 1.0 : 0.0) - m.customWeight) * LowPassAlpha(3.0, dt);
    if (!m.haveLastPose)
        m.customWeight = 0;

    bool     haveBike = false;
    Vec3     bikeFwd, bikeUp;
    Attitude bikeAtt;
    double   lookYaw = 0;
    if (pose.tracked && pose.fresh) {
        // The bike's own world-space axes from the last frame it was drawn. They come straight
        // from the bike, so the game camera's spring wobble against the bike can't leak in.
        bikeFwd = Normalize(Vec3{ pose.worldForward[0], pose.worldForward[1], pose.worldForward[2] });
        const Vec3 u{ pose.worldUp[0], pose.worldUp[1], pose.worldUp[2] };
        bikeUp = Normalize(u - bikeFwd * Dot(u, bikeFwd));
        haveBike = Length(Cross(bikeFwd, bikeUp)) > 0.5;
    } else if (m.haveLastPose && (pose.tracked || m.customWeight > 1e-3)) {
        // Bike not drawn last frame (a short gap, or easing out after losing it): its last known
        // pose relative to the camera, carried into world space through this frame's camera axes.
        const double* mv = m.lastPose.cameraSpace;
        const auto ToCamera = [mv](const double a[3]) {
            return Vec3{ mv[0] * a[0] + mv[4] * a[1] + mv[8] * a[2],
                         mv[1] * a[0] + mv[5] * a[1] + mv[9] * a[2],
                         mv[2] * a[0] + mv[6] * a[1] + mv[10] * a[2] };
        };
        const auto ToWorld = [&](Vec3 v) { return right * v.x + up * v.y + back * v.z; };
        bikeFwd = Normalize(ToWorld(ToCamera(m.lastPose.forward)));
        const Vec3 u = ToWorld(ToCamera(m.lastPose.up));
        bikeUp = Normalize(u - bikeFwd * Dot(u, bikeFwd));
        haveBike = Length(Cross(bikeFwd, bikeUp)) > 0.5;
    }
    if (haveBike) {
        bikeAtt = Measure(bikeFwd, bikeUp, worldUp, m.lastHeading);
        if (onGround && bikeAtt.level > 0.5 && pose.tracked)
            m.bikePitchRest.Add(bikeAtt.pitch * kRadToDeg, dt);

        // Where the game camera points relative to the bike: its look-down angle, and the
        // look-left / right / back keys.
        const Vec3 bikeRight = Cross(bikeFwd, bikeUp);
        const double x = Dot(rawFwd, bikeRight), y = Dot(rawFwd, bikeUp), z = Dot(rawFwd, bikeFwd);
        lookYaw = std::atan2(x, z);
        if (std::fabs(lookYaw) < 20.0 * kDegToRad && pose.tracked)
            m.gameAngleRest.Add(std::atan2(y, std::sqrt(x * x + z * z)) * kRadToDeg, dt);
    } else {
        m.customWeight = 0;
    }
    const double absYawDeg = std::fabs(lookYaw) * kRadToDeg;

    // Pitch lock works on the pitch *away from rest*, so riding on flat ground looks the same
    // at any setting and only wheelies, stoppies and jump faces are evened out.
    const double cameraPitchExcess = cameraAtt.pitch - m.cameraPitchRest.Degrees() * kDegToRad;
    const double bikePitchExcess   = haveBike ? bikeAtt.pitch - m.bikePitchRest.Degrees() * kDegToRad : 0.0;

    // ---- 4. Target view -------------------------------------------------------------------
    // Air looseness: in the air the view follows the nose's left / right swing only through a
    // slow filter, so most of a scrub or whip passes under the camera and the view keeps
    // pointing forward. Nothing is locked: the filter always heads for the nose, starts at the
    // first moment of free fall, and catches up smoothly after landing.
    // The view trails the nose by at most maxSwing (soft limit). A rider steering through a
    // jump out of a corner turns the bike one way and keeps it there (80 deg in 0.7 s seen in a
    // trace); unlimited, the view fell 30 deg behind and swung round on landing.
    const double airRate  = 30.0 * std::pow(1.5 / 30.0, cfg.airLooseness);  // 1/s: 30 (tight) .. 1.5 (loose)
    const double maxSwing = 15.0 * kDegToRad * cfg.airLooseness;
    const auto AirYaw = [&](Vec3 nose, Vec3& loose) {
        const bool falling = s.fallTime > 0 || s.inAir;
        if (cfg.airLooseness <= 0 || Length(loose) < 0.5 || (!falling && s.airborne < 0.01)) {
            loose = nose;
            return 0.0;
        }
        loose = Normalize(loose + (nose - loose) * LowPassAlpha(falling ? airRate : 10.0, dt));
        double w = std::atan2(Dot(nose, Normalize(Cross(loose, worldUp))), Dot(nose, loose));
        if (std::fabs(w) > 2.0 * maxSwing) {  // no wind-up past the soft limit
            w = w > 0 ? 2.0 * maxSwing : -2.0 * maxSwing;
            loose = Turn(nose, -w, worldUp);
        }
        return -maxSwing * std::tanh(w / maxSwing) * s.airborne;
    };

    // Game camera: the game's view with the locks applied. (It is fixed to the bike, so its
    // own pitch away from rest is the bike's.) A look-left / right / back isn't a swing.
    double airYaw = AirYaw(cameraAtt.heading, s.airNoseCamera) * (1.0 - SmoothStep(50.0, 75.0, absYawDeg));
    const double gamePitch = cameraAtt.pitch - cfg.pitchLock * cameraPitchExcess;
    const double gameRoll  = LockedRoll(cameraAtt, cfg.horizonLock, cfg);
    Quat target = Build(Turn(cameraAtt.heading, airYaw, worldUp), gamePitch, gameRoll, worldUp);

    // Custom camera: built from the bike only.
    if (m.customWeight > 1e-3) {
        const double angleDown = cfg.keepGameAngle ? -m.gameAngleRest.Degrees() : cfg.cameraAngleDeg;
        const double pitch = (1.0 - cfg.pitchLock) * bikePitchExcess - angleDown * kDegToRad;
        const double roll  = LockedRoll(bikeAtt, cfg.horizonLock, cfg);
        // The game's look keys turn the camera 90 / 180 degrees; smaller offsets (the game
        // camera's own wobble, whip or gimbal) are ignored.
        const double look = lookYaw * SmoothStep(50.0, 75.0, absYawDeg);
        const double customAirYaw = AirYaw(bikeAtt.heading, s.airNose);
        const Quat custom = Build(Turn(bikeAtt.heading, customAirYaw, worldUp), pitch, roll, worldUp, look);
        target = Slerp(target, custom, m.customWeight);
        airYaw += (customAirYaw - airYaw) * m.customWeight;
    }

    // Whip lock: in the air the view holds the direction of travel, level, at the resting
    // look-down angle, whatever the bike does under it. In the air that direction can't change,
    // so it is held from takeoff - as it was over kTravelHoldBack before free fall was detected,
    // as the head is already swinging sideways with the bike by then - and settles onto the
    // path actually flown. On the ground it follows the velocity (or the view, when slow),
    // filtered. All of it from the bike when the tracker has it: the eye swings with the head.
    {
        const bool fromBike = pose.tracked && pose.haveAccel;
        const Vec3 velocity = fromBike ? Vec3{ pose.velocity[0], pose.velocity[1], pose.velocity[2] } : s.velocity;
        const Vec3 horizontal = velocity - worldUp * Dot(velocity, worldUp);
        const double speed = Length(horizontal);
        const Vec3 viewFwd = Rotate(target, { 0, 0, -1 });
        const Vec3 viewHorizontal = viewFwd - worldUp * Dot(viewFwd, worldUp);
        const Vec3 direction = speed > 3.0 ? horizontal / speed
                             : Length(viewHorizontal) > 0.05 ? Normalize(viewHorizontal) : cameraAtt.heading;
        const bool falling = s.fallTime > 0 || s.inAir;
        const double rise = Dot(velocity, worldUp);
        const Vec3 bikePos{ pose.worldPosition[0], pose.worldPosition[1], pose.worldPosition[2] };
        if (snap || Length(s.travel) < 0.5) {
            s.travel = direction;
            s.travelCount = 0;
        }
        if (!falling) {
            s.travel = Normalize(s.travel + (direction - s.travel) * LowPassAlpha(5.0, dt));
            s.travelHistory[s.travelNext] = { direction, rise, dt };
            s.travelNext = (s.travelNext + 1) % kTravelSamples;
            s.travelCount = s.travelCount < kTravelSamples ? s.travelCount + 1 : kTravelSamples;
        } else if (!s.wasFalling) {
            // Free fall is detected a little after the lip, so the launch speed is the fastest
            // rise over the same stretch the direction is taken from.
            s.launchRise = rise;
            Vec3 sum{};
            double age = 0;
            for (int i = 1; i <= s.travelCount && age < kTravelHoldBack; ++i) {
                const TravelSample& sample = s.travelHistory[(s.travelNext - i + kTravelSamples) % kTravelSamples];
                sum = sum + sample.direction * sample.dt;
                s.launchRise = std::fmax(s.launchRise, sample.rise);
                age += sample.dt;
            }
            if (Length(sum) > 1e-9)
                s.travel = Normalize(sum);
            // The history only ever holds the ground since the last fall: a fall read again just
            // after landing must not reach back to before the jump.
            s.travelCount = 0;
            s.takeoffFromBike = fromBike && pose.fresh;
            s.takeoffPos = s.takeoffFromBike ? bikePos : s.prevPos;
        } else if (!s.takeoffFromBike || pose.fresh) {
            const Vec3 run = (s.takeoffFromBike ? bikePos : s.prevPos) - s.takeoffPos;
            const Vec3 path = run - worldUp * Dot(run, worldUp);
            const double length = Length(path);
            if (length > kPathRunMin)
                s.travel = Normalize(s.travel + (path / length - s.travel) *
                                     (LowPassAlpha(kPathResponse, dt) * SmoothStep(kPathRunMin, kPathRunFull, length)));
        }
        s.wasFalling = falling;
        // Engages within ~0.1 s of the first moment of deep free fall (the airborne estimate takes
        // too long for a whip thrown at the lip), only while the bike's path runs straight, as far
        // as the jump is big enough for it, and lets go over ~0.3 s after landing.
        const double maxTurn = (s.turnFromBike ? kWhipTurnRateDeg : kAirTurnRateDeg) * kDegToRad;
        const bool engage = falling && s.deepFall && s.turnRate < maxTurn;
        const double launchAirtime = 2.0 * std::fmax(s.launchRise, 0.0) / (std::fabs(m.airG) * kGravity);
        const double jumpSize = std::fmax(SmoothStep(kWhipAirtimeMin, kWhipAirtimeFull, launchAirtime),
                                          SmoothStep(kWhipFallTimeMin, kWhipFallTimeFull, s.fallTime));
        s.whip += ((engage ? jumpSize : 0.0) - s.whip) * LowPassAlpha(engage ? 25.0 : 6.0, dt);
    }
    const double whipWeight = cfg.whipLock * s.whip;
    if (whipWeight > 1e-3) {
        const double cameraRest = m.cameraPitchRest.Degrees();
        const double customRest = -(cfg.keepGameAngle ? -m.gameAngleRest.Degrees() : cfg.cameraAngleDeg);
        const double restPitch = (cameraRest + (customRest - cameraRest) * m.customWeight) * kDegToRad;
        target = Slerp(target, Build(s.travel, restPitch, 0.0, worldUp), whipWeight);
    }
    m.lastHeading = cameraAtt.heading;

    // Head position: along the bike (custom) or the game camera's own axes.
    const double lookAlpha = LowPassAlpha(10.0, dt);
    const double headDeg = std::fmax(absYawDeg, std::fabs(s.headTurn) * kRadToDeg);
    const double sideTarget = haveBike ? SmoothStep(50.0, 75.0, headDeg) * (1.0 - SmoothStep(110.0, 150.0, headDeg)) : 0.0;
    const double backTarget = haveBike ? SmoothStep(110.0, 150.0, headDeg) : 0.0;
    s.lookSide += (sideTarget - s.lookSide) * (snap ? 1.0 : lookAlpha);
    s.lookBack += (backTarget - s.lookBack) * (snap ? 1.0 : lookAlpha);
    const double side = s.lookSide * m.customWeight, rear = s.lookBack * m.customWeight;
    f.forwardOffset = cfg.headForward + side * cfg.lookSideForward + rear * cfg.lookBackForward;
    f.heightOffset  = cfg.headHeight + side * cfg.lookSideHeight + rear * cfg.lookBackHeight;
    f.offsetFwd = rawFwd;
    f.offsetUp  = up;
    if (haveBike) {
        f.offsetFwd = f.offsetFwd + (bikeFwd - f.offsetFwd) * m.customWeight;
        f.offsetUp  = f.offsetUp + (bikeUp - f.offsetUp) * m.customWeight;
    }

    // ---- 5. Inertial smoothing (pitch / yaw / roll) ---------------------------------------
    // Exponential slerp toward the target, filtering out high-frequency spikes. MaxLagDeg caps
    // how far behind the view can fall during sustained fast motion.
    Quat& smooth = s.smoothRot;
    if (snap)
        smooth = target;
    smooth = Slerp(smooth, target, LowPassAlpha(cfg.rotationResponse, dt));
    const double lag    = AngleBetween(smooth, target);
    const double maxLag = cfg.maxLagDeg * kDegToRad;
    if (lag > maxLag)
        smooth = Slerp(target, smooth, maxLag / lag);
    Quat rot = smooth;

    // The d-pad's head turn, about WorldUp so the horizon stays where the rest left it. After
    // the smoothing, so the head turns at its own pace.
    if (s.headTurn != 0)
        rot = AxisAngle(worldUp, -s.headTurn) * rot;

    // Neck compression also nods the head forward a little (rotation about the camera's right
    // axis; a negative angle pitches down). Uses last frame's neck, so the orientation is
    // complete here - the sky, drawn before the camera position is known, gets all of it.
    if (cfg.nodDegPerMeter != 0) {
        const Vec3 pitchAxis = Rotate(rot, { 1, 0, 0 });
        rot = AxisAngle(pitchAxis, s.neckOffset * cfg.nodDegPerMeter * kDegToRad) * rot;
    }

    f.rot      = Normalize(rot);
    f.dt       = dt;
    f.reset    = reset;
    f.rawRight = right;
    f.rawUp    = up;
    f.rawBack  = back;
    f.valid    = true;

    const Vec3 B = Rotate(f.rot, { 0, 0, 1 });
    const Attitude finalAtt = Measure(-B, Rotate(f.rot, { 0, 1, 0 }), worldUp, cameraAtt.heading);
    tel.dtMs              = dt * 1000.0;
    tel.lagDeg            = AngleBetween(s.smoothRot, target) * kRadToDeg;
    tel.rawRollDeg        = (haveBike ? bikeAtt.roll : cameraAtt.roll) * kRadToDeg;
    tel.rollCorrectionDeg = (finalAtt.roll - cameraAtt.roll) * kRadToDeg;
    tel.pitchExcessDeg    = (m.customWeight > 0.5 ? bikePitchExcess : cameraPitchExcess) * kRadToDeg;
    tel.viewPitchDeg      = finalAtt.pitch * kRadToDeg;
    tel.lookYawDeg        = (lookYaw + s.headTurn) * kRadToDeg;
    tel.airYawDeg         = airYaw * kRadToDeg;
    tel.bikeTracked       = pose.tracked;
    tel.bikeFresh         = pose.tracked && pose.fresh;
    tel.customActive      = m.customWeight > 0.5;
    tel.customWeight      = m.customWeight;
    tel.cut               = cut;
    tel.reset             = reset;
    tel.rawYaw    = YawDeg(cameraAtt.heading, worldUp);
    tel.rawPitch  = cameraAtt.pitch * kRadToDeg;
    tel.rawRoll   = cameraAtt.roll * kRadToDeg;
    tel.bikeYaw   = haveBike ? YawDeg(bikeAtt.heading, worldUp) : 0.0;
    tel.bikePitch = haveBike ? bikeAtt.pitch * kRadToDeg : 0.0;
    tel.bikeRoll  = haveBike ? bikeAtt.roll * kRadToDeg : 0.0;
    tel.outYaw    = YawDeg(finalAtt.heading, worldUp);
    tel.outPitch  = finalAtt.pitch * kRadToDeg;
    tel.outRoll   = finalAtt.roll * kRadToDeg;
    return true;
}

void RotationCorrectionOf(const Frame& f, double out[16])
{
    // C = NewRotation * RawRotation^T; element (r, c) = dot(new axis r, raw axis c).
    const Vec3 newAxes[3] = { Rotate(f.rot, { 1, 0, 0 }), Rotate(f.rot, { 0, 1, 0 }), Rotate(f.rot, { 0, 0, 1 }) };
    const Vec3 rawAxes[3] = { f.rawRight, f.rawUp, f.rawBack };
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r)
            out[c * 4 + r] = (r < 3 && c < 3) ? Dot(newAxes[r], rawAxes[c]) : (r == c ? 1.0 : 0.0);
    }
}

bool FinishFrame(const double view[16], double correction[16])
{
    const Config& cfg = GetConfig();
    const Vec3 worldUp{ cfg.worldUp[0], cfg.worldUp[1], cfg.worldUp[2] };
    State& s = g_model.state;
    Telemetry& tel = g_frame.telemetry;

    Vec3 right, up, back;
    double scale;
    if (!DecomposeView(view, right, up, back, scale))
        return false;
    const Vec3 t{ view[12], view[13], view[14] };
    const Vec3 rawPos = -(right * t.x + up * t.y + back * t.z) / scale;  // eye = -R^T t / s
    double dt = g_frame.dt;

    // A position jump (respawn) restarts the position filters only.
    if (s.havePos && Length(rawPos - s.prevPos) > cfg.teleportDistance) {
        LogEvent(g_teleportLog, "camera: cut detected (%.2f units), resetting the position filters", Length(rawPos - s.prevPos));
        s.havePos = false;
    }
    if (!s.havePos) {
        s.havePos      = true;
        s.velocity = s.accel = Vec3{};
        s.neckOffset = s.neckVelocity = 0;
        s.inAir = false;
        s.fallTime = 0;
        s.motionHeading = Vec3{};
        s.turnRate = 0;
        s.prevPos      = rawPos;
        s.smoothHeight = Dot(rawPos, worldUp);
        g_eyeFit.Reset();
        dt = 0;
    }

    // ---- 6. Motion: velocity, acceleration, airborne ------------------------------------
    // How many physics steps the game moved on this frame: at speed the camera covers a whole
    // multiple of an ordinary frame's distance. Only counted on an ordinary frame interval, so a
    // hitch or a pause just advances the clock by the time that passed.
    const double wallDt = g_lastMotionTime >= 0 ? std::fmax(g_contentTime - g_lastMotionTime, 0.0) : 0.0;
    g_lastMotionTime = g_contentTime;
    int steps = 1;
    const double expectedTravel = Length(s.velocity) * wallDt;
    if (wallDt < 1.5 * g_frameInterval && expectedTravel > kMinStepTravel) {
        const double ratio = Length(rawPos - s.prevPos) / expectedTravel;
        const int n = int(std::floor(ratio + 0.5));
        if (n >= 2 && n <= 3 && std::fabs(ratio - n) < 0.25)
            steps = n;
    }
    g_gameTime += wallDt * steps;

    // A least-squares fit over the last ~0.1 s (AccelFilterHz), stamped with the game's clock:
    // frame-to-frame timing noise averages out instead of being differentiated twice.
    g_eyeFit.Add(g_gameTime, rawPos);
    const double window = Clamp(cfg.accelFilterHz > 0 ? 1.2 / cfg.accelFilterHz : 0.1, 0.04, 0.3);
    Vec3 velocity, accel;
    if (g_eyeFit.Estimate(window, velocity, accel)) {
        const double maxAccel = 30.0 * kGravity;  // reject glitches (e.g. a missed cut)
        if (Length(accel) > maxAccel)
            accel = Normalize(accel) * maxAccel;
        s.velocity = velocity;
        s.accel = accel;
    }
    const double verticalAccel = Dot(s.accel, worldUp);

    // How fast the direction of travel turns (only measurable when moving). From the bike when
    // the tracker has it: the eye is on the rider's head, which a whip swings from side to side
    // (up to 90 deg/s of apparent turning in a test), while the bike's own path holds straight.
    const bike::Pose& pose = bike::Current();
    const bool bikeAccel = pose.tracked && pose.haveAccel;
    {
        const Vec3 travel = bikeAccel ? Vec3{ pose.velocity[0], pose.velocity[1], pose.velocity[2] } : s.velocity;
        const Vec3 horizontal = travel - worldUp * Dot(travel, worldUp);
        const double speed = Length(horizontal);
        double rate = 0;
        if (speed > 3.0) {
            const Vec3 heading = horizontal / speed;
            if (Length(s.motionHeading) > 0.5 && dt > 1e-5)
                rate = std::acos(Clamp(Dot(heading, s.motionHeading), -1.0, 1.0)) / dt;
            s.motionHeading = heading;
        } else {
            s.motionHeading = Vec3{};
        }
        s.turnRate += (rate - s.turnRate) * LowPassAlpha(10.0, dt);
        s.turnFromBike = bikeAccel;
    }
    const bool turning = s.turnRate > kAirTurnRateDeg * kDegToRad;

    // Airborne: in the air everything is in free fall (-1 g); on the ground it hovers around 0 g.
    // Takeoff: both the eye and (when the tracker has it) the bike must be in free fall - either
    // alone can fake it for a moment: the sprung game camera bouncing, or the bike flicking from
    // one lean to the other through a corner - for a quarter second (a real jump lasts far longer).
    // Landing: the eye alone can't be trusted. It is on the rider's head, which swings through
    // an arc as the bike is laid over in a whip or scrub, and can read near 0 g for a few tenths
    // of a second in mid-air. So it takes the bike back out of free fall, a clear impact, or
    // (untracked) the eye out of free fall for longer than a swing lasts.
    // Either way, a direction of travel that is turning means the wheels are on the ground.
    double eyeFallG = verticalAccel / kGravity;
    {
        Vec3 unusedVelocity, slowAccel;
        if (g_eyeFit.Estimate(0.15, unusedVelocity, slowAccel))
            eyeFallG = Dot(slowAccel, worldUp) / kGravity;
    }
    const double fallG = bikeAccel ? std::fmax(eyeFallG, pose.verticalAccelG) : eyeFallG;
    if (!s.inAir) {
        if (fallG < -0.7 && !turning) {
            s.fallTime += dt;
            if (s.fallTime > 0.25)
                s.inAir = true;
        } else if (fallG > -0.4 || turning) {
            s.fallTime = 0;
        }
        s.outOfFallTime = 0;
    } else {
        s.outOfFallTime = eyeFallG > -0.4 ? s.outOfFallTime + dt : 0.0;
        const bool impact = eyeFallG > kImpactG || (bikeAccel && pose.verticalAccelG > kImpactG);
        const bool landed = impact || turning ||
                            (bikeAccel ? pose.verticalAccelG > -0.4 : s.outOfFallTime > kSwingTime);
        if (landed) {
            s.inAir = false;
            s.fallTime = 0;
        } else {
            s.fallTime += dt;
        }
    }
    // How deep free fall reads well into a jump: the game's gravity in this mod's units.
    if (s.inAir && s.fallTime > 0.4)
        g_model.airG = Clamp(g_model.airG + (fallG - g_model.airG) * LowPassAlpha(2.0, dt), -6.0, -0.8);
    if (s.fallTime <= 0 && !s.inAir)
        s.deepFall = false;
    else if (fallG < kWhipFallFraction * g_model.airG)
        s.deepFall = true;
    // Eases in over a few tenths of a second, and out twice as fast on landing.
    s.airborne += ((s.inAir ? 1.0 : 0.0) - s.airborne) * LowPassAlpha(s.inAir ? 4.0 : 8.0, dt);

    // ---- 7. Vertical damping & G-force neck compression ---------------------------------
    const double rawHeight = Dot(rawPos, worldUp);
    if (dt > 1e-5) {
        // The neck is loaded by the bike's acceleration when the tracker has it: the game's head
        // animation (bobbing, swinging through a scrub) isn't a G-force and would only shake it.
        StepNeck(s, pose.tracked && pose.haveAccel ? pose.verticalAccelG * kGravity : verticalAccel, dt, cfg);
        // Low-pass the eye height itself to iron out head-bob jitter from the animation.
        s.smoothHeight += (rawHeight - s.smoothHeight) * LowPassAlpha(cfg.verticalResponse, dt);
    }
    s.smoothHeight = Clamp(s.smoothHeight, rawHeight - cfg.maxVerticalLag, rawHeight + cfg.maxVerticalLag);
    s.prevPos = rawPos;

    const double verticalOffset = (s.smoothHeight - rawHeight) + s.neckOffset;
    const Vec3 headOffset = g_frame.offsetFwd * g_frame.forwardOffset + g_frame.offsetUp * g_frame.heightOffset;
    const Vec3 eye = rawPos + headOffset + worldUp * verticalOffset;

    // ---- 8. Rebuild the view matrix and derive the eye-space correction -----------------
    const Vec3 R = Rotate(g_frame.rot, { 1, 0, 0 });
    const Vec3 U = Rotate(g_frame.rot, { 0, 1, 0 });
    const Vec3 B = Rotate(g_frame.rot, { 0, 0, 1 });
    const double sc = scale;
    const double newView[16] = {
        sc * R.x, sc * U.x, sc * B.x, 0,
        sc * R.y, sc * U.y, sc * B.y, 0,
        sc * R.z, sc * U.z, sc * B.z, 0,
        -sc * Dot(R, eye), -sc * Dot(U, eye), -sc * Dot(B, eye), 1,
    };
    double rawInverse[16];
    if (!AffineInverse(view, rawInverse))
        return false;
    MatMul(newView, rawInverse, correction);

    tel.airborne       = s.airborne;
    tel.verticalAccelG = verticalAccel / kGravity;
    tel.fallG          = fallG;
    tel.neckOffset     = s.neckOffset;
    tel.verticalOffset = verticalOffset;
    tel.rawPos[0] = rawPos.x;  tel.rawPos[1] = rawPos.y;  tel.rawPos[2] = rawPos.z;
    const Vec3 offset = eye - rawPos;
    tel.eyeOffset[0] = offset.x;  tel.eyeOffset[1] = offset.y;  tel.eyeOffset[2] = offset.z;
    return true;
}

}  // namespace

void Reset() { g_model.state.valid = false; }

const Telemetry& LastTelemetry() { return g_telemetry; }

void FrameTick()
{
    const double now = NowSeconds();
    if (g_contentTime > 0) {
        const double interval = now - g_contentTime;
        g_rawIntervalMs = interval * 1000.0;
        // Pauses (loading, alt-tab) don't count toward the typical frame interval.
        if (interval > 0 && interval < 0.25) {
            g_frameInterval = g_haveInterval ? g_frameInterval + (interval - g_frameInterval) * 0.2 : interval;
            g_haveInterval = true;
        }
    }
    g_contentTime = now;
    SummarizeEvents(g_cutLog, "camera cuts");
    SummarizeEvents(g_teleportLog, "position cuts");
}

void SetLook(Look look) { g_look = look; }

double ContentTime() { return g_contentTime; }
double GameTime() { return g_gameTime; }
double FrameInterval() { return g_frameInterval; }

bool PreviewRotation(const double view[16], double out[16])
{
    bool gap;
    const double dt = StepDt(gap);
    Model copy = g_model;
    if (gap)
        copy.state.valid = false;
    Frame f;
    if (!Orient(copy, view, dt, false, f))
        return false;
    RotationCorrectionOf(f, out);
    return true;
}

double AngleToLastCameraDeg(const double view[16])
{
    Vec3 right, up, back;
    double scale;
    if (!DecomposeView(view, right, up, back, scale))
        return -1;
    if (!g_model.state.valid)
        return 0;
    return AngleBetween(g_model.state.prevRawRot, QuatFromBasis(right, up, back)) * kRadToDeg;
}

bool RotationCorrection(double out[16])
{
    if (!g_frame.valid)
        return false;
    RotationCorrectionOf(g_frame, out);
    return true;
}

bool Update(const double view[16], double correction[16])
{
    bool gap;
    const double dt = StepDt(gap);
    if (gap)
        g_model.state.valid = false;  // paused, alt-tabbed or loading: resync rather than integrate
    Frame f;
    if (!Orient(g_model, view, dt, true, f))
        return false;
    g_frame = f;
    g_lastCommitTime = g_contentTime;
    g_lastCommitWallTime = NowSeconds();
    if (!FinishFrame(view, correction))
        return false;
    g_frame.telemetry.rawIntervalMs = g_rawIntervalMs;
    g_telemetry = g_frame.telemetry;
    return true;
}

}  // namespace camera
}  // namespace mxcam
