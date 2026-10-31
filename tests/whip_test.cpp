// whip_test.cpp
//   Drives the camera module (camera.cpp) through a whip: riding along +x at 15 m/s, up a face
//   into a 1.3 s jump; just after the lip the bike swings 70 deg sideways, leans 60 deg and dips
//   its nose, straightened before the
//   landing. The game camera is fixed to the bike's head, so it swings and leans with it.
//   Then a rut corner whose lean-in reads as free fall, a bump in a slight turn and a whoop
//   hopped while sliding, none of which whip lock may hold. Gravity is 3 g throughout, as MX
//   Simulator's airtime reads in the mod's units, and the whip is ridden again with the game
//   showing a double physics step now and then. Runs in real time (the camera times frames
//   with the clock), ~35 s.
//
//   Build: cl /EHsc /std:c++17 /W4 tests\whip_test.cpp src\camera.cpp src\bike.cpp src\config.cpp src\log.cpp user32.lib

#include <windows.h>
#include <cmath>
#include <cstdio>

#include "../src/bike.h"
#include "../src/camera.h"
#include "../src/config.h"

namespace mxcam {
bool IsDumping() { return false; }  // the frame dump lives in hooks.cpp, not linked here
}

using namespace mxcam;

namespace {

constexpr double kPi = 3.14159265358979;
constexpr double kSpeed = 15.0, kJumpStart = 1.6, kAirTime = 1.3, kEnd = 3.8, kGravity = 9.81;
constexpr double kAirG = 3.0 * kGravity;  // what the game's gravity reads as
constexpr double kFace = 0.3;                         // s on the jump face
constexpr double kWhipDelay = 0.1, kWhipTime = 1.05;  // thrown just after the lip, back before landing

int g_failures = 0;
bool g_verbose = false;  // -v: the frames through the jump
bool g_bikeLost = false;  // the tracker didn't have the bike at some point that counts
void Check(bool ok, const char* what)
{
    printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    g_failures += ok ? 0 : 1;
}

struct V { double x, y, z; };
V operator+(V a, V b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
V operator*(V a, double s) { return { a.x * s, a.y * s, a.z * s }; }
V Cross(V a, V b) { return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }
double Dot(V a, V b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
V Normalize(V a) { return a * (1.0 / std::sqrt(Dot(a, a))); }

// The view matrix of a camera at eye with the given yaw (0 = +x, + = toward +z), pitch and roll.
void View(V eye, double yaw, double pitch, double roll, double m[16])
{
    const V heading{ std::cos(yaw), 0, std::sin(yaw) }, worldUp{ 0, 1, 0 };
    const V forward = heading * std::cos(pitch) + worldUp * std::sin(pitch);
    const V levelRight = Normalize(Cross(heading, worldUp));
    const V levelUp = Cross(levelRight, forward);
    const V right = levelRight * std::cos(roll) + levelUp * std::sin(roll);
    const V up = Cross(right, forward);
    const V back = forward * -1.0;
    const V axes[3] = { right, up, back };
    for (int r = 0; r < 3; ++r) {
        m[0 * 4 + r] = axes[r].x;
        m[1 * 4 + r] = axes[r].y;
        m[2 * 4 + r] = axes[r].z;
        m[3 * 4 + r] = -Dot(axes[r], eye);
        m[r * 4 + 3] = 0;
    }
    m[15] = 1;
}

double Wrap(double degrees)
{
    while (degrees > 180) degrees -= 360;
    while (degrees < -180) degrees += 360;
    return degrees;
}

// One frame the way the hooks see it: the camera view, then the bike drawn at bike with the
// camera's orientation (the head is fixed to it), which the tracker locks onto.
const camera::Telemetry& Draw(V bike, V eye, double yaw, double pitch, double roll)
{
    double view[16], axes[16];
    View(eye, yaw, pitch, roll, view);
    View({ 0, 0, 0 }, yaw, pitch, roll, axes);
    double model[16] = {};  // the bike's axes as columns, at bike
    for (int c = 0; c < 3; ++c)
        for (int r = 0; r < 3; ++r)
            model[c * 4 + r] = axes[r * 4 + c];
    model[12] = bike.x; model[13] = bike.y; model[14] = bike.z; model[15] = 1;
    float viewModel[16];
    for (int c = 0; c < 4; ++c)
        for (int r = 0; r < 4; ++r) {
            double sum = 0;
            for (int k = 0; k < 4; ++k)
                sum += view[k * 4 + r] * model[c * 4 + k];
            viewModel[c * 4 + r] = float(sum);
        }
    camera::FrameTick();
    bike::BeginFrame(view);
    double correction[16];
    camera::Update(view, correction);
    bike::Observe(0, viewModel);
    bike::EndFrame(camera::GameTime(), camera::FrameInterval());
    return camera::LastTelemetry();
}

struct Result {
    double maxYaw = 0, maxRoll = 0, maxPitch = 0;  // biggest departure in the air
    double landedYaw = 0;                           // view vs bike heading at the end
    double airborneDrop = 0;                        // largest fall of the airborne estimate mid-air
};

// doubleSteps: like the game, show two physics steps on every 40th frame (one frame's worth of
// motion more than the frame's own time).
Result Ride(double whipLock, bool doubleSteps = false)
{
    Config& cfg = EditConfig();
    cfg.customCamera = false;
    cfg.whipLock = whipLock;
    camera::Reset();
    bike::Reset();

    Result result;
    double groundYaw = 0, groundPitch = 0, lastClock = 0, ahead = 0, lastAirborne = 0;
    int frame = 0;
    LARGE_INTEGER frequency, start, now;
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&start);
    for (;;) {
        QueryPerformanceCounter(&now);
        const double clock = double(now.QuadPart - start.QuadPart) / frequency.QuadPart;
        if (clock > kEnd)
            break;
        if (doubleSteps && ++frame % 40 == 0)
            ahead += clock - lastClock;
        lastClock = clock;
        const double t = clock + ahead;  // the game's time
        // Up a jump face (vertical speed building over kFace), then free fall; the whip is thrown
        // just after the lip and straightened before the landing.
        const double air = t - kJumpStart;
        const bool inAir = air > 0 && air < kAirTime;
        const double v0 = 0.5 * kAirG * kAirTime, faceStart = kJumpStart - kFace;
        double height = 0;
        if (t > faceStart && t <= kJumpStart)
            height = 0.5 * (v0 / kFace) * (t - faceStart) * (t - faceStart);
        else if (inAir)
            height = 0.5 * v0 * kFace + v0 * air - 0.5 * kAirG * air * air;
        else if (air >= kAirTime)
            height = 0.5 * v0 * kFace;  // landed on a raised table
        const double whip = air - kWhipDelay;
        const double phase = whip > 0 && whip < kWhipTime ? std::sin(kPi * whip / kWhipTime) : 0.0;
        const V bike{ kSpeed * t, height, 0 };
        const double yaw = 70.0 * kPi / 180 * phase, roll = -60.0 * kPi / 180 * phase, pitch = -10.0 * kPi / 180 * phase;
        // The head, 1 m above the bike along its up axis, swings with it.
        double m[16];
        View({ 0, 0, 0 }, yaw, pitch, roll, m);
        const V up{ m[1], m[5], m[9] };
        const camera::Telemetry& tel = Draw(bike, bike + up * 1.0, yaw, pitch, roll);
        if (t > 0.6 && t < kJumpStart - kFace) {  // level ground, before the face loads the neck
            groundYaw = tel.outYaw;
            groundPitch = tel.outPitch;
        }
        if (inAir) {
            if (!tel.bikeTracked && !g_bikeLost)
                printf("    (whip: bike not tracked at t=%.2f)\n", t);
            g_bikeLost |= !tel.bikeTracked;
            result.maxYaw = std::fmax(result.maxYaw, std::fabs(Wrap(tel.outYaw - groundYaw)));
            result.maxRoll = std::fmax(result.maxRoll, std::fabs(tel.outRoll));
            result.maxPitch = std::fmax(result.maxPitch, std::fabs(tel.outPitch - groundPitch));
            if (air > 0.6 && air < kAirTime - 0.05)
                result.airborneDrop = std::fmax(result.airborneDrop, lastAirborne - tel.airborne);
            lastAirborne = tel.airborne;
        }
        result.landedYaw = std::fabs(Wrap(tel.outYaw - tel.rawYaw));
        static double lastPrint = 0;
        if (g_verbose && whipLock > 0 && t > kJumpStart - 0.1 && t - lastPrint > 0.05) {
            lastPrint = t;
            printf("    t=%.2f view yaw %+6.1f bike yaw %+6.1f roll %+5.1f fall %.2fg airborne %.2f\n", t,
                   Wrap(tel.outYaw - groundYaw), Wrap(tel.rawYaw - groundYaw), tel.outRoll, tel.fallG, tel.airborne);
        }
        Sleep(7);
    }
    return result;
}

// 0 -> 1 over 0.3 s from x = 0.
double SmoothLean(double x)
{
    const double u = std::fmin(std::fmax(x / 0.3, 0.0), 1.0);
    return u * u * (3 - 2 * u);
}

// A rut: from 1.6 s (the tracker has the bike by then) the bike, laid over 80 deg, turns at 90 deg/s for a second. As it leans in,
// the eye drops like a falling body for 0.6 s (seen in a trace) - but it is turning, so it isn't
// in the air. Returns the furthest the view got from the bike's heading during the turn.
double Corner(double whipLock)
{
    Config& cfg = EditConfig();
    cfg.customCamera = false;
    cfg.whipLock = whipLock;
    camera::Reset();
    bike::Reset();

    constexpr double kTurnRate = 90.0 * kPi / 180, kLean = -80.0 * kPi / 180;
    double worst = 0, lastT = 0, yaw = 0, height = 0, vy = 0;
    V bike{ 0, 0, 0 };
    LARGE_INTEGER frequency, start, now;
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&start);
    for (;;) {
        QueryPerformanceCounter(&now);
        const double t = double(now.QuadPart - start.QuadPart) / frequency.QuadPart;
        if (t > 3.2)
            break;
        const double dt = t - lastT;
        lastT = t;
        const bool inTurn = t > 1.6 && t < 2.6;
        if (inTurn)
            yaw += kTurnRate * dt;
        vy = t > 1.6 && t < 2.2 ? vy - 2.0 * kGravity * dt : 0.0;
        height += vy * dt;
        bike = bike + V{ std::cos(yaw), 0, std::sin(yaw) } * (kSpeed * dt);
        bike.y = height;
        const double roll = kLean * SmoothLean(t - 1.3) * (1.0 - SmoothLean(t - 2.6));  // in by the turn, out after it
        double m[16];
        View({ 0, 0, 0 }, yaw, 0, roll, m);
        const V up{ m[1], m[5], m[9] };
        const camera::Telemetry& tel = Draw(bike, bike + up * 1.0, yaw, 0, roll);
        if (inTurn) {
            worst = std::fmax(worst, std::fabs(Wrap(tel.outYaw - tel.rawYaw)));
            if (!tel.bikeTracked && !g_bikeLost)
                printf("    (rut: bike not tracked at t=%.2f)\n", t);
            g_bikeLost |= !tel.bikeTracked;
        }
        Sleep(7);
    }
    return worst;
}

// A bump in a slight turn. A jump first lets the camera learn the game's gravity. Then, turning 10 deg/s leaned 15 deg, a bump drops the
// bike at 1.2 g for 0.1 s - free fall to the takeoff detector, but far shallower than airtime.
// Returns the furthest the view got from the game camera's heading around the bump.
double Bump(double whipLock)
{
    Config& cfg = EditConfig();
    cfg.customCamera = false;
    cfg.whipLock = whipLock;
    camera::Reset();
    bike::Reset();

    constexpr double kG = kAirG, kAir = 1.0, kLaunch = 1.6, kTurnStart = 3.2, kBump = 4.0, kEndBump = 5.0;
    double worst = 0, lastT = 0, yaw = 0, height = 0, vy = 0, dip = 0;
    V bike{ 0, 0, 0 };
    LARGE_INTEGER frequency, start, now;
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&start);
    for (;;) {
        QueryPerformanceCounter(&now);
        const double t = double(now.QuadPart - start.QuadPart) / frequency.QuadPart;
        if (t > kEndBump)
            break;
        const double dt = t - lastT;
        lastT = t;
        if (t > kLaunch && t < kLaunch + kAir) {
            const double air = t - kLaunch;
            height = 0.5 * kG * kAir * air - 0.5 * kG * air * air;
        } else {
            height = 0;
        }
        if (t > kTurnStart)
            yaw += 10.0 * kPi / 180 * dt;
        // The dip: down at 1.2 g for 0.1 s, then levelled out at 1.2 g, a little lower.
        if (t > kBump && t < kBump + 0.2) {
            vy += (t < kBump + 0.1 ? -1.2 : 1.2) * kGravity * dt;
            dip += vy * dt;
        }
        bike = bike + V{ std::cos(yaw), 0, std::sin(yaw) } * (kSpeed * dt);
        bike.y = height + dip;
        const double roll = t > kTurnStart ? -15.0 * kPi / 180 : 0.0;
        double m[16];
        View({ 0, 0, 0 }, yaw, 0, roll, m);
        const V up{ m[1], m[5], m[9] };
        const camera::Telemetry& tel = Draw(bike, bike + up * 1.0, yaw, 0, roll);
        if (t > kBump - 0.1 && t < kBump + 0.7) {
            worst = std::fmax(worst, std::fabs(Wrap(tel.outYaw - tel.rawYaw)));
            g_bikeLost |= !tel.bikeTracked;
        }
        static double lastPrint = 0;
        if (g_verbose && whipLock > 0 && ((t > 1.5 && t < 2.8) || (t > kBump - 0.1 && t < kBump + 0.7)) && t - lastPrint > 0.04) {
            lastPrint = t;
            printf("    t=%.2f view-game yaw %+5.1f fall %+.2fg airborne %.2f\n", t, Wrap(tel.outYaw - tel.rawYaw), tel.fallG, tel.airborne);
        }
        Sleep(7);
    }
    return worst;
}

// A whoop: riding along +x with the bike slid 10 deg off its path (nose toward +z), it hops for
// 0.4 s, the nose swinging back 7 deg in the air - real free fall, but no jump to whip.
// Returns the furthest the view got from the game camera's heading around the hop.
double Whoop(double whipLock)
{
    Config& cfg = EditConfig();
    cfg.customCamera = false;
    cfg.whipLock = whipLock;
    camera::Reset();
    bike::Reset();

    constexpr double kAir = 0.4, kLaunch = 1.6, kSlide = 10.0 * kPi / 180, kSwing = -7.0 * kPi / 180;
    double worst = 0;
    LARGE_INTEGER frequency, start, now;
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&start);
    for (;;) {
        QueryPerformanceCounter(&now);
        const double t = double(now.QuadPart - start.QuadPart) / frequency.QuadPart;
        if (t > kLaunch + 1.2)
            break;
        const double air = t - kLaunch;
        const bool inAir = air > 0 && air < kAir;
        const double height = inAir ? 0.5 * kAirG * kAir * air - 0.5 * kAirG * air * air : 0.0;
        const double yaw = kSlide + kSwing * SmoothLean(air * 0.3 / kAir);
        const V bike{ kSpeed * t, height, 0 };
        double m[16];
        View({ 0, 0, 0 }, yaw, 0, 0, m);
        const V up{ m[1], m[5], m[9] };
        const camera::Telemetry& tel = Draw(bike, bike + up * 1.0, yaw, 0, 0);
        if (air > -0.1 && air < kAir + 0.5) {
            worst = std::fmax(worst, std::fabs(Wrap(tel.outYaw - tel.rawYaw)));
            g_bikeLost |= !tel.bikeTracked;
        }
        static double lastPrint = 0;
        if (g_verbose && whipLock > 0 && air > -0.1 && air < kAir + 0.5 && t - lastPrint > 0.04) {
            lastPrint = t;
            printf("    t=%.2f view-game yaw %+5.1f fall %+.2fg airborne %.2f\n", t, Wrap(tel.outYaw - tel.rawYaw), tel.fallG, tel.airborne);
        }
        Sleep(7);
    }
    return worst;
}

}  // namespace

int main(int argc, char** argv)
{
    g_verbose = argc > 1 && argv[1][0] == '-' && argv[1][1] == 'v';
    printf("Whip: 70 deg swing, 60 deg lean, 10 deg nose down, 1.3 s in the air\n");
    const Result off = Ride(0.0);
    printf("  whip lock off:  view turned up to %.1f deg, rolled %.1f deg, pitched %.1f deg; %.1f deg off the bike at the end\n",
           off.maxYaw, off.maxRoll, off.maxPitch, off.landedYaw);
    const Result on = Ride(1.0);
    printf("  whip lock 100%%: view turned up to %.1f deg, rolled %.1f deg, pitched %.1f deg; %.1f deg off the bike at the end\n",
           on.maxYaw, on.maxRoll, on.maxPitch, on.landedYaw);
    Check(off.maxYaw > 30, "without whip lock the view swings with the bike (test is meaningful)");
    // A few degrees get through before free fall shows in the bike's own motion (~0.1 s after the lip).
    Check(on.maxYaw < 5, "with whip lock the view keeps pointing along the direction of travel");
    Check(on.maxRoll < 3, "with whip lock the horizon stays level");
    Check(on.maxPitch < 4, "with whip lock the view keeps its look-down angle");
    Check(on.landedYaw < 1, "after landing the view is back on the bike");

    printf("Whip again, the game showing two physics steps on every 40th frame\n");
    const Result stepped = Ride(1.0, true);
    printf("  view turned up to %.1f deg, pitched %.1f deg; airborne estimate fell by up to %.2f mid-air\n",
           stepped.maxYaw, stepped.maxPitch, stepped.airborneDrop);
    Check(stepped.airborneDrop < 0.01, "a double step doesn't read as a landing");
    Check(stepped.maxYaw < 5, "with double steps whip lock still holds the direction of travel");

    printf("Rut: laid over 80 deg, turning 90 deg/s, eye dropping like free fall for 0.6 s\n");
    const double cornerOff = Corner(0.0), cornerOn = Corner(1.0);
    printf("  view up to %.1f deg off the bike's heading without whip lock, %.1f deg with it\n", cornerOff, cornerOn);
    Check(cornerOn < cornerOff + 3, "whip lock doesn't hold the view while cornering");

    printf("Bump: turning 10 deg/s, a 0.1 s dip at 1.2 g (airtime reads 3 g)\n");
    const double bumpOff = Bump(0.0), bumpOn = Bump(1.0);
    printf("  view up to %.1f deg off the game camera's heading without whip lock, %.1f deg with it\n", bumpOff, bumpOn);
    Check(bumpOn < bumpOff + 0.5, "whip lock ignores a bump in a slight turn");

    printf("Whoop: a 0.4 s hop sliding 10 deg off the path, the nose swinging back 7 deg\n");
    const double whoopOff = Whoop(0.0), whoopOn = Whoop(1.0);
    printf("  view up to %.1f deg off the game camera's heading without whip lock, %.1f deg with it\n", whoopOff, whoopOn);
    Check(whoopOn < whoopOff + 0.5, "whip lock ignores a whoop");
    Check(!g_bikeLost, "the bike tracker had the bike throughout (the game's path was tested)");
    printf(g_failures ? "%d check(s) failed\n" : "All checks passed\n", g_failures);
    return g_failures ? 1 : 0;
}
