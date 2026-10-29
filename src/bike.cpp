// bike.cpp
//   Per frame:
//     1. BeginFrame  - the camera's world position, for measuring how far the camera travels
//     2. Observe     - every rigid object drawn within BikeSearchRadius of the camera becomes a
//                      candidate, described relative to the camera: q = the camera's position in
//                      the object's own space, plus the camera's forward / up in that space
//     3. EndFrame    - each track takes the candidate closest to what it was last frame;
//                      unmatched candidates start new tracks, tracks unseen for a while are dropped.
//
//   The game's first-person camera rides on the bike but is sprung: in MX Simulator it moves
//   ~10 cm against the bike over bumps. So instead of demanding a perfectly fixed q, a track
//   qualifies as the bike when, over about a second of travel,
//     - q stays within a few tens of centimetres of its average (scenery slides away by metres),
//     - its orientation relative to the camera stays steady (wheels spin, the fork steers).
//   The steadiest qualifying track wins, and the lock is held through short gaps so the view
//   never flips back and forth between the custom and the game camera.

#include <windows.h>
#include <cmath>
#include <vector>

#include "bike.h"
#include "config.h"
#include "hooks.h"
#include "log.h"
#include "mathlib.h"
#include "motion.h"

namespace mxcam {
namespace bike {
namespace {

constexpr size_t kMaxCandidates = 64;
constexpr size_t kMaxTracks     = 64;
constexpr double kMaxMissTime   = 0.5;   // seconds a track (and the lock) survives without a match
constexpr double kMinAlive      = 1.0;   // seconds a track must persist before it can be locked
constexpr double kMinTravel     = 5.0;   // ...while the camera travels this far
constexpr double kMaxAngleSpreadDeg = 8.0;
constexpr int    kMaxLogLines   = 60;

struct Candidate {
    double   viewModel[16];
    Vec3     q;            // camera position in the object's space
    Vec3     cameraFwd;    // camera forward / up in the object's space
    Vec3     cameraUp;
    unsigned program;
    int      order;        // draw order within the frame
    bool     used;
};

struct Track {
    int      id;
    Vec3     lastQ, lastFwd, lastUp;   // last frame's match
    Vec3     meanQ, meanFwd, meanUp;   // ~1 s running averages
    double   qVar;                     // running mean of |q - meanQ|^2
    double   angleVar;                 // running mean of (angle to the mean orientation)^2, degrees^2
    double   alive;                    // seconds tracked
    double   travel;                   // camera travel while tracked
    double   missTime;                 // seconds since the last match
    double   viewModel[16];
    unsigned program;
    int      order;
};

std::vector<Candidate> g_candidates;
std::vector<Track>     g_tracks;
bool          g_frameOpen  = false;
bool          g_havePos    = false;
Vec3          g_cameraPos;
Vec3          g_prevCameraPos;
LARGE_INTEGER g_lastTick{};
int           g_nextId     = 1;
int           g_lockedId   = 0;
Pose          g_pose;
int           g_logLines   = 0;
double        g_viewInverse[16];   // this frame's camera -> world
// The locked bike's world motion, for its vertical acceleration.
bool          g_haveBikePos = false;
int           g_motionTrackId = 0;
Vec3          g_bikePos;
MotionFit     g_bikeFit;

// Only to notice long gaps (menus, loading); the filters step by the frame time they're given.
double SecondsSinceLastFrame()
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

double AngleDeg(Vec3 a, Vec3 b)
{
    return std::acos(Clamp(Dot(Normalize(a), Normalize(b)), -1.0, 1.0)) * kRadToDeg;
}

// Angle between two relative orientations, each given by the camera's forward / up.
double OrientationDiffDeg(Vec3 fwdA, Vec3 upA, Vec3 fwdB, Vec3 upB)
{
    return std::fmax(AngleDeg(fwdA, fwdB), AngleDeg(upA, upB));
}

double QSpread(const Track& t) { return std::sqrt(t.qVar); }
double AngleSpread(const Track& t) { return std::sqrt(t.angleVar); }

// Lower is steadier. 10 cm of bounce counts like 1 degree of wobble.
double Score(const Track& t) { return AngleSpread(t) + 10.0 * QSpread(t); }

bool Qualifies(const Track& t)
{
    return t.missTime == 0 && t.alive >= kMinAlive && t.travel >= kMinTravel &&
           QSpread(t) < 0.3 + 0.05 * Length(t.meanQ) && AngleSpread(t) < kMaxAngleSpreadDeg;
}

// The signed unit axis (+-X, +-Y, +-Z) closest to v, skipping axis `exclude`.
Vec3 SnapToAxis(Vec3 v, int exclude, int& axis)
{
    const double c[3] = { v.x, v.y, v.z };
    axis = -1;
    for (int i = 0; i < 3; ++i) {
        if (i != exclude && (axis < 0 || std::fabs(c[i]) > std::fabs(c[axis])))
            axis = i;
    }
    Vec3 out;
    (axis == 0 ? out.x : axis == 1 ? out.y : out.z) = c[axis] < 0 ? -1.0 : 1.0;
    return out;
}

const char* AxisName(Vec3 a)
{
    if (a.x > 0.5) return "+X";
    if (a.x < -0.5) return "-X";
    if (a.y > 0.5) return "+Y";
    if (a.y < -0.5) return "-Y";
    return a.z > 0 ? "+Z" : "-Z";
}

Track* FindTrack(int id)
{
    for (Track& t : g_tracks) {
        if (t.id == id)
            return &t;
    }
    return nullptr;
}

void Lock(const Track& t)
{
    g_lockedId = t.id;
    // The game's camera looks roughly along the bike, so the model axes nearest to the camera's
    // average forward / up are the bike's forward / up.
    int forwardAxis, upAxis;
    const Vec3 forward = SnapToAxis(t.meanFwd, -1, forwardAxis);
    const Vec3 up      = SnapToAxis(t.meanUp, forwardAxis, upAxis);
    g_pose.forward[0] = forward.x; g_pose.forward[1] = forward.y; g_pose.forward[2] = forward.z;
    g_pose.up[0] = up.x;           g_pose.up[1] = up.y;           g_pose.up[2] = up.z;
    if (g_logLines++ < kMaxLogLines)
        Log("bike: tracking an object of program %u (draw #%d): the camera sits at (%.2f, %.2f, %.2f) in its space, "
            "bounce %.0f mm, wobble %.1f deg; forward axis %s, up axis %s",
            t.program, t.order, t.meanQ.x, t.meanQ.y, t.meanQ.z, QSpread(t) * 1000.0, AngleSpread(t),
            AxisName(forward), AxisName(up));
}

void Unlock(const char* why)
{
    if (g_lockedId && g_logLines++ < kMaxLogLines)
        Log("bike: lost track (%s); the custom camera eases back to the game camera until it is found again", why);
    g_lockedId = 0;
    g_pose.tracked = false;
    g_pose.fresh = false;
    g_pose.haveAccel = false;
}

}  // namespace

void Reset()
{
    g_tracks.clear();
    g_candidates.clear();
    g_frameOpen = false;
    g_havePos = false;
    g_lockedId = 0;
    g_pose.tracked = false;
    g_pose.fresh = false;
    g_pose.haveAccel = false;
    g_haveBikePos = false;
}

void BeginFrame(const double view[16])
{
    if (!AffineInverse(view, g_viewInverse))
        return;
    g_cameraPos = { g_viewInverse[12], g_viewInverse[13], g_viewInverse[14] };
    g_candidates.clear();
    g_frameOpen = true;
}

void Observe(unsigned program, const float viewModel[16])
{
    if (!g_frameOpen || g_candidates.size() >= kMaxCandidates)
        return;
    const Vec3 t{ viewModel[12], viewModel[13], viewModel[14] };  // object origin in camera space
    if (Length(t) > GetConfig().bikeSearchRadius)
        return;

    Candidate c;
    for (int i = 0; i < 16; ++i)
        c.viewModel[i] = viewModel[i];
    // Rotation part R (camera <- object). Column i of R is m[i*4 .. i*4+2], so (R^T v)_i is a
    // dot product with that column.
    const auto ColumnDot = [&](int i, Vec3 v) {
        return c.viewModel[i * 4] * v.x + c.viewModel[i * 4 + 1] * v.y + c.viewModel[i * 4 + 2] * v.z;
    };
    c.q         = { -ColumnDot(0, t), -ColumnDot(1, t), -ColumnDot(2, t) };
    c.cameraFwd = { -c.viewModel[2], -c.viewModel[6], -c.viewModel[10] };
    c.cameraUp  = { c.viewModel[1], c.viewModel[5], c.viewModel[9] };
    c.program   = program;
    c.order     = static_cast<int>(g_candidates.size());
    c.used      = false;

    // The same object is often drawn twice (e.g. once more for shadows): keep one.
    for (const Candidate& other : g_candidates) {
        if (Length(other.q - c.q) < 1e-4 && Length(other.cameraFwd - c.cameraFwd) < 1e-4)
            return;
    }
    g_candidates.push_back(c);
}

void EndFrame(double contentTime, double frameDt)
{
    if (!g_frameOpen)
        return;
    g_frameOpen = false;

    const Config& cfg = GetConfig();
    if (SecondsSinceLastFrame() > 1.0) {
        // Paused or in a menu for a while: start over.
        Unlock("no camera for over a second");
        g_tracks.clear();
        g_havePos = false;
    }
    const double dt = Clamp(frameDt, 0.0, 0.1);

    double travelStep = g_havePos ? Length(g_cameraPos - g_prevCameraPos) : 0.0;
    if (travelStep > cfg.teleportDistance)
        travelStep = 0;  // a respawn is not travel
    g_prevCameraPos = g_cameraPos;
    g_havePos = true;

    // ---- Match: the oldest tracks pick first ----
    std::vector<Track*> order;
    order.reserve(g_tracks.size());
    for (Track& t : g_tracks)
        order.push_back(&t);
    for (size_t i = 1; i < order.size(); ++i) {
        for (size_t j = i; j > 0 && order[j]->alive > order[j - 1]->alive; --j) {
            Track* swap = order[j];
            order[j] = order[j - 1];
            order[j - 1] = swap;
        }
    }

    const double alpha = 1.0 - std::exp(-1.0 * dt);  // ~1 s averages
    for (Track* t : order) {
        Candidate* best = nullptr;
        double bestCost = 1e30;
        for (Candidate& c : g_candidates) {
            if (c.used)
                continue;
            const double dq    = Length(c.q - t->lastQ);
            const double angle = OrientationDiffDeg(c.cameraFwd, c.cameraUp, t->lastFwd, t->lastUp);
            if (dq > 0.4 + 0.05 * Length(t->lastQ) || angle > 25.0)
                continue;
            const double cost = dq + 0.02 * angle;
            if (cost < bestCost) {
                best = &c;
                bestCost = cost;
            }
        }
        if (!best) {
            t->missTime += dt;
            continue;
        }
        best->used = true;
        t->lastQ   = best->q;
        t->lastFwd = best->cameraFwd;
        t->lastUp  = best->cameraUp;
        t->meanQ   = t->meanQ + (best->q - t->meanQ) * alpha;
        t->meanFwd = Normalize(t->meanFwd + (best->cameraFwd - t->meanFwd) * alpha);
        t->meanUp  = Normalize(t->meanUp + (best->cameraUp - t->meanUp) * alpha);
        const Vec3 dq = best->q - t->meanQ;
        t->qVar += (Dot(dq, dq) - t->qVar) * alpha;
        const double angle = OrientationDiffDeg(best->cameraFwd, best->cameraUp, t->meanFwd, t->meanUp);
        t->angleVar += (angle * angle - t->angleVar) * alpha;
        t->alive   += dt;
        t->travel  += travelStep;
        t->missTime = 0;
        t->program  = best->program;
        t->order    = best->order;
        for (int i = 0; i < 16; ++i)
            t->viewModel[i] = best->viewModel[i];
    }

    // ---- Drop stale tracks, start new ones ----
    for (size_t i = 0; i < g_tracks.size();) {
        if (g_tracks[i].missTime > kMaxMissTime) {
            if (g_tracks[i].id == g_lockedId)
                Unlock("the bike was not seen for half a second");
            g_tracks.erase(g_tracks.begin() + static_cast<std::ptrdiff_t>(i));
        } else {
            ++i;
        }
    }
    for (const Candidate& c : g_candidates) {
        if (c.used || g_tracks.size() >= kMaxTracks)
            continue;
        Track t{};
        t.id       = g_nextId++;
        t.lastQ    = t.meanQ = c.q;
        t.lastFwd  = t.meanFwd = c.cameraFwd;
        t.lastUp   = t.meanUp = c.cameraUp;
        t.qVar     = 0.3 * 0.3;   // unproven until it has a history
        t.angleVar = 10.0 * 10.0;
        t.program  = c.program;
        t.order    = c.order;
        for (int i = 0; i < 16; ++i)
            t.viewModel[i] = c.viewModel[i];
        g_tracks.push_back(t);
    }

    // ---- Lock onto the steadiest qualified track, and hold on to it ----
    Track* locked = g_lockedId ? FindTrack(g_lockedId) : nullptr;
    Track* best = nullptr;
    for (Track& t : g_tracks) {
        if (Qualifies(t) && (!best || Score(t) < Score(*best)))
            best = &t;
    }
    if (!locked && best) {
        Lock(*best);
        locked = best;
    } else if (locked && best && best != locked && best->alive > 2.0 && Score(*best) < 0.5 * Score(*locked) - 0.5) {
        Lock(*best);  // a clearly steadier object (e.g. the frame rather than the steering fork)
        locked = best;
    }
    g_pose.fresh = false;
    if (locked) {
        // During a short gap the last match is kept: the bike barely moves against the camera.
        for (int i = 0; i < 16; ++i)
            g_pose.cameraSpace[i] = locked->viewModel[i];
        g_pose.tracked = true;
    }
    if (locked && locked->missTime == 0) {
        // World transform of the bike this frame: camera -> world times View * Model.
        double world[16];
        MatMul(g_viewInverse, locked->viewModel, world);
        const auto Axis = [&](const double a[3]) {
            return Normalize(Vec3{ world[0] * a[0] + world[4] * a[1] + world[8] * a[2],
                                   world[1] * a[0] + world[5] * a[1] + world[9] * a[2],
                                   world[2] * a[0] + world[6] * a[1] + world[10] * a[2] });
        };
        const Vec3 forward = Axis(g_pose.forward), up = Axis(g_pose.up);
        g_pose.worldForward[0] = forward.x; g_pose.worldForward[1] = forward.y; g_pose.worldForward[2] = forward.z;
        g_pose.worldUp[0] = up.x;           g_pose.worldUp[1] = up.y;           g_pose.worldUp[2] = up.z;
        g_pose.fresh = true;

        // Vertical acceleration of the bike itself (not of the sprung camera): a least-squares
        // fit over the last 0.15 s, which frame-timing jitter can't throw off.
        const Vec3 pos{ world[12], world[13], world[14] };
        if (locked->id != g_motionTrackId || !g_haveBikePos || Length(pos - g_bikePos) > cfg.teleportDistance) {
            g_motionTrackId = locked->id;
            g_bikeFit.Reset();
        }
        g_bikeFit.Add(contentTime, pos);
        Vec3 velocity, accel;
        const Vec3 worldUp{ cfg.worldUp[0], cfg.worldUp[1], cfg.worldUp[2] };
        g_pose.haveAccel = g_bikeFit.Estimate(0.15, velocity, accel);
        g_pose.verticalAccelG = g_pose.haveAccel ? Dot(accel, worldUp) / kGravity : 0.0;
        g_bikePos = pos;
        g_haveBikePos = true;
    } else if (!locked) {
        g_haveBikePos = false;
        g_pose.haveAccel = false;
    }

    if (IsDumping()) {
        Log("  bike tracker: %zu candidates within %.1f units, %zu tracks, %s", g_candidates.size(),
            cfg.bikeSearchRadius, g_tracks.size(), g_pose.tracked ? "LOCKED" : "searching");
        // (`order` points into g_tracks from before the erase / push_back above: not usable here.)
        int shown = 0;
        for (const Track& t : g_tracks) {
            if (shown++ >= 16)
                break;
            Log("    track %d%s: program %u draw #%d, camera at (%.2f, %.2f, %.2f), bounce %.0f mm, wobble %.1f deg, "
                "alive %.1f s, travel %.1f%s", t.id, t.id == g_lockedId ? " [BIKE]" : "", t.program, t.order,
                t.meanQ.x, t.meanQ.y, t.meanQ.z, QSpread(t) * 1000.0, AngleSpread(t), t.alive, t.travel,
                t.missTime > 0 ? " (missing)" : "");
        }
    }
}

const Pose& Current() { return g_pose; }

const char* StatusText()
{
    if (g_pose.tracked)
        return "BIKE FOUND";
    return g_tracks.empty() ? "NO BIKE IN VIEW" : "LOOKING FOR THE BIKE";
}

}  // namespace bike
}  // namespace mxcam
