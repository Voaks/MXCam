// bike.cpp
//   Per frame:
//     1. BeginFrame  - the camera's world position, for measuring how far the camera travels
//     2. Observe     - every rigid object drawn within BikeSearchRadius of the camera becomes a
//                      candidate; its key is q = the camera's position in the object's own space
//     3. EndFrame    - each track takes the candidate whose q is closest to its own; unmatched
//                      candidates start new tracks, tracks unseen for a while are dropped.
//                      A track that kept a steady q for a second while the camera travelled
//                      several metres is rigidly attached to the camera: the rider's bike.

#include <windows.h>
#include <cmath>
#include <vector>

#include "bike.h"
#include "config.h"
#include "hooks.h"
#include "log.h"
#include "mathlib.h"

namespace mxcam {
namespace bike {
namespace {

constexpr size_t kMaxCandidates = 64;
constexpr size_t kMaxTracks     = 64;
constexpr int    kMaxMisses     = 30;    // frames a track survives without a match
constexpr int    kMaxStaleLock  = 3;     // frames the lock survives without a match
constexpr double kMinAlive      = 1.0;   // seconds a track must persist before it can be locked
constexpr double kMinTravel     = 5.0;   // ...while the camera travels this far
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
    Vec3     q;
    double   errorSq;      // low-passed squared deviation of q (rigidly attached: ~0)
    double   alive;        // seconds matched without a long gap
    double   travel;       // camera travel while this track was alive
    int      misses;
    double   viewModel[16];
    Vec3     cameraFwd;    // low-passed, used to work out the model's forward / up axes
    Vec3     cameraUp;
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

double Rms(const Track& t) { return std::sqrt(t.errorSq); }

bool Qualifies(const Track& t)
{
    return t.misses == 0 && t.alive >= kMinAlive && t.travel >= kMinTravel &&
           Rms(t) < 0.05 + 0.02 * Length(t.q);
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
    const Vec3 forward = SnapToAxis(t.cameraFwd, -1, forwardAxis);
    const Vec3 up      = SnapToAxis(t.cameraUp, forwardAxis, upAxis);
    g_pose.forward[0] = forward.x; g_pose.forward[1] = forward.y; g_pose.forward[2] = forward.z;
    g_pose.up[0] = up.x;           g_pose.up[1] = up.y;           g_pose.up[2] = up.z;
    if (g_logLines++ < kMaxLogLines)
        Log("bike: tracking an object of program %u (draw #%d): the camera sits at (%.3f, %.3f, %.3f) in its space, "
            "jitter %.1f mm; forward axis %s, up axis %s",
            t.program, t.order, t.q.x, t.q.y, t.q.z, Rms(t) * 1000.0, AxisName(forward), AxisName(up));
}

void Unlock(const char* why)
{
    if (g_lockedId && g_logLines++ < kMaxLogLines)
        Log("bike: lost track (%s); custom camera falls back to the game camera until it is found again", why);
    g_lockedId = 0;
    g_pose.tracked = false;
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
}

void BeginFrame(const double view[16])
{
    double inverse[16];
    if (!AffineInverse(view, inverse))
        return;
    g_cameraPos = { inverse[12], inverse[13], inverse[14] };
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

void EndFrame()
{
    if (!g_frameOpen)
        return;
    g_frameOpen = false;

    const Config& cfg = GetConfig();
    double dt = SecondsSinceLastFrame();
    if (dt > 1.0) {
        // Paused or in a menu for a while: start over.
        Unlock("no camera for over a second");
        g_tracks.clear();
        g_havePos = false;
    }
    dt = Clamp(dt, 0.0, 0.1);

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

    const double qAlpha   = 1.0 - std::exp(-2.0 * dt);
    const double errAlpha = 1.0 - std::exp(-3.0 * dt);
    for (Track* t : order) {
        Candidate* best = nullptr;
        double bestDistance = 0.10 + 0.04 * Length(t->q);  // match tolerance
        for (Candidate& c : g_candidates) {
            const double d = Length(c.q - t->q);
            if (!c.used && d < bestDistance) {
                best = &c;
                bestDistance = d;
            }
        }
        if (!best) {
            ++t->misses;
            continue;
        }
        best->used = true;
        t->errorSq += (bestDistance * bestDistance - t->errorSq) * errAlpha;
        t->q         = t->q + (best->q - t->q) * qAlpha;
        t->cameraFwd = t->cameraFwd + (best->cameraFwd - t->cameraFwd) * qAlpha;
        t->cameraUp  = t->cameraUp + (best->cameraUp - t->cameraUp) * qAlpha;
        t->alive  += dt;
        t->travel += travelStep;
        t->misses  = 0;
        t->program = best->program;
        t->order   = best->order;
        for (int i = 0; i < 16; ++i)
            t->viewModel[i] = best->viewModel[i];
    }

    // ---- Drop stale tracks, start new ones ----
    for (size_t i = 0; i < g_tracks.size();) {
        if (g_tracks[i].misses > kMaxMisses) {
            if (g_tracks[i].id == g_lockedId)
                Unlock("the bike stopped matching");
            g_tracks.erase(g_tracks.begin() + static_cast<std::ptrdiff_t>(i));
        } else {
            ++i;
        }
    }
    for (const Candidate& c : g_candidates) {
        if (c.used || g_tracks.size() >= kMaxTracks)
            continue;
        Track t{};
        t.id        = g_nextId++;
        t.q         = c.q;
        t.errorSq   = 0.05 * 0.05;  // unproven until it has a history
        t.cameraFwd = c.cameraFwd;
        t.cameraUp  = c.cameraUp;
        t.program   = c.program;
        t.order     = c.order;
        for (int i = 0; i < 16; ++i)
            t.viewModel[i] = c.viewModel[i];
        g_tracks.push_back(t);
    }

    // ---- Lock onto the steadiest qualified track ----
    Track* locked = g_lockedId ? FindTrack(g_lockedId) : nullptr;
    if (locked && locked->misses > kMaxStaleLock) {
        Unlock("the bike was not drawn for several frames");
        locked = nullptr;
    }
    Track* best = nullptr;
    for (Track& t : g_tracks) {
        if (Qualifies(t) && (!best || Rms(t) < Rms(*best) - 1e-6 || (Rms(t) <= Rms(*best) + 1e-6 && t.order < best->order)))
            best = &t;
    }
    if (!locked && best) {
        Lock(*best);
        locked = best;
    } else if (locked && best && best != locked && best->alive > 2.0 && Rms(*best) < 0.5 * Rms(*locked) - 0.002) {
        Lock(*best);  // a clearly steadier object (e.g. the frame rather than the steering bars)
        locked = best;
    }
    if (locked && locked->misses == 0) {
        for (int i = 0; i < 16; ++i)
            g_pose.cameraSpace[i] = locked->viewModel[i];
        g_pose.tracked = true;
    }

    if (IsDumping()) {
        Log("  bike tracker: %zu candidates within %.1f units, %zu tracks, %s", g_candidates.size(),
            cfg.bikeSearchRadius, g_tracks.size(), g_pose.tracked ? "LOCKED" : "searching");
        // (`order` points into g_tracks from before the erase / push_back above: not usable here.)
        int shown = 0;
        for (const Track& t : g_tracks) {
            if (shown++ >= 16)
                break;
            Log("    track %d%s: program %u draw #%d, camera at (%.3f, %.3f, %.3f), jitter %.1f mm, "
                "alive %.1f s, travel %.1f, misses %d", t.id, t.id == g_lockedId ? " [BIKE]" : "",
                t.program, t.order, t.q.x, t.q.y, t.q.z, Rms(t) * 1000.0, t.alive, t.travel, t.misses);
        }
    }
}

const Pose& Current() { return g_pose; }

const char* StatusText()
{
    if (g_pose.tracked)
        return "BIKE FOUND";
    return g_tracks.empty() ? "NO BIKE IN VIEW" : "LOOKING FOR THE BIKE - RIDE A FEW METRES";
}

}  // namespace bike
}  // namespace mxcam
