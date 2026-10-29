// bike.h
//   Finds the rider's own bike among the objects the game draws, so the custom camera can be
//   built from the bike itself instead of from the game's first-person camera.
//
//   The game's camera is mounted on the bike, so the camera's position *in the bike's local
//   space* never changes while riding, however the bike moves through the world. Every other
//   object fails that test: static scenery and other riders slide past, wheels spin, the bars
//   steer. The tracker follows every rigid object drawn near the camera from frame to frame
//   and locks onto the one that stays perfectly fixed relative to the camera while the camera
//   travels. No draw order, program number or model name is assumed.
#pragma once

namespace mxcam {
namespace bike {

// Called when the camera view matrix of a new frame is captured.
void BeginFrame(const double view[16]);

// Every rigid modelview upload after the camera in the same frame, as the game sent it
// (View * Model, before our correction). program is only used for logging.
void Observe(unsigned program, const float viewModel[16]);

// Called once per frame after drawing. Matches this frame's objects against the tracks.
void EndFrame();

// Forget every track (e.g. when the effect is switched off).
void Reset();

struct Pose {
    bool   tracked = false;
    // The bike's transform in the previous frame's camera space (View * Model as uploaded).
    // The camera is rigidly mounted, so this also holds in the current frame's camera space.
    double cameraSpace[16] = {};
    // The bike model's own axes for "forward" and "up", in model space (unit, axis-aligned).
    double forward[3] = { 0, 0, -1 };
    double up[3]      = { 0, 1, 0 };
};

const Pose& Current();

// Short status for the settings panel, e.g. "TRACKED" or "SEARCHING".
const char* StatusText();

}  // namespace bike
}  // namespace mxcam
