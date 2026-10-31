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
// gameTime: the game's clock for this frame's content (s, camera::GameTime()); frameDt: the
// smoothed frame interval.
void EndFrame(double gameTime, double frameDt);

// Forget every track (e.g. when the effect is switched off).
void Reset();

struct Pose {
    bool   tracked = false;
    bool   fresh   = false;  // the bike was matched in the latest frame (world values are current)
    // The bike's transform in the latest matched frame's camera space (View * Model as uploaded).
    double cameraSpace[16] = {};
    // The bike model's own axes for "forward" and "up", in model space (unit, axis-aligned).
    double forward[3] = { 0, 0, -1 };
    double up[3]      = { 0, 1, 0 };
    // The bike's forward / up in world space in the latest matched frame. Unlike cameraSpace
    // these don't move when the game's sprung camera wobbles against the bike.
    double worldForward[3] = { 0, 0, -1 };
    double worldUp[3]      = { 0, 1, 0 };
    // The bike model's origin in world space in the latest matched frame.
    double worldPosition[3] = {};
    // The bike's own vertical acceleration along WorldUp, in g (free fall: -1), and its velocity
    // in world space (units/s).
    bool   haveAccel      = false;
    double verticalAccelG = 0;
    double velocity[3]    = {};
};

const Pose& Current();

// Short status for the settings panel, e.g. "TRACKED" or "SEARCHING".
const char* StatusText();

}  // namespace bike
}  // namespace mxcam
