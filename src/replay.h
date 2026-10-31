// replay.h
//   Replays. The game films a replay from a fixed direction: the view rotation never changes
//   (facing -Z, 26.6 degrees down) while the camera follows the rider from about 5 m up and 10 m
//   back, so the rider swivels around in front of it. That steady rotation, at that angle, is how
//   a replay is recognised - no first-person camera holds it to a hundredth of a degree for a
//   third of a second.
//
//   While a replay is on, the helmet cam is skipped. The replay is shown the way the game films
//   it, or (Config::replayChaseCam) from a camera locked behind the rider, like the game's
//   third-person view when riding. The rider is found through its skinned mesh: 'u_bone_t'
//   carries the bones' world-space positions.
#pragma once

#include <cstddef>

namespace mxcam {
namespace replay {

enum class Mode {
    Off,    // not a replay: the helmet cam takes the frame
    Game,   // a replay, shown untouched
    Chase,  // a replay, filmed from behind the rider
};

// Every 'u_bone_t' upload: count world-space positions (vec3) of one rider's bones.
void ObserveBones(const float* positions, int count);

// The frame's raw camera view (the terrain's view matrix), before anything is drawn with it.
// With Chase, correction and rotationOnly receive the eye-space correction to the chase view,
// like camera::Update() and camera::RotationCorrection().
Mode Update(const double view[16], double contentTime, double correction[16], double rotationOnly[16]);

// Once the frame is drawn: follows the rider seen in it and prepares the next frame's view.
// captured: the frame had a camera (no camera means the replay is over or the mod is off).
void EndFrame(double contentTime, bool captured);

// As of the last Update().
Mode Current();

// For the sky, drawn before Update(): degrees between view's rotation and the replay camera's
// (-1 if view isn't a rotation), and the rotation-only correction this frame will get.
double AngleToCameraDeg(const double view[16]);
bool PreviewRotation(const double view[16], double out[16]);

// Short status for the settings panel, e.g. "REPLAY - 3RD PERSON CAMERA".
const char* StatusText();

// One line for the trace (no newline).
void Describe(char* text, size_t size);

}  // namespace replay
}  // namespace mxcam
