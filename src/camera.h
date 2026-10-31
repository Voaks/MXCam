// camera.h
//   The helmet-cam physics. Fed the game's raw view matrix once per frame, it returns an
//   eye-space correction matrix C such that  C * RawView == DynamicView.
//
//   Because C is applied in eye space, the same C is correct for every object drawn that
//   frame:  C * (RawView * Model) == DynamicView * Model.  That's what lets the hooks apply
//   it to every modelview load without knowing which object is which.
#pragma once

namespace mxcam {
namespace camera {

// Last frame's internals, for the frame dump and the trace.
struct Telemetry {
    double dtMs              = 0;  // the step the filters took (smoothed frame interval)
    double rawIntervalMs     = 0;  // the last measured swap-to-swap interval
    double lagDeg            = 0;  // smoothed view vs target
    double rawRollDeg        = 0;  // bike (or game camera) roll relative to the horizon
    double rollCorrectionDeg = 0;  // final view roll minus the game camera's roll
    double pitchExcessDeg    = 0;  // bike pitch away from its resting pitch (wheelie > 0)
    double viewPitchDeg      = 0;  // final view pitch (+ = up)
    double lookYawDeg        = 0;  // game camera vs bike heading (look left / right / back)
    double airYawDeg         = 0;  // air looseness: turn applied to the view in the air
    double airborne          = 0;  // 0..1
    bool   bikeTracked       = false;
    bool   bikeFresh         = false;  // the bike's own world pose was used (not carried via the camera)
    bool   customActive      = false;
    double customWeight      = 0;
    bool   cut               = false;  // the raw camera snapped this frame (look key, camera switch)
    bool   reset             = false;  // every filter started over this frame
    double verticalAccelG    = 0;
    double fallG             = 0;      // vertical acceleration used for airborne detection
    double neckOffset        = 0;  // spring displacement along WorldUp (negative = compressed)
    double verticalOffset    = 0;  // total eye height change (smoothing + neck)
    // Attitudes in degrees: yaw (heading about WorldUp), pitch, roll.
    double rawYaw = 0, rawPitch = 0, rawRoll = 0;     // game camera
    double bikeYaw = 0, bikePitch = 0, bikeRoll = 0;  // bike (0 when not tracked)
    double outYaw = 0, outPitch = 0, outRoll = 0;     // our view
    double rawPos[3] = {};                            // game camera position
    double eyeOffset[3] = {};                         // our eye minus the game camera's
};

// Forget all filter history; the next Update() starts from the raw camera again.
void Reset();

// Called once per presented frame (at the buffer swap). Measures the frame interval; all
// filters step by a smoothed version of it, so timing jitter can't make the view judder.
void FrameTick();

// Time the content of the frame being drawn belongs to (the previous swap), in seconds, and
// the smoothed frame interval.
double ContentTime();
double FrameInterval();

// The game's clock as of the last Update(), in seconds: content time, except that a frame on
// which the game showed more than one physics step counts for that many. Stamp motion samples
// with it; it drifts from ContentTime(), so don't compare the two.
double GameTime();

// view: the game's raw world->eye matrix (column-major). Returns false - leaving the frame
// untouched - if it isn't a rigid camera transform (e.g. the wrong load was picked).
// Advances every filter by one frame.
bool Update(const double view[16], double correction[16]);

// Works out, without advancing anything, the rotation-only correction this frame will get if
// `view` carries this frame's raw camera rotation (the sky's rotation-only view is drawn before
// the camera position is known). Returns false - out untouched - if view isn't a rotation.
bool PreviewRotation(const double view[16], double out[16]);

// Angle in degrees between view's rotation and the last raw camera rotation (0 before the
// first frame, -1 if view isn't a rotation). The caller uses it to tell the sky (within a few
// degrees) from other rotation-only draws before calling PreviewRotation().
double AngleToLastCameraDeg(const double view[16]);

// Rotation-only correction of the last Update().
bool RotationCorrection(double out[16]);

const Telemetry& LastTelemetry();

}  // namespace camera
}  // namespace mxcam
