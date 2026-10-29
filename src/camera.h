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

// Last frame's internals, for the frame dump.
struct Telemetry {
    double dtMs              = 0;
    double lagDeg            = 0;  // smoothed view vs target
    double rawRollDeg        = 0;  // bike (or game camera) roll relative to the horizon
    double rollCorrectionDeg = 0;  // final view roll minus the game camera's roll
    double pitchExcessDeg    = 0;  // bike pitch away from its resting pitch (wheelie > 0)
    double viewPitchDeg      = 0;  // final view pitch (+ = up)
    double lookYawDeg        = 0;  // game camera vs bike heading (look left / right / back)
    double whipDeg           = 0;  // whip-centering turn applied to the view
    double airborne          = 0;  // 0..1
    bool   bikeTracked       = false;
    bool   customActive      = false;
    double verticalAccelG    = 0;
    double neckOffset        = 0;  // spring displacement along WorldUp (negative = compressed)
    double verticalOffset    = 0;  // total eye height change (smoothing + neck)
};

// Forget all filter history; the next Update() starts from the raw camera again.
void Reset();

// view: the game's raw world->eye matrix (column-major). Returns false - leaving the frame
// untouched - if it isn't a rigid camera transform (e.g. the wrong load was picked).
// Runs OrientFrame() unless it already ran this frame for the same rotation, then the position
// half of the pipeline.
bool Update(const double view[16], double correction[16]);

// First half only: works out the view's orientation from any matrix that carries this frame's
// raw camera rotation (e.g. the sky's rotation-only view, drawn before the camera position).
bool OrientFrame(const double view[16]);

// Rotation-only correction (no translation) for the orientation worked out this frame. Things
// drawn infinitely far away, like the sky, take this instead of the full correction.
bool RotationCorrection(double out[16]);

// The frame ended without a camera: forget a pending OrientFrame().
void AbandonFrame();

const Telemetry& LastTelemetry();

}  // namespace camera
}  // namespace mxcam
