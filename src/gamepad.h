// gamepad.h
//   Reads a game controller's d-pad for looking around, which MX Simulator has no keys for.
//   Only Windows' own controller APIs are used (XInput, then the older joystick API for pads
//   XInput doesn't know); nothing in the game is read or changed, and the game still gets the
//   d-pad presses as usual.
#pragma once

namespace mxcam {
namespace gamepad {

struct DPad {
    bool left  = false;
    bool right = false;
    bool down  = false;
};

// The d-pad of the first connected controller, polled now. Cheap to call every frame: slots
// with nothing plugged in are only searched again once a second.
DPad Read();

// Short status for the settings panel: "XBOX PAD", "PAD" (another controller) or "NO PAD".
const char* StatusText();

}  // namespace gamepad
}  // namespace mxcam
