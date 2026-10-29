// overlay.h
//   In-game settings panel (Config::menuKey, E by default).
#pragma once

#include <windows.h>

namespace mxcam {

// Polls the menu key and, while the panel is open, handles its input and draws it into the
// current back buffer. Called by the swap hook right before the real buffer swap.
void OverlayFrame(HDC dc);

}  // namespace mxcam
