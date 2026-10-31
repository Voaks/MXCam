// gamepad.cpp
//   See gamepad.h. Both APIs are loaded on first use, so a system without them just reads no
//   controller. Asking either API about an empty slot can take a millisecond or more, so only
//   the controller in use is polled each frame; a background thread searches the other slots
//   once a second, so the search can't hitch a frame.
#include "gamepad.h"

#include <windows.h>
#include <mmsystem.h>
#include <xinput.h>

#include <atomic>

#include "log.h"

namespace mxcam {
namespace gamepad {
namespace {

typedef DWORD(WINAPI* XInputGetStateFn)(DWORD, XINPUT_STATE*);
typedef MMRESULT(WINAPI* JoyGetPosExFn)(UINT, LPJOYINFOEX);

constexpr DWORD kSearchIntervalMs = 1000;
constexpr UINT  kJoysticks        = 16;  // ids the joystick API hands out

bool             g_started = false;
XInputGetStateFn g_xinputGetState = nullptr;
JoyGetPosExFn    g_joyGetPosEx = nullptr;
std::atomic<int> g_xinputPad{ -1 };  // XInput slot in use, -1 = none
std::atomic<int> g_joystick{ -1 };   // joystick id in use, -1 = none

bool ReadXInput(int pad, DPad& out)
{
    XINPUT_STATE state{};
    if (g_xinputGetState(DWORD(pad), &state) != ERROR_SUCCESS)
        return false;
    const WORD buttons = state.Gamepad.wButtons;
    out.left  = (buttons & XINPUT_GAMEPAD_DPAD_LEFT) != 0;
    out.right = (buttons & XINPUT_GAMEPAD_DPAD_RIGHT) != 0;
    out.down  = (buttons & XINPUT_GAMEPAD_DPAD_DOWN) != 0;
    return true;
}

// The joystick API reports the d-pad as a hat: centred, or an angle in hundredths of a degree
// clockwise from up. Each direction takes the 90 deg around it; the diagonals into down count
// as down.
bool ReadJoystick(int id, DPad& out)
{
    JOYINFOEX info{};
    info.dwSize = sizeof info;
    info.dwFlags = JOY_RETURNPOV | JOY_RETURNPOVCTS;
    if (g_joyGetPosEx(UINT(id), &info) != JOYERR_NOERROR)
        return false;
    out = DPad{};
    const DWORD pov = info.dwPOV;
    if (pov <= 35999) {
        out.down  = pov >= 13500 && pov <= 22500;
        out.right = pov > 4500 && pov < 13500;
        out.left  = pov > 22500 && pov < 31500;
    }
    return true;
}

// XInput controllers are preferred: one plugged in takes over from a joystick.
DWORD WINAPI SearchThread(void*)
{
    for (;;) {
        if (g_xinputPad < 0 && g_xinputGetState) {
            for (int i = 0; i < XUSER_MAX_COUNT; ++i) {
                DPad unused;
                if (ReadXInput(i, unused)) {
                    g_xinputPad = i;
                    Log("gamepad: using XInput controller %d", i);
                    break;
                }
            }
        }
        if (g_xinputPad < 0 && g_joystick < 0 && g_joyGetPosEx) {
            for (UINT id = 0; id < kJoysticks; ++id) {
                JOYINFOEX info{};
                info.dwSize = sizeof info;
                info.dwFlags = JOY_RETURNPOV;
                if (g_joyGetPosEx(id, &info) == JOYERR_NOERROR) {
                    g_joystick = int(id);
                    Log("gamepad: using joystick %u", id);
                    break;
                }
            }
        }
        Sleep(kSearchIntervalMs);
    }
}

void Start()
{
    g_started = true;
    const wchar_t* const xinputs[] = { L"xinput1_4.dll", L"xinput1_3.dll", L"xinput9_1_0.dll" };
    for (const wchar_t* name : xinputs) {
        if (HMODULE dll = LoadLibraryW(name)) {
            g_xinputGetState = reinterpret_cast<XInputGetStateFn>(GetProcAddress(dll, "XInputGetState"));
            if (g_xinputGetState)
                break;
        }
    }
    if (HMODULE winmm = LoadLibraryW(L"winmm.dll"))
        g_joyGetPosEx = reinterpret_cast<JoyGetPosExFn>(GetProcAddress(winmm, "joyGetPosEx"));
    Log("gamepad: XInput %s, joystick API %s", g_xinputGetState ? "found" : "missing",
        g_joyGetPosEx ? "found" : "missing");
    if (!g_xinputGetState && !g_joyGetPosEx)
        return;
    if (HANDLE thread = CreateThread(nullptr, 0, SearchThread, nullptr, 0, nullptr))
        CloseHandle(thread);
    else
        Log("gamepad: could not start the controller search (error %lu)", GetLastError());
}

}  // namespace

DPad Read()
{
    if (!g_started)
        Start();
    DPad pad;
    const int xinputPad = g_xinputPad;
    if (xinputPad >= 0) {
        if (ReadXInput(xinputPad, pad)) {
            g_joystick = -1;
            return pad;
        }
        Log("gamepad: XInput controller %d disconnected", xinputPad);
        g_xinputPad = -1;
    }
    const int joystick = g_joystick;
    if (joystick >= 0 && !ReadJoystick(joystick, pad)) {
        Log("gamepad: joystick %d disconnected", joystick);
        g_joystick = -1;
    }
    return pad;
}

const char* StatusText()
{
    if (g_xinputPad >= 0)
        return "XBOX PAD";
    if (g_joystick >= 0)
        return "PAD";
    return "NO PAD";
}

}  // namespace gamepad
}  // namespace mxcam
