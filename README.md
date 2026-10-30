# MXCam: dynamic helmet cam for MX Simulator

MXCam is a proxy `opengl32.dll`. The game loads it instead of the system OpenGL library. It forwards
every OpenGL call to the real driver, but first it picks out the first-person camera matrix and
reworks it with some simple physics:

- **Inertial smoothing**: the view follows the head bone through a frame-rate-independent
  low-pass filter, which removes violent pitch and yaw spikes from the animation.
- **Horizon lock**: the view is rolled back toward the horizon by a share of the bike's lean,
  the way a real rider keeps their eyes level.
- **Pitch lock**: a share of the bike's pitch is taken out, so a wheelie or a jump face doesn't
  point the view at the sky.
- **Custom camera** (optional): the view is built from the rider's own bike, so the game's
  1st Person Angle and Gimbal settings no longer matter. It has its own look-down angle, head
  position, air looseness and look-around offsets.
- **Neck compression**: a spring-damper driven by vertical acceleration. Hard landings make the
  head sag and nod, and airtime lets it float. The eye height is also low-pass filtered to iron
  out jitter.

## Build

You need Visual Studio 2022 with the "Desktop development with C++" workload.

- **IDE**: open `MXCamProxy.vcxproj`, pick `Release` and `Win32` or `x64`, then build.
- **Command line**: run `build.bat`. It builds both architectures.

The output goes to `bin\<Win32|x64>\Release\` as `opengl32.dll`, together with a copy of
`mxcam.ini`. The CRT is linked statically, so the DLL doesn't need the VC++ redistributable.

## Install

1. **Match the game's bitness.** A 32-bit game can only load the `Win32` DLL and a 64-bit game
   can only load the `x64` one. To check, start the game and look in Task Manager: 32-bit
   processes show "(32 bit)". You can also run this in PowerShell:
   ```powershell
   $b=[IO.File]::ReadAllBytes("C:\path\to\mx.exe"); $pe=[BitConverter]::ToInt32($b,0x3C)
   if ([BitConverter]::ToUInt16($b,$pe+4) -eq 0x8664) {"64-bit"} else {"32-bit"}
   ```
2. Copy `opengl32.dll` and `mxcam.ini` into the folder that holds the game's executable.
   `opengl32.dll` is not a "KnownDLL", so Windows loads it from the application folder first.
3. Start the game. A new `mxcam.log` should appear next to the DLL. To uninstall, delete
   `opengl32.dll`.

Try it offline first. The mod only changes what you see, but check your server's rules about
modified game files before you race online with it.

| Key | Action |
|-----|--------|
| E   | Open or close the settings panel (`MenuKey` in `mxcam.ini`). |
| F8  | Turn the effect on or off. Leave it off for third-person, replays and menus. |
| F9  | Reload `mxcam.ini` without restarting the game. |
| F10 | Dump every matrix call of the next frame to `mxcam.log`. |
| F11 | Save the last 15 s (and the next 5 s) of per-frame camera data to `mxcam_trace.csv` (`TraceKey`): press it right after a jerk. |

## First run: finding the camera matrix

The mod can't be checked against MX Simulator until it runs inside the game, so plan on this
calibration step:

1. Ride in first-person view and press **F10**.
2. Open `mxcam.log`. Each line shows the matrix mode, whether a `3D` (perspective) or `2D`
   projection is active, and the call itself, with matrices printed row by row. Every modelview
   load in a 3D pass is numbered `^ scene load #N`, and the one marked `CAMERA` is the load the
   mod treats as the view matrix.
3. The camera load is usually the first 3D load of the frame. Its rotation part changes as you
   look around, and its translation is roughly minus your position in the world. If `CAMERA`
   landed on something else, such as a shadow, reflection or sky pass, set `ViewMatrixIndex` to
   the right number and press **F9**.
4. If the world uses Z as up (the log shows gravity along Z), set `WorldUp=0,0,1`.

The log also says when the game doesn't seem to use the hooked functions. See "Limits" below.

## How it works

```
 game ──► opengl32.dll (MXCam, game folder)
            │  368 exports. Each one is a 16-byte stub that runs "jmp [target]".
            │  347 stubs jump straight to the system DLL, so any signature passes through.
            │  21 stubs jump to typed hooks in hooks.cpp:
            │    matrix state:  glMatrixMode, glLoadIdentity, glLoad/MultMatrix{f,d},
            │                   glPush/PopMatrix, glFrustum, glOrtho, glRotate/Translate/Scale{f,d}
            │    display lists: glNewList, glEndList
            │    frame end:     wglSwapBuffers, wglSwapLayerBuffers, glClear (fallback)
            ▼
          C:\Windows\System32\opengl32.dll  (SysWOW64 for 32-bit, via WOW64 redirection)
```

**Proxy (`src/proxy.cpp`, `src/gl_exports.inl`).** An X-macro list holds every export name. For
each name the code emits a stub array in its own `.pxystub` section and a
`#pragma comment(linker, "/EXPORT:name=Proxy_name")`, so no `.def` file is needed. When the DLL
attaches, it loads the real DLL by its full system path, writes each stub's jump target, and
seals the section as read+execute. The same code works on x86 and x64, where MSVC has no inline
asm or naked functions.

**Finding the camera (`src/hooks.cpp`).** The hooks mirror just enough fixed-function state to
know whether the current projection is perspective (3D scene) or not (HUD, menus) at any moment.
That state covers the matrix mode, the projection stack with push/pop, and whether the modelview
top is a fresh identity. Modelview loads in 3D passes are numbered each frame, and load number
`ViewMatrixIndex` is taken as the view matrix. `glLoadIdentity()` followed by `glMultMatrix(view)`
counts as a load.

**Finding the bike (`src/bike.cpp`).** The game camera rides on the bike, sprung so that it
bounces about 10 cm against it. The tracker follows every rigid object drawn within
`BikeSearchRadius` of the camera from frame to frame. It locks onto the steadiest one: over a
second, while the camera travels five metres, the camera's position in that object's space
must stay within a few tens of centimetres, and the object's orientation relative to the camera
must barely change. Scenery and other riders slide away by metres, wheels spin and the fork
steers, so the bike frame wins. The lock is held through gaps of up to half a second, and the
custom camera eases in and out instead of snapping. The bike's forward and up model axes are
taken as the ones nearest the camera's. The custom camera then uses the bike's own world-space
orientation, so the game camera's wobble against the bike never reaches the view, and the
bike's own vertical acceleration decides when you're airborne (for air looseness), so camera
bounces on the ground can't fake a jump. The log says when the bike is found or lost, and the
F10 dump lists every track with its bounce and wobble.

**Rewriting the view (`src/camera.cpp`).** The raw view is split into a head orientation
(quaternion) and an eye position. The physics produces a new view, and the code computes an
*eye-space* correction from it:

```
C = NewView * inverse(RawView)
```

Every later 3D modelview load in that frame is replaced with `C * M`. This is correct whether the
engine loads the bare view and multiplies objects onto it, or loads a full `View * Model` per
object, because `C * View * Model = NewView * Model`. The hooks never need to know which object
is which.

**Safeguards:**
- Loads under an orthographic projection (HUD) are never touched.
- Display-list compilation passes through untouched, so a correction never gets baked into a list.
- If a matrix the mod already corrected is read back and loaded again, it isn't corrected twice.
- Non-rigid or mirrored "view" matrices are rejected.
- Respawns and camera switches (large jumps) reset the filters.
- A pause or alt-tab longer than 0.25 s resyncs the filters instead of integrating one huge step.

## Settings panel

Press **E** in-game to open a black-and-white panel on the left of the screen. It has a switch
that turns the effect on or off, and three tabs. Changes apply immediately.

- **VIEW**: game or custom camera, keep game angle, look-down angle, horizon lock, pitch lock,
  head forward / back and head height.
- **AIR & LOOK**: air looseness, and the eye offsets while
  looking sideways or back.
- **NECK**: rotation smoothing and the neck / impact physics.

Rows that have no effect with the current choices (custom-camera settings while the game camera
is selected, for example) are dimmed. The line under the rows shows live values for the tab.

- **Mouse:** drag a slider, or click anywhere on its track to jump there.
- **Keyboard:** Up and Down move between controls (the tab bar is the first one). Left and Right
  adjust the selected one or switch tabs, and holding Shift makes each step ten times bigger.
  Enter presses buttons and flips switches.
- **SAVE** writes the values into `mxcam.ini`. The file's comments and other keys are kept.
- **DEFAULTS** restores the built-in values. They aren't saved until you press SAVE.
- **E** or **Esc** closes the panel. Unsaved changes stay active until you quit the game.

While the panel is open, mouse clicks and key presses go to the panel instead of the game. Key
and button releases still reach the game, so nothing gets stuck down. If the game reads the
keyboard directly (for example through DirectInput), the arrow keys may steer the bike as well,
so stop before adjusting.

The panel is drawn into the game's own frame with a small shader of its own. It saves and
restores every piece of GL state it touches, and costs nothing while it's closed.

## Tuning (`mxcam.ini`)

| Key | Default | Effect |
|-----|---------|--------|
| `RotationResponse` | 14 /s | How quickly the view catches up with the head. Lower is floatier, 0 turns smoothing off. |
| `MaxLagDeg` | 12 | The furthest the smoothed view may trail the real head. |
| `CustomCamera` | 0 | 1 builds the view from the bike, ignoring the game's 1st Person Angle and Gimbal. |
| `KeepGameAngle` / `CameraAngleDeg` | 0 / 8 | Custom camera: keep the game's look-down angle, or use this one. |
| `HeadForward` / `HeadHeight` | 0 / 0 | Eye offset in metres along the bike (game camera: its own axes). |
| `HorizonLock` | 0.6 | 0 keeps the stock roll, 1 locks the horizon level. (Was `LeanCompensation`.) |
| `PitchLock` | 0.5 | Share of the bike's pitch removed: wheelies, stoppies, jump faces. |
| `AirLooseness` | 0.6 | In the air, the view follows the bike's left / right swing (scrubs, whips) only through a slow filter, so it keeps pointing forward more; it catches up smoothly after landing. The view never trails the nose by more than 15 deg x looseness, so turning the bike through a jump out of a corner doesn't leave it behind. 0 = tight, 1 = very loose. Nothing is locked to a direction of travel. |
| `LookSide*` / `LookBack*` | 0 | Custom camera: extra eye offsets while looking sideways or back. |
| `VerticalResponse` / `MaxVerticalLag` | 20 /s / 0.05 | Low-pass filter on eye height. |
| `CompressionPerG` | 0.012 | How far the neck sags per g of vertical acceleration. |
| `SpringHz` / `DampingRatio` | 3.0 / 0.5 | Neck stiffness and bounce. |
| `MaxCompression` / `MaxExtension` | 0.07 / 0.04 | End stops for neck travel. |
| `NodDegPerMeter` | 40 | Forward head nod per unit of compression. |
| `AccelFilterHz` | 12 | Speed of the velocity / acceleration estimate (fit window = 1.2 / value s). |

`mxcam.ini` has a comment on every key.

## Limits and what to try next

- **Shader matrices (how MX Simulator actually draws).** MX Simulator doesn't use the
  fixed-function matrix stack. It sends View × Model to every shader as a `u_modelview` uniform.
  `src/proc_hooks.cpp` wraps `glUniformMatrix4fv` through `wglGetProcAddress`. It takes the camera
  from the terrain program (the one with a `u_heights` uniform, which is drawn with the bare view
  matrix) and corrects every rigid `u_modelview` upload. Scaled uploads (HUD) and identity uploads
  (2D screens) are left alone. The sky is drawn before the terrain with a rotation-only view.
  That rotation is all the orientation half of the pipeline needs, so the view's orientation is
  previewed at the sky and the sky turns in step with the world in the same frame. The sky
  gets only the rotation part of the correction, so the neck and head offsets never move it.
  A preview runs on a copy of the filter state: only the terrain, once per frame, advances the
  filters. The game also draws other rotation-only things before the terrain, tens of degrees
  off the camera; the one closest to last frame's camera is taken as the sky, and the others
  can never step or reset the filters. (Before this, such a draw reset every filter twice in a
  row a few times a second, which showed as constant stutter.)
  Every filter steps by the smoothed frame interval, and velocity and acceleration come from a
  least-squares fit over the last ~0.1 s (`src/motion.h`). Uneven frame timing can't make the
  view judder or fake a jump.
  Reflections use `u_envmat` (`EnvMatrixUniform`), the eye-to-world rotation, which gets the
  same correction so chrome and paint reflections don't swim when the view is leveled.
  Lighting is computed in eye space: the light direction (`u_light_position`, set by
  `LightUniform`) arrives already turned by the game's view, so it gets the same rotation as the
  view. Without that, leveling the horizon would also tilt the sun, and the track would get
  lighter or darker as you ride.
- **Cameras built from `glRotate` / `glTranslate` / `gluLookAt`.** The camera is only detected
  through a load, or a multiply onto identity. With `gluLookAt` the rotation is still smoothed
  and leveled, but the position-based neck physics sees no movement.
- **HUD drawn with a perspective projection.** It would sway with the view. If that happens, the
  dump shows which loads they are.
- **Custom camera position.** The view's orientation comes from the bike alone, but its
  position still starts from where the game puts its camera, so the game's 1st Person Height
  still moves the base point. Leave it at one value and adjust with Head forward / Head height.
- **Custom camera needs the bike on screen.** In replays, menus and third person there is no bike
  next to the camera, so the game camera is used. After loading a track, ride a few metres
  before the custom camera takes over.
- **Pitch lock learns the bike's resting pitch** over the last half minute of riding on the
  ground. Right after loading it starts from the first pitch it sees.
- **One GL thread and context** is assumed, which is the normal setup for a game like this.

## Smoke test

`tests/smoke_test.cpp` loads a built DLL, creates a real GL context through it, and checks these
cases by reading the modelview matrix back from the driver:

- 2D passes are left untouched.
- Roll compensation works (20° becomes 8° with the default settings).
- While riding, the tracker finds a bike drawn under the camera (not the spinning wheel or the
  scenery), and pitch lock halves a 30° wheelie.
- Object matrices stay consistent with the corrected view.
- Push/pop of an ortho projection for a HUD is handled.
- The identity-plus-multiply camera idiom is corrected like a load.
- The neck compresses on a simulated landing, then recovers.
- A rotation-only draw 90° off the camera before the sky, and uneven frame times, never reset
  the filters, and the sky still turns exactly with the world.

Build and run it from a Developer Command Prompt that matches the DLL's bitness:

```
cl /nologo /EHsc /std:c++17 tests\smoke_test.cpp user32.lib gdi32.lib
smoke_test.exe bin\x64\Release\opengl32.dll
```

To check the custom camera, copy the DLL into a folder with an `mxcam.ini` that holds
`[Camera]`, `CustomCamera=1` and `HeadHeight=0.1`, then run the test on that copy with `--custom`.
