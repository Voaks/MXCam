// terrain.h
//   Fills in the ground the game leaves out of 3rd-person replays.
//
//   The terrain is drawn as a quadtree of square patches - one mesh, placed by 'u_meshoffset'
//   (the patch's corner, world x and z) and sized by 'u_meshscale' - finer near the camera. The
//   game skips every patch that lies entirely behind the plane through its own camera. Nothing
//   else is culled (trees, fences and buildings are always drawn), so when the 3rd person camera
//   faces away from where the game's replay camera looks, the ground under them goes missing.
//
//   Each terrain pass, the patches the game drew are recorded here. Fill() then picks the cells
//   that cover the rest of the map in front of the camera actually used, finer near it, without
//   overlapping anything the game drew; proc_hooks.cpp draws them the way the game draws its own.
//
//   The game draws each patch with one of several vertex arrays: the same mesh, its edges
//   stitched (every other vertex dropped) where the neighbour is twice as big, so the two meet
//   without cracks. Which variant stitches which edges is learnt from the game's own patches,
//   and filled cells are sized to fit the game's patches along them and stitched the same way.
#pragma once

namespace mxcam {
namespace terrain {

struct Patch {
    float offset[2]    = {};  // u_meshoffset: the patch's corner (world x, z)
    float scale        = 0;   // u_meshscale: its size
    float texOffset[2] = {};  // u_texoffset / u_texscale: a texture laid over part of the map
    float texScale     = 0;   // (0 = none), carried over to filled cells inside it
    int   variant      = -1;  // the game's: the vertex array it was drawn with (-1: not replayable);
                              // a filled cell's: the one to draw it with (-1: none learnt yet)
    int   edges        = 0;   // a filled cell's edges with a bigger neighbour (1 -x, 2 +x, 4 -z, 8 +z)
};

// A new terrain pass: forgets the previous pass's patches (not what was learnt about the map).
void BeginPass();

// A patch the game drew this pass. Also learns how big the map is and which patch sizes it uses.
void AddGamePatch(const Patch& patch);

// A new heightmap was uploaded (another track): forget the map.
void ResetMap();

// The width of the map from its heightmap, for before the game has drawn its far side.
void SetMapSize(double size);

// How many stitched-edge combinations have a known vertex array variant.
int LearntEdgeMasks();

// The cells that complete this pass for a camera at eye looking along forward (world space,
// y up). Writes up to maxOut of them and returns how many.
int Fill(const double eye[3], const double forward[3], Patch* out, int maxOut);

}  // namespace terrain
}  // namespace mxcam
