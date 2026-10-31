// terrain.cpp
//   See terrain.h. Cells are quadtree nodes: integer corner (x, z) and power-of-two size, the
//   root being the smallest power-of-two square at the origin that holds every patch seen.

#include "terrain.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "log.h"

namespace mxcam {
namespace terrain {
namespace {

// A cell is split while the camera is closer to it than this many times its size. The game
// splits by roughness as well as distance; this errs on the fine side.
constexpr double kSplitDistance = 3.0;
// Cells entirely this far (world units) behind the camera's horizontal plane are skipped. Generous, as
// heights aren't known and a camera looking down sees a little behind its own position.
constexpr double kBehindMargin = 48.0;
// Rounds of splitting filled cells next to much smaller ones.
constexpr int kBalancePasses = 6;
// How far (in samples) a patch corner or size may be off a whole number and still count as one.
constexpr double kAlignTolerance = 1e-2;
// The coverage grid has a square per smallest patch; maps needing more aren't stitched.
constexpr long long kMaxGridSquares = 1 << 20;
// A map size from the heightmap beyond this is taken for a misreading.
constexpr double kMaxMapSize = 65536;

// A cell's edges, as bits of a mask: the edges with a bigger neighbour, which the game's
// vertex array variants stitch (dropping every other vertex) so the mesh has no cracks.
constexpr int kMinusX = 1, kPlusX = 2, kMinusZ = 4, kPlusZ = 8;
constexpr int kEdges[4] = { kMinusX, kPlusX, kMinusZ, kPlusZ };

struct Cell {
    int x = 0, z = 0, size = 0;
};

struct GameCell {
    Cell cell;
    int  variant = -1;
};

// A texture laid over a square of the map (u_texoffset = -corner, u_texscale = 1 / size).
struct Overlay {
    double x = 0, z = 0, size = 0;
    float  texOffset[2] = {};
    float  texScale = 0;
};

struct FillCell {
    Cell  cell;
    float texOffset[2] = {};
    float texScale = 0;
};

struct Map {
    bool known      = false;
    bool unaligned  = false;  // a patch that isn't a quadtree cell from the origin: don't fill
    int  minSize    = 0;      // smallest and largest patches the game draws
    int  maxSize    = 0;
    int  extentX    = 0;      // the map ends at the furthest patch edge seen
    int  extentZ    = 0;
    std::unordered_map<int, int> variantEdges;  // vertex array variant -> the edges it stitches
    std::unordered_map<int, int> edgeVariant;   // and back
};

Map g_map;
double g_mapSize = 0;                  // from the heightmap, in samples (0: unknown)
double g_unit = 1;                     // world units per sample: cells are in samples
bool   g_unalignedLogged = false;      // the first patch off the grid has been reported
std::vector<GameCell> g_passCells;     // drawn by the game this pass
std::vector<Overlay>  g_passOverlays;
std::vector<Overlay>  g_overlays;      // from the latest pass that had any
bool g_passUnaligned = false;

// Built by Fill() from the pass's cells.
std::unordered_set<uint64_t> g_drawn;    // cells the game drew
std::unordered_set<uint64_t> g_parents;  // cells with a drawn cell somewhere inside
std::vector<FillCell> g_fill;
const char* g_fillNote = "";

// Which cell covers each smallest square of the map: the game's, then the filled ones.
struct Owner {
    Cell cell;
    bool game  = false;
    int  edges = -1;  // a game patch's stitched edges (-1: its variant isn't learnt)
};
std::vector<Owner> g_owners;
std::vector<int>   g_grid;  // per square: index into g_owners, -1 for none
int g_gridWidth = 0, g_gridDepth = 0;

struct Camera {
    double x = 0, z = 0;
    double forwardX = 0, forwardZ = 0;  // horizontal, unit length (both 0: looking straight down)
};

uint64_t Key(int x, int z, int size)
{
    unsigned level = 0;
    while ((1 << level) < size)
        ++level;
    return (static_cast<uint64_t>(level) << 58) | (static_cast<uint64_t>(x & 0x1FFFFFFF) << 29) |
           static_cast<uint64_t>(z & 0x1FFFFFFF);
}

bool IsPowerOfTwo(int v) { return v > 0 && (v & (v - 1)) == 0; }

int RootSize()
{
    int root = g_map.maxSize;
    while (root < g_map.extentX || root < g_map.extentZ)
        root *= 2;
    return root;
}

int EdgesOf(int variant)
{
    const auto found = g_map.variantEdges.find(variant);
    return found != g_map.variantEdges.end() ? found->second : -1;
}

bool ResetGrid()
{
    g_owners.clear();
    g_gridWidth = g_map.extentX / g_map.minSize;
    g_gridDepth = g_map.extentZ / g_map.minSize;
    if (static_cast<long long>(g_gridWidth) * g_gridDepth > kMaxGridSquares) {
        g_grid.clear();
        return false;
    }
    g_grid.assign(static_cast<size_t>(g_gridWidth) * g_gridDepth, -1);
    return true;
}

void AddOwner(const Owner& owner)
{
    if (g_grid.empty())
        return;
    const int index = static_cast<int>(g_owners.size());
    g_owners.push_back(owner);
    const int unit = g_map.minSize;
    const int x0 = owner.cell.x / unit, z0 = owner.cell.z / unit, n = owner.cell.size / unit;
    for (int z = z0; z < z0 + n && z < g_gridDepth; ++z) {
        for (int x = x0; x < x0 + n && x < g_gridWidth; ++x)
            g_grid[static_cast<size_t>(z) * g_gridWidth + x] = index;
    }
}

void AddGameOwners()
{
    for (const GameCell& g : g_passCells)
        AddOwner({ g.cell, true, EdgesOf(g.variant) });
}

const Owner* OwnerAt(int x, int z)
{
    if (g_grid.empty() || x < 0 || z < 0 || x >= g_map.extentX || z >= g_map.extentZ)
        return nullptr;
    const int index = g_grid[static_cast<size_t>(z / g_map.minSize) * g_gridWidth + x / g_map.minSize];
    return index < 0 ? nullptr : &g_owners[index];
}

// The point just across one edge of a cell, t along that edge.
void Across(const Cell& c, int edge, int t, int& x, int& z)
{
    switch (edge) {
    case kMinusX: x = c.x - 1;      z = c.z + t;      break;
    case kPlusX:  x = c.x + c.size; z = c.z + t;      break;
    case kMinusZ: x = c.x + t;      z = c.z - 1;      break;
    default:      x = c.x + t;      z = c.z + c.size; break;
    }
}

int Opposite(int edge)
{
    return edge == kMinusX ? kPlusX : edge == kPlusX ? kMinusX : edge == kMinusZ ? kPlusZ : kMinusZ;
}

// The edges of a cell with a bigger neighbour across them. A bigger neighbour spans the whole
// edge, so one point tells. known = false when there is nothing across some edge.
int CoarserEdges(const Cell& c, bool& known)
{
    int edges = 0;
    known = true;
    for (int edge : kEdges) {
        int x, z;
        Across(c, edge, 0, x, z);
        const Owner* o = OwnerAt(x, z);
        if (!o)
            known = false;
        else if (o->cell.size > c.size)
            edges |= edge;
    }
    return edges;
}

// Which vertex array variant stitches which edges, from the game's patches that have a
// neighbour drawn across every edge.
void Learn()
{
    for (const GameCell& g : g_passCells) {
        if (g.variant < 0)
            continue;
        bool known;
        const int edges = CoarserEdges(g.cell, known);
        if (!known)
            continue;
        const auto old = g_map.variantEdges.find(g.variant);
        if (old != g_map.variantEdges.end() && old->second != edges) {
            const auto back = g_map.edgeVariant.find(old->second);
            if (back != g_map.edgeVariant.end() && back->second == g.variant)
                g_map.edgeVariant.erase(back);
        }
        g_map.variantEdges[g.variant] = edges;
        g_map.edgeVariant[edges] = g.variant;
    }
}

// The sizes the game's patches along a cell's edges leave it: one drawn stitched toward the
// cell expects a neighbour twice its size, any other one no bigger and no smaller than half.
void Bounds(const Cell& c, int& lo, int& hi)
{
    lo = 0;
    hi = INT_MAX;
    for (int edge : kEdges) {
        for (int t = 0; t < c.size; t += g_map.minSize) {
            int x, z;
            Across(c, edge, t, x, z);
            const Owner* o = OwnerAt(x, z);
            if (!o || !o->game)
                continue;
            const int g = o->cell.size;
            if (o->edges >= 0 && (o->edges & Opposite(edge))) {
                lo = std::max(lo, 2 * g);
                hi = std::min(hi, 2 * g);
            } else {
                lo = std::max(lo, g / 2);
                hi = std::min(hi, g);
            }
        }
    }
}

// The cell itself, or a cell containing it, was drawn by the game.
bool Covered(const Cell& cell, int root)
{
    for (int size = cell.size; size <= root; size *= 2) {
        if (g_drawn.count(Key(cell.x - cell.x % size, cell.z - cell.z % size, size)))
            return true;
    }
    return false;
}

bool Behind(const Cell& cell, const Camera& camera)
{
    if (camera.forwardX == 0 && camera.forwardZ == 0)
        return false;
    double nearest = -1e300;
    for (int corner = 0; corner < 4; ++corner) {
        const double x = cell.x + (corner & 1 ? cell.size : 0) - camera.x;
        const double z = cell.z + (corner & 2 ? cell.size : 0) - camera.z;
        nearest = std::fmax(nearest, x * camera.forwardX + z * camera.forwardZ);
    }
    return nearest < -kBehindMargin / g_unit;
}

double Distance(const Cell& cell, const Camera& camera)
{
    const double dx = std::fmax(0.0, std::fmax(cell.x - camera.x, camera.x - (cell.x + cell.size)));
    const double dz = std::fmax(0.0, std::fmax(cell.z - camera.z, camera.z - (cell.z + cell.size)));
    return std::sqrt(dx * dx + dz * dz);
}

// 1: inside an overlay (out = that overlay), -1: partly over one, 0: clear of them all.
int OverlayOf(const Cell& cell, const Overlay*& out)
{
    for (const Overlay& o : g_overlays) {
        const bool apart = cell.x >= o.x + o.size || cell.x + cell.size <= o.x ||
                           cell.z >= o.z + o.size || cell.z + cell.size <= o.z;
        if (apart)
            continue;
        const bool inside = cell.x >= o.x && cell.x + cell.size <= o.x + o.size &&
                            cell.z >= o.z && cell.z + cell.size <= o.z + o.size;
        if (!inside)
            return -1;
        out = &o;
        return 1;
    }
    return 0;
}

void Visit(const Cell& cell, int root, const Camera& camera, int maxOut)
{
    if (static_cast<int>(g_fill.size()) >= maxOut || cell.x >= g_map.extentX || cell.z >= g_map.extentZ)
        return;
    if (Behind(cell, camera) || Covered(cell, root))
        return;

    const bool outside = cell.x + cell.size > g_map.extentX || cell.z + cell.size > g_map.extentZ;
    const Overlay* overlay = nullptr;
    const int overlayState = OverlayOf(cell, overlay);
    bool split = g_parents.count(Key(cell.x, cell.z, cell.size)) || outside || overlayState < 0;
    if (!split) {
        int lo, hi;
        Bounds(cell, lo, hi);
        split = cell.size > hi || (cell.size / 2 >= lo && (cell.size > g_map.maxSize ||
                                                           Distance(cell, camera) < kSplitDistance * cell.size));
    }
    if (split && cell.size > g_map.minSize) {
        const int half = cell.size / 2;
        for (int child = 0; child < 4; ++child)
            Visit({ cell.x + (child & 1 ? half : 0), cell.z + (child & 2 ? half : 0), half }, root, camera, maxOut);
        return;
    }
    if (outside)
        return;  // a smallest cell hanging over the edge of the map

    FillCell fill;
    fill.cell = cell;
    if (overlay) {
        fill.texOffset[0] = overlay->texOffset[0];
        fill.texOffset[1] = overlay->texOffset[1];
        fill.texScale = overlay->texScale;
    }
    g_fill.push_back(fill);
}

// Splits filled cells next to one less than half their size (the mesh only stitches to a
// neighbour twice as big), unless a game patch along them needs them this big.
bool Balance(int maxOut)
{
    bool changed = false;
    std::vector<FillCell> next;
    next.reserve(g_fill.size());
    for (size_t i = 0; i < g_fill.size(); ++i) {
        const FillCell& f = g_fill[i];
        bool finer = false;
        for (int edge : kEdges) {
            for (int t = 0; t < f.cell.size && !finer; t += g_map.minSize) {
                int x, z;
                Across(f.cell, edge, t, x, z);
                const Owner* o = OwnerAt(x, z);
                finer = o && o->cell.size * 2 < f.cell.size;
            }
        }
        int lo = 0, hi = 0;
        if (finer)
            Bounds(f.cell, lo, hi);
        const size_t remaining = g_fill.size() - i - 1;
        if (!finer || f.cell.size <= g_map.minSize || f.cell.size / 2 < lo ||
            next.size() + 4 + remaining > static_cast<size_t>(maxOut)) {
            next.push_back(f);
            continue;
        }
        const int half = f.cell.size / 2;
        for (int child = 0; child < 4; ++child) {
            FillCell c = f;
            c.cell = { f.cell.x + (child & 1 ? half : 0), f.cell.z + (child & 2 ? half : 0), half };
            if (c.cell.x < g_map.extentX && c.cell.z < g_map.extentZ)
                next.push_back(c);
        }
        changed = true;
    }
    g_fill.swap(next);
    return changed;
}

}  // namespace

void BeginPass()
{
    g_passCells.clear();
    g_passOverlays.clear();
    g_passUnaligned = false;
}

void AddGamePatch(const Patch& patch)
{
    const double x = patch.offset[0] / g_unit, z = patch.offset[1] / g_unit, size = patch.scale / g_unit;
    const Cell cell{ static_cast<int>(std::lround(x)), static_cast<int>(std::lround(z)),
                     static_cast<int>(std::lround(size)) };
    if (!IsPowerOfTwo(cell.size) || cell.x < 0 || cell.z < 0 || cell.x % cell.size || cell.z % cell.size ||
        std::fabs(x - cell.x) > kAlignTolerance || std::fabs(z - cell.z) > kAlignTolerance ||
        std::fabs(size - cell.size) > kAlignTolerance) {
        if (!g_unalignedLogged) {
            g_unalignedLogged = true;
            Log("terrain: a patch off the quadtree grid (corner %g %g, size %g; %g world units per sample): "
                "the ground can't be filled in on this map", patch.offset[0], patch.offset[1], patch.scale, g_unit);
        }
        g_passUnaligned = true;
        return;
    }
    g_passCells.push_back({ cell, patch.variant });

    if (!g_map.known) {
        g_map.known = true;
        g_map.minSize = g_map.maxSize = cell.size;
    }
    g_map.minSize = cell.size < g_map.minSize ? cell.size : g_map.minSize;
    g_map.maxSize = cell.size > g_map.maxSize ? cell.size : g_map.maxSize;
    g_map.extentX = cell.x + cell.size > g_map.extentX ? cell.x + cell.size : g_map.extentX;
    g_map.extentZ = cell.z + cell.size > g_map.extentZ ? cell.z + cell.size : g_map.extentZ;

    if (patch.texScale > 0) {
        Overlay overlay;
        overlay.size = 1.0 / (patch.texScale * g_unit);
        overlay.x = -patch.texOffset[0] / g_unit;
        overlay.z = -patch.texOffset[1] / g_unit;
        overlay.texOffset[0] = patch.texOffset[0];
        overlay.texOffset[1] = patch.texOffset[1];
        overlay.texScale = patch.texScale;
        bool known = false;
        for (const Overlay& o : g_passOverlays)
            known = known || (o.texOffset[0] == overlay.texOffset[0] && o.texOffset[1] == overlay.texOffset[1] &&
                              o.texScale == overlay.texScale);
        if (!known)
            g_passOverlays.push_back(overlay);
    }
}

void ResetMap()
{
    g_map = Map{};
    g_overlays.clear();
    g_unalignedLogged = false;
}

void SetMapSize(double size)
{
    const double samples = size / g_unit;
    g_mapSize = samples > 0 && samples <= kMaxMapSize ? samples : 0;
}

void SetGridUnit(double unit)
{
    g_unit = unit > 1e-6 ? unit : 1;
}

int LearntEdgeMasks()
{
    return static_cast<int>(g_map.edgeVariant.size());
}

int Fill(const double eye[3], const double forward[3], Patch* out, int maxOut)
{
    if (!g_passOverlays.empty())
        g_overlays = g_passOverlays;
    g_fillNote = g_passUnaligned ? "unaligned" : !g_map.known ? "no-map" : g_passCells.empty() ? "no-patches" : "ok";
    if (!g_map.known || g_passUnaligned || g_passCells.empty() || maxOut <= 0)
        return 0;
    if (g_mapSize > 0) {
        const int size = static_cast<int>(std::lround(g_mapSize / g_map.minSize)) * g_map.minSize;
        g_map.extentX = std::max(g_map.extentX, size);
        g_map.extentZ = std::max(g_map.extentZ, size);
    }

    const int root = RootSize();
    g_drawn.clear();
    g_parents.clear();
    for (const GameCell& g : g_passCells) {
        const Cell& cell = g.cell;
        g_drawn.insert(Key(cell.x, cell.z, cell.size));
        for (int size = cell.size * 2; size <= root; size *= 2)
            g_parents.insert(Key(cell.x - cell.x % size, cell.z - cell.z % size, size));
    }

    // The game's patches on the grid, what their variants stitch, and what that says about them.
    const bool grid = ResetGrid();
    if (grid) {
        AddGameOwners();
        Learn();
        for (size_t i = 0; i < g_passCells.size() && i < g_owners.size(); ++i)
            g_owners[i].edges = EdgesOf(g_passCells[i].variant);
    }

    Camera camera;
    camera.x = eye[0] / g_unit;
    camera.z = eye[2] / g_unit;
    const double horizontal = std::sqrt(forward[0] * forward[0] + forward[2] * forward[2]);
    if (horizontal > 1e-3) {
        camera.forwardX = forward[0] / horizontal;
        camera.forwardZ = forward[2] / horizontal;
    }
    g_fill.clear();
    Visit({ 0, 0, root }, root, camera, maxOut);

    if (grid) {
        for (int pass = 0;; ++pass) {
            ResetGrid();
            AddGameOwners();
            for (const FillCell& f : g_fill)
                AddOwner({ f.cell, false, -1 });
            if (pass == kBalancePasses || !Balance(maxOut))
                break;
        }
    }

    const int count = std::min(static_cast<int>(g_fill.size()), maxOut);
    if (count >= maxOut)
        g_fillNote = "limit";
    for (int i = 0; i < count; ++i) {
        const FillCell& f = g_fill[i];
        Patch& patch = out[i];
        patch = Patch{};
        patch.offset[0] = static_cast<float>(f.cell.x * g_unit);
        patch.offset[1] = static_cast<float>(f.cell.z * g_unit);
        patch.scale = static_cast<float>(f.cell.size * g_unit);
        patch.texOffset[0] = f.texOffset[0];
        patch.texOffset[1] = f.texOffset[1];
        patch.texScale = f.texScale;
        if (grid) {
            bool known;
            patch.edges = CoarserEdges(f.cell, known);
            auto found = g_map.edgeVariant.find(patch.edges);
            if (found == g_map.edgeVariant.end())
                found = g_map.edgeVariant.find(0);
            patch.variant = found != g_map.edgeVariant.end() ? found->second : -1;
        }
    }
    return count;
}

const char* LastFillNote() { return g_fillNote; }

}  // namespace terrain
}  // namespace mxcam
