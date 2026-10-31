// terrain_test.cpp
//   Checks terrain::Fill against a simulated MX Simulator terrain pass: a 2048 x 2048 map drawn
//   as a balanced quadtree of patches, each with the vertex array variant that stitches its
//   edges toward bigger neighbours, skipping the patches entirely behind the game camera.
//
//   Build: cl /EHsc /std:c++17 /W4 tests\terrain_test.cpp src\terrain.cpp

#include "../src/terrain.h"

#include <cmath>
#include <cstdio>
#include <map>
#include <vector>

using namespace mxcam;

namespace {

constexpr int kMap = 2048, kMin = 32, kMax = 512;
constexpr int kMinusX = 1, kPlusX = 2, kMinusZ = 4, kPlusZ = 8;
constexpr int kVariantBase = 100;  // the simulated game's vertex array for edge mask m: 100 + m

int g_failures = 0;
bool g_verbose = false;  // -v: the first problem of each kind per pass
void Check(bool ok, const char* what)
{
    printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    g_failures += ok ? 0 : 1;
}

struct Eye {
    double x, y, z;
};

// The game's split rule (distance only here; balanced, as the real tree is).
bool GameSplits(int x, int z, int size, const Eye& eye)
{
    const double dx = std::fmax(0.0, std::fmax(x - eye.x, eye.x - (x + size)));
    const double dz = std::fmax(0.0, std::fmax(z - eye.z, eye.z - (z + size)));
    return size > kMax || (size > kMin && std::sqrt(dx * dx + dz * dz) < 2.0 * size);
}

// The size of the game's leaf (culled or not) holding a point, 0 outside the map.
int LeafSize(int px, int pz, const Eye& eye)
{
    if (px < 0 || pz < 0 || px >= kMap || pz >= kMap)
        return 0;
    int x = 0, z = 0, size = kMap;
    while (GameSplits(x, z, size, eye)) {
        size /= 2;
        x += px >= x + size ? size : 0;
        z += pz >= z + size ? size : 0;
    }
    return size;
}

// The game's stitching: edges with a bigger leaf across. Past the +x / +z edge of the map the
// game's tree goes on, as one big node (the real game stitches those edges).
int GameEdges(int x, int z, int size, const Eye& eye)
{
    const auto bigger = [&](int px, int pz, bool beyondMap) {
        if (px >= kMap || pz >= kMap)
            return beyondMap;
        const int leaf = LeafSize(px, pz, eye);
        return leaf > size;
    };
    int edges = 0;
    edges |= bigger(x - 1, z, false) ? kMinusX : 0;
    edges |= bigger(x + size, z, true) ? kPlusX : 0;
    edges |= bigger(x, z - 1, false) ? kMinusZ : 0;
    edges |= bigger(x, z + size, true) ? kPlusZ : 0;
    return edges;
}

struct Square {
    int x, z, size, edges;
    bool game;
};

struct Result {
    int game = 0, filled = 0;
    int holes = 0, overlaps = 0, wrongVariant = 0, unlearnt = 0, seams = 0, jumps = 0, sharedEdges = 0;
};

Result Run(const Eye& gameEye, const double gameForward[3], const Eye& chase, const double chaseForward[3])
{
    Result r;
    std::vector<Square> squares;

    // The game's pass.
    terrain::BeginPass();
    std::vector<int> stack{ 0, 0, kMap };
    while (!stack.empty()) {
        const int size = stack.back(), z = stack[stack.size() - 2], x = stack[stack.size() - 3];
        stack.resize(stack.size() - 3);
        double ahead = -1e9;
        for (int c = 0; c < 4; ++c) {
            const double vx = x + (c & 1 ? size : 0) - gameEye.x, vy = -gameEye.y, vz = z + (c & 2 ? size : 0) - gameEye.z;
            ahead = std::fmax(ahead, vx * gameForward[0] + vy * gameForward[1] + vz * gameForward[2]);
        }
        if (ahead < 0)
            continue;
        if (GameSplits(x, z, size, gameEye)) {
            const int half = size / 2;
            for (int c = 0; c < 4; ++c)
                stack.insert(stack.end(), { x + (c & 1 ? half : 0), z + (c & 2 ? half : 0), half });
            continue;
        }
        terrain::Patch patch;
        patch.offset[0] = float(x);
        patch.offset[1] = float(z);
        patch.scale = float(size);
        const int edges = GameEdges(x, z, size, gameEye);
        patch.variant = kVariantBase + edges;
        terrain::AddGamePatch(patch);
        squares.push_back({ x, z, size, edges, true });
        ++r.game;
    }

    static terrain::Patch filled[1024];
    const double eye[3] = { chase.x, chase.y, chase.z };
    const int count = terrain::Fill(eye, chaseForward, filled, 1024);
    r.filled = count;
    // A filled cell's edges: those its variant actually stitches.
    std::vector<int> fillVariant;
    for (int i = 0; i < count; ++i) {
        const int edges = filled[i].variant >= kVariantBase ? filled[i].variant - kVariantBase : 0;
        squares.push_back({ int(filled[i].offset[0]), int(filled[i].offset[1]), int(filled[i].scale), edges, false });
        fillVariant.push_back(filled[i].variant);
    }

    // Who covers each 32 m square.
    const int n = kMap / kMin;
    std::vector<int> owner(n * n, -1);
    for (int i = 0; i < int(squares.size()); ++i) {
        const Square& s = squares[i];
        for (int z = s.z / kMin; z < (s.z + s.size) / kMin; ++z) {
            for (int x = s.x / kMin; x < (s.x + s.size) / kMin; ++x) {
                int& o = owner[z * n + x];
                if (o >= 0)
                    ++r.overlaps;
                o = i;
            }
        }
    }
    // Holes: squares wholly in front of the chase camera (horizontally) that nothing covers.
    const double h = std::sqrt(chaseForward[0] * chaseForward[0] + chaseForward[2] * chaseForward[2]);
    for (int z = 0; z < n; ++z) {
        for (int x = 0; x < n; ++x) {
            bool ahead = true;
            for (int c = 0; c < 4; ++c) {
                const double vx = (x + (c & 1)) * kMin - chase.x, vz = (z + (c >> 1)) * kMin - chase.z;
                ahead = ahead && (vx * chaseForward[0] + vz * chaseForward[2]) / h > 0;
            }
            if (ahead && owner[z * n + x] < 0) {
                if (g_verbose && r.holes == 0)
                    printf("    hole at %d,%d (chase %.0f,%.0f)\n", x * kMin, z * kMin, chase.x, chase.z);
                ++r.holes;
            }
        }
    }
    const auto at = [&](int px, int pz) -> const Square* {
        if (px < 0 || pz < 0 || px >= kMap || pz >= kMap)
            return nullptr;
        const int o = owner[(pz / kMin) * n + px / kMin];
        return o < 0 ? nullptr : &squares[o];
    };

    // Each filled cell: the variant for its actual bigger neighbours.
    for (int i = 0; i < count; ++i) {
        const Square& s = squares[r.game + i];
        int edges = 0;
        const Square* across[4] = { at(s.x - 1, s.z), at(s.x + s.size, s.z), at(s.x, s.z - 1), at(s.x, s.z + s.size) };
        const int bits[4] = { kMinusX, kPlusX, kMinusZ, kPlusZ };
        for (int e = 0; e < 4; ++e)
            edges |= across[e] && across[e]->size > s.size ? bits[e] : 0;
        if (s.edges == edges)
            continue;
        if (s.edges == 0 && fillVariant[i] == kVariantBase) {
            ++r.unlearnt;  // not seen in a game patch with every neighbour drawn yet: unstitched
            continue;
        }
        if (g_verbose && r.wrongVariant == 0)
            printf("    variant: cell %d,%d %d drawn with edges %d, expected %d\n", s.x, s.z, s.size, s.edges, edges);
        ++r.wrongVariant;
    }

    // Seams along every edge with a filled cell on either side: sizes more than 2x apart, or a
    // finer side that isn't stitched (or an equal one that is).
    for (const Square& s : squares) {
        const int bits[4] = { kMinusX, kPlusX, kMinusZ, kPlusZ };
        for (int e = 0; e < 4; ++e) {
            std::map<const Square*, bool> seen;
            for (int t = 0; t < s.size; t += kMin) {
                const int px = e == 0 ? s.x - 1 : e == 1 ? s.x + s.size : s.x + t;
                const int pz = e == 2 ? s.z - 1 : e == 3 ? s.z + s.size : s.z + t;
                const Square* o = at(px, pz);
                if (!o || seen.count(o) || (s.game && o->game))
                    continue;
                seen[o] = true;
                ++r.sharedEdges;
                const bool stitched = (s.edges & bits[e]) != 0;
                if (o->size > 2 * s.size || o->size * 2 < s.size)
                    ++r.jumps;
                else if ((o->size > s.size) != stitched)
                    ++r.seams;
            }
        }
    }
    return r;
}

}  // namespace

int main(int argc, char** argv)
{
    g_verbose = argc > 1 && argv[1][0] == '-' && argv[1][1] == 'v';
    printf("terrain fill against a simulated game\n");
    terrain::SetMapSize(kMap);  // as the heightmap uniforms give it
    Result total;
    int scenarios = 0;
    for (int ride = 0; ride < 3; ++ride) {
        const Eye rider{ 400.0 + 600 * ride, 1, 1000.0 - 250 * ride };
        for (int a = 0; a < 360; a += 15) {
            // The game's replay camera orbits the rider, 10 m out and 5 up, looking 26.6 deg down
            // at it; the chase camera sits 4.5 m behind it, looking along +x and 12 deg down.
            const double rad = a * 3.14159265358979 / 180;
            const Eye game{ rider.x + 10 * std::sin(rad), rider.y + 5, rider.z + 10 * std::cos(rad) };
            double gf[3] = { rider.x - game.x, rider.y - game.y, rider.z - game.z };
            const double len = std::sqrt(gf[0] * gf[0] + gf[1] * gf[1] + gf[2] * gf[2]);
            for (double& v : gf)
                v /= len;
            const double heading = ride * 2.1;  // a different riding direction per ride
            const Eye chase{ rider.x - 4.5 * std::cos(heading), rider.y + 1.55, rider.z - 4.5 * std::sin(heading) };
            const double cf[3] = { std::cos(heading) * 0.978, -0.208, std::sin(heading) * 0.978 };
            const Result r = Run(game, gf, chase, cf);
            if (scenarios == 0)
                printf("  first pass: %d game patches, %d filled (%d edge combinations learnt)\n", r.game, r.filled,
                       terrain::LearntEdgeMasks());
            total.game += r.game;
            total.filled += r.filled;
            total.holes += r.holes;
            total.overlaps += r.overlaps;
            total.wrongVariant += r.wrongVariant;
            total.unlearnt += r.unlearnt;
            total.seams += r.seams;
            total.jumps += r.jumps;
            total.sharedEdges += r.sharedEdges;
            ++scenarios;
        }
    }
    printf("  %d passes: %d game patches, %d filled; %d holes, %d overlaps;\n"
           "  %d wrong variants, %d left unstitched while their edge combination was unlearnt;\n"
           "  %d edges with a filled side: %d mismatched seams, %d size jumps over 2x\n",
           scenarios, total.game, total.filled, total.holes, total.overlaps, total.wrongVariant, total.unlearnt,
           total.sharedEdges, total.seams, total.jumps);
    Check(total.filled > 0, "cells are filled in");
    Check(total.holes == 0, "no ground in front of the chase camera is left out");
    Check(total.overlaps == 0, "filled cells never overlap the game's patches or each other");
    Check(total.wrongVariant == 0, "each filled cell is drawn with the variant for its bigger neighbours");
    Check(terrain::LearntEdgeMasks() >= 9, "every stitching combination of a balanced tree is learnt");
    Check(total.jumps == 0, "no filled cell meets one more than twice its size");
    Check(total.seams <= total.sharedEdges / 100, "the fill meets the game's patches and itself almost without seams");
    printf(g_failures ? "%d check(s) failed\n" : "All checks passed\n", g_failures);
    return g_failures ? 1 : 0;
}
