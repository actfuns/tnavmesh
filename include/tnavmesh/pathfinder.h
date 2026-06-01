#ifndef PATHFINDER_H
#define PATHFINDER_H

#include "navmesh_config.h"
#include "tmx_parser.h"
#include <DetourNavMesh.h>
#include <DetourNavMeshQuery.h>
#include <string>
#include <vector>

struct PathResult {
    bool found = false;
    std::vector<float> waypoints;  // TMX x,y pairs
    float totalLength = 0;
    std::string error;
};

// Find path on an already-loaded dtNavMesh.
// startX/startY and endX/endY are in TMX coordinates (y-down).
// mapHeight is used for TMX-to-Detour coordinate conversion.
[[nodiscard]] PathResult findPath(dtNavMesh* navMesh,
                                 float startX, float startY,
                                 float endX, float endY,
                                 float mapHeight,
                                 float searchRadius);

// Find path using coordinates already in Detour space (x, z).
// No TMX-to-Detour conversion is performed.
[[nodiscard]] PathResult findPathDetour(dtNavMesh* navMesh,
                                        float startX, float startZ,
                                        float endX, float endZ,
                                        float searchRadius);

// Load a .bin file and create a dtNavMesh from it.
// Returns nullptr on failure.
[[nodiscard]] dtNavMesh* loadDetourNavMesh(const char* path);

// Auto-generate start/end points from the navmesh (for --auto mode).
// Picks two points as far apart as possible among the tile's vertices.
void generateAutoPoints(const rcPolyMesh* mesh,
                        float mapHeight,
                        float& sx, float& sy,
                        float& ex, float& ey);

#endif
