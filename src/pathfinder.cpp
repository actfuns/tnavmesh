#include "tnavmesh/pathfinder.h"
#include "tnavmesh/detour_builder.h"
#include <DetourCommon.h>
#include <iostream>
#include <cmath>
#include <algorithm>

// TMX y-down → Detour y-up
static void tmxToDetour(float tmxX, float tmxY, float mapHeight,
                        float& dx, float& dy, float& dz) {
    dx = tmxX;
    dy = 0;
    dz = mapHeight - tmxY;
}

// Detour y-up → TMX y-down
static void detourToTmx(float dx, float /*dy*/, float dz, float mapHeight,
                        float& tmxX, float& tmxY) {
    tmxX = dx;
    tmxY = mapHeight - dz;
}

// Internal: find path given pre-converted start/end Detour positions.
// Output waypoints in TMX coordinates if mapHeight > 0, else Detour (x,z).
static PathResult findPathInternal(dtNavMeshQuery& query,
                                    const float* startPos, const float* endPos,
                                    float mapHeight, float searchRadius) {
    PathResult result;

    dtQueryFilter filter;
    filter.setIncludeFlags(0xFFFF);
    filter.setExcludeFlags(0);
    for (int i = 0; i < DT_MAX_AREAS; i++)
        filter.setAreaCost(i, 1.0f);

    float halfExtents[3] = { searchRadius, searchRadius * 2, searchRadius };

    dtPolyRef startRef, endRef;
    float startNearest[3], endNearest[3];

    dtStatus status = query.findNearestPoly(startPos, halfExtents, &filter, &startRef, startNearest);
    if (dtStatusFailed(status) || !startRef) {
        char buf[256];
        std::snprintf(buf, sizeof(buf), "Start point (%.0f, %.0f) not on navmesh (search radius %.0f)",
                      startPos[0], startPos[2], searchRadius);
        result.error = buf;
        return result;
    }

    status = query.findNearestPoly(endPos, halfExtents, &filter, &endRef, endNearest);
    if (dtStatusFailed(status) || !endRef) {
        char buf[256];
        std::snprintf(buf, sizeof(buf), "End point (%.0f, %.0f) not on navmesh (search radius %.0f)",
                      endPos[0], endPos[2], searchRadius);
        result.error = buf;
        return result;
    }

    const int MAX_POLYS = 256;
    dtPolyRef path[MAX_POLYS];
    int pathCount = 0;

    status = query.findPath(startRef, endRef, startNearest, endNearest, &filter, path, &pathCount, MAX_POLYS);
    if (dtStatusFailed(status) || pathCount == 0) {
        result.error = "No path found";
        return result;
    }

    const int MAX_STRAIGHT = 256;
    float straightPos[MAX_STRAIGHT * 3];
    unsigned char straightFlags[MAX_STRAIGHT];
    dtPolyRef straightRefs[MAX_STRAIGHT];
    int straightCount = 0;

    status = query.findStraightPath(startNearest, endNearest, path, pathCount,
                                     straightPos, straightFlags, straightRefs,
                                     &straightCount, MAX_STRAIGHT, DT_STRAIGHTPATH_ALL_CROSSINGS);
    if (dtStatusFailed(status) || straightCount == 0) {
        result.error = "No straight path found";
        return result;
    }

    result.found = true;
    for (int i = 0; i < straightCount; i++) {
        if (mapHeight > 0) {
            float tx, ty;
            detourToTmx(straightPos[i*3], straightPos[i*3+1], straightPos[i*3+2],
                        mapHeight, tx, ty);
            result.waypoints.push_back(tx);
            result.waypoints.push_back(ty);
        } else {
            result.waypoints.push_back(straightPos[i*3]);
            result.waypoints.push_back(straightPos[i*3+2]);
        }
    }

    result.totalLength = 0;
    for (size_t i = 1; i < result.waypoints.size() / 2; i++) {
        float dx = result.waypoints[i*2] - result.waypoints[(i-1)*2];
        float dy = result.waypoints[i*2+1] - result.waypoints[(i-1)*2+1];
        result.totalLength += std::sqrt(dx*dx + dy*dy);
    }

    return result;
}

PathResult findPath(dtNavMesh* navMesh,
                    float startX, float startY,
                    float endX, float endY,
                    float mapHeight,
                    float searchRadius) {
    if (!navMesh) {
        PathResult r; r.error = "navMesh is null"; return r;
    }

    dtNavMeshQuery query;
    if (dtStatusFailed(query.init(navMesh, 2048))) {
        PathResult r; r.error = "dtNavMeshQuery::init failed"; return r;
    }

    float startPos[3], endPos[3];
    tmxToDetour(startX, startY, mapHeight, startPos[0], startPos[1], startPos[2]);
    tmxToDetour(endX, endY, mapHeight, endPos[0], endPos[1], endPos[2]);

    return findPathInternal(query, startPos, endPos, mapHeight, searchRadius);
}

PathResult findPathDetour(dtNavMesh* navMesh,
                           float startX, float startZ,
                           float endX, float endZ,
                           float searchRadius) {
    if (!navMesh) {
        PathResult r; r.error = "navMesh is null"; return r;
    }

    dtNavMeshQuery query;
    if (dtStatusFailed(query.init(navMesh, 2048))) {
        PathResult r; r.error = "dtNavMeshQuery::init failed"; return r;
    }

    float startPos[3] = { startX, 0, startZ };
    float endPos[3] = { endX, 0, endZ };

    return findPathInternal(query, startPos, endPos, 0, searchRadius);
}

dtNavMesh* loadDetourNavMesh(const char* path) {
    unsigned char* data = nullptr;
    int dataSize = 0;

    if (!loadNavMesh(path, data, dataSize))
        return nullptr;

    dtNavMesh* navMesh = dtAllocNavMesh();
    if (!navMesh) {
        delete[] data;
        std::cerr << "Error: dtAllocNavMesh failed.\n";
        return nullptr;
    }

    dtStatus status = navMesh->init(data, dataSize, DT_TILE_FREE_DATA);
    if (dtStatusFailed(status)) {
        dtFreeNavMesh(navMesh);
        delete[] data;  // init failed — data not owned by navmesh
        std::cerr << "Error: dtNavMesh::init failed.\n";
        return nullptr;
    }

    return navMesh;
}

void generateAutoPoints(const rcPolyMesh* mesh,
                        float mapHeight,
                        float& sx, float& sy,
                        float& ex, float& ey) {
    // Pick two farthest-apart vertices from the poly mesh
    if (!mesh || mesh->nverts < 2) {
        sx = sy = ex = ey = 0;
        std::cerr << "Warning: not enough vertices for auto point generation.\n";
        return;
    }

    float bestDist = -1;
    for (int i = 0; i < mesh->nverts && i < 50; i++) {
        for (int j = i + 1; j < mesh->nverts && j < 50; j++) {
            float dx = (mesh->verts[i*3] - mesh->verts[j*3]) * mesh->cs;
            float dz = (mesh->verts[i*3+2] - mesh->verts[j*3+2]) * mesh->cs;
            float dist = dx*dx + dz*dz;
            if (dist > bestDist) {
                bestDist = dist;
                sx = mesh->verts[i*3] * mesh->cs + mesh->bmin[0];
                sy = mapHeight - (mesh->verts[i*3+2] * mesh->cs + mesh->bmin[2]);
                ex = mesh->verts[j*3] * mesh->cs + mesh->bmin[0];
                ey = mapHeight - (mesh->verts[j*3+2] * mesh->cs + mesh->bmin[2]);
            }
        }
    }
}
