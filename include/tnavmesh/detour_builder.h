#ifndef DETOUR_BUILDER_H
#define DETOUR_BUILDER_H

#include "navmesh_config.h"
#include <DetourNavMesh.h>
#include <Recast.h>

// Build a dtNavMesh from rcPolyMesh using Detour's dtCreateNavMeshData.
// Returns nullptr on failure.
[[nodiscard]] dtNavMesh* buildDetourNavMesh(rcPolyMesh* mesh, const NavmeshConfig& config);

// Get raw tile data from a built dtNavMesh (single-tile) for saving.
// Returns pointer to internal data and its size. The data remains owned by
// the navmesh — do not free it. Returns false if no tile data available.
[[nodiscard]] bool getNavMeshData(const dtNavMesh* navMesh, const unsigned char*& data, int& dataSize);

// .bin file format:
//   [4 bytes] dataSize (little-endian uint32)
//   [dataSize bytes] raw dtCreateNavMeshData output
//
// saveNavMesh writes this format. loadNavMesh reads it and allocates data
// (caller must free[]).

[[nodiscard]] bool saveNavMesh(const char* path, const unsigned char* data, int dataSize);
[[nodiscard]] bool loadNavMesh(const char* path, unsigned char*& data, int& dataSize);

#endif
