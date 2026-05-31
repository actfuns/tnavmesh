#ifndef RECAST_BUILDER_H
#define RECAST_BUILDER_H

#include "tmx_parser.h"
#include "navmesh_config.h"
#include <Recast.h>
#include <vector>

// Build Recast navmesh from merged obstacle regions.
// Input regions in TMX coords (y-down), this function handles coordinate conversion.
rcPolyMesh* buildNavMesh(const MapInfo& mapInfo,
                          const std::vector<MergedRegion>& regions,
                          const NavmeshConfig& config);

#endif
