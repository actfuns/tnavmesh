#ifndef GEOS_UTILS_H
#define GEOS_UTILS_H

#include "tmx_parser.h"
#include <vector>

// Merge overlapping/touching obstacles using GEOS, clipped to map bounds.
// Input and output obstacles in TMX coordinate space (y-down).
// Returns merged regions with possible holes.
[[nodiscard]] std::vector<MergedRegion> mergeObstacles(const std::vector<Obstacle>& obstacles,
                                                       float mapWidth, float mapHeight);

// Compute overlap regions between obstacles.
// Returns a set of polygons where two or more obstacles intersect.
// Each polygon is returned as an Obstacle (exterior points only).
// Empty if no overlaps.
[[nodiscard]] std::vector<Obstacle> computeOverlaps(const std::vector<Obstacle>& obstacles);

#endif
