#include "tnavmesh/geos_utils.h"
#include <iostream>

#ifdef HAVE_GEOS
#include <geos_c.h>
#include <vector>

// GEOS context (thread-local)
static GEOSContextHandle_t initGEOS() {
    GEOSContextHandle_t ctx = GEOS_init_r();
    // Suppress GEOS messages by setting no-op notice handlers
    GEOSContext_setNoticeMessageHandler_r(ctx, [](const char*, void*) {}, nullptr);
    return ctx;
}

static void destroyGEOS(GEOSContextHandle_t ctx) {
    GEOS_finish_r(ctx);
}

// Convert Obstacle (TMX y-down) to GEOS Polygon (y-up for GEOS, but we keep as-is
// since GEOS doesn't care about coordinate orientation for topological ops)
static GEOSGeometry* obstacleToGeos(GEOSContextHandle_t ctx, const Obstacle& obs) {
    unsigned int n = static_cast<unsigned int>(obs.points.size() / 2);
    GEOSCoordSequence* seq = GEOSCoordSeq_create_r(ctx, n + 1, 2);
    if (!seq) return nullptr;

    for (unsigned int i = 0; i < n; i++) {
        GEOSCoordSeq_setXY_r(ctx, seq, i, obs.points[i * 2], obs.points[i * 2 + 1]);
    }
    // Close the ring
    GEOSCoordSeq_setXY_r(ctx, seq, n, obs.points[0], obs.points[1]);

    GEOSGeometry* ring = GEOSGeom_createLinearRing_r(ctx, seq);
    if (!ring) return nullptr;

    return GEOSGeom_createPolygon_r(ctx, ring, nullptr, 0);
}

// Extract ring points to Obstacle (from an exterior or interior ring)
static Obstacle ringToObstacle(GEOSContextHandle_t ctx, const GEOSGeometry* ring) {
    const GEOSCoordSequence* seq = GEOSGeom_getCoordSeq_r(ctx, ring);
    if (!seq) return {};
    unsigned int nPts = 0;
    GEOSCoordSeq_getSize_r(ctx, seq, &nPts);
    Obstacle o;
    for (unsigned int i = 0; i < nPts - 1; i++) {
        double x, y;
        GEOSCoordSeq_getXY_r(ctx, seq, i, &x, &y);
        o.points.push_back((float)x);
        o.points.push_back((float)y);
    }
    return o;
}

// Extract a MergedRegion from a GEOS Polygon geometry
static MergedRegion polygonToMergedRegion(GEOSContextHandle_t ctx, const GEOSGeometry* geom) {
    MergedRegion region;
    int type = GEOSGeomTypeId_r(ctx, geom);
    if (type != GEOS_POLYGON) return region;

    // Exterior ring
    const GEOSGeometry* exteriorRing = GEOSGetExteriorRing_r(ctx, geom);
    if (!exteriorRing) return region;
    region.exterior = ringToObstacle(ctx, exteriorRing);
    if (region.exterior.points.empty()) return region;

    // Interior rings (holes)
    int nHoles = GEOSGetNumInteriorRings_r(ctx, geom);
    for (int i = 0; i < nHoles; i++) {
        const GEOSGeometry* holeRing = GEOSGetInteriorRingN_r(ctx, geom, i);
        if (holeRing) {
            Obstacle hole = ringToObstacle(ctx, holeRing);
            if (!hole.points.empty()) {
                region.holes.push_back(std::move(hole));
            }
        }
    }

    return region;
}

// Create a map boundary rectangle in TMX coords
static GEOSGeometry* createMapBoundary(GEOSContextHandle_t ctx,
                                         float mapWidth, float mapHeight) {
    GEOSCoordSequence* seq = GEOSCoordSeq_create_r(ctx, 5, 2);
    GEOSCoordSeq_setXY_r(ctx, seq, 0, 0, 0);
    GEOSCoordSeq_setXY_r(ctx, seq, 1, mapWidth, 0);
    GEOSCoordSeq_setXY_r(ctx, seq, 2, mapWidth, mapHeight);
    GEOSCoordSeq_setXY_r(ctx, seq, 3, 0, mapHeight);
    GEOSCoordSeq_setXY_r(ctx, seq, 4, 0, 0);

    GEOSGeometry* ring = GEOSGeom_createLinearRing_r(ctx, seq);
    return GEOSGeom_createPolygon_r(ctx, ring, nullptr, 0);
}

std::vector<MergedRegion> mergeObstacles(const std::vector<Obstacle>& obstacles,
                                          float mapWidth, float mapHeight) {
    if (obstacles.empty()) return {};

    GEOSContextHandle_t ctx = initGEOS();

    // Create GEOS geometries for each obstacle
    std::vector<GEOSGeometry*> geoms;
    for (const auto& obs : obstacles) {
        GEOSGeometry* g = obstacleToGeos(ctx, obs);
        if (g) {
            // Check if geometry is valid (GEOS can handle some invalidity but let's skip bad ones)
            if (GEOSisValid_r(ctx, g) != 1) {
                // Try to fix with buffer(0)
                GEOSGeometry* fixed = GEOSBuffer_r(ctx, g, 0, 1);
                GEOSGeom_destroy_r(ctx, g);
                if (fixed && GEOSisValid_r(ctx, fixed) == 1) {
                    g = fixed;
                } else {
                    if (fixed) GEOSGeom_destroy_r(ctx, fixed);
                    continue;
                }
            }
            geoms.push_back(g);
        }
    }

    if (geoms.empty()) {
        destroyGEOS(ctx);
        return {};
    }

    // Build a GeometryCollection for unary union
    GEOSGeometry* merged = nullptr;
    auto nGeoms = static_cast<unsigned int>(geoms.size());
    if (geoms.size() == 1) {
        merged = GEOSGeom_clone_r(ctx, geoms[0]);
        for (auto* g : geoms) GEOSGeom_destroy_r(ctx, g);
    } else {
        GEOSGeometry* collection = GEOSGeom_createCollection_r(ctx, GEOS_GEOMETRYCOLLECTION, geoms.data(), nGeoms);
        if (!collection) {
            for (auto* g : geoms) GEOSGeom_destroy_r(ctx, g);
            destroyGEOS(ctx);
            return {};
        }
        geoms.clear();
        merged = GEOSUnaryUnion_r(ctx, collection);
        GEOSGeom_destroy_r(ctx, collection);
    }

    if (!merged) {
        destroyGEOS(ctx);
        return {};
    }

    // Clip merged obstacles to map boundary
    GEOSGeometry* boundary = createMapBoundary(ctx, mapWidth, mapHeight);
    GEOSGeometry* clipped = GEOSIntersection_r(ctx, merged, boundary);
    GEOSGeom_destroy_r(ctx, merged);
    GEOSGeom_destroy_r(ctx, boundary);

    if (!clipped) {
        destroyGEOS(ctx);
        return {};
    }

    // Extract regions (polygons with possible holes)
    std::vector<MergedRegion> result;
    int geomType = GEOSGeomTypeId_r(ctx, clipped);

    auto extractPolygon = [&](const GEOSGeometry* geom) {
        if (GEOSisEmpty_r(ctx, geom) == 1) return;
        MergedRegion region = polygonToMergedRegion(ctx, geom);
        if (region.exterior.points.size() >= 6) { // at least 3 points
            result.push_back(std::move(region));
        }
    };

    if (geomType == GEOS_POLYGON) {
        extractPolygon(clipped);
    } else if (geomType == GEOS_MULTIPOLYGON || geomType == GEOS_GEOMETRYCOLLECTION) {
        int nGeoms = GEOSGetNumGeometries_r(ctx, clipped);
        for (int i = 0; i < nGeoms; i++) {
            const GEOSGeometry* part = GEOSGetGeometryN_r(ctx, clipped, i);
            if (part) extractPolygon(part);
        }
    }

    GEOSGeom_destroy_r(ctx, clipped);
    destroyGEOS(ctx);

    int totalHoles = 0;
    for (const auto& r : result) totalHoles += (int)r.holes.size();
    std::cout << "GEOS merged " << obstacles.size() << " obstacles into "
              << result.size() << " regions (" << totalHoles << " holes)" << std::endl;
    return result;
}

std::vector<Obstacle> computeOverlaps(const std::vector<Obstacle>& obstacles) {
    if (obstacles.size() < 2) return {};

    GEOSContextHandle_t ctx = initGEOS();

    // Convert all obstacles to GEOS geometries
    std::vector<GEOSGeometry*> geoms;
    for (const auto& obs : obstacles) {
        GEOSGeometry* g = obstacleToGeos(ctx, obs);
        if (g && GEOSisValid_r(ctx, g) == 1) {
            geoms.push_back(g);
        } else if (g) {
            GEOSGeom_destroy_r(ctx, g);
        }
    }

    if (geoms.size() < 2) {
        for (auto* g : geoms) GEOSGeom_destroy_r(ctx, g);
        destroyGEOS(ctx);
        return {};
    }

    // For each pair, compute intersection
    std::vector<GEOSGeometry*> overlaps;
    for (size_t i = 0; i < geoms.size(); i++) {
        for (size_t j = i + 1; j < geoms.size(); j++) {
            // Quick bounding box test first
            if (!GEOSIntersects_r(ctx, geoms[i], geoms[j]))
                continue;

            GEOSGeometry* inter = GEOSIntersection_r(ctx, geoms[i], geoms[j]);
            if (!inter || GEOSisEmpty_r(ctx, inter) == 1) {
                if (inter) GEOSGeom_destroy_r(ctx, inter);
                continue;
            }
            overlaps.push_back(inter);
        }
    }

    for (auto* g : geoms) GEOSGeom_destroy_r(ctx, g);

    // Union all overlaps into a single multi-geometry, then collect
    std::vector<Obstacle> result;
    if (overlaps.empty()) {
        destroyGEOS(ctx);
        return result;
    }

    GEOSGeometry* unioned = nullptr;
    auto nOverlaps = static_cast<unsigned int>(overlaps.size());
    if (overlaps.size() == 1) {
        unioned = GEOSGeom_clone_r(ctx, overlaps[0]);
    } else {
        GEOSGeometry* coll = GEOSGeom_createCollection_r(ctx, GEOS_GEOMETRYCOLLECTION,
                                                          overlaps.data(), nOverlaps);
        overlaps.clear();
        if (coll) {
            unioned = GEOSUnaryUnion_r(ctx, coll);
            GEOSGeom_destroy_r(ctx, coll);
        }
    }

    for (auto* g : overlaps) GEOSGeom_destroy_r(ctx, g);

    if (!unioned) {
        destroyGEOS(ctx);
        return result;
    }

    // Extract polygon exteriors
    auto extract = [&](const GEOSGeometry* g) {
        if (GEOSGeomTypeId_r(ctx, g) != GEOS_POLYGON) return;
        const GEOSGeometry* extRing = GEOSGetExteriorRing_r(ctx, g);
        if (extRing) {
            Obstacle o = ringToObstacle(ctx, extRing);
            if (!o.points.empty()) result.push_back(std::move(o));
        }
    };

    int type = GEOSGeomTypeId_r(ctx, unioned);
    if (type == GEOS_POLYGON) {
        extract(unioned);
    } else if (type == GEOS_MULTIPOLYGON || type == GEOS_GEOMETRYCOLLECTION) {
        int n = GEOSGetNumGeometries_r(ctx, unioned);
        for (int i = 0; i < n; i++)
            extract(GEOSGetGeometryN_r(ctx, unioned, i));
    }

    GEOSGeom_destroy_r(ctx, unioned);
    destroyGEOS(ctx);
    return result;
}

#else // !HAVE_GEOS — stub implementations

std::vector<MergedRegion> mergeObstacles(const std::vector<Obstacle>& obstacles,
                                          float /*mapWidth*/, float /*mapHeight*/) {
    // Without GEOS, just return each obstacle as its own region (no actual merging)
    std::cerr << "Warning: GEOS not available — obstacle merging disabled.\n";
    std::vector<MergedRegion> result;
    for (const auto& obs : obstacles) {
        MergedRegion r;
        r.exterior = obs;
        result.push_back(std::move(r));
    }
    return result;
}

std::vector<Obstacle> computeOverlaps(const std::vector<Obstacle>& /*obstacles*/) {
    std::cerr << "Warning: GEOS not available — overlap computation disabled.\n";
    return {};
}

#endif
