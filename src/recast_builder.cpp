#include "tnavmesh/recast_builder.h"
#include <iostream>
#include <cmath>
#include <cstring>
#include <functional>
#include <vector>
#include <memory>

#ifdef HAVE_GEOS
#include <geos_c.h>
#include <poly2tri.h>
#endif

// TMX y-down → Recast z-up
static float tmxToRecastZ(float tmxY, float mapH) {
    return mapH - tmxY;
}

#ifdef HAVE_GEOS
// Extract ring points from GEOS geometry
static std::vector<float> geosRingPoints(const GEOSContextHandle_t& ctx,
                                          const GEOSGeometry* ring) {
    std::vector<float> pts;
    const GEOSCoordSequence* seq = GEOSGeom_getCoordSeq_r(ctx, ring);
    if (!seq) return pts;
    unsigned int nPts = 0;
    GEOSCoordSeq_getSize_r(ctx, seq, &nPts);
    for (unsigned int i = 0; i < nPts - 1; i++) {
        double x, y;
        GEOSCoordSeq_getXY_r(ctx, seq, i, &x, &y);
        pts.push_back((float)x);
        pts.push_back((float)y);
    }
    return pts;
}
#endif

// Remove consecutive duplicate points
static std::vector<float> dedupPoints(const std::vector<float>& pts) {
    if (pts.size() < 4) return pts;
    std::vector<float> out;
    for (size_t i = 0; i < pts.size() / 2; i++) {
        size_t next = (i + 1) % (pts.size() / 2);
        float dx = pts[i*2] - pts[next*2];
        float dy = pts[i*2+1] - pts[next*2+1];
        if (dx*dx + dy*dy < 0.001f) continue;
        out.push_back(pts[i*2]);
        out.push_back(pts[i*2+1]);
    }
    if (out.size() >= 6) return out;
    return pts;
}

// Run the Recast pipeline
static rcPolyMesh* runRecast(const rcConfig& config,
                              const std::vector<float>& verts,
                              const std::vector<int>& tris) {
    rcContext ctx;
    rcHeightfield* hf = rcAllocHeightfield();
    if (!hf || !rcCreateHeightfield(&ctx, *hf, config.width, config.height,
                                     config.bmin, config.bmax, config.cs, config.ch)) {
        if (hf) rcFreeHeightField(hf);
        return nullptr;
    }

    std::vector<unsigned char> triAreas(tris.size() / 3, RC_WALKABLE_AREA);
    if (!rcRasterizeTriangles(&ctx, verts.data(), (int)verts.size() / 3,
                              tris.data(), triAreas.data(), (int)triAreas.size(), *hf, 1)) {
        rcFreeHeightField(hf);
        return nullptr;
    }

    rcFilterLowHangingWalkableObstacles(&ctx, config.walkableClimb, *hf);
    rcFilterLedgeSpans(&ctx, config.walkableHeight, config.walkableClimb, *hf);
    rcFilterWalkableLowHeightSpans(&ctx, config.walkableHeight, *hf);

    rcCompactHeightfield* chf = rcAllocCompactHeightfield();
    if (!chf) { rcFreeHeightField(hf); return nullptr; }
    if (!rcBuildCompactHeightfield(&ctx, config.walkableHeight, config.walkableClimb, *hf, *chf)) {
        rcFreeHeightField(hf); rcFreeCompactHeightfield(chf); return nullptr;
    }
    rcFreeHeightField(hf);

    rcErodeWalkableArea(&ctx, config.walkableRadius, *chf);
    if (!rcBuildDistanceField(&ctx, *chf)) {
        std::cerr << "Error: rcBuildDistanceField failed.\n";
        rcFreeCompactHeightfield(chf);
        return nullptr;
    }
    if (!rcBuildRegions(&ctx, *chf, 0, config.minRegionArea, config.mergeRegionArea)) {
        std::cerr << "Error: rcBuildRegions failed.\n";
        rcFreeCompactHeightfield(chf);
        return nullptr;
    }

    rcContourSet* cset = rcAllocContourSet();
    if (!cset) { rcFreeCompactHeightfield(chf); return nullptr; }
    if (!rcBuildContours(&ctx, *chf, config.maxSimplificationError, config.maxEdgeLen, *cset)) {
        std::cerr << "Error: rcBuildContours failed.\n";
        rcFreeCompactHeightfield(chf);
        rcFreeContourSet(cset);
        return nullptr;
    }

    rcPolyMesh* pmesh = rcAllocPolyMesh();
    if (!pmesh) { rcFreeCompactHeightfield(chf); rcFreeContourSet(cset); return nullptr; }
    if (!rcBuildPolyMesh(&ctx, *cset, config.maxVertsPerPoly, *pmesh)) {
        std::cerr << "Error: rcBuildPolyMesh failed.\n";
        rcFreeCompactHeightfield(chf);
        rcFreeContourSet(cset);
        rcFreePolyMesh(pmesh);
        return nullptr;
    }

    rcFreeCompactHeightfield(chf); rcFreeContourSet(cset);
    return pmesh;
}

#ifdef HAVE_GEOS

rcPolyMesh* buildNavMesh(const MapInfo& mapInfo,
                          const std::vector<MergedRegion>& regions,
                          const NavmeshConfig& config) {
    float mapW = static_cast<float>(mapInfo.width) * mapInfo.tileWidth;
    float mapH = static_cast<float>(mapInfo.height) * mapInfo.tileHeight;
    rcConfig cfg = config.toRcConfig(mapW, mapH);

    if (regions.empty()) {
        std::vector<float> verts = { 0,0,mapH, mapW,0,mapH, mapW,0,0, 0,0,0 };
        std::vector<int> tris = { 0,1,2, 0,2,3 };
        return runRecast(cfg, verts, tris);
    }

    // Step 1: GEOS Difference (map - obstacles)
    GEOSContextHandle_t geosCtx = GEOS_init_r();
    GEOSContext_setNoticeMessageHandler_r(geosCtx, [](const char*, void*){}, nullptr);

    std::vector<GEOSGeometry*> obsGeoms;
    for (const auto& region : regions) {
        auto makeRing = [&](const Obstacle& obs) -> GEOSGeometry* {
            size_t n = obs.points.size() / 2;
            if (n < 3) return nullptr;
            auto nUint = static_cast<unsigned int>(n);
            GEOSCoordSequence* seq = GEOSCoordSeq_create_r(geosCtx, nUint + 1, 2);
            if (!seq) return nullptr;
            for (unsigned int ui = 0; ui < nUint; ui++)
                GEOSCoordSeq_setXY_r(geosCtx, seq, ui, obs.points[ui*2], obs.points[ui*2+1]);
            GEOSCoordSeq_setXY_r(geosCtx, seq, nUint, obs.points[0], obs.points[1]);
            return GEOSGeom_createLinearRing_r(geosCtx, seq);
        };
        GEOSGeometry* exteriorRing = makeRing(region.exterior);
        if (!exteriorRing) continue;
        std::vector<GEOSGeometry*> holeRings;
        for (const auto& h : region.holes) {
            GEOSGeometry* hr = makeRing(h);
            if (hr) holeRings.push_back(hr);
        }
        GEOSGeometry* poly = GEOSGeom_createPolygon_r(geosCtx, exteriorRing,
            holeRings.empty() ? nullptr : holeRings.data(), (int)holeRings.size());
        if (poly) obsGeoms.push_back(poly);
    }

    // Map outer polygon
    GEOSCoordSequence* mapSeq = GEOSCoordSeq_create_r(geosCtx, 5, 2);
    GEOSCoordSeq_setXY_r(geosCtx, mapSeq, 0, 0, 0);
    GEOSCoordSeq_setXY_r(geosCtx, mapSeq, 1, mapW, 0);
    GEOSCoordSeq_setXY_r(geosCtx, mapSeq, 2, mapW, mapH);
    GEOSCoordSeq_setXY_r(geosCtx, mapSeq, 3, 0, mapH);
    GEOSCoordSeq_setXY_r(geosCtx, mapSeq, 4, 0, 0);
    GEOSGeometry* mapRing = GEOSGeom_createLinearRing_r(geosCtx, mapSeq);
    if (!mapRing) {
        for (auto* g : obsGeoms) GEOSGeom_destroy_r(geosCtx, g);
        GEOS_finish_r(geosCtx); return runRecast(cfg, {}, {});
    }
    GEOSGeometry* mapPoly = GEOSGeom_createPolygon_r(geosCtx, mapRing, nullptr, 0);
    if (!mapPoly) {
        GEOSGeom_destroy_r(geosCtx, mapRing);
        for (auto* g : obsGeoms) GEOSGeom_destroy_r(geosCtx, g);
        GEOS_finish_r(geosCtx); return runRecast(cfg, {}, {});
    }

    // Union obstacles
    GEOSGeometry* obsUnion = nullptr;
    if (obsGeoms.empty()) {
        obsUnion = GEOSGeom_createEmptyPolygon_r(geosCtx);
    } else if (obsGeoms.size() == 1) {
        obsUnion = GEOSGeom_clone_r(geosCtx, obsGeoms[0]);
        GEOSGeom_destroy_r(geosCtx, obsGeoms[0]);
    } else {
        GEOSGeometry* coll = GEOSGeom_createCollection_r(geosCtx, GEOS_GEOMETRYCOLLECTION,
                                                          obsGeoms.data(), obsGeoms.size());
        // createCollection takes ownership of obsGeoms elements.
        // After destroying the collection, the child geoms are also freed.
        obsUnion = GEOSUnaryUnion_r(geosCtx, coll);
        GEOSGeom_destroy_r(geosCtx, coll);
        // Do NOT destroy obsGeoms elements — collection already freed them.
    }
    obsGeoms.clear();

    GEOSGeometry* walkableGeo = GEOSDifference_r(geosCtx, mapPoly, obsUnion);
    GEOSGeom_destroy_r(geosCtx, mapPoly);
    GEOSGeom_destroy_r(geosCtx, obsUnion);

    if (!walkableGeo || GEOSisEmpty_r(geosCtx, walkableGeo) == 1) {
        if (walkableGeo) GEOSGeom_destroy_r(geosCtx, walkableGeo);
        GEOS_finish_r(geosCtx);
        return runRecast(cfg, {}, {});
    }

    // Step 2: Triangulate walkable polygons with poly2tri
    std::vector<float> verts;
    std::vector<int> tris;

    std::function<void(const GEOSGeometry*)> extractPolygons;
    extractPolygons = [&](const GEOSGeometry* geo) {
        if (GEOSGeomTypeId_r(geosCtx, geo) != GEOS_POLYGON) return;

        const GEOSGeometry* extRing = GEOSGetExteriorRing_r(geosCtx, geo);
        if (!extRing) return;
        auto extTmx = dedupPoints(geosRingPoints(geosCtx, extRing));
        if (extTmx.size() < 6) return;

        int nHoles = GEOSGetNumInteriorRings_r(geosCtx, geo);
        std::vector<std::vector<float>> holeTmxs;
        for (int i = 0; i < nHoles; i++) {
            const GEOSGeometry* hRing = GEOSGetInteriorRingN_r(geosCtx, geo, i);
            if (hRing) {
                auto hTmx = dedupPoints(geosRingPoints(geosCtx, hRing));
                if (hTmx.size() >= 6) holeTmxs.push_back(std::move(hTmx));
            }
        }

        // poly2tri CDT
        std::vector<p2t::Point*> polyline;
        std::vector<std::unique_ptr<p2t::Point>> ownedPts;
        auto makePt = [&](float x, float y) {
            ownedPts.push_back(std::make_unique<p2t::Point>(x, y));
            return ownedPts.back().get();
        };
        for (size_t i = 0; i < extTmx.size() / 2; i++)
            polyline.push_back(makePt(extTmx[i*2], extTmx[i*2+1]));

        p2t::CDT cdt(polyline);
        for (auto& htmx : holeTmxs) {
            std::vector<p2t::Point*> holePts;
            for (size_t i = 0; i < htmx.size() / 2; i++)
                holePts.push_back(makePt(htmx[i*2], htmx[i*2+1]));
            cdt.AddHole(holePts);
        }
        try {
            cdt.Triangulate();
        } catch (const std::exception& e) {
            std::cerr << "Warning: poly2tri triangulation failed for a region: " << e.what() << std::endl;
            return;
        }
        auto triangles = cdt.GetTriangles();

        int baseVert = (int)verts.size() / 3;
        for (auto* tri : triangles) {
            for (int i = 0; i < 3; i++) {
                p2t::Point* p = tri->GetPoint(i);
                verts.push_back((float)p->x);
                verts.push_back(0);
                verts.push_back(tmxToRecastZ((float)p->y, mapH));
            }
            tris.push_back(baseVert);
            tris.push_back(baseVert + 1);
            tris.push_back(baseVert + 2);
            baseVert += 3;
        }
    };

    // Walk walkable geometry
    std::function<void(const GEOSGeometry*)> walkGeos;
    walkGeos = [&](const GEOSGeometry* geo) {
        int type = GEOSGeomTypeId_r(geosCtx, geo);
        if (type == GEOS_POLYGON) {
            extractPolygons(geo);
        } else if (type == GEOS_MULTIPOLYGON || type == GEOS_GEOMETRYCOLLECTION) {
            int n = GEOSGetNumGeometries_r(geosCtx, geo);
            for (int i = 0; i < n; i++)
                walkGeos(GEOSGetGeometryN_r(geosCtx, geo, i));
        }
    };

    walkGeos(walkableGeo);
    GEOSGeom_destroy_r(geosCtx, walkableGeo);
    GEOS_finish_r(geosCtx);

    if (tris.empty()) return runRecast(cfg, {}, {});

    return runRecast(cfg, verts, tris);
}

#else // !HAVE_GEOS — stub: build flat navmesh for the whole map

rcPolyMesh* buildNavMesh(const MapInfo& mapInfo,
                          const std::vector<MergedRegion>& /*regions*/,
                          const NavmeshConfig& config) {
    float mapW = static_cast<float>(mapInfo.width) * mapInfo.tileWidth;
    float mapH = static_cast<float>(mapInfo.height) * mapInfo.tileHeight;
    rcConfig cfg = config.toRcConfig(mapW, mapH);

    std::cerr << "Warning: GEOS not available — building navmesh for full map area.\n";

    // Full map as a flat quad
    std::vector<float> verts = { 0,0,mapH, mapW,0,mapH, mapW,0,0, 0,0,0 };
    std::vector<int> tris = { 0,1,2, 0,2,3 };
    return runRecast(cfg, verts, tris);
}

#endif
