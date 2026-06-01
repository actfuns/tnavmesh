#ifndef NAVMESH_CONFIG_H
#define NAVMESH_CONFIG_H

#include "tmx_parser.h"
#include <Recast.h>
#include <cstring>
#include <algorithm>
#include <cmath>

enum class Quality {
    Low,
    Normal,
    High
};

struct NavmeshConfig {
    // 0 = auto-calculate from map info
    float cs = 0;
    float ch = 0;
    int walkableHeight = 0;
    int walkableRadius = 0;
    int walkableClimb = 0;
    float walkableSlopeAngle = 45.0f;
    int maxEdgeLen = 0;
    float maxSimplificationError = 0;
    int minRegionArea = 0;
    int mergeRegionArea = 0;
    int maxVertsPerPoly = 6;
    float detailSampleDist = 6.0f;
    float detailSampleMaxError = 1.0f;
    Quality quality = Quality::Normal;

    void autoCalc(const MapInfo& mapInfo) {
        float tileW = mapInfo.tileWidth;
        float tileH = mapInfo.tileHeight;
        float tileSize = std::min(tileW, tileH);

        // Agent (pixels)
        if (walkableRadius <= 0) walkableRadius = static_cast<int>(tileSize * 0.25f);
        if (walkableHeight <= 0) walkableHeight = static_cast<int>(tileSize * 2.0f);
        if (walkableClimb <= 0) walkableClimb = static_cast<int>(tileSize * 0.8f);

        // Voxel resolution
        if (cs <= 0) {
            float csFactor;
            switch (quality) {
                case Quality::Low:    csFactor = 0.4f;  break;
                case Quality::High:   csFactor = 0.1f;  break;
                default:              csFactor = 0.15f; break;
            }
            cs = tileSize * csFactor;
        }
        if (ch <= 0) ch = cs * 0.5f;

        // Convert agent values from pixels to voxels for rcConfig
        if (walkableRadius > 0) walkableRadius = std::max(1, static_cast<int>(walkableRadius / cs));
        if (walkableHeight > 0) walkableHeight = std::max(1, static_cast<int>(walkableHeight / cs));
        if (walkableClimb > 0) walkableClimb = std::max(1, static_cast<int>(walkableClimb / cs));

        // Region
        float mapW = static_cast<float>(mapInfo.width) * tileW;
        float mapH = static_cast<float>(mapInfo.height) * tileH;
        float mapArea = mapW * mapH;

        if (minRegionArea <= 0)
            minRegionArea = static_cast<int>(mapArea * 0.0005f / (cs * cs));
        if (mergeRegionArea <= 0)
            mergeRegionArea = minRegionArea * 8;
        if (maxSimplificationError <= 0)
            maxSimplificationError = 2.0f;
        if (maxEdgeLen <= 0)
            maxEdgeLen = static_cast<int>(tileSize * 2.0f / cs);

        // Sanity: minimum region area of at least 8 voxels
        if (minRegionArea < 8) minRegionArea = 8;
        if (mergeRegionArea < minRegionArea) mergeRegionArea = minRegionArea;

        // Sanity: at least 1 voxel for walkableHeight/Radius/Climb
        if (walkableHeight < 1) walkableHeight = 1;
        if (walkableRadius < 1) walkableRadius = 1;
        if (walkableClimb < 1) walkableClimb = 1;
    }

    void applyQuality(Quality q) {
        quality = q;
        switch (q) {
            case Quality::Low:
                if (maxSimplificationError <= 0) maxSimplificationError = 3.0f;
                break;
            case Quality::High:
                if (maxSimplificationError <= 0 || maxSimplificationError > 1.0f) maxSimplificationError = 1.0f;
                break;
            case Quality::Normal:
            default:
                break; // use as-is or let autoCalc fill
        }
    }

    rcConfig toRcConfig(float mapW, float mapH) const {
        rcConfig cfg;
        memset(&cfg, 0, sizeof(cfg));
        cfg.width = (int)std::ceil(mapW / cs);
        cfg.height = (int)std::ceil(mapH / cs);
        cfg.cs = cs;
        cfg.ch = ch;
        cfg.bmin[0] = 0;
        cfg.bmin[1] = 0;
        cfg.bmin[2] = 0;
        cfg.bmax[0] = mapW;
        cfg.bmax[1] = std::max(2.0f, (float)walkableHeight * ch * 2.0f);  // voxels * ch = world units, x2 margin
        cfg.bmax[2] = mapH;
        cfg.walkableSlopeAngle = walkableSlopeAngle;
        cfg.walkableHeight = walkableHeight;
        cfg.walkableClimb = walkableClimb;
        cfg.walkableRadius = walkableRadius;
        cfg.maxEdgeLen = maxEdgeLen;
        cfg.maxSimplificationError = maxSimplificationError;
        cfg.minRegionArea = minRegionArea;
        cfg.mergeRegionArea = mergeRegionArea;
        cfg.maxVertsPerPoly = maxVertsPerPoly;
        cfg.tileSize = 0;
        cfg.borderSize = 0;
        cfg.detailSampleDist = detailSampleDist;
        cfg.detailSampleMaxError = detailSampleMaxError;
        return cfg;
    }
};

#endif
