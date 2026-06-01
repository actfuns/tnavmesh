#include "tnavmesh/svg_writer.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include <cstdio>

static std::string pts(const std::vector<float>& pts) {
    std::string s;
    char buf[32];
    for (size_t i = 0; i < pts.size() / 2; i++) {
        if (i > 0) s += " ";
        std::snprintf(buf, sizeof(buf), "%.2f,%.2f", pts[i*2], pts[i*2+1]);
        s += buf;
    }
    return s;
}

void writeSVG(const std::string& path,
              const MapInfo& mapInfo,
              const std::vector<Obstacle>& obstacles,
              const std::vector<MergedRegion>& merged,
              const rcPolyMesh* mesh,
              const SvgOptions& options,
              const std::vector<float>& pathPts,
              const std::vector<Obstacle>& overlaps,
              const std::vector<float>& annotations) {
    float mapW = static_cast<float>(mapInfo.width) * mapInfo.tileWidth;
    float mapH = static_cast<float>(mapInfo.height) * mapInfo.tileHeight;

    std::string svg;
    svg.reserve(65536);

    // Output size: use custom if set, otherwise map dimensions
    int outW = options.svgWidth > 0 ? options.svgWidth : (int)mapW;
    int outH = options.svgHeight > 0 ? options.svgHeight : (int)mapH;

    char hdr[384];
    std::snprintf(hdr, sizeof(hdr),
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        "<svg xmlns=\"http://www.w3.org/2000/svg\" "
        "viewBox=\"0 0 %.1f %.1f\" width=\"%d\" height=\"%d\""
        " preserveAspectRatio=\"xMidYMid meet\">\n",
        mapW, mapH, outW, outH);
    svg += hdr;

    svg += "<style>\n"
           "  .bg { fill: #f0f0f0; }\n"
           "  .grid { stroke: #ddd; stroke-width: 0.5; }\n"
           "  .obs { fill: rgba(255,0,0,0.3); stroke: #f00; stroke-width: 1.5; }\n"
           "  .merged { fill: rgba(0,102,255,0.15); stroke: #06f; stroke-width: 2.5; }\n"
           "  .navmesh { fill: rgba(0,200,0,0.35); stroke: #080; stroke-width: 0.8; }\n"
           "  .path { fill: none; stroke: #f80; stroke-width: 3; stroke-linejoin: round; stroke-linecap: round; }\n"
           "  .path-start { fill: #0a0; }\n"
           "  .path-end { fill: #f00; }\n"
           "  .overlap { fill: rgba(255,165,0,0.45); stroke: #f60; stroke-width: 2; stroke-dasharray: 6,3; }\n"
           "  .tri { stroke: rgba(100,100,100,0.4); stroke-width: 0.5; }\n"
           "  .annot { font-family: monospace; font-size: 12px; fill: #333; font-weight: bold; }\n"
           "</style>\n";

    svg += "<rect class=\"bg\" x=\"0\" y=\"0\" width=\"" + std::to_string(mapW) + "\" height=\"" + std::to_string(mapH) + "\"/>\n";

    // Grid
    if (options.showGrid) {
        svg += "<g class=\"grid\">\n";
        for (int x = 0; x <= mapInfo.width; x++) {
            float px = static_cast<float>(x) * mapInfo.tileWidth;
            char buf[128];
            std::snprintf(buf, sizeof(buf), "<line x1=\"%.1f\" y1=\"0\" x2=\"%.1f\" y2=\"%.1f\"/>\n", px, px, mapH);
            svg += buf;
        }
        for (int y = 0; y <= mapInfo.height; y++) {
            float py = static_cast<float>(y) * mapInfo.tileHeight;
            char buf[128];
            std::snprintf(buf, sizeof(buf), "<line x1=\"0\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\"/>\n", py, mapW, py);
            svg += buf;
        }
        svg += "</g>\n";
    }

    // Original obstacles
    if (options.showObstacles) {
        svg += "<g class=\"obs\">\n";
        for (const auto& obs : obstacles) {
            if (obs.points.size() < 6) continue; // need at least 3 pts for a polygon
            svg += "<polygon points=\"" + pts(obs.points) + "\"/>";
            if (obs.id > 0) {
                float cx = 0, cy = 0;
                for (size_t i = 0; i < obs.points.size() / 2; i++) {
                    cx += obs.points[i*2];
                    cy += obs.points[i*2+1];
                }
                cx /= (obs.points.size() / 2);
                cy /= (obs.points.size() / 2);
                char buf[128];
                std::snprintf(buf, sizeof(buf),
                    "<text x=\"%.1f\" y=\"%.1f\" font-family=\"monospace\" font-size=\"13\" font-weight=\"bold\" fill=\"#600\">%d</text>\n",
                    cx, cy, obs.id);
                svg += buf;
            } else {
                svg += "\n";
            }
        }
        svg += "</g>\n";
    }

    // Overlap regions
    if (options.showOverlaps && !overlaps.empty()) {
        svg += "<g class=\"overlap\">\n";
        for (const auto& o : overlaps)
            svg += "<polygon points=\"" + pts(o.points) + "\"/>\n";
        svg += "</g>\n";
    }

    // Merged obstacles
    if (options.showMerged) {
        svg += "<g class=\"merged\">\n";
        for (const auto& region : merged)
            if (region.exterior.points.size() >= 6)
                svg += "<polygon points=\"" + pts(region.exterior.points) + "\"/>\n";
        svg += "</g>\n";
        svg += "<g fill=\"#f0f0f0\" stroke=\"#06f\" stroke-width=\"1.5\" stroke-dasharray=\"4,3\">\n";
        for (const auto& region : merged)
            for (const auto& hole : region.holes)
                if (hole.points.size() >= 6)
                    svg += "<polygon points=\"" + pts(hole.points) + "\"/>\n";
        svg += "</g>\n";
    }

    // NavMesh
    if (options.showNavmesh && mesh && mesh->npolys > 0 && mesh->nverts > 0) {
        svg += "<g class=\"navmesh\">\n";
        unsigned short* verts = mesh->verts;
        unsigned short* polys = mesh->polys;
        for (int i = 0; i < mesh->npolys; i++) {
            svg += "<polygon points=\"";
            for (int j = 0; j < mesh->nvp; j++) {
                int vi = polys[i * mesh->nvp * 2 + j];
                if (vi == RC_MESH_NULL_IDX) break;
                float rx = verts[vi * 3 + 0] * mesh->cs + mesh->bmin[0];
                float ry = verts[vi * 3 + 2] * mesh->cs + mesh->bmin[2];
                float sy = options.tmxCoords ? ry : (mapH - ry);
                char buf[32];
                std::snprintf(buf, sizeof(buf), "%.1f,%.1f", rx, sy);
                if (j > 0) svg += " ";
                svg += buf;
            }
            svg += "\"/>\n";
        }
        svg += "</g>\n";
    }

    // Coordinate annotations
    if (options.showAnnotations && !annotations.empty()) {
        svg += "<g class=\"annot\">\n";
        for (size_t i = 0; i < annotations.size() / 2; i++) {
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                "<text x=\"%.1f\" y=\"%.1f\" paint-order=\"stroke\" stroke=\"white\" stroke-width=\"3\">(%.0f, %.0f)</text>\n",
                annotations[i*2] + 3, annotations[i*2+1] - 3,
                annotations[i*2], annotations[i*2+1]);
            svg += buf;
        }
        svg += "</g>\n";
    }

    // Path
    if (options.showPath && !pathPts.empty()) {
        svg += "<g class=\"path\">\n<polyline points=\"" + pts(pathPts) + "\"/>\n</g>\n";
        // Waypoint dots
        svg += "<g fill=\"#f80\">\n";
        for (size_t i = 0; i < pathPts.size() / 2; i++) {
            char buf[128];
            std::snprintf(buf, sizeof(buf), "<circle cx=\"%.1f\" cy=\"%.1f\" r=\"2.5\"/>\n",
                pathPts[i*2], pathPts[i*2+1]);
            svg += buf;
        }
        svg += "</g>\n";
        // Start/end markers (use first/last waypoint)
        if (pathPts.size() >= 4) {
            float sx = pathPts[0], sy = pathPts[1];
            float ex = pathPts[pathPts.size()-2], ey = pathPts[pathPts.size()-1];
            float diag = std::sqrt(mapW * mapW + mapH * mapH);
            float r = std::max(4.0f, diag / 400.0f);
            float fontSize = std::max(9.0f, diag / 180.0f);
            float offset = r + 4;
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                "<circle class=\"path-start\" cx=\"%.1f\" cy=\"%.1f\" r=\"%.1f\"/>\n"
                "<text x=\"%.1f\" y=\"%.1f\" font-family=\"monospace\" font-size=\"%.1f\" fill=\"#0a0\">Start</text>\n",
                sx, sy, r, sx + offset, sy - offset * 0.6f, fontSize);
            svg += buf;
            std::snprintf(buf, sizeof(buf),
                "<rect class=\"path-end\" x=\"%.1f\" y=\"%.1f\" width=\"%.1f\" height=\"%.1f\" rx=\"1\"/>\n"
                "<text x=\"%.1f\" y=\"%.1f\" font-family=\"monospace\" font-size=\"%.1f\" fill=\"#f00\">End</text>\n",
                ex - r * 0.5f, ey - r * 0.5f, r, r, ex + offset, ey + offset * 0.4f, fontSize);
            svg += buf;
        }
    }

    // Legend
    if (options.showLegend) {
        float legScale = std::min(mapW, mapH) / 800.0f;
        if (legScale < 1.0f) legScale = 1.0f;
        float legFont = 12.0f * legScale;
        float legSize = 12.0f * legScale;
        float legStroke = std::min(legScale, 3.0f);
        int dx = (int)(10 * legScale);
        int dy = (int)(20 * legScale);
        char leg[1024];
        std::snprintf(leg, sizeof(leg),
            "<g transform=\"translate(%d,%d)\" font-family=\"monospace\" font-size=\"%.1f\">\n"
            "<rect x=\"0\" y=\"%.1f\" width=\"%.1f\" height=\"%.1f\" fill=\"rgba(255,0,0,0.3)\" stroke=\"red\" stroke-width=\"%.1f\"/>"
            "<text x=\"%.1f\" y=\"0\">Obstacle</text>\n"
            "<rect x=\"0\" y=\"%.1f\" width=\"%.1f\" height=\"%.1f\" fill=\"none\" stroke=\"#06f\" stroke-width=\"%.1f\"/>"
            "<text x=\"%.1f\" y=\"%.1f\">Merged</text>\n"
            "<rect x=\"0\" y=\"%.1f\" width=\"%.1f\" height=\"%.1f\" fill=\"rgba(0,200,0,0.35)\" stroke=\"#080\" stroke-width=\"%.1f\"/>"
            "<text x=\"%.1f\" y=\"%.1f\">NavMesh</text>\n",
            dx, dy, legFont,
            -legSize * 0.9f, legSize, legSize, legStroke,
            legSize * 1.5f,
            legSize * 0.3f, legSize, legSize, legStroke,
            legSize * 1.5f, legSize + legSize * 0.3f,
            legSize * 1.3f, legSize, legSize, legStroke,
            legSize * 1.5f, legSize * 2 + legSize * 0.3f);
        svg += leg;
        if (options.showOverlaps && !overlaps.empty()) {
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                "<rect x=\"0\" y=\"%.1f\" width=\"%.1f\" height=\"%.1f\" fill=\"rgba(255,165,0,0.45)\" stroke=\"#f60\" stroke-width=\"%.1f\"/>"
                "<text x=\"%.1f\" y=\"%.1f\">Overlap</text>\n",
                legSize * 2.7f, legSize, legSize, legStroke,
                legSize * 1.5f, legSize * 3 + legSize * 0.3f);
            svg += buf;
        }
        svg += "</g>\n";
    }

    svg += "</svg>\n";

    std::ofstream ofs(path);
    if (!ofs) {
        std::cerr << "Error: could not write " << path << std::endl;
        return;
    }
    ofs << svg;
    ofs.close();
}
