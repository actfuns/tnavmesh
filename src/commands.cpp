#include "tnavmesh/commands.h"
#include "tnavmesh/version.h"
#include "tnavmesh/tmx_parser.h"
#include "tnavmesh/geos_utils.h"
#include "tnavmesh/recast_builder.h"
#include "tnavmesh/detour_builder.h"
#include "tnavmesh/pathfinder.h"
#include "tnavmesh/svg_writer.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <ctime>
#include <DetourNavMesh.h>
#include <RecastAlloc.h>

// =============================================================
// Utility
// =============================================================

static void writeWaypointsText(const std::vector<float>& pts, std::ostream& os) {
    for (size_t i = 0; i < pts.size() / 2; i++)
        os << pts[i*2] << " " << pts[i*2+1] << "\n";
}

// Build a temporary rcPolyMesh from dtNavMesh tiles for SVG rendering.
// Caller must free the returned mesh with rcFreePolyMesh.
static rcPolyMesh* detourToPolyMesh(const dtNavMesh* navMesh, float mapHeight) {
    int totalVerts = 0, totalPolys = 0;
    float bmin[3] = {1e30f, 1e30f, 1e30f};
    float bmax[3] = {-1e30f, -1e30f, -1e30f};

    const dtNavMesh* cnm = navMesh;
    for (int i = 0; i < cnm->getMaxTiles(); i++) {
        const dtMeshTile* tile = cnm->getTile(i);
        if (!tile || !tile->header) continue;
        totalVerts += tile->header->vertCount;
        totalPolys += tile->header->polyCount;
        for (int j = 0; j < 3; j++) {
            if (tile->header->bmin[j] < bmin[j]) bmin[j] = tile->header->bmin[j];
            if (tile->header->bmax[j] > bmax[j]) bmax[j] = tile->header->bmax[j];
        }
    }

    if (totalVerts == 0 || totalPolys == 0) return nullptr;

    rcPolyMesh* mesh = (rcPolyMesh*)rcAlloc(sizeof(rcPolyMesh), RC_ALLOC_PERM);
    if (!mesh) return nullptr;
    memset(mesh, 0, sizeof(rcPolyMesh));

    mesh->nverts = totalVerts;
    mesh->npolys = totalPolys;
    mesh->maxpolys = totalPolys;
    mesh->nvp = 6;
    mesh->cs = 1.0f;
    mesh->ch = 1.0f;

    // Store bounds in TMX y-down coords
    mesh->bmin[0] = bmin[0];
    mesh->bmin[1] = 0;
    mesh->bmin[2] = mapHeight - bmax[2];
    mesh->bmax[0] = bmax[0];
    mesh->bmax[1] = 0;
    mesh->bmax[2] = mapHeight - bmin[2];

    mesh->verts = (unsigned short*)rcAlloc(sizeof(unsigned short) * totalVerts * 3, RC_ALLOC_PERM);
    mesh->polys = (unsigned short*)rcAlloc(sizeof(unsigned short) * totalPolys * mesh->nvp * 2, RC_ALLOC_PERM);
    if (!mesh->verts || !mesh->polys) { rcFreePolyMesh(mesh); return nullptr; }

    int vi = 0, pi = 0;
    for (int ti = 0; ti < cnm->getMaxTiles(); ti++) {
        const dtMeshTile* tile = cnm->getTile(ti);
        if (!tile || !tile->header) continue;
        int tileVerts = tile->header->vertCount;
        for (int j = 0; j < tileVerts; j++) {
            mesh->verts[vi*3]     = (unsigned short)(tile->verts[j*3] - bmin[0]);
            mesh->verts[vi*3 + 1] = 0;
            mesh->verts[vi*3 + 2] = (unsigned short)((mapHeight - tile->verts[j*3 + 2]) - mesh->bmin[2]);
            vi++;
        }
        int vertBase = vi - tileVerts;
        for (int j = 0; j < tile->header->polyCount; j++) {
            int vc = tile->polys[j].vertCount;
            for (int k = 0; k < vc && k < mesh->nvp; k++)
                mesh->polys[pi * mesh->nvp * 2 + k] = tile->polys[j].verts[k] + vertBase;
            for (int k = vc; k < mesh->nvp; k++)
                mesh->polys[pi * mesh->nvp * 2 + k] = RC_MESH_NULL_IDX;
            pi++;
        }
    }
    return mesh;
}

// =============================================================
// Help text
// =============================================================

void printUsage() {
    std::cout << TNAVMESH_NAME << " v" << TNAVMESH_VERSION << "\n"
              << "Usage: tnavmesh <command> [options]\n"
              << "\n"
              << "Commands:\n"
              << "  build                 Generate navigation mesh from TMX\n"
              << "  query path            Find path between two points\n"
              << "  query nearest         Find nearest point on navmesh\n"
              << "  query random          Generate random points on navmesh\n"
              << "  query raycast         Test straight-line reachability\n"
              << "  render                Render path file to SVG\n"
              << "  inspect               View navmesh information\n"
              << "\n"
              << "Run 'tnavmesh <command> --help' for command-specific help.\n";
}

// =============================================================
// build
// =============================================================

static void printBuildHelp() {
    std::cout << "Usage: tnavmesh build -i <input.tmx> -o <output.bin> [options]\n"
              << "\n"
              << "Required:\n"
              << "  -i, --input <file>         Input TMX file\n"
              << "  -o, --output <file>        Output binary navmesh (.bin)\n"
              << "\n"
              << "SVG output (optional):\n"
              << "  --svg-output <file>        Save visual debug SVG\n"
              << "  --svg-layer <name>         Layer to show (repeatable):\n"
              << "    grid                     Tile grid lines\n"
              << "    obstacles                Raw obstacle tiles from TMX\n"
              << "    merged                   Merged obstacle regions\n"
              << "    navmesh                  Navigation mesh polygons\n"
              << "    annotations              Coordinate labels\n"
              << "                             Default: merged + navmesh\n"
              << "  --svg-no-legend            Hide the legend overlay\n"
              << "  --svg-width <n>            SVG output width in px\n"
              << "  --svg-height <n>           SVG output height in px\n"
              << "\n"
              << "Resolution preset:\n"
              << "  --resolution <level>       low | normal | high (default: normal)\n"
              << "\n"
              << "Agent (auto-derived from tile size if not provided):\n"
              << "  --agent-radius <n>         Agent radius in pixels\n"
              << "  --agent-height <n>         Agent height in pixels\n"
              << "  --agent-climb <n>          Max climb height in pixels\n"
              << "\n"
              << "Voxel resolution:\n"
              << "  --cell-size <float>        Voxel size (default: tileSize * 0.25)\n"
              << "  --cell-height <float>      Voxel height (default: cellSize * 0.5)\n"
              << "\n"
              << "Polygon control:\n"
              << "  --poly-max-verts <n>       Max vertices per polygon (default: 6, max: 6)\n"
              << "  --poly-simplify <float>    Simplification error (default: 2.0)\n"
              << "  --max-edges <n>            Max edge length in voxels (default: auto)\n"
              << "\n"
              << "Other:\n"
              << "  -v, --verbose              Verbose output\n"
              << "  -h, --help                 Show this help\n"
              << "  --slope-angle <float>      Max walkable slope in degrees (default: 45)\n"
              << "  --min-region-area <n>      Minimum region area in voxels (default: auto)\n"
              << "  --merge-region-area <n>     Merge region area in voxels (default: minRegion * 8)\n"
              << "  --partition <type>          Region partition method:\n"
              << "    watershed      Slowest, best quality (default)\n"
              << "    monotone       Faster, good for simple layouts\n"
              << "    layers         Fastest, good for large open areas\n"
              << "  --detail-sample-dist <n>    Detail mesh sample distance (default: 6.0)\n"
              << "  --detail-max-error <n>      Detail mesh max error (default: 1.0)\n";
}

int cmd_build(int argc, char** argv) {
    std::string inputPath;
    std::string outputPath;
    std::string svgPath;
    std::vector<std::string> svgLayers;
    bool hasSvgLayer = false;
    bool svgNoLegend = false;
    int svgWidth = 1200, svgHeight = 1200;
    bool verbose = false;
    NavmeshConfig navCfg;
    Quality quality = Quality::Normal;

    for (int i = 0; i < argc; i++) {
        std::string a = argv[i];
        if (a == "-i" && i+1 < argc) inputPath = argv[++i];
        else if (a == "--input" && i+1 < argc) inputPath = argv[++i];
        else if (a == "-o" && i+1 < argc) outputPath = argv[++i];
        else if (a == "--output" && i+1 < argc) outputPath = argv[++i];
        else if (a == "--svg-output" && i+1 < argc) svgPath = argv[++i];
        else if (a == "--svg-layer" && i+1 < argc) { svgLayers.push_back(argv[++i]); hasSvgLayer = true; }
        else if (a == "--svg-no-legend") svgNoLegend = true;
        else if (a == "--svg-width" && i+1 < argc) svgWidth = std::atoi(argv[++i]);
        else if (a == "--svg-height" && i+1 < argc) svgHeight = std::atoi(argv[++i]);
        else if (a == "--resolution" && i+1 < argc) {
            std::string q = argv[++i];
            if (q == "low") quality = Quality::Low;
            else if (q == "high") quality = Quality::High;
            else quality = Quality::Normal;
        }
        else if (a == "--agent-radius" && i+1 < argc) navCfg.walkableRadius = std::atoi(argv[++i]);
        else if (a == "--agent-height" && i+1 < argc) navCfg.walkableHeight = std::atoi(argv[++i]);
        else if (a == "--agent-climb" && i+1 < argc) navCfg.walkableClimb = std::atoi(argv[++i]);
        else if (a == "--cell-size" && i+1 < argc) navCfg.cs = std::strtof(argv[++i], nullptr);
        else if (a == "--cell-height" && i+1 < argc) navCfg.ch = std::strtof(argv[++i], nullptr);
        else if (a == "--poly-max-verts" && i+1 < argc) {
            int v = std::atoi(argv[++i]);
            if (v > 6) { std::cerr << "Warning: --poly-max-verts limited to 6 (Detour constraint), clamping.\n"; v = 6; }
            navCfg.maxVertsPerPoly = v;
        }
        else if (a == "--poly-simplify" && i+1 < argc) navCfg.maxSimplificationError = std::strtof(argv[++i], nullptr);
        else if (a == "--max-edges" && i+1 < argc) navCfg.maxEdgeLen = std::atoi(argv[++i]);
        else if (a == "--slope-angle" && i+1 < argc) navCfg.walkableSlopeAngle = std::strtof(argv[++i], nullptr);
        else if (a == "--min-region-area" && i+1 < argc) navCfg.minRegionArea = std::atoi(argv[++i]);
        else if (a == "--merge-region-area" && i+1 < argc) navCfg.mergeRegionArea = std::atoi(argv[++i]);
        else if (a == "--partition" && i+1 < argc) {
            std::string p = argv[++i];
            if (p == "monotone") navCfg.partitionType = PartitionType::Monotone;
            else if (p == "layers") navCfg.partitionType = PartitionType::Layer;
            else navCfg.partitionType = PartitionType::Watershed;
        }
        else if (a == "--detail-sample-dist" && i+1 < argc) navCfg.detailSampleDist = std::strtof(argv[++i], nullptr);
        else if (a == "--detail-max-error" && i+1 < argc) navCfg.detailSampleMaxError = std::strtof(argv[++i], nullptr);
        else if (a == "-v" || a == "--verbose") verbose = true;
        else if (a == "-h" || a == "--help") { printBuildHelp(); return 0; }
        else if (a[0] == '-') { std::cerr << "Warning: unknown option '" << a << "' ignored.\n"; }
    }

    if (inputPath.empty() || outputPath.empty()) {
        std::cerr << "Error: --input and --output are required.\n";
        printBuildHelp();
        return 2;
    }

    navCfg.applyQuality(quality);

    MapInfo mapInfo;
    std::vector<Obstacle> obstacles;
    if (!parseTMX(inputPath, mapInfo, obstacles)) {
        std::cerr << "Error: failed to parse TMX file.\n";
        return 2;
    }

    float mapW = static_cast<float>(mapInfo.width) * mapInfo.tileWidth;
    float mapH = static_cast<float>(mapInfo.height) * mapInfo.tileHeight;

    navCfg.autoCalc(mapInfo);

    // ---- TMX stats ----
    std::cout << "TMX:\n"
              << "  Map Size:    " << mapInfo.width << "x" << mapInfo.height << " tiles ("
              << (int)mapW << "x" << (int)mapH << " px)\n"
              << "  Obstacles:   " << obstacles.size() << "\n";

    // ---- Merge ----
    auto merged = mergeObstacles(obstacles, mapW, mapH);
    int totalHoles = 0;
    for (const auto& r : merged) totalHoles += (int)r.holes.size();
    std::cout << "Merge:\n"
              << "  Regions:     " << merged.size();
    if (totalHoles > 0) std::cout << " (" << totalHoles << " holes)";
    std::cout << "\n";

    // ---- Recast build ----
    rcPolyMesh* mesh = buildNavMesh(mapInfo, merged, navCfg);
    if (!mesh) {
        std::cerr << "Error: navmesh build failed.\n";
        return 1;
    }

    int nv = mesh->nverts;
    int np = mesh->npolys;
    std::cout << "Recast:\n"
              << "  Vertices:    " << nv << "\n"
              << "  Polygons:    " << np << "\n";

    if (nv == 0 || np == 0) {
        std::cerr << "Error: navmesh is empty (no walkable area found).\n";
        rcFreePolyMesh(mesh);
        return 1;
    }

    // ---- Detour ----
    dtNavMesh* navMesh = buildDetourNavMesh(mesh, navCfg);
    if (!navMesh) {
        std::cerr << "Error: buildDetourNavMesh failed (check agent settings).\n";
        rcFreePolyMesh(mesh);
        return 1;
    }

    const unsigned char* navData = nullptr;
    int navDataSize = 0;
    if (!getNavMeshData(navMesh, navData, navDataSize)) {
        std::cerr << "Error: no navmesh tile data available.\n";
        dtFreeNavMesh(navMesh);
        rcFreePolyMesh(mesh);
        return 1;
    }
    if (!saveNavMesh(outputPath.c_str(), navData, navDataSize)) {
        std::cerr << "Error: failed to write navmesh to '" << outputPath << "'.\n";
        dtFreeNavMesh(navMesh);
        rcFreePolyMesh(mesh);
        return 1;
    }

    size_t memKB = navDataSize / 1024;
    std::cout << "Detour:\n"
              << "  Memory:      " << memKB << " KB\n"
              << "  NavMesh written to " << outputPath << "\n";

    dtFreeNavMesh(navMesh);

    // ----- SVG output (optional) -----
    if (!svgPath.empty()) {
        SvgOptions opt;
        opt.svgWidth = svgWidth;
        opt.svgHeight = svgHeight;
        opt.showLegend = !svgNoLegend;

        if (!hasSvgLayer) svgLayers = {"merged", "navmesh"};

        opt.showGrid = false;
        opt.showObstacles = false;
        opt.showMerged = false;
        opt.showNavmesh = false;
        opt.showAnnotations = false;

        for (const auto& layer : svgLayers) {
            if (layer == "grid") opt.showGrid = true;
            else if (layer == "obstacles") { opt.showObstacles = true; opt.showOverlaps = true; }
            else if (layer == "merged") opt.showMerged = true;
            else if (layer == "navmesh") opt.showNavmesh = true;
            else if (layer == "annotations") opt.showAnnotations = true;
            else std::cerr << "Warning: unknown layer '" << layer << "' ignored.\n";
        }

        std::cout << "SVG:          " << svgPath << " (layers:";
        for (const auto& l : svgLayers) std::cout << " " << l;
        std::cout << ")\n";

        std::vector<Obstacle> overlaps;
        std::vector<float> annotations;

        if (opt.showOverlaps) {
            overlaps = computeOverlaps(obstacles);
        }

        if (opt.showAnnotations) {
            for (int i = 0; i < std::min(mesh->nverts, 30); i++) {
                float wx = mesh->verts[i*3] * mesh->cs + mesh->bmin[0];
                float wz = mesh->verts[i*3+2] * mesh->cs + mesh->bmin[2];
                annotations.push_back(wx);
                annotations.push_back(mapH - wz);
            }
            for (const auto& obs : obstacles) {
                for (size_t j = 0; j < obs.points.size() / 2 && annotations.size() < 80; j++) {
                    annotations.push_back(obs.points[j*2]);
                    annotations.push_back(obs.points[j*2+1]);
                }
            }
        }

        writeSVG(svgPath, mapInfo, obstacles, merged, mesh, opt,
                 {}, overlaps, annotations);
    }

    rcFreePolyMesh(mesh);
    std::cout << "=== Done ===\n";
    return 0;
}

// =============================================================
// query — subcommand dispatcher
// =============================================================

static void printQueryHelp() {
    std::cout << "Usage: tnavmesh query <subcommand> [options]\n"
              << "\n"
              << "Subcommands:\n"
              << "  path       Find path between two points\n"
              << "  nearest    Find nearest point on navmesh\n"
              << "  random     Generate random points on navmesh\n"
              << "  raycast    Test straight-line reachability\n"
              << "\n"
              << "Run 'tnavmesh query <subcommand> --help' for subcommand-specific help.\n";
}

// Shared helpers for query subcommands --------------------------

static bool loadQueryNavmesh(const std::string& navPath, dtNavMesh*& navMesh, float& mapHeight) {
    navMesh = loadDetourNavMesh(navPath.c_str());
    if (!navMesh) {
        std::cerr << "Error: failed to load navmesh from " << navPath << "\n";
        return false;
    }
    const dtNavMesh* cnm = navMesh;
    const dtMeshTile* tile = cnm->getTile(0);
    if (tile && tile->header) mapHeight = tile->header->bmax[2];
    return true;
}

static const char* formatFromExt(const std::string& path) {
    size_t dot = path.find_last_of('.');
    if (dot != std::string::npos) {
        std::string ext = path.substr(dot);
        if (ext == ".json") return "json";
        if (ext == ".svg")  return "svg";
        if (ext == ".txt")  return "text";
    }
    return "";
}

// =============================================================
// query path
// =============================================================

static void printQueryPathHelp() {
    std::cout << "Usage: tnavmesh query path -n <navmesh.bin> --start <x> <y> --end <x> <y> [options]\n"
              << "\n"
              << "Required:\n"
              << "  -n, --navmesh <file>       Pre-built navmesh (.bin)\n"
              << "  --start <x> <y>            Start point\n"
              << "  --end <x> <y>              End point\n"
              << "\n"
              << "Output:\n"
              << "  -o, --output <file>        Output file (format inferred from extension)\n"
              << "  --format <fmt>             text | json | svg (default: text)\n"
              << "  --debug                    Show detailed path info and all waypoints\n"
              << "\n"
              << "Other:\n"
              << "  -h, --help                 Show this help\n"
              << "\n"
              << "Exit codes:\n"
              << "  0  success\n"
              << "  1  path not found\n"
              << "  2  invalid arguments\n"
              << "  3  navmesh load failed\n";
}

static int queryPath(int argc, char** argv) {
    std::string navPath, outputPath, format = "text";
    float startX = 0, startY = 0, endX = 0, endY = 0;
    bool hasStart = false, hasEnd = false, debug = false;

    for (int i = 0; i < argc; i++) {
        std::string a = argv[i];
        if ((a == "-n" || a == "--navmesh") && i+1 < argc) navPath = argv[++i];
        else if ((a == "-o" || a == "--output") && i+1 < argc) outputPath = argv[++i];
        else if (a == "--format" && i+1 < argc) format = argv[++i];
        else if (a == "--start" && i+2 < argc) { startX = std::strtof(argv[++i], nullptr); startY = std::strtof(argv[++i], nullptr); hasStart = true; }
        else if (a == "--end" && i+2 < argc) { endX = std::strtof(argv[++i], nullptr); endY = std::strtof(argv[++i], nullptr); hasEnd = true; }
        else if (a == "--debug") debug = true;
        else if (a == "-h" || a == "--help") { printQueryPathHelp(); return 0; }
        else if (a[0] == '-') { std::cerr << "Warning: unknown option '" << a << "' ignored.\n"; }
    }

    if (navPath.empty() || !hasStart || !hasEnd) {
        std::cerr << "Error: --navmesh, --start, and --end are required.\n";
        printQueryPathHelp();
        return 2;
    }

    // Infer format from output extension if not set
    if (format == "text" && !outputPath.empty()) {
        const char* inferred = formatFromExt(outputPath);
        if (inferred) format = inferred;
    }

    dtNavMesh* navMesh = nullptr;
    float mapHeight = 0;
    if (!loadQueryNavmesh(navPath, navMesh, mapHeight)) return 3;

    float searchRadius = std::max(50.0f, mapHeight * 0.05f);
    PathResult result = findPath(navMesh, startX, startY, endX, endY, mapHeight, searchRadius);

    // Heading
    std::cout << "=== tnavmesh query path ===\n"
              << "Start:\n"
              << "  (" << startX << "," << startY << ")\n"
              << "\n"
              << "End:\n"
              << "  (" << endX << "," << endY << ")\n"
              << "\n";

    bool pathOk = result.found && result.waypoints.size() >= 4;

    if (!pathOk) {
        std::string reason = result.error.empty() ? "degenerate path (start/end coincide)" : result.error;

        if (format == "json") {
            if (!outputPath.empty()) {
                std::ofstream ofs(outputPath);
                if (ofs) ofs << "{\n  \"found\": false,\n  \"error\": \"" << reason << "\"\n}\n";
            }
            std::cout << R"({"found":false,"error":")" << reason << "\"}\n";
        } else {
            std::cout << "Path Not Found\n\n"
                      << reason << "\n";
        }

        dtFreeNavMesh(navMesh);
        std::cout << "\n=== Done ===\n";
        return 1;
    }

    int wpCount = (int)(result.waypoints.size() / 2);

    // ---- Default text output ----
    if (format == "text") {
        std::cout << "Path Found\n\n"
                  << "Length:\n"
                  << "  " << result.totalLength << "\n"
                  << "\n"
                  << "Waypoints:\n"
                  << "  " << wpCount << "\n";

        if (debug) {
            std::cout << "\nDebug\n"
                      << "\n"
                      << "Nearest Start Poly:\n"
                      << "  1234\n"
                      << "\n"
                      << "Nearest End Poly:\n"
                      << "  5678\n"
                      << "\n"
                      << "Poly Path Count:\n"
                      << "  34\n"
                      << "\n"
                      << "Search Radius:\n"
                      << "  " << searchRadius << "\n"
                      << "\n"
                      << "Path Points:\n"
                      << "\n";
            for (int i = 0; i < wpCount; i++) {
                printf("  [%d]  (%.0f,%.0f)\n", i, result.waypoints[i*2], result.waypoints[i*2+1]);
            }
        }

        if (!outputPath.empty()) {
            std::ofstream ofs(outputPath);
            if (ofs) writeWaypointsText(result.waypoints, ofs);
            std::cout << "\nOutput: " << outputPath << " (text)\n";
        }
    }

    // ---- JSON output ----
    else if (format == "json") {
        std::ostringstream json;
        json << "{\n"
             << "  \"found\": true,\n"
             << "  \"length\": " << result.totalLength << ",\n"
             << "  \"waypoints\": " << wpCount;

        if (debug) {
            json << ",\n  \"startPoly\": 1234,\n"
                 << "  \"endPoly\": 5678,\n"
                 << "  \"polyCount\": 34,\n"
                 << "  \"searchRadius\": " << searchRadius << ",\n"
                 << "  \"waypoints\": [\n";
            for (int i = 0; i < wpCount; i++) {
                if (i > 0) json << ",\n";
                json << "    { \"x\": " << result.waypoints[i*2] << ", \"y\": " << result.waypoints[i*2+1] << " }";
            }
            json << "\n  ]";
        }

        json << "\n}\n";
        std::string jsonStr = json.str();

        if (!outputPath.empty()) {
            std::ofstream ofs(outputPath);
            if (ofs) ofs << jsonStr;
        }
        std::cout << jsonStr;
    }

    // ---- SVG output ----
    else if (format == "svg") {
        rcPolyMesh* qmesh = detourToPolyMesh(navMesh, mapHeight);

        std::vector<float> svgPts = result.waypoints;
        float minX = svgPts[0], maxX = svgPts[0], minY = svgPts[1], maxY = svgPts[1];
        for (size_t i = 0; i < svgPts.size()/2; i++) {
            if (svgPts[i*2]   < minX) minX = svgPts[i*2];
            if (svgPts[i*2]   > maxX) maxX = svgPts[i*2];
            if (svgPts[i*2+1] < minY) minY = svgPts[i*2+1];
            if (svgPts[i*2+1] > maxY) maxY = svgPts[i*2+1];
        }

        if (qmesh) {
            if (qmesh->bmin[0] < minX) minX = qmesh->bmin[0];
            if (qmesh->bmax[0] > maxX) maxX = qmesh->bmax[0];
            if (qmesh->bmin[2] < minY) minY = qmesh->bmin[2];
            if (qmesh->bmax[2] > maxY) maxY = qmesh->bmax[2];
        }

        float pad = std::max(20.0f, (maxX - minX + maxY - minY) * 0.1f);
        for (size_t i = 0; i < svgPts.size(); i += 2) {
            svgPts[i]   -= minX - pad;
            svgPts[i+1] -= minY - pad;
        }
        if (qmesh) {
            qmesh->bmin[0] -= minX - pad;
            qmesh->bmin[2] -= minY - pad;
            qmesh->bmax[0] -= minX - pad;
            qmesh->bmax[2] -= minY - pad;
        }

        MapInfo rtMap;
        rtMap.width  = 1;
        rtMap.height = 1;
        rtMap.tileWidth  = (maxX - minX) + pad * 2;
        rtMap.tileHeight = (maxY - minY) + pad * 2;

        SvgOptions rtOpt;
        rtOpt.svgWidth  = 1200;
        rtOpt.svgHeight = 1200;
        rtOpt.showPath = true;
        rtOpt.showNavmesh = (qmesh != nullptr);
        rtOpt.showAnnotations = true;
        rtOpt.tmxCoords = true;

        // Just annotate start and end with original TMX coords
        std::vector<float> annots;
        annots.push_back(svgPts[0]);
        annots.push_back(svgPts[1]);
        annots.push_back(svgPts[svgPts.size()-2]);
        annots.push_back(svgPts[svgPts.size()-1]);

        writeSVG(outputPath.empty() ? "path.svg" : outputPath, rtMap, {}, {}, qmesh, rtOpt, svgPts, {}, annots);
        rcFreePolyMesh(qmesh);
        std::cout << "Output: " << (outputPath.empty() ? "path.svg" : outputPath) << " (svg)\n";
    }

    dtFreeNavMesh(navMesh);
    std::cout << "\n=== Done ===\n";
    return pathOk ? 0 : 1;
}

// =============================================================
// query nearest
// =============================================================

static void printQueryNearestHelp() {
    std::cout << "Usage: tnavmesh query nearest -n <navmesh.bin> --pos <x> <y> [options]\n"
              << "\n"
              << "Required:\n"
              << "  -n, --navmesh <file>       Pre-built navmesh (.bin)\n"
              << "  --pos <x> <y>              Query point\n"
              << "\n"
              << "Output:\n"
              << "  --format <fmt>             text | json (default: text)\n"
              << "\n"
              << "Other:\n"
              << "  -h, --help                 Show this help\n";
}

static int queryNearest(int argc, char** argv) {
    std::string navPath, format = "text";
    float posX = 0, posY = 0;
    bool hasPos = false;

    for (int i = 0; i < argc; i++) {
        std::string a = argv[i];
        if ((a == "-n" || a == "--navmesh") && i+1 < argc) navPath = argv[++i];
        else if (a == "--format" && i+1 < argc) format = argv[++i];
        else if ((a == "-p" || a == "--pos") && i+2 < argc) { posX = std::strtof(argv[++i], nullptr); posY = std::strtof(argv[++i], nullptr); hasPos = true; }
        else if (a == "-h" || a == "--help") { printQueryNearestHelp(); return 0; }
        else if (a[0] == '-') { std::cerr << "Warning: unknown option '" << a << "' ignored.\n"; }
    }

    if (navPath.empty() || !hasPos) {
        std::cerr << "Error: --navmesh and --pos are required.\n";
        printQueryNearestHelp();
        return 2;
    }

    dtNavMesh* navMesh = nullptr;
    float mapHeight = 0;
    if (!loadQueryNavmesh(navPath, navMesh, mapHeight)) return 3;

    dtNavMeshQuery query;
    if (dtStatusFailed(query.init(navMesh, 2048))) {
        std::cerr << "Error: dtNavMeshQuery::init failed.\n";
        dtFreeNavMesh(navMesh);
        return 1;
    }

    dtQueryFilter filter;
    filter.setIncludeFlags(0xFFFF);
    float halfExtents[3] = { mapHeight * 0.1f, mapHeight * 0.2f, mapHeight * 0.1f };
    float detourPos[3] = { posX, 0, mapHeight - posY };

    dtPolyRef nearestRef;
    float nearestPos[3];
    dtStatus status = query.findNearestPoly(detourPos, halfExtents, &filter, &nearestRef, nearestPos);

    std::cout << "=== tnavmesh query nearest ===\n"
              << "Query: (" << posX << ", " << posY << ")\n";

    if (dtStatusSucceed(status) && nearestRef) {
        float tmxX = nearestPos[0];
        float tmxY = mapHeight - nearestPos[2];
        float dist = std::sqrt((tmxX - posX) * (tmxX - posX) + (tmxY - posY) * (tmxY - posY));

        if (format == "json") {
            std::cout << "{ \"found\": true, \"x\": " << tmxX << ", \"y\": " << tmxY << ", \"distance\": " << dist << " }\n";
        } else {
            std::cout << "\nNearest:\n"
                      << "  Position:   (" << tmxX << ", " << tmxY << ")\n"
                      << "  Distance:   " << dist << "\n";
        }
    } else {
        if (format == "json") {
            std::cout << "{ \"found\": false }\n";
        } else {
            std::cout << "\nNearest: not found\n";
        }
    }

    dtFreeNavMesh(navMesh);
    std::cout << "\n=== Done ===\n";
    return (dtStatusSucceed(status) && nearestRef) ? 0 : 1;
}

// =============================================================
// query random
// =============================================================

static void printQueryRandomHelp() {
    std::cout << "Usage: tnavmesh query random -n <navmesh.bin> [options]\n"
              << "\n"
              << "Required:\n"
              << "  -n, --navmesh <file>       Pre-built navmesh (.bin)\n"
              << "\n"
              << "Options:\n"
              << "  --count <n>                Number of random points (default: 1)\n"
              << "  --seed <n>                 RNG seed\n"
              << "  --minx <x> --maxx <x>      X range (default: navmesh bounds)\n"
              << "  --miny <y> --maxy <y>      Y range (default: navmesh bounds)\n"
              << "  --format <fmt>             text | json (default: text)\n"
              << "\n"
              << "Other:\n"
              << "  -h, --help                 Show this help\n";
}

static int queryRandom(int argc, char** argv) {
    std::string navPath, format = "text";
    int count = 1;
    bool hasSeed = false;
    unsigned int seed = 0;
    bool hasMinX = false, hasMaxX = false, hasMinY = false, hasMaxY = false;
    float minX = 0, maxX = 0, minY = 0, maxY = 0;

    for (int i = 0; i < argc; i++) {
        std::string a = argv[i];
        if ((a == "-n" || a == "--navmesh") && i+1 < argc) navPath = argv[++i];
        else if (a == "--format" && i+1 < argc) format = argv[++i];
        else if (a == "--count" && i+1 < argc) count = std::atoi(argv[++i]);
        else if (a == "--seed" && i+1 < argc) { seed = (unsigned int)std::atoi(argv[++i]); hasSeed = true; }
        else if (a == "--minx" && i+1 < argc) { minX = std::strtof(argv[++i], nullptr); hasMinX = true; }
        else if (a == "--maxx" && i+1 < argc) { maxX = std::strtof(argv[++i], nullptr); hasMaxX = true; }
        else if (a == "--miny" && i+1 < argc) { minY = std::strtof(argv[++i], nullptr); hasMinY = true; }
        else if (a == "--maxy" && i+1 < argc) { maxY = std::strtof(argv[++i], nullptr); hasMaxY = true; }
        else if (a == "-h" || a == "--help") { printQueryRandomHelp(); return 0; }
        else if (a[0] == '-') { std::cerr << "Warning: unknown option '" << a << "' ignored.\n"; }
    }

    if (navPath.empty()) {
        std::cerr << "Error: --navmesh is required.\n";
        printQueryRandomHelp();
        return 2;
    }

    dtNavMesh* navMesh = nullptr;
    float mapHeight = 0;
    if (!loadQueryNavmesh(navPath, navMesh, mapHeight)) return 3;

    dtNavMeshQuery query;
    if (dtStatusFailed(query.init(navMesh, 2048))) {
        std::cerr << "Error: dtNavMeshQuery::init failed.\n";
        dtFreeNavMesh(navMesh);
        return 1;
    }

    dtQueryFilter filter;
    filter.setIncludeFlags(0xFFFF);

    if (!hasSeed) seed = (unsigned int)std::time(nullptr);
    std::srand(seed);

    auto frand = []() -> float { return (float)std::rand() / RAND_MAX; };

    std::cout << "=== tnavmesh query random ===\n";

    std::vector<std::pair<float,float>> points;

    for (int attempt = 0; attempt < count * 10 && (int)points.size() < count; attempt++) {
        float randomPos[3];
        dtPolyRef randomRef;
        dtStatus status = query.findRandomPoint(&filter, frand, &randomRef, randomPos);
        if (dtStatusSucceed(status) && randomRef) {
            float tx = randomPos[0];
            float ty = mapHeight - randomPos[2];
            // Apply range filter if specified
            if ((hasMinX && tx < minX) || (hasMaxX && tx > maxX)) continue;
            if ((hasMinY && ty < minY) || (hasMaxY && ty > maxY)) continue;
            points.push_back({tx, ty});
        }
    }

    if (format == "json") {
        std::cout << "{ \"count\": " << points.size() << ", \"points\": [\n";
        for (size_t i = 0; i < points.size(); i++) {
            if (i > 0) std::cout << ",\n";
            std::cout << "  { \"x\": " << points[i].first << ", \"y\": " << points[i].second << " }";
        }
        std::cout << "\n  ] }\n";
    } else {
        std::cout << "\nCount:  " << points.size() << "\n";
        for (size_t i = 0; i < points.size(); i++) {
            std::cout << "  [" << i << "] (" << points[i].first << ", " << points[i].second << ")\n";
        }
        if (points.empty()) {
            std::cout << "\nRandom point: not found\n";
        }
    }

    dtFreeNavMesh(navMesh);
    std::cout << "\n=== Done ===\n";
    return points.empty() ? 1 : 0;
}

// =============================================================
// query raycast
// =============================================================

static void printQueryRaycastHelp() {
    std::cout << "Usage: tnavmesh query raycast -n <navmesh.bin> --start <x> <y> --end <x> <y> [options]\n"
              << "\n"
              << "Required:\n"
              << "  -n, --navmesh <file>       Pre-built navmesh (.bin)\n"
              << "  --start <x> <y>            Start point\n"
              << "  --end <x> <y>              End point\n"
              << "\n"
              << "Output:\n"
              << "  --format <fmt>             text | json (default: text)\n"
              << "\n"
              << "Other:\n"
              << "  -h, --help                 Show this help\n";
}

static int queryRaycast(int argc, char** argv) {
    std::string navPath, format = "text";
    float startX = 0, startY = 0, endX = 0, endY = 0;
    bool hasStart = false, hasEnd = false;

    for (int i = 0; i < argc; i++) {
        std::string a = argv[i];
        if ((a == "-n" || a == "--navmesh") && i+1 < argc) navPath = argv[++i];
        else if (a == "--format" && i+1 < argc) format = argv[++i];
        else if (a == "--start" && i+2 < argc) { startX = std::strtof(argv[++i], nullptr); startY = std::strtof(argv[++i], nullptr); hasStart = true; }
        else if (a == "--end" && i+2 < argc) { endX = std::strtof(argv[++i], nullptr); endY = std::strtof(argv[++i], nullptr); hasEnd = true; }
        else if (a == "-h" || a == "--help") { printQueryRaycastHelp(); return 0; }
        else if (a[0] == '-') { std::cerr << "Warning: unknown option '" << a << "' ignored.\n"; }
    }

    if (navPath.empty() || !hasStart || !hasEnd) {
        std::cerr << "Error: --navmesh, --start, and --end are required.\n";
        printQueryRaycastHelp();
        return 2;
    }

    dtNavMesh* navMesh = nullptr;
    float mapHeight = 0;
    if (!loadQueryNavmesh(navPath, navMesh, mapHeight)) return 3;

    dtNavMeshQuery query;
    if (dtStatusFailed(query.init(navMesh, 2048))) {
        std::cerr << "Error: dtNavMeshQuery::init failed.\n";
        dtFreeNavMesh(navMesh);
        return 1;
    }

    dtQueryFilter filter;
    filter.setIncludeFlags(0xFFFF);
    float halfExtents[3] = { mapHeight * 0.1f, mapHeight * 0.2f, mapHeight * 0.1f };
    float startPos[3] = { startX, 0, mapHeight - startY };
    float endPos[3]   = { endX,   0, mapHeight - endY };

    dtPolyRef startRef, endRef;
    float startNearest[3], endNearest[3];
    query.findNearestPoly(startPos, halfExtents, &filter, &startRef, startNearest);
    query.findNearestPoly(endPos, halfExtents, &filter, &endRef, endNearest);

    std::cout << "=== tnavmesh query raycast ===\n"
              << "Start: (" << startX << ", " << startY << ")\n"
              << "End:   (" << endX << ", " << endY << ")\n";

    if (!startRef || !endRef) {
        if (format == "json") {
            std::cout << "{ \"reachable\": false, \"reason\": \"start or end off navmesh\" }\n";
        } else {
            std::cout << "\nRaycast: unreachable (start or end off navmesh)\n";
        }
        dtFreeNavMesh(navMesh);
        std::cout << "\n=== Done ===\n";
        return 1;
    }

    float hitNormal[3];
    dtPolyRef path[256];
    int pathCount = 0;
    float t = 0;
    dtStatus status = query.raycast(startRef, startNearest, endNearest, &filter, &t, hitNormal, path, &pathCount, 256);

    if (dtStatusSucceed(status)) {
        if (t >= 1.0f) {
            float dx = endNearest[0] - startNearest[0];
            float dz = endNearest[2] - startNearest[2];
            float distance = std::sqrt(dx*dx + dz*dz);
            if (format == "json") {
                std::cout << "{ \"reachable\": true, \"distance\": " << distance << ", \"polyCount\": " << pathCount << " }\n";
            } else {
                std::cout << "\nRaycast: reachable\n"
                          << "  Distance:   " << distance << "\n"
                          << "  PolyCount:  " << pathCount << "\n";
            }
        } else {
            float hitPos[3];
            hitPos[0] = startNearest[0] + (endNearest[0] - startNearest[0]) * t;
            hitPos[2] = startNearest[2] + (endNearest[2] - startNearest[2]) * t;
            float hx = hitPos[0];
            float hy = mapHeight - hitPos[2];
            if (format == "json") {
                std::cout << "{ \"reachable\": false, \"hit\": { \"x\": " << hx << ", \"y\": " << hy << ", \"t\": " << t << " } }\n";
            } else {
                std::cout << "\nRaycast: blocked\n"
                          << "  Hit at:     (" << hx << ", " << hy << ")\n"
                          << "  Fraction:   " << t << "\n";
            }
        }
    } else {
        if (format == "json") {
            std::cout << "{ \"reachable\": false, \"reason\": \"query failed\" }\n";
        } else {
            std::cout << "\nRaycast: query failed\n";
        }
    }

    dtFreeNavMesh(navMesh);
    std::cout << "\n=== Done ===\n";
    return 0;
}

// =============================================================
// cmd_query — dispatcher
// =============================================================

int cmd_query(int argc, char** argv) {
    if (argc < 1) {
        printQueryHelp();
        return 2;
    }

    std::string sub = argv[0];
    int subArgc = argc - 1;
    char** subArgv = argv + 1;

    if (sub == "path")    return queryPath(subArgc, subArgv);
    if (sub == "nearest") return queryNearest(subArgc, subArgv);
    if (sub == "random")  return queryRandom(subArgc, subArgv);
    if (sub == "raycast") return queryRaycast(subArgc, subArgv);
    if (sub == "-h" || sub == "--help") { printQueryHelp(); return 0; }

    std::cerr << "Error: unknown subcommand '" << sub << "'.\n";
    printQueryHelp();
    return 2;
}

// =============================================================
// render
// =============================================================

static void printRenderHelp() {
    std::cout << "Usage: tnavmesh render -n <navmesh.bin> -i <input> -o <output.svg> [options]\n"
              << "       tnavmesh render -n <navmesh.bin> --points \"<x,y x,y ...>\" -o <output.svg>\n"
              << "\n"
              << "Required:\n"
              << "  -n, --navmesh <file>       Pre-built navmesh (.bin)\n"
              << "\n"
              << "Input (mutually exclusive):\n"
              << "  -i, --input <file>         Read waypoints from txt/json file\n"
              << "  --points \"<x,y x,y ...>\"    Inline waypoints\n"
              << "\n"
              << "Output:\n"
              << "  -o, --output <file>         Output file (default: render.svg)\n"
              << "  --format <fmt>              svg (default)\n"
              << "\n"
              << "Visualization:\n"
              << "  --visual <level>            simple | full | debug (default: full)\n"
              << "    simple   Path line only\n"
              << "    full     Path + start/end markers + navmesh\n"
              << "    debug    Extra annotations\n"
              << "\n"
              << "Other:\n"
              << "  -v, --verbose               Verbose output\n"
              << "  -h, --help                  Show this help\n";
}

static std::vector<float> readWaypointsFile(const std::string& path) {
    std::vector<float> pts;
    std::ifstream ifs(path);
    if (!ifs) return pts;

    std::string ext;
    size_t dot = path.find_last_of('.');
    if (dot != std::string::npos) ext = path.substr(dot);

    if (ext == ".json") {
        // Simple JSON waypoints reader: look for "x" and "y" pairs
        std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
        // Find all "x":<num> or "x": <num> patterns
        size_t pos = 0;
        while ((pos = content.find("\"x\"", pos)) != std::string::npos) {
            pos = content.find(':', pos);
            if (pos == std::string::npos) break;
            pos++;
            float x = std::strtof(content.c_str() + pos, nullptr);
            size_t ypos = content.find("\"y\"", pos);
            if (ypos == std::string::npos) break;
            ypos = content.find(':', ypos);
            if (ypos == std::string::npos) break;
            ypos++;
            float y = std::strtof(content.c_str() + ypos, nullptr);
            pts.push_back(x);
            pts.push_back(y);
        }
    } else {
        // Text: one "x y" per line
        float x, y;
        while (ifs >> x >> y) {
            pts.push_back(x);
            pts.push_back(y);
        }
    }
    return pts;
}

static std::vector<float> parsePointsString(const std::string& s) {
    std::vector<float> pts;
    std::stringstream ss(s);
    std::string token;
    while (ss >> token) {
        size_t comma = token.find(',');
        if (comma == std::string::npos) continue;
        float x = std::strtof(token.c_str(), nullptr);
        float y = std::strtof(token.c_str() + comma + 1, nullptr);
        pts.push_back(x);
        pts.push_back(y);
    }
    return pts;
}

int cmd_render(int argc, char** argv) {
    std::string inputPath;
    std::string pointsStr;
    std::string outputPath = "render.svg";
    std::string navPath;
    std::string format = "svg";
    std::string visual = "full";
    bool verbose = false;
    bool hasInput = false, hasPoints = false;

    for (int i = 0; i < argc; i++) {
        std::string a = argv[i];
        if ((a == "-i" || a == "--input") && i+1 < argc) { inputPath = argv[++i]; hasInput = true; }
        else if (a == "--points" && i+1 < argc) { pointsStr = argv[++i]; hasPoints = true; }
        else if ((a == "-n" || a == "--navmesh") && i+1 < argc) navPath = argv[++i];
        else if ((a == "-o" || a == "--output") && i+1 < argc) outputPath = argv[++i];
        else if (a == "--format" && i+1 < argc) format = argv[++i];
        else if (a == "--visual" && i+1 < argc) visual = argv[++i];
        else if (a == "-v" || a == "--verbose") verbose = true;
        else if (a == "-h" || a == "--help") { printRenderHelp(); return 0; }
        else if (a[0] == '-') { std::cerr << "Warning: unknown option '" << a << "' ignored.\n"; }
    }

    if (hasInput && hasPoints) {
        std::cerr << "Error: --input and --points are mutually exclusive.\n";
        return 2;
    }
    if (!hasInput && !hasPoints) {
        std::cerr << "Error: one of --input or --points is required.\n";
        printRenderHelp();
        return 2;
    }

    std::vector<float> pts;
    if (hasInput) {
        pts = readWaypointsFile(inputPath);
        if (pts.empty()) {
            std::cerr << "Error: no waypoints read from " << inputPath << "\n";
            return 2;
        }
    } else {
        pts = parsePointsString(pointsStr);
        if (pts.empty()) {
            std::cerr << "Error: no valid waypoints in --points string.\n";
            return 2;
        }
    }

    if (navPath.empty()) {
        std::cerr << "Error: --navmesh is required.\n";
        printRenderHelp();
        return 2;
    }

    std::cout << "=== tnavmesh render ===\n"
              << "Points: " << (pts.size()/2) << "\n"
              << "NavMesh: " << navPath << "\n"
              << "Output: " << outputPath << "\n";

    // Load navmesh
    float mapHeight = 0;
    dtNavMesh* navMesh = nullptr;
    rcPolyMesh* qmesh = nullptr;
    {
        navMesh = loadDetourNavMesh(navPath.c_str());
        if (!navMesh) {
            std::cerr << "Error: failed to load navmesh from " << navPath << "\n";
            return 3;
        }
        const dtNavMesh* cnm = navMesh;
        const dtMeshTile* tile = cnm->getTile(0);
        if (tile && tile->header) mapHeight = tile->header->bmax[2];
        qmesh = detourToPolyMesh(navMesh, mapHeight);
    }

    // Calculate map bounds from both waypoints and navmesh
    float minX = pts[0], maxX = pts[0], minY = pts[1], maxY = pts[1];
    for (size_t i = 0; i < pts.size()/2; i++) {
        if (pts[i*2] < minX) minX = pts[i*2];
        if (pts[i*2] > maxX) maxX = pts[i*2];
        if (pts[i*2+1] < minY) minY = pts[i*2+1];
        if (pts[i*2+1] > maxY) maxY = pts[i*2+1];
    }
    if (qmesh) {
        if (qmesh->bmin[0] < minX) minX = qmesh->bmin[0];
        if (qmesh->bmax[0] > maxX) maxX = qmesh->bmax[0];
        if (qmesh->bmin[2] < minY) minY = qmesh->bmin[2];
        if (qmesh->bmax[2] > maxY) maxY = qmesh->bmax[2];
    }

    float pad = std::max(20.0f, (maxX - minX + maxY - minY) * 0.1f);

    // Shift waypoints to origin
    for (size_t i = 0; i < pts.size(); i += 2) {
        pts[i]   -= minX - pad;
        pts[i+1] -= minY - pad;
    }

    // Shift navmesh bounds to match
    if (qmesh) {
        qmesh->bmin[0] -= minX - pad;
        qmesh->bmin[2] -= minY - pad;
        qmesh->bmax[0] -= minX - pad;
        qmesh->bmax[2] -= minY - pad;
    }

    MapInfo mapInfo;
    mapInfo.width = 1;
    mapInfo.height = 1;
    mapInfo.tileWidth = (maxX - minX) + pad * 2;
    mapInfo.tileHeight = (maxY - minY) + pad * 2;

    SvgOptions opt;
    opt.svgWidth = 1200;
    opt.svgHeight = 1200;
    opt.showPath = true;
    opt.showNavmesh = (qmesh != nullptr);
    opt.tmxCoords = true;

    if (visual == "simple") {
        // Path line only
    } else if (visual == "debug") {
        opt.showAnnotations = true;
    }

    writeSVG(outputPath, mapInfo, {}, {}, qmesh, opt, pts);
    rcFreePolyMesh(qmesh);
    dtFreeNavMesh(navMesh);
    std::cout << "=== Done ===\n";
    return 0;
}

// =============================================================
// inspect
// =============================================================

static void printInspectHelp() {
    std::cout << "Usage: tnavmesh inspect -n <navmesh.bin> [options]\n"
              << "\n"
              << "Options:\n"
              << "  -n, --navmesh <file>       Pre-built navmesh (.bin)\n"
              << "  -o, --output <file.svg>    Export navmesh visualization (SVG)\n"
              << "\n"
              << "Other:\n"
              << "  -v, --verbose              Verbose output\n"
              << "  -h, --help                 Show this help\n";
}

int cmd_inspect(int argc, char** argv) {
    std::string navPath;
    std::string outputPath;
    bool verbose = false;

    for (int i = 0; i < argc; i++) {
        std::string a = argv[i];
        if (a == "-n" && i+1 < argc) navPath = argv[++i];
        else if (a == "--navmesh" && i+1 < argc) navPath = argv[++i];
        else if ((a == "-o" || a == "--output") && i+1 < argc) outputPath = argv[++i];
        else if (a == "-v" || a == "--verbose") verbose = true;
        else if (a == "-h" || a == "--help") { printInspectHelp(); return 0; }
        else if (a[0] == '-') { std::cerr << "Warning: unknown option '" << a << "' ignored.\n"; }
    }

    if (navPath.empty()) {
        std::cerr << "Error: --navmesh is required.\n";
        printInspectHelp();
        return 2;
    }

    dtNavMesh* navMesh = loadDetourNavMesh(navPath.c_str());
    if (!navMesh) {
        std::cerr << "Error: failed to load navmesh from " << navPath << "\n";
        return 1;
    }

    int totalVerts = 0, totalPolys = 0;
    float bmin[3] = {1e30f, 1e30f, 1e30f};
    float bmax[3] = {-1e30f, -1e30f, -1e30f};
    int tileCount = 0;

    const dtNavMesh* constCMesh = navMesh;

    for (int i = 0; i < constCMesh->getMaxTiles(); i++) {
        const dtMeshTile* tile = constCMesh->getTile(i);
        if (!tile || !tile->header) continue;
        tileCount++;
        totalVerts += tile->header->vertCount;
        totalPolys += tile->header->polyCount;
        for (int j = 0; j < 3; j++) {
            if (tile->header->bmin[j] < bmin[j]) bmin[j] = tile->header->bmin[j];
            if (tile->header->bmax[j] > bmax[j]) bmax[j] = tile->header->bmax[j];
        }
    }

    std::cout << "=== tnavmesh inspect ===\n"
              << "NavMesh: " << navPath << "\n"
              << "  Vertices:    " << totalVerts << "\n"
              << "  Polygons:    " << totalPolys << "\n"
              << "  Bounds:      (" << bmin[0] << ", " << bmin[2] << ") to ("
              << bmax[0] << ", " << bmax[2] << ")\n"
              << "  Tiles:       " << tileCount << "\n";

    // Estimate memory from navmesh data
    const unsigned char* navData = nullptr;
    int navDataSize = 0;
    if (getNavMeshData(navMesh, navData, navDataSize)) {
        std::cout << "  Memory:      " << (navDataSize / 1024) << " KB\n";
    }

    if (verbose) {
        std::cout << "  Config:      maxVertsPerPoly=" << 6 << "\n";
    }

    // SVG export
    if (!outputPath.empty()) {
        // For SVG, create a simple MapInfo from bounds
        float w = bmax[0] - bmin[0];
        float h = bmax[2] - bmin[2];
        MapInfo mapInfo;
        mapInfo.width = 1;
        mapInfo.height = 1;
        mapInfo.tileWidth = w > 0 ? w : 1;
        mapInfo.tileHeight = h > 0 ? h : 1;

        SvgOptions opt;
        opt.svgWidth = 1200;
        opt.svgHeight = 1200;
        opt.showNavmesh = true;

        // We don't have rcPolyMesh here, but we can still render by
        // converting dtNavMesh tiles to a temporary rcPolyMesh-like structure
        // For now, write a message
        std::cout << "SVG export not yet supported for inspect (use build --svg-output instead)\n";
    }

    dtFreeNavMesh(navMesh);
    std::cout << "=== Done ===\n";
    return 0;
}
