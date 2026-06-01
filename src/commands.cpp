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

// --- JSON output helpers ---

static void writeJSONWaypoints(const std::vector<float>& pts, std::ostream& os) {
    os << "{\n  \"waypoints\": [\n";
    for (size_t i = 0; i < pts.size() / 2; i++) {
        if (i > 0) os << ",\n";
        os << "    { \"x\": " << pts[i*2] << ", \"y\": " << pts[i*2+1] << " }";
    }
    os << "\n  ]\n";
    if (!pts.empty()) {
        float len = 0;
        for (size_t i = 1; i < pts.size()/2; i++) {
            float dx = pts[i*2] - pts[(i-1)*2];
            float dy = pts[i*2+1] - pts[(i-1)*2+1];
            len += std::sqrt(dx*dx + dy*dy);
        }
        os << ",\n  \"length\": " << len << "\n";
    }
    os << "}\n";
}

static void writeJSONObstacles(const std::vector<Obstacle>& obstacles, std::ostream& os) {
    os << "\"obstacles\": [\n";
    for (size_t i = 0; i < obstacles.size(); i++) {
        if (i > 0) os << ",\n";
        os << "  { \"id\": " << obstacles[i].id << ", \"points\": [";
        for (size_t j = 0; j < obstacles[i].points.size() / 2; j++) {
            if (j > 0) os << ", ";
            os << "{ \"x\": " << obstacles[i].points[j*2] << ", \"y\": " << obstacles[i].points[j*2+1] << " }";
        }
        os << "] }";
    }
    os << "\n  ]\n";
}

static void writeJSONMerged(const std::vector<MergedRegion>& merged, std::ostream& os) {
    os << "\"mergedRegions\": [\n";
    for (size_t i = 0; i < merged.size(); i++) {
        if (i > 0) os << ",\n";
        os << "  { \"exterior\": [";
        for (size_t j = 0; j < merged[i].exterior.points.size() / 2; j++) {
            if (j > 0) os << ", ";
            os << "{ \"x\": " << merged[i].exterior.points[j*2] << ", \"y\": " << merged[i].exterior.points[j*2+1] << " }";
        }
        os << "]";
        // Holes (interior rings)
        if (!merged[i].holes.empty()) {
            os << ", \"holes\": [\n";
            for (size_t h = 0; h < merged[i].holes.size(); h++) {
                if (h > 0) os << ",\n";
                os << "    [";
                for (size_t j = 0; j < merged[i].holes[h].points.size() / 2; j++) {
                    if (j > 0) os << ", ";
                    os << "{ \"x\": " << merged[i].holes[h].points[j*2] << ", \"y\": " << merged[i].holes[h].points[j*2+1] << " }";
                }
                os << "]";
            }
            os << "\n  ]";
        }
        os << " }";
    }
    os << "\n  ]\n";
}

static void writeJSONOverlaps(const std::vector<Obstacle>& overlaps, std::ostream& os) {
    os << "\"overlaps\": [\n";
    for (size_t i = 0; i < overlaps.size(); i++) {
        if (i > 0) os << ",\n";
        os << "  { \"points\": [";
        for (size_t j = 0; j < overlaps[i].points.size() / 2; j++) {
            if (j > 0) os << ", ";
            os << "{ \"x\": " << overlaps[i].points[j*2] << ", \"y\": " << overlaps[i].points[j*2+1] << " }";
        }
        os << "] }";
    }
    os << "\n  ]\n";
}

// --- Help text functions ---

void printUsage() {
    std::cout << TNAVMESH_NAME << " v" << TNAVMESH_VERSION << "\n"
              << "Usage: tnavmesh <command> [options]\n"
              << "\n"
              << "Commands:\n"
              << "  build     Generate navigation mesh from TMX\n"
              << "  path      Compute or render navigation paths\n"
              << "\n"
              << "Run 'tnavmesh <command> --help' for command-specific help.\n";
}

// ---------- build ----------

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
              << "  --cell-size <float>        Voxel size (default: tileSize * 0.15)\n"
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
              << "                              Larger value = fewer triangles\n"
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
    std::string svgPath;       // optional SVG output
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
        else if (a == "--resolution" && i+1 < argc) { std::string q = argv[++i]; if (q == "low") quality = Quality::Low; else if (q == "high") quality = Quality::High; else quality = Quality::Normal; }
        else if (a == "--agent-radius" && i+1 < argc) navCfg.walkableRadius = std::atoi(argv[++i]);
        else if (a == "--agent-height" && i+1 < argc) navCfg.walkableHeight = std::atoi(argv[++i]);
        else if (a == "--agent-climb" && i+1 < argc) navCfg.walkableClimb = std::atoi(argv[++i]);
        else if (a == "--cell-size" && i+1 < argc) navCfg.cs = std::strtof(argv[++i], nullptr);
        else if (a == "--cell-height" && i+1 < argc) navCfg.ch = std::strtof(argv[++i], nullptr);
        else if (a == "--poly-max-verts" && i+1 < argc) {
            int v = std::atoi(argv[++i]);
            if (v > 6) {
                std::cerr << "Warning: --poly-max-verts limited to 6 (Detour constraint), clamping.\n";
                v = 6;
            }
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
        else if (a[0] == '-') {
            std::cerr << "Warning: unknown option '" << a << "' ignored.\n";
        }
    }

    if (inputPath.empty() || outputPath.empty()) {
        std::cerr << "Error: --input and --output are required.\n";
        printBuildHelp();
        return 2;
    }

    navCfg.applyQuality(quality);

    std::cout << "=== tnavmesh build ===\n"
              << "Input:  " << inputPath << "\n"
              << "Output: " << outputPath << "\n";

    MapInfo mapInfo;
    std::vector<Obstacle> obstacles;
    if (!parseTMX(inputPath, mapInfo, obstacles)) {
        std::cerr << "Error: failed to parse TMX file.\n";
        return 2;
    }

    if (obstacles.empty()) {
        std::cout << "  (no obstacles found — building full-map flat navmesh)\n";
    }

    float mapW = static_cast<float>(mapInfo.width) * mapInfo.tileWidth;
    float mapH = static_cast<float>(mapInfo.height) * mapInfo.tileHeight;

    navCfg.autoCalc(mapInfo);
    if (verbose) {
        std::cout << "Config: cs=" << navCfg.cs << " ch=" << navCfg.ch
                  << " agent-radius=" << navCfg.walkableRadius
                  << " agent-height=" << navCfg.walkableHeight
                  << " agent-climb=" << navCfg.walkableClimb << "\n";
    }

    auto merged = mergeObstacles(obstacles, mapW, mapH);

    rcPolyMesh* mesh = buildNavMesh(mapInfo, merged, navCfg);
    if (!mesh) {
        std::cerr << "Error: navmesh build failed.\n";
        return 1;
    }

    int nv = mesh->nverts;
    int np = mesh->npolys;
    std::cout << "NavMesh: " << nv << " verts, " << np << " polys\n";

    // Build statistics
    {
        std::string qStr = "normal";
        switch (quality) { case Quality::Low: qStr = "low"; break; case Quality::High: qStr = "high"; break; default: break; }
        std::string pStr = "watershed";
        switch (navCfg.partitionType) { case PartitionType::Monotone: pStr = "monotone"; break; case PartitionType::Layer: pStr = "layers"; break; default: break; }
        std::cout << "Stats: resolution=" << qStr << " partition=" << pStr
                  << " cs=" << navCfg.cs << " ch=" << navCfg.ch
                  << " agent-radius=" << navCfg.walkableRadius
                  << " agent-height=" << navCfg.walkableHeight
                  << " agent-climb=" << navCfg.walkableClimb << "\n";
    }

    if (nv == 0 || np == 0) {
        std::cerr << "Error: navmesh is empty (no walkable area found).\n";
        rcFreePolyMesh(mesh);
        return 1;
    }

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

    dtFreeNavMesh(navMesh);

    // ----- SVG output (optional) -----
    if (!svgPath.empty()) {
        SvgOptions opt;
        opt.svgWidth = svgWidth;
        opt.svgHeight = svgHeight;
        opt.showLegend = !svgNoLegend;

        // Default layers: merged + navmesh
        if (!hasSvgLayer) {
            svgLayers = {"merged", "navmesh"};
        }

        // Set all layer toggles to false, then enable requested ones
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

        std::cout << "SVG output: " << svgPath << " (layers:";
        for (const auto& l : svgLayers) std::cout << " " << l;
        std::cout << ")\n";

        std::vector<Obstacle> overlaps;
        std::vector<float> annotations;

        if (opt.showOverlaps) {
            overlaps = computeOverlaps(obstacles);
            if (verbose) {
                std::cout << "Overlaps: " << overlaps.size() << " regions\n";
            }
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
        std::cout << "SVG written to " << svgPath << "\n";
    }

    rcFreePolyMesh(mesh);
    std::cout << "=== Done ===\n";
    return 0;
}

// ---------- path ----------

static void printPathHelp() {
    std::cout << "Usage: tnavmesh path [options]\n"
              << "\n"
              << "Input modes (mutually exclusive):\n"
              << "  -n, --navmesh <file.bin>   Pre-built navmesh (runtime query)\n"
              << "  --draw <file.txt>          Render precomputed path (no navmesh)\n"
              << "\n"
              << "Points:\n"
              << "  -s, --start <x> <y>        Start point (navmesh coords)\n"
              << "  -e, --end <x> <y>          End point\n"
              << "  --auto                     Auto-generate start/end points\n"
              << "  (--start/--end or --auto required)\n"
              << "\n"
              << "Output:\n"
              << "  --output-svg <file>        Output SVG (default: path.svg)\n"
              << "  --text-output <file>       Save waypoints as text\n"
              << "  --format <fmt>             svg | json | text (default: svg)\n"
              << "\n"
              << "Visualization:\n"
              << "  --visual <level>           simple | full | debug (default: full)\n"
              << "    simple   Path line only\n"
              << "    full     Path + start/end markers\n"
              << "    debug    Extra annotations\n"
              << "\n"
              << "Other:\n"
              << "  -v, --verbose              Verbose output\n"
              << "  -h, --help                 Show this help\n";
}

static std::vector<float> readPathFile(const std::string& path) {
    std::vector<float> pts;
    std::ifstream ifs(path);
    if (!ifs) {
        std::cerr << "Error: cannot open " << path << "\n";
        return pts;
    }
    float x, y;
    while (ifs >> x >> y) {
        pts.push_back(x);
        pts.push_back(y);
    }
    return pts;
}

int cmd_path(int argc, char** argv) {
    std::string navPath;      // -n .bin
    std::string drawPath;     // --draw .txt
    std::string outputSvg = "path.svg";
    std::string textOutput;
    std::string visual = "full";
    std::string format = "svg";
    float startX = 0, startY = 0, endX = 0, endY = 0;
    bool hasStart = false, hasEnd = false, autoMode = false;
    bool verbose = false;
    NavmeshConfig navCfg;

    for (int i = 0; i < argc; i++) {
        std::string a = argv[i];
        if (a == "-n" && i+1 < argc) navPath = argv[++i];
        else if (a == "--navmesh" && i+1 < argc) navPath = argv[++i];
        else if (a == "--draw" && i+1 < argc) drawPath = argv[++i];
        else if (a == "--output-svg" && i+1 < argc) outputSvg = argv[++i];
        else if (a == "--text-output" && i+1 < argc) textOutput = argv[++i];
        else if (a == "--visual" && i+1 < argc) visual = argv[++i];
        else if (a == "--format" && i+1 < argc) format = argv[++i];
        else if ((a == "-o" || a == "--output") && i+1 < argc) outputSvg = argv[++i];
        else if (a == "-s" && i+2 < argc) { startX = std::strtof(argv[++i], nullptr); startY = std::strtof(argv[++i], nullptr); hasStart = true; }
        else if (a == "--start" && i+2 < argc) { startX = std::strtof(argv[++i], nullptr); startY = std::strtof(argv[++i], nullptr); hasStart = true; }
        else if (a == "-e" && i+2 < argc) { endX = std::strtof(argv[++i], nullptr); endY = std::strtof(argv[++i], nullptr); hasEnd = true; }
        else if (a == "--end" && i+2 < argc) { endX = std::strtof(argv[++i], nullptr); endY = std::strtof(argv[++i], nullptr); hasEnd = true; }
        else if (a == "--auto") autoMode = true;
        else if (a == "-v" || a == "--verbose") verbose = true;
        else if (a == "-h" || a == "--help") { printPathHelp(); return 0; }
        else if (a[0] == '-') {
            std::cerr << "Warning: unknown option '" << a << "' ignored.\n";
        }
    }

    // Determine mode
    int modeCount = (!navPath.empty() ? 1 : 0) + (!drawPath.empty() ? 1 : 0);
    if (modeCount == 0) {
        std::cerr << "Error: one of --navmesh or --draw is required.\n";
        printPathHelp();
        return 2;
    }
    if (modeCount > 1) {
        std::cerr << "Error: --navmesh and --draw are mutually exclusive.\n";
        return 2;
    }

    SvgOptions opt;
    opt.showGrid = true;
    opt.showObstacles = true;
    opt.showMerged = true;
    opt.showNavmesh = true;
    opt.showPath = true;

    if (visual == "simple") {
        opt.showGrid = false;
        opt.showObstacles = false;
        opt.showMerged = false;
        opt.showNavmesh = false;
    } else if (visual == "debug") {
        opt.showAnnotations = true;
    }

    // ----- Mode: --draw (pure path rendering) -----
    if (!drawPath.empty()) {
        std::vector<float> pts = readPathFile(drawPath);
        if (pts.empty()) {
            std::cerr << "Error: no waypoints read from " << drawPath << "\n";
            return 2;
        }
        std::cout << "=== tnavmesh path (draw) ===\n"
                  << "Input: " << drawPath << "\n"
                  << "Points: " << (pts.size()/2) << "\n";

        opt.showGrid = false;
        opt.showObstacles = false;
        opt.showMerged = false;
        opt.showNavmesh = false;

        // For pure rendering, create a dummy mapInfo at natural scale
        float minX = pts[0], maxX = pts[0], minY = pts[1], maxY = pts[1];
        for (size_t i = 0; i < pts.size()/2; i++) {
            if (pts[i*2] < minX) minX = pts[i*2];
            if (pts[i*2] > maxX) maxX = pts[i*2];
            if (pts[i*2+1] < minY) minY = pts[i*2+1];
            if (pts[i*2+1] > maxY) maxY = pts[i*2+1];
        }
        MapInfo mapInfo;
        mapInfo.width = 1;
        mapInfo.height = 1;
        mapInfo.tileWidth = maxX - minX + 20;
        mapInfo.tileHeight = maxY - minY + 20;

        writeSVG(outputSvg, mapInfo, {}, {}, nullptr, opt, pts);
        std::cout << "SVG written to " << outputSvg << "\n";
        if (verbose) {
            std::cout << "Waypoints (" << (pts.size()/2) << "):\n";
            for (size_t i = 0; i < pts.size()/2; i++)
                std::cout << "  " << pts[i*2] << " " << pts[i*2+1] << "\n";
        }
        std::cout << "=== Done ===\n";
        return 0;
    }

    // ----- Runtime mode (-n .bin) -----
    if (!navPath.empty()) {
        if (!hasStart || !hasEnd) {
            std::cerr << "Error: --start and --end are required for runtime mode.\n";
            printPathHelp();
            return 2;
        }

        std::cout << "=== tnavmesh path (runtime) ===\n"
                  << "NavMesh: " << navPath << "\n"
                  << "Start:   (" << startX << ", " << startY << ")\n"
                  << "End:     (" << endX << ", " << endY << ")\n";

        dtNavMesh* navMesh = loadDetourNavMesh(navPath.c_str());
        if (!navMesh) {
            std::cerr << "Error: failed to load navmesh from " << navPath << "\n";
            return 1;
        }

        // Derive map height from navmesh bounds for TMX↔Detour conversion
        float mapHeight = 0;
        const dtMeshTile* tile = static_cast<const dtNavMesh*>(navMesh)->getTile(0);
        if (tile && tile->header) {
            mapHeight = tile->header->bmax[2];
        }

        float searchRadius = std::max(50.0f, mapHeight * 0.05f);
        PathResult result = findPath(navMesh, startX, startY, endX, endY, mapHeight, searchRadius);

        dtFreeNavMesh(navMesh);

        if (result.found && result.waypoints.size() >= 4) {
            std::cout << "Path found: " << (result.waypoints.size()/2)
                      << " waypoints, length=" << result.totalLength << "\n";
        } else {
            std::string reason = result.error.empty() ? "degenerate path (start/end coincide)" : result.error;
            std::cout << "Path not found: " << reason << "\n";
        }

        if (!textOutput.empty() && !result.waypoints.empty()) {
            std::ofstream ofs(textOutput);
            if (ofs) {
                for (size_t i = 0; i < result.waypoints.size()/2; i++)
                    ofs << result.waypoints[i*2] << " " << result.waypoints[i*2+1] << "\n";
                std::cout << "Waypoints written to " << textOutput << "\n";
            }
        }

        // Output in requested format (always, even if path failed)
        if (format == "json") {
            std::ofstream ofs(outputSvg);
            if (ofs) {
                writeJSONWaypoints(result.waypoints, ofs);
                std::cout << "JSON written to " << outputSvg << "\n";
            }
        } else if (format == "text") {
            std::ofstream ofs(outputSvg);
            if (ofs) {
                for (size_t i = 0; i < result.waypoints.size()/2; i++)
                    ofs << result.waypoints[i*2] << " " << result.waypoints[i*2+1] << "\n";
                std::cout << "Waypoints written to " << outputSvg << "\n";
            }
        } else {
            // SVG output — construct a simple mapInfo from waypoints bounds (or start/end if no path)
            float minX, maxX, minY, maxY;
            if (!result.waypoints.empty()) {
                minX = maxX = result.waypoints[0];
                minY = maxY = result.waypoints[1];
                for (size_t i = 0; i < result.waypoints.size() / 2; i++) {
                    if (result.waypoints[i*2] < minX) minX = result.waypoints[i*2];
                    if (result.waypoints[i*2] > maxX) maxX = result.waypoints[i*2];
                    if (result.waypoints[i*2+1] < minY) minY = result.waypoints[i*2+1];
                    if (result.waypoints[i*2+1] > maxY) maxY = result.waypoints[i*2+1];
                }
            } else {
                minX = std::min(startX, endX);
                maxX = std::max(startX, endX);
                minY = std::min(startY, endY);
                maxY = std::max(startY, endY);
            }
            float pad = std::max(20.0f, (maxX - minX + maxY - minY) * 0.1f);
            MapInfo rtMap;
            rtMap.width = 1;
            rtMap.height = 1;
            rtMap.tileWidth = maxX - minX + pad * 2;
            rtMap.tileHeight = maxY - minY + pad * 2;

            SvgOptions rtOpt;
            rtOpt.showGrid = false;
            rtOpt.showObstacles = false;
            rtOpt.showMerged = false;
            rtOpt.showNavmesh = false;
            rtOpt.showPath = true;

            writeSVG(outputSvg, rtMap, {}, {}, nullptr, rtOpt, result.waypoints);
            std::cout << "SVG written to " << outputSvg << "\n";
        }

        std::cout << "=== Done ===\n";
        bool pathOk = result.found && result.waypoints.size() >= 4;
        return pathOk ? 0 : 1;
    }

    return 1; // unreachable
}
