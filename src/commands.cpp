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
              << "  inspect   Visual debugging of the navigation pipeline\n"
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
              << "\n"
              << "Output:\n"
              << "  -o, --output <file>        Output binary navmesh (.bin)\n"
              << "\n"
              << "Quality preset:\n"
              << "  --quality <level>          low | normal | high (default: normal)\n"
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
              << "  --poly-max-verts <n>       Max vertices per polygon (default: 6)\n"
              << "  --poly-simplify <float>    Simplification error (default: 2.0)\n"
              << "  --max-edges <n>            Max edge length in pixels (default: auto)\n"
              << "\n"
              << "Other:\n"
              << "  -v, --verbose              Verbose output\n"
              << "  -h, --help                 Show this help\n"
              << "  --slope-angle <float>      Max walkable slope in degrees (default: 45)\n";
}

int cmd_build(int argc, char** argv) {
    std::string inputPath;
    std::string outputPath;
    bool verbose = false;
    NavmeshConfig navCfg;
    Quality quality = Quality::Normal;

    for (int i = 0; i < argc; i++) {
        std::string a = argv[i];
        if (a == "-i" && i+1 < argc) inputPath = argv[++i];
        else if (a == "--input" && i+1 < argc) inputPath = argv[++i];
        else if (a == "-o" && i+1 < argc) outputPath = argv[++i];
        else if (a == "--output" && i+1 < argc) outputPath = argv[++i];
        else if (a == "--quality" && i+1 < argc) { std::string q = argv[++i]; if (q == "low") quality = Quality::Low; else if (q == "high") quality = Quality::High; else quality = Quality::Normal; }
        else if (a == "--agent-radius" && i+1 < argc) navCfg.walkableRadius = std::atoi(argv[++i]);
        else if (a == "--agent-height" && i+1 < argc) navCfg.walkableHeight = std::atoi(argv[++i]);
        else if (a == "--agent-climb" && i+1 < argc) navCfg.walkableClimb = std::atoi(argv[++i]);
        else if (a == "--cell-size" && i+1 < argc) navCfg.cs = std::strtof(argv[++i], nullptr);
        else if (a == "--cell-height" && i+1 < argc) navCfg.ch = std::strtof(argv[++i], nullptr);
        else if (a == "--poly-max-verts" && i+1 < argc) navCfg.maxVertsPerPoly = std::atoi(argv[++i]);
        else if (a == "--poly-simplify" && i+1 < argc) navCfg.maxSimplificationError = std::strtof(argv[++i], nullptr);
        else if (a == "--max-edges" && i+1 < argc) navCfg.maxEdgeLen = std::atoi(argv[++i]);
        else if (a == "--slope-angle" && i+1 < argc) navCfg.walkableSlopeAngle = std::strtof(argv[++i], nullptr);
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
    rcFreePolyMesh(mesh);
    std::cout << "=== Done ===\n";
    return 0;
}

// ---------- inspect ----------

static void printInspectHelp() {
    std::cout << "Usage: tnavmesh inspect -i <input.tmx> -o <output> [options]\n"
              << "\n"
              << "Required:\n"
              << "  -i, --input <file>         Input TMX file\n"
              << "\n"
              << "Output:\n"
              << "  -o, --output <file>        Output file (default: debug.svg)\n"
              << "  --format <fmt>             svg | json | text (default: svg)\n"
              << "\n"
              << "Mode:\n"
              << "  --mode <level>             minimal | normal | full (default: normal)\n"
              << "    minimal    Raw obstacles only\n"
              << "    normal     Obstacles + merged regions\n"
              << "    full       Full pipeline: triangulation + navmesh + annotations\n"
              << "\n"
              << "Other:\n"
              << "  --width <n>                SVG output width in px (SVG format only)\n"
              << "  --height <n>               SVG output height in px\n"
              << "  -v, --verbose              Verbose output\n"
              << "  -h, --help                 Show this help\n";
}

int cmd_inspect(int argc, char** argv) {
    std::string inputPath;
    std::string outputPath = "debug.svg";
    std::string mode = "normal";
    std::string format = "svg";
    bool verbose = false;
    bool debugMode = false;
    int svgWidth = 0, svgHeight = 0;

    for (int i = 0; i < argc; i++) {
        std::string a = argv[i];
        if (a == "-i" && i+1 < argc) inputPath = argv[++i];
        else if (a == "--input" && i+1 < argc) inputPath = argv[++i];
        else if (a == "-o" && i+1 < argc) outputPath = argv[++i];
        else if (a == "--output" && i+1 < argc) outputPath = argv[++i];
        else if (a == "--mode" && i+1 < argc) mode = argv[++i];
        else if (a == "--format" && i+1 < argc) format = argv[++i];
        else if (a == "--width" && i+1 < argc) svgWidth = std::atoi(argv[++i]);
        else if (a == "--height" && i+1 < argc) svgHeight = std::atoi(argv[++i]);
        else if (a == "-v" || a == "--verbose") verbose = true;
        else if (a == "--debug") debugMode = true;
        else if (a == "-h" || a == "--help") { printInspectHelp(); return 0; }
        else if (a[0] == '-') {
            std::cerr << "Warning: unknown option '" << a << "' ignored.\n";
        }
    }

    if (inputPath.empty()) {
        std::cerr << "Error: --input is required.\n";
        printInspectHelp();
        return 2;
    }

    std::cout << "=== tnavmesh inspect ===\n"
              << "Input:  " << inputPath << "\n"
              << "Output: " << outputPath << "\n"
              << "Mode:   " << mode << "\n"
              << "Format: " << format << "\n";

    MapInfo mapInfo;
    std::vector<Obstacle> obstacles;
    if (!parseTMX(inputPath, mapInfo, obstacles)) {
        std::cerr << "Error: failed to parse TMX file.\n";
        return 2;
    }

    if (obstacles.empty()) {
        std::cout << "  (no obstacles found — full map area is walkable)\n";
    }

    float mapW = static_cast<float>(mapInfo.width) * mapInfo.tileWidth;
    float mapH = static_cast<float>(mapInfo.height) * mapInfo.tileHeight;

    SvgOptions opt;
    opt.svgWidth = svgWidth;
    opt.svgHeight = svgHeight;
    if (debugMode) opt.debug = true;

    std::vector<Obstacle> overlaps;
    std::vector<MergedRegion> merged;
    rcPolyMesh* mesh = nullptr;
    std::vector<float> annotations;

    if (mode == "minimal") {
        opt.showGrid = true;
        opt.showObstacles = true;
        opt.showMerged = false;
        opt.showNavmesh = false;
        opt.showOverlaps = false;
        opt.showTriangulation = false;
        opt.showAnnotations = false;
    } else if (mode == "full") {
        opt.showGrid = true;
        opt.showObstacles = true;
        opt.showMerged = true;
        opt.showNavmesh = true;
        opt.showOverlaps = true;
        opt.showTriangulation = true;
        opt.showAnnotations = true;
        opt.debug = true;

        // Compute overlaps
        overlaps = computeOverlaps(obstacles);
        if (verbose) {
            std::cout << "Overlaps: " << overlaps.size() << " regions\n";
        }

        // Merge
        merged = mergeObstacles(obstacles, mapW, mapH);

        // Build navmesh for triangulation + annotations
        NavmeshConfig cfg;
        cfg.autoCalc(mapInfo);
        mesh = buildNavMesh(mapInfo, merged, cfg);
        if (mesh) {
            std::cout << "NavMesh: " << mesh->nverts << " verts, " << mesh->npolys << " polys\n";
            // Annotations: navmesh vertices + obstacle vertices
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
        } else {
            std::cerr << "Warning: navmesh build failed.\n";
            opt.showNavmesh = false;
            opt.showTriangulation = false;
        }
    } else { // normal (default)
        opt.showGrid = true;
        opt.showObstacles = true;
        opt.showMerged = true;
        opt.showNavmesh = false;
        opt.showOverlaps = false;
        opt.showTriangulation = false;
        opt.showAnnotations = debugMode;
        if (!debugMode) opt.debug = false;

        // Still merge for display
        merged = mergeObstacles(obstacles, mapW, mapH);
    }

    if (format == "json") {
        std::ofstream ofs(outputPath);
        if (!ofs) {
            std::cerr << "Error: cannot write " << outputPath << "\n";
            if (mesh) rcFreePolyMesh(mesh);
            return 1;
        }
        ofs << "{\n";
        writeJSONObstacles(obstacles, ofs);
        ofs << ",\n";
        writeJSONMerged(merged, ofs);
        if (!overlaps.empty()) {
            ofs << ",\n";
            writeJSONOverlaps(overlaps, ofs);
        }
        if (mesh) {
            ofs << ",\n  \"navmeshVerts\": " << mesh->nverts << ",\n";
            ofs << "  \"navmeshPolys\": " << mesh->npolys << "\n";
        }
        ofs << "}\n";
        std::cout << "JSON written to " << outputPath << "\n";
    } else if (format == "text") {
        std::ofstream ofs(outputPath);
        if (!ofs) {
            std::cerr << "Error: cannot write " << outputPath << "\n";
            if (mesh) rcFreePolyMesh(mesh);
            return 1;
        }
        for (const auto& obs : obstacles) {
            for (size_t j = 0; j < obs.points.size() / 2; j++)
                ofs << obs.points[j*2] << " " << obs.points[j*2+1] << "\n";
        }
        std::cout << "Text written to " << outputPath << "\n";
    } else { // svg
        writeSVG(outputPath, mapInfo, obstacles, merged, mesh, opt,
                 {}, overlaps, annotations);
        std::cout << "SVG written to " << outputPath << "\n";
    }

    if (mesh) rcFreePolyMesh(mesh);
    std::cout << "=== Done ===\n";
    return 0;
}

// ---------- path ----------

static void printPathHelp() {
    std::cout << "Usage: tnavmesh path [options]\n"
              << "\n"
              << "Input modes (mutually exclusive):\n"
              << "  -i, --input <file.tmx>     TMX map (build + pathfind)\n"
              << "  -n, --navmesh <file.bin>   Pre-built navmesh (runtime query)\n"
              << "  --draw <file.txt>          Render precomputed path (no navmesh)\n"
              << "\n"
              << "Points:\n"
              << "  -s, --start <x> <y>        Start point (TMX or navmesh coords)\n"
              << "  -e, --end <x> <y>          End point\n"
              << "  --auto                     Auto-generate start/end points\n"
              << "  (--start/--end or --auto required for TMX/runtime mode)\n"
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
              << "Agent (TMX mode only, auto-derived from tile size if not provided):\n"
              << "  --agent-radius <n>         Agent radius in pixels\n"
              << "  --agent-height <n>         Agent height in pixels\n"
              << "  --agent-climb <n>          Max climb height in pixels\n"
              << "\n"
              << "Voxel resolution (TMX mode only):\n"
              << "  --cell-size <float>        Voxel size (default: tileSize * 0.15)\n"
              << "  --cell-height <float>      Voxel height (default: cellSize * 0.5)\n"
              << "\n"
              << "Polygon control (TMX mode only):\n"
              << "  --poly-max-verts <n>       Max vertices per polygon (default: 6)\n"
              << "  --poly-simplify <float>    Simplification error (default: 2.0)\n"
              << "  --max-edges <n>            Max edge length in pixels (default: auto)\n"
              << "\n"
              << "Other:\n"
              << "  -v, --verbose              Verbose output\n"
              << "  -h, --help                 Show this help\n"
              << "  --slope-angle <float>      Max walkable slope in degrees (default: 45)\n";
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
    std::string inputPath;    // -i .tmx
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
        if (a == "-i" && i+1 < argc) inputPath = argv[++i];
        else if (a == "--input" && i+1 < argc) inputPath = argv[++i];
        else if (a == "-n" && i+1 < argc) navPath = argv[++i];
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
        else if (a == "--agent-radius" && i+1 < argc) navCfg.walkableRadius = std::atoi(argv[++i]);
        else if (a == "--agent-height" && i+1 < argc) navCfg.walkableHeight = std::atoi(argv[++i]);
        else if (a == "--agent-climb" && i+1 < argc) navCfg.walkableClimb = std::atoi(argv[++i]);
        else if (a == "--cell-size" && i+1 < argc) navCfg.cs = std::strtof(argv[++i], nullptr);
        else if (a == "--cell-height" && i+1 < argc) navCfg.ch = std::strtof(argv[++i], nullptr);
        else if (a == "--poly-simplify" && i+1 < argc) navCfg.maxSimplificationError = std::strtof(argv[++i], nullptr);
        else if (a == "--poly-max-verts" && i+1 < argc) navCfg.maxVertsPerPoly = std::atoi(argv[++i]);
        else if (a == "--max-edges" && i+1 < argc) navCfg.maxEdgeLen = std::atoi(argv[++i]);
        else if (a == "--slope-angle" && i+1 < argc) navCfg.walkableSlopeAngle = std::strtof(argv[++i], nullptr);
        else if (a == "-v" || a == "--verbose") verbose = true;
        else if (a == "-h" || a == "--help") { printPathHelp(); return 0; }
        else if (a[0] == '-') {
            std::cerr << "Warning: unknown option '" << a << "' ignored.\n";
        }
    }

    // Determine mode
    int modeCount = (!inputPath.empty() ? 1 : 0) + (!navPath.empty() ? 1 : 0) + (!drawPath.empty() ? 1 : 0);
    if (modeCount == 0) {
        std::cerr << "Error: one of --input, --navmesh, or --draw is required.\n";
        printPathHelp();
        return 2;
    }
    if (modeCount > 1) {
        std::cerr << "Error: --input, --navmesh, and --draw are mutually exclusive.\n";
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
        opt.debug = true;
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

    // ----- TMX mode (full build + pathfind) -----
    if (!inputPath.empty()) {
        bool hasPoints = hasStart && hasEnd;
        if (!hasPoints && !autoMode) {
            std::cerr << "Error: specify --start/--end or use --auto.\n";
            printPathHelp();
            return 2;
        }

        std::cout << "=== tnavmesh path (TMX) ===\n"
                  << "Input:  " << inputPath << "\n";

        MapInfo mapInfo;
        std::vector<Obstacle> obstacles;
        if (!parseTMX(inputPath, mapInfo, obstacles)) {
            std::cerr << "Error: failed to parse TMX file.\n";
            return 2;
        }

        float mapW = static_cast<float>(mapInfo.width) * mapInfo.tileWidth;
        float mapH = static_cast<float>(mapInfo.height) * mapInfo.tileHeight;

        navCfg.autoCalc(mapInfo);
        if (verbose) {
            std::cout << "Config: cs=" << navCfg.cs << " ch=" << navCfg.ch << "\n";
        }

        auto merged = mergeObstacles(obstacles, mapW, mapH);

        rcPolyMesh* mesh = buildNavMesh(mapInfo, merged, navCfg);
        if (!mesh) {
            std::cerr << "Error: navmesh build failed.\n";
            return 1;
        }
        std::cout << "NavMesh: " << mesh->nverts << " verts, " << mesh->npolys << " polys\n";

        if (autoMode)
            generateAutoPoints(mesh, mapH, startX, startY, endX, endY);

        dtNavMesh* navMesh = buildDetourNavMesh(mesh, navCfg);
        if (!navMesh) {
            std::cerr << "Error: failed to build Detour navmesh.\n";
            rcFreePolyMesh(mesh);
            return 1;
        }

        float searchRadius = std::max(navCfg.cs * 16, 100.0f);
        PathResult result = findPath(navMesh, startX, startY, endX, endY, mapH, searchRadius);

        // Output
        if (result.found && result.waypoints.size() >= 4) {
            std::cout << "Path found: " << (result.waypoints.size()/2)
                      << " waypoints, length=" << result.totalLength << "\n";
            if (!textOutput.empty()) {
                std::ofstream ofs(textOutput);
                if (ofs) {
                    for (size_t i = 0; i < result.waypoints.size()/2; i++)
                        ofs << result.waypoints[i*2] << " " << result.waypoints[i*2+1] << "\n";
                    std::cout << "Waypoints written to " << textOutput << "\n";
                }
            }
            if (verbose) {
                std::cout << "Waypoints:\n";
                for (size_t i = 0; i < result.waypoints.size()/2; i++)
                    std::cout << "  " << result.waypoints[i*2] << " " << result.waypoints[i*2+1] << "\n";
            }
        } else {
            std::string reason = result.error.empty() ? "degenerate path (start/end coincide)" : result.error;
            std::cout << "Path not found: " << reason << "\n";
        }

        // Write output in requested format (always write SVG even if path failed)
        if (format == "json") {
            std::ofstream ofs(outputSvg);
            if (!ofs) {
                std::cerr << "Error: cannot write " << outputSvg << "\n";
                dtFreeNavMesh(navMesh);
                rcFreePolyMesh(mesh);
                return 1;
            }
            writeJSONWaypoints(result.waypoints, ofs);
            std::cout << "JSON written to " << outputSvg << "\n";
        } else if (format == "text") {
            std::ofstream ofs(outputSvg);
            if (ofs) {
                for (size_t i = 0; i < result.waypoints.size()/2; i++)
                    ofs << result.waypoints[i*2] << " " << result.waypoints[i*2+1] << "\n";
                std::cout << "Waypoints written to " << outputSvg << "\n";
            }
        } else { // svg
            writeSVG(outputSvg, mapInfo, obstacles, merged, mesh, opt, result.waypoints);
            std::cout << "SVG written to " << outputSvg << "\n";
        }

        dtFreeNavMesh(navMesh);
        rcFreePolyMesh(mesh);

        std::cout << "=== Done ===\n";
        bool pathOk = result.found && result.waypoints.size() >= 4;
        return pathOk ? 0 : 1;
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
            if (!textOutput.empty()) {
                std::ofstream ofs(textOutput);
                if (ofs) {
                    for (size_t i = 0; i < result.waypoints.size()/2; i++)
                        ofs << result.waypoints[i*2] << " " << result.waypoints[i*2+1] << "\n";
                    std::cout << "Waypoints written to " << textOutput << "\n";
                }
            }

            // Output in requested format
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
                if (verbose) {
                    std::cout << "Waypoints:\n";
                    for (size_t i = 0; i < result.waypoints.size()/2; i++)
                        std::cout << "  " << result.waypoints[i*2] << " " << result.waypoints[i*2+1] << "\n";
                }
            }
        } else {
            std::string reason = result.error.empty() ? "degenerate path (start/end coincide)" : result.error;
            std::cout << "Path not found: " << reason << "\n";
        }

        std::cout << "=== Done ===\n";
        bool pathOk = result.found && result.waypoints.size() >= 4;
        return pathOk ? 0 : 1;
    }

    return 1; // unreachable
}
