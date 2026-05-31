#ifndef SVG_WRITER_H
#define SVG_WRITER_H

#include "tmx_parser.h"
#include <Recast.h>
#include <string>
#include <vector>

struct SvgOptions {
    bool showGrid = true;
    bool showObstacles = true;
    bool showMerged = true;
    bool showNavmesh = true;
    bool showPath = true;

    // Output image size (0 = use map dimensions)
    int svgWidth = 0;
    int svgHeight = 0;

    // Debug mode options
    bool debug = false;
    bool showOverlaps = true;
    bool showTriangulation = false;
    bool showAnnotations = false;
};

// Write SVG with layer control.
// If pathPts is non-empty, it's treated as [x,y,x,y,...] in TMX coordinates
// and rendered as the path overlay.
// If overlaps is non-empty, renders overlap regions in debug mode.
// If annotations is non-empty (alternating x,y), renders coordinate labels.
void writeSVG(const std::string& path,
              const MapInfo& mapInfo,
              const std::vector<Obstacle>& obstacles,
              const std::vector<MergedRegion>& merged,
              const rcPolyMesh* mesh,
              const SvgOptions& options = SvgOptions{},
              const std::vector<float>& pathPts = {},
              const std::vector<Obstacle>& overlaps = {},
              const std::vector<float>& annotations = {});

#endif
