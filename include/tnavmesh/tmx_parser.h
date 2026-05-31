#ifndef TMX_PARSER_H
#define TMX_PARSER_H

#include <string>
#include <vector>

struct MapInfo {
    int width = 0;
    int height = 0;
    float tileWidth = 0;
    float tileHeight = 0;
};

struct Obstacle {
    int id = 0;                 // object id from TMX (for debugging)
    std::vector<float> points; // x, y pairs in TMX coordinate space (y-down)
};

struct MergedRegion {
    Obstacle exterior;                    // outer boundary
    std::vector<Obstacle> holes;          // inner boundaries (walkable areas)
};

bool parseTMX(const std::string& path, MapInfo& mapInfo, std::vector<Obstacle>& obstacles);

#endif
