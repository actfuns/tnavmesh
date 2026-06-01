#include "tnavmesh/tmx_parser.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cstring>
#include <cstdlib>

static std::string readFile(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs) {
        std::cerr << "Error: cannot open file: " << path << std::endl;
        return "";
    }
    return std::string((std::istreambuf_iterator<char>(ifs)),
                        std::istreambuf_iterator<char>());
}

// Find a substring, return position or std::string::npos
static size_t findTagAttr(const std::string& xml, const std::string& attr, size_t pos) {
    std::string target = " " + attr + "=\"";
    size_t found = xml.find(target, pos);
    if (found == std::string::npos) {
        target = " " + attr + "='";
        found = xml.find(target, pos);
    }
    return found;
}

// Extract attribute value: assumes the XML is well-formed
// Returns the value string (without quotes)
static std::string getAttrValue(const std::string& xml, const std::string& attr, size_t tagStart) {
    size_t apos = findTagAttr(xml, attr, tagStart);
    if (apos == std::string::npos) return "";
    apos += attr.size() + 3; // skip ` attr="`
    char quote = xml[apos - 1];
    size_t end = xml.find(quote, apos);
    if (end == std::string::npos) return "";
    return xml.substr(apos, end - apos);
}

// Extract a float attribute
static float getFloatAttr(const std::string& xml, const std::string& attr, size_t tagStart, float def = 0) {
    std::string val = getAttrValue(xml, attr, tagStart);
    if (val.empty()) return def;
    return std::strtof(val.c_str(), nullptr);
}

// Trim whitespace
static std::string trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

// Split string by delimiter
static std::vector<std::string> split(const std::string& s, char delim) {
    std::vector<std::string> parts;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        std::string t = trim(item);
        if (!t.empty()) parts.push_back(t);
    }
    return parts;
}

bool parseTMX(const std::string& path, MapInfo& mapInfo, std::vector<Obstacle>& obstacles) {
    std::string xml = readFile(path);
    if (xml.empty()) return false;

    // Parse <map> attributes
    size_t mapStart = xml.find("<map");
    if (mapStart == std::string::npos) {
        std::cerr << "Error: no <map> tag found" << std::endl;
        return false;
    }
    mapInfo.width       = (int)getFloatAttr(xml, "width", mapStart);
    mapInfo.height      = (int)getFloatAttr(xml, "height", mapStart);
    mapInfo.tileWidth   = getFloatAttr(xml, "tilewidth", mapStart);
    mapInfo.tileHeight  = getFloatAttr(xml, "tileheight", mapStart);

    // Find <objectgroup name="obstacle">
    size_t groupPos = 0;
    while (true) {
        size_t ogStart = xml.find("<objectgroup", groupPos);
        if (ogStart == std::string::npos) break;
        size_t ogEnd = xml.find("</objectgroup>", ogStart);
        if (ogEnd == std::string::npos) break;

        std::string name = getAttrValue(xml, "name", ogStart);
        if (name == "obstacle") {
            // Parse objects within this group
            // First, find the end of <objectgroup ...> tag
            size_t ogTagEnd = xml.find('>', ogStart);
            if (ogTagEnd == std::string::npos || ogTagEnd >= ogEnd) break;

            size_t objPos = ogTagEnd;
            while (true) {
                // Search for <object...> but not <objectgroup
                objPos = xml.find('<', objPos + 1);
                if (objPos == std::string::npos || objPos >= ogEnd) break;

                // Check if this is an <object...> tag (not </object>, not <objectgroup)
                if (xml.compare(objPos, 8, "<object ") != 0 &&
                    xml.compare(objPos, 9, "<object\t") != 0 &&
                    xml.compare(objPos, 8, "<object/") != 0) {
                    continue;
                }

                // Find the end of the opening tag: either '/>' (self-closing) or '>'
                size_t gtPos = xml.find('>', objPos);
                if (gtPos == std::string::npos || gtPos >= ogEnd) break;

                bool isSelfClosing = (gtPos >= 2 && xml[gtPos - 1] == '/');

                // Parse all attributes from the opening tag only
                std::string openTag = xml.substr(objPos, gtPos - objPos);
                float x = getFloatAttr(openTag, "x", 0);
                float y = getFloatAttr(openTag, "y", 0);
                float w = getFloatAttr(openTag, "width", 0, -1);
                float h = getFloatAttr(openTag, "height", 0, -1);

                // Check for gid attribute (tile placement)
                std::string gidStr = getAttrValue(openTag, "gid", 0);
                if (!gidStr.empty()) {
                    // This is a tile reference, not obstacle geometry — skip it
                    objPos = gtPos;
                    if (isSelfClosing) objPos += 2;
                    else {
                        size_t ct = xml.find("</object>", gtPos + 1);
                        if (ct != std::string::npos && ct < ogEnd) objPos = ct + 9;
                        else objPos = gtPos + 1;
                    }
                    continue;
                }

                Obstacle obs;
                obs.id = (int)getFloatAttr(openTag, "id", 0, 0);
                bool hasPolygon = false;

                if (!isSelfClosing) {
                    // Object has content — check for <polygon> child
                    size_t closeTag = xml.find("</object>", gtPos + 1);
                    if (closeTag == std::string::npos || closeTag >= ogEnd) break;

                    std::string content = xml.substr(gtPos + 1, closeTag - gtPos - 1);
                    size_t polyStart = content.find("<polygon");
                    if (polyStart != std::string::npos) {
                        std::string pointsStr = getAttrValue(content, "points", polyStart);
                        if (!pointsStr.empty()) {
                            hasPolygon = true;
                            std::vector<std::string> pointPairs = split(pointsStr, ' ');
                            for (const auto& pair : pointPairs) {
                                std::vector<std::string> xy = split(pair, ',');
                                if (xy.size() == 2) {
                                    float px = std::strtof(xy[0].c_str(), nullptr);
                                    float py = std::strtof(xy[1].c_str(), nullptr);
                                    obs.points.push_back(x + px);
                                    obs.points.push_back(y + py);
                                }
                            }
                        }
                    }
                    objPos = closeTag + 9; // skip </object>
                }

                if (!hasPolygon && w > 0 && h > 0) {
                    // Rectangle: store 4 corner points
                    obs.points = {
                        x,     y,
                        x+w,   y,
                        x+w,   y+h,
                        x,     y+h
                    };
                }

                if (!obs.points.empty()) {
                    std::cout << "  obstacle id=" << obs.id << " (" << (obs.points.size()/2) << " pts)" << std::endl;
                    obstacles.push_back(std::move(obs));
                }
            }
        }
        groupPos = ogEnd + 14;
    }

    std::cout << "Parsed TMX: " << mapInfo.width << "x" << mapInfo.height
              << " tiles, tile " << mapInfo.tileWidth << "x" << mapInfo.tileHeight
              << ", found " << obstacles.size() << " obstacles" << std::endl;
    return true;
}
