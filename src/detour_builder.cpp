#include "tnavmesh/detour_builder.h"
#include <DetourNavMeshBuilder.h>
#include <cstdio>
#include <cstring>
#include <iostream>

dtNavMesh* buildDetourNavMesh(rcPolyMesh* mesh, const NavmeshConfig& config) {
    if (!mesh || mesh->nverts == 0 || mesh->npolys == 0)
        return nullptr;

    dtNavMeshCreateParams params;
    memset(&params, 0, sizeof(params));

    params.verts = mesh->verts;
    params.vertCount = mesh->nverts;
    params.polys = mesh->polys;
    params.polyAreas = mesh->areas;
    // Ensure flags array is populated
    if (mesh->flags) {
        for (int i = 0; i < mesh->npolys; i++)
            mesh->flags[i] = 1;
    }
    params.polyFlags = mesh->flags;
    params.polyCount = mesh->npolys;
    params.nvp = mesh->nvp;

    // No detail mesh data — poly mesh only
    params.detailMeshes = nullptr;
    params.detailVerts = nullptr;
    params.detailVertsCount = 0;
    params.detailTris = nullptr;
    params.detailTriCount = 0;

    params.walkableHeight = (float)config.walkableHeight * config.ch;
    params.walkableRadius = (float)config.walkableRadius * config.cs;
    params.walkableClimb = (float)config.walkableClimb * config.ch;
    params.cs = config.cs;
    params.ch = config.ch;
    params.bmin[0] = mesh->bmin[0];
    params.bmin[1] = mesh->bmin[1];
    params.bmin[2] = mesh->bmin[2];
    params.bmax[0] = mesh->bmax[0];
    params.bmax[1] = mesh->bmax[1];
    params.bmax[2] = mesh->bmax[2];

    params.tileX = 0;
    params.tileY = 0;
    params.tileLayer = 0;
    params.buildBvTree = true;

    unsigned char* navData = nullptr;
    int navDataSize = 0;

    if (!dtCreateNavMeshData(&params, &navData, &navDataSize)) {
        std::cerr << "Error: dtCreateNavMeshData failed.\n";
        return nullptr;
    }

    dtNavMesh* navMesh = dtAllocNavMesh();
    if (!navMesh) {
        dtFree(navData);
        std::cerr << "Error: dtAllocNavMesh failed.\n";
        return nullptr;
    }

    // Single-tile init: owns the data
    dtStatus status = navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
    if (dtStatusFailed(status)) {
        dtFreeNavMesh(navMesh);
        dtFree(navData);  // init failed — data not owned by navmesh
        std::cerr << "Error: dtNavMesh::init failed.\n";
        return nullptr;
    }

    return navMesh;
}

bool getNavMeshData(const dtNavMesh* navMesh, const unsigned char*& data, int& dataSize) {
    const dtMeshTile* tile = navMesh->getTile(0);
    if (!tile || !tile->data || tile->dataSize <= 0)
        return false;
    data = tile->data;
    dataSize = tile->dataSize;
    return true;
}

bool saveNavMesh(const char* path, const unsigned char* data, int dataSize) {
    FILE* fp = std::fopen(path, "wb");
    if (!fp) {
        std::cerr << "Error: could not open " << path << " for writing.\n";
        return false;
    }

    unsigned int sz = (unsigned int)dataSize;
    if (std::fwrite(&sz, sizeof(sz), 1, fp) != 1) {
        std::cerr << "Error: failed to write size header.\n";
        std::fclose(fp);
        return false;
    }

    if (std::fwrite(data, 1, dataSize, fp) != (size_t)dataSize) {
        std::cerr << "Error: failed to write navmesh data.\n";
        std::fclose(fp);
        return false;
    }

    std::fclose(fp);
    std::cout << "NavMesh data written to " << path
              << " (" << dataSize << " bytes)\n";
    return true;
}

bool loadNavMesh(const char* path, unsigned char*& data, int& dataSize) {
    FILE* fp = std::fopen(path, "rb");
    if (!fp) {
        std::cerr << "Error: could not open " << path << " for reading.\n";
        return false;
    }

    unsigned int sz = 0;
    if (std::fread(&sz, sizeof(sz), 1, fp) != 1) {
        std::cerr << "Error: failed to read size header.\n";
        std::fclose(fp);
        return false;
    }

    dataSize = (int)sz;
    if (dataSize <= 0 || dataSize > 100 * 1024 * 1024) {
        std::cerr << "Error: invalid navmesh data size " << dataSize << ".\n";
        std::fclose(fp);
        return false;
    }

    data = new unsigned char[dataSize];
    if (std::fread(data, 1, dataSize, fp) != (size_t)dataSize) {
        std::cerr << "Error: failed to read navmesh data.\n";
        delete[] data;
        data = nullptr;
        std::fclose(fp);
        return false;
    }

    std::fclose(fp);
    return true;
}
