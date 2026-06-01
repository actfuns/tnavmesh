# tnavmesh — TMX Map to Navigation Mesh Converter

Converts 2D TMX tile maps into navigation meshes using [Recast](https://github.com/recastnavigation/recastnavigation) and [Detour](https://github.com/recastnavigation/recastnavigation), with obstacle merging via [GEOS](https://libgeos.org/).

## Features

- **build** — Parse TMX, merge obstacles (GEOS), triangulate (poly2tri), and build a Recast+Detour navmesh. Output binary navmesh (.bin) with optional SVG visualization.
- **path** — Runtime pathfinding on pre-built navmesh or render precomputed paths.

## Quick Start

```bash
# Build (automatically clones third-party dependencies)
make

# Build with custom cmake args
make CMAKE_ARGS="-DCMAKE_BUILD_TYPE=Debug"

# Or manually
cmake -S . -B build
cmake --build build

# Run
./build/tnavmesh build -i assets/maps/simple.tmx -o simple.bin

# Pathfinding on pre-built navmesh
./build/tnavmesh path -n simple.bin -s 32 32 -e 288 288
```

## Run smoke tests

```bash
bash test/smoke.sh
```

## Usage

```
tnavmesh <command> [options]

Commands:
  build     Generate navigation mesh from TMX
  path      Compute or render navigation paths
```

### build

```bash
tnavmesh build -i map.tmx -o map.bin [options]
```

| Option | Description |
|--------|-------------|
| `-i, --input` | Input TMX file (required) |
| `-o, --output` | Output binary navmesh (.bin) |
| `--resolution` | `low` \| `normal` \| `high` (default: normal) |
| `--agent-radius` | Agent radius in pixels (auto-derived from tile size) |
| `--agent-height` | Agent height in pixels (auto-derived) |
| `--agent-climb` | Max climb height in pixels (auto-derived) |
| `--cell-size` | Voxel size (default: tileSize × 0.25) |
| `--cell-height` | Voxel height (default: cellSize × 0.5) |
| `--poly-max-verts` | Max vertices per polygon (default: 6, max: 6) |
| `--poly-simplify` | Simplification error (default: 2.0) |
| `--max-edges` | Max edge length in voxels (auto-derived) |
| `--slope-angle` | Max walkable slope in degrees (default: 45) |
| `--min-region-area` | Minimum region area in voxels (auto-derived) |
| `--merge-region-area` | Merge region area in voxels (minRegion × 8) |
| `--partition` | `watershed` \| `monotone` \| `layers` (default: watershed) |
| `--detail-sample-dist` | Detail mesh sample distance (default: 6.0) |
| `--detail-max-error` | Detail mesh max error (default: 1.0) |
| `--svg-output <file>` | Save visual debug SVG |
| `--svg-layer <name>` | Layer to show (repeatable): `grid`, `obstacles`, `merged`, `navmesh`, `annotations`. Default: merged + navmesh |
| `--svg-no-legend` | Hide the legend overlay |
| `--svg-width <n>` | SVG output width in px (default: 1200) |
| `--svg-height <n>` | SVG output height in px (default: 1200) |
| `-v, --verbose` | Verbose output |

### path

```bash
tnavmesh path -n map.bin -s 32 32 -e 288 288          # Runtime mode
tnavmesh path --draw waypoints.txt --output-svg out.svg  # Draw mode
```

| Option | Description |
|--------|-------------|
| `-n, --navmesh <file.bin>` | Pre-built navmesh (runtime query) |
| `--draw <file.txt>` | Render precomputed path (no navmesh) |
| `-s, --start <x> <y>` | Start point |
| `-e, --end <x> <y>` | End point |
| `--auto` | Auto-generate start/end points |
| `--output-svg <file>` | Output SVG (default: path.svg) |
| `--text-output <file>` | Save waypoints as text |
| `--format` | `svg` \| `json` \| `text` (default: svg) |
| `--visual` | `simple` \| `full` \| `debug` (default: full) |
| `-v, --verbose` | Verbose output |

## Build Dependencies

| Library | Linux | macOS | Windows |
|---------|-------|-------|---------|
| **GEOS** | `libgeos-dev` | `brew install geos` | vcpkg `geos` |
| **poly2tri** | bundled (`third_party/`) | bundled | bundled |
| **Recast/Detour** | bundled (`third_party/`) | bundled | bundled (built via cmake subdirectory) |

> **Note:** GEOS is a **required** dependency. Install it first, then build.

On Windows with vcpkg (manifest mode — vcpkg.json in project root):

```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE="%VCPKG_INSTALLATION_ROOT%/scripts/buildsystems/vcpkg.cmake"
cmake --build build --config Release
```

Or with vcpkg classic mode:

```bash
vcpkg install geos --triplet x64-windows
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE="C:/vcpkg/scripts/buildsystems/vcpkg.cmake"
cmake --build build --config Release
```

### Build using Makefile (Linux only)

The Makefile handles dependency checks, third-party cloning, and cmake configuration automatically:

```bash
# Build (checks deps, clones third-party, configures, compiles)
make

# Clean build
make rebuild
```

## Coordinate System

- TMX: x→right, y→down (screen coordinates)
- Detour: x→right, y→up, z→depth
- Conversion: `detourPos = (tmxX, 0, mapH - tmxY)`

## File Format (.bin)

```
[4 bytes] dataSize (little-endian uint32)
[dataSize bytes] dtCreateNavMeshData output
```

## License

MIT
