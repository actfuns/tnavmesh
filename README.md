# tnavmesh — TMX Map to Navigation Mesh Converter

[![CI](https://github.com/actfuns/recastnavigation/actions/workflows/build.yml/badge.svg)](https://github.com/actfuns/recastnavigation/actions/workflows/build.yml)

Converts 2D TMX tile maps into navigation meshes using [Recast](https://github.com/recastnavigation/recastnavigation) and [Detour](https://github.com/recastnavigation/recastnavigation), with obstacle merging via [GEOS](https://libgeos.org/).

## Features

- **build** — Parse TMX, merge obstacles (GEOS), triangulate (poly2tri), and build a Recast+Detour navmesh. Output binary navmesh (.bin).
- **inspect** — Visual debugging: obstacle overlaps, merged regions, triangulation edges, coordinate annotations. Supports `--format svg|json|text`.
- **path** — Full pipeline or runtime pathfinding. Three input modes:
  - `-i map.tmx` — TMX → build → pathfind
  - `-n navmesh.bin` — Runtime query on pre-built navmesh
  - `--draw path.txt` — Render precomputed path SVG (no navmesh needed)

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
./build/tnavmesh path -i assets/maps/simple.tmx --auto
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
  inspect   Visual debugging of the navigation pipeline
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
| `--quality` | `low` \| `normal` \| `high` (default: normal) |
| `--agent-radius` | Agent radius in pixels (auto-derived) |
| `--agent-height` | Agent height in pixels (auto-derived) |
| `--agent-climb` | Max climb height in pixels (auto-derived) |
| `--cell-size` | Voxel size (default: tileSize × 0.15) |
| `--cell-height` | Voxel height (default: cellSize × 0.5) |
| `--poly-max-verts` | Max vertices per polygon (default: 6) |
| `--poly-simplify` | Simplification error (default: 2.0) |
| `-v` | Verbose output |

### inspect

```bash
tnavmesh inspect -i map.tmx -o output.svg [options]
tnavmesh inspect -i map.tmx -o output.json --format json
```

| Option | Description |
|--------|-------------|
| `--mode` | `minimal` \| `normal` \| `full` (default: normal) |
| `--format` | `svg` \| `json` \| `text` (default: svg) |
| `--debug` | Show obstacle IDs and coordinate annotations |
| `--width`, `--height` | SVG output size in pixels |

### path

```bash
tnavmesh path -i map.tmx -s 32 32 -e 288 288         # TMX mode
tnavmesh path -i map.tmx --auto                       # Auto points
tnavmesh path -n map.bin -s 32 32 -e 288 288          # Runtime mode
tnavmesh path --draw waypoints.txt --output-svg out.svg  # Draw mode
```

| Option | Description |
|--------|-------------|
| `-s, --start <x> <y>` | Start point |
| `-e, --end <x> <y>` | End point |
| `--auto` | Auto-generate start/end |
| `--visual` | `simple` \| `full` \| `debug` (default: full) |
| `--format` | `svg` \| `json` \| `text` (default: svg) |

## Build Dependencies

| Library | Linux | macOS | Windows |
|---------|-------|-------|---------|
| **GEOS** | `libgeos-dev` | `brew install geos` | vcpkg `geos` |
| **poly2tri** | bundled (`third_party/`) | bundled | bundled |
| **Recast/Detour** | bundled (`third_party/`) | bundled | bundled (built via cmake subdirectory) |

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
