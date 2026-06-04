# tnavmesh — TMX 地图转导航网格转换器

使用 [Recast](https://github.com/recastnavigation/recastnavigation) 和 [Detour](https://github.com/recastnavigation/recastnavigation) 将 2D TMX 瓦片地图转换为导航网格，并通过 [GEOS](https://libgeos.org/) 合并障碍物。

## 功能特性

- **build** — 解析 TMX，合并障碍物（GEOS），三角化（poly2tri），构建 Recast+Detour 导航网格。输出二进制导航网格（.bin）并支持可选的 SVG 可视化。
- **path** — 在预构建的导航网格上进行运行时寻路，或渲染预计算路径。

## 快速开始

```bash
# 构建（自动克隆第三方依赖）
make

# 使用自定义 cmake 参数构建
make CMAKE_ARGS="-DCMAKE_BUILD_TYPE=Debug"

# 或手动构建
cmake -S . -B build
cmake --build build

# 运行
./build/tnavmesh build -i assets/maps/simple.tmx -o simple.bin

# 在预构建导航网格上寻路
./build/tnavmesh path -n simple.bin -s 32 32 -e 288 288
```

## 运行冒烟测试

```bash
bash test/smoke.sh
```

## 使用说明

```
tnavmesh <command> [options]

命令:
  build              从 TMX 生成导航网格
  query path         查找两点之间的路径
  query nearest      查找导航网格上的最近点
  query random       在导航网格上生成随机点
  query raycast      测试直线可达性
  render             将路径点渲染为 SVG
  inspect            查看导航网格信息
```

### build

```bash
tnavmesh build -i map.tmx -o map.bin [options]
```

| 选项 | 说明 |
|--------|------|
| `-i, --input` | 输入 TMX 文件（必需） |
| `-o, --output` | 输出二进制导航网格 (.bin) |
| `--resolution` | `low` \| `normal` \| `high`（默认：normal） |
| `--agent-radius` | 智能体半径（像素，根据瓦片大小自动推导） |
| `--agent-height` | 智能体高度（像素，自动推导） |
| `--agent-climb` | 最大攀爬高度（像素，自动推导） |
| `--cell-size` | 体素大小（默认：tileSize × 0.25） |
| `--cell-height` | 体素高度（默认：cellSize × 0.5） |
| `--poly-max-verts` | 每个多边形最大顶点数（默认：6，最大：6） |
| `--poly-simplify` | 简化误差（默认：2.0） |
| `--max-edges` | 最大边长（体素单位，自动推导） |
| `--slope-angle` | 最大可行走坡度（度，默认：45） |
| `--min-region-area` | 最小区域面积（体素单位，自动推导） |
| `--merge-region-area` | 合并区域面积（体素单位，minRegion × 8） |
| `--partition` | `watershed` \| `monotone` \| `layers`（默认：watershed） |
| `--detail-sample-dist` | 细节网格采样距离（默认：6.0） |
| `--detail-max-error` | 细节网格最大误差（默认：1.0） |
| `--svg-output <file>` | 保存可视化调试 SVG |
| `--svg-layer <name>` | 要显示的图层（可重复）：`grid`、`obstacles`、`merged`、`navmesh`、`annotations`。默认：merged + navmesh |
| `--svg-no-legend` | 隐藏图例覆盖层 |
| `--svg-width <n>` | SVG 输出宽度（像素，默认：1200） |
| `--svg-height <n>` | SVG 输出高度（像素，默认：1200） |
| `-v, --verbose` | 详细输出 |

### query path

```bash
tnavmesh query path -n navmesh.bin --start 32 32 --end 288 288
```

| 选项 | 说明 |
|--------|------|
| `-n, --navmesh <file>` | 预构建导航网格 (.bin，必需) |
| `--start <x> <y>` | 起点（必需） |
| `--end <x> <y>` | 终点（必需） |
| `-o, --output <file>` | 输出文件（根据扩展名推断格式） |
| `--format <fmt>` | `text` \| `json` \| `svg`（默认：text） |
| `--debug` | 显示详细路径信息及所有路径点 |

退出码：`0` 成功，`1` 未找到路径，`2` 无效参数，`3` 导航网格加载失败。

### query nearest

```bash
tnavmesh query nearest -n navmesh.bin --pos 500 500
```

| 选项 | 说明 |
|--------|------|
| `-n, --navmesh <file>` | 预构建导航网格 (.bin，必需) |
| `--pos <x> <y>` | 查询点（必需） |
| `--format <fmt>` | `text` \| `json`（默认：text） |

### query random

```bash
tnavmesh query random -n navmesh.bin --count 5
```

| 选项 | 说明 |
|--------|------|
| `-n, --navmesh <file>` | 预构建导航网格 (.bin，必需) |
| `--count <n>` | 随机点数量（默认：1） |
| `--seed <n>` | 随机数种子 |
| `--minx <x> --maxx <x>` | X 范围过滤 |
| `--miny <y> --maxy <y>` | Y 范围过滤 |
| `--format <fmt>` | `text` \| `json`（默认：text） |

### query raycast

```bash
tnavmesh query raycast -n navmesh.bin --start 32 32 --end 288 288
```

| 选项 | 说明 |
|--------|------|
| `-n, --navmesh <file>` | 预构建导航网格 (.bin，必需) |
| `--start <x> <y>` | 起点（必需） |
| `--end <x> <y>` | 终点（必需） |
| `--format <fmt>` | `text` \| `json`（默认：text） |

### render

```bash
tnavmesh render -i waypoints.txt -o out.svg
tnavmesh render --points "5000,2000 10000,12000" -o out.svg
```

| 选项 | 说明 |
|--------|------|
| `-i, --input <file>` | 从 txt/json 文件读取路径点 |
| `--points "<x,y x,y ...>"` | 内联路径点 |
| `-o, --output <file>` | 输出文件（默认：render.svg） |
| `--format <fmt>` | `svg`（默认） |
| `--visual <level>` | `simple` \| `full` \| `debug`（默认：full） |

### inspect

```bash
tnavmesh inspect -n navmesh.bin
```

| 选项 | 说明 |
|--------|------|
| `-n, --navmesh <file>` | 预构建导航网格 (.bin，必需) |
| `-o, --output <file.svg>` | 导出导航网格可视化 |
| `-v, --verbose` | 显示详细配置 |

## 构建依赖

| 库 | Linux | macOS | Windows |
|---------|-------|-------|---------|
| **GEOS** | `libgeos-dev` | `brew install geos` | vcpkg `geos` |
| **poly2tri** | 内置（`third_party/`） | 内置 | 内置 |
| **Recast/Detour** | 内置（`third_party/`） | 内置 | 内置（通过 cmake 子目录构建） |

> **注意：** GEOS 是**必需**依赖。请先安装它，然后再构建。

在 Windows 上使用 vcpkg（清单模式 — vcpkg.json 位于项目根目录）：

```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE="%VCPKG_INSTALLATION_ROOT%/scripts/buildsystems/vcpkg.cmake"
cmake --build build --config Release
```

或使用 vcpkg 经典模式：

```bash
vcpkg install geos --triplet x64-windows
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE="C:/vcpkg/scripts/buildsystems/vcpkg.cmake"
cmake --build build --config Release
```

### 使用 Makefile 构建（仅限 Linux）

Makefile 会自动处理依赖检查、第三方库克隆和 cmake 配置：

```bash
# 构建（检查依赖、克隆第三方库、配置、编译）
make

# 清理构建
make rebuild
```

## 坐标系

- TMX：x→右，y→下（屏幕坐标）
- Detour：x→右，y→上，z→深度
- 转换：`detourPos = (tmxX, 0, mapH - tmxY)`

## 文件格式 (.bin)

```
[4 字节] dataSize（小端序 uint32）
[dataSize 字节] dtCreateNavMeshData 输出
```

## 许可证

MIT
