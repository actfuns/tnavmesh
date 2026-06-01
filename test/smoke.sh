#!/bin/bash
# Smoke test for tnavmesh
set -e

BIN="${1:-./build/tnavmesh}"
echo "=== Smoke test: $BIN ==="

# --version and --help
$BIN --version
$BIN --help
echo "  [OK] version/help"

# Build simple.tmx
$BIN build -i assets/maps/simple.tmx -o /tmp/test_simple.bin > /dev/null 2>&1
echo "  [OK] build simple.tmx"

# Build complex.tmx
$BIN build -i assets/maps/complex.tmx -o /tmp/test_complex.bin > /dev/null 2>&1
echo "  [OK] build complex.tmx"

# query path
$BIN query path -n /tmp/test_simple.bin --start 32 32 --end 288 288 > /dev/null 2>&1
echo "  [OK] query path"

# query path --debug
$BIN query path -n /tmp/test_complex.bin --start 5000 2000 --end 10000 12000 --debug > /dev/null 2>&1
echo "  [OK] query path --debug"

# query path --format json
$BIN query path -n /tmp/test_simple.bin --start 32 32 --end 288 288 --format json > /dev/null 2>&1
echo "  [OK] query path --format json"

# query path -o svg
$BIN query path -n /tmp/test_simple.bin --start 32 32 --end 288 288 -o /tmp/test_path.svg > /dev/null 2>&1
echo "  [OK] query path -o svg"

# query nearest
$BIN query nearest -n /tmp/test_simple.bin --pos 100 100 > /dev/null 2>&1
echo "  [OK] query nearest"

# query random
$BIN query random -n /tmp/test_complex.bin --count 3 > /dev/null 2>&1
echo "  [OK] query random"

# query raycast
$BIN query raycast -n /tmp/test_complex.bin --start 5000 2000 --end 10000 12000 > /dev/null 2>&1
echo "  [OK] query raycast"

# render --points
$BIN render --points "32,32 160,160 288,288" -o /tmp/test_render.svg > /dev/null 2>&1
echo "  [OK] render --points"

# query render -i (text)
echo "32 32" > /tmp/test_pts.txt
echo "160 160" >> /tmp/test_pts.txt
echo "288 288" >> /tmp/test_pts.txt
$BIN render -i /tmp/test_pts.txt -o /tmp/test_render2.svg > /dev/null 2>&1
echo "  [OK] render -i"

# inspect
$BIN inspect -n /tmp/test_simple.bin > /dev/null 2>&1
echo "  [OK] inspect"

# Invalid input
$BIN build 2>&1 | grep -q "Error" && echo "  [OK] build without args prints error"

# Unknown option warning
$BIN build -i assets/maps/simple.tmx -o /dev/null --bogus-flag 2>&1 | grep -q "Warning" && echo "  [OK] unknown option warning"

echo "=== All smoke tests passed ==="
