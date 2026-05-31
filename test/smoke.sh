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

# Path TMX mode
$BIN path -i assets/maps/simple.tmx --auto > /dev/null 2>&1
echo "  [OK] path simple.tmx --auto"

$BIN path -i assets/maps/complex.tmx --auto > /dev/null 2>&1
echo "  [OK] path complex.tmx --auto"

# Path runtime mode
$BIN path -n /tmp/test_simple.bin -s 0 0 -e 278 41 > /dev/null 2>&1
echo "  [OK] path runtime mode"

# Path draw mode
echo "0 0 100 100 200 0" | tr ' ' '\n' | paste - - > /tmp/test_pts.txt
$BIN path --draw /tmp/test_pts.txt --output-svg /dev/null > /dev/null 2>&1
echo "  [OK] path draw mode"

# Inspect
$BIN inspect -i assets/maps/simple.tmx > /dev/null 2>&1
echo "  [OK] inspect simple.tmx"

$BIN inspect -i assets/maps/simple.tmx --debug > /dev/null 2>&1
echo "  [OK] inspect --debug"

$BIN inspect -i assets/maps/simple.tmx --format json -o /dev/null > /dev/null 2>&1
echo "  [OK] inspect --format json"

$BIN inspect -i assets/maps/simple.tmx --format text -o /dev/null > /dev/null 2>&1
echo "  [OK] inspect --format text"

$BIN inspect -i assets/maps/simple.tmx --mode full -o /dev/null > /dev/null 2>&1
echo "  [OK] inspect --mode full"

# JSON output with obstacle IDs
JSON_OUT=$($BIN inspect -i assets/maps/simple.tmx --format json -o /dev/null 2>&1)
echo "$JSON_OUT" | grep -q '"id"' && echo "  [OK] JSON contains obstacle IDs"
echo "$JSON_OUT" | grep -q '"length"' && echo "  [OK] JSON contains length"

# Invalid input
$BIN build 2>&1 | grep -q "Error" && echo "  [OK] build without args prints error"

# Unknown option warning
$BIN build -i assets/maps/simple.tmx -o /dev/null --bogus-flag 2>&1 | grep -q "Warning" && echo "  [OK] unknown option warning"

echo "=== All smoke tests passed ==="
