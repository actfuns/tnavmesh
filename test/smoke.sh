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
$BIN build -i simple.tmx -o /tmp/test_simple.bin > /dev/null 2>&1
echo "  [OK] build simple.tmx"

# Build ruins.tmx
$BIN build -i ruins.tmx -o /tmp/test_ruins.bin > /dev/null 2>&1
echo "  [OK] build ruins.tmx"

# Path TMX mode
$BIN path -i simple.tmx --auto > /dev/null 2>&1
echo "  [OK] path simple.tmx --auto"

$BIN path -i ruins.tmx --auto > /dev/null 2>&1
echo "  [OK] path ruins.tmx --auto"

# Path runtime mode
$BIN path -n /tmp/test_simple.bin -s 0 0 -e 278 41 > /dev/null 2>&1
echo "  [OK] path runtime mode"

# Path draw mode
echo "0 0 100 100 200 0" | tr ' ' '\n' | paste - - > /tmp/test_pts.txt
$BIN path --draw /tmp/test_pts.txt --output-svg /dev/null > /dev/null 2>&1
echo "  [OK] path draw mode"

# Inspect
$BIN inspect -i simple.tmx > /dev/null 2>&1
echo "  [OK] inspect simple.tmx"

$BIN inspect -i simple.tmx --debug > /dev/null 2>&1
echo "  [OK] inspect --debug"

$BIN inspect -i simple.tmx --format json -o /dev/null > /dev/null 2>&1
echo "  [OK] inspect --format json"

$BIN inspect -i simple.tmx --format text -o /dev/null > /dev/null 2>&1
echo "  [OK] inspect --format text"

$BIN inspect -i simple.tmx --mode full -o /dev/null > /dev/null 2>&1
echo "  [OK] inspect --mode full"

# JSON output with obstacle IDs
JSON_OUT=$($BIN inspect -i simple.tmx --format json -o /dev/null 2>&1)
echo "$JSON_OUT" | grep -q '"id"' && echo "  [OK] JSON contains obstacle IDs"
echo "$JSON_OUT" | grep -q '"length"' && echo "  [OK] JSON contains length"

# Invalid input
$BIN build 2>&1 | grep -q "Error" && echo "  [OK] build without args prints error"

# Unknown option warning
$BIN build -i simple.tmx -o /dev/null --bogus-flag 2>&1 | grep -q "Warning" && echo "  [OK] unknown option warning"

echo "=== All smoke tests passed ==="
