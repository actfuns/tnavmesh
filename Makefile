.PHONY: all build clean clone check-deps

BUILD_DIR ?= build
CMAKE ?= cmake
THIRD_PARTY_DIR := third_party

# Read tag from .gitmodules
RECAST_TAG := $(shell sed -n '/recastnavigation/,/\[/{/tag = /s/.*tag = //p}' .gitmodules 2>/dev/null)
POLY2TRI_TAG := $(shell sed -n '/poly2tri/,/\[/{/tag = /s/.*tag = //p}' .gitmodules 2>/dev/null)

all: check-deps clone build
	$(CMAKE) --build $(BUILD_DIR)

check-deps:
	@echo "Checking dependencies..."
	@which cmake > /dev/null 2>&1 || { echo "Error: cmake not found. Install with: sudo apt-get install cmake"; exit 1; }
	@which g++ > /dev/null 2>&1 || { echo "Error: g++ not found. Install with: sudo apt-get install g++"; exit 1; }
	@if [ "$$(uname -s)" = "Linux" ]; then \
		dpkg -l libgeos-dev > /dev/null 2>&1 || { echo "Error: libgeos-dev not found. Install with: sudo apt-get install libgeos-dev"; exit 1; }; \
	fi
	@echo "All dependencies ok."

third_party/recastnavigation:
	git clone --depth 1 --branch $(RECAST_TAG) https://github.com/recastnavigation/recastnavigation $(THIRD_PARTY_DIR)/recastnavigation

third_party/poly2tri:
	git clone --depth 1 --branch $(POLY2TRI_TAG) https://github.com/jhasse/poly2tri $(THIRD_PARTY_DIR)/poly2tri

clone: third_party/recastnavigation third_party/poly2tri

build: clone
	$(CMAKE) -S . -B $(BUILD_DIR) $(CMAKE_ARGS)

clean:
	rm -rf $(BUILD_DIR)

distclean: clean
	rm -rf $(THIRD_PARTY_DIR)

rebuild: distclean all
