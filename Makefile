.PHONY: all build clean

BUILD_DIR ?= build
CMAKE ?= cmake

all: build
	$(CMAKE) --build $(BUILD_DIR)

build:
	$(CMAKE) -S . -B $(BUILD_DIR) $(CMAKE_ARGS)

clean:
	rm -rf $(BUILD_DIR)

rebuild: clean all
