cmake_minimum_required(VERSION 3.14)
project(tnavmesh VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Output binary name
set(TARGET_NAME tnavmesh)

# -------------------------------------------------------------------
# Recast / Detour — build from source as subdirectory
# -------------------------------------------------------------------
set(RECASTNAVIGATION_DEMO OFF CACHE BOOL "" FORCE)
set(RECASTNAVIGATION_TESTS OFF CACHE BOOL "" FORCE)
set(RECASTNAVIGATION_EXAMPLES OFF CACHE BOOL "" FORCE)

add_subdirectory(third_party/recastnavigation EXCLUDE_FROM_ALL)

# -------------------------------------------------------------------
# poly2tri — bundled sources, non-cmake project
# -------------------------------------------------------------------
set(POLY2TRI_DIR "${CMAKE_SOURCE_DIR}/third_party/poly2tri/poly2tri")
set(POLY2TRI_SOURCES
  ${POLY2TRI_DIR}/common/shapes.cc
  ${POLY2TRI_DIR}/sweep/advancing_front.cc
  ${POLY2TRI_DIR}/sweep/cdt.cc
  ${POLY2TRI_DIR}/sweep/sweep.cc
  ${POLY2TRI_DIR}/sweep/sweep_context.cc
)

# -------------------------------------------------------------------
# GEOS — optional dependency
# -------------------------------------------------------------------
# Strategy: try vcpkg CMake config first (Windows), then pkg-config / find_path (Linux/macOS)
find_package(geos CONFIG QUIET)

if(NOT geos_FOUND)
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(GEOS geos)
  endif()

  if(NOT GEOS_FOUND)
    # Fallback: try find_path/find_library (works with vcpkg classic)
    find_path(GEOS_INCLUDE_DIRS geos_c.h)
    find_library(GEOS_LIBRARIES geos_c)
    if(GEOS_INCLUDE_DIRS AND GEOS_LIBRARIES)
      set(GEOS_FOUND TRUE)
      # Try to extract version from header
      if(EXISTS "${GEOS_INCLUDE_DIRS}/geos_c.h")
        file(STRINGS "${GEOS_INCLUDE_DIRS}/geos_c.h" GEOS_VERSION_STR REGEX "#define GEOS_VERSION ")
        string(REGEX REPLACE ".*\"([^\"]+)\".*" "\\1" GEOS_VERSION "${GEOS_VERSION_STR}")
      endif()
    endif()
  endif()
endif()

if(geos_FOUND)
  add_compile_definitions(HAVE_GEOS)
  message(STATUS "GEOS found via CMake config: ${geos_VERSION}")
elseif(GEOS_FOUND)
  add_compile_definitions(HAVE_GEOS)
  message(STATUS "GEOS found: ${GEOS_VERSION}")
else()
  message(WARNING "GEOS not found — building with stubs. Inspect overlaps and full pipeline will be unavailable.")
endif()

# -------------------------------------------------------------------
# Sources
# -------------------------------------------------------------------
file(GLOB_RECURSE SOURCES src/*.cpp)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS src/*.cpp)

add_executable(${TARGET_NAME} ${SOURCES} ${POLY2TRI_SOURCES})

target_include_directories(${TARGET_NAME} PRIVATE
  ${POLY2TRI_DIR}
  include
)

target_link_libraries(${TARGET_NAME} PRIVATE
  RecastNavigation::Recast
  RecastNavigation::Detour
)

if(GEOS_FOUND)
  target_include_directories(${TARGET_NAME} PRIVATE ${GEOS_INCLUDE_DIRS})
  target_link_libraries(${TARGET_NAME} PRIVATE ${GEOS_LIBRARIES})
  # pkg-config may also set GEOS_LDFLAGS for additional link flags
  if(GEOS_LDFLAGS)
    target_link_libraries(${TARGET_NAME} PRIVATE ${GEOS_LDFLAGS})
  endif()
endif()

if(geos_FOUND)
  target_link_libraries(${TARGET_NAME} PRIVATE GEOS::geos_c)
endif()

# math library — Linux only
if(UNIX AND NOT APPLE)
  target_link_libraries(${TARGET_NAME} PRIVATE m)
endif()

# -------------------------------------------------------------------
# Windows-specific
# -------------------------------------------------------------------
if(WIN32)
  # MSVC: suppress secure-crt warnings for fopen etc.
  target_compile_definitions(${TARGET_NAME} PRIVATE _CRT_SECURE_NO_WARNINGS)
  # MSVC: ensure UTF-8 source encoding
  target_compile_options(${TARGET_NAME} PRIVATE /utf-8)
  # poly2tri throws exceptions — enable C++ exception handling in MSVC
  if(MSVC)
    target_compile_options(${TARGET_NAME} PRIVATE /EHsc)
  endif()
  # Ensure consistent output name (tnavmesh.exe)
  set_target_properties(${TARGET_NAME} PROPERTIES RUNTIME_OUTPUT_NAME tnavmesh)
endif()

# -------------------------------------------------------------------
# Install
# -------------------------------------------------------------------
install(TARGETS ${TARGET_NAME} DESTINATION bin)
install(DIRECTORY include/tnavmesh DESTINATION include)
