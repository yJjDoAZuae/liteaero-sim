cmake_minimum_required(VERSION 3.16)
project(LiteAeroSim LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Project-wide settings
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(BUILD_SHARED_LIBS "Build libraries as shared instead of static" OFF)

# Enable CTest at the project root so all subdirectories are registered
enable_testing()

# ---------------------------------------------------------------------------
# Dependencies
#
# All external dependencies are declared here in one place.
# See docs/guidelines/cpp.md — "External Dependency Management" for policy.
#
# Registry:
#   Dependency       | Version/SHA                              | License      | Method
#   -----------------|------------------------------------------|--------------|----------------
#   Eigen3           | 3.4+ (system)                            | MPL-2        | find_package
#   nlohmann_json    | v3.12.0                                  | MIT          | FetchContent (1a)
#   googletest       | v1.17.0                                  | BSD-3-Clause | FetchContent (1a)
#   trochoids        | 38d23eb3346737fe9d6e9ff57c742113e29dfe4f | Clear BSD    | FetchContent (1b)
#   protobuf         | v3.21.12                                 | BSD-3-Clause | find_package + FetchContent (1a)
#   mcap             | releases/cpp/v1.4.0                      | MIT          | FetchContent (1b)
# ---------------------------------------------------------------------------
include(FetchContent)

# --- Eigen3 (MPL-2) — linear algebra ---
# Allow the caller to point at a non-system Eigen installation via the
# EIGEN3_INCLUDE_DIR environment variable.
if(DEFINED ENV{EIGEN3_INCLUDE_DIR})
    list(APPEND CMAKE_PREFIX_PATH "$ENV{EIGEN3_INCLUDE_DIR}")
endif()
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

# --- nlohmann/json v3.12.0 (MIT) — JSON serialization ---
# Prefer a system installation; fall back to FetchContent.
find_package(nlohmann_json 3.12 QUIET)
if(NOT nlohmann_json_FOUND)
    FetchContent_Declare(
        nlohmann_json
        URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
    )
    FetchContent_MakeAvailable(nlohmann_json)
endif()

# --- Google Test v1.17.0 (BSD-3-Clause) — unit testing ---
# Prefer a system installation; fall back to FetchContent.
find_package(GTest QUIET)
if(NOT GTest_FOUND)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
    )
    # Prevent gtest from overriding the parent project's compiler/linker settings on MSVC
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
endif()

# --- trochoids (Clear BSD) — time-optimal UAV path planning (Dubins + trochoids) ---
# Source: https://github.com/castacks/trochoids
# Pattern 1b: upstream CMakeLists.txt uses catkin/ROS and is not compatible with this
# project. The library source has no ROS dependencies, so we use FetchContent_Populate
# to download the source and define our own target, bypassing the upstream build system.
#
# NOTE: FetchContent_Populate(<name>) is deprecated in CMake 3.30. If the project minimum
# is raised to 3.28+, migrate to: FetchContent_Declare(...) with a cmake_override/
# directory providing a standalone CMakeLists.txt, then use FetchContent_MakeAvailable.
FetchContent_Declare(
    trochoids
    GIT_REPOSITORY https://github.com/castacks/trochoids.git
    GIT_TAG        38d23eb3346737fe9d6e9ff57c742113e29dfe4f
    # GIT_SHALLOW omitted — shallow clones require a tag or branch, not a bare commit SHA
)
FetchContent_GetProperties(trochoids)
if(NOT trochoids_POPULATED)
    FetchContent_Populate(trochoids)
    add_library(trochoids STATIC
        ${trochoids_SOURCE_DIR}/src/trochoids.cpp
        ${trochoids_SOURCE_DIR}/src/trochoid_utils.cpp
        ${trochoids_SOURCE_DIR}/src/DubinsStateSpace.cpp
    )
    # SYSTEM suppresses warnings from third-party headers in consumer targets
    target_include_directories(trochoids SYSTEM PUBLIC
        ${trochoids_SOURCE_DIR}/include
    )
    target_compile_features(trochoids PUBLIC cxx_std_17)
    # Suppress warnings from third-party source — we do not own this code
    target_compile_options(trochoids PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-w>
        $<$<CXX_COMPILER_ID:MSVC>:/w>
    )
endif()

# --- protobuf v3.21.12 (BSD-3-Clause) — binary serialization ---
# v3.21.x is the last pre-abseil release; avoids pulling in ~30 abseil targets.
# Prefer a system installation (e.g. MSYS2 pacman); fall back to FetchContent.
find_package(Protobuf QUIET)
if(NOT Protobuf_FOUND)
    FetchContent_Declare(
        protobuf
        GIT_REPOSITORY https://github.com/protocolbuffers/protobuf.git
        GIT_TAG        v3.21.12
    )
    set(protobuf_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
    set(protobuf_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(protobuf)
endif()

# --- mcap (MIT) — MCAP binary log format; C++ header-with-implementation library ---
# Pattern 1b: upstream CMakeLists in cpp/mcap/ pulls in optional compression deps.
# We download source and expose only the include path via an INTERFACE target.
# MCAP_IMPLEMENTATION is defined in src/logger/mcap_impl.cpp.
# Compression is disabled at compile time (no lz4/zstd deps required).
FetchContent_Declare(
    mcap
    GIT_REPOSITORY https://github.com/foxglove/mcap.git
    GIT_TAG        releases/cpp/v1.4.0
)
FetchContent_GetProperties(mcap)
if(NOT mcap_POPULATED)
    FetchContent_Populate(mcap)
    add_library(mcap_headers INTERFACE)
    target_include_directories(mcap_headers SYSTEM INTERFACE
        ${mcap_SOURCE_DIR}/cpp/mcap/include
    )
    target_compile_definitions(mcap_headers INTERFACE
        MCAP_COMPRESSION_NO_LZ4
        MCAP_COMPRESSION_NO_ZSTD
    )
endif()

# ---------------------------------------------------------------------------
# Subdirectories
# ---------------------------------------------------------------------------
add_subdirectory(proto)
add_subdirectory(src)
add_subdirectory(test)
