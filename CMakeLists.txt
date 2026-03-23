cmake_minimum_required(VERSION 3.16)
project(LiteAeroSim LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Project-wide settings
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Prevent CMake from using @response-files for include paths in compile_commands.json,
# which VSCode IntelliSense cannot expand.
set(CMAKE_CXX_USE_RESPONSE_FILE_FOR_INCLUDES OFF)

option(BUILD_SHARED_LIBS "Build libraries as shared instead of static" OFF)

# M_PI and other POSIX math constants are not defined in strict C++17 mode
# (-std=c++17, not -std=gnu++17). The Conan toolchain disables GNU extensions,
# so _USE_MATH_DEFINES must be set globally to restore them on MinGW/MSVC.
add_compile_definitions(_USE_MATH_DEFINES)

# Enable CTest at the project root so all subdirectories are registered
enable_testing()

# ---------------------------------------------------------------------------
# Dependencies
#
# C++ dependencies are managed via Conan 2.x. Run:
#   conan install . --output-folder=build --build=missing --profile=liteaero-gcc
# then configure with:
#   cmake -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake
#
# See docs/dependencies/README.md and docs/installation/README.md.
#
# Registry:
#   Dependency       | Version/SHA                              | License      | Method
#   -----------------|------------------------------------------|--------------|----------------------
#   Eigen3           | 3.4.0                                    | MPL-2        | Conan (find_package)
#   nlohmann_json    | 3.12.0                                   | MIT          | Conan (find_package)
#   gtest            | 1.14.0                                   | BSD-3-Clause | Conan (find_package)
#   protobuf         | 3.21.12                                  | BSD-3-Clause | Conan (find_package)
#   trochoids        | 38d23eb3346737fe9d6e9ff57c742113e29dfe4f | Clear BSD    | FetchContent (pattern 1b — catkin CMake)
#   mcap             | releases/cpp/v1.4.0                      | MIT          | FetchContent (pattern 1b — header-only)
#   tinygltf         | v2.9.3                                   | MIT          | FetchContent (pattern 1b — header-only)
# ---------------------------------------------------------------------------

# --- Eigen3 (MPL-2) — linear algebra ---
find_package(Eigen3 REQUIRED NO_MODULE)

# --- nlohmann/json (MIT) — JSON serialization ---
find_package(nlohmann_json REQUIRED)

# --- Google Test (BSD-3-Clause) — unit testing ---
find_package(GTest REQUIRED)

include(FetchContent)

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

# --- protobuf (BSD-3-Clause) — binary serialization ---
# v3.21.x is the last pre-abseil release; avoids pulling in ~30 abseil targets.
find_package(protobuf REQUIRED CONFIG)

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

# --- tinygltf v2.9.3 (MIT) — glTF 2.0 / GLB export for game engine rendering ---
# Pattern 1b: upstream CMakeLists.txt pulls in stb_image and optional draco;
# we only need the header.  FetchContent_Populate + manual INTERFACE target.
# stb_image disabled via compile definitions; nlohmann_json is already provided.
# TINYGLTF_IMPLEMENTATION is defined in exactly one TU: src/environment/tinygltf_impl.cpp.
FetchContent_Declare(
    tinygltf
    GIT_REPOSITORY https://github.com/syoyo/tinygltf.git
    GIT_TAG        v2.9.3
    GIT_SHALLOW    TRUE
)
FetchContent_GetProperties(tinygltf)
if(NOT tinygltf_POPULATED)
    FetchContent_Populate(tinygltf)
    add_library(tinygltf_headers INTERFACE)
    target_include_directories(tinygltf_headers SYSTEM INTERFACE
        ${tinygltf_SOURCE_DIR}
    )
    target_compile_definitions(tinygltf_headers INTERFACE
        TINYGLTF_NO_STB_IMAGE
        TINYGLTF_NO_STB_IMAGE_WRITE
        TINYGLTF_NO_EXTERNAL_IMAGE
    )
endif()

# ---------------------------------------------------------------------------
# Subdirectories
# ---------------------------------------------------------------------------
add_subdirectory(proto)
add_subdirectory(src)
add_subdirectory(test)
