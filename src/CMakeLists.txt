# ---------------------------------------------------------------------------
# LiteAeroSim core library
#
# All external dependencies (Eigen3, nlohmann_json, trochoids) are fetched and
# made available by the root CMakeLists.txt before this file is processed.
# ---------------------------------------------------------------------------

# Collect all C++ source files under src/ recursively, excluding src/tools/
# (tools/ contains main() executables built separately in the tools/ subdirectory).
# CONFIGURE_DEPENDS causes CMake to re-glob when files are added or removed.
file(GLOB_RECURSE LITEAEROSIM_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cxx"
)
list(FILTER LITEAEROSIM_SOURCES EXCLUDE REGEX ".*/tools/.*")
# Exclude src/python/: pybind11 extension files are built as a separate target.
list(FILTER LITEAEROSIM_SOURCES EXCLUDE REGEX ".*/python/.*")

# Core simulation library (static or shared, governed by BUILD_SHARED_LIBS)
add_library(liteaerosim ${LITEAEROSIM_SOURCES})

# Public headers are in the top-level include/ directory.
# Generator expressions ensure the correct path is used for build vs install.
target_include_directories(liteaerosim
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# SDL2 is PUBLIC because SDL_Scancode and Sint16 appear in public headers
# (KeyboardInputConfig, JoystickInput). Consumers that include these headers
# need SDL2 on their include path.
target_link_libraries(liteaerosim
    PUBLIC
        liteaero::log
        liteaero::control
        liteaero::terrain
        liteaero::nav
        SDL2::SDL2
    PRIVATE
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
        trochoids
        liteaerosim_proto
        tinygltf_headers
        # Winsock2 is required by UdpSimulationBroadcaster on Windows.
        $<$<PLATFORM_ID:Windows>:ws2_32>
)

target_compile_features(liteaerosim PUBLIC cxx_std_17)
set_target_properties(liteaerosim PROPERTIES
    POSITION_INDEPENDENT_CODE ${CMAKE_POSITION_INDEPENDENT_CODE}
)

# ---------------------------------------------------------------------------
# Python extension module (optional)
# ---------------------------------------------------------------------------

if(LITEAERO_SIM_BUILD_PYTHON_BINDINGS)
    add_subdirectory(python)
endif()

# ---------------------------------------------------------------------------
# Install rules (uncomment when packaging is needed)
# ---------------------------------------------------------------------------
# install(TARGETS liteaerosim
#     EXPORT  liteaerosimTargets
#     ARCHIVE DESTINATION lib
#     LIBRARY DESTINATION lib
#     RUNTIME DESTINATION bin
# )
# install(DIRECTORY "${CMAKE_SOURCE_DIR}/include/" DESTINATION include)
# install(EXPORT liteaerosimTargets
#     FILE      liteaerosimTargets.cmake
#     NAMESPACE liteaerosim::
#     DESTINATION lib/cmake/liteaerosim
# )
