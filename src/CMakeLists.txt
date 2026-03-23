# ---------------------------------------------------------------------------
# LiteAeroSim core library
#
# All external dependencies (Eigen3, nlohmann_json, trochoids) are fetched and
# made available by the root CMakeLists.txt before this file is processed.
# ---------------------------------------------------------------------------

# Collect all C++ source files under src/ recursively.
# CONFIGURE_DEPENDS causes CMake to re-glob when files are added or removed.
file(GLOB_RECURSE LITEAEROSIM_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cxx"
)

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

# External dependencies are PRIVATE: they are implementation details of the
# library and do not form part of its public API. Consumers of liteaerosim
# do not need to link these separately.
target_link_libraries(liteaerosim
    PUBLIC
        liteaero::log
    PRIVATE
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
        trochoids
        liteaerosim_proto
        tinygltf_headers
)

target_compile_features(liteaerosim PUBLIC cxx_std_17)
set_target_properties(liteaerosim PROPERTIES
    POSITION_INDEPENDENT_CODE ${CMAKE_POSITION_INDEPENDENT_CODE}
)

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
