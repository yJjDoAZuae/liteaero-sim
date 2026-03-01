# ---------------------------------------------------------------------------
# LiteAeroSim unit tests
#
# enable_testing() is called at the project root. All external dependencies
# are fetched and made available by the root CMakeLists.txt.
# ---------------------------------------------------------------------------

# Collect test sources: files matching *_test.cpp / *_test.cc in this directory.
# To add a new test file, name it <subject>_test.cpp and it will be picked up
# automatically on the next CMake configure.
file(GLOB TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*_test.cc"
)

if(NOT TEST_SOURCES)
    message(STATUS "liteaerosim_test: no test sources found in ${CMAKE_CURRENT_SOURCE_DIR} — skipping test target")
    return()
endif()

add_executable(liteaerosim_test ${TEST_SOURCES})

target_include_directories(liteaerosim_test PRIVATE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
)

target_compile_features(liteaerosim_test PRIVATE cxx_std_17)

# liteaerosim brings in Eigen, JSON, and trochoids transitively (PRIVATE).
# Eigen3::Eigen and nlohmann_json::nlohmann_json are listed explicitly here
# so that test files may include their headers directly (e.g. for test fixtures
# that construct Eigen matrices or JSON objects directly).
target_link_libraries(liteaerosim_test
    PRIVATE
        liteaerosim
        GTest::gtest_main
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
)

# Register individual test cases with CTest via Google Test discovery.
# WORKING_DIRECTORY keeps test-generated output files (e.g. CSVs) inside the
# build tree instead of the source root.
include(GoogleTest)
gtest_discover_tests(liteaerosim_test
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)
