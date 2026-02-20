cmake_minimum_required(VERSION 3.14)
project(cppplot VERSION 1.0.0 LANGUAGES CXX)

# C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Options
option(CPPPLOT_BUILD_EXAMPLES "Build example programs" ON)
option(CPPPLOT_BUILD_TESTS "Build test programs" ON)
option(CPPPLOT_HEADER_ONLY "Use header-only mode" ON)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Header-only library (interface library)
add_library(cppplot INTERFACE)
target_include_directories(cppplot INTERFACE 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(cppplot INTERFACE cxx_std_17)

# Examples
if(CPPPLOT_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Tests
if(CPPPLOT_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Installation
install(DIRECTORY include/cppplot DESTINATION include)
install(TARGETS cppplot EXPORT cppplotTargets)
install(EXPORT cppplotTargets
    FILE cppplotTargets.cmake
    NAMESPACE cppplot::
    DESTINATION lib/cmake/cppplot
)

# Package configuration
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/cppplotConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cppplotConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cppplotConfig.cmake"
    INSTALL_DESTINATION lib/cmake/cppplot
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/cppplotConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/cppplotConfigVersion.cmake"
    DESTINATION lib/cmake/cppplot
)

message(STATUS "cppplot v${PROJECT_VERSION} configured")
message(STATUS "  Build examples: ${CPPPLOT_BUILD_EXAMPLES}")
message(STATUS "  Build tests: ${CPPPLOT_BUILD_TESTS}")
