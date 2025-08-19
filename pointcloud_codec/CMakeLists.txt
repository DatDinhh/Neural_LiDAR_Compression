cmake_minimum_required(VERSION 3.22)

project(pointcloud_codec
  VERSION 0.1.0
  DESCRIPTION "LiDAR point cloud codec (quantisation + Draco back-end) with CLI and examples"
  LANGUAGES CXX C)

# Build options
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(BUILD_TESTS        "Build unit tests"            OFF)
option(BUILD_BENCHMARKS   "Build benchmarks"            OFF)
option(BUILD_VIEWER       "Build PCL viewer (needs VTK)" ON)
option(WITH_CUDA          "Build CUDA components"       OFF) # placeholder flag

# Helper: warnings
function(pcc_target_warnings tgt)
  if (MSVC)
    target_compile_options(${tgt} PRIVATE /W4 /permissive-)
    target_compile_definitions(${tgt} PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX)
  else()
    target_compile_options(${tgt} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endfunction()

# Dependencies via vcpkg
# Core
find_package(draco  CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(cxxopts CONFIG REQUIRED)
find_package(PCL CONFIG REQUIRED COMPONENTS common io)

# Library: pcc
add_library(pcc
  src/codec/stream_encoder.cpp
  src/codec/stream_decoder.cpp
  src/codec/quantisation.cpp
  src/codec/draco_bridge.cpp
)

target_include_directories(pcc
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${PCL_INCLUDE_DIRS}
)

# Link PCL the safe way to pull transitive dependencies automatically
target_link_libraries(pcc
  PUBLIC
    draco::draco
    Eigen3::Eigen
    ${PCL_LIBRARIES}
)

target_compile_definitions(pcc PRIVATE ${PCL_DEFINITIONS})
pcc_target_warnings(pcc)

# CLI tool
add_executable(pcc_tool
  src/cli/main.cpp
  src/cli/cli_options.cpp
  src/io/pcd_reader.cpp
  src/io/las_reader.cpp
)

target_include_directories(pcc_tool PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pcc_tool
  PRIVATE
    pcc
    cxxopts::cxxopts
    ${PCL_LIBRARIES}  
)
target_compile_definitions(pcc_tool PRIVATE ${PCL_DEFINITIONS})
pcc_target_warnings(pcc_tool)

# Draco + PCL I/O
if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/examples/minimal_encode.cpp")
  add_executable(minimal_encode examples/minimal_encode.cpp)
  target_include_directories(minimal_encode PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(minimal_encode
    PRIVATE
      draco::draco
      ${PCL_LIBRARIES}
  )
  target_compile_definitions(minimal_encode PRIVATE ${PCL_DEFINITIONS})
  pcc_target_warnings(minimal_encode)
endif()

# Optional viewer
if (BUILD_VIEWER AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/viz/pcl_view.cpp")
  find_package(PCL CONFIG QUIET COMPONENTS visualization)
  if (PCL_VISUALIZATION_LIBRARY OR PCL_VISUALIZATION_FOUND)
    add_executable(pcc_viewer src/viz/pcl_view.cpp)
    target_include_directories(pcc_viewer PRIVATE ${PCL_INCLUDE_DIRS})
    target_link_libraries(pcc_viewer PRIVATE ${PCL_LIBRARIES})
    target_compile_definitions(pcc_viewer PRIVATE ${PCL_DEFINITIONS})
    pcc_target_warnings(pcc_viewer)
    message(STATUS "PCL viewer enabled (visualization component found).")
  else()
    message(STATUS "PCL visualization not found (likely no VTK on this triplet). Skipping viewer.")
  endif()
endif()

# Tests 
if (BUILD_TESTS)
  enable_testing()
  find_package(GTest CONFIG REQUIRED)
  add_subdirectory(src/tests)
endif()

# Benchmarks
if (BUILD_BENCHMARKS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/benchmarks/CMakeLists.txt")
  find_package(benchmark CONFIG QUIET)
  if (TARGET benchmark::benchmark)
    add_subdirectory(src/benchmarks)
  else()
    message(STATUS "Google Benchmark not found; skipping benchmarks.")
  endif()
endif()

# Install
install(TARGETS pcc
  EXPORT pointcloud_codecTargets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
# install(TARGETS pcc_tool minimal_encode

install(DIRECTORY include/ DESTINATION include)

# RMSE tool (PCL required)
add_executable(pcd_rmse src/tools/pcd_rmse.cpp)
target_link_libraries(pcd_rmse PRIVATE ${PCL_LIBRARIES})
target_include_directories(pcd_rmse PRIVATE ${PCL_INCLUDE_DIRS})

add_executable(pcd_chamfer src/tools/pcd_chamfer.cpp)
target_link_libraries(pcd_chamfer PRIVATE ${PCL_LIBRARIES})
target_include_directories(pcd_chamfer PRIVATE ${PCL_INCLUDE_DIRS})

# Export cmake package
install(EXPORT pointcloud_codecTargets
  NAMESPACE pcc::
  DESTINATION share/pointcloud_codec
)
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/pointcloud_codecConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/pointcloud_codecConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/pointcloud_codecConfig.cmake"
  @ONLY
)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/pointcloud_codecConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/pointcloud_codecConfigVersion.cmake"
  DESTINATION share/pointcloud_codec
)

message(STATUS "---- Summary ----")
message(STATUS "C++ standard        : ${CMAKE_CXX_STANDARD}")
message(STATUS "Build tests         : ${BUILD_TESTS}")
message(STATUS "Build benchmarks    : ${BUILD_BENCHMARKS}")
message(STATUS "Build viewer        : ${BUILD_VIEWER}")
message(STATUS "-------------------")
