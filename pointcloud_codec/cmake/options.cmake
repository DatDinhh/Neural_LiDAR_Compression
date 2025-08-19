cmake_minimum_required(VERSION 3.21)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_TESTS "Build unit tests" ON)
option(WITH_PCL "Enable PCL-based I/O helpers" ON)
option(WITH_DRACO "Enable Draco compression backend" ON)
option(WITH_CUDA "Enable CUDA-accelerated paths (future)" OFF)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if (MSVC)
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
endif()

if (WITH_PCL)
  add_compile_definitions(PCC_WITH_PCL=1)
else()
  add_compile_definitions(PCC_WITH_PCL=0)
endif()

if (WITH_DRACO)
  add_compile_definitions(PCC_WITH_DRACO=1)
else()
  add_compile_definitions(PCC_WITH_DRACO=0)
endif()

if (WITH_CUDA)
  add_compile_definitions(PCC_WITH_CUDA=1)
  # find_package(CUDAToolkit REQUIRED)
else()
  add_compile_definitions(PCC_WITH_CUDA=0)
endif()

message(STATUS "=== Build options ===")
message(STATUS "  CMAKE_BUILD_TYPE:     ${CMAKE_BUILD_TYPE}")
message(STATUS "  CMAKE_CXX_COMPILER:   ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  WITH_PCL:             ${WITH_PCL}")
message(STATUS "  WITH_DRACO:           ${WITH_DRACO}")
message(STATUS "  WITH_CUDA:            ${WITH_CUDA}")
message(STATUS "  BUILD_TESTS:          ${BUILD_TESTS}")
