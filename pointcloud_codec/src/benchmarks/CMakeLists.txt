# Only configure benchmarks when Google Benchmark exists
find_package(benchmark CONFIG QUIET)
if(NOT TARGET benchmark::benchmark)
  message(STATUS "Google Benchmark not found; skipping benchmarks.")
  return()
endif()
cmake_minimum_required(VERSION 3.20)

add_executable(bench_encode bench_encode.cpp)
target_compile_features(bench_encode PRIVATE cxx_std_20)
target_link_libraries(bench_encode PRIVATE
    pcl_io pcl_common
    benchmark::benchmark
)
if (MSVC)
  target_compile_options(bench_encode PRIVATE /wd4996)
endif()

