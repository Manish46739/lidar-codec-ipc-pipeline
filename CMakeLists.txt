cmake_minimum_required(VERSION 3.14)
project(LidarCodecIpcPipeline VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

add_compile_options(-Wall -Wextra -Wpedantic)
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3)
endif()

if(ENABLE_SANITIZERS)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(Threads REQUIRED)

# 1. Synthetic LiDAR UDP Simulator
add_executable(lidar_simulator src/simulator_main.cpp)
target_link_libraries(lidar_simulator PRIVATE Threads::Threads)

add_executable(lidar_replay src/replay_main.cpp)
target_link_libraries(lidar_replay PRIVATE Threads::Threads)

# 2. Driver node (epoll + SPSC + codec + POSIX shared memory)
add_executable(lidar_driver src/driver_main.cpp)
target_link_libraries(lidar_driver PRIVATE Threads::Threads rt)

# 3. Downstream consumer (shared-memory reader + PLY exporter)
add_executable(perception_consumer src/consumer_main.cpp)
target_link_libraries(perception_consumer PRIVATE Threads::Threads rt)

# 4. Unit Tests with GoogleTest
find_package(GTest REQUIRED)

enable_testing()
add_executable(test_vlp16_codec tests/test_vlp16_codec.cpp)
target_link_libraries(test_vlp16_codec PRIVATE GTest::gtest_main Threads::Threads)

include(GoogleTest)
gtest_discover_tests(test_vlp16_codec)
