cmake_minimum_required(VERSION 3.16)

project(SLAM LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED NO_MODULE)

add_library(slam_core
    src/Pose2.cpp
    src/ScanLogLoader.cpp
)

target_include_directories(slam_core PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)


target_link_libraries(slam_core Eigen3::Eigen)

add_executable(Pose2Tests tests/Pose2Tests.cpp)
target_link_libraries(Pose2Tests PRIVATE slam_core)

add_executable(ScanLogLoaderTests tests/ScanLogLoaderTests.cpp)
target_link_libraries(ScanLogLoaderTests PRIVATE slam_core)

add_executable(VisualizeScan apps/VisualizeScan.cpp)
target_link_libraries(VisualizeScan PRIVATE slam_core)

enable_testing()

add_test(NAME Pose2Tests COMMAND Pose2Tests)
add_test(NAME ScanLogLoaderTests COMMAND ScanLogLoaderTests)
