cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(HandEyeCalib_RECODE VERSION 1.0)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Debug")

find_package(OpenCV REQUIRED)
find_package (Eigen3 REQUIRED)
find_package(PCL 1.11 REQUIRED COMPONENTS common io)

set(HeadFiles "utils.hpp" "FeatureDetection.hpp") 
set(SourceFiles "main.cc" "utils.cc" "FeatureDetection.cc")

add_executable(${PROJECT_NAME} ${HeadFiles} ${SourceFiles})

target_link_libraries(${PROJECT_NAME} PRIVATE ${PCL_LIBRARIES} Eigen3::Eigen ${OpenCV_LIBS})