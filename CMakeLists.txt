cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

project(col2plane)

include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_SOURCE_DIR}/include)
find_package( OpenCV REQUIRED )
find_package (Eigen3 REQUIRED NO_MODULE)
find_package( CCTag REQUIRED)

file(GLOB_RECURSE SOURCES "main.cpp" "src/*.cpp" "src/*.h" "include/*.h")

add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME}   CCTag::CCTag 
                                        Eigen3::Eigen
                                        ${OpenCV_LIBS})




