cmake_minimum_required(VERSION 3.0)

project(pcltools)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
#set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/lib)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${CMAKE_HOME_DIRECTORY}/include)

set(SOURCE_FILES src/fileio.cpp src/segmentation.cpp)

add_library(pcltools SHARED ${SOURCE_FILES})

target_link_libraries(pcltools ${PCL_LIBRARIES})
#add_executable(pcl_base_project ${SOURCE_FILES})


#target_link_libraries(pcl_base_project ${OpenCV_LIBS})

