cmake_minimum_required(VERSION 3.0)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

set(PCLTOOLS_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PCLTOOLS_LIB_DIR})

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(PCLTOOLS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${PCLTOOLS_INCLUDE_DIR})

set(SOURCE_FILES src/pcltools/fileio.cpp src/pcltools/segmentation.cpp include/pcltools/linalg.hpp)

add_library(pcltools SHARED ${SOURCE_FILES})

target_link_libraries(pcltools ${PCL_LIBRARIES})
target_include_directories(pcltools PUBLIC ${PCLTOOLS_INCLUDE_DIR})

