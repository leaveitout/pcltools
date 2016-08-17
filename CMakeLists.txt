cmake_minimum_required(VERSION 3.0)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

set(PCLTOOLS_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PCLTOOLS_LIB_DIR})

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(PCLTOOLS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${PCLTOOLS_INCLUDE_DIR})

set(SOURCE_FILES
        src/pcltools/fileio.cpp
        src/pcltools/segmentation.cpp
        src/pcltools/pipeline/node.cpp
        src/pcltools/object_pose_storage.cpp
        src/pcltools/arm_pose_storage.cpp
        include/pcltools/common.hpp
        include/pcltools/fileio.hpp
        include/pcltools/linalg.hpp
        include/pcltools/segmentation.hpp
        include/pcltools/pipeline/buffer.hpp
        include/pcltools/pipeline/node.hpp
        include/pcltools/pipeline/processor.hpp
        include/pcltools/pipeline/producer.hpp
        include/pcltools/pipeline/threaded_cloud_viewer.hpp
        include/pcltools/pipeline/cloud_producer.hpp
        include/pcltools/object_pose_storage.hpp
        include/pcltools/arm_pose_storage.hpp
        src/pcltools/palette.cpp include/pcltools/palette.hpp)

add_library(pcltools SHARED ${SOURCE_FILES})

target_link_libraries(pcltools ${PCL_LIBRARIES} ${OpenCV_LIBS})

target_include_directories(pcltools PUBLIC ${PCLTOOLS_INCLUDE_DIR})

