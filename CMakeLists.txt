cmake_minimum_required(VERSION 3.15)
project(hybrid_astar)

## C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_BUILD_TYPE "Debug")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    tf
    steering_functions
    )
find_package(PythonLibs REQUIRED)
FIND_PACKAGE(Glog REQUIRED)
FIND_PACKAGE(Gflags)
find_package(Python COMPONENTS Interpreter Development NumPy)
if (${Python_FOUND})
  include_directories(${Python_INCLUDE_DIRS})
  include_directories(${Python_NumPy_INCLUDE_DIRS})
else()
  message(FATAL_ERROR "Python3 not found, please install it.")
endif()

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/node2d.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/node3d.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/collisiondetection.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/planner.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/path.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/smoother.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/visualize.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dubins.cpp #Andrew Walker
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamicvoronoi.cpp #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bucketedqueue.cpp #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/src/path_evaluator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utility.cpp
    )
set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/algorithm.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/node2d.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/node3d.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/collisiondetection.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/planner.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/path.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/smoother.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/vector2d.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/visualize.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/helper.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/constants.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/lookup.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/gradient.h #Andrew Noske
    ${CMAKE_CURRENT_SOURCE_DIR}/include/dubins.h #Andrew Walker
    ${CMAKE_CURRENT_SOURCE_DIR}/include/dynamicvoronoi.h #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/include/bucketedqueue.h #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/include/point.h #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/include/path_evaluator.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/utility.h
    )
add_library(HYAS ${SOURCES} ${HEADERS} )
## Declare a catkin package
catkin_package()

## OPEN MOTION PLANNING LIBRARY
find_package(ompl REQUIRED)

if(NOT OMPL_FOUND)
    message(AUTHOR_WARNING,"Open Motion Planning Library not found")
endif(NOT OMPL_FOUND)


include_directories(
    include 
    ${catkin_INCLUDE_DIRS} 
    ${OMPL_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
    ${GFLAGS_INCLUDE_DIRS}
    ) 


add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})
# guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt. ")
endif()

add_executable(hybrid_astar src/main.cpp ${HEADERS} ${SOURCES})
target_link_libraries(hybrid_astar
${PYTHON_LIBRARIES}                    
${catkin_LIBRARIES} 
                    ${OMPL_LIBRARIES} 
                    glog::glog 
                        ${GFLAGS_LIBRARY} 
                    ) 
add_executable(node_path_evaluator 
            src/node_path_evaluator.cpp 
            ${SOURCES}
            ${HEADERS}
            )
target_link_libraries(node_path_evaluator 
                        ${PYTHON_LIBRARIES}
                        ${catkin_LIBRARIES} 
                        ${OMPL_LIBRARIES}
                        glog::glog 
                        ${GFLAGS_LIBRARY} 
)

install(TARGETS ${PROJECT_NAME} tf_broadcaster
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )
