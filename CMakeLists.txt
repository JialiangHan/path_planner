cmake_minimum_required(VERSION 3.15)
project(hybrid_astar)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
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

set(CMAKE_BUILD_TYPE Release)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    tf
    # steering_functions
    # base_global_planner
    costmap_2d
    costmap_converter
    cmake_modules
    dynamic_reconfigure
    geometry_msgs
    interactive_markers
    message_generation
    nav_core
    nav_msgs
    pluginlib
    visualization_msgs
    base_local_planner
    tf2_ros
    ompl
    )

FIND_PACKAGE(Glog REQUIRED)
FIND_PACKAGE(Gflags)
find_package(GTest REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development NumPy)
find_package(PythonLibs 3.0 REQUIRED)
include_directories(${Python3_INCLUDE_DIRS} ${NumPy_INCLUDE_DIRS})
# if (${Python_FOUND})
#   include_directories(${Python_INCLUDE_DIRS})
#   include_directories(${Python_NumPy_INCLUDE_DIRS})
# else()
#   message(FATAL_ERROR "Python3 not found, please install it.")
# endif()
find_package(Eigen3 REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIRS}
    include
    test/test_the_plugin/include
    ${catkin_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    )
set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/a_star.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bucketedqueue.cpp #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/src/collisiondetection.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/computational_geometry.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/cubic_bezier.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dubins.cpp #Andrew Walker
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamicvoronoi.cpp #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hybrid_a_star_planner.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hybrid_a_star.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/lookup_table.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/node2d.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/parameter_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/path_evaluator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/path_publisher.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/piecewise_cubic_bezier.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/planner.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ReedsShepp.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rrt.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/smoother.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utility.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/visualize.cpp
    )
set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/a_star.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/bezier.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/bucketedqueue.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/bucketedqueue.h #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/include/collisiondetection.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/computational_geometry.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/constants.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cubic_bezier.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/dubins.h #Andrew Walker
    ${CMAKE_CURRENT_SOURCE_DIR}/include/dynamicvoronoi.h #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/include/expander.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/gradient.h #Andrew Noske
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_a_star_planner.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_a_star.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/lookup_table.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/lookup.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/matplotlibcpp.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/node2d.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/node3d.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/parameter_manager.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/path_evaluator.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/path_publisher.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/piecewise_cubic_bezier.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/planner.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/point.h #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ReedsShepp.h 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/rrt.h 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/smoother.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/utility.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/visualize.h
    )
# add_library(HYAS ${SOURCES} ${HEADERS} )

add_library(hybrid_a_star_planner_lib 
    ${SOURCES})

target_link_libraries(hybrid_a_star_planner_lib
    ${PYTHON_LIBRARIES}                    
    ${catkin_LIBRARIES} 
    ${OMPL_LIBRARIES} 
    # glog::glog 
    ${GFLAGS_LIBRARY} 
)

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
    ${GTEST_INCLUDE_DIRS}
    ) 


add_executable(tf_broadcaster src/tf_broadcaster.cpp   )
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES} glog::glog)
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
add_executable(hybrid_a_star_test 
            test/test.cpp 
            ${SOURCES}
            ${HEADERS}
            )
target_link_libraries(hybrid_a_star_test 
                        ${PYTHON_LIBRARIES}
                        ${catkin_LIBRARIES} 
                        ${OMPL_LIBRARIES}
                        glog::glog 
                        ${GFLAGS_LIBRARY} 
                       ${GTEST_BOTH_LIBRARIES}
                       pthread)


add_executable(test_planner test/test_the_plugin/src/test.cpp test/test_the_plugin/src/test_plugins.cpp)
target_link_libraries(test_planner ${catkin_LIBRARIES} glog::glog)

add_executable(tf_test_broadcaster test/test_the_plugin/src/tf_broadcaster.cpp)
target_link_libraries(tf_test_broadcaster ${catkin_LIBRARIES} glog::glog)

gtest_add_tests(hybrid_a_star_test
                .noArgs
                 AUTO)
add_test(hybrid_a_star_test hybrid_a_star_test)
add_custom_target(check_hybrid_a COMMAND ${CMAKE_CTEST_COMMAND}
                  DEPENDS hybrid_a_star_test)
install(TARGETS ${PROJECT_NAME} tf_broadcaster
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )

    install(FILES bgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)