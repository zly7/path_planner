cmake_minimum_required(VERSION 2.8.3)
project(hybrid_astar)


## C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Ofast -funroll-loops -finline-functions -ffast-math -Wall")
    # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O0 -Wall")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Ofast -funroll-loops -finline-functions -ffast-math")
    # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++14 or C++11 support. Please use a different C++ compiler.")
endif()
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    tf
    )


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
    ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithmcontour.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithmsplit.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/multiGoalAstar.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/directionVector.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rrtalgorithm.cpp
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
    ${CMAKE_CURRENT_SOURCE_DIR}/include/algorithmcontour.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/algorithmsplit.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/multiGoalAstar.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/directionVector.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/rrtalgorithm.h
    )
add_library(HYAS ${SOURCES} ${HEADERS})

## Declare a catkin package
catkin_package()

## OPEN MOTION PLANNING LIBRARY
find_package(ompl REQUIRED)

if(NOT OMPL_FOUND)
    message(AUTHOR_WARNING,"Open Motion Planning Library not found")
endif(NOT OMPL_FOUND)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(CGAL REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${OMPL_INCLUDE_DIRS})
include_directories(include include)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    add_definitions("-DHAS_BOOST")
endif()
add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

add_executable(hybrid_astar src/main.cpp ${HEADERS} ${SOURCES})
target_link_libraries(hybrid_astar 
    ${catkin_LIBRARIES}
    ${OMPL_LIBRARIES}
    ${OpenCV_LIBS}
    CGAL::CGAL
    yaml-cpp
    ${Boost_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} tf_broadcaster
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )
