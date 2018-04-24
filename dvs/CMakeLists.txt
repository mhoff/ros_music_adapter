cmake_minimum_required(VERSION 2.8)

project("ros_music_dvs_adapter")

# add module for finding MUSIC
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")

# generates catkin_INCLUDE_DIRS and catkin_LIBRARIES for all components
find_package(catkin REQUIRED
    COMPONENTS
    ros_music_adapter
    dvs_msgs
    roscpp
    roscpp_serialization
    rosconsole
    rostime
)

find_package(MUSIC REQUIRED)
find_package(MPI REQUIRED)

set(CMAKE_CXX_COMPILE_FLAGS ${CMAKE_CXX_COMPILE_FLAGS} ${MPI_COMPILE_FLAGS})
set(CMAKE_CXX_LINK_FLAGS ${CMAKE_CXX_LINK_FLAGS} ${MPI_LINK_FLAGS})
set(CMAKE_CXX_FLAGS "-O3")


catkin_package(
    CATKIN_DEPENDS
        ros_music_adapter
        dvs_msgs
        roscpp
        roscpp_serialization
        rosconsole
        rostime
    DEPENDS
        MUSIC
        MPI
)

find_library(GSL gsl REQUIRED)
find_library(BLAS gslcblas REQUIRED)
find_library(PTHREAD pthread REQUIRED)

message (STATUS "catkin include: " ${catkin_INCLUDE_DIRS})
message (STATUS "catkin libraries: " ${catkin_LIBRARIES})

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${MUSIC_INCLUDE_DIRS}
    ${MPI_INCLUDE_PATH}
)

# TODO static library rtclock is found implicitly... hack?
# find_library(RTCLOCK rtclock REQUIRED)
include_directories(../core/lib/)

add_executable(dvs_adapter adapters/dvs_adapter.cpp)
add_dependencies(dvs_adapter ${catkin_EXPORTED_TARGETS})

target_link_libraries(dvs_adapter ${catkin_LIBRARIES} ${MUSIC_LIBRARIES} ${PTHREAD} ${MPI_LIBRARIES} rtclock)

install(
    TARGETS dvs_adapter
    RUNTIME DESTINATION bin
)
