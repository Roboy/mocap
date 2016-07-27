CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(vision_node C CXX)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# RelWithDebInfo, Release or Debug
SET(CMAKE_BUILD_TYPE Debug FORCE)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++11")

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake )

execute_process(COMMAND grep -o BCM2709 /proc/cpuinfo OUTPUT_VARIABLE IAMRASPBERRYPI3)
if (NOT IAMRASPBERRYPI3)
    message(STATUS "vision_node is supposed to run on raspi not host, aborting generation")
else ()
    #==============================================================================
    # Dependencies
    #==============================================================================
    # ros
    find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
    catkin_package(CATKIN_DEPENDS message_runtime)

    # OpenCV
    find_package(OpenCV COMPONENTS opencv_core opencv_highgui opencv_imgproc opencv_calib3d REQUIRED)

    # raspi camera
    find_package(picam REQUIRED)
    include_directories(
	    ${picam_INCLUDE_DIRS}
	    ${CMAKE_CURRENT_SOURCE_DIR}/picam
	    )

    include_directories(
            ${catkin_INCLUDE_DIRS}
            include/
    )
    #==============================================================================
    # Application
    #==============================================================================
    ADD_EXECUTABLE(vision_node src/visionNode.cpp src/main.cpp)
    add_dependencies(vision_node communication_gencpp)
    TARGET_LINK_LIBRARIES(vision_node ${OpenCV_LIBS} ${catkin_LIBRARIES} ${picam_LIBRARIES} -lpthread)
endif ()
