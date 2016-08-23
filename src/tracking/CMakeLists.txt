CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(tracking_node C CXX)

# RelWithDebInfo, Release or Debug
SET(CMAKE_BUILD_TYPE Debug FORCE)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++11 -DMAGICKCORE_QUANTUM_DEPTH=16 -DMAGICKCORE_HDRI_ENABLE=0")

# ros
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs cv_bridge image_transport visualization_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)

execute_process(COMMAND dpkg --print-architecture OUTPUT_VARIABLE PROCESSORTYPE)
if ( PROCESSORTYPE STREQUAL "armhf")
    message(STATUS "tracking_node is supposed to run on host not raspi, aborting generation")
else ()
    #==============================================================================
    # Dependencies
    #==============================================================================
    # Eigen
    find_package(PkgConfig)
    pkg_check_modules(EIGEN3 REQUIRED eigen3)
    include_directories(${EIGEN3_INCLUDE_DIRS})

    # OpenCV
    find_package(OpenCV COMPONENTS opencv_core opencv_highgui opencv_imgproc opencv_calib3d REQUIRED)

    # ncurse
    pkg_check_modules(ncurses REQUIRED ncurses)
    include_directories(${ncurses_INCLUDE_DIRS})

    include_directories(
            ${catkin_INCLUDE_DIRS}
            include/
    )
    #==============================================================================
    # Application
    #==============================================================================

    ADD_EXECUTABLE(tracking_node src/markertracker.cpp src/main.cpp)
    add_dependencies(tracking_node communication_gencpp)
    TARGET_LINK_LIBRARIES(tracking_node ${OpenCV_LIBS}  ${catkin_LIBRARIES} ${ncurses_LIBRARIES} -lpthread )

    #==============================================================================
    # Installation
    #==============================================================================
    install( TARGETS        
             tracking_node
             DESTINATION
	     ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    install(FILES 
    	    package.xml
            DESTINATION
            ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

endif ()
