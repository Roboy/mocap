CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(tracking_node C CXX)

# RelWithDebInfo, Release or Debug
SET(CMAKE_BUILD_TYPE Debug FORCE)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++11 -DMAGICKCORE_QUANTUM_DEPTH=16 -DMAGICKCORE_HDRI_ENABLE=0")

execute_process(COMMAND grep -o BCM2709 /proc/cpuinfo OUTPUT_VARIABLE IAMRASPBERRYPI3)
if (IAMRASPBERRYPI3)
    message(STATUS "tracking_node is supposed to run on host not raspi, aborting generation")
else ()
	message(STATUS ${PROCESSORTYPE})
    # ros
    find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs cv_bridge image_transport visualization_msgs rviz)
    catkin_package(CATKIN_DEPENDS message_runtime)
    #==============================================================================
    # Dependencies
    #==============================================================================
    #Qt4
    find_package(Qt4)
    find_package(Protobuf REQUIRED)
    include(${QT_USE_FILE})
    add_definitions(${QT_DEFINITIONS})

    # Eigen
    find_package(PkgConfig)
    pkg_check_modules(EIGEN3 REQUIRED eigen3)
    include_directories(${EIGEN3_INCLUDE_DIRS})

    # OpenCV
    find_package(OpenCV 2.4.12 EXACT COMPONENTS opencv_core opencv_highgui opencv_imgproc opencv_calib3d REQUIRED)

    # ncurse
    pkg_check_modules(ncurses REQUIRED ncurses)
    include_directories(${ncurses_INCLUDE_DIRS})

    include_directories(
            ${catkin_INCLUDE_DIRS}
            include/
            ${CMAKE_CURRENT_SOURCE_DIR}/../communication/include
    )
    #==============================================================================
    # Application
    #==============================================================================
    qt4_wrap_cpp(MOC_FILES include/mocap_plugin.hpp)
    add_library(mocap_rviz src/mocap_plugin.cpp src/cameraMarkerModel.cpp ${MOC_FILES})
    add_dependencies(mocap_rviz communication_gencpp)
    target_link_libraries(mocap_rviz ${QT_LIBRARIES} ${catkin_LIBRARIES})

    #==============================================================================
    # Installation
    #==============================================================================
    install(TARGETS
            mocap_rviz
            DESTINATION
            ${CATKIN_PACKAGE_LIB_DESTINATION}
            )
    install(FILES
            package.xml
            DESTINATION
            ${CATKIN_PACKAGE_SHARE_DESTINATION}
            )

endif ()
