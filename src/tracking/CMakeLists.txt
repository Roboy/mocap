CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(tracking_node C CXX)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# RelWithDebInfo, Release or Debug
SET(CMAKE_BUILD_TYPE Debug FORCE)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++11 -DMAGICKCORE_QUANTUM_DEPTH=16 -DMAGICKCORE_HDRI_ENABLE=0")

SET( CMAKE_EXPORT_COMPILE_COMMANDS 1 )

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

execute_process(COMMAND grep -o BCM2709 /proc/cpuinfo OUTPUT_VARIABLE IAMRASPBERRYPI3)
if (IAMRASPBERRYPI3)
    message(STATUS "tracking_node is supposed to run on host not raspi, aborting generation")
else ()
    #==============================================================================
    # Dependencies
    #==============================================================================
    # ros
    find_package(catkin REQUIRED COMPONENTS roscpp controller_manager std_msgs sensor_msgs cv_bridge image_transport)
    catkin_package(CATKIN_DEPENDS message_runtime)

    # Eigen
    find_package(PkgConfig)
    pkg_check_modules(EIGEN3 REQUIRED eigen3)
    include_directories(${EIGEN3_INCLUDE_DIRS})

    # SFML
    pkg_check_modules(SFML REQUIRED sfml-graphics)

    # image magick
    find_package(ImageMagick COMPONENTS Magick++ MagickWand MagickCore REQUIRED)
    include_directories(${ImageMagick_Magick++_INCLUDE_DIR})

    # assimp
    pkg_check_modules(assimp REQUIRED assimp)
    include_directories(${assimp_INCLUDE_DIRS})

    # glew
    pkg_check_modules(glew REQUIRED glew)
    include_directories(${glew_INCLUDE_DIRS})

    # opengl
    find_package(OpenGL REQUIRED)

    # OpenCV
    find_package(OpenCV COMPONENTS opencv_core opencv_highgui opencv_imgproc opencv_calib3d REQUIRED)

    # sdformat
    find_package(SDFormat 4 REQUIRED)
    include_directories(${SDFormat_INCLUDE_DIRS})

    # pcl
    find_package(PCL REQUIRED)
    include_directories(${PCL_INCLUDE_DIRS})
    add_definitions(${PCL_DEFINITIONS})

    # boost
    find_package(Boost COMPONENTS filesystem REQUIRED)

    # ncurse
    pkg_check_modules(ncurses REQUIRED ncurses)
    include_directories(${ncurses_INCLUDE_DIRS})

    include_directories(
            ${catkin_INCLUDE_DIRS}
            ${CMAKE_CURRENT_SOURCE_DIR}/omgl/include
            include/
    )
    #==============================================================================
    # Application
    #==============================================================================

    ADD_EXECUTABLE(tracking_node src/markertracker.cpp src/main.cpp omgl/src/mesh.cpp omgl/src/model.cpp omgl/src/renderer.cpp)
    add_dependencies(tracking_node communication_gencpp)
    TARGET_LINK_LIBRARIES(tracking_node ${SFML_LIBRARIES} ${glew_LIBRARIES} ${assimp_LIBRARIES} ${OPENGL_LIBRARIES}
            ${ImageMagick_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${SDFormat_LIBRARIES} ${BOOST_LIBRARIES} -lpthread
            ${catkin_LIBRARIES} ${ncurses_LIBRARIES} )

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
