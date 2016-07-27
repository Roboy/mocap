# - Find the raspi includes and libraries
# This module defines
#  picam_INCLUDE_DIRS
#  picam_LIBRARIES, the libraries required to use picam.
#  picam_FOUND


FIND_LIBRARY(mmal_core_LIB mmal_core
        /opt/vc/lib
        NO_DEFAULT_PATH)

FIND_LIBRARY(mmal_util_LIB mmal_util
        /opt/vc/lib
        NO_DEFAULT_PATH)

FIND_LIBRARY(mmal_vc_client_LIB mmal_vc_client
        /opt/vc/lib
        NO_DEFAULT_PATH)

FIND_LIBRARY(vcos_LIB vcos
        /opt/vc/lib
        NO_DEFAULT_PATH)

FIND_LIBRARY(bcm_host_LIB bcm_host
        /opt/vc/lib
        NO_DEFAULT_PATH)

SET(picam_LIBRARIES 
	${mmal_core_LIB} ${mmal_util_LIB} ${mmal_vc_client_LIB} ${vcos_LIB} ${bcm_host_LIB}
   )

SET(picam_INCLUDE_DIRS
        /opt/vc/interface/vcos
        /opt/vc/include
        /opt/vc/interface/vcos/pthreads
        /opt/vc/interface/vmcs_host/linux
        /opt/vc/host_applications/linux/libs/bcm_host/include)

MARK_AS_ADVANCED(
        picam_INCLUDE_DIRS
        picam_LIBRARIES)

SET( picam_FOUND "NO" )
IF(picam_LIBRARIES)
    SET( picam_FOUND "YES" )
ENDIF(picam_LIBRARIES)

IF(picam_FOUND)
    MESSAGE(STATUS "Found picam library")
    MESSAGE(STATUS "picam include dir: ${picam_INCLUDE_DIRS}" )
    MESSAGE(STATUS "picam libraries: ${picam_LIBRARIES}" )
ELSE(picam_FOUND)
    IF(picam_FIND_REQUIRED)
        MESSAGE(FATAL_ERROR "Could not find picam,
-- please make sure userland https://github.com/raspberrypi/userland is installed on your system")
    ENDIF(picam_FIND_REQUIRED)
ENDIF(picam_FOUND)
