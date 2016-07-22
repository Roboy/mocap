# - Find the raspi includes and libraries
# This module defines
#  picam_INCLUDE_DIRS
#  picam_LIBRARIES, the libraries required to use picam.
#  picam_FOUND


FIND_LIBRARY(picam_LIBRARIES mmal_core mmal_util mmal_vc_client vcos bcm_host
        /opt/vc/lib
        NO_DEFAULT_PATH)

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
    MESSAGE(STATUS "picam library: ${picam_LIBRARIES}" )
ELSE(picam_FOUND)
    IF(picam_FIND_REQUIRED)
        MESSAGE(FATAL_ERROR "Could not find picam,
-- please make sure userland is installed on your system")
    ENDIF(picam_FIND_REQUIRED)
ENDIF(picam_FOUND)