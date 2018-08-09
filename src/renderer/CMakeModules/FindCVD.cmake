# - Try to find libCVD
#
#  CVD_FOUND - system has libCVD
#  CVD_INCLUDE_DIR - the libCVD include directories
#  CVD_LIBRARY - link these to use libCVD

FIND_PATH(
  CVD_INCLUDE_DIR
  NAMES cvd/image.h
  PATHS
    /usr/include
    /usr/local/include
    ${CMAKE_SOURCE_DIR}/../libcvd
    ${CMAKE_SOURCE_DIR}/../cvd
)

FIND_LIBRARY(
  CVD_LIBRARY
  NAMES cvd
  PATHS
    /usr/lib
    /usr/local/lib
    ${CMAKE_SOURCE_DIR}/../libcvd
    ${CMAKE_SOURCE_DIR}/../cvd
) 

IF(CVD_INCLUDE_DIR AND CVD_LIBRARY)
  SET(CVD_FOUND TRUE)
ENDIF()

IF(CVD_FOUND)
   IF(NOT CVD_FIND_QUIETLY)
      MESSAGE(STATUS "Found CVD: ${CVD_LIBRARY}")
   ENDIF()
ELSE()
   IF(CVD_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find CVD")
   ENDIF()
ENDIF()
