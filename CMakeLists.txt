cmake_minimum_required (VERSION 2.6) 

project(imageclipper)
SET(PROJECT_VERSION "0.1")


SET(OpenCV_DIR "~/projects/anpr/opencv-2.4.5/")


# Opencv Package
FIND_PACKAGE( OpenCV REQUIRED )
IF (${OpenCV_VERSION} VERSION_LESS 2.3.0)
	MESSAGE(FATAL_ERROR "OpenCV version is not compatible : ${OpenCV_VERSION}")
ENDIF()



SET(SRC
  src/imageclipper.cpp
)

ADD_EXECUTABLE( ${PROJECT_NAME} ${SRC} )
TARGET_LINK_LIBRARIES( ${PROJECT_NAME}  ${OpenCV_LIBS} 
	/usr/lib/libboost_system.so.1.46.1
	/usr/lib/libboost_filesystem.so.1.46.1 
)

