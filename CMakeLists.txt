cmake_minimum_required(VERSION 2.8.3)
project(rgbd2vr)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
        roscpp 
        tf 
        cv_bridge 
        image_transport 
        image_geometry 
        nodelet)

## System dependencies
find_package(OpenGL REQUIRED)
find_package(OpenCV 3 REQUIRED core imgproc)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
find_package(GLEW 1.11 REQUIRED)
find_package(SDL2 REQUIRED)


##Hardcoded OpenVR Path please change
set(OPENVR_PREFIX $ENV{DEV_PREFIX})

## OpenVR API path
find_library(OPENVR_LIBRARIES
  NAMES
    openvr_api
  PATHS
    ${OPENVR_PREFIX}/lib
  PATH_SUFFIXES
    osx32
    linux64
    ${WINDOWS_PATH_SUFFIXES}
  NO_DEFAULT_PATH
  NO_CMAKE_FIND_ROOT_PATH
)
set(OPENVR_INCLUDE_DIRS ${OPENVR_PREFIX}/include)




###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_markers
  CATKIN_DEPENDS 
    tf
    nodelet
  DEPENDS OpenCV
  LIBRARIES 
)


###########
## Build ##
###########

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${OPENGL_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${GLEW_INCLUDE_DIRS}
                    ${SDL2_INCLUDE_DIRS}
                    ${OPENVR_INCLUDE_DIRS})

add_definitions(-std=c++11)

set(LIBS 
    ${catkin_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${SDL2_LIBRARIES}
    ${OPENVR_LIBRARIES}
    )


add_executable(run src/main.cpp src/rgbd2vr.cpp src/Matrices.cpp)
target_link_libraries(run ${LIBS})

#add_library(markers_nodelet src/chilitagsdetector.cpp src/nodelet.cpp)
#add_dependencies(markers_nodelet ${catkin_EXPORTED_TARGETS})
#target_link_libraries(markers_nodelet ${LIBS})


#############
## Install ##
#############

install(TARGETS run
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
    launch/run.launch
    launch/run_gscam.launch
#    launch/gscam_nodelet_pipeline.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#install(FILES
#    config/markers_configuration_sample.yml
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
#)

#install(FILES
#    nodelet_markers.xml
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

