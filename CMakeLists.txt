cmake_minimum_required(VERSION 2.8.3)
project(feature_frontend)

set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fopenmp")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    camera_model
    vignetting_model
    message_filters
    code_utils
    )

find_package(OpenCV REQUIRED)
find_package(Eigen3  REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    )

include_directories("include")

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    )

#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(FRONTEND_LIB_SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/src/feature_lib/frontendBase/detectorfast.cpp
    ${PROJECT_SOURCE_DIR}/src/feature_lib/frontendBase/frontendbase.cpp
    ${PROJECT_SOURCE_DIR}/src/feature_lib/frontendBase/frontendcamera.cpp
    ${PROJECT_SOURCE_DIR}/src/feature_lib/frontendMono/frontendmono.cpp
    ${PROJECT_SOURCE_DIR}/src/feature_lib/frontendStereo/frontendstereo.cpp
    ${PROJECT_SOURCE_DIR}/src/feature_lib/frontendinit.cpp
)

add_executable(maskGenerater    src/maskGenerater.cpp    )
target_link_libraries(maskGenerater dw ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(image_variance     src/image_variance.cpp    )
target_link_libraries(image_variance dw ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(stereoPreprocess    src/stereoPreprocess.cc )
target_link_libraries(stereoPreprocess    ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(dualfisheyefeaturenode
    src/dualfisheye_node.cpp
    src/dualFisheye/dualfisheyetracker.cpp
    ${FRONTEND_LIB_SOURCE_FILES}
    )
target_link_libraries(dualfisheyefeaturenode dw ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(monocam_node
    src/monocam_node.cpp
    src/monotrackerRos/monotrackerRos.cpp
    ${FRONTEND_LIB_SOURCE_FILES}
    )
target_link_libraries(monocam_node dw ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(stereocam_node
    src/stereocam_node.cpp
    src/stereoTrackerRos/stereotrackerRos.cpp
    ${FRONTEND_LIB_SOURCE_FILES}
    )
target_link_libraries(stereocam_node dw ${catkin_LIBRARIES} ${OpenCV_LIBS})

