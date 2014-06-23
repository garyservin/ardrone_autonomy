# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(ardrone_autonomy)

# Load catkin and all dependencies required for this package
find_package(
  catkin
  REQUIRED COMPONENTS
    roscpp
    image_transport
    sensor_msgs
    std_srvs
    tf
    camera_info_manager
    message_generation
    std_msgs
    geometry_msgs
)

add_service_files(
	DIRECTORY srv
	FILES
	CamSelect.srv
	FlightAnim.srv
	LedAnim.srv
	RecordEnable.srv
)

add_message_files(
	DIRECTORY msg
	FILES
	matrix33.msg
	navdata_adc_data_frame.msg
	navdata_altitude.msg
	navdata_demo.msg
	navdata_euler_angles.msg
	navdata_games.msg
	navdata_gyros_offsets.msg
	navdata_hdvideo_stream.msg
	navdata_kalman_pressure.msg
	navdata_magneto.msg
	Navdata.msg
	navdata_phys_measures.msg
	navdata_pressure_raw.msg
	navdata_pwm.msg
	navdata_raw_measures.msg
	navdata_rc_references.msg
	navdata_references.msg
	navdata_time.msg
	navdata_trackers_send.msg
	navdata_trims.msg
	navdata_video_stream.msg
	navdata_vision_detect.msg
	navdata_vision.msg
	navdata_vision_of.msg
	navdata_vision_perf.msg
	navdata_vision_raw.msg
	navdata_watchdog.msg
	navdata_wifi.msg
	navdata_wind_speed.msg
	navdata_zimmu_3000.msg
	vector21.msg
	vector31.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    ardrone_driver
  CATKIN_DEPENDS
    message_runtime
    roscpp
    image_transport
    sensor_msgs
    std_srvs
    tf
    camera_info_manager
    std_msgs
    geometry_msgs
  DEPENDS
    libsdl-dev
)

include(ExternalProject)
externalproject_add(ardronelib
	URL ${PROJECT_SOURCE_DIR}/ARDroneSDK/ardrone-sdk-stripped-2.0.1.tgz
	PREFIX ${CMAKE_BINARY_DIR}/ardrone
	PATCH_COMMAND patch -p1 < ${PROJECT_SOURCE_DIR}/ardrone_lib.patch
	CONFIGURE_COMMAND echo "No configure"
	BUILD_COMMAND make
	INSTALL_COMMAND make install INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}/lib/ardrone
	BUILD_IN_SOURCE 1
)

# find sdk directories for headers and libraries
set(SDK ${CMAKE_BINARY_DIR}/ardrone/src/ardronelib/ARDroneLib/)
include_directories(${SDK} ${SDK}/FFMPEG/Includes ${SDK}/Soft/Common ${SDK}/Soft/Lib ${SDK}/VP_SDK ${SDK}/VP_SDK/VP_Os/linux )
link_directories(${CATKIN_DEVEL_PREFIX}/lib/ardrone)

# add CATKIN_INCLUDE_DIRS
include_directories(${catkin_INCLUDE_DIRS})

# add Boost_INCLUDE_DIRS
include_directories(${Boost_INCLUDE_DIRS})

# add headers directory
include_directories(include)

# add library
add_library(ardrone_driver
    src/ardrone_driver.cpp
    src/video.cpp
    src/ardrone_sdk.cpp
    src/teleop_twist.cpp
)

target_link_libraries(ardrone_driver
  pc_ardrone
  avcodec
  swscale
  avutil
  vlib
  sdk
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

#If you have a package which builds messages and/or services as well as
#executables that use them, you need to create an explicit dependency on
#the automatically-generated message target so that they are built in
#the correct order
add_dependencies(ardrone_driver ardronelib ${PROJECT_NAME}_gencpp)

# build executable
add_executable(ardrone_driver_node src/ardrone_driver_node.cpp)
target_link_libraries(ardrone_driver_node ardrone_driver)

#install Rules
install(TARGETS ardrone_driver ardrone_driver_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
