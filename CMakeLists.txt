cmake_minimum_required(VERSION 3.30)
project( pan_tilt )

if (POLICY CMP0015)
  cmake_policy(SET CMP0015 NEW)
endif (POLICY CMP0015)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.47 REQUIRED system filesystem thread)
# add_definitions( "-Wno-unused-local-typedefs" )

include_directories( ${Boost_INCLUDE_DIRS} )

set( BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../coral )
set( INCLUDE_ROOT "${BASE_DIR}/includes" )
set( CORE_INCLUDE ${INCLUDE_ROOT}/core )
set( NETAPP_INCLUDE ${INCLUDE_ROOT}/netapp )
include_directories( "${CORE_INCLUDE}" )
include_directories( "${CORE_INCLUDE}/crypto" )
include_directories( "${CORE_INCLUDE}/file_system" )
include_directories( "${CORE_INCLUDE}/cli" )
include_directories( "${CORE_INCLUDE}/helpers" )
include_directories( "${CORE_INCLUDE}/container" )
include_directories( "${CORE_INCLUDE}/concurrency" )
include_directories( "${CORE_INCLUDE}/time" )
include_directories( "${CORE_INCLUDE}/logging" )

link_directories( ${BASE_DIR}/lib/core )

add_library( scanner_common STATIC 
   i2c_interface.cpp
   pwm_controller.cpp
   pan_tilt_controller.cpp
   adxl345_controller.cpp
   gimbal_controller_thread.cpp )

add_executable(
   pan_tilt_util
   pan_tilt_util.cpp )

target_link_libraries( pan_tilt_util scanner_common )
target_link_libraries( pan_tilt_util coral_core_shared )
target_link_libraries( coral_core_shared ${Boost_LIBRARIES} )
