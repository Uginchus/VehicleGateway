cmake_minimum_required(VERSION 3.5)
project(qgroundcontrol)

find_package(ament_cmake REQUIRED)

include(ExternalProject)

if(NOT DEFINED SKIP_QGROUNDCONTROL OR NOT ${SKIP_QGROUNDCONTROL})
  if(NOT EXISTS "${CMAKE_BINARY_DIR}/QGroundControl.AppImage")
    file(DOWNLOAD
      https://github.com/mavlink/qgroundcontrol/releases/download/v4.4.3/QGroundControl.AppImage
      QGroundControl.AppImage
    )
  endif()

  install(PROGRAMS
    ${CMAKE_BINARY_DIR}/QGroundControl.AppImage
    DESTINATION bin
  )
else()
  message(WARNING "Skipping QGroundControl")
endif()

ament_package()
