cmake_minimum_required(VERSION 3.5.0)

project(soss-mqtt-test)

find_package(soss-mqtt REQUIRED)
find_package(soss-mock REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_executable(soss-mqtt-test
  main.cpp
  integration/mqtt__roundtrip.cpp
)

set(thirdparty_dir "${CMAKE_CURRENT_LIST_DIR}/../../thirdparty")

list(APPEND CMAKE_MODULE_PATH "${thirdparty_dir}/catch2/cmake")

include(CTest)
include(Catch)
catch_discover_tests(soss-mqtt-test)

target_link_libraries(soss-mqtt-test
  PRIVATE
    soss::mock
)

target_include_directories(soss-mqtt-test
  PRIVATE
    "${thirdparty_dir}/catch2/include"
)

target_compile_definitions(soss-mqtt-test
  PRIVATE
    "MQTT__RESOURCES=\"${CMAKE_CURRENT_LIST_DIR}/resources/mqtt__\""
)

# Windows dll dependencies installation
if(WIN32)
  find_file(JSONDLL NAMES "soss-json.dll" PATHS "${soss-json_DIR}" PATH_SUFFIXES "lib" )
  install (FILES ${JSONDLL} DESTINATION "${CMAKE_INSTALL_PREFIX}/../bin")
endif()
