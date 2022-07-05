cmake_minimum_required(VERSION 3.5.0)

project(soss-mqtt)

if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()

find_package(soss-core REQUIRED)
find_package(soss-json REQUIRED)

find_package(Boost REQUIRED)

set (THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

set(PAHO_INSTALL_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/paho)
file(MAKE_DIRECTORY ${PAHO_INSTALL_PREFIX}/include)

find_package(OpenSSL QUIET)

include(ExternalProject)
ExternalProject_Add(
    pahomqttc
    URL https://github.com/eclipse/paho.mqtt.c/archive/v1.3.8.tar.gz
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${PAHO_INSTALL_PREFIX} -DPAHO_WITH_SSL=${OPENSSL_FOUND} -DPAHO_BUILD_STATIC=ON -DPAHO_BUILD_SHARED=OFF -DPAHO_ENABLE_TESTING=OFF
)

add_library(paho-mqtt3 STATIC IMPORTED)
add_dependencies(paho-mqtt3 pahomqttc)
if(OPENSSL_FOUND)
  set_target_properties(paho-mqtt3 PROPERTIES IMPORTED_LOCATION ${PAHO_INSTALL_PREFIX}/lib/libpaho-mqtt3as.a
                                              INTERFACE_INCLUDE_DIRECTORIES ${PAHO_INSTALL_PREFIX}/include)
else()
  set_target_properties(paho-mqtt3 PROPERTIES IMPORTED_LOCATION ${PAHO_INSTALL_PREFIX}/lib/libpaho-mqtt3a.a
                                              INTERFACE_INCLUDE_DIRECTORIES ${PAHO_INSTALL_PREFIX}/include)
endif()

ExternalProject_Add(
    pahomqttcpp
    DEPENDS pahomqttc
    URL https://github.com/eclipse/paho.mqtt.cpp/archive/v1.2.0.tar.gz
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${PAHO_INSTALL_PREFIX} -DPAHO_WITH_SSL=${OPENSSL_FOUND} -DPAHO_BUILD_STATIC=ON -DPAHO_BUILD_SHARED=OFF
)

add_library(paho-mqttpp3 STATIC IMPORTED)
add_dependencies(paho-mqttpp3 pahomqttcpp)
set_target_properties(paho-mqttpp3 PROPERTIES IMPORTED_LOCATION ${PAHO_INSTALL_PREFIX}/lib/libpaho-mqttpp3.a
                                              INTERFACE_INCLUDE_DIRECTORIES ${PAHO_INSTALL_PREFIX}/include)
if(OPENSSL_FOUND)
  set_target_properties(paho-mqttpp3 PROPERTIES INTERFACE_COMPILE_DEFINITIONS "OPENSSL=1;PAHO_MQTT_STATIC=1"
                                                INTERFACE_LINK_LIBRARIES "paho-mqtt3;Threads::Threads;OpenSSL::SSL;OpenSSL::Crypto")
else()
  set_target_properties(paho-mqttpp3 PROPERTIES INTERFACE_COMPILE_DEFINITIONS "PAHO_MQTT_STATIC=1"
                                                INTERFACE_LINK_LIBRARIES "paho-mqtt3;Threads::Threads")
endif()

add_library(soss-mqtt SHARED
  src/SystemHandle.cpp
)

target_link_libraries(soss-mqtt
  PUBLIC
    Boost::boost
    soss::core
    paho-mqttpp3
    soss::json
)

target_include_directories(soss-mqtt
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
)

###############################
# Install soss-mqtt
soss_install_middleware_plugin(
  MIDDLEWARE mqtt
  TARGET soss-mqtt
)
