cmake_minimum_required(VERSION 3.5.0)

project(mqtt-client)

find_package(soss-core REQUIRED)
find_package(soss-mqtt REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED
  COMPONENTS
    program_options
)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_executable(mqtt-client src/main.cpp)

target_compile_options(mqtt-client
  PRIVATE
    -Werror -Wall -Wextra -Wpedantic
)
target_link_libraries(mqtt-client
  soss::core
  Boost::program_options
  yaml-cpp
)

include(GNUInstallDirs)
install(
  TARGETS mqtt-client
  DESTINATION "${CMAKE_INSTALL_BINDIR}"
)
