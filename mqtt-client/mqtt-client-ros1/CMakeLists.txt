cmake_minimum_required(VERSION 3.5.0)
project(mqtt-client-ros1)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(soss-ros1 REQUIRED)
find_package(soss-genmsg REQUIRED)

find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)

soss_genmsg_mix(
  PACKAGES
      std_msgs
      std_srvs
      #  geometry_msgs
      #  nav_msgs
      #  sensor_msgs
  MIDDLEWARES ros1
  REQUIRED
)

install(CODE "")
