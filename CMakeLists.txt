cmake_minimum_required(VERSION 3.0.2)
project(udp_communication)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-std=c++14)
add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  geometry_msgs
  sensor_msgs
  nav_msgs
)

set(ALL_TARGET_LIBRARIES "")

include(cmake/geographic.cmake)
include(cmake/PCL.cmake)
include(cmake/YAML.cmake)
include(cmake/glog.cmake)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)
include(cmake/global_defination.cmake)

catkin_package()

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/app/*_node.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})


add_executable(message_send_node src/app/message_send_node.cpp ${ALL_SRCS})
add_dependencies(message_send_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(message_send_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})


add_executable(message_server_node src/app/message_server_node.cpp ${ALL_SRCS})
add_dependencies(message_server_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(message_server_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(sbg_navigation_node src/app/sbg_navigation_node.cpp ${ALL_SRCS})
add_dependencies(sbg_navigation_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(sbg_navigation_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

install(TARGETS 

    message_send_node
    message_server_node
    sbg_navigation_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY 
        include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

## Mark other directories for installation:
install(DIRECTORY
        launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)
install(DIRECTORY
        config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    FILES_MATCHING PATTERN "*.yaml"
)

