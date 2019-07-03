# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(lwr_simulation)

find_package(catkin REQUIRED COMPONENTS kuka_lwr_fri kdl_parser geometry_msgs std_srvs message_generation)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)
find_package(orocos_kdl REQUIRED)

# http://stackoverflow.com/questions/7172670/best-shortest-way-to-join-a-list-in-cmake
function(JOIN VALUES GLUE OUTPUT)
  string (REGEX REPLACE "([^\\]|^);" "\\1${GLUE}" _TMP_STR "${VALUES}")
  string (REGEX REPLACE "[\\](.)" "\\1" _TMP_STR "${_TMP_STR}") #fixes escaping
  set (${OUTPUT} "${_TMP_STR}" PARENT_SCOPE)
endfunction()
JOIN("${GAZEBO_LDFLAGS}" " " LD_FLAGS)

add_service_files(FILES setPayload.srv)
generate_messages(DEPENDENCIES geometry_msgs)

message(STATUS "lwr_simulation: using LD_FLAGS = \"${LD_FLAGS}\"")

catkin_package(
  CATKIN_DEPENDS
    gazebo_ros geometry_msgs message_runtime
  LIBRARIES
    lwr_controller
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

link_directories(${orocos_kdl_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(lwr_controller src/lwr_controller.cpp)
add_dependencies(lwr_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_target_properties(lwr_controller PROPERTIES LINK_FLAGS "${LD_FLAGS}")
target_link_libraries(lwr_controller
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${kdl_parser_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)



# Install targets
install(TARGETS
  lwr_controller
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
  )
