CMAKE_MINIMUM_REQUIRED(VERSION 2.8.11)

PROJECT(torch-ros)

FIND_PACKAGE(Torch REQUIRED)
FIND_PACKAGE(Boost 1.47.0 REQUIRED COMPONENTS system thread chrono)
FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp std_msgs tf)

SET(SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")

SET(src
  "${SOURCE_DIR}/std/string.cpp"
  "${SOURCE_DIR}/std/string_vector.cpp"

  "${SOURCE_DIR}/ros/init.cpp"
  "${SOURCE_DIR}/ros/node_handle.cpp"
  "${SOURCE_DIR}/ros/message_buffer.cpp"
  "${SOURCE_DIR}/ros/subscriber.cpp"
  "${SOURCE_DIR}/ros/publisher.cpp"
  "${SOURCE_DIR}/ros/async_spinner.cpp"
  "${SOURCE_DIR}/ros/time.cpp"
  "${SOURCE_DIR}/ros/duration.cpp"
  "${SOURCE_DIR}/ros/console.cpp"

  "${SOURCE_DIR}/tf/transform.cpp"
  "${SOURCE_DIR}/tf/quaternion.cpp"
  "${SOURCE_DIR}/tf/stamped_transform.cpp"
  "${SOURCE_DIR}/tf/transform_broadcaster.cpp"
  "${SOURCE_DIR}/tf/transform_listener.cpp"
)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${Torch_INSTALL_INCLUDE}
  ${catkin_INCLUDE_DIRS}
)

LINK_DIRECTORIES(
  ${Torch_INSTALL_LIB}
  ${Boost_LIBRARY_DIRS}
)

FILE(GLOB luasrc *.lua)

ADD_LIBRARY(torch-ros MODULE ${src})

target_link_libraries(torch-ros TH ${catkin_LIBRARIES} ${Boost_LIBRARIES} )

INSTALL(TARGETS torch-ros LIBRARY DESTINATION ${Torch_INSTALL_LUA_CPATH_SUBDIR})
#INSTALL(DIRECTORY "doc" DESTINATION "${Torch_INSTALL_LUA_PATH_SUBDIR}/ros")
INSTALL(FILES "README.md" DESTINATION "${Torch_INSTALL_LUA_PATH_SUBDIR}/ros")
INSTALL(FILES ${luasrc} DESTINATION "${Torch_INSTALL_LUA_PATH_SUBDIR}/ros")
