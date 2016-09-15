cmake_minimum_required(VERSION 2.8.11)

project(rostorch)

find_package(Torch REQUIRED)
find_package(Boost 1.47.0 REQUIRED COMPONENTS system thread chrono)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package(tf)
find_package(pcl_ros)

set(SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmax-errors=3 -Wall" )

set(src
  "${SOURCE_DIR}/std/string.cpp"
  "${SOURCE_DIR}/std/string_vector.cpp"
  "${SOURCE_DIR}/std/string_map.cpp"
  "${SOURCE_DIR}/std/variable.cpp"
  "${SOURCE_DIR}/std/variable_vector.cpp"
  "${SOURCE_DIR}/std/variable_table.cpp"

  "${SOURCE_DIR}/ros/init.cpp"
  "${SOURCE_DIR}/ros/callback_queue.cpp"
  "${SOURCE_DIR}/ros/node_handle.cpp"
  "${SOURCE_DIR}/ros/master.cpp"
  "${SOURCE_DIR}/ros/this_node.cpp"
  "${SOURCE_DIR}/ros/message_buffer.cpp"
  "${SOURCE_DIR}/ros/subscriber.cpp"
  "${SOURCE_DIR}/ros/publisher.cpp"
  "${SOURCE_DIR}/ros/service_client.cpp"
  "${SOURCE_DIR}/ros/service_server.cpp"
  "${SOURCE_DIR}/ros/async_spinner.cpp"
  "${SOURCE_DIR}/ros/time.cpp"
  "${SOURCE_DIR}/ros/duration.cpp"
  "${SOURCE_DIR}/ros/console.cpp"
)

# compile TF support if available
if (${tf_FOUND})
  set(src ${src}
    "${SOURCE_DIR}/tf/transform.cpp"
    "${SOURCE_DIR}/tf/quaternion.cpp"
    "${SOURCE_DIR}/tf/stamped_transform.cpp"
    "${SOURCE_DIR}/tf/transform_broadcaster.cpp"
    "${SOURCE_DIR}/tf/transform_listener.cpp"
  )
  list(APPEND catkin_INCLUDE_DIRS ${tf_INCLUDE_DIRS})
  list(APPEND catkin_LIBRARIES ${tf_LIBRARIES})
endif()

# compile PCL support if available
if (${pcl_ros_FOUND})
  set(src ${src}
    "${SOURCE_DIR}/ros/point_cloud2.cpp"
  )
  list(APPEND catkin_INCLUDE_DIRS ${pcl_ros_INCLUDE_DIRS})
  list(APPEND catkin_LIBRARIES ${pcl_ros_LIBRARIES})
endif()

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${Torch_INSTALL_INCLUDE}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${Torch_INSTALL_LIB}
  ${Boost_LIBRARY_DIRS}
)

add_library(rostorch MODULE ${src})

target_link_libraries(rostorch TH ${catkin_LIBRARIES} ${Boost_LIBRARIES} )

install(TARGETS rostorch LIBRARY DESTINATION ${Torch_INSTALL_LUA_CPATH_SUBDIR})
install(DIRECTORY "lua/" DESTINATION "${Torch_INSTALL_LUA_PATH_SUBDIR}/ros" FILES_MATCHING PATTERN "*.lua")
