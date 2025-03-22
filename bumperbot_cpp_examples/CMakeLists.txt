cmake_minimum_required(VERSION 3.8)
project(bumperbot_cpp_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

# bumperbot custom srv, server and client
find_package(bumperbot_msgs REQUIRED)

# simple_tf_kinematics
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(bumperbot_msgs)

find_package(tf2)

# For Simple Serial Transmitter
find_package(PkgConfig REQUIRED)

pkg_check_modules(SERIAL libserial)

# For lifecycle
find_package(rclcpp_lifecycle REQUIRED)



include_directories(include)

# Publisher, Subscriber, Service Client, Service Server
add_executable(simple_publisher src/simple_publisher.cpp)
ament_target_dependencies(simple_publisher rclcpp std_msgs)

add_executable(simple_subscriber src/simple_subscriber.cpp)
ament_target_dependencies(simple_subscriber rclcpp std_msgs)

add_executable(simple_service_server src/simple_service_server.cpp)
ament_target_dependencies(simple_service_server rclcpp bumperbot_msgs)

add_executable(simple_service_client src/simple_service_client.cpp)
ament_target_dependencies(simple_service_client rclcpp bumperbot_msgs)


add_executable(simple_parameter src/simple_parameter.cpp)
ament_target_dependencies(simple_parameter rclcpp rcl_interfaces)

add_executable(simple_tf_kinematics src/simple_tf_kinematics.cpp)
ament_target_dependencies(simple_tf_kinematics rclcpp tf2_ros geometry_msgs)

add_executable(simple_tf_kinematics_dynamic src/simple_tf_kinematics_dynamic.cpp)
ament_target_dependencies(simple_tf_kinematics_dynamic rclcpp tf2_ros geometry_msgs)

# For testing
# Add include directories
# include_directories(include ${bumperbot_msgs_INCLUDE_DIRS})
add_executable(simple_tf_listener src/simple_tf_listener.cpp)
ament_target_dependencies(simple_tf_listener rclcpp tf2_ros geometry_msgs bumperbot_msgs)

add_executable(simple_tf_euler_to_quaternion src/simple_tf_euler_to_quaternion.cpp)
ament_target_dependencies(simple_tf_euler_to_quaternion rclcpp tf2_ros geometry_msgs bumperbot_msgs tf2)

# For Simple Transmitter/Receiver
add_executable(simple_transmitter src/simple_transmitter.cpp)
ament_target_dependencies(simple_transmitter rclcpp std_msgs)
target_include_directories(simple_transmitter PUBLIC "${SERIAL_INCLUDE_DIRS}")
target_link_libraries(simple_transmitter "${SERIAL_LDFLAGS}")

add_executable(simple_receiver src/simple_receiver.cpp)
ament_target_dependencies(simple_receiver rclcpp std_msgs)
target_include_directories(simple_receiver PUBLIC "${SERIAL_INCLUDE_DIRS}")
target_link_libraries(simple_receiver "${SERIAL_LDFLAGS}")

add_executable(simple_lifecycle_node src/simple_lifecycle_node.cpp)
ament_target_dependencies(simple_lifecycle_node rclcpp rclcpp_lifecycle std_msgs)


install(
  DIRECTORY include
  DESTINATION include
)
install(TARGETS
  simple_publisher
  simple_subscriber
  simple_service_server
  simple_service_client

  simple_parameter
  simple_tf_kinematics
  simple_tf_kinematics_dynamic
  simple_tf_listener
  simple_tf_euler_to_quaternion
  simple_lifecycle_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  simple_transmitter
  simple_receiver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
