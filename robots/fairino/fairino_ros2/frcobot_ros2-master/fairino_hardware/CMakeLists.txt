cmake_minimum_required(VERSION 3.8)
project(fairino_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC -g -ggdb3)
endif()

#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(fairino_msgs REQUIRED)
find_package(hardware_interface QUIET)
find_package(pluginlib QUIET)
find_package(rclcpp_lifecycle QUIET)


#set variable
set(SRC_DIR EXP_SRC_DIR)
file(GLOB SRC_DIR ./src/command_server.cpp ./src/command_server_node.cpp)
file(GLOB EXP_SRC_DIR ./examples/src/*.cpp)

#compile
file(CREATE_LINK 
"${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so.2.2.3"
"${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so.2"
SYMBOLIC)

file(CREATE_LINK 
"${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so.2"
"${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so"
SYMBOLIC)

add_executable(ros2_cmd_server ${SRC_DIR})
add_executable(srv_test_client ${EXP_SRC_DIR})

#target_compile_definitions(ros2_cmd_server PRIVATE CHN_VERSION)
#target_compile_definitions(ros2_cmd_server PRIVATE ENG_VERSION)


#add headfile source
target_include_directories(ros2_cmd_server PUBLIC 
${CMAKE_CURRENT_SOURCE_DIR}/include/ 
${CMAKE_CURRENT_SOURCE_DIR}/)

target_include_directories(srv_test_client PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/examples/include)
target_link_libraries(ros2_cmd_server ${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so)

#add dependencies
ament_target_dependencies(ros2_cmd_server rclcpp fairino_msgs)
ament_target_dependencies(srv_test_client rclcpp fairino_msgs)

if(hardware_interface_FOUND AND pluginlib_FOUND AND rclcpp_lifecycle_FOUND)
  add_library(
    fairino_hardware
    SHARED
    ./src/fairino_hardware_interface.cpp
  )

  target_include_directories(fairino_hardware PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
    $<INSTALL_INTERFACE:include/fairino_hardware>
  )
  target_link_libraries(fairino_hardware ${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so)
  ament_target_dependencies(fairino_hardware
    hardware_interface
    pluginlib
    rclcpp
    fairino_msgs
    rclcpp_lifecycle
  )

  # export hardware plugins when ros2_control dependencies are available
  pluginlib_export_plugin_description_file(hardware_interface fairino_hardware.xml)
else()
  message(WARNING "ros2_control dependencies not found; building ros2_cmd_server only")
endif()

#install all files
#command server
install(TARGETS ros2_cmd_server
  DESTINATION lib/${PROJECT_NAME}
)

#test example
install(TARGETS srv_test_client
  DESTINATION lib/${PROJECT_NAME}
)

#ros2_control hardware
if(TARGET fairino_hardware)
  install(TARGETS fairino_hardware
    EXPORT fairino_hardware
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
  )
endif()

#copy fairino SDK lib file to specfic path
file(MAKE_DIRECTORY 
"${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}"
)

file(COPY_FILE "libfairino/lib/libfairino.so.2.2.3" 
"${CMAKE_INSTALL_PREFIX}/lib/libfairino.so.2.2.3" 
)

file(CREATE_LINK 
"${CMAKE_INSTALL_PREFIX}/lib/libfairino.so.2.2.3" 
"${CMAKE_INSTALL_PREFIX}/lib/libfairino.so.2" 
SYMBOLIC)

file(CREATE_LINK 
"${CMAKE_INSTALL_PREFIX}/lib/libfairino.so.2" 
"${CMAKE_INSTALL_PREFIX}/lib/libfairino.so" 
SYMBOLIC)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(hardware_interface REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#export
if(TARGET fairino_hardware)
  ament_export_targets(fairino_hardware HAS_LIBRARY_TARGET)
  ament_export_dependencies(ament_cmake rclcpp hardware_interface pluginlib rclcpp_lifecycle)
else()
  ament_export_dependencies(ament_cmake rclcpp)
endif()

ament_package()
