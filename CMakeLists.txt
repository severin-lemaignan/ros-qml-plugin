cmake_minimum_required(VERSION 3.8)
project(ros_qml_plugin)
set(CMAKE_CXX_STANDARD 14)

option(GLOBAL_INSTALL "Install the plugin globally instead of as a ROS 2 overlay." OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Qt5Core REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(Qt5Qml REQUIRED)
find_package(Qt5Multimedia REQUIRED)

# note: it is *necessary* to include the .hpp in the list of source,
# otherwise the moc will not be generated and the plugin will not be found
set(SOURCES
    include/ros_qml_plugin/qobject_ros2.hpp
    include/ros_qml_plugin/qmlobjects.hpp
    include/ros_qml_plugin/ros2.hpp
    src/rosplugin.cpp
    src/image_provider.cpp
    src/ros2.cpp
    src/qobject_ros2.cpp
    src/qmlobjects.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  image_transport
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  Qt5Core
  Qt5Quick
  Qt5Qml
  Qt5Multimedia
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

# based on https://github.com/StefanFabian/qml_ros2_plugin/blob/b1f7b9368743d8c146145d0398a5fe3bb34aa870/CMakeLists.txt#L146

if(${GLOBAL_INSTALL})
  message(STATUS "Installing plugin globally.")
  # Install Qml plugin as found here
  # https://github.com/4rtzel/cmake-qml-plugin-example/issues/1
  set(URI Ros)
  string(REPLACE "." "/" TARGETPATH ${URI})
  execute_process(COMMAND qmake -qt5 -query QT_INSTALL_QML
    OUTPUT_VARIABLE QT_INSTALL_QML_RAW)
  string(REPLACE "\n" "" QT_INSTALL_QML ${QT_INSTALL_QML_RAW})
  if("${QT_INSTALL_QML}" STREQUAL "**Unknown**")
    message(FATAL_ERROR "Could not find qml plugin dir. Is qml installed?")
  endif()
  message(STATUS "Plugin will be installed to ${QT_INSTALL_QML}")
  set(DESTDIR "${QT_INSTALL_QML}/${TARGETPATH}")
  install(TARGETS ${PROJECT_NAME} DESTINATION ${DESTDIR})
  install(FILES ${CMAKE_CURRENT_LIST_DIR}/qmldir DESTINATION ${DESTDIR})
else()
  message(STATUS "Installing as part of a ROS2 workspace.")
  # Register plugin to be found by QML
  ament_environment_hooks(rosqmlplugin.dsv.in)
  install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME} DESTINATION lib/Ros)
  install(FILES qmldir DESTINATION lib/Ros)
  # The export is only necessary if part of a ROS2 workspace
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
endif()

ament_package()
