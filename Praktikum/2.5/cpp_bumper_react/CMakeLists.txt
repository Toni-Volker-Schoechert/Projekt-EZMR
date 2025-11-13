cmake_minimum_required(VERSION 3.8)
project(cpp_bumper_react)

# Stelle sicher, dass alle Compilerwarnungen angezeigt werden
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Finde die benötigten Abhängigkeiten
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(irobot_create_msgs REQUIRED)

# Erzeuge das ausführbare Programm aus der C++-Datei
add_executable(bumper_react src/turtlebot_bumper_react.cpp)

# Verknüpfe die Abhängigkeiten mit dem Executable
ament_target_dependencies(bumper_react
  rclcpp
  std_msgs
  geometry_msgs
  irobot_create_msgs
)

# Installationsregel: legt das Binary im lib-Ordner des Pakets ab
install(TARGETS
  bumper_react
  DESTINATION lib/${PROJECT_NAME}
)

# Ermöglicht Integration in colcon build
ament_package()
