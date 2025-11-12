cmake_minimum_required(VERSION 3.8)
project(cpp_turtle_controller)

# Stelle sicher, dass alle Compilerwarnungen angezeigt werden
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Finde die benötigten Abhängigkeiten
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Erzeuge das ausführbare Programm aus der C++-Datei
add_executable(turtle_square src/turtle_square.cpp)

# Verknüpfe die Abhängigkeiten mit dem Executable
ament_target_dependencies(turtle_square
  rclcpp
  geometry_msgs
)

# Installationsregel: legt das Binary im lib-Ordner des Pakets ab
install(TARGETS
  turtle_square
  DESTINATION lib/${PROJECT_NAME}
)

# Ermöglicht Integration in colcon build
ament_package()

