cmake_minimum_required(VERSION 3.22)
project(dummy_moveit_config)

find_package(ament_cmake REQUIRED)

# The upstream workspace contains Humble-era Servo demo executables.  The
# perception service only needs the robot model and MoveIt configuration, so
# the isolated Jazzy overlay installs resources without compiling those demos.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
