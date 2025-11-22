cmake_minimum_required(VERSION 3.22)
project(ikarus_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/joint_state_to_array.py
    scripts/openteach_to_twist.py
    scripts/test_twist.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
