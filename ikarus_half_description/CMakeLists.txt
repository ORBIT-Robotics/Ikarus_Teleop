cmake_minimum_required(VERSION 3.10)
project(ikarus_half_description)

find_package(ament_cmake REQUIRED)

# Install directories so resources resolve via package share
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

ament_package()