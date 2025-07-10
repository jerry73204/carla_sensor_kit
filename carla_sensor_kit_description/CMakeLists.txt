cmake_minimum_required(VERSION 3.8)
project(carla_sensor_kit_description)

find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  urdf
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
