cmake_minimum_required(VERSION 3.10)
project(carla_vehicle_launch)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

# Install Python executables
install(PROGRAMS
  src/carla_vehicle_interface.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
install(DIRECTORY
  src/
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)

# Add tests
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_vehicle_interface test/test_vehicle_interface.py)
endif()
