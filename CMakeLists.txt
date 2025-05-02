# Minimum required CMake version
cmake_minimum_required(VERSION 3.8)

# Project name and version
project(manipulator_gui)

# Compiler flags for GCC and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(manipulator REQUIRED)
find_package(manipulator_control_strategies REQUIRED)
find_package(manipulator_skill_acquisition REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Script installation
# Create scripts directory
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/scripts)

# Create launcher script
set(GUI_SCRIPT_CONTENT "#!/bin/bash
python3 -m manipulator_gui.GUI
")

# Write and make executable
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/scripts/GUI ${GUI_SCRIPT_CONTENT})
file(CHMOD ${CMAKE_CURRENT_BINARY_DIR}/scripts/GUI
     PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE 
                 GROUP_READ GROUP_EXECUTE 
                 WORLD_READ WORLD_EXECUTE)

# Install launcher script
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/scripts/GUI
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE 
              GROUP_READ GROUP_EXECUTE 
              WORLD_READ WORLD_EXECUTE
)

# Install resource files
install(
  DIRECTORY
    resources
  DESTINATION share/${PROJECT_NAME}
)

# Testing configuration
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Finalize package
ament_package() 