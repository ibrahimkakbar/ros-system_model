cmake_minimum_required(VERSION 3.0.2)
project(system_model)

# Find catkin and catkin components.
find_package(catkin REQUIRED COMPONENTS
  roscpp
  kalman_filter
  system_model_msgs)

# Configure catkin package.
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES system_model
  CATKIN_DEPENDS system_model_msgs kalman_filter roscpp)

# Specify include directories.
include_directories(
  include
  ${catkin_INCLUDE_DIRS})

# Build the system_model plugin library.
add_library(${PROJECT_NAME} src/${PROJECT_NAME}/system_model.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

# Build the system_model node.
add_executable(${PROJECT_NAME}_node src/system_model/main.cpp)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES})

# Install system_model plugin library and headers.
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

# Install system_model node.
install(TARGETS ${PROJECT_NAME}_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})