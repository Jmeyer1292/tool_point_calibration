cmake_minimum_required(VERSION 2.8.3)
project(tool_point_calibration)

find_package(catkin REQUIRED COMPONENTS
  tf
  tf_conversions
)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS tf tf_conversions
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
add_definitions(-std=c++11)

# The core tool calibration library
add_library(${PROJECT_NAME}
  src/tool_point_calibration.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)

# A utility for calibrating a named tool point on the command line
add_executable(console_calibration
  src/console_calibration.cpp
)

add_dependencies(console_calibration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(console_calibration
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  tool_point_calibration
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS tool_point_calibration tool_point_calibration_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test
  test/utest.cpp
  test/test_tool.cpp
)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()

# Add folders to be run by python nosetests
# catkin_add_nosetests(test)
