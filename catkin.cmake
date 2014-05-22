cmake_minimum_required(VERSION 2.8.3)
project(or_sbpl)

# TODO: We might be missing dependencies here.
find_package(catkin REQUIRED)
catkin_package(
    INCLUDE_DIRS include/
    LIBRARIES ${PROJECT_NAME}
)

include(FindPkgConfig)
pkg_check_modules(OPENRAVE REQUIRED openrave0.9)
pkg_check_modules(SBPL REQUIRED sbpl)
pkg_check_modules(Yaml REQUIRED yaml-cpp)

MESSAGE(STATUS "openrave0.0: " ${OPENRAVE_INCLUDE_DIRS})

include_directories(
  include
  ${OPENRAVE_INCLUDE_DIRS}
  ${SBPL_INCLUDE_DIRS}
  ${Yaml_INCLUDE_DIRS})

link_directories(${OPENRAVE_LIBRARY_DIRS} ${SBPL_LIBRARY_DIRS} ${Yaml_LIBRARY_DIRS})

add_library(${PROJECT_NAME} 
  src/SBPLBasePlanner.cpp 
  src/SBPLBasePlannerEnvironment.cpp 
  src/SBPLBasePlannerTypes.cpp
  src/CachedAction.cpp
  src/TwistAction.cpp)
target_link_libraries(${PROJECT_NAME}
  ${SBPL_LIBRARIES} ${OPENRAVE_LIBRARIES} ${Yaml_LIBRARIES})

add_library(${PROJECT_NAME}_plugin src/SBPLMain.cpp)
target_link_libraries(${PROJECT_NAME}_plugin
  ${SBPL_LIBRARIES} ${OPENRAVE_LIBRARIES})
set_target_properties(${PROJECT_NAME}_plugin PROPERTIES PREFIX "")

install(TARGETS or_sbpl
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(TARGETS or_sbpl_plugin
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/openrave0.9)
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN ".svn" EXCLUDE)
