cmake_minimum_required(VERSION 2.8.3)
project(or_sbpl)

find_package(catkin REQUIRED COMPONENTS openrave_catkin)
catkin_package(
    INCLUDE_DIRS include/
    LIBRARIES ${PROJECT_NAME}
)

find_package(OpenRAVE REQUIRED)

include(FindPkgConfig)
pkg_check_modules(SBPL REQUIRED sbpl)
pkg_check_modules(Yaml REQUIRED yaml-cpp)

include_directories(
  include
  ${OpenRAVE_INCLUDE_DIRS}
  ${SBPL_INCLUDE_DIRS}
  ${Yaml_INCLUDE_DIRS})

link_directories(${OpenRAVE_LIBRARY_DIRS} ${SBPL_LIBRARY_DIRS} ${Yaml_LIBRARY_DIRS})

add_library(${PROJECT_NAME} 
  src/SBPLBasePlanner.cpp 
  src/SBPLBasePlannerEnvironment.cpp 
  src/SBPLBasePlannerTypes.cpp
  src/CachedAction.cpp
  src/TwistAction.cpp)
target_link_libraries(${PROJECT_NAME}
    ${SBPL_LIBRARIES} ${OpenRAVE_LIBRARIES} ${Yaml_LIBRARIES})

openrave_plugin(${PROJECT_NAME}_plugin src/SBPLMain.cpp)
target_link_libraries(${PROJECT_NAME}_plugin
    ${PROJECT_NAME} ${SBPL_LIBRARIES} ${OpenRAVE_LIBRARIES})

install(TARGETS or_sbpl
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN ".svn" EXCLUDE)
