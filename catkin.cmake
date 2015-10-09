cmake_minimum_required(VERSION 2.8.3)
project(or_sbpl)

find_package(catkin REQUIRED COMPONENTS openrave_catkin)
catkin_package(
    INCLUDE_DIRS include/
    LIBRARIES ${PROJECT_NAME}
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenRAVE REQUIRED)

include(FindPkgConfig)
pkg_check_modules(SBPL REQUIRED sbpl)

pkg_check_modules(YamlCpp REQUIRED yaml-cpp)
if (${YamlCpp_VERSION} VERSION_LESS 0.5.0)
    message(STATUS "Using the old-style yaml-cpp (< 0.5.0) API.")
else ()
    add_definitions(-DYAMLCPP_NEWAPI)
    message(STATUS "Using the new-style yaml-cpp (>= 0.5.0) API.")
endif ()

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
  ${catkin_LIBRARIES} ${SBPL_LIBRARIES} ${OpenRAVE_LIBRARIES} ${Yaml_LIBRARIES})

openrave_plugin(${PROJECT_NAME}_plugin src/SBPLMain.cpp)
target_link_libraries(${PROJECT_NAME}_plugin
    ${PROJECT_NAME} yaml-cpp ${SBPL_LIBRARIES} ${OpenRAVE_LIBRARIES})

add_executable(yamltest
  test/YamlTest.cpp
  src/CachedAction.cpp
  src/SBPLBasePlannerTypes.cpp
)
target_link_libraries(yamltest
   yaml-cpp ${OpenRAVE_LIBRARIES} boost_system)

install(TARGETS or_sbpl
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN ".svn" EXCLUDE)


catkin_add_gtest(test_SBPLBasePlannerEnvironment tests/test_SBPLBasePlannerEnvironment.cpp)
target_link_libraries(test_SBPLBasePlannerEnvironment
  ${OpenRAVE_LIBRARIES}
  ${OpenRAVE_CORE_LIBRARIES}
  ${SBPL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROJECT_NAME}
  yaml-cpp
)

catkin_add_gtest(test_SBPLBasePlanner tests/test_SBPLBasePlanner.cpp)
target_link_libraries(test_SBPLBasePlanner
  ${OpenRAVE_LIBRARIES}
  ${OpenRAVE_CORE_LIBRARIES}
  ${SBPL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROJECT_NAME}
  yaml-cpp
)