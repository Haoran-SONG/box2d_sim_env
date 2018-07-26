cmake_minimum_required(VERSION 2.8.3)
project(box2d_sim_env)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
        COMPONENTS
        cmake_modules
        box2d_catkin
        sim_env
        )
## System dependencies are found with CMake's conventions
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(Threads REQUIRED)
# find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML yaml-cpp REQUIRED)

# TODO the following line shouldn't be here. BUt linking without catkin_make does not work otherwise
# link_directories("/home/joshua/projects/planning_catkin/devel/lib/")
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES box2d_sim_env
        CATKIN_DEPENDS box2d_catkin sim_env
        DEPENDS EIGEN3 YAML Boost 
)

###########
## Build ##
###########
#set(CMAKE_CXX_COMPILER /usr/bin/clang++)

set(CMAKE_CXX_STANDARD 11)
add_definitions(-std=gnu++11)
add_definitions(-Wno-deprecated-register)
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
#set(CMAKE_VERBOSE_MAKEFILE on)
#add_definitions(-fext-numeric-literals)

## Qt specific
add_definitions(${QT_DEFINITIONS})

set(SOURCE_FILES
        src/sim_env/Box2DWorld.cpp
        src/sim_env/Box2DIOUtils.cpp
        src/sim_env/Box2DWorldViewer.cpp
        src/sim_env/Box2DController.cpp)

## Additional Qt specific things (in case we need it)
 set (MOC_HEADERS include/sim_env/Box2DWorldViewer.h)
# set (UIS ...)
# set (RESOURCES ...)
# QT4_ADD_RESOURCES(RES_SOURCES ${RESOURCES})
# QT4_WRAP_UI(UI_HEADERS ${UIS})
 QT4_WRAP_CPP( MOC_SRCS ${MOC_HEADERS})

## Specify additional locations of header files
## Your package locations should be listed before other locations
 include_directories(include
         ${sim_env_INCLUDE_DIRS}
         ${Eigen3_INCLUDE_DIRS}
         ${catkin_INCLUDE_DIRS}
         ${Boost_INCLUDE_DIRS}
         ${CMAKE_BINARY_DIR})

## Declare a C++ library
 add_library(box2d_sim_env
   ${SOURCE_FILES}
   ${MOC_SRCS}
 )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(box2d_sim_env ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(box2d_sim_env_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(box2d_sim_env
        sim_env
        ${catkin_LIBRARIES}
        yaml-cpp
        ${QT_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT}
 )

## Declare a C++ executable
add_executable(box2d_sim_env_manual_test tests/test_manual.cpp)

target_link_libraries(box2d_sim_env_manual_test
        box2d_sim_env
        sim_env
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        yaml-cpp
        ${QT_LIBRARIES}
        )

# Define tests
if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_Box2DWorld tests/test_Box2DWorld.cpp)
    target_link_libraries(test_Box2DWorld
            box2d_sim_env
            sim_env
            ${catkin_LIBRARIES}
            ${Boost_LIBRARIES}
            ${gtest_LIBRARIES}
            yaml-cpp)
endif(CATKIN_ENABLE_TESTING)
