cmake_minimum_required(VERSION 2.8.3)
project(robo_gui)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

if(ANDROID)
    set (ENV{JAVA_HOME} /usr/lib/jvm/java-7-openjdk-amd64)
    set (QT_ANDROID_ANT /usr/bin/ant)
    set (PYTHON_EXECUTABLE /usr/bin/python)
    set (ANDROID_COMPILER_VERSION 4.9)

    set (QTDIR /opt/Qt/5.5)
    set (Qt5Core_DIR ${QTDIR}/android_armv7/lib/cmake/Qt5Core)
    set (Qt5Gui_DIR ${QTDIR}/android_armv7/lib/cmake/Qt5Gui)
    set (Qt5Quick_DIR ${QTDIR}/android_armv7/lib/cmake/Qt5Quick)
    set (Qt5Svg_DIR ${QTDIR}/android_armv7/lib/cmake/Qt5Svg)
    set (Qt5Multimedia_DIR ${QTDIR}/android_armv7/lib/cmake/Qt5Multimedia)
    set (Qt5AndroidExtras_DIR ${QTDIR}/android_armv7/lib/cmake/Qt5AndroidExtras)
    # path to cross-compiled dependency of compressed_image_transport tiff, jpeg
    LINK_DIRECTORIES(${LINK_DIRECTORIES} /opt/roscpp_output/libs/opencv-2.4.9/build/3rdparty/lib)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  std_msgs
  image_transport
  compressed_image_transport
  compressed_depth_image_transport
  rospack
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Qt5Core)
find_package(Qt5Gui)
find_package(Qt5Quick)
find_package(Qt5Multimedia)
find_package(Qt5Svg)

set(CMAKE_AUTOMOC ON)

set(LIBS
    Qt5::Core
    Qt5::Gui
    Qt5::Quick
    Qt5::Multimedia
    Qt5::Svg
    z
)

if (ANDROID)
    find_package(Qt5AndroidExtras)
    set (LIBS ${LIBS} Qt5::AndroidExtras log libjpeg libtiff libjasper libpng IlmImf android)
endif()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#  Rect.msg
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  sensor_msgs
#  std_msgs
#)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
#generate_dynamic_reconfigure_options(
#   cfg/Camera.cfg
#   cfg/DynReconf2.cfg
#)

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
#  INCLUDE_DIRS include
#  LIBRARIES robo_gui
  CATKIN_DEPENDS tracker gamepad_controller robo_core
#                message_runtime dynamic_reconfigure
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

## Declare a C++ library
# add_library(robo_gui
#   src/${PROJECT_NAME}/robo_gui.cpp
# )

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/RecurseDirs.cmake)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(robo_gui ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
HEADER_DIRECTORIES(src DIRS)

qt5_add_resources(RESOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/qml.qrc
                            ${CMAKE_CURRENT_SOURCE_DIR}/src/resources.qrc)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${CATKIN_DEVEL_PREFIX}/include)

include_directories(${DIRS})

## Declare a C++ executable
if(ANDROID)
## Include NDK headers
    include_directories(${ANDROID_SYSROOT}/usr/include)
    add_library(${PROJECT_NAME} SHARED ${SOURCES} ${RESOURCES})
    set_target_properties(${PROJECT_NAME} PROPERTIES VERSION 0.1)
else(ANDROID)
    add_executable(${PROJECT_NAME} ${SOURCES} ${RESOURCES})
endif()

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${LIBS}
)

if(ANDROID)
    set (QT_ANDROID_APP_NAME ${PROJECT_NAME})
    include(3rdparty/qt-android-cmake/AddQtAndroidApk.cmake)
    add_qt_android_apk(${PROJECT_NAME}_apk.
                        ${PROJECT_NAME}
                        NAME "Robotank"
                        PACKAGE_NAME "vitaliy.bondar.robotank"
                        PACKAGE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/android
)
endif()

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
# install(TARGETS robo_gui robo_gui_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robo_gui.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
