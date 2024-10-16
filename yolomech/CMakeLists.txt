cmake_minimum_required(VERSION 3.0.2)
project(yolomech)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  # pcl library
  pcl_conversions
)

## System dependencies are found with CMake's conventions
# Mech EYE
# find_package(PkgConfig)
# if(NOT PkgConfig_FOUND)
#     message(FATAL_ERROR "PkgConfig not found.")
# else()
#     pkg_check_modules(MECHEYEAPI REQUIRED MechEyeApi)
#     if(NOT MECHEYEAPI_FOUND)
#       message(
#         FATAL_ERROR "MechEyeApi not found. Please install MechEyeApi first.")
#     endif()
# endif()
# include_directories(${MECHEYEAPI_INCLUDE_DIRS})
# link_directories(${MECHEYEAPI_LIBRARY_DIRS})

# pcl
find_package(PCL REQUIRED)
if(NOT PCL_FOUND)
  message(
    FATAL_ERROR
      "Point Cloud Library (PCL) not found. Please point PCL_DIR to the directory of your PCL installation (containing the file PCLConfig.cmake)."
  )
endif()
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})
add_definitions(-DBOOST_ALL_DYN_LINK)

# TBB
find_package(TBB REQUIRED)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp rospy std_msgs
 DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
link_directories(
  include
)

## Declare a C++ library
add_library(template_alignment SHARED src/template_alignment.cpp)
# add_executable (shoot_template src/shoot_template.cpp)

add_dependencies(template_alignment ${catkin_EXPORTED_TARGETS})
# add_dependencies(shoot_template ${catkin_EXPORTED_TARGETS})

target_link_libraries (template_alignment pthread ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} TBB::tbb)
# target_link_libraries (shoot_template pthread ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} TBB::tbb)