cmake_minimum_required(VERSION 2.8.3)
project(hades_comm_handler)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    theora_image_transport
    message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg
    FILES
    EZR_Pkt.msg)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    theora_image_transport  
)

###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
catkin_package(
    CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(hades_streamer
    src/hades_streamer.cpp
)

## Add cmake target dependencies of the executable
add_dependencies(hades_streamer
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(hades_streamer
   ${catkin_LIBRARIES}
)


