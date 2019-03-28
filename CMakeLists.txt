cmake_minimum_required(VERSION 2.8)
project(motion_controller)

find_package(qibuild)

# Create a list of source files
set(_srcs
    motion_controller.cpp
    motion_controller.h
    main.cpp)

# Create a executable named web_controller
# with the source file: main.cpp
qi_create_lib(motion_controller SHARED ${_srcs} SUBFOLDER naoqi)
#qi_create_bin(motion_controller ${_srcs} SUBFOLDER naoqi)

qi_use_lib(motion_controller ALCOMMON ALPROXIES BOOST)
