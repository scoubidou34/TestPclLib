cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcltest2)

find_package(PCL REQUIRED)
find_package(VTK REQUIRED)

include(${VTK_USE_FILE})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcltest src/main.cpp)
target_link_libraries (pcltest ${PCL_LIBRARIES})
target_link_libraries (pcltest ${VTK_LIBRARIES})

#set_target_properties(pcltest PROPERTIES RUNTIME_OUTPUT_DIRECTORY "./dist/Debug/GNU-Linux/")
