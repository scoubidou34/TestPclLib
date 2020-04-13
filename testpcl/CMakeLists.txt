cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(testpcl)

# This setting is useful for providing JSON file used by CodeLite for code completion
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

set(CONFIGURATION_NAME Debug)

set(CL_WORKSPACE_DIRECTORY ..)
# Set default locations
set(CL_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/${CL_WORKSPACE_DIRECTORY}/cmake-build-${CONFIGURATION_NAME}/output)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CL_OUTPUT_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CL_OUTPUT_DIRECTORY})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CL_OUTPUT_DIRECTORY})

# Projects


# Top project
# Define some variables
set(PROJECT_testpcl_PATH "${CMAKE_CURRENT_LIST_DIR}")
set(WORKSPACE_PATH "${CMAKE_CURRENT_LIST_DIR}/..")

find_package(PCL REQUIRED)
find_package(VTK REQUIRED)



include_directories(
../testPCLDll
)

#link_directories(${PCL_LIBRARY_DIRS})
# Library path
link_directories(    
    /usr/local/lib
    ${CL_OUTPUT_DIRECTORY}    
    /home/tom
)
add_definitions(${PCL_DEFINITIONS})

add_executable (testpcl src/main.cpp)
target_link_libraries (testpcl ${PCL_LIBRARIES})
target_link_libraries (testpcl testPCLDll)
target_link_libraries (testpcl ${VTK_LIBRARIES})
#target_link_libraries (testpcl ${Boost_LIBRARIES})

#set_target_properties(testpcl PROPERTIES RUNTIME_OUTPUT_DIRECTORY "./dist/Debug/GNU-Linux/")
