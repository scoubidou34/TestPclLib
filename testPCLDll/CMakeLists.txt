
cmake_minimum_required(VERSION 2.8.11)

# Project name
project(testPCLDll)

# This setting is useful for providing JSON file used by CodeLite for code completion
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# Compiler options
# Compiler options
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CONFIGURATION_NAME Debug)
    add_definitions(-g)
    add_definitions(-O0)
    add_definitions(-Wall)
    set_source_files_properties(
    ${CXX_SRCS} PROPERTIES COMPILE_FLAGS 
    " -g")
else(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CONFIGURATION_NAME Release)
    add_definitions(-O2)
    add_definitions(-Wall)
    add_definitions(-DNDEBUG)
    set_source_files_properties(
    ${CXX_SRCS} PROPERTIES COMPILE_FLAGS 
    " -O2")
endif(CMAKE_BUILD_TYPE STREQUAL "Debug")
message( ${CONFIGURATION_NAME})

set(CL_WORKSPACE_DIRECTORY ..)
# Set default locations
set(CL_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/${CL_WORKSPACE_DIRECTORY}/cmake-build-${CONFIGURATION_NAME}/output)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CL_OUTPUT_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CL_OUTPUT_DIRECTORY})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CL_OUTPUT_DIRECTORY})

# Projects


# Top project
# Define some variables
set(PROJECT_testPCLDll_PATH "${CMAKE_CURRENT_LIST_DIR}")
set(WORKSPACE_PATH "${CMAKE_CURRENT_LIST_DIR}/..")

find_package(PCL REQUIRED)
find_package(Boost REQUIRED)
find_package(VTK REQUIRED)

include(${VTK_USE_FILE})

include_directories(
${PCL_INCLUDE_DIRS}
/usr/include/libjpeg-turbo/include/
)

INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#target_link_libraries (testPCLDll ${PCL_LIBRARIES})
link_directories(${VTK_LIBRARIES})

#{{{{ User Code 1
# Place your code here
#}}}}





# Linker options



# Library path
link_directories(
    .
    /usr/local/lib
    ${Boost_LIBRARIES}
)

# Define the CXX sources
set ( CXX_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/PclRenderer.cpp
    ${CMAKE_CURRENT_LIST_DIR}/PclRendererImpl.cpp
)


set ( JPEGTURBOLIB 
    /usr/local/lib/libjpeg.so
    /usr/local/lib/libturbojpeg.so
)



#{{{{ User Code 2
# Place your code here
#}}}}

add_library(testPCLDll SHARED ${RC_SRCS} ${CXX_SRCS} ${C_SRCS} )
target_link_libraries(testPCLDll ${LINK_OPTIONS})
target_link_libraries(testPCLDll ${Boost_LIBRARIES})
target_link_libraries (testPCLDll ${PCL_LIBRARIES})
target_link_libraries (testPCLDll ${VTK_LIBRARIES})
target_link_libraries (testPCLDll ${JPEGTURBOLIB})
#{{{{ User Code 3
# Place your code here
#}}}}

