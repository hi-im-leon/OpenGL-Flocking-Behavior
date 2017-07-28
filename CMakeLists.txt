cmake_minimum_required(VERSION 2.8.11)
set(CMAKE_CXX_FLAGS "-Wall -g -std=c++11 -DUNIX")
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

#for FindGLM.cmake, FindGLFW3.cmake, other helpers
set (CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake_modules) 

FIND_PACKAGE(OpenGL REQUIRED) #for graphics
FIND_PACKAGE(GLEW REQUIRED)

add_subdirectory(aBasicViewer)
add_subdirectory(aCurveEditor)
add_subdirectory(aRotations)
add_subdirectory(aIKViewer)
add_subdirectory(libsrc)
add_subdirectory(aChaseViewer)