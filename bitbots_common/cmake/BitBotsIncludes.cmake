# BiBotsIncludes.cmake
#
# This File includes various CMake definition needet for BitBots relatet Code
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") #enable c++11 for all build-types

# Python-Version finden
include(FindPythonInterp)
# Cython danach angeben, sonst vcersionskonflikte
include(UseCython)

#add_custom_target(ReplicatePythonSourceTree ALL ${CMAKE_COMMAND} -P
#  ${cython_catkin_example_ROOT}/cmake/ReplicatePythonSourceTree.cmake
#  ${CMAKE_CURRENT_BINARY_DIR}
#  WORKING_DIRECTORY src/${PROJECT_NAME}/src/${PROJECT_NAME})