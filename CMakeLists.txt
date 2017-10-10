cmake_minimum_required (VERSION 2.6)
project (Tutorial)

# The version number.
set (Tutorial_VERSION_MAJOR 2)
set (Tutorial_VERSION_MINOR 1)

# configure a header file to pass some of the CMake settings
# to the source code
configure_file (
#  "${PROJECT_SOURCE_DIR}/TutorialConfig.h.in"
  "${PROJECT_BINARY_DIR}/TutorialConfig.h"
  )
  
# add the binary tree to the search path for include files
# so that we will find TutorialConfig.h
include_directories("${PROJECT_BINARY_DIR}")

# add the executable
add_executable(main main.cxx)

SET_TARGET_PROPERTIES(main PROPERTIES LINKER_LANGUAGE C++)

#$(info $$PROJECT_SOURCE_DIR is [${PROJECT_SOURCE_DIR}])
