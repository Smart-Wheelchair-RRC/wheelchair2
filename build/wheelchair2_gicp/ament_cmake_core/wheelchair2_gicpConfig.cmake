# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_wheelchair2_gicp_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED wheelchair2_gicp_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(wheelchair2_gicp_FOUND FALSE)
  elseif(NOT wheelchair2_gicp_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(wheelchair2_gicp_FOUND FALSE)
  endif()
  return()
endif()
set(_wheelchair2_gicp_CONFIG_INCLUDED TRUE)

# output package information
if(NOT wheelchair2_gicp_FIND_QUIETLY)
  message(STATUS "Found wheelchair2_gicp: 0.0.0 (${wheelchair2_gicp_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'wheelchair2_gicp' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${wheelchair2_gicp_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(wheelchair2_gicp_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${wheelchair2_gicp_DIR}/${_extra}")
endforeach()
