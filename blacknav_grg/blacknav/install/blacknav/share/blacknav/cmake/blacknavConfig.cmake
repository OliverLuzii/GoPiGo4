# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_blacknav_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED blacknav_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(blacknav_FOUND FALSE)
  elseif(NOT blacknav_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(blacknav_FOUND FALSE)
  endif()
  return()
endif()
set(_blacknav_CONFIG_INCLUDED TRUE)

# output package information
if(NOT blacknav_FIND_QUIETLY)
  message(STATUS "Found blacknav: 0.0.0 (${blacknav_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'blacknav' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT blacknav_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(blacknav_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${blacknav_DIR}/${_extra}")
endforeach()
