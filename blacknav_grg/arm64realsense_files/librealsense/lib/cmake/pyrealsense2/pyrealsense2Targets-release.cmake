#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "pyrealsense2::pyrsutils" for configuration "Release"
set_property(TARGET pyrealsense2::pyrsutils APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(pyrealsense2::pyrsutils PROPERTIES
  IMPORTED_LOCATION_RELEASE "/usr/local/lib/python3.12/dist-packages/pyrealsense2/pyrsutils.cpython-312-aarch64-linux-gnu.so.2.56.5"
  IMPORTED_SONAME_RELEASE "pyrsutils.cpython-312-aarch64-linux-gnu.so.2.56"
  )

list(APPEND _cmake_import_check_targets pyrealsense2::pyrsutils )
list(APPEND _cmake_import_check_files_for_pyrealsense2::pyrsutils "/usr/local/lib/python3.12/dist-packages/pyrealsense2/pyrsutils.cpython-312-aarch64-linux-gnu.so.2.56.5" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
