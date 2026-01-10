#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "wheelchair2_kangaroo_x2_driver::wheelchair2_kangaroo_x2_driver" for configuration ""
set_property(TARGET wheelchair2_kangaroo_x2_driver::wheelchair2_kangaroo_x2_driver APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(wheelchair2_kangaroo_x2_driver::wheelchair2_kangaroo_x2_driver PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libwheelchair2_kangaroo_x2_driver.so"
  IMPORTED_SONAME_NOCONFIG "libwheelchair2_kangaroo_x2_driver.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS wheelchair2_kangaroo_x2_driver::wheelchair2_kangaroo_x2_driver )
list(APPEND _IMPORT_CHECK_FILES_FOR_wheelchair2_kangaroo_x2_driver::wheelchair2_kangaroo_x2_driver "${_IMPORT_PREFIX}/lib/libwheelchair2_kangaroo_x2_driver.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
