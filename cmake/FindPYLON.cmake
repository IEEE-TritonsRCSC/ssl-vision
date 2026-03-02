macro(PYLON_REPORT_NOT_FOUND REASON_MSG)
    unset(PYLON_FOUND)
    unset(PYLON_INCLUDE_DIRS)
    unset(PYLON_LIBRARIES)
    
    if(PYLON_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find pylon - " ${REASON_MSG} ${ARGN})    
    else()
        message("-- Failed to find pylon - " ${REASON_MSG} ${ARGN})
    endif()
    
endmacro(PYLON_REPORT_NOT_FOUND)

# === FIND PYLON_INCLUDE_DIR === #
find_path(PYLON_INCLUDE_DIR
    NAMES pylon/PylonBase.h
    PATHS
      /opt/pylon/include
      /usr/local/pylon/include
      /Library/Frameworks/pylon.framework/Headers
      $ENV{PYLON_ROOT}/include)

# macOS framework installations ship flat headers (e.g. PylonBase.h).
if(NOT PYLON_INCLUDE_DIR)
  find_path(PYLON_INCLUDE_DIR
      NAMES PylonBase.h
      PATHS
        /Library/Frameworks/pylon.framework/Headers
        /Library/Frameworks/pylon.framework/Versions/Current/Headers
        $ENV{PYLON_ROOT}/include)
endif()

if(NOT PYLON_INCLUDE_DIR OR NOT EXISTS ${PYLON_INCLUDE_DIR})
    PYLON_REPORT_NOT_FOUND(
        "Could not find pylon include directory. Set PYLON_INCLUDE_DIR "
        "to full path to pylon include directory,"
        "e.g. -DPYLON_INCLUDE_DIR=/opt/pylon/include/")
else()
    if(PYLON_INCLUDE_DIR MATCHES "\\.framework$")
      set(PYLON_INCLUDE_DIR "${PYLON_INCLUDE_DIR}/Headers")
    endif()
    message(STATUS "  pylon include dir found: " ${PYLON_INCLUDE_DIR})
endif()



# === FIND PYLON_LIBRARY_DIR === #
find_path(PYLON_LIBRARY_DIR
    NAMES libpylonbase.so libpylonutility.so libpylonbase.dylib libpylonutility.dylib
    PATHS
      /opt/pylon/lib
      /usr/local/pylon/lib
      /Library/Frameworks/pylon.framework/Libraries
      /Library/Frameworks/pylon.framework/Versions/Current/Libraries
      $ENV{PYLON_ROOT}/lib)

# === FIND PYLON framework library (macOS) === #
find_library(PYLON_FRAMEWORK
    NAMES pylon
    PATHS
      /Library/Frameworks
      /Library/Frameworks/pylon.framework
      /Library/Frameworks/pylon.framework/Versions/Current
      $ENV{PYLON_ROOT})

if(NOT PYLON_LIBRARY_DIR AND NOT PYLON_FRAMEWORK)
    PYLON_REPORT_NOT_FOUND(
        "Could not find pylon library. Set PYLON_LIBRARY_DIR "
        "to full path to pylon library directory, "
        "e.g. -DPYLON_LIBRARY_DIR=/opt/pylon/lib/")
else()
    if(PYLON_LIBRARY_DIR)
      message(STATUS "  pylon library dir found: " ${PYLON_LIBRARY_DIR})
    endif()
    if(PYLON_FRAMEWORK)
      message(STATUS "  pylon framework found: " ${PYLON_FRAMEWORK})
    endif()
endif()

# === SET PYLON_FOUND AND SET PYLON_INCLUDE_DIRS / PYLON_LIBRARIES === #
if(PYLON_INCLUDE_DIR AND (PYLON_LIBRARY_DIR OR PYLON_FRAMEWORK))
    set(PYLON_FOUND TRUE)
endif()

set(PYLON_INCLUDE_DIRS ${PYLON_INCLUDE_DIR})
if(EXISTS "${PYLON_INCLUDE_DIR}/GenICam")
  list(APPEND PYLON_INCLUDE_DIRS "${PYLON_INCLUDE_DIR}/GenICam")
endif()

if(APPLE)
  if(PYLON_FRAMEWORK)
    # On macOS, link the framework target instead of globs in Libraries/
    # to avoid pulling non-linkable transport layer modules (*.so).
    set(PYLON_LIBRARIES ${PYLON_FRAMEWORK})
  else()
    find_library(PYLON_BASE_LIB NAMES pylonbase libpylonbase.dylib PATHS ${PYLON_LIBRARY_DIR})
    find_library(PYLON_UTILITY_LIB NAMES pylonutility libpylonutility.dylib PATHS ${PYLON_LIBRARY_DIR})
    set(PYLON_LIBRARIES ${PYLON_BASE_LIB} ${PYLON_UTILITY_LIB})
  endif()
elseif(PYLON_LIBRARY_DIR)
  file(GLOB PYLON_LIBRARIES ${PYLON_LIBRARY_DIR}/lib*.so)
endif()
