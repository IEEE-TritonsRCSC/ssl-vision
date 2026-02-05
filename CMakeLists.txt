# Minimum cmake version is set to current version in Ubuntu 20.04
cmake_minimum_required(VERSION 3.16.3)
project(ssl-vision)

set(OpenGL_GL_PREFERENCE LEGACY)
set(USE_DC1394 TRUE CACHE BOOL "Compile with DC1394 driver (firewire cameras)")
set(USE_SPINNAKER FALSE CACHE BOOL "Compile with Spinnaker driver (FLIR cameras)")
set(USE_mvIMPACT FALSE CACHE BOOL "Compile with mvImpact driver (bluefox USB2 + USB3 cameras)")
set(USE_PYLON FALSE CACHE BOOL "Compile with pylon driver (Basler cameras)")
set(USE_FLYCAP FALSE CACHE BOOL "Compile with flycap driver (FLIR cameras, predecessor of Spinnaker)")

if(APPLE)
  set(_DEFAULT_USE_V4L FALSE)
else()
  set(_DEFAULT_USE_V4L TRUE)
endif()
set(USE_V4L ${_DEFAULT_USE_V4L} CACHE BOOL "Compile with Video4Linux support (generic webcams)")
unset(_DEFAULT_USE_V4L)

if(APPLE)
  set(USE_AVFOUNDATION TRUE CACHE BOOL "Compile with AVFoundation capture support (macOS cameras)")
else()
  set(USE_AVFOUNDATION FALSE CACHE BOOL "Compile with AVFoundation capture support (macOS cameras)")
endif()

set(USE_SPLITTER FALSE CACHE BOOL "Compile with Camera splitter support (virtual cameras with part of a full image)")

if(USE_DC1394 AND USE_mvIMPACT)
	message(FATAL_ERROR "DC1394 and mvImpact are not compatible: mvImpact crashes when creating device manager")
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_STANDARD 11)

if(APPLE)
  set(CMAKE_OSX_ARCHITECTURES "arm64" CACHE STRING "macOS architectures")
  if(NOT DEFINED CMAKE_OSX_DEPLOYMENT_TARGET)
    if(CMAKE_HOST_SYSTEM_VERSION)
      set(CMAKE_OSX_DEPLOYMENT_TARGET "${CMAKE_HOST_SYSTEM_VERSION}" CACHE STRING "Minimum macOS version")
    else()
      set(CMAKE_OSX_DEPLOYMENT_TARGET "13.0" CACHE STRING "Minimum macOS version")
    endif()
  endif()
  set(CMAKE_MACOSX_RPATH ON)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated-declarations")

  # Qt6-specific macOS fixes for better UI performance
  if(_qt_version EQUAL 6)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DQT_NO_DEBUG_OUTPUT")
    # Disable high DPI scaling as it causes issues with LUT widget
    add_definitions(-DQT_DISABLE_DEPRECATED_BEFORE=0x050f00)
  endif()
endif()

# Find third-party libraries
find_package(OpenGL REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
if(Eigen3_VERSION VERSION_LESS 3.3)
  message(FATAL_ERROR "Eigen3 3.3 or newer is required; found ${Eigen3_VERSION}")
endif()

# Fix protobuf library linking (use pkg-config to get abseil dependencies)
pkg_check_modules(PROTOBUF REQUIRED protobuf)
set(PROTOBUF_LIBRARIES ${PROTOBUF_LDFLAGS})
include_directories(${PROTOBUF_INCLUDE_DIRS})
set(_qt_components Core Widgets OpenGL)
set(_qt_version 5)
find_package(Qt${_qt_version} COMPONENTS ${_qt_components} QUIET)
if(NOT Qt${_qt_version}_FOUND)
  set(_qt_version 6)
  find_package(Qt6 COMPONENTS ${_qt_components} OpenGLWidgets REQUIRED)
endif()

if(_qt_version EQUAL 6)
  set(QT_PACKAGE Qt6)
  macro(qt_wrap_cpp)
    qt6_wrap_cpp(${ARGN})
  endmacro()
  macro(qt_wrap_ui)
    qt6_wrap_ui(${ARGN})
  endmacro()
  macro(qt_add_resources)
    qt6_add_resources(${ARGN})
  endmacro()
  set(QT_LIBRARIES ${QT_PACKAGE}::Core ${QT_PACKAGE}::Widgets ${QT_PACKAGE}::OpenGL Qt6::OpenGLWidgets)
else()
  set(QT_PACKAGE Qt5)
  macro(qt_wrap_cpp)
    qt5_wrap_cpp(${ARGN})
  endmacro()
  macro(qt_wrap_ui)
    qt5_wrap_ui(${ARGN})
  endmacro()
  macro(qt_add_resources)
    qt5_add_resources(${ARGN})
  endmacro()
  set(QT_LIBRARIES ${QT_PACKAGE}::Core ${QT_PACKAGE}::Widgets ${QT_PACKAGE}::OpenGL)
endif()

set(AVFOUNDATION_LIBS "")

# Note: Bluefox SDK is not compatible with some components of OpenCV. Take care when enabling more.
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio calib3d)

include(src/shared/CMakeLists.txt.inc)

if( USE_mvIMPACT )
	find_package(mvIMPACT REQUIRED)
	include_directories(${mvIMPACT_INCLUDE_DIR})
	add_definitions(-D MVIMPACT3)
	add_definitions(-D MVIMPACT2)
	set (SHARED_SRCS ${SHARED_SRCS}
		  ${shared_dir}/capture/capture_bluefox2.cpp
		  ${shared_dir}/capture/capture_bluefox3.cpp
		)
	set (SHARED_HEADERS ${SHARED_HEADERS}
		  ${shared_dir}/capture/capture_bluefox2.h
		  ${shared_dir}/capture/capture_bluefox3.h
		)
	message(STATUS "mvIMPACT enabled")
else()
	message(STATUS "mvIMPACT disabled")
endif()

if( USE_FLYCAP )
	set(FLYCAP_HINTS "/usr/include/flycapture")
	find_package(FLYCAP REQUIRED)
	include_directories(${FLYCAP_INCLUDE_DIRS})
	add_definitions(-D FLYCAP)
	set (SHARED_SRCS ${SHARED_SRCS} ${shared_dir}/capture/capture_flycap.cpp)
	set (SHARED_HEADERS ${SHARED_HEADERS} ${shared_dir}/capture/capture_flycap.h)
	set (FLYCAP_LIBS flycapture)
	message(STATUS "Flycap enabled")
else()
  message(STATUS "Flycap disabled")
endif()

if(USE_DC1394)
	pkg_check_modules(DC1394 REQUIRED libdc1394-2)
	add_definitions(-D DC1394)
	set (SHARED_SRCS ${SHARED_SRCS}
			${shared_dir}/capture/capturedc1394v2.cpp
			)
	set (SHARED_HEADERS ${SHARED_HEADERS}
			${shared_dir}/capture/capturedc1394v2.h
			)
	message(STATUS "DC1394 enabled")
else()
	add_definitions(-D NO_DC1394_CONVERSIONS)
	set(DC1394_LIBRARIES "")
	message(STATUS "DC1394 disabled")
endif()

if( USE_PYLON )
	find_package( PYLON REQUIRED )
	include_directories(SYSTEM ${PYLON_INCLUDE_DIRS})
	add_definitions(-DPYLON)
	set (SHARED_SRCS ${SHARED_SRCS} ${shared_dir}/capture/capture_basler.cpp)
	set (SHARED_HEADERS ${SHARED_HEADERS} ${shared_dir}/capture/capture_basler.h)
	message(STATUS "Pylon enabled")
else()
	message(STATUS "Pylon disabled")
endif()

if( USE_SPINNAKER )
	find_package( SPINNAKER REQUIRED )
	add_definitions( -DSPINNAKER )
	include_directories( ${SPINNAKER_INCLUDE_DIRS} )
	set (SHARED_SRCS ${SHARED_SRCS} ${shared_dir}/capture/capture_spinnaker.cpp)
	set (SHARED_HEADERS ${SHARED_HEADERS} ${shared_dir}/capture/capture_spinnaker.h)
	message(STATUS "SPINNAKER enabled")
else()
	set( SPINNAKER_LIBS "" )
	message(STATUS "SPINNAKER disabled")
endif()

if(USE_V4L)
    add_definitions(-DV4L)
    set (SHARED_SRCS ${SHARED_SRCS}
        ${shared_dir}/capture/capturev4l.cpp
        )
    set (SHARED_HEADERS ${SHARED_HEADERS}
        ${shared_dir}/capture/capturev4l.h
        )
    if(APPLE)
      message(FATAL_ERROR "USE_V4L is not supported on macOS. Disable it or build on Linux.")
    else()
      pkg_check_modules(V4L REQUIRED libv4l2)
      include_directories(${V4L_INCLUDE_DIRS})
      set(V4L_LIBS ${V4L_LIBS} ${V4L_LIBRARIES})
    endif()
endif()

if(APPLE AND USE_AVFOUNDATION)
  add_definitions(-DAVFOUNDATION_CAPTURE)
  find_library(AVFOUNDATION_FRAMEWORK AVFoundation REQUIRED)
  find_library(COREMEDIA_FRAMEWORK CoreMedia REQUIRED)
  find_library(COREVIDEO_FRAMEWORK CoreVideo REQUIRED)
  find_library(QUARTZCORE_FRAMEWORK QuartzCore REQUIRED)
  find_library(APPSERVICES_FRAMEWORK ApplicationServices REQUIRED)
  find_library(ACCELERATE_FRAMEWORK Accelerate REQUIRED)
  set(AVFOUNDATION_LIBS
      ${AVFOUNDATION_FRAMEWORK}
      ${COREMEDIA_FRAMEWORK}
      ${COREVIDEO_FRAMEWORK}
      ${QUARTZCORE_FRAMEWORK}
      ${APPSERVICES_FRAMEWORK}
      ${ACCELERATE_FRAMEWORK})
  set(SHARED_SRCS ${SHARED_SRCS}
      ${shared_dir}/capture/capture_avfoundation.mm)
  set(SHARED_HEADERS ${SHARED_HEADERS}
      ${shared_dir}/capture/capture_avfoundation.h)
endif()

if( USE_SPLITTER )
	add_definitions( -DCAMERA_SPLITTER )
	set (SHARED_SRCS ${SHARED_SRCS} ${shared_dir}/capture/capture_splitter.cpp)
	set (SHARED_HEADERS ${SHARED_HEADERS} ${shared_dir}/capture/capture_splitter.h)
	set (SRCS ${SRCS}
			src/app/plugins/plugin_distribute.cpp
			src/app/stacks/DistributorStack.cpp
		)

	message(STATUS "Camera splitter enabled")
else()
	message(STATUS "Camera splitter disabled")
endif()

include_directories(${PROJECT_SOURCE_DIR}/src/app)
include_directories(${PROJECT_SOURCE_DIR}/src/app/gui)
include_directories(${PROJECT_SOURCE_DIR}/src/app/plugins)
include_directories(${PROJECT_SOURCE_DIR}/src/app/stacks)

set (SRCS ${SRCS}
	src/app/capture_thread.cpp
	src/app/framedata.cpp
	src/app/main.cpp

    src/app/gui/maskwidget.cpp
	src/app/gui/automatedcolorcalibwidget.cpp
	src/app/gui/camera_intrinsic_calib_widget.cpp
	src/app/gui/cameracalibwidget.cpp
	src/app/gui/colorpicker.cpp
	src/app/gui/glLUTwidget.cpp
	src/app/gui/glwidget.cpp
	src/app/gui/lutwidget.cpp
	src/app/gui/mainwindow.cpp
	src/app/gui/realtimedisplaywidget.cpp
	src/app/gui/renderoptions.cpp
	src/app/gui/videowidget.cpp
	src/app/gui/jog_dial.cpp

	src/app/plugins/plugin_mask.cpp
	src/app/plugins/plugin_cameracalib.cpp
	src/app/plugins/plugin_camera_intrinsic_calib.cpp
	src/app/plugins/plugin_colorcalib.cpp
	src/app/plugins/plugin_colorthreshold.cpp
	src/app/plugins/plugin_detect_balls.cpp
	src/app/plugins/plugin_detect_robots.cpp
	src/app/plugins/plugin_find_blobs.cpp
	src/app/plugins/plugin_publishgeometry.cpp
	src/app/plugins/plugin_legacypublishgeometry.cpp
	src/app/plugins/plugin_runlength_encode.cpp
	src/app/plugins/plugin_sslnetworkoutput.cpp
	src/app/plugins/plugin_legacysslnetworkoutput.cpp
	src/app/plugins/plugin_visualize.cpp
	src/app/plugins/plugin_dvr.cpp
	src/app/plugins/plugin_auto_color_calibration.cpp
	src/app/plugins/visionplugin.cpp

	src/app/stacks/multistack_robocup_ssl.cpp
	src/app/stacks/multivisionstack.cpp
	src/app/stacks/stack_robocup_ssl.cpp
	src/app/stacks/visionstack.cpp
)

qt_wrap_cpp (MOC_SRCS
	src/app/capture_thread.h

	src/app/gui/maskwidget.h
	src/app/gui/automatedcolorcalibwidget.h
	src/app/gui/camera_intrinsic_calib_widget.h
	src/app/gui/cameracalibwidget.h
	src/app/gui/glLUTwidget.h
	src/app/gui/glwidget.h
	src/app/gui/lutwidget.h
	src/app/gui/mainwindow.h
	src/app/gui/videowidget.h
	src/app/gui/jog_dial.h

	src/app/plugins/plugin_dvr.h
	src/app/plugins/plugin_publishgeometry.h
	src/app/plugins/plugin_legacypublishgeometry.h
	src/app/plugins/visionplugin.h
	src/app/plugins/plugin_colorcalib.h
	src/app/plugins/plugin_colorthreshold.h
	src/app/plugins/plugin_auto_color_calibration.h
	src/app/plugins/plugin_camera_intrinsic_calib.h

	src/app/stacks/multistack_robocup_ssl.h

	src/shared/util/camera_parameters.h

	${OPTIONAL_HEADERS}
)

qt_wrap_ui (UI_SRCS
	src/app/gui/mainwindow.ui
	src/app/gui/videowidget.ui
	${OPTIONAL_UI_SRCS}
)

qt_add_resources(RC_SRCS
	src/app/gui/icons/icons_gui.qrc
	${SHARED_RC}
)

## generate moc files for graphicalClient
qt_wrap_cpp(GCLIENT_MOC_SRCS
	src/graphicalClient/soccerview.h
)

## shared qt wrappings
qt_wrap_cpp (SHARED_MOC_SRCS
	${SHARED_HEADERS}
)

qt_add_resources(SHARED_RC_SRCS
	${SHARED_RC}
)

# enable warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated-declarations")

#flags to set in debug mode
if(NOT APPLE)
  set (CMAKE_CXX_FLAGS_DEBUG "-g -Wl,--no-as-needed")
else()
  set (CMAKE_CXX_FLAGS_DEBUG "-g")
endif()

#flags to set in release mode
if(NOT APPLE)
  set (CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG -O3 -march=native -Wl,--no-as-needed")
else()
  set (CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG -O3 -march=native")
endif()

## build the common code
add_library(sslvision ${SHARED_MOC_SRCS} ${SHARED_RC_SRCS} ${CC_PROTO} ${SHARED_SRCS})
add_dependencies(sslvision GenerateProto)

set (libs
  ${QT_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  ${OPENGL_gl_LIBRARY}
  ${OPENGL_glu_LIBRARY}
  ${PROTOBUF_LIBRARIES}
  ${DC1394_LIBRARIES}
  ${mvIMPACT_LIBS}
  ${PYLON_LIBRARIES}
  ${OpenCV_LIBS}
  ${FLYCAP_LIBS}
  ${SPINNAKER_LIBS}
  ${V4L_LIBS}
  ${AVFOUNDATION_LIBS}
  Eigen3::Eigen
)
target_link_libraries(sslvision ${libs} ${QT_PACKAGE}::Widgets)
set (libs ${libs} sslvision)

## build the main app
add_executable(vision ${UI_SRCS} ${MOC_SRCS} ${RC_SRCS} ${SRCS})
if(_qt_version EQUAL 6)
  target_link_libraries(vision ${libs} ${QT_PACKAGE}::Widgets ${QT_PACKAGE}::OpenGL Qt6::OpenGLWidgets)
else()
  target_link_libraries(vision ${libs} ${QT_PACKAGE}::Widgets ${QT_PACKAGE}::OpenGL)
endif()

## build non graphical client
add_executable(client src/client/main.cpp )
target_link_libraries(client ${libs} ${QT_PACKAGE}::Core)

## build graphical client
add_executable(graphicalClient ${GCLIENT_MOC_SRCS}
  src/graphicalClient/main.cpp
  src/graphicalClient/soccerview.cpp
  src/graphicalClient/gltext.cpp
)
target_link_libraries(graphicalClient ${libs} ${QT_PACKAGE}::Widgets ${QT_PACKAGE}::OpenGL)
