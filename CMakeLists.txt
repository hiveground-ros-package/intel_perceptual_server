cmake_minimum_required(VERSION 2.8.3)
project(intel_perceptual_server)

find_package(catkin REQUIRED COMPONENTS roscpp interaction_msgs intel_perceptual_msgs)
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)

include(${QT_USE_FILE})
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

set(PCSDK_INCLUDE_DIR "$ENV{PCSDK_DIR}/include" CACHE PATH "docstring")
set(PCSDK_LIB_DIR "$ENV{PCSDK_DIR}/lib/Win32" CACHE PATH "docstring")

link_directories(
  ${PCSDK_LIB_DIR}
)

include_directories(
  include
  include/pcsdk_util
  ${CMAKE_CURRENT_BINARY_DIR}
  ${catkin_INCLUDE_DIRS}
  ${PCSDK_INCLUDE_DIR}
)

# Declare catkin package
catkin_package(
  INCLUDE_DIRS include
)

set(qt_hdrs
  include/intel_perceptual_server/intel_perceptual_server.h
)

set(qt_srcs	  
  src/main.cpp
  src/intel_perceptual_server.cpp
)

set(qt_uis
  src/main_window.ui
)

set(qt_rccs
  res/intel_perceptual_server.qrc
)

qt4_add_resources(qt_qrc ${qt_rccs})
qt4_automoc(${qt_srcs})
qt4_wrap_cpp(qt_moc_srcs ${qt_hdrs})
qt4_wrap_ui(uis_h ${qt_uis}) 

set(psdk_util_srcs
  src/pcsdk_util/util_capture.cpp
  src/pcsdk_util/util_capture_file.cpp
  src/pcsdk_util/util_captureimpl.cpp
  src/pcsdk_util/util_pipeline_face.cpp
  src/pcsdk_util/util_pipeline_gesture.cpp
  src/pcsdk_util/util_pipeline_raw.cpp
  src/pcsdk_util/util_pipeline_voice.cpp
  src/pcsdk_util/util_render.cpp
)

add_executable(${PROJECT_NAME}
  ${qt_qrc}
  ${qt_srcs}
  ${qt_moc_srcs}
  ${uis_h}
  ${psdk_util_srcs}
)   

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
  libpxc 
)
