
cmake_minimum_required(VERSION 2.8)

PROJECT(Viewer)
set (Viewer_VERSION_MAJOR 0)
set (Viewer_VERSION_MINOR 1)


find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

#set(OpenCV_DIR "c:/opencv/build")
#set(OpenCV_LIBS "c:/opencv/build/x64/vc12")
#set(OpenCV_INCLUDE_DIRS "c:/opencv/build/include")
#find_package(OpenCV REQUIRED)

add_executable(Viewer MACOSX_BUNDLE Viewer)

target_link_libraries(Viewer ${VTK_LIBRARIES})
#target_link_libraries(Viewer ${OpenCV_LIBS})
#target_link_libraries(Viewer ${OpenCV_LIBS})
