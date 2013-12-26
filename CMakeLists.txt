cmake_minimum_required(VERSION 2.8.9)

project(ScannerProje)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

FIND_PACKAGE(PCL 1.6 REQUIRED)
FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui REQUIRED)
FIND_PACKAGE(VTK)

INCLUDE(${QT_USE_FILE})
INCLUDE(${VTK_USE_FILE})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)


set(qt_HEADERS mainwindow.h)
set(qt_SOURCES main.cpp mainwindow.cpp)
set(qt_UIS mainwindow.ui)
set(qt_RESOURCES resources.qrc)

##optimization
add_definitions(-O3 -Wno-deprecated -DNDEBUG -pipe -ffast-math -funroll-loops -ftree-vectorize -fomit-frame-pointer -pipe -mfpmath=sse -mmmx -msse -mtune=core2 -march=core2 -msse2 -msse3 -mssse3 -msse4)
#add_definitions(-momit-leaf-frame-pointer -fomit-frame-pointer -floop-block -ftree-loop-distribution -ftree-loop-linear -floop-interchange -floop-strip-mine -fgcse-lm -fgcse-sm -fsched-spec-load)
add_definitions (-O3 -Winvalid-pch -pipe -funroll-loops -fno-strict-aliasing)

qt4_wrap_cpp(myproject_MOC ${qt_HEADERS})
qt4_wrap_ui(qt_UIS_H ${qt_UIS})
qt4_add_resources(qt_RESOURCES_QRC ${qt_RESOURCES})


add_executable(ScannerProje ${qt_SOURCES}
    ${myproject_MOC}
    ${qt_RESOURCES_QRC}
    ${qt_UIS_H} rs232.c motordriver.cpp)

target_link_libraries(ScannerProje ${QT_LIBRARIES} QVTK vtkRendering ${PCL_LIBRARIES})
