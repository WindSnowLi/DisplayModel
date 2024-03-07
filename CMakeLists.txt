cmake_minimum_required(VERSION 3.27)
cmake_policy(SET CMP0074 NEW) 
if(POLICY CMP0020)
  cmake_policy(SET CMP0020 NEW)
  cmake_policy(SET CMP0071 NEW)
endif()
project(DisplayModel VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 REQUIRED COMPONENTS Widgets LinguistTools OpenGLWidgets Concurrent)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets LinguistTools OpenGLWidgets Concurrent)


find_package(VTK COMPONENTS 
  CommonColor
  CommonCore  
  CommonDataModel
  CommonTransforms  
  IOPLY
  IOGeometry
  IOChemistry
  RenderingGL2PSOpenGL2
  RenderingAnnotation
  FiltersSources
  InteractionStyle
  InteractionWidgets
  RenderingContextOpenGL2
  RenderingCore
  RenderingFreeType
  RenderingGL2PSOpenGL2
  RenderingOpenGL2
  GUISupportQt
  RenderingQt
)

find_package(PCL REQUIRED COMPONENTS common visualization)

if(NOT VTK_FOUND)
  message(FATAL_ERROR "Unable to find the VTK build folder.")
endif()

if(NOT(TARGET VTK::GUISupportQt))
  message(FATAL_ERROR "VTK not built with Qt support.")
endif()

if(NOT DEFINED VTK_QT_VERSION)
  set(VTK_QT_VERSION 6)
endif()

message("found VTK. Version:" ${VTK_VERSION}. VTK_DIR: ${VTK_DIR})


if(NOT PCL_FOUND)
	message(FATAL_ERROR "PCL not found.")
else()
	add_definitions(${PCL_DEFINITIONS})
	message("found PCL. Version:" ${PCL_VERSION}. PCL_DIR: ${PCL_DIR})
	# 输出 PCL 的头文件路径
	message("PCL include directories: ${PCL_INCLUDE_DIRS}")
	# 输出 PCL 的库文件路径
	message("PCL library directories: ${PCL_LIBRARY_DIRS}")
	# 引入PCL的头文件
	include_directories(${PCL_INCLUDE_DIRS})
	link_directories(${PCL_LIBRARY_DIRS})
endif()


set(TS_FILES "interactive/i18/DisplayModel_zh_CN.ts")

set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/interactive/ui)

add_subdirectory(io)

# 引入include目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/interactive/include)

# 引入src目录
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/interactive/src sources)

set(PROJECT_SOURCES
        main.cpp
        "interactive/src/widget/DisplayModel.cpp"
        "interactive/include/widget/DisplayModel.h"
        ${TS_FILES}
)


if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(DisplayModel
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET DisplayModel APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation

    qt_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})
else()
    if(ANDROID)
        add_library(DisplayModel SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(DisplayModel
            ${PROJECT_SOURCES}
        )
    endif()

    qt5_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})
endif()

target_link_libraries(DisplayModel PRIVATE io)
target_link_libraries(DisplayModel PRIVATE ${PCL_LIBRARIES})
target_link_libraries(DisplayModel PRIVATE ${VTK_LIBRARIES})
target_link_libraries(DisplayModel PRIVATE Qt${QT_VERSION_MAJOR}::Widgets Qt${QT_VERSION_MAJOR}::OpenGLWidgets Qt6::Concurrent)

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
if(${QT_VERSION} VERSION_LESS 6.1.0)
  set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.DisplayModel)
endif()
set_target_properties(DisplayModel PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

include(GNUInstallDirs)
install(TARGETS DisplayModel
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(DisplayModel)
endif()

vtk_module_autoinit(
  TARGETS DisplayModel
  MODULES ${VTK_LIBRARIES}
)