﻿cmake_minimum_required(VERSION 3.27)
project(ply_io)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
set(BUILD_SHARED_LIBS ON)

# 引入include目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
# 引入src目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

 find_package(VTK COMPONENTS 
  CommonCore
  IOPLY
  IOGeometry
  IOChemistry
  RenderingGL2PSOpenGL2
)

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

# 导入VTK头文件
message("VTK root directory: ${VTK_DIR}")

# 输出 VTK 的头文件路径
message("VTK include directories: ${VTK_INCLUDE_DIRS}")

set(PROJECT_SOURCES
        "src/IO.cpp"
        "include/IO.h"
)

# 生成 submodule1 的 DLL 文件
add_library(io SHARED ${PROJECT_SOURCES})

target_link_libraries(io ${VTK_LIBRARIES})

target_include_directories(io PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# 设置生成的 DLL 文件的输出路径
set_target_properties(io PROPERTIES
		ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../out/build
		LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../out/build
		RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../out/build
)