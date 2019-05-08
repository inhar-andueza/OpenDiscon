cmake_minimum_required (VERSION 3.5.1)
project (FarmControl)

# FarmControl include directories
#set (FARMCONTROL_INCLUDE_DIRS ${FARMCONTROL_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/src/FCDiscon/)
#set (FARMCONTROL_INCLUDE_DIRS ${FARMCONTROL_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/src/FarmControl/)

# FarmControl source files
#set (FARMCONTROL_SOURCES ${FARMCONTROL_SOURCES} ${PROJECT_SOURCE_DIR}/src/FCDiscon/FCDiscon.c)
#set (FARMCONTROL_SOURCES ${FARMCONTROL_SOURCES} ${PROJECT_SOURCE_DIR}/src/FarmControl/FarmControl.c)

# shared FCDiscon library
#include_directories (${FARMCONTROL_INCLUDE_DIRS})
#include_directories (${PROJECT_BINARY_DIR})
#include (GenerateExportHeader)
#add_library (FarmControl SHARED ${FARMCONTROL_SOURCES})

#include_directories (${CMAKE_SOURCE_DIR}/src/FCDiscon/
#		     ${CMAKE_SOURCE_DIR}/src/FarmControl/)
#include (GenerateExportHeader)
#add_library (FarmControl SHARED ${CMAKE_SOURCE_DIR}/src/FCDiscon/FCDiscon.c
#		     		${CMAKE_SOURCE_DIR}/src/FarmControl/FarmControl.c)

#GENERATE_EXPORT_HEADER (FarmControl
#	BASE_NAME FarmControl
#	EXPORT_MACRO_NAME FarmControl_EXPORT
#	EXPORT_FILE_NAME FarmControl_EXPORT.h
#	STATIC_DEFINE FarmControl_BUILT_AS_STATIC
#)

set (C_SOURCE FCDiscon.c)
add_library(FarmControl SHARED ${C_SOURCE} )

