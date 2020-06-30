cmake_minimum_required (VERSION 3.0.0)
project (OpenDiscon)

# choose a configuration
set( CONFIGURATION "MOORINGSENSE" CACHE STRING
"Choose a configuration.\n\
Available configurations are:\n\
CL-Windcon\n\
MOORINGSENSE" )

# run configuration-specific cmake script
include( ${PROJECT_SOURCE_DIR}/CONFIGURATION/${CONFIGURATION}/CMakeLists.txt )
