CMAKE_MINIMUM_REQUIRED(VERSION 3.2)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

PROJECT(CS500_Raytracer)

SET(SOURCE_DIR ${CMAKE_SOURCE_DIR}/src)
SET(LIB_DIR ${CMAKE_SOURCE_DIR}/libs)

SET(SOURCE_FILES geom.cpp main.cpp raytrace.cpp readAssimpFile.cpp realtime.cpp rgbe.cpp)
SET(HEADER_FILES geom.h raytrace.h realtime.h rgbe.h)

#########################################################################
#find necessary libraries
#########################################################################
FIND_PACKAGE(Assimp REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)

FIND_LIBRARY(GLBINDING_LOC glbinding)
IF(NOT GLBINDING_LOC)
    MESSAGE(FATAL_ERROR "glbinding not found")
ENDIF()
MESSAGE("glbinding found at ${GLBINDING_LOC}")

FIND_LIBRARY(FREEGLUT_LOC freeglut PATHS "${CMAKE_SOURCE_DIR}/libs")
IF(NOT FREEGLUT_LOC)
    MESSAGE(FATAL_ERROR "freeglut not found")
ENDIF()
MESSAGE("freeglut found at ${FREEGLUT_LOC}")

FIND_LIBRARY(X11_LOC X11)
IF(NOT X11_LOC)
    MESSAGE(FATAL_ERROR "X11 not found")
ENDIF()
MESSAGE("X11 found at ${X11_LOC}")

FIND_PACKAGE(OpenGL REQUIRED)
#########################################################################

FOREACH(_FILE ${SOURCE_FILES})
    LIST(APPEND FULL_SOURCE ${SOURCE_DIR}/${_FILE})
ENDFOREACH()

FOREACH(_FILE ${HEADER_FILES})
    LIST(APPEND FULL_SOURCE ${SOURCE_DIR}/${_FILE})
ENDFOREACH()

ADD_EXECUTABLE(Raytracer ${FULL_SOURCE})
TARGET_LINK_LIBRARIES(Raytracer 
    ${Assimp_LIB} 
    ${GLBINDING_LOC} 
    ${FREEGLUT_LOC} 
    ${X11_LOC} 
    ${OPENGL_gl_LIBRARY}
    -fopenmp)
TARGET_INCLUDE_DIRECTORIES(Raytracer
    PUBLIC
    ${EIGEN_INCLUDE_DIR}
    ${LIB_DIR}/glbinding
    ${LIB_DIR}/freeglut
    ${LIB_DIR}
    ${Assimp_INCLUDE_DIR}
    ${SOURCE_DIR}
    )
TARGET_COMPILE_DEFINITIONS(Raytracer
    PUBLIC
    -DEIGEN_NO_DEBUG
    -DEIGEN_DONT_ALIGN_STATICALLY)
TARGET_COMPILE_OPTIONS(Raytracer
    PUBLIC
    -Wnarrowing -fopenmp -msse3 -Wno-deprecated-declarations -std=c++11 -g -O4 -Werror)
SET_TARGET_PROPERTIES(Raytracer PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})
