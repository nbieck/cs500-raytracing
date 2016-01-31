CMAKE_MINIMUM_REQUIRED(VERSION 3.2)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

PROJECT(CS500_Raytracer)

SET(SOURCE_DIR ${CMAKE_SOURCE_DIR}/src)
SET(LIB_DIR ${CMAKE_SOURCE_DIR}/libs)
SET(TESTS_DIR ${SOURCE_DIR}/test)

SET(SOURCE_FILES geom.cpp main.cpp raytrace.cpp readAssimpFile.cpp realtime.cpp rgbe.cpp)
SET(HEADER_FILES geom.h raytrace.h realtime.h rgbe.h ray_shapes.h material.h)

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

FIND_PACKAGE(X11 REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)

FIND_PACKAGE(GTest) #in preparation for setting up testing
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
    ${X11_LIBRARIES}
    ${OPENGL_gl_LIBRARY}
    -fopenmp)
TARGET_INCLUDE_DIRECTORIES(Raytracer
    PUBLIC
    ${EIGEN_INCLUDE_DIR}
    ${X11_INCLUDE_DIR}
    ${LIB_DIR}/glbinding
    ${LIB_DIR}/freeglut
    ${LIB_DIR}
    ${Assimp_INCLUDE_DIR}
    ${OPENGL_INCLUDE_DIR}
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

#SETUP TESTING ONLY IF GTEST EXISTS
IF (GTEST_FOUND)
    ENABLE_TESTING()

    ADD_EXECUTABLE(Tests ${TESTS_DIR}/temp.cpp)
    TARGET_LINK_LIBRARIES(Tests
        PUBLIC
        ${GTEST_MAIN_LIBRARY}
        ${GTEST_LIBRARY}
        pthread)
    TARGET_INCLUDE_DIRECTORIES(Tests
        PUBLIC
        ${GTEST_INCLUDE_PATH})

    ADD_TEST(TestSuite Tests)
ENDIF()
