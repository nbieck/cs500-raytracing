#This will "find" the gtest library included in the repository
#if the find fails, make sure to build it first
#(only gtest, run cmake with -DBUILD_GTEST=ON -DBUILD_GMOCK=OFF)
#
# This package will set the following variables:
#
#   GTEST_FOUND
#   GTEST_LIBRARY
#   GTEST_MAIN_LIBRARY
#   GTEST_INCLUDE_PATH

SET(_GTEST_REQUIRED_VARS GTEST_LIBRARY GTEST_MAIN_LIBRARY GTEST_INCLUDE_PATH)

FIND_PATH(GTEST_INCLUDE_PATH
    gtest/gtest.h
    PATHS ${CMAKE_SOURCE_DIR}/libs/gtest/googletest/include)

FIND_LIBRARY(GTEST_LIBRARY
    gtest
    PATHS ${CMAKE_SOURCE_DIR}/libs/gtest/build/googletest)

FIND_LIBRARY(GTEST_MAIN_LIBRARY
    gtest_main
    PATHS ${CMAKE_SOURCE_DIR}/libs/gtest/build/googletest)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GTest REQUIRED_VARS ${_GTEST_REQUIRED_VARS})
UNSET(_GTEST_REQUIRED_VARS)

MARK_AS_ADVANCED(GTEST_LIBRARY
    GTEST_MAIN_LIBRARY
    GTEST_INCLUDE_PATH)
