#Finds Eigen (eigen3), assumed to be installed 
#In the default place on unix
#
# Sets the following variables
# `EIGEN_FOUND`
#   Was the library successfully found?
# `EIGEN_INCLUDE_DIR`
#   The base Eigen include directory
#

SET(_EIGEN_REQUIRED_VARS EIGEN_INCLUDE_DIR)

IF (UNIX)
    FIND_PATH(EIGEN_INCLUDE_DIR
        signature_of_eigen3_matrix_library
        PATHS /usr/include/eigen3)
ENDIF()

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Eigen REQUIRED_VARS ${_EIGEN_REQUIRED_VARS})
UNSET(_EIGEN_REQUIRED_VARS)

MARK_AS_ADVANCED(EIGEN_INCLUDE_DIR)
