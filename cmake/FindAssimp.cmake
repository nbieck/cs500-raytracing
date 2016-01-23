#Locate Assimp library headers and libs
#assumes installation on linux using apt-get (standard locations)
#
#Will set ASSIMP_FOUND if it managed to locate the library
#
# Assimp_INCLUDE_DIR 
# path to include files
#
# Assimp_LIB
# path to assimp library

SET(_ASSIMP_REQUIRED_VARS Assimp_INCLUDE_DIR Assimp_LIB)

IF (UNIX)
    FIND_LIBRARY(Assimp_LIB
        assimp
        PATHS /usr/lib)
        
    FIND_PATH(Assimp_INCLUDE_DIR
        assimp/types.h
        PATHS /usr/include)
ENDIF()

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Assimp REQUIRED_VARS ${_ASSIMP_REQUIRED_VARS})
UNSET(_ASSIMP_REQUIRED_VARS)

MARK_AS_ADVANCED(Assimp_LIB
    Assimp_INCLUDE_DIR)
