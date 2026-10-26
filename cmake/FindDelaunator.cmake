# Find the pinned, header-only Delaunator implementation.
# DELAUNATOR_DIR may point to a pre-populated source or include directory.
# Otherwise fetch it into the shared third-party cache.

include("${CMAKE_CURRENT_LIST_DIR}/FetchThirdParty.cmake")

set(DELAUNATOR_DIR "" CACHE PATH "Delaunator source or include directory")
set(_delaunator_commit "f9ded0bb1457d62f36549b3a02a64aead7299553")
set(_delaunator_header "src/libInterpolate/Utils/Meshing/delaunator-cpp.hpp")
set(_delaunator_sha256 "4c9d651d279e199ce6d129afefbae01661152b9f381d24a7a3ce13897f600f51")

if(DELAUNATOR_INCLUDE_DIR AND NOT EXISTS "${DELAUNATOR_INCLUDE_DIR}/delaunator-cpp.hpp")
    unset(DELAUNATOR_INCLUDE_DIR CACHE)
    unset(DELAUNATOR_INCLUDE_DIR)
endif()

find_path(DELAUNATOR_INCLUDE_DIR NAMES delaunator-cpp.hpp
    HINTS
        "${DELAUNATOR_DIR}"
        "$ENV{DELAUNATOR_DIR}"
        "${_FETCH_TP_CACHE_ROOT}/delaunator"
    PATH_SUFFIXES include src/libInterpolate/Utils/Meshing
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH
)

if(NOT DELAUNATOR_INCLUDE_DIR)
    fetch_thirdparty(
        NAME delaunator
        GIT_REPO "https://github.com/CD3/libInterpolate.git"
        GIT_COMMIT "${_delaunator_commit}"
        SUBDIR "${_delaunator_header}"
        OUT_SOURCE_DIR _delaunator_header_path
    )
    get_filename_component(_delaunator_include_dir "${_delaunator_header_path}" DIRECTORY)
    set(DELAUNATOR_INCLUDE_DIR "${_delaunator_include_dir}" CACHE PATH
        "Delaunator include directory" FORCE)
endif()

file(SHA256 "${DELAUNATOR_INCLUDE_DIR}/delaunator-cpp.hpp" _delaunator_actual_sha256)
if(NOT _delaunator_actual_sha256 STREQUAL _delaunator_sha256)
    message(FATAL_ERROR
        "Delaunator header checksum mismatch at ${DELAUNATOR_INCLUDE_DIR}. "
        "Use the unmodified header from CD3/libInterpolate commit ${_delaunator_commit}.")
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Delaunator REQUIRED_VARS DELAUNATOR_INCLUDE_DIR)

if(Delaunator_FOUND AND NOT TARGET delaunator)
    add_library(delaunator INTERFACE)
    target_include_directories(delaunator SYSTEM INTERFACE "${DELAUNATOR_INCLUDE_DIR}")
endif()

mark_as_advanced(DELAUNATOR_INCLUDE_DIR)
