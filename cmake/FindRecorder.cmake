# This file was copied from Eigen.
# Eigen is primarily MPL2 licensed. See COPYING.MPL2 and these links:
#   http://www.mozilla.org/MPL/2.0/
#     http://www.mozilla.org/MPL/2.0/FAQ.html
if (RECORDER_INCLUDES AND RECORDER_LIBRARIES)
    set(RECORDER_FIND_QUIETLY TRUE)
endif (RECORDER_INCLUDES AND RECORDER_LIBRARIES)

set(RECORDER_DIR $ENV{RECORDER_DIR} CACHE PATH
    "Path to RECORDER install directory.")

find_path(RECORDER_INCLUDES
        NAMES
        recorder.hpp
        PATHS
        "${RECORDER_DIR}/include"
)

find_library(RECORDER_LIBRARIES recorder PATHS
        "${RECORDER_DIR}/lib"
        )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(RECORDER DEFAULT_MSG
                                  RECORDER_INCLUDES RECORDER_LIBRARIES)

mark_as_advanced(RECORDER_INCLUDES RECORDER_LIBRARIES)
