#pragma once
#include <stdint.h>

// Major and minor mirror racecar_neo_ros2_driver; see README "Versioning".
namespace fw
{
    constexpr uint8_t version_major = 0;
    constexpr uint8_t version_minor = 9;
    constexpr uint8_t version_patch = 2;
}

// Set by scripts/build_flags.py on firmware builds, in the generated
// build_info.h. FW_LABEL is the latest labeled commit on the branch
// ("[vM.m.p.N-pcb] ..."), or dev. Host builds have no header and use these.
#if __has_include("build_info.h")
#include "build_info.h"
#endif
#ifndef FW_LABEL
#define FW_LABEL dev
#endif
#ifndef FW_GIT_HASH
#define FW_GIT_HASH unknown
#endif
#ifndef FW_GIT_DIRTY
#define FW_GIT_DIRTY 0
#endif

#define FW_STR_(x) #x
#define FW_STR(x) FW_STR_(x)
