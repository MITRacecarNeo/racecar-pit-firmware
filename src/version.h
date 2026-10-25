#pragma once
#include <stdint.h>

// Major and minor mirror racecar_neo_ros2_driver; see README "Versioning".
namespace fw
{
    constexpr uint8_t version_major = 0;
    constexpr uint8_t version_minor = 9;
    constexpr uint8_t version_patch = 0;
}

// Set by scripts/build_flags.py on firmware builds.
#ifndef FW_GIT_HASH
#define FW_GIT_HASH unknown
#endif
#ifndef FW_GIT_DIRTY
#define FW_GIT_DIRTY 0
#endif

#define FW_STR_(x) #x
#define FW_STR(x) FW_STR_(x)
