#pragma once
// mcap/visibility.hpp maps MCAP_PUBLIC to dllimport/dllexport on Windows, which is
// incompatible with static linking (the linker looks for __imp_ symbols that a static
// library never provides).  This wrapper includes visibility.hpp first so that
// #pragma once fires for it, then resets all three macros to empty before the class
// and function declarations in the remaining mcap headers are processed.
//
// All logger TUs must include this file instead of <mcap/mcap.hpp> directly.
// mcap_impl.cpp must #define MCAP_IMPLEMENTATION before including this file.
#include <cstdint>
#include <mcap/visibility.hpp>
#undef MCAP_PUBLIC
#define MCAP_PUBLIC
#undef MCAP_EXPORT
#define MCAP_EXPORT
#undef MCAP_IMPORT
#define MCAP_IMPORT
#include <mcap/mcap.hpp>
