// Instantiates the MCAP library implementation in this translation unit only.
// mcap_static.hpp resets MCAP_PUBLIC/MCAP_EXPORT/MCAP_IMPORT to empty so that
// mcap can be statically linked on Windows without dllexport/dllimport annotations.
#define MCAP_IMPLEMENTATION
#include "mcap_static.hpp"
