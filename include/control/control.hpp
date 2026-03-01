#pragma once

#include "numerics.hpp"

namespace liteaerosim::control {

enum FilterError : uint16_t {
    NONE               = 0,
    INVALID_DIMENSION  = 1,
    INVALID_TIMESTEP   = 2,
    UNSTABLE           = 4,
    INFINITE_DC_GAIN   = 8,
    ZERO_DC_GAIN       = 16,
    INVALID_POLYNOMIAL = 32,
};

enum DiscretizationMethod {
    FwdEuler  = 0,
    BackEuler = 1,
    Bilinear  = 2,
    Prewarp   = 3,
    PZMatch   = 4,
};

}  // namespace liteaerosim::control
