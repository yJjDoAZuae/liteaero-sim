#pragma once

#include "aerodynamics/AircraftGeometry.hpp"
#include "aerodynamics/AeroPerformance.hpp"
#include "aerodynamics/LiftCurveModel.hpp"
#include <utility>

namespace liteaerosim::aerodynamics {

// Stateless utility that derives all trim-aero-model coefficients from major
// aircraft dimensional parameters. Implements the derivation chain defined in
// docs/algorithms/aerodynamics.md Parts 1–8.
//
// All inputs and outputs are in SI units. No I/O; no unit conversions.
// Throws std::invalid_argument if the geometry violates physical constraints
// (e.g. zero area, negative span).
class AeroCoeffEstimator {
public:
    // Estimate AeroPerformance and LiftCurveParams from aircraft geometry.
    // Returns {AeroPerformance, LiftCurveParams}.
    [[nodiscard]] static std::pair<AeroPerformance, LiftCurveParams>
        estimate(const AircraftGeometry& geom);
};

} // namespace liteaerosim::aerodynamics
