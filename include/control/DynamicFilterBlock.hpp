#pragma once

#include "DynamicBlock.hpp"
#include <cstdint>

namespace liteaerosim::control {

/// Abstract base for standalone SISO filter elements.
///
/// Extends DynamicBlock with filter-specific query and reset methods.
/// All concrete filters (FilterSS2, FilterTF, FilterFIR, …) derive from this.
class DynamicFilterBlock : public liteaerosim::DynamicBlock {
public:
    /// Number of poles (1 or 2 for standard designs; up to NUM_STATES for FIR/high-order).
    virtual uint8_t order() const = 0;

    /// DC gain of the discrete filter transfer function.
    virtual float dcGain() const = 0;

    /// Reset filter state assuming the input has been at in_val for a long time.
    virtual void resetToInput(float in_val) = 0;

    /// Reset filter state to produce output value out_val.
    /// If DC gain is zero the filter resets to zero regardless of out_val.
    virtual void resetToOutput(float out_val) = 0;

    /// Bitmask of active error flags (0 = no error).
    /// See liteaerosim::control::FilterError for bit definitions.
    virtual uint16_t errorCode() const = 0;
};

} // namespace liteaerosim::control
