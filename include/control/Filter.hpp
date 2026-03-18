#pragma once

#include "SisoElement.hpp"
#include "control/control.hpp"
#include <Eigen/Dense>

namespace liteaerosim::control {

/// Abstract base for all discrete filter implementations.
///
/// Extends SisoElement with filter-specific query and reset methods.
/// All concrete filters (FilterSS2, FilterSS2Clip, FilterTF, FilterTF2,
/// FilterFIR) derive from this class.
///
/// Tier-1 filters (FilterSS2Clip, FilterTF, FilterTF2, FilterFIR) override
/// step() directly and provide their own in()/out() members. They inherit
/// default no-op lifecycle hooks from Filter and do not use the SisoElement
/// NVI step() mechanism.
///
/// FilterSS2 uses the full SisoElement lifecycle: it implements all lifecycle
/// hooks and uses onStep() through the NVI step() wrapper.
class Filter : public liteaerosim::SisoElement {
public:
    constexpr static char maxNumStates = liteaerosim::kFilterMaxStates;

    /// Number of filter poles.
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

protected:
    // -----------------------------------------------------------------------
    // Default lifecycle hooks — no-ops for Tier-1 filters that override
    // step() directly and do not use the DynamicElement lifecycle.
    // FilterSS2 overrides all of these with real implementations.
    // -----------------------------------------------------------------------
    void           onInitialize(const nlohmann::json&)     override {}
    nlohmann::json onSerializeJson()               const   override { return {}; }
    void           onDeserializeJson(const nlohmann::json&) override {}
    int            schemaVersion()                 const   override { return 0; }
    const char*    typeName()                      const   override { return "Filter"; }

    /// Default onStep for Tier-1 filters that override step() directly.
    /// FilterSS2 overrides this with the real state-space update.
    float          onStep(float u)                         override { return u; }
};

} // namespace liteaerosim::control
