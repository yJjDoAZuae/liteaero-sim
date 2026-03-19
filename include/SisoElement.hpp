#pragma once

#include "DynamicElement.hpp"

namespace liteaerosim {

/// Abstract base for all single-input, single-output dynamic elements.
///
/// Derives from DynamicElement and adds the SISO step interface.
///
/// The public step(float u) is the NVI entry point:
///   - records in_ and out_
///   - calls onStep() for the subclass-defined update
///   - calls onLog() if a logger is attached
///
/// Subclasses implement onStep() and the lifecycle hooks from DynamicElement.
/// reset() zeros in_ and out_ before forwarding to onReset().
class SisoElement : public DynamicElement {
public:
    [[nodiscard]] float in()  const { return in_; }
    [[nodiscard]] float out() const { return out_; }
    operator float()          const { return out_; }

    /// Advance internal state by one timestep. Records in_ and out_,
    /// calls onStep(), then calls onLog() if a logger is attached.
    virtual float step(float u);

    // DynamicElement override — zeros in_/out_ before calling onReset()
    void reset() override;

protected:
    float in_  = 0.0f;
    float out_ = 0.0f;

    /// Compute and return the scalar output for input u.
    /// Called from step(). Update any internal state here.
    virtual float onStep(float u) = 0;

    /// Called from reset() after zeroing in_ and out_.
    /// Override to zero any additional subclass state.
    void onReset() override {}
};

} // namespace liteaerosim
