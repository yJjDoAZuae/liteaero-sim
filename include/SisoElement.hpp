#pragma once

#include "DynamicElement.hpp"
#include "SISOBlock.hpp"

namespace liteaerosim {

/// Fully-capable SISO dynamic element.
///
/// Merges the lifecycle contract of DynamicElement with the SISO step
/// interface of SISOBlock. Use for standalone control elements that need
/// their own lifecycle (initialize / reset / serialize / deserialize).
///
/// The public step(float u) is the NVI entry point:
///   - records in_ and out_
///   - calls onStep() for the subclass-defined update
///   - calls onLog() if a logger is attached
///
/// Subclasses implement onStep() and the lifecycle hooks from DynamicElement.
/// reset() zeros in_ and out_ before forwarding to onReset().
class SisoElement : public DynamicElement, public SISOBlock {
public:
    // -----------------------------------------------------------------------
    // SISOBlock interface — implemented here; satisfy the pure-virtual contract
    // -----------------------------------------------------------------------
    [[nodiscard]] float in()  const override { return in_; }
    [[nodiscard]] float out() const override { return out_; }
    operator float()          const override { return out_; }

    /// Advance internal state by one timestep. Records in_ and out_,
    /// calls onStep(), then calls onLog() if a logger is attached.
    float step(float u) override;

    // -----------------------------------------------------------------------
    // DynamicElement override — zeros in_/out_ before calling onReset()
    // -----------------------------------------------------------------------
    void reset() override;

protected:
    float in_  = 0.0f;
    float out_ = 0.0f;

    // -----------------------------------------------------------------------
    // Customization hooks
    // -----------------------------------------------------------------------

    /// Compute and return the scalar output for input u.
    /// Called from step(). Update any internal state here.
    virtual float onStep(float u) = 0;

    /// Called from reset() after zeroing in_ and out_.
    /// Override to zero any additional subclass state.
    void onReset() override {}
};

} // namespace liteaerosim
