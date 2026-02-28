#pragma once

namespace liteaerosim {

/// Minimal composable SISO building block.
///
/// Tier 1 of the two-tier dynamic element hierarchy.
/// Use directly for composable primitives embedded inside a larger element
/// (e.g. a Limit embedded inside SISOPIDFF).
/// Use DynamicBlock (Tier 2) for standalone elements that need their own
/// lifecycle (initialize / reset / serialize / deserialize).
class SISOBlock {
public:
    SISOBlock() = default;
    virtual ~SISOBlock() = default;

    [[nodiscard]] virtual float in()  const = 0;
    [[nodiscard]] virtual float out() const = 0;
    virtual operator float()          const = 0;

    virtual float step(float u) = 0;
};

} // namespace liteaerosim
