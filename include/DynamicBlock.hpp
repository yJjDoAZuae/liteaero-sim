#pragma once

#include "SISOBlock.hpp"
#include "ILogger.hpp"
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>

namespace liteaerosim {

/// Abstract base for all standalone SISO simulation elements.
///
/// Lifecycle:
///   initialize(config) → reset() → step(u) ↔ serialize() / deserialize()
///
/// The timestep dt_s is a configuration parameter parsed in onInitialize().
/// It is fixed for the lifetime of the element. Elements that need it store
/// it as a private member and precompute discrete coefficients there.
///
/// Subclasses implement the protected on*() hooks only.
/// The public non-virtual methods enforce cross-cutting concerns once:
///   - in_ / out_ recording
///   - logging (if a logger is attached)
///   - schema version validation on deserialize
///   - type name and schema_version injection on serialize
class DynamicBlock : public SISOBlock {
public:
    // -----------------------------------------------------------------------
    // Lifecycle — non-virtual public API (NVI)
    // -----------------------------------------------------------------------

    /// Parse parameters and configure internal structure from JSON config.
    /// Must be called once before reset() or step().
    void initialize(const nlohmann::json& config);

    /// Restore element to initial post-initialize conditions.
    /// Zeros in_ and out_, then calls onReset().
    void reset();

    /// Advance internal state by one timestep. Returns the scalar output.
    /// Records in_ and out_, calls onStep(), then calls onLog() if a logger
    /// is attached.
    float step(float u) override;

    /// Return a complete SI-unit JSON snapshot of internal state.
    /// Base injects "schema_version", "type", "in", "out".
    /// Subclass provides remaining fields via onSerialize().
    [[nodiscard]] nlohmann::json serialize() const;

    /// Restore internal state from a snapshot produced by serialize().
    /// Throws std::runtime_error if schema_version does not match.
    void deserialize(const nlohmann::json& state);

    /// Attach a logger. Pass nullptr to detach.
    /// The logger is called at the end of every step() when attached.
    void attachLogger(ILogger* logger) noexcept;

    // -----------------------------------------------------------------------
    // I/O accessors — satisfy SISOBlock contract
    // -----------------------------------------------------------------------
    [[nodiscard]] float in()  const noexcept override { return in_; }
    [[nodiscard]] float out() const noexcept override { return out_; }
    operator float()          const noexcept override { return out_; }

protected:
    float in_  = 0.0f;
    float out_ = 0.0f;

    // -----------------------------------------------------------------------
    // Customisation hooks — implement in derived classes
    // -----------------------------------------------------------------------

    virtual void           onInitialize(const nlohmann::json& config) = 0;
    virtual void           onReset()                                   = 0;
    virtual float          onStep(float u)                             = 0;
    virtual nlohmann::json onSerialize()                         const = 0;
    virtual void           onDeserialize(const nlohmann::json& state)  = 0;

    /// Called at the end of step() when a logger is attached. Default: no-op.
    virtual void onLog(ILogger& /*logger*/) const {}

    /// Schema version for this subclass. Increment when serialized fields change.
    virtual int schemaVersion() const = 0;

    /// Human-readable type name injected into every serialized snapshot.
    /// Used for diagnostics and future polymorphic restore.
    /// Example implementation: return "FilterSS2";
    virtual const char* typeName() const = 0;

private:
    ILogger* logger_ = nullptr;

    void validateSchema(const nlohmann::json& state) const;
};

} // namespace liteaerosim
