#pragma once

#include "ILogger.hpp"
#include <nlohmann/json.hpp>

namespace liteaerosim {

/// Abstract base for all stateful, time-evolving simulation components.
///
/// Lifecycle (in order):
///   initialize(config) — one-time setup from JSON config
///   reset()            — return to post-initialize state; may be called
///                        between runs without re-reading config
///   step(...)  × N     — advance one timestep; signature defined by each
///                        concrete class; not declared on this base
///   serializeJson() / deserializeJson() — checkpoint at any point after initialize()
///
/// initialize() and deserializeJson() validate "schema_version" in the JSON
/// object before forwarding to the protected hook; throw std::runtime_error on
/// mismatch.
///
/// Proto serialization is not declared on this base because proto message types
/// are component-specific. Each concrete class declares serializeProto() /
/// deserializeProto() with the appropriate message type.
class DynamicElement {
public:
    virtual ~DynamicElement() = default;

    /// Parse parameters and configure internal structure from JSON config.
    /// Validates schema_version before forwarding to onInitialize().
    /// Must be called once before reset() or step().
    void initialize(const nlohmann::json& config);

    /// Restore element to initial post-initialize conditions.
    virtual void reset();

    /// Return a complete SI-unit JSON snapshot of internal state.
    /// Base injects "schema_version" and "type".
    [[nodiscard]] nlohmann::json serializeJson() const;

    /// Restore internal state from a snapshot produced by serializeJson().
    /// Validates schema_version before forwarding to onDeserializeJson().
    /// Throws std::runtime_error if schema_version does not match.
    void deserializeJson(const nlohmann::json& state);

    /// Attach a logger. Pass nullptr to detach.
    /// The logger is available to onLog() calls after each step.
    void attachLogger(ILogger* logger) noexcept;

protected:
    virtual void           onInitialize(const nlohmann::json& config) = 0;
    virtual void           onReset()                                   = 0;
    virtual nlohmann::json onSerializeJson()                   const   = 0;
    virtual void           onDeserializeJson(const nlohmann::json&)    = 0;

    /// Called after each step() when a logger is attached. Default: no-op.
    virtual void onLog(ILogger& /*logger*/) const {}

    /// Schema version for this subclass. Increment when serialized fields change.
    virtual int schemaVersion() const = 0;

    /// Human-readable type name injected into every serialized snapshot.
    virtual const char* typeName() const = 0;

    ILogger* logger_ = nullptr;

private:
    void validateSchema(const nlohmann::json& state) const;
};

} // namespace liteaerosim
