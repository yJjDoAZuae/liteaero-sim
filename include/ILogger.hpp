#pragma once

#include <string_view>
#include <nlohmann/json.hpp>

namespace liteaerosim {

/// Logging sink interface injected into DynamicElement components.
/// Implementations write to CSV, binary ring buffer, in-memory store, etc.
/// Attach via DynamicElement::attachLogger(); pass nullptr to detach.
class ILogger {
public:
    virtual ~ILogger() = default;

    /// Log a scalar SI-unit value on the named channel.
    virtual void log(std::string_view channel, float value_si) = 0;

    /// Log a structured JSON snapshot on the named channel.
    virtual void log(std::string_view channel, const nlohmann::json& snapshot) = 0;
};

} // namespace liteaerosim
