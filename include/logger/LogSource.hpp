#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace liteaerosim::logger {

// Forward declaration — full definition lives in Logger.cpp.
struct LogSourceState;

/// Handle to a registered log source returned by Logger::addSource().
/// Lightweight value type; safe to copy. The underlying state is shared with
/// the Logger that created it. Calling log() after Logger::close() throws.
class LogSource {
public:
    /// Write a timestamped float-vector record. Thread-safe, non-blocking.
    /// values.size() must equal channel_names().size() passed to addSource().
    /// Throws std::logic_error if the parent Logger has been closed.
    void log(double time_s, const std::vector<float>& values);

    uint16_t                        channel_id()    const;
    const std::string&              name()          const;
    const std::vector<std::string>& channel_names() const;
    const std::vector<std::string>& channel_units() const;

private:
    friend class Logger;
    explicit LogSource(std::shared_ptr<LogSourceState> state);
    std::shared_ptr<LogSourceState> _state;
};

}  // namespace liteaerosim::logger
