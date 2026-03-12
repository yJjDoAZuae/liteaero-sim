#pragma once

#include "logger/LogSource.hpp"
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace liteaerosim::logger {

enum class LogFormat { Mcap, Csv };

/// Session manager for telemetry logging.
///
/// Lifecycle:
///   open(path) → addSource() × N → [log() per source per timestep] → close()
///
/// All writes are synchronous and mutex-protected. The Logger is not a
/// DynamicBlock; it lives in the Infrastructure layer and has no physics logic.
class Logger {
public:
    Logger();
    ~Logger();  // calls close() if open

    /// Open a new log session. Throws std::runtime_error on I/O failure.
    /// Must be called before addSource() or flush().
    void open(const std::filesystem::path& path,
              LogFormat format = LogFormat::Mcap);

    /// Register a float-array source. Returns a handle for logging.
    /// Must be called before the first log() call on the returned source.
    /// Throws std::logic_error if the Logger is not open.
    LogSource addSource(std::string name,
                        std::vector<std::string> channel_names,
                        std::vector<std::string> channel_units = {});

    /// Flush all buffered data to disk synchronously.
    void flush();

    /// Close the session, finalize the file, and release I/O resources.
    void close();

    bool is_open() const;

private:
    struct Impl;
    std::unique_ptr<Impl> _impl;
};

}  // namespace liteaerosim::logger
