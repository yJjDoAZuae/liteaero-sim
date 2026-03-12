#pragma once

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace liteaerosim::logger {

/// Post-session reader for MCAP log files produced by Logger.
///
/// Lifecycle:
///   open(path) → [sources()] → [next() × N] → close()
///   exportCsv() may be called after open() and before close().
class LogReader {
public:
    LogReader();
    ~LogReader();

    /// Open an MCAP file for reading. Throws std::runtime_error on failure.
    void open(const std::filesystem::path& path);

    /// Describes one registered log source (channel metadata).
    struct SourceDescriptor {
        uint16_t                 channel_id;
        std::string              name;
        std::vector<std::string> channel_names;
        std::vector<std::string> channel_units;
    };

    /// Returns all source descriptors found in the file.
    /// Valid after open(); empty before open() or after close().
    const std::vector<SourceDescriptor>& sources() const;

    /// A single decoded log record.
    struct Record {
        uint16_t           channel_id;
        double             time_s;
        std::vector<float> values;
    };

    /// Advance to the next record. Returns false when exhausted.
    bool next(Record& out);

    /// Export all records to a CSV file (re-reads from the beginning).
    /// Format: time_s,source,channel_0,channel_1,...
    void exportCsv(const std::filesystem::path& path) const;

    void close();

private:
    struct Impl;
    std::unique_ptr<Impl> _impl;
};

}  // namespace liteaerosim::logger
