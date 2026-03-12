#include "logger/LogReader.hpp"

#include "liteaerosim.pb.h"
#include "mcap_static.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace liteaerosim::logger {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Split a comma-delimited string into a vector of strings.
static std::vector<std::string> splitComma(const std::string& s) {
    std::vector<std::string> out;
    if (s.empty()) return out;
    std::istringstream ss(s);
    std::string token;
    while (std::getline(ss, token, ',')) {
        out.push_back(token);
    }
    return out;
}

// ---------------------------------------------------------------------------
// LogReader::Impl
// All records are decoded into memory during open() so that:
//  (a) we avoid storing the mcap iterator (its default ctor is private), and
//  (b) exportCsv() can iterate without reopening the file.
// ---------------------------------------------------------------------------
struct LogReader::Impl {
    std::vector<SourceDescriptor> sources;
    std::filesystem::path         path;
    bool                          open_flag{false};

    // Records buffered during open().
    std::vector<Record>           records;
    std::size_t                   read_index{0};
};

// ---------------------------------------------------------------------------
// LogReader
// ---------------------------------------------------------------------------
LogReader::LogReader() : _impl(std::make_unique<Impl>()) {}

LogReader::~LogReader() {
    // Nothing to explicitly close for the buffered approach.
}

void LogReader::open(const std::filesystem::path& path) {
    _impl->path = path;
    _impl->sources.clear();
    _impl->records.clear();
    _impl->read_index = 0;

    mcap::McapReader reader;
    const auto open_status = reader.open(path.string());
    if (!open_status.ok()) {
        throw std::runtime_error("LogReader::open: " +
                                 std::string(open_status.message));
    }

    const auto sum_status =
        reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);
    if (!sum_status.ok()) {
        throw std::runtime_error("LogReader::open (readSummary): " +
                                 std::string(sum_status.message));
    }

    // Build SourceDescriptors from channel metadata.
    std::unordered_map<uint16_t, std::size_t> chan_to_sd_idx;
    for (const auto& [id, chan_ptr] : reader.channels()) {
        SourceDescriptor sd;
        sd.channel_id = static_cast<uint16_t>(chan_ptr->id);
        sd.name       = chan_ptr->topic;
        const auto& meta = chan_ptr->metadata;
        {
            auto it = meta.find("channel_names");
            if (it != meta.end()) sd.channel_names = splitComma(it->second);
        }
        {
            auto it = meta.find("channel_units");
            if (it != meta.end()) sd.channel_units = splitComma(it->second);
        }
        chan_to_sd_idx[sd.channel_id] = _impl->sources.size();
        _impl->sources.push_back(std::move(sd));
    }

    // Decode all messages into the record buffer.
    auto view = reader.readMessages();
    for (auto it = view.begin(); it != view.end(); ++it) {
        const auto& msg = it->message;

        las_proto::FloatArray arr;
        if (!arr.ParseFromArray(static_cast<const void*>(msg.data),
                                static_cast<int>(msg.dataSize))) {
            continue;  // skip malformed records
        }

        Record r;
        r.channel_id = static_cast<uint16_t>(msg.channelId);
        r.time_s     = static_cast<double>(msg.logTime) * 1e-9;
        r.values.resize(static_cast<std::size_t>(arr.values_size()));
        for (int i = 0; i < arr.values_size(); ++i) {
            r.values[static_cast<std::size_t>(i)] = arr.values(i);
        }
        _impl->records.push_back(std::move(r));
    }

    reader.close();
    _impl->open_flag = true;
}

const std::vector<LogReader::SourceDescriptor>& LogReader::sources() const {
    return _impl->sources;
}

bool LogReader::next(Record& out) {
    if (!_impl->open_flag || _impl->read_index >= _impl->records.size()) {
        return false;
    }
    out = _impl->records[_impl->read_index++];
    return true;
}

void LogReader::exportCsv(const std::filesystem::path& csv_path) const {
    if (!_impl->open_flag) {
        throw std::logic_error("LogReader::exportCsv: reader is not open");
    }

    // Build channel_id → source name map.
    std::unordered_map<uint16_t, std::string> id_to_name;
    for (const auto& sd : _impl->sources) {
        id_to_name[sd.channel_id] = sd.name;
    }

    std::ofstream csv(csv_path);
    if (!csv.is_open()) {
        throw std::runtime_error("LogReader::exportCsv: cannot open " +
                                 csv_path.string());
    }
    csv << "time_s,source";
    for (const auto& r : _impl->records) {
        const auto& name = id_to_name.count(r.channel_id)
                               ? id_to_name.at(r.channel_id)
                               : "";
        csv << "\n" << r.time_s << "," << name;
        for (float v : r.values) {
            csv << "," << v;
        }
    }
}

void LogReader::close() {
    _impl->sources.clear();
    _impl->records.clear();
    _impl->read_index = 0;
    _impl->open_flag  = false;
}

}  // namespace liteaerosim::logger
