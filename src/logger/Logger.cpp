#include "logger/Logger.hpp"

#include "liteaerosim.pb.h"
#include <google/protobuf/descriptor.h>
#include <google/protobuf/descriptor.pb.h>
#include "mcap_static.hpp"

#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace liteaerosim::logger {

// ---------------------------------------------------------------------------
// LogSourceState — shared between Logger::Impl and every LogSource handle.
// ---------------------------------------------------------------------------
struct LogSourceState {
    uint16_t                 channel_id{0};
    std::string              name;
    std::vector<std::string> channel_names;
    std::vector<std::string> channel_units;
    // Raw pointers into Logger::Impl — valid as long as the Logger lives.
    std::mutex*       write_mutex{nullptr};
    mcap::McapWriter* writer{nullptr};
    bool*             is_open_flag{nullptr};
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Build a serialized FileDescriptorSet for a proto message descriptor.
/// This is the schema data MCAP needs to embed for self-describing files.
static std::string buildFileDescriptorSet(const google::protobuf::Descriptor* descriptor) {
    google::protobuf::FileDescriptorSet fd_set;
    // Recursively collect all transitive file dependencies.
    std::function<void(const google::protobuf::FileDescriptor*)> collect;
    collect = [&](const google::protobuf::FileDescriptor* fd) {
        for (int i = 0; i < fd->dependency_count(); ++i) {
            collect(fd->dependency(i));
        }
        fd->CopyTo(fd_set.add_file());
    };
    collect(descriptor->file());
    return fd_set.SerializeAsString();
}

/// Join a vector of strings with a delimiter.
static std::string join(const std::vector<std::string>& v, char delim = ',') {
    std::string out;
    for (std::size_t i = 0; i < v.size(); ++i) {
        if (i) out += delim;
        out += v[i];
    }
    return out;
}

// ---------------------------------------------------------------------------
// Logger::Impl
// ---------------------------------------------------------------------------
struct Logger::Impl {
    mcap::McapWriter                             writer;
    std::mutex                                   mutex;
    bool                                         open_flag{false};
    mcap::SchemaId                               float_array_schema_id{0};
    std::vector<std::shared_ptr<LogSourceState>> sources;
};

// ---------------------------------------------------------------------------
// Logger
// ---------------------------------------------------------------------------
Logger::Logger() : _impl(std::make_unique<Impl>()) {}

Logger::~Logger() {
    if (_impl && _impl->open_flag) {
        close();
    }
}

void Logger::open(const std::filesystem::path& path, LogFormat /*format*/) {
    if (_impl->open_flag) {
        throw std::logic_error("Logger::open: session already open");
    }

    mcap::McapWriterOptions opts("liteaerosim");
    opts.compression = mcap::Compression::None;

    const auto status = _impl->writer.open(path.string(), opts);
    if (!status.ok()) {
        throw std::runtime_error("Logger::open: " + std::string(status.message));
    }

    // Register the FloatArray schema once; store its ID for addSource().
    const std::string schema_data =
        buildFileDescriptorSet(las_proto::FloatArray::descriptor());
    mcap::Schema schema("las_proto.FloatArray", "protobuf",
                        std::string_view(schema_data));
    _impl->writer.addSchema(schema);
    _impl->float_array_schema_id = schema.id;

    _impl->open_flag = true;
}

LogSource Logger::addSource(std::string name,
                             std::vector<std::string> channel_names,
                             std::vector<std::string> channel_units) {
    if (!_impl->open_flag) {
        throw std::logic_error("Logger::addSource: logger is not open");
    }

    mcap::Channel channel(name, "protobuf", _impl->float_array_schema_id);
    channel.metadata["channel_names"] = join(channel_names);
    channel.metadata["channel_units"] = join(channel_units);
    _impl->writer.addChannel(channel);

    auto state              = std::make_shared<LogSourceState>();
    state->channel_id       = channel.id;
    state->name             = name;
    state->channel_names    = std::move(channel_names);
    state->channel_units    = std::move(channel_units);
    state->write_mutex      = &_impl->mutex;
    state->writer           = &_impl->writer;
    state->is_open_flag     = &_impl->open_flag;

    _impl->sources.push_back(state);
    return LogSource(std::move(state));
}

void Logger::flush() {
    // McapWriter has no explicit flush() — writes are committed on chunk
    // rotation or close(). This method is a no-op for synchronous writes.
    (void)_impl;
}

void Logger::close() {
    std::lock_guard<std::mutex> lock(_impl->mutex);
    _impl->open_flag = false;
    _impl->writer.close();
}

bool Logger::is_open() const {
    return _impl->open_flag;
}

// ---------------------------------------------------------------------------
// LogSource
// ---------------------------------------------------------------------------
LogSource::LogSource(std::shared_ptr<LogSourceState> state)
    : _state(std::move(state)) {}

void LogSource::log(double time_s, const std::vector<float>& values) {
    if (!(*_state->is_open_flag)) {
        throw std::logic_error("LogSource::log: logger has been closed");
    }

    las_proto::FloatArray msg;
    for (float v : values) {
        msg.add_values(v);
    }
    const std::string buf = msg.SerializeAsString();

    mcap::Message mcap_msg;
    mcap_msg.channelId   = _state->channel_id;
    mcap_msg.logTime     = static_cast<uint64_t>(time_s * 1e9);
    mcap_msg.publishTime = mcap_msg.logTime;
    mcap_msg.data        = reinterpret_cast<const std::byte*>(buf.data());
    mcap_msg.dataSize    = buf.size();

    std::lock_guard<std::mutex> lock(*_state->write_mutex);
    const auto write_status = _state->writer->write(mcap_msg);
    if (!write_status.ok()) {
        throw std::runtime_error("LogSource::log: write failed: " +
                                 std::string(write_status.message));
    }
}

uint16_t LogSource::channel_id() const { return _state->channel_id; }
const std::string& LogSource::name() const { return _state->name; }
const std::vector<std::string>& LogSource::channel_names() const { return _state->channel_names; }
const std::vector<std::string>& LogSource::channel_units() const { return _state->channel_units; }

}  // namespace liteaerosim::logger
