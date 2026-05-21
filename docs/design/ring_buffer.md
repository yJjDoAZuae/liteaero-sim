# Ring Buffer and Channel Registry — Architecture

Design authority for the in-process ring buffer and channel registry that carries
real-time simulation output from `SimRunner` (C++ simulation thread) to Python
visualization subscribers (rendering timer thread).

Implemented by roadmap item SB-2. Decisions DR-5 and DR-6 in
[`post_processing.md`](post_processing.md) are reproduced here in resolved form;
see that document for the full alternatives analysis.

---

## Purpose

`SimRunner` runs the simulation loop at the configured `dt_s` rate. Python
visualization components (`LiveSimView`, `LiveTimeHistoryFigure`) need to consume
simulation output in real time without blocking the simulation or waiting for a
log file to be written. The ring buffer provides an in-process, low-overhead
channel from C++ to Python with independent draining per subscriber.

---

## Decisions (from DR-5 and DR-6)

| Decision | Resolution |
| --- | --- |
| Transport mechanism | Custom in-process ring buffer (DR-6 Option B — no ROS2 dependency) |
| Subscriber attachment | Polling: Python calls `drain()` on a timer (DR-5a) |
| Per-channel type | Per-channel scalar `float` (DR-5b); `double` used only for timestamps |
| Channel registry | Registry object owned by `SimRunner`; exposed to Python via pybind11 reference (DR-5c) |
| Buffer depth | Configurable depth in seconds, converted to samples at registration (DR-5d) |
| Access mechanism | pybind11 binding to the C++ registry object — in-process, no IPC (DR-5e) |
| Buffer model | Per-subscriber circular buffer — independent storage per subscriber |
| Late-join policy | No backfill; subscriber starts with empty buffer (PP-F37) |
| Channel registration | Producer-driven; buffer storage allocated only when subscriber attaches (PP-F34) |

---

## Classes

### `Sample`

Plain value struct. The unit of data returned by `drain()`.

```cpp
struct Sample {
    double time_s;  // simulation time of this sample (s)
    float  value;   // channel value in SI units
};
```

### `ChannelSubscriber`

RAII subscription token returned by `ChannelRegistry::subscribe()`. Owns a
circular buffer that is written by `SimRunner` (via `ChannelRegistry::publish()`)
and drained by the Python caller. Non-copyable.

#### API

```cpp
class ChannelSubscriber {
public:
    ~ChannelSubscriber();                       // RAII: removes self from channel
    const std::string& channel_name() const;
    std::vector<Sample> drain();                // returns all buffered samples, resets buffer
};
```

`drain()` is safe to call from any thread. It takes the subscriber's per-instance
mutex, copies all buffered samples in chronological order, resets the buffer to
empty, and returns. Calling `drain()` while no samples have arrived returns an
empty vector.

The circular buffer silently overwrites the oldest sample when it is full (overflow
policy). Consumers that poll infrequently relative to the channel sample rate may
lose old samples but never miss recent ones.

### `ChannelRegistry`

Owns the channel metadata and the subscriber lists. Held as a member of
`SimRunner`. Python code receives a reference via `SimRunner::channel_registry()`.

#### API

```cpp
class ChannelRegistry {
public:
    // Register a channel. Idempotent: calling again with the same name is a no-op.
    // sample_rate_hz: expected production rate (used to compute buffer capacity).
    // depth_s:        seconds of history each subscriber buffer retains.
    void register_channel(const std::string& name,
                          float sample_rate_hz,
                          float depth_s);

    // List all registered channel names (regardless of subscriber count).
    std::vector<std::string> available_channels() const;

    // Subscribe to a registered channel. Returns a shared_ptr<ChannelSubscriber>.
    // The subscriber's buffer is empty at creation (no backfill, PP-F37).
    // Throws std::out_of_range if the channel has not been registered.
    std::shared_ptr<ChannelSubscriber> subscribe(const std::string& name);

    // Publish a sample. Called by SimRunner from the simulation thread.
    // No-op if the channel has no active subscribers.
    void publish(const std::string& name, double time_s, float value);
};
```

---

## Threading Contract

`SimRunner` calls `publish()` from the simulation thread. Python callers call
`drain()` from arbitrary threads (Qt timer, render loop). The following invariants
hold:

- **Lock ordering:** registry mutex → subscriber mutex. Code that takes both
  mutexes always acquires the registry mutex first.
- **`publish()`** takes the registry mutex to access the subscriber list, then
  takes each subscriber's mutex to write. Total hold time: O(N × 1 write)
  where N is the subscriber count for that channel (typically ≤ 2).
- **`drain()`** takes only the subscriber's own mutex. It never touches the
  registry mutex. No contention with other subscribers draining different channels.
- **`~ChannelSubscriber()`** calls `registry_->unsubscribe(name, this)`. This
  takes the registry mutex to remove the subscriber from the channel's list.
  The subscriber mutex is not taken inside `unsubscribe()`.

**No deadlock is possible** under this lock ordering: `drain()` holds subscriber
mutex only (never escalates to registry mutex); `publish()` holds registry mutex
then subscriber mutex; `unsubscribe()` holds registry mutex only. Circular
wait cannot occur.

---

## Buffer Internals

Each `ChannelSubscriber` holds a circular buffer of capacity
`ceil(sample_rate_hz × depth_s)` samples (minimum 2).

```
write:  buffer_[head_] = {time_s, value};
        head_ = (head_ + 1) % capacity_;
        if (count_ < capacity_) ++count_;
        // If count_ == capacity_, oldest sample is silently overwritten.

drain:  read_pos = (head_ + capacity_ - count_) % capacity_;
        copy count_ samples starting at read_pos (wrapping);
        head_ = 0; count_ = 0;
        return copied samples.
```

---

## `SimRunner` Integration

`SimRunner` owns a `ChannelRegistry` member. In `initialize()`, it registers the
14 kinematic state channels listed below at `1/dt_s` Hz with a 60-second depth.
In `runLoop()`, after each `aircraft_->step()`, it calls `publish()` for each
channel — these are no-ops when no subscriber is attached (PP-F34).

### Registered Channels

All channels use the `"source/field_si_unit"` naming convention consistent with the
MCAP per-field topic schema (DR-9 in `post_processing.md`). All values are SI.

| Channel name | Source accessor | Units |
| --- | --- | --- |
| `kinematic/time_s` | `state().time_sec()` | s |
| `kinematic/latitude_rad` | `positionDatum().latitudeGeodetic_rad()` | rad |
| `kinematic/longitude_rad` | `positionDatum().longitude_rad()` | rad |
| `kinematic/altitude_m` | `positionDatum().height_WGS84_m()` | m |
| `kinematic/velocity_north_mps` | `velocity_NED_mps()(0)` | m/s |
| `kinematic/velocity_east_mps` | `velocity_NED_mps()(1)` | m/s |
| `kinematic/velocity_down_mps` | `velocity_NED_mps()(2)` | m/s |
| `kinematic/heading_rad` | `heading()` | rad |
| `kinematic/pitch_rad` | `pitch()` | rad |
| `kinematic/roll_rad` | `roll()` | rad |
| `kinematic/alpha_rad` | `alpha()` | rad |
| `kinematic/beta_rad` | `beta()` | rad |
| `kinematic/airspeed_m_s` | `velocity_Wind_mps()(0)` | m/s |
| `kinematic/roll_rate_rad_s` | `rollRate_Wind_rps()` | rad/s |

---

## Python Binding

`ChannelRegistry` and `ChannelSubscriber` are exposed via
`src/python/bind_ring_buffer.cpp`. `SimRunner` exposes its registry via the
`channel_registry()` method.

```python
registry = runner.channel_registry()             # ChannelRegistry reference
channels = registry.available_channels()         # list[str]
sub = registry.subscribe("kinematic/altitude_m") # ChannelSubscriber

# On a timer (e.g., 20 Hz):
samples = sub.drain()  # list of (time_s, value) tuples; empty if no new data
```

`drain()` returns a `list[tuple[float, float]]` — each element is
`(time_s, value)`. Convert to numpy with `np.array(sub.drain())`.

---

## File Map

| File | Role |
| --- | --- |
| `include/runner/ChannelRegistry.hpp` | `Sample`, `ChannelSubscriber`, `ChannelRegistry` declarations |
| `src/runner/ChannelRegistry.cpp` | `ChannelRegistry` and `ChannelSubscriber` implementation |
| `src/python/bind_ring_buffer.cpp` | pybind11 bindings |
| `test/RingBuffer_test.cpp` | C++ unit tests |
| `python/test/test_ring_buffer_bindings.py` | Python binding tests |

---

## Open Questions

None. All design questions resolved above and in DR-5/DR-6.
