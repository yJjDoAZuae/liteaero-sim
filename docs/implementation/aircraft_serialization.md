# Aircraft Serialization — Implementation Plan (Roadmap Item 5)

Implementation notes for `Aircraft::serializeJson()` / `deserializeJson()` and
`serializeProto()` / `deserializeProto()`.  For design intent, contract, and JSON schema,
see [`docs/architecture/aircraft.md — Serialization`](../architecture/aircraft.md#serialization).

---

## Current State

`serializeJson()` and `deserializeJson()` exist in `src/Aircraft.cpp` (items 3 & 4) but
are incomplete:

| Gap | Location |
| ----- | ---------- |
| `deserializeJson()` does not check `schema_version` or `"type"` | `src/Aircraft.cpp:170` |
| `deserializeJson()` does not restore `_initial_state` | `src/Aircraft.cpp:182` |
| `serializeProto()` and `deserializeProto()` are not declared or implemented | — |
| No `Aircraft` proto message in `proto/liteaerosim.proto` | — |
| `deserializeJson()` calls `_propulsion->deserializeJson()` but does not handle `_propulsion == nullptr` | `src/Aircraft.cpp:183` |

---

## Files to Create / Modify

| File | Action |
| ------ | -------- |
| `src/Aircraft.cpp` | Fix `deserializeJson()` gaps; add `serializeProto()` / `deserializeProto()` |
| `include/Aircraft.hpp` | Add `serializeProto()` / `deserializeProto()` declarations |
| `proto/liteaerosim.proto` | Add `AircraftState` message |
| `test/Aircraft_test.cpp` | Add round-trip + schema-version tests |

---

## 1. Proto Message

Add to `proto/liteaerosim.proto` after `PropulsionPropState`:

```proto
// ---------------------------------------------------------------------------
// AircraftState — full restart snapshot for Aircraft
// ---------------------------------------------------------------------------
message AircraftState {
    int32                    schema_version  = 1;
    KinematicState           kinematic_state = 2;
    KinematicState           initial_state   = 3;
    LoadFactorAllocatorState allocator       = 4;
    LiftCurveParams          lift_curve      = 5;
    AeroPerformanceParams    aero_performance = 6;
    AirframePerformanceParams airframe        = 7;
    InertiaParams            inertia         = 8;
    oneof propulsion {
        PropulsionJetState  jet  = 9;
        PropulsionEdfState  edf  = 10;
        PropulsionPropState prop = 11;
    }
}
```

The `oneof propulsion` field encodes both the propulsion **type** and its **state** in a
single discriminated field — no separate `"type"` string needed in proto.

---

## 2. JSON Fixes (`deserializeJson`)

Add the following to the top of `Aircraft::deserializeJson()`:

```cpp
if (j.at("schema_version").get<int>() != 1) {
    throw std::runtime_error("Aircraft::deserializeJson: unsupported schema_version");
}
if (j.at("type").get<std::string>() != "Aircraft") {
    throw std::runtime_error("Aircraft::deserializeJson: unexpected type");
}
```

Add `_initial_state` restore (currently missing):

```cpp
_initial_state.deserializeJson(j.at("initial_state"));
```

Add null-check before propulsion restore:

```cpp
if (_propulsion && j.contains("propulsion")) {
    _propulsion->deserializeJson(j.at("propulsion"));
}
```

---

## 3. Proto Implementation

### `serializeProto()`

```text
1. Construct las_proto::AircraftState proto.
2. proto.set_schema_version(1).
3. *proto.mutable_kinematic_state()    = parse(_state.serializeProto())
4. *proto.mutable_initial_state()      = parse(_initial_state.serializeProto())
5. *proto.mutable_allocator()          = parse(_allocator->serializeProto())
6. *proto.mutable_lift_curve()         = parse(_liftCurve->serializeProto())
7. *proto.mutable_aero_performance()   = parse(_aeroPerf->serializeProto())
8. *proto.mutable_airframe()           = parse(_airframe.serializeProto())
9. *proto.mutable_inertia()            = parse(_inertia.serializeProto())
10. Dispatch on propulsion type:
      dynamic_cast<PropulsionJet*>  → *proto.mutable_jet()
      dynamic_cast<PropulsionEDF*>  → *proto.mutable_edf()
      dynamic_cast<PropulsionProp*> → *proto.mutable_prop()
11. Return proto.SerializeAsString() as std::vector<uint8_t>.
```

Embedding sub-messages: each subsystem's `serializeProto()` returns a
`std::vector<uint8_t>` that is a serialized proto message. Rather than embedding raw
bytes, deserialize those bytes into the appropriate sub-message type and assign to the
`AircraftState` fields via `mutable_*()`.

### `deserializeProto()`

Mirror of `deserializeJson()`:

```text
1. ParseFromArray → las_proto::AircraftState proto.
2. Check schema_version == 1; throw on mismatch.
3. Restore _airframe, _inertia from proto fields.
4. Emplace _liftCurve from proto.lift_curve().SerializeAsString().
5. Emplace _aeroPerf  from proto.aero_performance().SerializeAsString().
6. Emplace _allocator (placeholder); call _allocator->deserializeProto().
7. Restore _state, _initial_state.
8. Dispatch propulsion: check proto.propulsion_case() and call
   _propulsion->deserializeProto() if present.
```

---

## 4. Self-Contained Deserialization (Future)

The current design requires the caller to inject the correct `V_Propulsion` subclass
before calling `deserializeJson()` or `deserializeProto()`. To make deserialization fully
self-contained, a propulsion factory is needed.

**JSON factory design:**

```cpp
// include/propulsion/PropulsionFactory.hpp
namespace liteaerosim::propulsion {

// Creates and returns a fully-initialized V_Propulsion from a JSON snapshot.
// Dispatches on j["type"]: "PropulsionJet", "PropulsionEDF", "PropulsionProp".
// Throws std::runtime_error for unknown types.
std::unique_ptr<V_Propulsion> fromJson(const nlohmann::json& j);

} // namespace liteaerosim::propulsion
```

This is optional for item 5 — the current pre-injection approach is acceptable for the
simulator's construction pattern (the scenario always knows which propulsion model to use).
Defer to a later item if not needed sooner.

---

## 5. Test Specification (TDD Order)

Write failing tests first, then implement.

### Failing tests to add to `test/Aircraft_test.cpp`

**JSON round-trip:**

```text
1. Initialize Aircraft, step 10 times.
2. snapshot = serializeJson()
3. Create new Aircraft with same StubPropulsion type.
4. deserializeJson(snapshot).
5. step() once on both; compare state().velocity_NED_mps() — must match within float epsilon.
```

**Proto round-trip:**

```text
Same pattern using serializeProto() / deserializeProto().
```

**Schema-version mismatch — JSON:**

```text
snapshot = serializeJson(); snapshot["schema_version"] = 99;
EXPECT_THROW(deserializeJson(snapshot), std::runtime_error) with match "schema_version".
```

**Schema-version mismatch — Proto:**

```text
bytes = serializeProto(); corrupt byte at offset 0x08 (schema_version tag) to set value 99.
EXPECT_THROW(deserializeProto(bytes), std::runtime_error) with match "schema_version".
```

### Corruption pattern for proto mismatch test

All existing proto mismatch tests (e.g., `LiftCurveModel_test.cpp`) use:

```cpp
bytes[1] = static_cast<uint8_t>(99);  // overwrite schema_version value byte
```

Field 1 (`schema_version`) in proto3 wire format: tag byte `0x08` at `bytes[0]`,
value varint at `bytes[1]`. Setting `bytes[1] = 99` sets `schema_version = 99`.

---

## 6. Implementation Order

1. Add `AircraftState` message to `proto/liteaerosim.proto`.
2. Write failing tests (4 tests above).
3. Fix `deserializeJson()` gaps (schema_version check, `_initial_state`, null-check).
4. Add `serializeProto()` / `deserializeProto()` declarations to `include/Aircraft.hpp`.
5. Implement `serializeProto()` / `deserializeProto()` in `src/Aircraft.cpp`.
6. Run tests; confirm all 10 Aircraft tests pass.
