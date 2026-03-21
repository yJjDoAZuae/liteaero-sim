# Originating Requirements — Present State

These requirements are derived from the implemented codebase, the design documents, and
the project guidelines. They represent the constraints that shaped the current architecture.

| ID | Requirement |
| --- | --- |
| SR-1 | Simulate the 6-DOF rigid-body dynamics of a fixed-wing aircraft at a fixed timestep. |
| SR-2 | Model the atmospheric environment: ISA pressure, temperature, and density with configurable deviations; relative humidity; wind (constant, power-law, logarithmic profile); continuous turbulence (Dryden, 6 filters, Tustin-discretized); discrete gust (1-cosine, MIL-SPEC-8785C). |
| SR-3 | Model terrain geometry with variable level of detail; support height and surface normal queries from arbitrary NED positions. |
| SR-4 | Provide sensor measurement models that produce realistic instrument outputs (noise, lag, bias, geometric error) from simulation truth state. |
| SR-5 | Log simulation state to file in a format suitable for post-flight analysis. |
| SR-6 | Support deterministic replay: given the same configuration seed and initial conditions, the simulation produces bitwise-identical output across runs. |
| SR-7 | Support batch simulation: reset and re-initialize all components to arbitrary conditions without re-reading configuration files. |
| SR-8 | Serialize and restore complete simulation state at any point (checkpoint and restore for branched Monte Carlo and reproducible debugging). |
| SR-9 | All domain-layer computations use SI units. Unit conversion is permitted only at external interfaces (configuration parsing, display). |
| SR-10 | Every stateful component implements a common lifecycle: `initialize(config)` → `reset()` → `step()×N` → `serializeJson()` / `deserializeJson()`. |
| SR-11 | Every stateful component supports both JSON and protobuf serialization with round-trip fidelity. |
| SR-12 | The control library (filters, integrators, PID) is general-purpose and usable within simulation physics models and, in future, in flight code. |
