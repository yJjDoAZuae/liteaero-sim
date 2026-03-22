# LiteAeroSim Documentation

## Contents

| Section | Description |
| --- | --- |
| [Architecture](architecture/overview.md) | System design, layer model, subsystem breakdown |
| [Dynamic Element Design](architecture/dynamic_element.md) | `DynamicElement` and `SisoElement` — the unified lifecycle base for all stateful components |
| [Aircraft Design](architecture/aircraft.md) | Aircraft class — use cases, physics integration loop, serialization, JSON initialization |
| [Propulsion Design](architecture/propulsion.md) | V_Propulsion virtual interface, PropulsionJet, PropulsionProp — class hierarchy, use cases, serialization |
| [Algorithms](algorithms/filters.md) | Filter design, discretization, control algorithms |
| [Implementation Notes](implementation/equations_of_motion.md) | Implementation decisions, API references, build notes |
| [Roadmap](roadmap/aircraft.md) | Recommended next steps and open work |
| [Dependencies](dependencies/README.md) | External libraries — versions, licenses, integration method |
| [Installation](installation/README.md) | Build from source, toolchain setup, first run |
| [Testing](testing/strategy.md) | TDD strategy, test patterns, coverage requirements |
| [Examples](examples/siso_elements.md) | Usage examples for simulation elements |
| [Guidelines](guidelines/general.md) | Coding standards — TDD, SI units, naming, serialization |

## Quick Links

- [Coding guidelines — general](guidelines/general.md)
- [Coding guidelines — C++](guidelines/cpp.md)
- [Coding guidelines — Python](guidelines/python.md)
- [Component lifecycle](architecture/overview.md#component-lifecycle)
- [DynamicBlock proposed interface](architecture/dynamic_block.md#proposed-interface)
- [Migration strategy](architecture/dynamic_block.md#migration-strategy)
