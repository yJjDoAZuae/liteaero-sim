# LiteAeroSim

A lightweight, parameterizable, computationally efficient simulation for aircraft

## Building

Use [`build.sh`](build.sh) — the unified build manager. See [BUILD.md](BUILD.md) for full docs.

```bash
./build.sh            # build everything (Release)
./build.sh help       # commands, components, options
./build.sh gdext      # (re)build the Godot viewer plugin
./build.sh test       # build + run the C++ test suite
```

To run the live simulation + Godot viewer: [`./run_sim.sh`](run_sim.sh).
