"""SimSession — Python live simulation session with UDP broadcast.

Design authority: docs/architecture/live_sim_view.md §Python SimSession Design

SimSession owns the C++ objects (Aircraft, SimRunner, UdpSimulationBroadcaster)
for the lifetime of the simulation.  It does not own or manage the Godot window.
"""
from __future__ import annotations

import json
from pathlib import Path

import liteaero_sim_py


class SimSession:
    """Python live simulation controller.

    Constructs Aircraft, SimRunner, and UdpSimulationBroadcaster via pybind11
    bindings, wires them together, and starts the run loop in RealTime mode.

    Args:
        aircraft_config: JSON string or file path for the aircraft configuration.
        mode: Execution mode string — ``"realtime"`` (default) or ``"batch"``.
        dt_s: Simulation timestep in seconds (default 0.02).
        duration_s: Run duration in seconds; 0 = run until :meth:`stop` (default 0).
        broadcast_port: UDP port to broadcast SimulationFrame datagrams (default 14560).
    """

    def __init__(
        self,
        aircraft_config: str,
        mode: str = "realtime",
        dt_s: float = 0.02,
        duration_s: float = 0.0,
        broadcast_port: int = 14560,
    ) -> None:
        # Accept either a JSON string or a file path.
        config_str = aircraft_config
        if not aircraft_config.lstrip().startswith("{"):
            config_str = Path(aircraft_config).read_text(encoding="utf-8")

        self._aircraft = liteaero_sim_py.Aircraft(config_str, dt_s=dt_s)

        cfg = liteaero_sim_py.RunnerConfig(
            dt_s=dt_s,
            duration_s=duration_s,
            time_scale=1.0,
            mode=mode,
        )
        self._runner = liteaero_sim_py.SimRunner()
        self._runner.initialize(cfg, self._aircraft)

        self._broadcaster = liteaero_sim_py.UdpSimulationBroadcaster(broadcast_port)
        self._runner.set_broadcaster(self._broadcaster)

        self._scripted_input: liteaero_sim_py.ScriptedInput | None = None

    # ------------------------------------------------------------------

    def start(self) -> None:
        """Start the simulation run loop.  Returns immediately in realtime mode."""
        self._runner.start()

    def stop(self) -> None:
        """Request termination and block until the run loop exits."""
        self._runner.stop()

    def is_running(self) -> bool:
        """True while the run loop is active."""
        return self._runner.is_running()

    # ------------------------------------------------------------------

    def set_scripted_input(self, scripted: liteaero_sim_py.ScriptedInput) -> None:
        """Wire a ScriptedInput adapter.  Must be called before :meth:`start`."""
        self._scripted_input = scripted
        self._runner.setManualInput(scripted)

    @property
    def scripted_input(self) -> liteaero_sim_py.ScriptedInput | None:
        """The active ScriptedInput adapter, or None if not set."""
        return self._scripted_input
