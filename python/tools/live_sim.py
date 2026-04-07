"""live_sim.py — Python CLI launcher for the live simulation viewer.

Design authority: docs/architecture/live_sim_view.md §Launch Modes

Starts the simulation (Aircraft + SimRunner + UdpSimulationBroadcaster) and
broadcasts SimulationFrame datagrams to the Godot 4 scene.  The Godot window
is opened separately by the developer.

Usage::

    python tools/live_sim.py --config aircraft_configs/ga_aircraft.json
    python tools/live_sim.py --config ga.json --dt 0.01 --port 15000
"""
from __future__ import annotations

import argparse
import signal
import sys
import time

from live_sim_session import SimSession


def build_arg_parser() -> argparse.ArgumentParser:
    """Return the argument parser.  Exposed for testing."""
    p = argparse.ArgumentParser(
        description="LiteAero Sim live simulation launcher",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--config", required=True,
                   help="Aircraft JSON config file path")
    p.add_argument("--scripted", action="store_true", default=True,
                   help="Use ScriptedInput with neutral commands (default)")
    p.add_argument("--joystick", default=None, metavar="PATH",
                   help="JoystickInput JSON config file; enables joystick mode "
                        "(joystick mode requires the C++ live_sim binary)")
    p.add_argument("--dt", type=float, default=0.02,
                   help="Simulation timestep (seconds)")
    p.add_argument("--port", type=int, default=14560,
                   help="UDP broadcast port")
    return p


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    if args.joystick is not None:
        print(
            "Warning: --joystick is not supported in the Python launcher. "
            "Use the C++ live_sim binary for joystick mode.",
            file=sys.stderr,
        )
        return 1

    session = SimSession(
        aircraft_config=args.config,
        mode="realtime",
        dt_s=args.dt,
        broadcast_port=args.port,
    )

    stop_requested = False

    def _handle_signal(sig, frame):  # noqa: ANN001
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT,  _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    print(f"Broadcasting to 127.0.0.1:{args.port}  (dt={args.dt} s)")
    print("Press Ctrl+C to stop.")

    session.start()

    while not stop_requested and session.is_running():
        time.sleep(0.05)

    session.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
