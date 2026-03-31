"""ModeEventSeries — parses a step-function mode channel into transition events.

Consumed by both TimeHistoryFigure and TrajectoryView to overlay mode
transitions consistently across all views.
"""
from __future__ import annotations

from dataclasses import dataclass

import pandas as pd


@dataclass
class ModeEvent:
    """A single flight control mode transition."""

    time_s: float
    mode_id: int
    mode_name: str


class ModeEventSeries:
    """Ordered list of mode events parsed from a DataFrame channel.

    The first event always records the initial mode value at the first sample
    time (DR-2).  Subsequent events record each transition.

    An optional ``name_map`` may be passed to the constructor to apply
    human-readable names to all events (DR-2).
    """

    def __init__(
        self,
        events: list[ModeEvent],
        name_map: dict[int, str] | None = None,
    ) -> None:
        self.events: list[ModeEvent] = events
        if name_map:
            for event in self.events:
                event.mode_name = name_map.get(event.mode_id, str(event.mode_id))

    @classmethod
    def from_dataframe(
        cls,
        df: pd.DataFrame,
        mode_channel: str,
    ) -> ModeEventSeries:
        """Extract mode events from a step-function channel.

        The initial mode value is emitted as the first event at the first
        sample time.  Each subsequent state change produces an additional event.

        Parameters
        ----------
        df:
            DataFrame with a float64 ``time_s`` index.
        mode_channel:
            Column name of the integer mode-ID channel.
        """
        if mode_channel not in df.columns:
            raise KeyError(f"Mode channel {mode_channel!r} not in DataFrame")

        events: list[ModeEvent] = []
        prev: int | None = None

        for time_s, raw in df[mode_channel].items():
            mode_id = int(raw)
            if prev is None:
                events.append(
                    ModeEvent(
                        time_s=float(time_s),
                        mode_id=mode_id,
                        mode_name=str(mode_id),
                    )
                )
                prev = mode_id
            elif mode_id != prev:
                events.append(
                    ModeEvent(
                        time_s=float(time_s),
                        mode_id=mode_id,
                        mode_name=str(mode_id),
                    )
                )
                prev = mode_id

        return cls(events)
