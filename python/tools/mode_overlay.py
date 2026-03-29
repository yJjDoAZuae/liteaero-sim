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
    """Ordered list of mode transition events parsed from a DataFrame channel."""

    def __init__(self, events: list[ModeEvent]) -> None:
        self.events: list[ModeEvent] = events

    @classmethod
    def from_dataframe(
        cls,
        df: pd.DataFrame,
        mode_channel: str,
        name_map: dict[int, str] | None = None,
    ) -> ModeEventSeries:
        """Extract mode transitions from a step-function channel.

        The initial value of the channel is not emitted as an event; only
        subsequent state changes produce a ``ModeEvent``.

        Parameters
        ----------
        df:
            DataFrame with a float64 ``time_s`` index.
        mode_channel:
            Column name of the integer mode-ID channel.
        name_map:
            Optional mapping from mode ID to a human-readable name string.
            Unmapped IDs fall back to ``str(mode_id)``.
        """
        if mode_channel not in df.columns:
            raise KeyError(f"Mode channel {mode_channel!r} not in DataFrame")

        name_map = name_map or {}
        events: list[ModeEvent] = []
        prev: int | None = None

        for time_s, raw in df[mode_channel].items():
            mode_id = int(raw)
            if prev is None:
                prev = mode_id
                continue
            if mode_id != prev:
                events.append(
                    ModeEvent(
                        time_s=float(time_s),
                        mode_id=mode_id,
                        mode_name=name_map.get(mode_id, str(mode_id)),
                    )
                )
                prev = mode_id

        return cls(events)
