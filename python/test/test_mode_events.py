"""Tests for ModeEventSeries — mode_overlay.py."""
from __future__ import annotations

import pandas as pd
import pytest

from mode_overlay import ModeEvent, ModeEventSeries


@pytest.fixture
def step_df() -> pd.DataFrame:
    """Mode channel with three transitions: 0→1 at t=1.0, 1→2 at t=3.0, 2→0 at t=5.0."""
    times = [0.0, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0, 6.0]
    modes = [0,   0,   1,   1,   1,   2,   2,   0,   0  ]
    return pd.DataFrame(
        {"flight_control_mode_id": modes},
        index=pd.Index(times, name="time_s"),
    )


class TestModeEventSeries:
    def test_mode_events_from_step_channel(self, step_df: pd.DataFrame) -> None:
        events = ModeEventSeries.from_dataframe(step_df, "flight_control_mode_id")
        # Three transitions only (initial value is not an event)
        assert len(events.events) == 3
        assert events.events[0].time_s == pytest.approx(1.0)
        assert events.events[0].mode_id == 1
        assert events.events[1].time_s == pytest.approx(3.0)
        assert events.events[1].mode_id == 2
        assert events.events[2].time_s == pytest.approx(5.0)
        assert events.events[2].mode_id == 0

    def test_mode_names_mapped(self, step_df: pd.DataFrame) -> None:
        name_map = {0: "IDLE", 1: "TAKEOFF", 2: "CRUISE"}
        events = ModeEventSeries.from_dataframe(
            step_df, "flight_control_mode_id", name_map=name_map
        )
        assert events.events[0].mode_name == "TAKEOFF"
        assert events.events[1].mode_name == "CRUISE"
        assert events.events[2].mode_name == "IDLE"

    def test_unmapped_id_falls_back_to_string(self, step_df: pd.DataFrame) -> None:
        events = ModeEventSeries.from_dataframe(
            step_df, "flight_control_mode_id", name_map={}
        )
        assert events.events[0].mode_name == "1"

    def test_no_transitions_yields_empty(self) -> None:
        df = pd.DataFrame(
            {"mode_id": [2, 2, 2]},
            index=pd.Index([0.0, 0.1, 0.2], name="time_s"),
        )
        events = ModeEventSeries.from_dataframe(df, "mode_id")
        assert len(events.events) == 0

    def test_missing_channel_raises(self, step_df: pd.DataFrame) -> None:
        with pytest.raises(KeyError):
            ModeEventSeries.from_dataframe(step_df, "nonexistent_channel")
