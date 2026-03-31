"""Tests for TimeHistoryFigure — time_history.py."""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pandas as pd
import pytest

from time_history import TimeHistoryFigure


@pytest.fixture
def sample_frames() -> dict[str, pd.DataFrame]:
    t = np.linspace(0.0, 10.0, 100)
    df = pd.DataFrame(
        {
            "altitude_m": 100.0 + t,
            "roll_rad": 0.1 * np.sin(t),
            "airspeed_ias_m_s": 50.0 + 0.1 * t,
        },
        index=pd.Index(t, name="time_s"),
    )
    return {"aircraft": df}


class TestTimeHistoryFigure:
    def test_export_html_creates_file(
        self, sample_frames: dict, tmp_path: Path
    ) -> None:
        fig = TimeHistoryFigure(title="Test")
        fig.load(sample_frames)
        fig.add_panel(["altitude_m"], title="Altitude", y_label="m")
        out = tmp_path / "output.html"
        fig.export_html(out)
        assert out.exists()
        assert out.stat().st_size > 0

    def test_add_panel_channel_appears_in_figure(
        self, sample_frames: dict
    ) -> None:
        fig = TimeHistoryFigure()
        fig.load(sample_frames)
        fig.add_panel(["altitude_m"], title="Altitude", y_label="m")
        plotly_fig = fig.figure()
        trace_names = [t.name for t in plotly_fig.data]
        assert "altitude_m" in trace_names

    def test_shared_xaxis_present(self, sample_frames: dict) -> None:
        fig = TimeHistoryFigure()
        fig.load(sample_frames)
        fig.add_panel(["altitude_m"], title="Altitude", y_label="m")
        fig.add_panel(["roll_rad"], title="Roll", y_label="rad")
        plotly_fig = fig.figure()
        layout = plotly_fig.to_dict()["layout"]
        # make_subplots(shared_xaxes=True) links axes via 'matches'.
        # Plotly ≤5: xaxis2.matches = "x" (non-bottom points to bottom).
        # Plotly ≥6: xaxis.matches = "x2" (top points to bottom).
        # Either way, at least one x-axis carries 'matches'.
        assert "xaxis2" in layout, "Expected two x-axes for a two-panel figure"
        x_axes = {k: v for k, v in layout.items() if k.startswith("xaxis")}
        has_matches = any(v.get("matches") is not None for v in x_axes.values())
        assert has_matches, (
            "Expected at least one x-axis with 'matches' set "
            "(shared_xaxes=True was not applied)"
        )

    def test_multiple_channels_in_one_panel(self, sample_frames: dict) -> None:
        fig = TimeHistoryFigure()
        fig.load(sample_frames)
        fig.add_panel(["altitude_m", "airspeed_ias_m_s"], title="Multi", y_label="SI")
        plotly_fig = fig.figure()
        trace_names = [t.name for t in plotly_fig.data]
        assert "altitude_m" in trace_names
        assert "airspeed_ias_m_s" in trace_names

    def test_y2_channel_uses_secondary_axis(self, sample_frames: dict) -> None:
        fig = TimeHistoryFigure()
        fig.load(sample_frames)
        fig.add_panel(
            ["altitude_m"],
            title="Alt/Roll",
            y_label="m",
            y2_channels=["roll_rad"],
            y2_label="rad",
        )
        plotly_fig = fig.figure()
        # Secondary y-axis traces have yaxis='y2'
        secondary_traces = [t for t in plotly_fig.data if getattr(t, "yaxis", None) == "y2"]
        assert len(secondary_traces) == 1
        assert secondary_traces[0].name == "roll_rad"

    def test_mode_event_overlay_adds_shape(self, sample_frames: dict) -> None:
        from mode_overlay import ModeEvent, ModeEventSeries

        events = ModeEventSeries([ModeEvent(time_s=3.0, mode_id=1, mode_name="CRUISE")])
        fig = TimeHistoryFigure()
        fig.load(sample_frames)
        fig.add_panel(["altitude_m"], title="Alt", y_label="m")
        fig.set_mode_events(events)
        plotly_fig = fig.figure()
        layout = plotly_fig.to_dict()["layout"]
        shapes = layout.get("shapes", [])
        assert any(s.get("x0") == pytest.approx(3.0) for s in shapes)
