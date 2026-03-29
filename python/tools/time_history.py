"""TimeHistoryFigure — multi-panel Plotly time history with shared x-axis.

All panels share a single x-axis so pan and zoom propagate to every panel.
Traces with more than 50 000 points use ``go.Scattergl`` (WebGL renderer).
Mode events are overlaid as vertical dashed lines with annotations.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING

import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots

if TYPE_CHECKING:
    from mode_overlay import ModeEventSeries

_SCATTERGL_THRESHOLD = 50_000


@dataclass
class _Panel:
    channels: list[str]
    title: str
    y_label: str
    y2_channels: list[str] = field(default_factory=list)
    y2_label: str = ""


class TimeHistoryFigure:
    """Vertically stacked Plotly subplot panels with a shared time axis.

    Usage::

        fig = TimeHistoryFigure(title="Landing Review")
        fig.load(frames)
        fig.add_panel(["altitude_m"], title="Altitude", y_label="m")
        fig.add_panel(["roll_rad"], title="Roll", y_label="rad")
        fig.set_mode_events(mode_events)
        fig.export_html("output/review.html")
    """

    def __init__(self, title: str = "") -> None:
        self._title = title
        self._panels: list[_Panel] = []
        self._mode_events: ModeEventSeries | None = None
        self._data: dict[str, pd.DataFrame] = {}

    # ------------------------------------------------------------------
    # Configuration
    # ------------------------------------------------------------------

    def load(self, frames: dict[str, pd.DataFrame]) -> None:
        """Supply source DataFrames keyed by source name."""
        self._data = frames

    def add_panel(
        self,
        channels: list[str],
        title: str = "",
        y_label: str = "",
        y2_channels: list[str] | None = None,
        y2_label: str = "",
    ) -> _Panel:
        """Append a panel.  Returns the panel object (for further configuration)."""
        panel = _Panel(
            channels=channels,
            title=title,
            y_label=y_label,
            y2_channels=y2_channels or [],
            y2_label=y2_label,
        )
        self._panels.append(panel)
        return panel

    def set_mode_events(self, events: ModeEventSeries) -> None:
        """Overlay mode transition markers on all panels."""
        self._mode_events = events

    # ------------------------------------------------------------------
    # Build / export
    # ------------------------------------------------------------------

    def build(self) -> go.Figure:
        """Construct and return the Plotly figure.  Does not open a browser."""
        n = len(self._panels)
        if n == 0:
            return go.Figure()

        has_secondary = [bool(p.y2_channels) for p in self._panels]
        specs = [[{"secondary_y": h}] for h in has_secondary]

        fig = make_subplots(
            rows=n,
            cols=1,
            shared_xaxes=True,
            vertical_spacing=0.04,
            subplot_titles=[p.title for p in self._panels] if any(p.title for p in self._panels) else None,
            specs=specs,
        )

        # Flatten all frames into a single channel lookup
        channels: dict[str, pd.Series] = {}
        for df in self._data.values():
            for col in df.columns:
                channels[col] = df[col]

        for row_idx, (panel, secondary) in enumerate(zip(self._panels, has_secondary), start=1):
            for ch in panel.channels:
                if ch not in channels:
                    continue
                series = channels[ch]
                x = list(series.index)
                y = list(series.values)
                trace_cls = go.Scattergl if len(x) > _SCATTERGL_THRESHOLD else go.Scatter
                kwargs = dict(row=row_idx, col=1)
                if secondary:
                    kwargs["secondary_y"] = False
                fig.add_trace(trace_cls(x=x, y=y, name=ch, mode="lines"), **kwargs)

            for ch in panel.y2_channels:
                if ch not in channels:
                    continue
                series = channels[ch]
                x = list(series.index)
                y = list(series.values)
                trace_cls = go.Scattergl if len(x) > _SCATTERGL_THRESHOLD else go.Scatter
                fig.add_trace(
                    trace_cls(x=x, y=y, name=ch, mode="lines"),
                    row=row_idx,
                    col=1,
                    secondary_y=True,
                )

        # Y-axis labels
        for row_idx, panel in enumerate(self._panels, start=1):
            axis_key = "yaxis" if row_idx == 1 else f"yaxis{row_idx}"
            if panel.y_label:
                fig.update_layout(**{axis_key: {"title_text": panel.y_label}})

        # Mode event overlays: vertical dashed lines across the full figure
        if self._mode_events:
            for event in self._mode_events.events:
                fig.add_vline(
                    x=event.time_s,
                    line_dash="dash",
                    line_color="gray",
                    annotation_text=event.mode_name,
                    annotation_position="top",
                )

        fig.update_layout(title_text=self._title, showlegend=True)
        return fig

    def show(self) -> None:
        """Open the figure in a browser."""
        self.build().show()

    def export_html(self, path: str | Path) -> None:
        """Write a self-contained HTML file."""
        self.build().write_html(str(path))
