"""Tests for FlightLogReader — log_reader.py.

Tests are ordered to fail first (TDD): run before implementing log_reader.py.
"""
from __future__ import annotations

import json
from pathlib import Path

import pandas as pd
import pytest

from log_reader import FlightLogReader

# ---------------------------------------------------------------------------
# Recognized SI unit suffixes (subset covering Logger channel names)
# ---------------------------------------------------------------------------
_SI_SUFFIXES = (
    "_m",
    "_m_s",
    "_m_s2",
    "_rad",
    "_rad_s",
    "_n",
    "_nd",
    "_kg",
    "_s",
    "_pa",
    "_k",
    "_deg",
    "_hz",
    "_w",
)

# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

_CSV_ROWS = [
    {"time_s": 0.0, "altitude_m": 100.0, "roll_rad": 0.10, "airspeed_ias_m_s": 50.0},
    {"time_s": 0.1, "altitude_m": 100.5, "roll_rad": 0.12, "airspeed_ias_m_s": 50.2},
    {"time_s": 0.2, "altitude_m": 101.0, "roll_rad": 0.11, "airspeed_ias_m_s": 50.1},
]


@pytest.fixture
def csv_fixture(tmp_path: Path) -> Path:
    p = tmp_path / "aircraft.csv"
    pd.DataFrame(_CSV_ROWS).to_csv(p, index=False)
    return p


@pytest.fixture
def mcap_fixture(tmp_path: Path) -> Path:
    """Minimal JSON-encoded MCAP with two sources: 'aircraft' and 'environment'."""
    from mcap.writer import Writer

    path = tmp_path / "test_log.mcap"
    with open(path, "wb") as f:
        writer = Writer(f)
        writer.start(profile="", library="liteaero-test")

        schema_a = writer.register_schema(
            name="aircraft_state",
            encoding="jsonschema",
            data=b'{"type":"object"}',
        )
        schema_e = writer.register_schema(
            name="environment_state",
            encoding="jsonschema",
            data=b'{"type":"object"}',
        )
        ch_a = writer.register_channel(
            topic="aircraft",
            message_encoding="json",
            schema_id=schema_a,
        )
        ch_e = writer.register_channel(
            topic="environment",
            message_encoding="json",
            schema_id=schema_e,
        )
        for i in range(3):
            t_ns = i * 100_000_000
            writer.add_message(
                channel_id=ch_a,
                log_time=t_ns,
                publish_time=t_ns,
                data=json.dumps(
                    {"time_s": i * 0.1, "altitude_m": 100.0 + i * 0.5, "roll_rad": 0.1}
                ).encode(),
            )
            writer.add_message(
                channel_id=ch_e,
                log_time=t_ns,
                publish_time=t_ns,
                data=json.dumps({"time_s": i * 0.1, "wind_speed_m_s": 5.0}).encode(),
            )
        writer.finish()
    return path


@pytest.fixture
def mcap_and_csv_fixture(tmp_path: Path) -> tuple[Path, Path]:
    """Same three-row dataset written as both MCAP and CSV."""
    from mcap.writer import Writer

    mcap_path = tmp_path / "aircraft.mcap"
    with open(mcap_path, "wb") as f:
        writer = Writer(f)
        writer.start(profile="", library="liteaero-test")
        schema_id = writer.register_schema(
            name="aircraft_state",
            encoding="jsonschema",
            data=b'{"type":"object"}',
        )
        ch_id = writer.register_channel(
            topic="aircraft",
            message_encoding="json",
            schema_id=schema_id,
        )
        for row in _CSV_ROWS:
            t_ns = int(row["time_s"] * 1_000_000_000)
            writer.add_message(
                channel_id=ch_id,
                log_time=t_ns,
                publish_time=t_ns,
                data=json.dumps(row).encode(),
            )
        writer.finish()

    csv_path = tmp_path / "aircraft.csv"
    pd.DataFrame(_CSV_ROWS).to_csv(csv_path, index=False)
    return mcap_path, csv_path


# ---------------------------------------------------------------------------
# CSV tests
# ---------------------------------------------------------------------------


class TestLoadCsv:
    def test_load_csv_returns_dataframe(self, csv_fixture: Path) -> None:
        frames = FlightLogReader().load_csv(csv_fixture)
        assert "aircraft" in frames
        df = frames["aircraft"]
        assert isinstance(df, pd.DataFrame)
        assert df.index.name == "time_s"
        assert df.index.dtype == "float64"
        assert "altitude_m" in df.columns

    def test_load_csv_si_units_in_columns(self, csv_fixture: Path) -> None:
        df = FlightLogReader().load_csv(csv_fixture)["aircraft"]
        for col in df.columns:
            assert any(col.endswith(s) for s in _SI_SUFFIXES), (
                f"Column {col!r} lacks a recognized SI suffix"
            )


# ---------------------------------------------------------------------------
# MCAP tests
# ---------------------------------------------------------------------------


class TestLoadMcap:
    def test_load_mcap_multi_source(self, mcap_fixture: Path) -> None:
        frames = FlightLogReader().load_mcap(mcap_fixture)
        assert "aircraft" in frames
        assert "environment" in frames
        assert frames["aircraft"].index.name == "time_s"
        assert frames["environment"].index.name == "time_s"
        assert len(frames["aircraft"]) == 3
        assert len(frames["environment"]) == 3

    def test_load_mcap_matches_csv(self, mcap_and_csv_fixture: tuple[Path, Path]) -> None:
        mcap_path, csv_path = mcap_and_csv_fixture
        reader = FlightLogReader()
        df_mcap = reader.load_mcap(mcap_path)["aircraft"]
        df_csv = reader.load_csv(csv_path)["aircraft"]
        cols = ["altitude_m", "roll_rad", "airspeed_ias_m_s"]
        pd.testing.assert_frame_equal(
            df_mcap.sort_index()[cols],
            df_csv.sort_index()[cols],
            rtol=1e-5,
        )
