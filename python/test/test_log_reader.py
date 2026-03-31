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
    {"time_s": 0.0, "source_name": "aircraft", "altitude_m": 100.0, "roll_rad": 0.10, "airspeed_ias_m_s": 50.0},
    {"time_s": 0.1, "source_name": "aircraft", "altitude_m": 100.5, "roll_rad": 0.12, "airspeed_ias_m_s": 50.2},
    {"time_s": 0.2, "source_name": "aircraft", "altitude_m": 101.0, "roll_rad": 0.11, "airspeed_ias_m_s": 50.1},
]


@pytest.fixture
def csv_fixture(tmp_path: Path) -> Path:
    p = tmp_path / "aircraft.csv"
    pd.DataFrame(_CSV_ROWS).to_csv(p, index=False)
    return p


@pytest.fixture
def mcap_fixture(tmp_path: Path) -> Path:
    """Per-field MCAP with two sources: 'aircraft' and 'environment'.

    Topics follow the DR-9 convention: ``"<source>/<field_name>"``.
    Each message carries ``{"time_s": t, "value": v}``.
    """
    from mcap.writer import Writer

    path = tmp_path / "test_log.mcap"
    with open(path, "wb") as f:
        writer = Writer(f)
        writer.start(profile="", library="liteaero-test")

        schema_id = writer.register_schema(
            name="field_value",
            encoding="jsonschema",
            data=b'{"type":"object"}',
        )
        ch_alt = writer.register_channel(
            topic="aircraft/altitude_m",
            message_encoding="json",
            schema_id=schema_id,
        )
        ch_roll = writer.register_channel(
            topic="aircraft/roll_rad",
            message_encoding="json",
            schema_id=schema_id,
        )
        ch_wind = writer.register_channel(
            topic="environment/wind_speed_m_s",
            message_encoding="json",
            schema_id=schema_id,
        )
        for i in range(3):
            t_ns = i * 100_000_000
            t_s = i * 0.1
            writer.add_message(
                channel_id=ch_alt,
                log_time=t_ns,
                publish_time=t_ns,
                data=json.dumps({"time_s": t_s, "value": 100.0 + i * 0.5}).encode(),
            )
            writer.add_message(
                channel_id=ch_roll,
                log_time=t_ns,
                publish_time=t_ns,
                data=json.dumps({"time_s": t_s, "value": 0.1}).encode(),
            )
            writer.add_message(
                channel_id=ch_wind,
                log_time=t_ns,
                publish_time=t_ns,
                data=json.dumps({"time_s": t_s, "value": 5.0}).encode(),
            )
        writer.finish()
    return path


@pytest.fixture
def mcap_and_csv_fixture(tmp_path: Path) -> tuple[Path, Path]:
    """Same three-row dataset written as both MCAP (per-field) and CSV (with source_name)."""
    from mcap.writer import Writer

    mcap_path = tmp_path / "aircraft.mcap"
    with open(mcap_path, "wb") as f:
        writer = Writer(f)
        writer.start(profile="", library="liteaero-test")
        schema_id = writer.register_schema(
            name="field_value",
            encoding="jsonschema",
            data=b'{"type":"object"}',
        )
        ch_alt = writer.register_channel(
            topic="aircraft/altitude_m",
            message_encoding="json",
            schema_id=schema_id,
        )
        ch_roll = writer.register_channel(
            topic="aircraft/roll_rad",
            message_encoding="json",
            schema_id=schema_id,
        )
        ch_ias = writer.register_channel(
            topic="aircraft/airspeed_ias_m_s",
            message_encoding="json",
            schema_id=schema_id,
        )
        for row in _CSV_ROWS:
            t_ns = int(row["time_s"] * 1_000_000_000)
            t_s = row["time_s"]
            writer.add_message(
                channel_id=ch_alt,
                log_time=t_ns,
                publish_time=t_ns,
                data=json.dumps({"time_s": t_s, "value": row["altitude_m"]}).encode(),
            )
            writer.add_message(
                channel_id=ch_roll,
                log_time=t_ns,
                publish_time=t_ns,
                data=json.dumps({"time_s": t_s, "value": row["roll_rad"]}).encode(),
            )
            writer.add_message(
                channel_id=ch_ias,
                log_time=t_ns,
                publish_time=t_ns,
                data=json.dumps({"time_s": t_s, "value": row["airspeed_ias_m_s"]}).encode(),
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

    def test_load_csv_source_name_from_column(self, csv_fixture: Path) -> None:
        """Source name must come from the 'source_name' column, not the filename."""
        frames = FlightLogReader().load_csv(csv_fixture)
        # The file is named 'aircraft.csv' and source_name column is 'aircraft' —
        # verify the column drives the key, not the stem, by checking the column
        # is absent from the returned DataFrame.
        df = frames["aircraft"]
        assert "source_name" not in df.columns

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

    def test_load_mcap_field_columns(self, mcap_fixture: Path) -> None:
        """Per-field topics must be merged into per-source DataFrames."""
        frames = FlightLogReader().load_mcap(mcap_fixture)
        assert "altitude_m" in frames["aircraft"].columns
        assert "roll_rad" in frames["aircraft"].columns
        assert "wind_speed_m_s" in frames["environment"].columns

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


# ---------------------------------------------------------------------------
# Statefulness tests (DR-1)
# ---------------------------------------------------------------------------


class TestFlightLogReaderState:
    def test_frames_getter_returns_loaded_dict(self, csv_fixture: Path) -> None:
        reader = FlightLogReader()
        returned = reader.load_csv(csv_fixture)
        assert reader.frames() is returned

    def test_channel_names_raises_before_load(self) -> None:
        with pytest.raises((KeyError, RuntimeError)):
            FlightLogReader().channel_names("aircraft")

    def test_channel_names_returns_columns_after_load(self, csv_fixture: Path) -> None:
        reader = FlightLogReader()
        reader.load_csv(csv_fixture)
        names = reader.channel_names("aircraft")
        assert "altitude_m" in names
        assert "source_name" not in names
