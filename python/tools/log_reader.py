"""FlightLogReader — loads MCAP and CSV simulation logs into DataFrames.

Each returned DataFrame uses ``time_s`` (float64) as its index.  Column names
carry SI unit suffixes matching the Logger schema (e.g. ``altitude_m``,
``roll_rate_rad_s``).

MCAP decoding
-------------
Messages must use ``message_encoding == "json"``.  Protobuf-encoded MCAP
(written by the C++ Logger) requires the ``mcap-protobuf-support`` package and
a defined Logged Channel Registry; that path is deferred to roadmap item 4.
"""
from __future__ import annotations

import json
from pathlib import Path

import pandas as pd
from mcap.reader import make_reader


class FlightLogReader:
    """Reads simulation log files and returns per-source pandas DataFrames."""

    def __init__(self) -> None:
        self._frames: dict[str, pd.DataFrame] = {}

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def load_csv(self, path: str | Path) -> dict[str, pd.DataFrame]:
        """Load a CSV file exported by ``Logger::exportCsv()``.

        Returns a single-entry dict keyed by the filename stem.
        """
        path = Path(path)
        df = pd.read_csv(path)
        if "time_s" not in df.columns:
            raise ValueError(f"CSV has no 'time_s' column: {path}")
        df = df.set_index("time_s")
        df.index = df.index.astype("float64")
        self._frames = {path.stem: df}
        return self._frames

    def load_mcap(self, path: str | Path) -> dict[str, pd.DataFrame]:
        """Load an MCAP log file.

        Returns one DataFrame per MCAP channel topic (= source name).  Each
        message is decoded to a flat dict and appended as a row; the resulting
        DataFrame is indexed by ``time_s``.

        Only JSON-encoded messages (``message_encoding == "json"``) are
        supported.  Protobuf support is deferred to roadmap item 4.
        """
        path = Path(path)
        rows: dict[str, list[dict]] = {}

        with open(path, "rb") as f:
            reader = make_reader(f)
            for _schema, channel, message in reader.iter_messages():
                topic = channel.topic
                if topic not in rows:
                    rows[topic] = []
                rows[topic].append(_decode_message(message.data, channel.message_encoding))

        result: dict[str, pd.DataFrame] = {}
        for topic, row_list in rows.items():
            df = pd.DataFrame(row_list)
            if "time_s" in df.columns:
                df = df.set_index("time_s")
                df.index = df.index.astype("float64")
            result[topic] = df

        self._frames = result
        return result

    def channel_names(self, source: str) -> list[str]:
        """Return column names for a previously loaded source."""
        if source not in self._frames:
            raise KeyError(f"Source {source!r} not loaded; call load_csv() or load_mcap() first")
        return list(self._frames[source].columns)


# ------------------------------------------------------------------
# Internal helpers
# ------------------------------------------------------------------


def _decode_message(data: bytes, encoding: str) -> dict:
    """Decode a raw MCAP message payload to a flat dict."""
    if encoding == "json":
        return json.loads(data.decode("utf-8"))
    if encoding == "protobuf":
        raise NotImplementedError(
            "Protobuf-encoded MCAP decoding is not yet implemented.  "
            "Use load_csv() or a JSON-encoded MCAP fixture.  "
            "Add mcap-protobuf-support when the Logged Channel Registry "
            "(roadmap item 4) is complete."
        )
    raise ValueError(f"Unsupported MCAP message encoding: {encoding!r}")
