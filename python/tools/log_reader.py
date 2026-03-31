"""FlightLogReader — loads MCAP and CSV simulation logs into DataFrames.

Each returned DataFrame uses ``time_s`` (float64) as its index.  Column names
carry SI unit suffixes matching the Logger schema (e.g. ``altitude_m``,
``roll_rate_rad_s``).

MCAP decoding
-------------
Messages use the DR-9 per-field topic convention: one topic per data field,
named ``"<source>/<field_name>"``.  Each message carries a JSON payload
``{"time_s": t, "value": v}``.  The reader groups topics by source prefix and
merges per-field DataFrames on a common ``time_s`` index to produce one
DataFrame per source.

Protobuf-encoded MCAP (written by the C++ Logger) requires the
``mcap-protobuf-support`` package and a defined Logged Channel Registry;
that path is deferred to roadmap item 4.

CSV decoding
------------
The CSV must contain a ``source_name`` column (written by ``Logger::exportCsv()``).
The source name is read from that column; the column itself is dropped from
the returned DataFrame.
"""
from __future__ import annotations

import json
from pathlib import Path

import pandas as pd
from mcap.reader import make_reader


class FlightLogReader:
    """Reads simulation log files and returns per-source pandas DataFrames."""

    def __init__(self) -> None:
        self._frames: dict[str, pd.DataFrame] | None = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def frames(self) -> dict[str, pd.DataFrame]:
        """Return the dict of DataFrames from the most recent load call.

        Raises ``RuntimeError`` if no file has been loaded yet.
        """
        if self._frames is None:
            raise RuntimeError("No file loaded; call load_csv() or load_mcap() first")
        return self._frames

    def load_csv(self, path: str | Path) -> dict[str, pd.DataFrame]:
        """Load a CSV file exported by ``Logger::exportCsv()``.

        The CSV must contain a ``source_name`` column; the source name key is
        read from that column.  Returns a dict keyed by source name.
        """
        path = Path(path)
        df = pd.read_csv(path)
        if "time_s" not in df.columns:
            raise ValueError(f"CSV has no 'time_s' column: {path}")
        if "source_name" not in df.columns:
            raise ValueError(f"CSV has no 'source_name' column: {path}")
        source_name = str(df["source_name"].iloc[0])
        df = df.drop(columns=["source_name"])
        df = df.set_index("time_s")
        df.index = df.index.astype("float64")
        self._frames = {source_name: df}
        return self._frames

    def load_mcap(self, path: str | Path) -> dict[str, pd.DataFrame]:
        """Load an MCAP log file using the DR-9 per-field topic convention.

        Topics must follow the pattern ``"<source>/<field_name>"``.  Messages
        must be JSON-encoded with payload ``{"time_s": t, "value": v}``.

        Returns one DataFrame per source, with field names as columns and
        ``time_s`` as the index.
        """
        path = Path(path)
        # rows[source][field] = list of (time_s, value) pairs
        rows: dict[str, dict[str, list[tuple[float, object]]]] = {}

        with open(path, "rb") as f:
            reader = make_reader(f)
            for _schema, channel, message in reader.iter_messages():
                topic = channel.topic
                if "/" not in topic:
                    # Legacy monolithic topic — skip; only per-field supported
                    continue
                source, field = topic.split("/", 1)
                payload = _decode_message(message.data, channel.message_encoding)
                time_s = float(payload["time_s"])
                value = payload["value"]
                rows.setdefault(source, {}).setdefault(field, []).append((time_s, value))

        result: dict[str, pd.DataFrame] = {}
        for source, fields in rows.items():
            per_field: list[pd.Series] = []
            for field, pairs in fields.items():
                times, values = zip(*pairs)
                s = pd.Series(values, index=pd.Index(times, name="time_s", dtype="float64"), name=field)
                per_field.append(s)
            df = pd.concat(per_field, axis=1)
            df.index = df.index.astype("float64")
            df.index.name = "time_s"
            result[source] = df

        self._frames = result
        return result

    def channel_names(self, source: str) -> list[str]:
        """Return column names for a previously loaded source.

        Raises ``RuntimeError`` if no file has been loaded yet, or ``KeyError``
        if ``source`` is not present in the loaded data.
        """
        if self._frames is None:
            raise RuntimeError("No file loaded; call load_csv() or load_mcap() first")
        if source not in self._frames:
            raise KeyError(f"Source {source!r} not loaded; available: {list(self._frames)}")
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
