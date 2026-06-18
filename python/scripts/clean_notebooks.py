#!/usr/bin/env python3
"""Strip transient data from Jupyter notebooks in place.

Removes everything that is a product of *running* a notebook rather than part of
its source, so committed notebooks carry only inputs:

  * code-cell ``outputs`` (cleared to ``[]``)
  * code-cell ``execution_count`` (reset to ``null``)
  * transient per-cell metadata (execution timing, scroll/collapse view state)
  * kernel identifiers at the notebook level (``kernelspec`` and
    ``language_info`` — the previously-used kernel name and interpreter version)

Implemented against the standard library only (a notebook is just JSON), so it
runs under any Python interpreter with no third-party dependency. Files are
rewritten in the canonical Jupyter on-disk form (``indent=1``, sorted keys, a
trailing newline) so the result matches what Jupyter itself would save, keeping
diffs minimal. A notebook with no transient data is left byte-for-byte untouched.

Use it directly (``python clean_notebooks.py nb1.ipynb ...``) or via the
repository's pre-commit hook.

Exit status:
  0  success (whether or not anything was changed)
  2  a notebook could not be read, parsed, or written
"""
from __future__ import annotations

import json
import sys

# Notebook-level metadata that identifies the kernel/interpreter that ran it.
TRANSIENT_NOTEBOOK_METADATA = ("kernelspec", "language_info")

# Per-cell metadata produced by execution or by the front-end's view state.
TRANSIENT_CELL_METADATA = ("execution", "ExecuteTime", "collapsed", "scrolled")


def clean_notebook(notebook: dict) -> bool:
    """Strip transient data from a parsed notebook dict in place.

    Returns True if anything was changed.
    """
    changed = False

    metadata = notebook.get("metadata", {})
    for key in TRANSIENT_NOTEBOOK_METADATA:
        if key in metadata:
            del metadata[key]
            changed = True

    for cell in notebook.get("cells", []):
        if cell.get("cell_type") == "code":
            if cell.get("outputs"):
                cell["outputs"] = []
                changed = True
            if cell.get("execution_count") is not None:
                cell["execution_count"] = None
                changed = True
        cell_metadata = cell.get("metadata", {})
        for key in TRANSIENT_CELL_METADATA:
            if key in cell_metadata:
                del cell_metadata[key]
                changed = True

    return changed


def main(paths: list[str]) -> int:
    for path in paths:
        try:
            with open(path, encoding="utf-8") as handle:
                notebook = json.load(handle)
        except (OSError, ValueError) as exc:
            print(f"clean_notebooks: cannot read {path}: {exc}", file=sys.stderr)
            return 2

        if not clean_notebook(notebook):
            continue

        # Canonical Jupyter serialization: indent=1, sorted keys, trailing newline.
        text = json.dumps(notebook, indent=1, sort_keys=True, ensure_ascii=False)
        try:
            with open(path, "w", encoding="utf-8", newline="\n") as handle:
                handle.write(text)
                handle.write("\n")
        except OSError as exc:
            print(f"clean_notebooks: cannot write {path}: {exc}", file=sys.stderr)
            return 2
        print(f"clean_notebooks: stripped transient data from {path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
