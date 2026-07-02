"""Make the shared ``tools/`` modules importable from terrain entry-point scripts.

When a terrain script such as ``build_terrain.py`` is executed directly, only its own
directory (``tools/terrain``) is on ``sys.path``, so top-level shared modules like
``geodesy`` and ``geometry`` (which live in ``tools/``) cannot be found.  Importing this
module — as the first import in such a script — inserts the parent ``tools/`` directory
onto ``sys.path``.  It is a harmless no-op under pytest, where ``tools/`` is already on the
path.
"""

from __future__ import annotations

import sys
from pathlib import Path

_tools_dir = str(Path(__file__).resolve().parent.parent)
if _tools_dir not in sys.path:
    sys.path.insert(0, _tools_dir)
