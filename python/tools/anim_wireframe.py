"""Parallel 4-view wireframe landing animation for the gear/terrain validation notebooks.

The notebook used to build the animation inline (matplotlib ``FuncAnimation`` + ``to_jshtml``),
which rasterizes every frame serially. This library renders the frames across CPU cores with a
``ProcessPoolExecutor`` and returns a self-contained HTML player, so the notebook is a single call:

    from tools import anim_wireframe
    HTML(anim_wireframe.render_landing_animation(heading_rad=..., pitch_rad=..., ...))

It must be an importable module (not notebook ``__main__``) so spawn workers can pickle the target.
"""

import base64
import io
import math
from concurrent.futures import ProcessPoolExecutor

import numpy as np
import matplotlib
matplotlib.use("Agg")               # headless raster backend in every worker
import matplotlib.pyplot as plt     # noqa: E402

# Per-worker static geometry, populated by init_worker().
_S: dict = {}


# ── Pure geometry ────────────────────────────────────────────────────────────────────────
def box_edges_body(center, half):
    """The 12 edges of an axis-aligned box as a list of (pt1, pt2) body-frame pairs."""
    c, h = np.asarray(center, float), np.asarray(half, float)
    signs = [(-1, -1, -1), (-1, -1, 1), (-1, 1, -1), (-1, 1, 1),
             (1, -1, -1), (1, -1, 1), (1, 1, -1), (1, 1, 1)]
    corners = np.array([c + np.array([s[0] * h[0], s[1] * h[1], s[2] * h[2]]) for s in signs])
    edges = []
    for i in range(8):
        for j in range(i + 1, 8):
            if np.sum(np.abs(corners[i] - corners[j]) > 1e-6) == 1:
                edges.append((corners[i], corners[j]))
    return edges


def euler_to_Rnb(heading, pitch, roll):
    """ZYX Euler angles -> body-to-NED rotation matrix."""
    ch, sh = math.cos(heading), math.sin(heading)
    cp, sp = math.cos(pitch),   math.sin(pitch)
    cr, sr = math.cos(roll),    math.sin(roll)
    return np.array([
        [ch * cp, ch * sp * sr - sh * cr, ch * sp * cr + sh * sr],
        [sh * cp, sh * sp * sr + ch * cr, sh * sp * cr - ch * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def body_to_enu(pts_body, R_nb, cg_enu):
    """Body-frame points (N,3) -> ENU world frame. ENU = [E, N, U] = [NED_y, NED_x, -NED_z]."""
    pts_ned = (R_nb @ pts_body.T).T
    pts_enu = np.column_stack([pts_ned[:, 1], pts_ned[:, 0], -pts_ned[:, 2]])
    return pts_enu + cg_enu


# ── Worker functions (module-level so spawn can pickle them) ───────────────────────────────
def init_worker(static):
    """Seed this worker with the static geometry (called once per worker)."""
    _S.clear()
    _S.update(static)


def render_frame(payload):
    """Rasterize one frame. payload = (idx, heading, pitch, roll, cg_e, cg_n, cg_u,
    s0, s1, s2, t, agl). Returns (idx, base64_png)."""
    (idx, heading, pitch, roll, cg_e, cg_n, cg_u, s0, s1, s2, t, agl) = payload
    S = _S
    R_nb = euler_to_Rnb(heading, pitch, roll)
    cg = np.array([[cg_e, cg_n, cg_u]])
    Nr, Er, Ur = S["ranges"]

    fig = plt.figure(figsize=S["figsize"], layout="constrained")
    ax_side = fig.add_subplot(2, 2, 1)
    ax_top  = fig.add_subplot(2, 2, 2)
    ax_rear = fig.add_subplot(2, 2, 3)
    ax_3d   = fig.add_subplot(2, 2, 4, projection="3d")
    fig.suptitle("SUAS Landing Approach — 4-View Wireframe", fontsize=11, fontweight="bold")

    ax_side.set_xlabel("North (m)"); ax_side.set_ylabel("Up (m)");   ax_side.set_title("Side View")
    ax_top.set_xlabel("East (m)");   ax_top.set_ylabel("North (m)"); ax_top.set_title("Top View")
    ax_rear.set_xlabel("East (m)");  ax_rear.set_ylabel("Up (m)");   ax_rear.set_title("Rear View")
    ax_3d.set_xlabel("East"); ax_3d.set_ylabel("North"); ax_3d.set_zlabel("Up"); ax_3d.set_title("Isometric")

    ax_side.axhline(0, color="saddlebrown", lw=2); ax_side.set_xlim(Nr); ax_side.set_ylim(Ur)
    ax_top.axhline(0, color="saddlebrown", lw=0.5, ls=":"); ax_top.set_xlim(Er); ax_top.set_ylim(Nr)
    ax_rear.axhline(0, color="saddlebrown", lw=2); ax_rear.set_xlim(Er); ax_rear.set_ylim(Ur)

    g = S["ghost"]
    ax_side.plot(g[:, 1], g[:, 2], "lightgray", lw=0.8, zorder=1)
    ax_top.plot( g[:, 0], g[:, 1], "lightgray", lw=0.8, zorder=1)
    ax_rear.plot(g[:, 0], g[:, 2], "lightgray", lw=0.8, zorder=1)
    for ax in (ax_side, ax_top, ax_rear):
        ax.set_aspect("equal"); ax.grid(True, alpha=0.3)
    ax_3d.set_box_aspect([1, 3, 1])

    for edges, col in S["vol_edges"]:
        for p1, p2 in edges:
            seg = body_to_enu(np.array([p1, p2]), R_nb, cg)   # (2,3) ENU
            ax_side.plot(seg[:, 1], seg[:, 2], color=col, lw=0.8)
            ax_top.plot( seg[:, 0], seg[:, 1], color=col, lw=0.8)
            ax_rear.plot(seg[:, 0], seg[:, 2], color=col, lw=0.8)
            ax_3d.plot(  seg[:, 0], seg[:, 1], seg[:, 2], color=col, lw=0.8)

    defl  = np.array([s0, s1, s2])
    upper = S["gear_attach"]
    lower = S["gear_attach"] + (defl[:, None] + S["gear_radius"][:, None]) * S["gear_axis"]
    for gi in range(len(S["gear_colors"])):
        col = S["gear_colors"][gi]
        seg = body_to_enu(np.array([upper[gi], lower[gi]]), R_nb, cg)   # [upper, lower]
        ax_side.plot(seg[:, 1], seg[:, 2], color=col, lw=1.5)
        ax_top.plot( seg[:, 0], seg[:, 1], color=col, lw=1.5)
        ax_rear.plot(seg[:, 0], seg[:, 2], color=col, lw=1.5)
        ax_3d.plot(  seg[:, 0], seg[:, 1], seg[:, 2], color=col, lw=1.5)
        low = seg[1]
        ax_side.plot([low[1]], [low[2]], "o", color=col, ms=3)
        ax_top.plot( [low[0]], [low[1]], "o", color=col, ms=3)
        ax_rear.plot([low[0]], [low[2]], "o", color=col, ms=3)

    ax_side.plot([cg_n], [cg_u], "ko", ms=4, zorder=5)
    ax_top.plot( [cg_e], [cg_n], "ko", ms=4, zorder=5)
    ax_rear.plot([cg_e], [cg_u], "ko", ms=4, zorder=5)
    ax_3d.plot(  [cg_e], [cg_n], [cg_u], "ko", ms=4)
    ax_3d.set_xlim(cg_e - 2, cg_e + 2); ax_3d.set_ylim(cg_n - 3, cg_n + 3); ax_3d.set_zlim(-0.2, 3.0)
    ax_side.text(0.02, 0.96, f"t = {t:.2f} s  AGL = {agl:.2f} m",
                 transform=ax_side.transAxes, fontsize=8, va="top")

    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=S.get("dpi", 80))
    plt.close(fig)
    return idx, base64.b64encode(buf.getvalue()).decode("ascii")


def frames_to_html(frames_b64, fps=25, uid="af"):
    """Assemble base64 PNG frames into a self-contained HTML player (play / pause / scrub)."""
    import json as _json
    interval = int(1000 / max(fps, 1))
    data = _json.dumps(frames_b64)
    return f"""
<div style="font-family:sans-serif">
  <img id="{uid}_img" style="max-width:100%;border:1px solid #ddd"/>
  <div style="margin-top:4px">
    <button id="{uid}_play">&#9654; Play</button>
    <button id="{uid}_pause">&#10073;&#10073; Pause</button>
    <input id="{uid}_slider" type="range" min="0" max="{len(frames_b64) - 1}" value="0"
           style="width:60%;vertical-align:middle"/>
    <span id="{uid}_lbl">0 / {len(frames_b64) - 1}</span>
  </div>
  <script>
  (function() {{
    var F = {data};
    var img = document.getElementById("{uid}_img");
    var sl  = document.getElementById("{uid}_slider");
    var lbl = document.getElementById("{uid}_lbl");
    var k = 0, timer = null;
    function show(i) {{
      k = ((+i) % F.length + F.length) % F.length;
      img.src = "data:image/png;base64," + F[k];
      sl.value = k; lbl.textContent = k + " / " + (F.length - 1);
    }}
    document.getElementById("{uid}_play").onclick = function() {{
      if (timer) return; timer = setInterval(function() {{ show(k + 1); }}, {interval});
    }};
    document.getElementById("{uid}_pause").onclick = function() {{ clearInterval(timer); timer = null; }};
    sl.oninput = function() {{ clearInterval(timer); timer = null; show(sl.value); }};
    show(0);
  }})();
  </script>
</div>"""


# ── High-level entry point ────────────────────────────────────────────────────────────────
def render_landing_animation(*, heading_rad, pitch_rad, roll_rad, cg_enu, strut_defl,
                             t_s, agl_m, vol_defs, gear_attach, gear_axis, gear_radius,
                             gear_colors, decimate=10, fps=25, figsize=(14, 11), dpi=80,
                             ground_up_max=None, max_workers=None, verbose=True):
    """Render the 4-view wireframe landing animation in parallel and return an HTML player.

    Per-step arrays (heading_rad, pitch_rad, roll_rad, cg_enu[N,3] ENU, strut_defl[N,3], t_s,
    agl_m) describe the trajectory; vol_defs = [(center, half, color), ...] the airframe boxes;
    gear_attach/axis[3,3] and gear_radius[3] the struts. Frames are decimated by ``decimate`` and
    rasterized across ``max_workers`` processes (default: all cores).
    """
    import os
    import time

    cg_enu = np.asarray(cg_enu, float)
    idx = np.arange(0, len(cg_enu), decimate)

    vol_edges = [(box_edges_body(c, h), col) for (c, h, col) in vol_defs]
    Nr = [float(cg_enu[:, 1].min() - 1), float(cg_enu[:, 1].max() + 1)]
    Er = [float(cg_enu[:, 0].min() - 1), float(cg_enu[:, 0].max() + 1)]
    up_max = float(cg_enu[:, 2].max() + 1) if ground_up_max is None else float(ground_up_max)
    static = dict(vol_edges=vol_edges,
                  gear_attach=np.asarray(gear_attach, float),
                  gear_axis=np.asarray(gear_axis, float),
                  gear_radius=np.asarray(gear_radius, float),
                  gear_colors=list(gear_colors),
                  ghost=cg_enu, ranges=(Nr, Er, [-0.5, up_max]),
                  figsize=tuple(figsize), dpi=dpi)

    payloads = [(int(k), float(heading_rad[k]), float(pitch_rad[k]), float(roll_rad[k]),
                 float(cg_enu[k, 0]), float(cg_enu[k, 1]), float(cg_enu[k, 2]),
                 float(strut_defl[k, 0]), float(strut_defl[k, 1]), float(strut_defl[k, 2]),
                 float(t_s[k]), float(agl_m[k])) for k in idx]

    # Each worker only rasterizes frames — it is not BLAS-bound. Left at their
    # defaults, N spawned workers each start a full multi-threaded BLAS pool
    # (OpenBLAS/OMP/MKL), oversubscribing the cores and exhausting memory
    # (OpenBLAS aborts with "Memory allocation still failed"). Pin the worker
    # BLAS to one thread. The vars are read at numpy import, which on spawn
    # happens in the fresh child *before* init_worker runs, so they must be set
    # in the parent environment here (restored afterward) to be inherited.
    _thread_vars = ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS",
                    "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS")
    _saved = {k: os.environ.get(k) for k in _thread_vars}
    for k in _thread_vars:
        os.environ[k] = "1"

    t0 = time.time()
    try:
        with ProcessPoolExecutor(max_workers=max_workers,
                                 initializer=init_worker, initargs=(static,)) as ex:
            results = list(ex.map(render_frame, payloads, chunksize=8))
    finally:
        for k, v in _saved.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v
    results.sort(key=lambda r: r[0])
    frames = [b for _, b in results]
    if verbose:
        nw = max_workers or os.cpu_count()
        print(f"Rendered {len(frames)} frames in {time.time() - t0:.1f}s "
              f"across {nw} worker processes ({fps} fps playback).")
    return frames_to_html(frames, fps=fps)
