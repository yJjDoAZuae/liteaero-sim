#!/usr/bin/env python3
"""
Touchdown animation — 2-D landing gear contact simulation.

Run:
    python python/scripts/touchdown_animation.py

Requires:  numpy  matplotlib

Three-DOF planar model (surge x, heave z, pitch θ).
Spring-damper contact at each wheel unit; no tyre lateral forces.

What is shown
─────────────
  • Rotating aircraft silhouette (fuselage, wing, tail surfaces)
  • Landing gear struts that shorten when the wheel contacts the ground
  • Per-wheel upward contact-force arrows (orange = main, blue = nose)
  • Lift arrow (green) and weight arrow (yellow) at the CG
  • Live time-series: contact forces and strut compression
"""

from __future__ import annotations

import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Polygon, Circle
import matplotlib.animation as animation

mpl.rcParams["toolbar"] = "None"

# ── Physical constants ─────────────────────────────────────────────────────────
G   = 9.81    # m/s²
RHO = 1.225   # kg/m³  (sea-level ISA)

# ── Aircraft parameters ────────────────────────────────────────────────────────
#   A 2 000 kg light-transport analog at 55 m/s approach speed.
MASS      = 2_000.0   # kg
I_YY      = 6_000.0   # kg·m²  pitch moment of inertia about CG
S_REF     = 20.0      # m²     reference wing area
MAC       = 2.0       # m      mean aerodynamic chord
AR        = 8.0       # —      wing aspect ratio  b²/S
E         = 0.82      # —      Oswald span efficiency
CD0       = 0.028     # —      zero-lift drag coefficient
CL_ALPHA  = 5.0       # rad⁻¹  pre-stall lift-curve slope  dCL/dα
#   Pitching moment about CG:  Cm = CM0 + CM_ALPHA·α + CM_Q_HAT·(q·c̄/2V)
#   CM0 chosen so trim occurs at α_trim = CL_trim/CL_ALPHA = 0.530/5 = 0.106 rad
#   → CM0 = −CM_ALPHA · α_trim = 0.90 × 0.106 ≈ 0.095
CM0       =  0.095   # —
CM_ALPHA  = -0.90    # rad⁻¹  pitch-stability derivative  (< 0 = statically stable)
CM_Q_HAT  = -8.0     # —      non-dimensional pitch-damping derivative

# ── Gear parameters ─────────────────────────────────────────────────────────────
#   attach: strut root offset from CG in body frame  (x forward, z up)  [m]
#   L0:     nominal (unloaded) strut extension  [m]
#   k, b:   spring stiffness [N/m] and damping [N·s/m]
#   r:      tyre radius  [m]
_GEAR = [
    dict(attach=np.array([-1.3, -0.65]), L0=0.80, k=55_000, b=4_000, r=0.32, label="main"),
    dict(attach=np.array([ 2.6, -0.60]), L0=0.72, k=27_000, b=2_000, r=0.27, label="nose"),
]

# ── Approach initial conditions ────────────────────────────────────────────────
#   3° glide slope, 55 m/s IAS, trimmed pitch ≈ 9.1° nose-up
#   Main-gear contact height (CG above ground) ≈ 1.96 m;
#   start at z = 10 m → ~2.8 s before touchdown
V_APPROACH = 55.0
GAMMA_RAD  = np.radians(-3.0)               # flight-path angle (negative = descending)
ALPHA_TRIM = 0.106                           # rad ≈ 6.1°
THETA0     = ALPHA_TRIM - GAMMA_RAD         # ≈ 0.158 rad ≈ 9.1° nose-up
X0, Z0     = 0.0, 10.0
VX0        = V_APPROACH * np.cos(abs(GAMMA_RAD))
VZ0        = V_APPROACH * np.sin(GAMMA_RAD)

# ── Simulation ─────────────────────────────────────────────────────────────────
DT    = 0.002   # s
T_MAX = 14.0    # s


def _rot2(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s,  c]])


def _gear_step(cg: np.ndarray, vel: np.ndarray, theta: float, q: float):
    """
    Quasi-static spring-damper contact.

    Penetration depth  h  is computed from the kinematic position of the
    fully-extended wheel-bottom relative to z = 0.  Strut compression is
    assumed equal to the ground-plane penetration (infinitely stiff airframe).

    Returns
    -------
    F_total  (2,) ndarray   net gear force on CG [N, world frame]
    M_total  float          net pitch moment about CG [N·m, +ve nose-up]
    states   list[dict]     per-wheel visual and scalar data
    """
    R          = _rot2(theta)
    strut_down = R @ np.array([0.0, -1.0])   # body −z  in world frame
    F_total    = np.zeros(2)
    M_total    = 0.0
    states: list[dict] = []

    for g in _GEAR:
        attach = cg + R @ g["attach"]       # attachment point, world frame
        r_arm  = attach - cg                # moment arm from CG

        # Tyre-contact z when strut is fully extended:
        contact_z_free = (attach + g["L0"] * strut_down)[1] - g["r"]

        # Penetration depth  h > 0  means contact
        h = max(0.0, -contact_z_free)

        if h > 0.0:
            # Velocity of strut attachment:  v = v_cg + ω × r_arm
            # 2-D:  v_x = q·r_z,  v_z = −q·r_x
            v_attach = vel + np.array([q * r_arm[1], -q * r_arm[0]])

            # Penetration rate (positive = deepening = compressive)
            dh_dt = -v_attach[1]

            # Spring-damper normal force; damp only compression
            Fz = g["k"] * h + g["b"] * max(0.0, dh_dt)
            Fz = max(0.0, Fz)

            F_total += np.array([0.0, Fz])
            M_total += r_arm[0] * Fz       # +ve r_x × F_z = nose-up moment
            strut_ext = max(0.0, g["L0"] - h)
        else:
            Fz, strut_ext = 0.0, g["L0"]

        wheel_center = attach + strut_ext * strut_down

        states.append(
            dict(
                label      = g["label"],
                in_contact = h > 0.0,
                h          = h,
                Fz         = Fz,
                attach_w   = attach,
                wheel_c    = wheel_center,
                r_tyre     = g["r"],
            )
        )

    return F_total, M_total, states


def _aero_step(vel: np.ndarray, theta: float, q: float):
    """
    Linear lift curve + parabolic drag polar + quasi-static pitch moment.

    Returns
    -------
    F_aero   (2,) ndarray   aerodynamic force [N, world frame]
    M_aero   float          aerodynamic pitch moment about CG [N·m]
    info     dict           scalar diagnostics (L, D, alpha, lift_dir, V)
    """
    V      = max(np.linalg.norm(vel), 1.0)
    gamma  = np.arctan2(vel[1], vel[0])
    alpha  = np.clip(theta - gamma, -0.25, 0.45)
    q_dyn  = 0.5 * RHO * V**2

    CL     = CL_ALPHA * alpha
    CD     = CD0 + CL**2 / (np.pi * E * AR)
    Cm     = CM0 + CM_ALPHA * alpha + CM_Q_HAT * q * MAC / (2.0 * V)

    L, D   = q_dyn * S_REF * CL, q_dyn * S_REF * CD
    M      = q_dyn * S_REF * MAC * Cm

    vh         = vel / V
    lift_dir   = np.array([-vh[1], vh[0]])   # 90° CCW from velocity unit vector
    F_aero     = L * lift_dir - D * vh

    return F_aero, M, dict(L=L, D=D, alpha=alpha, lift_dir=lift_dir, V=V)


def _integrate(state: np.ndarray, dt: float):
    x, z, vx, vz, theta, q = state
    cg  = np.array([x,  z])
    vel = np.array([vx, vz])

    F_g, M_g, gear_states = _gear_step(cg, vel, theta, q)
    F_a, M_a, aero_info   = _aero_step(vel, theta, q)
    F_w                   = np.array([0.0, -MASS * G])

    F = F_g + F_a + F_w
    M = M_g + M_a

    ax_ = F[0] / MASS
    az_ = F[1] / MASS
    qd  = M     / I_YY

    return (
        np.array([x  + vx * dt,
                  z  + vz * dt,
                  vx + ax_ * dt,
                  vz + az_ * dt,
                  np.clip(theta + q * dt, -0.15, 0.40),
                  q  + qd  * dt]),
        gear_states,
        aero_info,
    )


# ── Pre-compute full trajectory ────────────────────────────────────────────────
N     = int(T_MAX / DT)
state = np.array([X0, Z0, VX0, VZ0, THETA0, 0.0])

_t     = np.empty(N);  _x    = np.empty(N);  _z     = np.empty(N)
_theta = np.empty(N);  _vx   = np.empty(N);  _vz    = np.empty(N)
_Fz_m  = np.zeros(N);  _Fz_n = np.zeros(N)
_h_m   = np.zeros(N);  _h_n  = np.zeros(N)
_lift  = np.zeros(N)
_gear_hist = [None] * N
_aero_hist = [None] * N

for i in range(N):
    _t[i], _x[i], _z[i] = i * DT, state[0], state[1]
    _theta[i], _vx[i], _vz[i] = state[4], state[2], state[3]

    state, gs, ai = _integrate(state, DT)
    _gear_hist[i] = gs
    _aero_hist[i] = ai
    _lift[i]      = ai["L"]

    for g in gs:
        if g["label"] == "main":
            _Fz_m[i], _h_m[i] = g["Fz"], g["h"]
        else:
            _Fz_n[i], _h_n[i] = g["Fz"], g["h"]

print(f"Sim: {N} steps  |  peak main {_Fz_m.max()/1000:.1f} kN  "
      f"|  peak nose {_Fz_n.max()/1000:.1f} kN")

# ── Aircraft body polygons (body frame, CG at origin) ─────────────────────────
_FUS = np.array([[ 5.0,  0.0],
                 [ 4.0,  0.45],
                 [ 0.5,  0.55],
                 [-3.0,  0.42],
                 [-3.9,  0.12],
                 [-3.9, -0.18],
                 [-3.0, -0.38],
                 [ 0.5, -0.48],
                 [ 4.0, -0.38]])

_WING = np.array([[ 1.6,  0.12],
                  [-0.6,  0.12],
                  [-1.4, -0.12],
                  [ 0.8, -0.12]])   # thin swept bar — side view

_HT   = np.array([[-2.7,  0.18],
                  [-3.8,  0.18],
                  [-3.8, -0.02],
                  [-2.7, -0.02]])

_VT   = np.array([[-2.7,  0.18],
                  [-2.9,  1.25],
                  [-3.7,  0.18]])


def _to_world(pts: np.ndarray, cg: np.ndarray, theta: float) -> np.ndarray:
    R = _rot2(theta)
    return np.array([cg + R @ p for p in pts])


# ── Figure / axes ─────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(15, 9), facecolor="#080816")

AX_MAIN = fig.add_axes([0.02, 0.30, 0.96, 0.68])
AX_F    = fig.add_axes([0.05, 0.04, 0.42, 0.22])
AX_H    = fig.add_axes([0.55, 0.04, 0.42, 0.22])

for ax in (AX_MAIN, AX_F, AX_H):
    ax.set_facecolor("#06060f")
    ax.tick_params(colors="#888899", labelsize=8)
    for sp in ax.spines.values():
        sp.set_edgecolor("#22223a")

# Fixed camera window — chosen so touchdown occurs roughly mid-frame
_X_LO, _X_HI = -10.0, 120.0
_Z_LO, _Z_HI = -1.5, 12.0
AX_MAIN.set_xlim(_X_LO, _X_HI)
AX_MAIN.set_ylim(_Z_LO, _Z_HI)
AX_MAIN.set_aspect("equal")
AX_MAIN.set_xlabel("X  (m)", color="#888899", fontsize=9)
AX_MAIN.set_ylabel("Altitude  (m)", color="#888899", fontsize=9)

# Ground surface
AX_MAIN.axhline(0.0, color="#336622", lw=2.5, zorder=1)
AX_MAIN.fill_between([_X_LO, _X_HI], [_Z_LO, _Z_LO], [0, 0],
                     color="#0f1a0a", zorder=0)

# Runway centre-line markings
for xi in range(0, int(_X_HI), 30):
    AX_MAIN.fill_between([xi, xi + 14], [-0.04, -0.04], [0.0, 0.0],
                         color="#44443a", alpha=0.7, zorder=1)

# Glide-slope indicator line
gs_x = np.linspace(_X_LO, _X_HI, 2)
gs_z = Z0 + (gs_x - X0) * np.tan(GAMMA_RAD)
AX_MAIN.plot(gs_x, gs_z, "--", color="#335533", lw=0.9, alpha=0.5, zorder=1)

# Time-series subplots
for ax, title, ylabel in [
    (AX_F, "Wheel Contact Forces", "Force  (N)"),
    (AX_H, "Strut Compression",    "Compression  (m)"),
]:
    ax.set_facecolor("#06060f")
    ax.set_xlim(0, T_MAX)
    ax.set_xlabel("Time  (s)", color="#888899", fontsize=8)
    ax.set_ylabel(ylabel,      color="#888899", fontsize=8)
    ax.set_title(title,        color="#aaaacc", fontsize=9)

_fz_peak = max(_Fz_m.max(), _Fz_n.max(), 1.0)
_h_peak  = max(_h_m.max(),  _h_n.max(),  0.01)
AX_F.set_ylim(-500, _fz_peak * 1.2)
AX_H.set_ylim(-0.005, _h_peak * 1.25)

# Ghost traces (faint background)
AX_F.plot(_t, _Fz_m, color="#ff6633", alpha=0.12, lw=0.7)
AX_F.plot(_t, _Fz_n, color="#33aaff", alpha=0.12, lw=0.7)
AX_H.plot(_t, _h_m,  color="#ff6633", alpha=0.12, lw=0.7)
AX_H.plot(_t, _h_n,  color="#33aaff", alpha=0.12, lw=0.7)

from matplotlib.lines import Line2D as _L2D
_leg = [_L2D([0],[0], color="#ff6633", label="Main gear"),
        _L2D([0],[0], color="#33aaff", label="Nose gear")]
for ax in (AX_F, AX_H):
    ax.legend(handles=_leg, facecolor="#10102a", labelcolor="#ccccdd",
              fontsize=8, framealpha=0.85, edgecolor="#33334a")

# ── Animated artists ───────────────────────────────────────────────────────────

# Aircraft body
_p_fus  = Polygon(_FUS,  closed=True, fc="#3a5a8a", ec="#7799cc", lw=1.5, zorder=5)
_p_wing = Polygon(_WING, closed=True, fc="#2a4a7a", ec="#6688bb", lw=1.0, zorder=4)
_p_ht   = Polygon(_HT,   closed=True, fc="#2a4a7a", ec="#6688bb", lw=1.0, zorder=4)
_p_vt   = Polygon(_VT,   closed=True, fc="#2a4a7a", ec="#6688bb", lw=1.0, zorder=4)
for p in (_p_fus, _p_wing, _p_ht, _p_vt):
    AX_MAIN.add_patch(p)

# CG marker
(_cg_dot,) = AX_MAIN.plot([], [], "o", color="#ffdd44", ms=5, zorder=10)

# Gear struts
_strut_lines = {
    "main": AX_MAIN.plot([], [], "-", color="#8899bb", lw=3, zorder=6)[0],
    "nose": AX_MAIN.plot([], [], "-", color="#8899bb", lw=3, zorder=6)[0],
}

# Wheel circles
_wheels = {
    "main": Circle((0, -200), 0.32, fc="#131327", ec="#5566aa", lw=1.5, zorder=7),
    "nose": Circle((0, -200), 0.27, fc="#131327", ec="#5566aa", lw=1.5, zorder=7),
}
for c in _wheels.values():
    AX_MAIN.add_patch(c)

# Tyre contact marker (small dot at ground contact)
_contact_dots = {
    "main": AX_MAIN.plot([], [], "o", color="#ff6633", ms=4, zorder=11)[0],
    "nose": AX_MAIN.plot([], [], "o", color="#33aaff", ms=4, zorder=11)[0],
}

# Force arrows: posA = tail, posB = head
_FSCALE = 1.0 / 55_000   # m per N  (contact-force arrow length)
_LSCALE = 1.0 / 60_000   # m per N  (lift / weight arrow length)

_OFF = (0.0, -500.0)   # off-screen position used to hide arrows


def _arrow(color: str, lw: float = 2.0, **kwargs) -> FancyArrowPatch:
    return FancyArrowPatch(
        _OFF, _OFF,
        mutation_scale=14, arrowstyle="->",
        color=color, lw=lw, zorder=9,
        **kwargs,
    )


_arr_main   = _arrow("#ff6633")
_arr_nose   = _arrow("#33aaff")
_arr_lift   = _arrow("#33ff88", lw=1.8)
_arr_weight = _arrow("#ffcc00", lw=1.8)
for a in (_arr_main, _arr_nose, _arr_lift, _arr_weight):
    AX_MAIN.add_patch(a)

# Labels
def _lbl(color: str) -> mpl.text.Text:
    return AX_MAIN.text(0, -500, "", color=color, fontsize=8, zorder=12)


_lbl_main   = _lbl("#ff8855")
_lbl_nose   = _lbl("#55bbff")
_lbl_lift   = _lbl("#55ffaa")
_lbl_weight = _lbl("#ffdd55")

# Status text
_status = AX_MAIN.text(
    0.01, 0.97, "", transform=AX_MAIN.transAxes,
    color="#ffffff", fontsize=9, va="top", fontfamily="monospace",
    bbox=dict(boxstyle="round,pad=0.4", fc="#00000099", ec="none"),
    zorder=13,
)

# Live time-series traces
_live_fzm, = AX_F.plot([], [], color="#ff6633", lw=1.6)
_live_fzn, = AX_F.plot([], [], color="#33aaff", lw=1.6)
_live_hm,  = AX_H.plot([], [], color="#ff6633", lw=1.6)
_live_hn,  = AX_H.plot([], [], color="#33aaff", lw=1.6)
_cur_f     = AX_F.axvline(0, color="#ffffff", lw=0.8, alpha=0.45)
_cur_h     = AX_H.axvline(0, color="#ffffff", lw=0.8, alpha=0.45)

# ── Update function ────────────────────────────────────────────────────────────
_FPS    = 30
_STRIDE = max(1, int(round(1.0 / (_FPS * DT))))
_FRAMES = list(range(0, N, _STRIDE))


def _update(frame: int) -> tuple:
    i     = _FRAMES[frame]
    t     = _t[i]
    cg    = np.array([_x[i], _z[i]])
    theta = _theta[i]
    V     = np.hypot(_vx[i], _vz[i])

    # ── Aircraft body ────────────────────────────────────────────────────────
    _p_fus.set_xy(_to_world(_FUS,  cg, theta))
    _p_wing.set_xy(_to_world(_WING, cg, theta))
    _p_ht.set_xy(_to_world(_HT,   cg, theta))
    _p_vt.set_xy(_to_world(_VT,   cg, theta))
    _cg_dot.set_data([cg[0]], [cg[1]])

    # ── Landing gear ─────────────────────────────────────────────────────────
    for gs in _gear_hist[i]:
        lbl  = gs["label"]
        aw   = gs["attach_w"]
        wc   = gs["wheel_c"]
        r    = gs["r_tyre"]
        Fz   = gs["Fz"]
        in_c = gs["in_contact"]

        _strut_lines[lbl].set_data([aw[0], wc[0]], [aw[1], wc[1]])
        _wheels[lbl].set_center((wc[0], wc[1]))

        cp = np.array([wc[0], wc[1] - r])   # contact point at ground

        if in_c:
            _contact_dots[lbl].set_data([cp[0]], [0.0])
        else:
            _contact_dots[lbl].set_data([], [])

        arr      = _arr_main   if lbl == "main" else _arr_nose
        lbl_text = _lbl_main   if lbl == "main" else _lbl_nose

        if in_c and Fz > 200.0:
            tip = cp + np.array([0.0, Fz * _FSCALE])
            arr.set_positions(tuple(cp), tuple(tip))
            lbl_text.set_position(tip + np.array([0.2, 0.0]))
            lbl_text.set_text(f"{Fz/1000:.1f} kN")
        else:
            arr.set_positions(_OFF, _OFF)
            lbl_text.set_position(_OFF)
            lbl_text.set_text("")

    # ── Aerodynamic arrows ───────────────────────────────────────────────────
    ai = _aero_hist[i]
    L  = ai["L"]

    lift_tip = cg + ai["lift_dir"] * L * _LSCALE
    _arr_lift.set_positions(tuple(cg), tuple(lift_tip))
    _lbl_lift.set_position(lift_tip + np.array([0.15, 0.0]))
    _lbl_lift.set_text(f"L={L/1000:.1f} kN")

    W     = MASS * G
    w_tip = cg + np.array([0.0, -W * _LSCALE])
    _arr_weight.set_positions(tuple(cg), tuple(w_tip))
    _lbl_weight.set_position(w_tip + np.array([0.15, 0.0]))
    _lbl_weight.set_text(f"W={W/1000:.1f} kN")

    # ── Status text ──────────────────────────────────────────────────────────
    _status.set_text(
        f"t = {t:5.2f} s    V = {V:5.1f} m/s    θ = {np.degrees(theta):+5.1f}°    "
        f"α = {np.degrees(ai['alpha']):+4.1f}°\n"
        f"F_main = {_Fz_m[i]/1000:5.1f} kN    "
        f"F_nose = {_Fz_n[i]/1000:5.1f} kN    "
        f"L/W = {L/(MASS*G):.2f}"
    )

    # ── Live time series ─────────────────────────────────────────────────────
    _live_fzm.set_data(_t[:i+1], _Fz_m[:i+1])
    _live_fzn.set_data(_t[:i+1], _Fz_n[:i+1])
    _live_hm.set_data(_t[:i+1],  _h_m[:i+1])
    _live_hn.set_data(_t[:i+1],  _h_n[:i+1])
    _cur_f.set_xdata([t, t])
    _cur_h.set_xdata([t, t])

    return (
        _p_fus, _p_wing, _p_ht, _p_vt, _cg_dot,
        *_strut_lines.values(), *_wheels.values(), *_contact_dots.values(),
        _arr_main, _arr_nose, _arr_lift, _arr_weight,
        _lbl_main, _lbl_nose, _lbl_lift, _lbl_weight,
        _status, _live_fzm, _live_fzn, _live_hm, _live_hn,
        _cur_f, _cur_h,
    )


fig.suptitle(
    "Landing Gear Touchdown  —  2-D Contact Force Animation",
    color="#ccccee", fontsize=13, fontweight="bold", y=0.995,
)

anim = animation.FuncAnimation(
    fig, _update,
    frames=len(_FRAMES),
    interval=1000 // _FPS,
    blit=False,   # blit=False: axvline cursor updates need full redraw
)

# Optionally save:  anim.save("touchdown.mp4", fps=_FPS, dpi=120)

plt.show()
