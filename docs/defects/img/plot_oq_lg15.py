"""Generate OQ-LG-15 diagnostic plots from oq_lg15_diagnostic.csv.

Run from the repo root:
    python docs/defects/img/plot_oq_lg15.py build/test/oq_lg15_diagnostic.csv
"""

import sys
import pathlib
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

# ---------------------------------------------------------------------------
# Load data
# ---------------------------------------------------------------------------

csv_path = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else pathlib.Path(
    "build/test/oq_lg15_diagnostic.csv")
data = np.genfromtxt(csv_path, delimiter=",", names=True)

t       = data["time_s"]
vN      = data["v_north_mps"]
vD      = data["v_down_mps"]
alt     = data["altitude_m"]
d_nose  = data["strut_nose_m"]
d_left  = data["strut_left_m"]
d_right = data["strut_right_m"]
fz_body = data["contact_fz_body_n"]
fx_ned  = data["contact_fx_ned_n"]
fx_wind = data["contact_fx_wind_n"]
fz_wind = data["contact_fz_wind_n"]
pwr_h   = data["contact_power_horiz_w"]
pwr_t   = data["contact_power_total_w"]
pitch   = data["pitch_deg"]
alpha   = data["alpha_deg"]
fpa     = data["fpa_deg"]
wow     = data["wow"].astype(bool)

CONTACT_ALT  = 0.700    # nose attach_z (0.5) + tyre_radius (0.2)
STATIC_DEFL  = 10255.0 / 60000.0   # mg / k_total = 0.171 m (all-3-wheel static)
NOSE_STATIC  = 10255.0 / 20000.0   # mg / k_nose (1-wheel static upper bound) = 0.513 m

nose_contact  = d_nose  > 1e-4
left_contact  = d_left  > 1e-4
right_contact = d_right > 1e-4

# Time windows used throughout
wt  = (t >= 290) & (t <= 300)
t_z = t[wt]
mt  = (t >= 34)  & (t <= 45)
t_m = t[mt]

# Pitch convention: positive pitch_deg = nose DOWN (verified from data: contact events
# occur at positive pitch values, consistent with nose-down geometry)
pitch_rad = np.radians(pitch)

# Per-wheel contact patch AGL (m above terrain).  Negative = in contact.
# NED-z of attach relative to body CG (positive = below CG):
#   NED-z = -sin(pitch_physical) * x_body + cos(pitch_physical) * z_body
# With positive pitch_deg = nose DOWN => pitch_physical = -pitch_deg =>
#   -sin(-pitch_rad)*x = sin(pitch_rad)*x
def _wheel_agl(alt_arr, pr_arr, attach_xyz):
    x_b, _y_b, z_b = attach_xyz
    ned_z = -np.sin(pr_arr) * x_b + np.cos(pr_arr) * z_b
    return alt_arr - ned_z - 0.2   # 0.2 m tyre radius

agl_nose  = _wheel_agl(alt, pitch_rad, (2.0,  0.0, 0.5))
agl_left  = _wheel_agl(alt, pitch_rad, (-0.5, -1.0, 0.5))
agl_right = _wheel_agl(alt, pitch_rad, (-0.5,  1.0, 0.5))

out_dir = pathlib.Path(__file__).parent
plt.rcParams.update({
    "font.size": 9, "axes.titlesize": 9, "axes.labelsize": 8,
    "legend.fontsize": 7, "figure.dpi": 150,
})

# helper: shade contact regions on an axes
def shade_contact(ax, t_arr, contact_bool, color, alpha_val=0.18):
    for i in range(len(t_arr) - 1):
        if contact_bool[i]:
            ax.axvspan(t_arr[i], t_arr[i+1], color=color, alpha=alpha_val, lw=0)

# ---------------------------------------------------------------------------
# Figure 1 — Forward speed 0–300 s
# ---------------------------------------------------------------------------

fig, ax = plt.subplots(figsize=(7, 3))
ax.plot(t, vN, lw=0.6, color="steelblue")
ax.axhline(0.5,  color="red",    ls="--", lw=0.8, label="Pass threshold 0.5 m/s")
ax.axhline(5.07, color="orange", ls=":",  lw=0.8, label="Terminal vel ~5.07 m/s at t=300 s")
ax.axvline(38.4, color="purple", ls=":",  lw=1.0, label="Main gear liftoff t=38.4 s")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Forward speed (m/s)")
ax.set_title("OQ-LG-15 — Forward speed 0–300 s (hard equilibrium at 5.07 m/s)")
ax.legend(loc="upper right")
ax.set_xlim(0, 300); ax.set_ylim(0, 16)
ax.grid(True, lw=0.3)
fig.tight_layout()
fig.savefig(out_dir / "oq_lg15_forward_speed.png")
plt.close(fig)
print("Saved oq_lg15_forward_speed.png")

# ---------------------------------------------------------------------------
# Figure 2 — Full timeline: strut deflections + pitch (0–300 s)
# ---------------------------------------------------------------------------

fig, axes = plt.subplots(3, 1, figsize=(8, 7), sharex=True)

ax = axes[0]
ax.plot(t, d_nose,  lw=0.5, color="steelblue",  label="Nose strut (m)")
ax.plot(t, d_left,  lw=0.5, color="darkorange", label="Left main (m)", alpha=0.8)
ax.plot(t, d_right, lw=0.5, color="green",      label="Right main (m)", ls="--", alpha=0.8)
ax.axhline(STATIC_DEFL, color="gray", ls=":", lw=0.7,
           label=f"3-wheel static defl {STATIC_DEFL:.3f} m")
ax.axvline(38.4, color="purple", ls=":", lw=1.0, label="Main gear liftoff")
ax.set_ylabel("Strut deflection (m)")
ax.set_ylim(-0.005, 0.22)
ax.set_title("OQ-LG-15 — Strut deflections and pitch over full 300 s run\n"
             "Main gear lifts off at t=38.4 s, V=6.3 m/s; nose gear bouncing from t~36 s")
ax.legend(loc="upper right", ncol=3)
ax.grid(True, lw=0.3)

ax = axes[1]
ax.plot(t, vN, lw=0.5, color="steelblue", label="$V_N$ (m/s)")
ax.axvline(38.4, color="purple", ls=":", lw=1.0)
ax.set_ylabel("Forward speed (m/s)")
ax.set_ylim(0, 16)
ax.legend(loc="upper right")
ax.grid(True, lw=0.3)

ax = axes[2]
ax.plot(t, pitch, lw=0.4, color="darkred",  label="Pitch (deg, nose-up +)")
ax.plot(t, alpha, lw=0.4, color="steelblue", label="Alpha (deg)", ls="--")
ax.axhline(0, color="black", lw=0.5)
ax.axvline(38.4, color="purple", ls=":", lw=1.0)
ax.set_ylabel("Angle (deg)")
ax.set_xlabel("Time (s)")
ax.set_ylim(-5, 5)
ax.legend(loc="upper right")
ax.grid(True, lw=0.3)

axes[-1].set_xlim(0, 300)
fig.tight_layout()
fig.savefig(out_dir / "oq_lg15_full_timeline.png")
plt.close(fig)
print("Saved oq_lg15_full_timeline.png")

# ---------------------------------------------------------------------------
# Figure 3 — Wheelbarrow transition detail (t=34–45 s)
# ---------------------------------------------------------------------------

fig, axes = plt.subplots(3, 1, figsize=(7, 6.5), sharex=True)

ax = axes[0]
ax.plot(t_m, d_nose[mt],  lw=0.9, color="steelblue",  label="Nose strut (m)")
ax.plot(t_m, d_left[mt],  lw=0.9, color="darkorange",  label="Left main (m)")
ax.plot(t_m, d_right[mt], lw=0.9, color="green",       label="Right main (m)", ls="--")
ax.axhline(STATIC_DEFL, color="gray", ls=":", lw=0.7,
           label=f"3-wheel static defl {STATIC_DEFL:.3f} m")
ax.axvline(38.4, color="purple", ls=":", lw=1.2)
ax.set_ylabel("Strut defl. (m)")
ax.set_ylim(-0.005, 0.22)
ax.set_title("OQ-LG-15 — Wheelbarrowing transition detail, t = 34–45 s\n"
             "Left/right main deflect to ~0.015 m then leave ground at V~6.3 m/s")
ax.legend(loc="upper right", ncol=2)
ax.grid(True, lw=0.3)

ax = axes[1]
ax.plot(t_m, pitch[mt], lw=0.9, color="darkred",   label="Pitch (deg)")
ax.plot(t_m, alpha[mt], lw=0.9, color="steelblue", label="Alpha (deg)", ls="--")
ax.axhline(0, color="black", lw=0.5)
ax.axvline(38.4, color="purple", ls=":", lw=1.2, label="Main gear liftoff")
ax.set_ylabel("Angle (deg)")
ax.set_ylim(-4, 4)
ax.legend(loc="upper right", ncol=2)
ax.grid(True, lw=0.3)

ax = axes[2]
ax.plot(t_m, vN[mt], lw=0.9, color="steelblue", label="$V_N$ (m/s)")
ax.axvline(38.4, color="purple", ls=":", lw=1.2)
ax.set_xlabel("Time (s)")
ax.set_ylabel("Fwd speed (m/s)")
ax.set_ylim(4, 8)
ax.legend(loc="upper right")
ax.grid(True, lw=0.3)

fig.tight_layout()
fig.savefig(out_dir / "oq_lg15_wheelbarrow_transition.png")
plt.close(fig)
print("Saved oq_lg15_wheelbarrow_transition.png")

# ---------------------------------------------------------------------------
# Figure 3b — Per-wheel contact patch AGL, zoomed 290–300 s
#
# Contact patch AGL = body_altitude - NED_z(attach) - tyre_radius
# NED_z(attach) = sin(pitch_physical) * x_body + cos(pitch_physical) * z_body
#
# Sign convention: from the data, nose contacts at POSITIVE pitch_deg values,
# meaning pitch_deg > 0 = nose DOWN (θ_physical = −pitch_deg).
# Therefore: sin(θ_physical) = −sin(pitch_deg_rad)
# NED_z(attach) = −sin(pitch_deg_rad) * x_body + cos(pitch_deg_rad) * z_body
# ---------------------------------------------------------------------------

# (masks, pitch_rad, agl_* all defined at module level above)

fig, axes = plt.subplots(3, 1, figsize=(8, 7), sharex=True)

wheels_agl = [
    ("Nose contact patch AGL (m)",       agl_nose,  "steelblue"),
    ("Left main contact patch AGL (m)",  agl_left,  "darkorange"),
    ("Right main contact patch AGL (m)", agl_right, "green"),
]

for ax, (label, agl, color) in zip(axes, wheels_agl):
    ax.plot(t_z, agl[wt], lw=0.8, color=color, label=label)
    ax.axhline(0, color="red", ls="--", lw=0.9, label="Ground (0 m)")
    # Shade contact (AGL < 0)
    in_contact_agl = agl[wt] < 0
    for i in range(len(t_z) - 1):
        if in_contact_agl[i]:
            ax.axvspan(t_z[i], t_z[i+1], color=color, alpha=0.2, lw=0)
    ax.plot([], [], color=color, alpha=0.4, lw=6, label="Below ground (contact)")
    ax.set_ylabel("AGL (m)")
    ax.legend(loc="upper right", ncol=2)
    ax.grid(True, lw=0.3)

axes[0].set_title("OQ-LG-15 — Contact patch altitude above ground, t = 290–300 s\n"
                  "Shows WHY main gear is airborne while nose bounces: different AGL due to pitch")
axes[-1].set_xlabel("Time (s)")
fig.tight_layout()
fig.savefig(out_dir / "oq_lg15_wheel_agl.png")
plt.close(fig)
print("Saved oq_lg15_wheel_agl.png")

# ---------------------------------------------------------------------------
# Figure 4 — Strut deflections zoomed 290–300 s (each wheel own axes)
# ---------------------------------------------------------------------------

fig, axes = plt.subplots(3, 1, figsize=(7, 7), sharex=True)

# Nose strut: data range 0–0.084 m, auto-scale with padding
nose_max = d_nose[wt].max()
ax = axes[0]
ax.plot(t_z, d_nose[wt], lw=0.9, color="steelblue", label="Nose strut")
ax.axhline(STATIC_DEFL, color="gray", ls=":", lw=0.8,
           label=f"3-wheel static defl {STATIC_DEFL:.3f} m  (nose-only static would be {NOSE_STATIC:.2f} m)")
shade_contact(ax, t_z, nose_contact[wt], "steelblue")
ax.plot([], [], color="steelblue", alpha=0.3, lw=6, label="Nose in contact")
ax.set_ylabel("Defl. (m)")
ax.set_ylim(-0.005, max(nose_max * 1.3, 0.03))
ax.set_title("OQ-LG-15 — Per-wheel strut deflections, t = 290–300 s\n"
             "Main gear struts are ZERO throughout (aircraft wheelbarrowing on nose only)")
ax.legend(loc="upper right", ncol=2, fontsize=6.5)
ax.grid(True, lw=0.3)

# Left main: all zero — use a scale that makes the zero line clearly visible and unambiguous
ax = axes[1]
ax.plot(t_z, d_left[wt],  lw=0.9, color="darkorange", label="Left main strut")
ax.axhline(0, color="black", lw=0.5, ls="--")
ax.text(0.5, 0.55, "Zero throughout — main gear airborne",
        transform=ax.transAxes, ha="center", fontsize=8, color="darkred",
        bbox=dict(boxstyle="round", fc="lightyellow", ec="orange", alpha=0.8))
ax.set_ylabel("Defl. (m)")
ax.set_ylim(-0.002, 0.010)
ax.legend(loc="upper right")
ax.grid(True, lw=0.3)

# Right main: same
ax = axes[2]
ax.plot(t_z, d_right[wt], lw=0.9, color="green", label="Right main strut")
ax.axhline(0, color="black", lw=0.5, ls="--")
ax.text(0.5, 0.55, "Zero throughout — main gear airborne",
        transform=ax.transAxes, ha="center", fontsize=8, color="darkred",
        bbox=dict(boxstyle="round", fc="lightyellow", ec="green", alpha=0.8))
ax.set_xlabel("Time (s)")
ax.set_ylabel("Defl. (m)")
ax.set_ylim(-0.002, 0.010)
ax.legend(loc="upper right")
ax.grid(True, lw=0.3)

fig.tight_layout()
fig.savefig(out_dir / "oq_lg15_strut_deflection.png")
plt.close(fig)
print("Saved oq_lg15_strut_deflection.png")

# ---------------------------------------------------------------------------
# Figure 5 — VN+VD coupled with contact shading  (290–300 s)
# ---------------------------------------------------------------------------

fig, axes = plt.subplots(2, 1, figsize=(7, 5), sharex=True)

ax = axes[0]
ax.plot(t_z, vN[wt], lw=0.9, color="steelblue", label="$V_N$ forward speed (m/s)")
shade_contact(ax, t_z, nose_contact[wt], "salmon")
ax.plot([], [], color="salmon", alpha=0.5, lw=6, label="Nose contact")
ax.set_ylabel("Fwd speed (m/s)")
ax.set_title("OQ-LG-15 — Forward and vertical speed, t = 290–300 s\n"
             "VN sawtooth pattern coincides with nose contact events (shaded)")
ax.legend(loc="upper right")
ax.grid(True, lw=0.3)

ax = axes[1]
ax.plot(t_z, vD[wt], lw=0.9, color="darkorange", label="$V_D$ (m/s, down +)")
ax.axhline(0, color="black", lw=0.5)
shade_contact(ax, t_z, nose_contact[wt], "salmon")
ax.plot([], [], color="salmon", alpha=0.5, lw=6, label="Nose contact")
# WoW shading as narrow top band — positive, in legend
for i in range(len(t_z) - 1):
    if wow[wt][i]:
        ax.axvspan(t_z[i], t_z[i+1], ymin=0.88, ymax=1.0,
                   color="steelblue", alpha=0.6, lw=0)
ax.plot([], [], color="steelblue", alpha=0.6, lw=6, label="WoW=1 (top band)")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Vert speed (m/s, down+)")
ax.legend(loc="lower right", ncol=2)
ax.grid(True, lw=0.3)

fig.tight_layout()
fig.savefig(out_dir / "oq_lg15_vn_vd_coupled.png")
plt.close(fig)
print("Saved oq_lg15_vn_vd_coupled.png")

# ---------------------------------------------------------------------------
# Figure 6 — Contact energy: horizontal (NED-x) vs total  (290–300 s)
# ---------------------------------------------------------------------------

fig, axes = plt.subplots(3, 1, figsize=(7, 7), sharex=True)

ax = axes[0]
ax.plot(t_z, fx_ned[wt], lw=0.9, color="steelblue",
        label="$F_{contact,NED-x}$ (N, north = forward)")
ax.axhline(0, color="black", lw=0.5)
shade_contact(ax, t_z, nose_contact[wt], "salmon")
ax.plot([], [], color="salmon", alpha=0.5, lw=6, label="Nose contact")
ax.set_ylabel("NED-x force (N)")
ax.set_title("OQ-LG-15 — Contact force and power decomposition, t = 290–300 s")
ax.legend(loc="upper right", ncol=2)
ax.grid(True, lw=0.3)

ax = axes[1]
ax.plot(t_z, pwr_h[wt], lw=0.9, color="darkorange",
        label="Horizontal power $F_{NED-x} \\cdot V_N$ (W)")
ax.axhline(0, color="black", lw=0.5)
shade_contact(ax, t_z, nose_contact[wt], "salmon")
h_net = np.trapezoid(pwr_h[wt], t_z)
ax.text(0.02, 0.08, f"Net horiz. work 290-300 s: {h_net:.0f} J  ({h_net/10:.0f} W avg)",
        transform=ax.transAxes, fontsize=8, color="darkred")
ax.set_ylabel("Horiz. power (W)")
ax.legend(loc="upper right")
ax.grid(True, lw=0.3)

ax = axes[2]
ax.plot(t_z, pwr_t[wt], lw=0.9, color="purple",
        label="Total power $F_{NED} \\cdot v_{NED}$ (W)")
ax.axhline(0, color="black", lw=0.5)
shade_contact(ax, t_z, nose_contact[wt], "salmon")
t_net = np.trapezoid(pwr_t[wt], t_z)
ax.text(0.02, 0.08, f"Net total work 290-300 s: {t_net:.0f} J  ({t_net/10:.0f} W avg)",
        transform=ax.transAxes, fontsize=8, color="darkred")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Total power (W)")
ax.legend(loc="upper right")
ax.grid(True, lw=0.3)

fig.tight_layout()
fig.savefig(out_dir / "oq_lg15_contact_power.png")
plt.close(fig)
print("Saved oq_lg15_contact_power.png")

# ---------------------------------------------------------------------------
# Figure 7 — Altitude and WoW  (290–300 s, fixed WoW display)
# ---------------------------------------------------------------------------

fig, axes = plt.subplots(2, 1, figsize=(7, 5), sharex=True)

ax = axes[0]
ax.plot(t_z, alt[wt], lw=0.9, color="steelblue", label="Body altitude (m)")
ax.axhline(CONTACT_ALT, color="red", ls="--", lw=0.8,
           label=f"Level-attitude nose contact alt = {CONTACT_ALT:.3f} m")
ax.set_ylabel("Altitude AGL (m)")
ax.set_title("OQ-LG-15 — Altitude and WoW state, t = 290–300 s")
ax.legend(loc="upper right")
ax.grid(True, lw=0.3)

ax = axes[1]
ax.plot(t_z, vD[wt], lw=0.9, color="darkorange", label="$V_D$ (m/s, down+)")
ax.axhline(0, color="black", lw=0.5)
# WoW=1 indicator: narrow band at top of plot, with legend entry
for i in range(len(t_z) - 1):
    if wow[wt][i]:
        ax.axvspan(t_z[i], t_z[i+1], ymin=0.88, ymax=1.0,
                   color="steelblue", alpha=0.6, lw=0)
ax.plot([], [], color="steelblue", alpha=0.6, lw=6, label="WoW=1 (top band)")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Vert speed (m/s)")
ax.legend(loc="lower right")
ax.grid(True, lw=0.3)

fig.tight_layout()
fig.savefig(out_dir / "oq_lg15_altitude_bounce.png")
plt.close(fig)
print("Saved oq_lg15_altitude_bounce.png")

# ---------------------------------------------------------------------------
# Figure 8 — THE MECHANISM: backward-impulse train + periodic forward spike
#            Wind-frame gear force (what the EOM applies) and resulting VN.
#            Window 290–296 s shows ~2.5 limit-cycle periods.
# ---------------------------------------------------------------------------

mp = (t >= 290) & (t <= 296)
t_p = t[mp]

fig, axes = plt.subplots(2, 1, figsize=(8, 5.5), sharex=True)

ax = axes[0]
ax.plot(t_p, fx_wind[mp], lw=0.7, color="steelblue",
        label="$F_{gear,wind-x}$ (N, applied by EOM along velocity)")
ax.axhline(0, color="black", lw=0.5)
# Mark the big forward spikes
spikes = mp & (fx_wind > 5000)
ax.plot(t[spikes], fx_wind[spikes], "v", color="red", ms=7,
        label="Forward spike (+22.7 kN, deep penetration)")
ax.set_ylabel("Wind-x gear force (N)")
ax.set_title("OQ-LG-15 — THE MECHANISM: high-frequency backward impulse train\n"
             "punctuated by a periodic +22.7 kN forward spike every 2.41 s")
ax.legend(loc="upper right")
ax.grid(True, lw=0.3)

ax = axes[1]
ax.plot(t_p, vN[mp], lw=0.9, color="darkorange", label="$V_N$ forward speed (m/s)")
ax.plot(t[spikes], vN[spikes], "v", color="red", ms=7,
        label="+0.48 m/s jump at each forward spike")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Forward speed (m/s)")
ax.set_title("Each forward spike resets VN upward; backward impulses bleed it down between spikes",
             fontsize=8)
ax.legend(loc="upper right")
ax.grid(True, lw=0.3)

fig.tight_layout()
fig.savefig(out_dir / "oq_lg15_impulse_mechanism.png")
plt.close(fig)
print("Saved oq_lg15_impulse_mechanism.png")

# ---------------------------------------------------------------------------
# Figure 9 — Single forward-spike anatomy: strut, vD, pitch, force at one event
# ---------------------------------------------------------------------------

# Find a representative spike near t=294.42
spike_idx = np.where((t > 294) & (t < 295) & (fx_wind > 5000))[0]
if len(spike_idx):
    si = spike_idx[0]
    w = slice(si - 6, si + 6)
    t_w = t[w]

    fig, axes = plt.subplots(4, 1, figsize=(7, 8), sharex=True)
    axes[0].plot(t_w, d_nose[w], "o-", lw=0.9, color="steelblue", ms=3)
    axes[0].axvline(t[si], color="red", ls=":", lw=1.0)
    axes[0].set_ylabel("Nose strut\ndefl. (m)")
    axes[0].set_title("OQ-LG-15 — Anatomy of one forward-spike event (t ≈ %.2f s)\n"
                      "Deep single-step penetration → huge δ̇ → 22.7 kN impulse" % t[si])
    axes[0].grid(True, lw=0.3)

    axes[1].plot(t_w, vD[w], "o-", lw=0.9, color="darkorange", ms=3)
    axes[1].axhline(0, color="black", lw=0.5)
    axes[1].axvline(t[si], color="red", ls=":", lw=1.0)
    axes[1].set_ylabel("Vert speed\n$V_D$ (m/s, down+)")
    axes[1].grid(True, lw=0.3)

    axes[2].plot(t_w, pitch[w], "o-", lw=0.9, color="darkred", ms=3, label="Pitch")
    axes[2].plot(t_w, fpa[w], "s--", lw=0.9, color="green", ms=3, label="Flight path angle")
    axes[2].axhline(0, color="black", lw=0.5)
    axes[2].axvline(t[si], color="red", ls=":", lw=1.0)
    axes[2].set_ylabel("Angle (deg)")
    axes[2].legend(loc="upper right")
    axes[2].grid(True, lw=0.3)

    axes[3].plot(t_w, fx_wind[w], "o-", lw=0.9, color="purple", ms=3, label="$F_{wind-x}$")
    axes[3].plot(t_w, fx_ned[w], "s--", lw=0.9, color="gray", ms=3, label="$F_{NED-x}$")
    axes[3].axhline(0, color="black", lw=0.5)
    axes[3].axvline(t[si], color="red", ls=":", lw=1.0)
    axes[3].set_ylabel("Gear force\nx (N)")
    axes[3].set_xlabel("Time (s)")
    axes[3].legend(loc="upper left")
    axes[3].grid(True, lw=0.3)

    fig.tight_layout()
    fig.savefig(out_dir / "oq_lg15_spike_anatomy.png")
    plt.close(fig)
    print("Saved oq_lg15_spike_anatomy.png")

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------

delta0 = STATIC_DEFL
h_net_val  = np.trapezoid(pwr_h[wt], t_z)
t_net_val  = np.trapezoid(pwr_t[wt], t_z)
left_nz = d_left > 1e-5
last_left_t = t[left_nz][-1] if np.any(left_nz) else 0.0

contacts_z = nose_contact[wt].astype(int)
n_contacts = np.sum(np.diff(contacts_z) == 1)

print(f"\n--- Key findings ---")
print(f"Terminal speed at t=300 s:    {vN[-1]:.3f} m/s (hard equilibrium)")
print(f"Main gear leaves ground:       t={last_left_t:.2f} s  V~{vN[np.where(left_nz)[0][-1]]:.2f} m/s")
print(f"Max nose strut (290-300 s):    {d_nose[wt].max():.4f} m  (d0={delta0:.3f} m, nose-only static>{NOSE_STATIC:.2f} m)")
print(f"Nose contact fraction:         {nose_contact[wt].mean()*100:.1f}%")
print(f"Left/right contact fraction:   {left_contact[wt].mean()*100:.1f}% / {right_contact[wt].mean()*100:.1f}%")
print(f"Nose touchdowns in 290-300 s:  {n_contacts}  -> freq ~{n_contacts/10:.1f} Hz, period ~{10/max(n_contacts,1)*1000:.0f} ms")
print(f"Net horizontal work (NED-x):   {h_net_val:.0f} J  ({h_net_val/10:.0f} W avg)")
print(f"Net total contact work:        {t_net_val:.0f} J  ({t_net_val/10:.0f} W avg)")
print(f"Vertical component only:       {t_net_val - h_net_val:.0f} J  (total - horizontal)")
print(f"Alpha range (0-300 s):         {alpha.min():.2f} to {alpha.max():.2f} deg")
print(f"Pitch range (290-300 s):       {pitch[wt].min():.2f} to {pitch[wt].max():.2f} deg")
