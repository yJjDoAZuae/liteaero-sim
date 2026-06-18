"""Generate tire-dynamics verification plots.

Writes ONLY files named tire_*.png into docs/defects/img/ (does not touch any other
figures). Run from the repo root:

    python docs/defects/img/plot_tire_dynamics.py \
        build/test/tire_freeroll_sweep.csv build/test/oq_lg15_diagnostic.csv
"""

import sys
import pathlib
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUTDIR = pathlib.Path(__file__).resolve().parent   # docs/defects/img/

sweep_csv = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else pathlib.Path(
    "build/test/tire_freeroll_sweep.csv")
diag_csv = pathlib.Path(sys.argv[2]) if len(sys.argv) > 2 else pathlib.Path(
    "build/test/oq_lg15_diagnostic.csv")

# ---------------------------------------------------------------------------
# Figure 1 — single free-rolling tire: longitudinal force vs contact speed.
# ---------------------------------------------------------------------------
s = np.genfromtxt(sweep_csv, delimiter=",", names=True)
fig, ax = plt.subplots(figsize=(8, 5))
ax.axhline(0, color="0.7", lw=0.8)
ax.axvline(0, color="0.7", lw=0.8)
ax.plot(s["v_cx_mps"], s["F_x_pacejka_n"], label="F_x (Pacejka longitudinal / traction)",
        color="tab:red", lw=2.5)
ax.plot(s["v_cx_mps"], s["F_long_n"], label="F_long = F_x + rolling resistance",
        color="tab:blue", lw=1.5, ls="--")
ax.set_xlabel("contact-patch longitudinal speed  V_cx  (m/s)")
ax.set_ylabel("longitudinal contact force  (N)   [+ = forward]")
ax.set_title("Free-rolling tire: longitudinal force vs contact speed\n"
             "F_x (traction) ≡ 0 for both signs of V_cx — tire cannot propel; "
             "F_long is pure rolling resistance (opposes motion)")
ax.legend(loc="upper right")
ax.grid(True, alpha=0.3)
f1 = OUTDIR / "tire_freeroll_sweep.png"
fig.tight_layout(); fig.savefig(f1, dpi=110); plt.close(fig)
print("Saved", f1)

# ---------------------------------------------------------------------------
# Figure 2 — aircraft-level ground roll: tire traction vs total contact force.
# Shows the tire (Pacejka F_x) stays ≈ 0 while the total wind-frame contact force
# carries the (separate) normal-projection / strut effect.
# ---------------------------------------------------------------------------
d = np.genfromtxt(diag_csv, delimiter=",", names=True)
t = d["time_s"]
fig, (a0, a1) = plt.subplots(2, 1, figsize=(9, 6.5), sharex=True)

a0.plot(t, d["nose_Fx"], color="tab:red", lw=0.8)
a0.set_ylabel("nose tire F_x  (N)")
a0.set_title("Aircraft ground roll: tire longitudinal (traction) force ≈ 0 throughout "
             "— the tire never propels")
a0.grid(True, alpha=0.3)
a0.set_ylim(-5, 5)

a1.plot(t, d["contact_fx_wind_n"], color="tab:gray", lw=0.6,
        label="total contact force, wind-x (incl. normal projection F_z·sinγ)")
a1.plot(t, d["nose_Fx"], color="tab:red", lw=0.9, label="nose tire F_x (traction)")
a1.axhline(0, color="0.7", lw=0.8)
a1.set_ylabel("longitudinal force (N)")
a1.set_xlabel("time (s)")
a1.set_title("Total contact force carries the separate strut/FPA normal-projection effect; "
             "the tire contributes ≈ 0")
a1.legend(loc="upper right", fontsize=8)
a1.grid(True, alpha=0.3)
f2 = OUTDIR / "tire_aircraft_fx.png"
fig.tight_layout(); fig.savefig(f2, dpi=110); plt.close(fig)
print("Saved", f2)

print(f"\nFree-roll sweep: max |F_x_pacejka| = {np.max(np.abs(s['F_x_pacejka_n'])):.4f} N")
print(f"Aircraft run:    max |nose tire F_x| = {np.max(np.abs(d['nose_Fx'])):.4f} N")
