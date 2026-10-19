"""
Plot a recorded lowering: the hand's 6 velocities (and its z force) against time.

    python3 plot_descent.py                      # the latest ~/.table_place/descents/*.csv
    python3 plot_descent.py path/to/descent_....csv

Writes a PNG next to the CSV. Every `place` lowering on the kinova backend records one
(kinova_arm.LOG_DIR): every /ee_state message from impedance-on to the end of the hold, with the
phase it was in (settle, descend, contact, open, retreat, done). Time 0 = the descent starts;
the dashed lines mark where each phase began.
"""

import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

LOG_DIR = Path.home() / ".table_place" / "descents"

SURFACE = "#fcfcfb"
SERIES = "#2a78d6"        # one series per panel: the first categorical slot
INK = "#3d3d3a"           # text
MUTED = "#8a8980"         # axes, phase markers
GRID = "#e8e7e1"

PANELS = [   # column, label, scale
    ("z", "z  (cm)", 100.0),
    ("vx", "vx  (cm/s)", 100.0),
    ("vy", "vy  (cm/s)", 100.0),
    ("vz", "vz  (cm/s)", 100.0),
    ("wx", "wx  (deg/s)", 57.29578),
    ("wy", "wy  (deg/s)", 57.29578),
    ("wz", "wz  (deg/s)", 57.29578),
    ("fz", "force z  (N)", 1.0),
]


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit(f"{path} is empty")
    return rows


def main():
    if len(sys.argv) > 1:
        path = Path(sys.argv[1])
    else:
        logs = sorted(LOG_DIR.glob("descent_*.csv"))
        if not logs:
            sys.exit(f"No recordings in {LOG_DIR}: run place_bowl.py place ... --execute first.")
        path = logs[-1]
    rows = load(path)
    t_all = [float(r["t"]) for r in rows]
    phase = [r["phase"] for r in rows]
    start = next((t for t, p in zip(t_all, phase) if p == "descend"), t_all[0])
    contact = next((t for t, p in zip(t_all, phase) if p == "contact"), None)
    t = [v - start for v in t_all]
    names = {"descend": "descent starts", "contact": "contact", "open": "gripper opens",
             "retreat": "retreat", "done": "retreat done"}
    marks = [(t[i], names.get(p, p)) for i, p in enumerate(phase)
             if i > 0 and p != phase[i - 1] and p != "settle"]
    if phase and phase[0] == "descend":
        marks.insert(0, (t[0], names["descend"]))

    plt.rcParams.update({"font.size": 9, "text.color": INK, "axes.labelcolor": INK,
                         "xtick.color": MUTED, "ytick.color": MUTED})
    fig, axes = plt.subplots(len(PANELS), 1, sharex=True, figsize=(9, 13.5), facecolor=SURFACE)
    for ax, (col, label, scale) in zip(axes, PANELS):
        ys = [float(r[col]) * scale for r in rows]
        ax.set_facecolor(SURFACE)
        ax.plot(t, ys, color=SERIES, linewidth=1.5)
        if col != "z":            # heights: zoom to their range (a few mm must show)
            ax.axhline(0.0, color=MUTED, linewidth=0.8)
        ax.set_ylabel(label, rotation=0, ha="right", va="center")
        ax.grid(True, axis="y", color=GRID, linewidth=0.8)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            ax.spines[side].set_color(MUTED)
        for when, _name in marks:
            ax.axvline(when, color=MUTED, linewidth=1.0, linestyle="--")
    for k, (when, name) in enumerate(marks):
        axes[0].annotate(name, (when, 1.0), xycoords=("data", "axes fraction"),
                         xytext=(3, 2 + 9 * (k % 2)), textcoords="offset points", fontsize=8, color=INK)
    axes[-1].set_xlabel("time from the start of the descent (s)")
    fig.suptitle(f"Hand while lowering and backing out, arm_mount frame  ({path.name})", color=INK, fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.98))
    out = path.with_suffix(".png")
    fig.savefig(out, dpi=120, facecolor=SURFACE)
    print(f"{len(rows)} samples, {t[-1] - t[0]:.1f} s; "
          + (f"contact {contact - start:.2f} s into the descent" if contact else "no contact recorded")
          + f" -> {out}")


if __name__ == "__main__":
    main()
