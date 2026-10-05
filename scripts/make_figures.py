#!/usr/bin/env python3
"""Regenerate every figure in figures/ from the stored trajectories.

Usage (from the repository root):

    python scripts/make_figures.py
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D

import evaluate as ev
import trajectory_utils as tu

FIGURES = os.path.join(ev.ROOT, "figures")

# One colour per estimator, everywhere. Ground truth is neutral ink.
COLOR = {"dmvio": "#2a78d6", "dso": "#eb6834"}
LABEL = {"dmvio": "DM-VIO", "dso": "DSO"}
INK, INK_2, MUTED = "#0b0b0b", "#52514e", "#898781"
GRID, AXIS, SURFACE = "#e1e0d9", "#c3c2b7", "#fcfcfb"

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "font.size": 10, "text.color": INK, "axes.labelcolor": INK_2,
    "axes.edgecolor": AXIS, "axes.linewidth": 0.8,
    "axes.spines.top": False, "axes.spines.right": False,
    "axes.titlesize": 11, "axes.titleweight": "bold", "axes.titlelocation": "left",
    "xtick.color": MUTED, "ytick.color": MUTED, "xtick.labelsize": 9, "ytick.labelsize": 9,
    "grid.color": GRID, "grid.linewidth": 0.6, "axes.grid": True, "axes.axisbelow": True,
    "legend.frameon": False, "legend.fontsize": 9,
})


def _save(fig, name):
    path = os.path.join(FIGURES, name)
    fig.savefig(path, dpi=170, bbox_inches="tight")
    plt.close(fig)
    print("wrote", os.path.relpath(path, ev.ROOT))


def _gt_segments(ax, p_gt, split, **kw):
    """Draw start and end ground-truth segments without joining them."""
    ax.plot(p_gt[:split, 0], p_gt[:split, 1], color=INK, lw=2.4, solid_capstyle="round", **kw)
    ax.plot(p_gt[split:, 0], p_gt[split:, 1], color=INK, lw=2.4, solid_capstyle="round")


def fig_magistrale5(res):
    """Hero figure: both estimators aligned on the first 50 s only."""
    r = res["tumvi"]["magistrale5"]
    fig, axes = plt.subplots(1, 2, figsize=(9, 8), sharex=True, sharey=True)
    headline = {
        "dmvio": "DM-VIO  -  back in the room {:.1f} m off",
        "dso": "DSO  -  scale drifts {:.1f}x, {:.0f} m off",
    }
    for ax, key in zip(axes, ("dmvio", "dso")):
        m = r[key]
        p = tu.apply_transform(m["p_est"], *m["T_start"])
        ax.plot(p[:, 0], p[:, 1], color=COLOR[key], lw=1.6, label=LABEL[key] + " estimate")
        _gt_segments(ax, m["p_gt"], m["gt_split"], label="Motion-capture ground truth")
        ax.plot(*p[0, :2], "o", ms=8, color=SURFACE, mec=INK, mew=1.6, zorder=5)
        ax.plot(*p[-1, :2], "o", ms=8, color=COLOR[key], mec=SURFACE, mew=1.5, zorder=5)
        if key == "dmvio":  # start and end markers coincide at this zoom level
            ax.annotate("start and estimated end", p[0, :2], xytext=(10, 12),
                        textcoords="offset points", color=INK_2, fontsize=9)
        else:
            ax.annotate("start", p[0, :2], xytext=(10, 12), textcoords="offset points",
                        color=INK_2, fontsize=9)
            ax.annotate("estimated end", p[-1, :2], xytext=(10, -3), textcoords="offset points",
                        color=INK_2, fontsize=9)
        if key == "dmvio":
            ax.set_title(headline[key].format(m["end_error_mean_m"]))
        else:
            ax.set_title(headline[key].format(
                max(m["scale_drift"], 1 / m["scale_drift"]), m["end_error_mean_m"]))
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("x (m)")
        ax.legend(loc="upper left")
    axes[0].set_ylabel("y (m)")
    fig.suptitle("TUM-VI magistrale5: a 446 s walk that starts and ends in the mocap room",
                 x=0.1, y=0.985, ha="left", fontsize=12.5, fontweight="bold")
    fig.text(0.1, 0.915, "Top-down view. Each trajectory is Sim(3)-aligned to ground truth using the "
             "first 50 s only,\nso the error at the end is drift accumulated over the whole walk.",
             color=INK_2, fontsize=9.5, linespacing=1.4)
    fig.subplots_adjust(top=0.86, wspace=0.08)
    _save(fig, "magistrale5_drift.png")


def fig_ate_summary(cfg, res):
    """Dot plot of ATE per sequence on a log axis (values span 2 mm to 26 cm)."""
    names = [s["name"] for s in cfg["tumvi"] if "ate_m" in res["tumvi"][s["name"]]["dmvio"]]
    fig, ax = plt.subplots(figsize=(8.2, 3.6))
    for i, name in enumerate(names):
        r = res["tumvi"][name]
        vals = {k: 100 * r[k]["ate_m"] for k in ("dmvio", "dso")}
        ax.plot(list(vals.values()), [i, i], color=AXIS, lw=2, zorder=1)
        lo = min(vals, key=vals.get)
        for k, v in vals.items():
            ax.plot(v, i, "o", ms=11, color=COLOR[k], mec=SURFACE, mew=2, zorder=3)
            left = k == lo
            ax.annotate("{:.2f} cm".format(v) if v < 10 else "{:.1f} cm".format(v), (v, i),
                        xytext=(-11 if left else 11, 0), textcoords="offset points",
                        ha="right" if left else "left", va="center", fontsize=9.5, color=INK)
    ax.set_xscale("log")
    ax.set_xlim(0.06, 90)
    ax.set_xticks([0.1, 1, 10])
    ax.set_xticklabels(["0.1 cm", "1 cm", "10 cm"])
    ax.set_yticks(range(len(names)))
    ax.set_yticklabels(names, color=INK, fontsize=10)
    ax.set_ylim(len(names) - 0.5, -0.5)
    ax.grid(axis="y", visible=False)
    ax.spines["left"].set_visible(False)
    ax.tick_params(axis="y", length=0)
    ax.set_xlabel("ATE RMSE after Sim(3) alignment, log scale (lower is better)")
    ax.set_title("Position error on the TUM-VI motion-capture sequences", pad=28)
    ax.legend(handles=[Line2D([], [], marker="o", ls="", ms=9, color=COLOR[k], label=LABEL[k])
                       for k in ("dmvio", "dso")],
              loc="lower left", bbox_to_anchor=(-0.01, 1.0), ncol=2, handletextpad=0.2, columnspacing=1.2)
    _save(fig, "ate_summary.png")


def fig_error_over_time(cfg, res):
    """Per-pose position error against time, one panel per sequence."""
    names = [s["name"] for s in cfg["tumvi"] if "ate_m" in res["tumvi"][s["name"]]["dmvio"]]
    fig, axes = plt.subplots(2, 2, figsize=(10.5, 6), sharey=True)
    for ax, name in zip(axes.ravel(), names):
        for k in ("dso", "dmvio"):
            m = res["tumvi"][name][k]
            ax.plot(m["err_t"], 100 * m["err"], color=COLOR[k], lw=1.6, label=LABEL[k])
        ax.set_yscale("log")
        ax.set_ylim(0.01, 200)
        ax.set_yticks([0.1, 1, 10, 100])
        ax.set_yticklabels(["0.1 cm", "1 cm", "10 cm", "1 m"])
        ax.set_title(name)
    for ax in axes[1]:
        ax.set_xlabel("time since first pose (s)")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles[::-1], labels[::-1], loc="upper right", bbox_to_anchor=(0.985, 0.985), ncol=2)
    fig.suptitle("Where the error comes from: position error of every pose (log scale)",
                 x=0.06, ha="left", fontsize=12.5, fontweight="bold")
    fig.subplots_adjust(top=0.88, hspace=0.32, wspace=0.06)
    _save(fig, "error_over_time.png")


def fig_midterm_dso(cfg, res):
    """Mid-term: DSO alone on TUM monoVO loops and EuRoC."""
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.6))
    for ax, seq in zip(axes[:2], cfg["tum_mono"]):
        m = res["tum_mono"][seq["name"]]
        p = tu.apply_transform(m["p_est"], *m["T_start"])
        # The motion-capture frame is not gravity-aligned the same way in every
        # sequence, so show the two axes the trajectory actually moves along.
        i, j = sorted(np.argsort(np.ptp(p, axis=0))[1:])
        ax.plot(p[:, i], p[:, j], color=COLOR["dso"], lw=1.6, label="DSO estimate")
        _gt_segments(ax, m["p_gt"][:, [i, j]], m["gt_split"], label="Ground truth (start / end)")
        ax.set_title("TUM monoVO {}\nscale drift {:.2f}x, e_align {:.2f} m".format(
            seq["name"], max(m["scale_drift"], 1 / m["scale_drift"]), m["e_align_m"]))
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("{} (m)".format("xyz"[i]))
        ax.set_ylabel("{} (m)".format("xyz"[j]))

    _, p, _ = tu.load_tum(os.path.join(ev.ROOT, cfg["euroc"][0]["dir"], "dso_result.txt"))
    ax = axes[2]
    ax.plot(p[:, 0], p[:, 2], color=COLOR["dso"], lw=1.6)
    ax.set_title("EuRoC {}\n{} keyframe poses, DSO frame".format(cfg["euroc"][0]["name"], len(p)))
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (arbitrary scale)")
    ax.set_ylabel("z (arbitrary scale)")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper left", bbox_to_anchor=(0.06, 1.0), ncol=2)
    fig.subplots_adjust(top=0.82, wspace=0.28)
    _save(fig, "midterm_dso_trajectories.png")


def main():
    os.makedirs(FIGURES, exist_ok=True)
    cfg, res = ev.run_all()
    fig_magistrale5(res)
    fig_ate_summary(cfg, res)
    fig_error_over_time(cfg, res)
    fig_midterm_dso(cfg, res)


if __name__ == "__main__":
    main()
