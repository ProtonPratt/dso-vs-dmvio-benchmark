#!/usr/bin/env python3
"""Evaluate every stored DSO / DM-VIO trajectory against ground truth.

Usage (from the repository root):

    python scripts/evaluate.py

Reads configs/sequences.json, prints the result tables and writes
results/metrics.json and results/metrics.md.
"""
import json
import os

import numpy as np

import trajectory_utils as tu

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
METHODS = (("dmvio", "DM-VIO", "dmvio_result.txt"), ("dso", "DSO", "dso_result.txt"))


def load_config():
    with open(os.path.join(ROOT, "configs", "sequences.json")) as f:
        return json.load(f)


def _path(*parts):
    return os.path.join(ROOT, *parts)


def evaluate_tumvi(seq, est_file, cfg):
    """One estimator on one TUM-VI sequence.

    Sequences recorded entirely inside the motion-capture room get a
    lever-arm-compensated Sim(3) ATE. Sequences that leave the room
    (ground truth only at start and end) get the start/end drift metrics.
    """
    t_est, p_est, q_est = tu.load_tum(_path(seq["dir"], est_file))
    t_gt, p_gt = tu.load_ground_truth(_path(seq["dir"], "gt_imu.csv"))
    ie, ig = tu.associate(t_est, t_gt, cfg["max_association_dt_s"])

    out = {
        "poses": int(len(t_est)),
        "matched": int(len(ie)),
        "coverage_pct": 100.0 * (t_est[-1] - t_est[0]) / seq["duration_s"],
        "t_est": t_est, "p_est": p_est, "t_gt": t_gt, "p_gt": p_gt,
    }

    split = tu.split_at_gap(t_gt)
    if split is None:
        raw_rmse, _, _ = tu.ate(p_est[ie], p_gt[ig])
        rmse, err, T, p_imu = tu.ate_with_lever_arm(
            p_est[ie], tu.quat_to_rot(q_est[ie]), p_gt[ig], cfg["t_cam_imu"])
        out.update({
            "ate_raw_m": raw_rmse, "ate_m": rmse, "scale": T[0],
            "err_t": t_est[ie] - t_est[0], "err": err,
            "aligned": tu.apply_transform(p_imu, *T), "gt_matched": p_gt[ig],
        })
    else:
        is_start = t_est[ie] <= t_gt[split - 1]
        out.update(tu.start_end_metrics(p_est, p_est[ie], p_gt[ig], is_start))
        out["ate_joint_m"], _, _ = tu.ate(p_est[ie], p_gt[ig])
        out["gt_split"] = split
    return out


def evaluate_metric_scale(check, cfg):
    """DM-VIO's own metric output, aligned with a rigid SE(3) only (no scale).

    ``resultScaled.txt`` is already expressed in the IMU frame and in metres,
    so any scale the Sim(3) fit still wants to apply is DM-VIO's scale error.
    """
    t_est, p_est, _ = tu.load_tum(_path(check["metric"]))
    t_gt, p_gt = tu.load_ground_truth(_path(check["gt"]))
    ie, ig = tu.associate(t_est, t_gt, cfg["max_association_dt_s"])
    se3_rmse, _, _ = tu.ate(p_est[ie], p_gt[ig], with_scale=False)
    sim3_rmse, _, (s, _, _) = tu.ate(p_est[ie], p_gt[ig], with_scale=True)
    return {"ate_se3_m": se3_rmse, "ate_sim3_m": sim3_rmse, "scale_error_pct": 100.0 * abs(1.0 - s)}


def evaluate_tum_mono(seq, cfg):
    """DSO on a TUM monoVO loop: ground truth exists only at start and end."""
    t_est, p_est, _ = tu.load_tum(_path(seq["dir"], "dso_result.txt"))
    t_gt, p_gt = tu.load_ground_truth(_path(seq["dir"], "groundtruthSync.txt"))
    ie, ig = tu.associate(t_est, t_gt, cfg["max_association_dt_s"])
    split = tu.split_at_gap(t_gt)
    is_start = t_est[ie] <= t_gt[split - 1]
    out = tu.start_end_metrics(p_est, p_est[ie], p_gt[ig], is_start)
    out.update({
        "keyframes": int(len(t_est)),
        "coverage_pct": 100.0 * (t_est[-1] - t_est[0]) / seq["duration_s"],
        "t_est": t_est, "p_est": p_est, "t_gt": t_gt, "p_gt": p_gt, "gt_split": split,
    })
    return out


def run_all():
    cfg = load_config()
    res = {"tumvi": {}, "tum_mono": {}}
    for seq in cfg["tumvi"]:
        res["tumvi"][seq["name"]] = {
            key: evaluate_tumvi(seq, fname, cfg) for key, _, fname in METHODS}
    res["metric_scale_check"] = evaluate_metric_scale(cfg["metric_scale_check"], cfg)
    for seq in cfg["tum_mono"]:
        res["tum_mono"][seq["name"]] = evaluate_tum_mono(seq, cfg)
    return cfg, res


# --------------------------------------------------------------------------- #
# Reporting
# --------------------------------------------------------------------------- #
def markdown_report(cfg, res):
    lines = []
    w = lines.append

    w("### TUM-VI, full motion-capture ground truth")
    w("")
    w("Sim(3)-aligned, lever-arm-compensated ATE RMSE of the position.")
    w("")
    w("| Sequence | Length | DM-VIO ATE | DSO ATE | DM-VIO coverage | DSO coverage |")
    w("| :-- | --: | --: | --: | --: | --: |")
    for seq in cfg["tumvi"]:
        r = res["tumvi"][seq["name"]]
        if "ate_m" not in r["dmvio"]:
            continue
        w("| `{}` | {:.0f} s | {:.2f} cm | {:.2f} cm | {:.0f} % | {:.0f} % |".format(
            seq["name"], seq["duration_s"],
            100 * r["dmvio"]["ate_m"], 100 * r["dso"]["ate_m"],
            r["dmvio"]["coverage_pct"], r["dso"]["coverage_pct"]))
    w("")

    w("### Effect of the camera-IMU lever arm on the ATE")
    w("")
    w("| Sequence | DM-VIO raw | DM-VIO compensated | DSO raw | DSO compensated |")
    w("| :-- | --: | --: | --: | --: |")
    for seq in cfg["tumvi"]:
        r = res["tumvi"][seq["name"]]
        if "ate_m" not in r["dmvio"]:
            continue
        w("| `{}` | {:.2f} cm | {:.2f} cm | {:.2f} cm | {:.2f} cm |".format(
            seq["name"], 100 * r["dmvio"]["ate_raw_m"], 100 * r["dmvio"]["ate_m"],
            100 * r["dso"]["ate_raw_m"], 100 * r["dso"]["ate_m"]))
    w("")

    w("### TUM-VI, ground truth at start and end only")
    w("")
    w("| Sequence | Method | Scale drift | End error (start-aligned) | e_align | Coverage |")
    w("| :-- | :-- | --: | --: | --: | --: |")
    for seq in cfg["tumvi"]:
        r = res["tumvi"][seq["name"]]
        if "scale_drift" not in r["dmvio"]:
            continue
        for key, label, _ in METHODS:
            m = r[key]
            w("| `{}` | {} | {:.2f}x | {:.2f} m | {:.2f} m | {:.0f} % |".format(
                seq["name"], label, max(m["scale_drift"], 1 / m["scale_drift"]),
                m["end_error_mean_m"], m["e_align_m"], m["coverage_pct"]))
    w("")

    m = res["metric_scale_check"]
    w("### DM-VIO metric-scale output ({})".format(cfg["metric_scale_check"]["name"]))
    w("")
    w("| Alignment | ATE RMSE | Residual scale error |")
    w("| :-- | --: | --: |")
    w("| SE(3), no scale correction | {:.2f} cm | {:.1f} % |".format(
        100 * m["ate_se3_m"], m["scale_error_pct"]))
    w("| Sim(3) | {:.2f} cm | - |".format(100 * m["ate_sim3_m"]))
    w("")

    w("### DSO on TUM monoVO (mid-term)")
    w("")
    w("| Sequence | Length | Keyframes | Scale drift | e_align | Coverage |")
    w("| :-- | --: | --: | --: | --: | --: |")
    for seq in cfg["tum_mono"]:
        m = res["tum_mono"][seq["name"]]
        w("| `{}` | {:.0f} s | {} | {:.3f}x | {:.2f} m | {:.0f} % |".format(
            seq["name"], seq["duration_s"], m["keyframes"],
            max(m["scale_drift"], 1 / m["scale_drift"]), m["e_align_m"], m["coverage_pct"]))
    w("")
    return "\n".join(lines)


def _scalars(d):
    """Keep only JSON-friendly scalar entries of a result dict."""
    return {k: (float(v) if isinstance(v, (float, np.floating)) else v)
            for k, v in d.items() if isinstance(v, (int, float, np.floating))}


def main():
    cfg, res = run_all()
    report = markdown_report(cfg, res)
    print(report)

    summary = {
        "tumvi": {name: {k: _scalars(v) for k, v in r.items()} for name, r in res["tumvi"].items()},
        "metric_scale_check": _scalars(res["metric_scale_check"]),
        "tum_mono": {name: _scalars(r) for name, r in res["tum_mono"].items()},
    }
    os.makedirs(_path("results"), exist_ok=True)
    with open(_path("results", "metrics.json"), "w") as f:
        json.dump(summary, f, indent=2)
    with open(_path("results", "metrics.md"), "w") as f:
        f.write(report)
    print("Wrote results/metrics.json and results/metrics.md")


if __name__ == "__main__":
    main()
