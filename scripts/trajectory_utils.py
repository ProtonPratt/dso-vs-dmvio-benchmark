"""Trajectory I/O, timestamp association, alignment and error metrics.

Everything here is plain numpy so the evaluation has no dependency on the
estimators themselves: it only needs the trajectory text files they wrote.

Conventions
-----------
* Estimated trajectories are in TUM format: ``t x y z qx qy qz qw`` with t in
  seconds. Both DSO and DM-VIO write this.
* Positions are (N, 3) arrays, rotations are (N, 3, 3) world-from-body.
"""
import numpy as np


# --------------------------------------------------------------------------- #
# I/O
# --------------------------------------------------------------------------- #
def load_tum(path):
    """Load a TUM-format trajectory. Rows containing NaN are dropped.

    Returns (timestamps [s], positions (N,3), quaternions (N,4) as x y z w).
    """
    data = np.loadtxt(path)
    data = data[~np.isnan(data).any(axis=1)]
    return data[:, 0], data[:, 1:4], data[:, 4:8]


def load_ground_truth(path):
    """Load ground-truth positions.

    ``.csv``  -> TUM-VI / EuRoC style: ``t[ns], x, y, z, qw, qx, qy, qz``
    otherwise -> TUM monoVO ``groundtruthSync.txt``: TUM format where frames
                 outside the motion-capture volume are NaN.

    Returns (timestamps [s], positions (N,3)).
    """
    if path.endswith(".csv"):
        data = np.loadtxt(path, delimiter=",", comments="#")
        return data[:, 0] * 1e-9, data[:, 1:4]
    t, p, _ = load_tum(path)
    return t, p


# --------------------------------------------------------------------------- #
# Geometry
# --------------------------------------------------------------------------- #
def quat_to_rot(q):
    """Unit quaternions (N,4) as x y z w -> rotation matrices (N,3,3)."""
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    x, y, z, w = q.T
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1),
    ], -2)


def path_length(p):
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


def umeyama(src, dst, with_scale=True):
    """Least-squares similarity transform so that  dst ~ s * R @ src + t.

    Umeyama, "Least-squares estimation of transformation parameters between
    two point patterns", PAMI 1991. ``with_scale=False`` gives the rigid
    SE(3) (Horn / Kabsch) solution instead of Sim(3).
    """
    mu_s, mu_d = src.mean(0), dst.mean(0)
    sc, dc = src - mu_s, dst - mu_d
    cov = dc.T @ sc / len(src)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:  # avoid a reflection
        S[2, 2] = -1
    R = U @ S @ Vt
    s = float(np.trace(np.diag(D) @ S) / sc.var(0).sum()) if with_scale else 1.0
    t = mu_d - s * R @ mu_s
    return s, R, t


def apply_transform(p, s, R, t):
    return s * (R @ p.T).T + t


# --------------------------------------------------------------------------- #
# Association
# --------------------------------------------------------------------------- #
def associate(t_est, t_gt, max_dt=0.02):
    """Match every estimate to its nearest ground-truth sample in time.

    Pairs further apart than ``max_dt`` seconds are rejected. This matters:
    without the gate, estimates outside the motion-capture volume get matched
    to a ground-truth pose that was recorded seconds (or minutes) away.

    Returns (indices into est, indices into gt).
    """
    right = np.clip(np.searchsorted(t_gt, t_est), 1, len(t_gt) - 1)
    left = right - 1
    nearest = np.where(np.abs(t_gt[left] - t_est) < np.abs(t_gt[right] - t_est), left, right)
    keep = np.abs(t_gt[nearest] - t_est) <= max_dt
    return np.nonzero(keep)[0], nearest[keep]


def split_at_gap(t, min_gap=5.0):
    """Index where a time series has its largest gap, or None if it is continuous.

    TUM-VI corridor sequences and TUM monoVO only have ground truth at the
    start and the end (inside the motion-capture room).
    """
    gaps = np.diff(t)
    k = int(np.argmax(gaps))
    return k + 1 if gaps[k] > min_gap else None


# --------------------------------------------------------------------------- #
# Metrics
# --------------------------------------------------------------------------- #
def ate(est, gt, with_scale=True):
    """Absolute trajectory error after alignment.

    Returns (rmse, per-pose error norms, (s, R, t)).
    """
    s, R, t = umeyama(est, gt, with_scale)
    err = np.linalg.norm(apply_transform(est, s, R, t) - gt, axis=1)
    return float(np.sqrt(np.mean(err ** 2))), err, (s, R, t)


def ate_with_lever_arm(p_cam, R_wc, gt_imu, t_cam_imu, max_iters=20, tol=1e-9):
    """Sim(3) ATE between a *camera* trajectory and *IMU* ground truth.

    The estimators report the camera centre, the motion capture reports the
    IMU. The two are ``|t_cam_imu|`` apart (9.6 cm on the TUM-VI rig), which
    shows up as centimetres of fake error whenever the rig rotates.

    The IMU origin expressed in the estimator's world frame is

        p_imu = p_cam + R_wc @ t_cam_imu / s

    where s is the metres-per-unit scale of the (scale-free) visual frame.
    s is exactly what the Sim(3) alignment estimates, so the two are solved
    by fixed-point iteration, starting from the uncompensated alignment.

    Returns (rmse, per-pose errors, (s, R, t), compensated positions).
    """
    lever_world = np.einsum("nij,j->ni", R_wc, np.asarray(t_cam_imu))
    _, _, (s, _, _) = ate(p_cam, gt_imu)
    for _ in range(max_iters):
        p_imu = p_cam + lever_world / s
        rmse, err, (s_new, R, t) = ate(p_imu, gt_imu)
        converged = abs(s_new - s) < tol
        s = s_new
        if converged:
            break
    return rmse, err, (s, R, t), p_imu


def start_end_metrics(p_est_all, est_matched, gt_matched, is_start):
    """Drift metrics for sequences with ground truth only at the start and end.

    Follows the TUM monoVO protocol (Engel et al., 2016): align the trajectory
    to the start segment and, separately, to the end segment.

    * scale drift  = s_end / s_start. 1.0 means the scale never changed.
    * e_align      = RMSE between the two differently-aligned copies of the
                     *whole* trajectory. Captures translation, rotation and
                     scale drift accumulated over the unobserved middle part.
    * end error    = mean position error on the end segment when only the
                     start segment was used for alignment, i.e. "how far off
                     are you when you walk back into the room".
    """
    a, b = is_start, ~is_start
    Ts = umeyama(est_matched[a], gt_matched[a])
    Te = umeyama(est_matched[b], gt_matched[b])
    e_align = np.sqrt(np.mean(np.sum(
        (apply_transform(p_est_all, *Ts) - apply_transform(p_est_all, *Te)) ** 2, axis=1)))
    end_err = np.linalg.norm(apply_transform(est_matched[b], *Ts) - gt_matched[b], axis=1)
    return {
        "scale_start": Ts[0],
        "scale_end": Te[0],
        "scale_drift": Te[0] / Ts[0],
        "e_align_m": float(e_align),
        "end_error_mean_m": float(end_err.mean()),
        "T_start": Ts,
    }
