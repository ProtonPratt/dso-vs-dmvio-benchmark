#!/usr/bin/env bash
# Run (patched) DSO on one TUM-VI sequence (512x512, 16-bit export).
#
#   ./run/run_dso_tumvi.sh <dso dir> <dataset dir> [mode]
#
# mode=0 (default) uses the photometric calibration (gamma + vignette).
# mode=1 runs without it; calib-cam1 was run this way.
#
# DSO writes result.txt into the current directory. It logs keyframe poses
# only (setting_onlyLogKFPoses = true), in the camera frame, at arbitrary scale.
set -euo pipefail

DSO=${1:?path to the dso checkout}
DATA=${2:?path to dataset-<name>_512_16}
MODE=${3:-0}

ARGS=(
    files="$DATA/dso/cam0/images"
    calib="$DATA/dso/cam0/camera.txt"
    preset=0
    mode="$MODE"
)
if [ "$MODE" = "0" ]; then
    ARGS+=(gamma="$DATA/dso/cam0/pcalib.txt" vignette="$DATA/dso/cam0/vignette.png")
fi

"$DSO/build/bin/dso_dataset" "${ARGS[@]}"
