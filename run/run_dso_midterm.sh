#!/usr/bin/env bash
# Mid-term runs: (patched) DSO on TUM monoVO and EuRoC.
#
#   ./run/run_dso_midterm.sh <dso dir> tum_mono <sequence dir>     # e.g. .../sequence_14
#   ./run/run_dso_midterm.sh <dso dir> euroc    <sequence dir>     # e.g. .../V2_02_medium
#
# TUM monoVO ships a full photometric calibration, so it runs with mode=0.
# EuRoC has none, so it runs with mode=1. Reading images.zip needs DSO built
# with libzip support. EuRoC needs a DSO-style camera file (EUROC.txt) next
# to the cam0 folder.
set -euo pipefail

DSO=${1:?path to the dso checkout}
KIND=${2:?tum_mono or euroc}
SEQ=${3:?path to the sequence}

case "$KIND" in
    tum_mono)
        "$DSO/build/bin/dso_dataset" \
            files="$SEQ/images.zip" \
            calib="$SEQ/camera.txt" \
            gamma="$SEQ/pcalib.txt" \
            vignette="$SEQ/vignette.png" \
            preset=0 \
            mode=0
        ;;
    euroc)
        "$DSO/build/bin/dso_dataset" \
            files="$SEQ/mav0/cam0/data" \
            calib="$SEQ/mav0/EUROC.txt" \
            preset=0 \
            mode=1
        ;;
    *)
        echo "unknown dataset kind: $KIND" >&2
        exit 1
        ;;
esac
