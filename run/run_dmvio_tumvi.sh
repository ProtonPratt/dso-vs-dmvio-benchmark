#!/usr/bin/env bash
# Run DM-VIO on one TUM-VI sequence (512x512, 16-bit export).
#
#   ./run/run_dmvio_tumvi.sh <dm-vio dir> <dataset dir> [results dir]
#   ./run/run_dmvio_tumvi.sh ~/dm-vio ~/data/dataset-magistrale5_512_16 results/magistrale5/
#
# Writes into the results dir:
#   result.txt        all frames, camera pose in the visual frame (no metric scale)
#   resultScaled.txt  all frames, IMU pose in metres  <- use this one for metric evaluation
#   resultKFs.txt     keyframes only
set -euo pipefail

DMVIO=${1:?path to the dm-vio checkout}
DATA=${2:?path to dataset-<name>_512_16}
OUT=${3:-results/}
mkdir -p "$OUT"

"$DMVIO/build/bin/dmvio_dataset" \
    files="$DATA/mav0/cam0/data" \
    vignette="$DATA/dso/cam0/vignette.png" \
    imuFile="$DATA/dso/imu.txt" \
    gtFile="$DATA/dso/gt_imu.csv" \
    tsFile="$DATA/dso/cam0/times.txt" \
    calib="$DMVIO/configs/tumvi_calib/camera02.txt" \
    gamma="$DMVIO/configs/tumvi_calib/pcalib.txt" \
    imuCalib="$DMVIO/configs/tumvi_calib/camchain.yaml" \
    settingsFile="$DMVIO/configs/tumvi.yaml" \
    mode=0 \
    use16Bit=1 \
    preset=0 \
    nogui=0 \
    resultsPrefix="$OUT" \
    start=2
