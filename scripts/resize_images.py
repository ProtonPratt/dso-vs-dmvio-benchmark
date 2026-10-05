#!/usr/bin/env python3
"""Resize every image in a folder, in parallel.

Used to bring KITTI raw frames to 752x480 so they could be fed to DSO with
an existing calibration layout.

    python scripts/resize_images.py /path/to/image_00/data_old /path/to/image_00/data
"""
import argparse
import os
from functools import partial
from multiprocessing import Pool

import cv2


def resize_one(name, src_dir, dst_dir, size):
    img = cv2.imread(os.path.join(src_dir, name))
    if img is None:
        print(f"Failed to load image: {name}")
        return
    cv2.imwrite(os.path.join(dst_dir, name), cv2.resize(img, size))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("src_dir")
    parser.add_argument("dst_dir")
    parser.add_argument("--width", type=int, default=752)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args()

    image_files = [f for f in os.listdir(args.src_dir) if f.endswith(('.png', '.jpg', '.jpeg'))]
    if not image_files:
        print(f"No images found in {args.src_dir}")
        return

    os.makedirs(args.dst_dir, exist_ok=True)
    with Pool() as pool:
        pool.map(partial(resize_one, src_dir=args.src_dir, dst_dir=args.dst_dir,
                         size=(args.width, args.height)), image_files)
    print(f"Resized {len(image_files)} images to {args.width}x{args.height}")


if __name__ == '__main__':
    main()
