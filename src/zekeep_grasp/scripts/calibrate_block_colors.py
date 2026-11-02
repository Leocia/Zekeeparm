"""Fit block colors from manually labeled rectangles in a saved camera image."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import cv2

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from utils.block_colors import fit_color_calibration  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", required=True, type=Path)
    parser.add_argument("--sample", required=True, action="append", metavar="COLOR:X1,Y1,X2,Y2",
                        help="A rectangle entirely inside a known color block; may repeat a color.")
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()
    image = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    if image is None:
        parser.error(f"Cannot read image: {args.image}")
    samples = {}
    try:
        for item in args.sample:
            name, coords = item.split(":", 1)
            x1, y1, x2, y2 = map(int, coords.split(","))
            if not (0 <= x1 < x2 <= image.shape[1] and 0 <= y1 < y2 <= image.shape[0]):
                raise ValueError(f"Sample outside image: {item}")
            samples.setdefault(name, []).append(image[y1:y2, x1:x2])
        calibration = fit_color_calibration(samples)
    except (ValueError, TypeError) as exc:
        parser.error(str(exc))
    calibration["source_image"] = str(args.image.resolve())
    calibration["samples"] = args.sample
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(calibration, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(calibration, indent=2))


if __name__ == "__main__":
    main()
