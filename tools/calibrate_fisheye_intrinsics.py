#!/usr/bin/env python3
"""
Calibrate a single fisheye camera using OpenCV's fisheye model.

The script searches the provided chessboard images, estimates intrinsic parameters,
and writes the result into a JSON file compatible with libxcam's calibration parser.


surrond view needs

"""

import argparse
import glob
import json
import os
import sys
from typing import Iterable, List, Tuple

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Calibrate fisheye intrinsics from chessboard images.")
    parser.add_argument(
        "--images",
        nargs="+",
        default=[],
        help="Explicit list of calibration images.")
    parser.add_argument(
        "--image-glob",
        action="append",
        default=[],
        help="Glob pattern for calibration images (can be given multiple times).")
    parser.add_argument(
        "--board-cols",
        type=int,
        required=True,
        help="Number of inner corners per chessboard row.")
    parser.add_argument(
        "--board-rows",
        type=int,
        required=True,
        help="Number of inner corners per chessboard column.")
    parser.add_argument(
        "--square-size",
        type=float,
        required=True,
        help="Chessboard square size in user-defined units (e.g. millimetres).")
    parser.add_argument(
        "--max-iters",
        type=int,
        default=200,
        help="Maximum iterations for corner refinement.")
    parser.add_argument(
        "--eps",
        type=float,
        default=1e-6,
        help="Termination epsilon for corner refinement.")
    parser.add_argument(
        "--model-id",
        type=int,
        default=0,
        help="Optional camera model id to store in the JSON (defaults to 0).")
    parser.add_argument(
        "--flip",
        action="store_true",
        help="Mark the resulting camera as vertically flipped in the JSON output.")
    parser.add_argument(
        "--output-json",
        required=True,
        help="Path to the output JSON file.")
    return parser.parse_args()


def collect_image_paths(explicit: Iterable[str], patterns: Iterable[str]) -> List[str]:
    paths: List[str] = []
    for item in explicit:
        paths.append(item)
    for pattern in patterns:
        paths.extend(glob.glob(pattern))
    unique = sorted(set(paths))
    return unique


def prepare_object_points(cols: int, rows: int, square_size: float) -> np.ndarray:
    objp = np.zeros((1, rows * cols, 3), dtype=np.float32)
    grid = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp[0, :, :2] = grid * square_size
    return objp


def find_corners(
    image_path: str,
    board_size: Tuple[int, int],
    term_criteria: Tuple[int, int, float]
) -> Tuple[bool, np.ndarray, Tuple[int, int]]:
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        print(f"[WARN] Unable to read image: {image_path}", file=sys.stderr)
        return False, np.empty(0), (0, 0)

    ret, corners = cv2.findChessboardCorners(
        image,
        board_size,
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_FILTER_QUADS)
    if not ret:
        print(f"[WARN] Chessboard not found in: {image_path}", file=sys.stderr)
        return False, np.empty(0), (0, 0)

    cv2.cornerSubPix(
        image,
        corners,
        winSize=(3, 3),
        zeroZone=(-1, -1),
        criteria=term_criteria)

    height, width = image.shape[:2]
    return True, corners, (width, height)


def estimate_radius(width: int, height: int, cx: float, cy: float) -> float:
    dx = max(cx, width - cx)
    dy = max(cy, height - cy)
    return float(np.hypot(dx, dy))


def main() -> int:
    args = parse_args()
    image_paths = collect_image_paths(args.images, args.image_glob)
    if not image_paths:
        print("[ERROR] No calibration images provided.", file=sys.stderr)
        return 1

    board_size = (args.board_cols, args.board_rows)
    term_criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        args.max_iters,
        args.eps)

    objp = prepare_object_points(args.board_cols, args.board_rows, args.square_size)
    objpoints: List[np.ndarray] = []
    imgpoints: List[np.ndarray] = []
    used_images: List[str] = []
    image_size = None

    for path in image_paths:
        found, corners, size = find_corners(path, board_size, term_criteria)
        if not found:
            continue
        objpoints.append(objp.copy())
        imgpoints.append(corners.reshape(1, -1, 2))
        used_images.append(path)
        image_size = size

    if len(objpoints) < 3 or image_size is None:
        print("[ERROR] Need corners from at least three images for calibration.", file=sys.stderr)
        return 1

    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_CHECK_COND | cv2.fisheye.CALIB_FIX_SKEW

    rms, K, D, _, _ = cv2.fisheye.calibrate(
        objpoints,
        imgpoints,
        image_size,
        K,
        D,
        None,
        None,
        flags=flags,
        criteria=term_criteria)

    width, height = image_size
    fx = float(K[0, 0])
    fy = float(K[1, 1])
    cx = float(K[0, 2])
    cy = float(K[1, 2])
    skew = float(K[0, 1])
    radius = estimate_radius(width, height, cx, cy)
    h_fov = float(np.degrees(2.0 * np.arctan(width / (2.0 * fx)))) if fx > 0 else 0.0
    v_fov = float(np.degrees(2.0 * np.arctan(height / (2.0 * fy)))) if fy > 0 else 0.0

    camera_entry = {
        "radius": radius,
        "w": width,
        "h": height,
        "fx": fx,
        "fy": fy,
        "cx": cx,
        "cy": cy,
        "skew": skew,
        "fov": max(h_fov, v_fov),
        "flip": "true" if args.flip else "false",
        "D": [float(v) for v in D.flatten()],
        "K": [float(v) for v in K.flatten()],
        "c": [1.0, 0.0, 0.0]
    }

    output = {
        "model": args.model_id,
        "info": {
            "rms": float(rms),
            "num_images_used": len(used_images),
            "board_cols": args.board_cols,
            "board_rows": args.board_rows,
            "square_size": args.square_size,
            "image_paths": used_images
        },
        "cameras": {
            "camera": [camera_entry]
        }
    }

    os.makedirs(os.path.dirname(os.path.abspath(args.output_json)), exist_ok=True)
    with open(args.output_json, "w", encoding="utf-8") as fh:
        json.dump(output, fh, indent=2)

    print(f"[INFO] Calibration RMS error: {rms:.6f}")
    print(f"[INFO] Intrinsics saved to: {args.output_json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
