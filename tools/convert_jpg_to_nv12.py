#!/usr/bin/env python3
"""
Convert all JPG images in a given directory to raw NV12 files.

The output file name pattern is:

    <name>_<width>x<height>.nv12

where <name> is the source file basename (without extension), and
<width>/<height> are the decoded image resolution.

Example:
    python3 convert_jpg_to_nv12.py /path/to/frames

This requires `ffmpeg` and `ffprobe` to be installed and visible in PATH.
"""

import argparse
import subprocess
import sys
from pathlib import Path


def get_image_size(src: Path):
    """Return (width, height) of an image using ffprobe, or None on failure."""
    cmd = [
        "ffprobe",
        "-v",
        "error",
        "-select_streams",
        "v:0",
        "-show_entries",
        "stream=width,height",
        "-of",
        "csv=p=0:s=x",
        str(src),
    ]
    try:
        proc = subprocess.run(cmd, check=True, capture_output=True, text=True)
    except FileNotFoundError:
        print("Error: ffprobe not found in PATH. Please install ffmpeg/ffprobe.", file=sys.stderr)
        return None
    except subprocess.CalledProcessError as exc:
        print(f"ffprobe failed for {src}: {exc}", file=sys.stderr)
        return None

    line = proc.stdout.strip()
    if not line:
        print(f"ffprobe returned empty size for {src}", file=sys.stderr)
        return None

    try:
        width_str, height_str = line.split("x")
        return int(width_str), int(height_str)
    except ValueError:
        print(f"Unexpected ffprobe output for {src}: {line}", file=sys.stderr)
        return None


def run_ffmpeg_to_nv12(src: Path, dst: Path, overwrite: bool) -> bool:
    """Invoke ffmpeg to convert a single JPEG to NV12 rawvideo."""
    cmd = [
        "ffmpeg",
        "-loglevel",
        "error",
        "-y" if overwrite else "-n",
        "-i",
        str(src),
        "-pix_fmt",
        "nv12",
        "-f",
        "rawvideo",
        str(dst),
    ]

    try:
        subprocess.run(cmd, check=True)
        return True
    except FileNotFoundError:
        print("Error: ffmpeg not found in PATH. Please install ffmpeg.", file=sys.stderr)
        return False
    except subprocess.CalledProcessError as exc:
        print(f"ffmpeg failed for {src}: {exc}", file=sys.stderr)
        return False


def convert_directory(root: Path, recursive: bool, overwrite: bool) -> int:
    if not root.is_dir():
        print(f"Error: {root} is not a directory.", file=sys.stderr)
        return 1

    patterns = ("*.jpg", "*.jpeg", "*.JPG", "*.JPEG")
    images = []
    if recursive:
        for pat in patterns:
            images.extend(root.rglob(pat))
    else:
        for pat in patterns:
            images.extend(root.glob(pat))

    if not images:
        print(f"No JPG files found under {root}")
        return 0

    images = sorted(set(images))
    print(f"Found {len(images)} JPG files under {root}")

    success = True
    for img in images:
        size = get_image_size(img)
        if size is None:
            success = False
            continue

        width, height = size
        out_name = f"{img.stem}_{width}x{height}.nv12"
        out_path = img.with_name(out_name)

        if out_path.exists() and not overwrite:
            print(f"Skip existing: {out_path}")
            continue

        print(f"Converting: {img} -> {out_path}")
        if not run_ffmpeg_to_nv12(img, out_path, overwrite):
            success = False

    return 0 if success else 1


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Convert all JPG images in a directory to NV12 raw files."
    )
    parser.add_argument(
        "directory",
        help="Directory that contains JPG/JPEG files.",
    )
    parser.add_argument(
        "-r",
        "--recursive",
        action="store_true",
        help="Search JPG files recursively.",
    )
    parser.add_argument(
        "-f",
        "--force",
        action="store_true",
        help="Overwrite existing .nv12 files.",
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    root = Path(args.directory).resolve()
    return convert_directory(root, recursive=args.recursive, overwrite=args.force)


if __name__ == "__main__":
    raise SystemExit(main())
