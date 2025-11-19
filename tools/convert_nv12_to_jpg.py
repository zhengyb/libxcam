#!/usr/bin/env python3
"""
Convert NV12 raw video to JPEG images using ffmpeg.

Typical NV12 from libxcam tests is a raw sequence of frames with no header,
so the resolution must be specified explicitly.

Examples:
    # Convert all frames in input.nv12 (1280x800) to input_00001.jpg, ...
    python3 convert_nv12_to_jpg.py input.nv12 --width 1280 --height 800

    # Only dump the first frame to front_1280x800.jpg
    python3 convert_nv12_to_jpg.py front.nv12 --width 1280 --height 800 --single

Requires `ffmpeg` in PATH.
"""

import argparse
import subprocess
import sys
from pathlib import Path


def run_ffmpeg_nv12_to_jpg(
    src: Path,
    width: int,
    height: int,
    output: Path,
    single: bool,
    overwrite: bool,
) -> bool:
    """Invoke ffmpeg to convert NV12 rawvideo to JPG."""
    cmd = [
        "ffmpeg",
        "-loglevel",
        "error",
        "-y" if overwrite else "-n",
        "-f",
        "rawvideo",
        "-pixel_format",
        "nv12",
        "-video_size",
        f"{width}x{height}",
        "-i",
        str(src),
    ]

    if single:
        cmd += ["-frames:v", "1"]

    cmd.append(str(output))

    try:
        subprocess.run(cmd, check=True)
        return True
    except FileNotFoundError:
        print("Error: ffmpeg not found in PATH. Please install ffmpeg.", file=sys.stderr)
        return False
    except subprocess.CalledProcessError as exc:
        print(f"ffmpeg failed for {src}: {exc}", file=sys.stderr)
        return False


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Convert NV12 raw video to JPEG images using ffmpeg."
    )
    parser.add_argument(
        "input",
        help="Input NV12 file path.",
    )
    parser.add_argument(
        "--width",
        type=int,
        required=True,
        help="Frame width in pixels.",
    )
    parser.add_argument(
        "--height",
        type=int,
        required=True,
        help="Frame height in pixels.",
    )
    parser.add_argument(
        "-o",
        "--output",
        help=(
            "Output file name or pattern. "
            "If omitted, use <stem>_00001.jpg (single) or <stem>_%05d.jpg (multi)."
        ),
    )
    parser.add_argument(
        "--single",
        action="store_true",
        help="Only convert the first frame.",
    )
    parser.add_argument(
        "-f",
        "--force",
        action="store_true",
        help="Overwrite existing output file(s).",
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)

    src = Path(args.input).resolve()
    if not src.is_file():
        print(f"Error: input file not found: {src}", file=sys.stderr)
        return 1

    if args.width <= 0 or args.height <= 0:
        print("Error: width and height must be positive integers.", file=sys.stderr)
        return 1

    # Decide default output naming.
    if args.output:
        out_path = Path(args.output)
    else:
        stem = src.stem
        if args.single:
            out_path = src.with_name(f"{stem}_{args.width}x{args.height}.jpg")
        else:
            out_path = src.with_name(f"{stem}_%05d.jpg")

    print(
        f"Converting NV12 {src} ({args.width}x{args.height}) "
        f"-> {out_path} ({'single frame' if args.single else 'all frames'})"
    )

    ok = run_ffmpeg_nv12_to_jpg(
        src=src,
        width=args.width,
        height=args.height,
        output=out_path,
        single=args.single,
        overwrite=args.force,
    )
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

