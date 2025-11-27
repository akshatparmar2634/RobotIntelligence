#!/usr/bin/env python3
"""Convenience launcher for the ri_pkg autonomous navigation + VLM pipeline."""
from __future__ import annotations

import argparse
import os
import stat
import subprocess
import sys
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
RUN_SH = THIS_DIR / "run.sh"


def ensure_executable(script: Path) -> None:
    mode = script.stat().st_mode
    if not mode & stat.S_IXUSR:
        script.chmod(mode | stat.S_IXUSR)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Launch autonomous navigation, the YOLO+VLM detector, or both."
    )
    parser.add_argument(
        "mode",
        choices=["all", "navigation", "vlm"],
        nargs="?",
        default="all",
        help="Which part of the pipeline to run (default: all)",
    )
    parser.add_argument(
        "extra",
        nargs=argparse.REMAINDER,
        help="Additional arguments forwarded to ros2",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])

    if not RUN_SH.exists():
        print(f"run.py expected helper script at {RUN_SH}, but it was not found.", file=sys.stderr)
        return 2

    ensure_executable(RUN_SH)

    cmd = [str(RUN_SH), args.mode]
    if args.extra:
        cmd.extend(args.extra)

    env = os.environ.copy()
    try:
        subprocess.run(cmd, check=True, env=env)
    except subprocess.CalledProcessError as exc:
        return exc.returncode
    return 0


if __name__ == "__main__":
    sys.exit(main())
