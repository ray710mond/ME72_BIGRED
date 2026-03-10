#!/usr/bin/env python3
"""
View live camera feed from the Pi. Run on Mac/PC while the Pi streams.

Usage:
  python3 scripts/camera/live_feed.py in local computer terminal
"""

import os
import platform
import shutil
import subprocess
import sys

PORT = 5000
UDP_RECV_BUF = 2 * 1024 * 1024


def find_ffplay() -> str:
    system = platform.system()

    if system == "Darwin":
        path = shutil.which("ffplay")
        if path:
            return path
        sys.exit("ffplay not found. Install it with:  brew install ffmpeg")

    if system == "Windows":
        path = shutil.which("ffplay")
        if path:
            return path
        common = os.path.expanduser(
            r"~\Downloads\ffmpeg-8.0.1-essentials_build"
            r"\ffmpeg-8.0.1-essentials_build\bin\ffplay.exe"
        )
        if os.path.isfile(common):
            return common
        sys.exit(
            "ffplay not found on PATH or at the default Downloads location.\n"
            "Download ffmpeg from https://ffmpeg.org/download.html and add it to PATH."
        )

    sys.exit(f"Unsupported OS: {system}")


def main():
    ffplay = find_ffplay()
    print(f"Listening for MJPEG stream on port {PORT}  (Ctrl+C to stop)")
    try:
        subprocess.run([
            ffplay,
            "-fflags", "nobuffer+discardcorrupt",
            "-flags", "low_delay",
            "-framedrop",
            "-avioflags", "direct",
            "-probesize", "32",
            "-analyzeduration", "0",
            "-fpsprobesize", "0",
            "-sync", "video",
            "-vf", "transpose=2",
            "-f", "mjpeg",
            f"udp://0.0.0.0:{PORT}?buffer_size={UDP_RECV_BUF}",
        ])
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
