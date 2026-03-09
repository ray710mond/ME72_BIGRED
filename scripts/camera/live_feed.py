#!/usr/bin/env python3
"""
One-command live video feed from the Pi.

Usage:
  python live_feed.py <PI_IP>                  # 720p, saves recording on Pi
  python live_feed.py <PI_IP> hd               # 1080p, saves recording on Pi
  python live_feed.py <PI_IP> --no-save        # 720p, no recording
  python live_feed.py <PI_IP> hd --no-save     # 1080p, no recording
  python live_feed.py                          # will prompt for Pi IP
"""

import argparse
import base64
import os
import platform
import shutil
import socket
import subprocess
import sys
import time

PI_USER = "bigred"
PORT = 5000
UDP_RECV_BUF = 2 * 1024 * 1024  # 2 MB receive buffer
RECORDINGS_DIR = "~/recordings"

PROFILES = {
    "default": {"width": 1280, "height": 720,  "fps": 30},
    "hd":      {"width": 1920, "height": 1080, "fps": 30},
}

# Template for a tiny UDP forwarder that runs on the Pi.
# rpicam-vid pipes through tee (to save) then into this script (to stream).
UDP_FORWARDER = """\
import sys, socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
while True:
    d = sys.stdin.buffer.read(32768)
    if not d:
        break
    s.sendto(d, ("{host_ip}", {port}))
"""

FFPLAY_ARGS = [
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
]


def get_local_ip(pi_ip: str) -> str:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.connect((pi_ip, 1))
        return s.getsockname()[0]


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


def start_pi_stream(pi_ip: str, host_ip: str, profile: dict,
                    save: bool) -> subprocess.Popen:
    w, h, fps = profile["width"], profile["height"], profile["fps"]
    rpicam = (
        f"rpicam-vid -n -t 0 --width {w} --height {h} --framerate {fps} "
        f"--codec mjpeg --buffer-count 1"
    )

    if save:
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        rec_file = f"{RECORDINGS_DIR}/{timestamp}.mjpeg"

        fwd_script = UDP_FORWARDER.format(host_ip=host_ip, port=PORT)
        encoded = base64.b64encode(fwd_script.encode()).decode()

        remote = (
            f"pkill -f rpicam-vid 2>/dev/null; sleep 0.5; "
            f"mkdir -p {RECORDINGS_DIR}; "
            f"echo {encoded} | base64 -d > /tmp/_udp_fwd.py && "
            f"{rpicam} -o - 2>/dev/null | "
            f"tee {rec_file} | python3 /tmp/_udp_fwd.py"
        )
        print(f"Recording to Pi: {rec_file}")
    else:
        remote = (
            f"pkill -f rpicam-vid 2>/dev/null; sleep 0.5; "
            f"{rpicam} -o udp://{host_ip}:{PORT}"
        )

    ssh_cmd = [
        "ssh", "-o", "StrictHostKeyChecking=no",
        f"{PI_USER}@{pi_ip}",
        remote,
    ]
    print(f"Starting stream on Pi ({pi_ip}) -> {host_ip}:{PORT}")
    return subprocess.Popen(ssh_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    parser = argparse.ArgumentParser(description="Live video feed from Pi")
    parser.add_argument("pi_ip", nargs="?", help="Pi IP address")
    parser.add_argument("profile", nargs="?", default="default",
                        choices=PROFILES.keys(), help="Quality profile (default: 720p)")
    parser.add_argument("--no-save", action="store_true",
                        help="Don't save recording on the Pi")
    args = parser.parse_args()

    pi_ip = args.pi_ip
    if not pi_ip:
        pi_ip = input("Enter Pi IP address: ").strip()
        if not pi_ip:
            sys.exit("No IP provided.")

    profile = PROFILES[args.profile]
    save = not args.no_save
    host_ip = get_local_ip(pi_ip)
    ffplay = find_ffplay()

    print(f"OS:        {platform.system()}")
    print(f"Pi:        {PI_USER}@{pi_ip}")
    print(f"Local IP:  {host_ip}")
    print(f"Stream:    {profile['width']}x{profile['height']} @ {profile['fps']}fps")
    print(f"Recording: {'ON -> ~/recordings/ on Pi' if save else 'OFF'}")
    print(f"ffplay:    {ffplay}")
    print()

    ssh_proc = start_pi_stream(pi_ip, host_ip, profile, save)
    time.sleep(1)

    print(f"Opening viewer on port {PORT}  (Ctrl+C to stop)\n")
    try:
        subprocess.run([ffplay] + FFPLAY_ARGS)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nStopping Pi stream...")
        ssh_proc.terminate()
        subprocess.run(
            ["ssh", "-o", "StrictHostKeyChecking=no",
             f"{PI_USER}@{pi_ip}",
             "pkill -f rpicam-vid; pkill -f _udp_fwd.py"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        print("Done.")


if __name__ == "__main__":
    main()
