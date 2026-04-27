#!/usr/bin/env python

import argparse
import math
import time

from lerobot.utils.import_utils import require_package

require_package("python-osc", extra="hardware", import_name="pythonosc")

from pythonosc.udp_client import SimpleUDPClient  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send fake Cluster /ik/target OSC messages.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--address", default="/ik/target")
    parser.add_argument("--mode", choices=["point", "circle", "line"], default="circle")
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.0)
    parser.add_argument("--radius", type=float, default=0.05)
    parser.add_argument("--height", type=float, default=0.0)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--period", type=float, default=4.0)
    return parser.parse_args()


def target_at(args: argparse.Namespace, elapsed_s: float) -> tuple[float, float, float]:
    if args.mode == "point":
        return args.x, args.y, args.z

    phase = 2.0 * math.pi * elapsed_s / args.period
    if args.mode == "circle":
        return (
            args.x + args.radius * math.cos(phase),
            args.y + args.height,
            args.z + args.radius * math.sin(phase),
        )

    line_pos = math.sin(phase)
    return args.x + args.radius * line_pos, args.y + args.height, args.z


def main() -> None:
    args = parse_args()
    if args.fps <= 0:
        raise ValueError("--fps must be positive")
    if args.duration <= 0:
        raise ValueError("--duration must be positive")
    if args.period <= 0:
        raise ValueError("--period must be positive")

    client = SimpleUDPClient(args.host, args.port)
    interval_s = 1.0 / args.fps
    start = time.perf_counter()
    next_send = start

    print(f"Sending {args.address} to udp://{args.host}:{args.port} mode={args.mode}")
    try:
        while True:
            now = time.perf_counter()
            elapsed = now - start
            if elapsed >= args.duration:
                break

            xyz = target_at(args, elapsed)
            client.send_message(args.address, list(xyz))
            print(f"{args.address} x={xyz[0]: .4f} y={xyz[1]: .4f} z={xyz[2]: .4f}", end="\r")

            next_send += interval_s
            time.sleep(max(next_send - time.perf_counter(), 0.0))
    except KeyboardInterrupt:
        pass
    finally:
        print()


if __name__ == "__main__":
    main()
