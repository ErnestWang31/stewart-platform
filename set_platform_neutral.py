#!/usr/bin/env python3
"""
Helper tool that sends zero roll/pitch to the Stewart platform so you can
inspect/adjust the mechanical neutral position manually.
"""

import argparse
import sys
import time

from stewart_platform_controller import StewartPlatformController


def parse_args():
    parser = argparse.ArgumentParser(description="Send neutral tilt command for manual leveling.")
    parser.add_argument("--config", default="config_stewart.json", help="Path to config JSON.")
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=0.0,
        help="Optional timeout; omit or set to 0 to hold until you interrupt.",
    )
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="Skip servo connection; useful if you just want to inspect neutral values.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    controller = StewartPlatformController(args.config)

    if args.simulate:
        print("[NEUTRAL] Simulation mode enabled; not connecting to servos.")
    else:
        if not controller.connect_servos():
            print("[NEUTRAL] Failed to connect to servos.")
            sys.exit(1)

    try:
        print("[NEUTRAL] Sending roll=0°, pitch=0° command.")
        controller.send_platform_tilt(0.0, 0.0)

        if args.hold_seconds > 0:
            print(f"[NEUTRAL] Holding position for {args.hold_seconds:.1f} seconds...")
            time.sleep(args.hold_seconds)
        else:
            print("[NEUTRAL] Holding indefinitely; press Ctrl+C when finished leveling.")
            while True:
                time.sleep(1.0)

    except KeyboardInterrupt:
        print("\n[NEUTRAL] Interrupted by user.")
    finally:
        print("[NEUTRAL] Returning platform to neutral and closing connection.")
        controller.send_platform_tilt(0.0, 0.0)
        if controller.servo_serial:
            controller.servo_serial.close()


if __name__ == "__main__":
    main()

