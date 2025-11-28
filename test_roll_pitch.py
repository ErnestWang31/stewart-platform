#!/usr/bin/env python3
"""
Utility script to exercise pure roll and pitch motions using the same mapping
that the PID autotuner/controller rely on (including axis rotation offsets).

This helps verify that each servo responds symmetrically to ±commands along
the conceptual roll/pitch axes that the rest of the software understands.
"""

import argparse
import sys
import time
from typing import Iterable

import numpy as np

from stewart_platform_controller import StewartPlatformController


def _sweep_angles(amplitude: float, steps: int, repeats: int) -> Iterable[float]:
    """Generate a back-and-forth sweep between -amplitude and +amplitude."""
    if steps < 2:
        raise ValueError("steps must be >= 2")
    ramp = np.linspace(-amplitude, amplitude, steps)
    pattern = np.concatenate([ramp, ramp[::-1]])
    for _ in range(repeats):
        for value in pattern:
            yield float(value)


def run_test(
    controller: StewartPlatformController,
    axis: str,
    amplitude: float,
    steps: int,
    repeats: int,
    dwell: float,
    verbose: bool,
):
    """Send a sequence of roll/pitch commands and optionally print details."""
    axis = axis.lower()
    axes_to_test = ["roll", "pitch"] if axis == "both" else [axis]
    print(
        f"[TEST] Starting roll/pitch sweep (axes={axes_to_test}, amplitude=±{amplitude}°, "
        f"steps={steps}, repeats={repeats}, dwell={dwell:.2f}s)"
    )

    for current_axis in axes_to_test:
        print(f"\n[TEST] Sweeping {current_axis.upper()} axis")
        for angle in _sweep_angles(amplitude, steps, repeats):
            roll_cmd = angle if current_axis == "roll" else 0.0
            pitch_cmd = angle if current_axis == "pitch" else 0.0
            controller.send_platform_tilt(roll_cmd, pitch_cmd)
            if verbose:
                print(
                    f"[TEST] Commanded Roll={roll_cmd:+.2f}°, Pitch={pitch_cmd:+.2f}° "
                    f"(after internal axis rotation)"
                )
            time.sleep(dwell)

    print("\n[TEST] Returning platform to neutral")
    controller.send_platform_tilt(0.0, 0.0)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Roll/pitch sweep tester aligned with autotune axis mapping."
    )
    parser.add_argument("--config", default="config_stewart.json", help="Config JSON path.")
    parser.add_argument(
        "--axis",
        choices=["roll", "pitch", "both"],
        default="both",
        help="Which axis to sweep.",
    )
    parser.add_argument(
        "--amplitude",
        type=float,
        default=6.0,
        help="Peak roll/pitch command in degrees (clipped internally to safe range).",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=11,
        help="Number of samples between -amplitude and +amplitude (>=2).",
    )
    parser.add_argument(
        "--repeats",
        type=int,
        default=2,
        help="How many times to repeat the back-and-forth sweep per axis.",
    )
    parser.add_argument(
        "--dwell",
        type=float,
        default=0.4,
        help="Pause between successive commands (seconds).",
    )
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="Skip servo connection; useful for checking mapping printouts only.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print each command as it is sent.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    controller = StewartPlatformController(args.config)

    if not args.simulate:
        if not controller.connect_servos():
            print("[TEST] Failed to connect to servos; re-run with --simulate to preview mapping.")
            sys.exit(1)
    else:
        print("[TEST] Simulation mode: commands will be printed but not sent to hardware.")

    try:
        run_test(
            controller=controller,
            axis=args.axis,
            amplitude=args.amplitude,
            steps=args.steps,
            repeats=args.repeats,
            dwell=args.dwell,
            verbose=args.verbose or args.simulate,
        )
    except KeyboardInterrupt:
        print("\n[TEST] Interrupted by user, returning to neutral.")
        controller.send_platform_tilt(0.0, 0.0)
    finally:
        if controller.servo_serial:
            controller.servo_serial.close()


if __name__ == "__main__":
    main()

