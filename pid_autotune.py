#!/usr/bin/env python3
"""
PID autotune helper for the Stewart platform.

Performs a single-axis Åström–Hägglund relay test to estimate the ultimate
gain/period, derives PID gains via classic tuning rules, and optionally writes
the shared gains back into config_stewart.json (applied to both axes).
"""

import argparse
import json
import math
import os
import shutil
import sys
import time
from collections import deque
from datetime import datetime

import cv2
import numpy as np

from stewart_platform_controller import StewartPlatformController


def _compute_pid_from_rule(ku, tu, rule):
    """Return PID gains (parallel form) for the selected tuning rule."""
    if ku <= 0 or tu <= 0:
        raise ValueError("Ultimate gain and period must be positive for tuning.")

    if rule == "zn":
        kp = 0.6 * ku
        ki = 2.0 * kp / tu
        kd = kp * tu / 8.0
        name = "Ziegler–Nichols"
    elif rule == "tyreus":
        # Tyreus–Luyben recommendations for PI/PID
        kp = 0.454 * ku
        ki = kp / (2.2 * tu)  # Ti = 2.2 Tu
        kd = kp * tu / 6.3
        name = "Tyreus–Luyben"
    else:
        raise ValueError(f"Unsupported tuning rule '{rule}'")

    return {"rule": name, "Kp": kp, "Ki": ki, "Kd": kd}


class RelayAutotuneRunner:
    """Runs a relay autotune on a single axis."""

    def __init__(
        self,
        config_path,
        axis,
        relay_amplitude_deg,
        deadband_m,
        cycles,
        max_duration,
        display,
        simulate,
    ):
        self.config_path = config_path
        self.axis = axis
        self.relay_amplitude = relay_amplitude_deg
        self.deadband = deadband_m
        self.target_cycles = cycles
        self.max_duration = max_duration
        self.display = display
        self.simulate = simulate

        self.controller = StewartPlatformController(config_path)
        self.detector = self.controller.detector
        self.camera_cfg = self.controller.config.get("camera", {})
        self.has_servo = False

    def connect(self):
        """Connect to servos unless running purely in simulation."""
        if self.simulate:
            print("[AUTOTUNE] Simulation mode enabled (no servo connection).")
            return
        self.has_servo = self.controller.connect_servos()
        if not self.has_servo:
            raise RuntimeError("Failed to connect to servos. Use --simulate to skip.")

    def cleanup(self):
        """Return platform to neutral and release hardware resources."""
        try:
            self.controller.send_platform_tilt(0.0, 0.0)
        except Exception:
            pass
        if self.controller.servo_serial:
            try:
                self.controller.servo_serial.close()
            except Exception:
                pass
        cv2.destroyAllWindows()

    def _open_camera(self):
        index = self.camera_cfg.get("index", 0)
        frame_width = self.camera_cfg.get("frame_width", 640)
        frame_height = self.camera_cfg.get("frame_height", 480)

        cap = cv2.VideoCapture(index)
        if not cap.isOpened():
            raise RuntimeError(f"Unable to open camera index {index}")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def run(self):
        """Execute the relay test and return raw measurements."""
        cap = self._open_camera()
        measurement_log = []
        command_log = []
        zero_crossings = []
        peaks = []
        segment_values = []

        if not self.simulate and not self.has_servo:
            self.connect()

        last_sign = 0
        last_time = time.time()
        relay_output = self.relay_amplitude
        settle_start = time.time()

        try:
            while True:
                if time.time() - settle_start > self.max_duration:
                    raise TimeoutError("Autotune timed out before completing cycles.")

                ret, frame = cap.read()
                if not ret:
                    continue

                frame = cv2.resize(
                    frame,
                    (
                        self.camera_cfg.get("frame_width", 640),
                        self.camera_cfg.get("frame_height", 480),
                    ),
                )
                found, _, _, pos_x, pos_y = self.detector.detect_ball(frame)
                if not found:
                    if self.display:
                        cv2.imshow("PID Autotune (no ball)", frame)
                        if cv2.waitKey(1) & 0xFF == 27:
                            raise KeyboardInterrupt
                    continue

                axis_value = pos_x if self.axis == "x" else pos_y
                error = -axis_value  # setpoint is 0

                if abs(error) > self.deadband:
                    relay_output = self.relay_amplitude if error > 0 else -self.relay_amplitude

                roll_cmd = relay_output if self.axis == "x" else 0.0
                pitch_cmd = relay_output if self.axis == "y" else 0.0

                if self.has_servo:
                    self.controller.send_platform_tilt(roll_cmd, pitch_cmd)

                now = time.time()
                measurement_scaled = axis_value * 100.0  # match PID scaling
                measurement_log.append((now, measurement_scaled))
                command_log.append((now, relay_output))
                segment_values.append(measurement_scaled)

                current_sign = 0
                if measurement_scaled > 0:
                    current_sign = 1
                elif measurement_scaled < 0:
                    current_sign = -1

                if last_sign == 0:
                    last_sign = current_sign

                # Detect zero crossing (sign change)
                if current_sign != 0 and last_sign != 0 and current_sign != last_sign:
                    zero_crossings.append(now)
                    if segment_values:
                        peaks.append(max(abs(v) for v in segment_values))
                        segment_values = []

                    completed_cycles = max(0, (len(zero_crossings) - 1) // 2)
                    if completed_cycles >= self.target_cycles and len(peaks) >= self.target_cycles:
                        break

                    last_sign = current_sign

                last_time = now

                if self.display:
                    vis_frame, _, _, _ = self.detector.draw_detection(frame)
                    cv2.putText(
                        vis_frame,
                        f"Axis {self.axis.upper()} Error: {axis_value:.3f} m",
                        (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2,
                    )
                    cv2.putText(
                        vis_frame,
                        f"Relay Cmd: {relay_output:.1f} deg",
                        (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 255),
                        2,
                    )
                    cv2.imshow("PID Autotune", vis_frame)
                    if cv2.waitKey(1) & 0xFF == 27:
                        raise KeyboardInterrupt

        finally:
            cap.release()
            self.cleanup()

        if len(zero_crossings) < 3 or len(peaks) == 0:
            raise RuntimeError("Insufficient oscillation data collected for tuning.")

        return {
            "measurement_log": measurement_log,
            "command_log": command_log,
            "zero_crossings": zero_crossings,
            "peaks": peaks,
        }


def _summarize_results(raw, relay_amplitude):
    """Compute Ku, Tu and relay details from raw logs."""
    zero_crossings = raw["zero_crossings"]
    peaks = raw["peaks"]

    periods = []
    for i in range(len(zero_crossings) - 2):
        periods.append(zero_crossings[i + 2] - zero_crossings[i])

    if not periods:
        raise RuntimeError("Unable to compute oscillation period (Tu).")

    tu = float(np.mean(periods))
    avg_peak = float(np.mean(peaks))
    if avg_peak <= 0:
        raise RuntimeError("Oscillation amplitude is zero.")

    ku = (4.0 * relay_amplitude) / (math.pi * avg_peak)

    return {
        "Tu": tu,
        "Ku": ku,
        "avg_peak_scaled": avg_peak,
        "relay_amplitude": relay_amplitude,
        "cycles_used": len(periods),
    }


def _write_results(results_path, entry):
    os.makedirs(os.path.dirname(results_path) or ".", exist_ok=True)
    data = []
    if os.path.exists(results_path):
        try:
            with open(results_path, "r") as f:
                data = json.load(f)
        except Exception:
            data = []

    data.append(entry)
    with open(results_path, "w") as f:
        json.dump(data, f, indent=2)


def _apply_to_config(config_path, gains):
    timestamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
    backup_path = f"{config_path}.bak_{timestamp}"
    shutil.copy2(config_path, backup_path)

    with open(config_path, "r") as f:
        config = json.load(f)

    pid_cfg = config.setdefault("pid", {})
    pid_cfg["Kp_x"] = pid_cfg["Kp_y"] = gains["Kp"]
    pid_cfg["Ki_x"] = pid_cfg["Ki_y"] = gains["Ki"]
    pid_cfg["Kd_x"] = pid_cfg["Kd_y"] = gains["Kd"]

    with open(config_path, "w") as f:
        json.dump(config, f, indent=2)

    return backup_path


def parse_args():
    parser = argparse.ArgumentParser(description="1D PID autotune for the Stewart platform.")
    parser.add_argument("--config", default="config_stewart.json", help="Path to configuration JSON.")
    parser.add_argument("--axis", choices=["x", "y"], default="x", help="Axis to tune (roll=X, pitch=Y).")
    parser.add_argument("--amplitude", type=float, default=4.0, help="Relay amplitude in degrees.")
    parser.add_argument("--deadband", type=float, default=0.002, help="Deadband around setpoint in meters.")
    parser.add_argument("--cycles", type=int, default=4, help="Number of oscillation cycles to capture.")
    parser.add_argument("--max-duration", type=float, default=120.0, help="Safety timeout in seconds.")
    parser.add_argument("--rule", choices=["zn", "tyreus"], default="zn", help="PID tuning rule.")
    parser.add_argument("--results-file", default="pid_autotune_results.json", help="Where to store tuning logs.")
    parser.add_argument("--display", action="store_true", help="Show OpenCV window with overlays.")
    parser.add_argument("--simulate", action="store_true", help="Skip servo connection, log data only.")
    parser.add_argument(
        "--apply-config",
        action="store_true",
        help="Write the tuned gains back into config_stewart.json for both axes.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    runner = RelayAutotuneRunner(
        config_path=args.config,
        axis=args.axis,
        relay_amplitude_deg=args.amplitude,
        deadband_m=args.deadband,
        cycles=args.cycles,
        max_duration=args.max_duration,
        display=args.display,
        simulate=args.simulate,
    )

    print(
        f"[AUTOTUNE] Starting relay test on axis {args.axis.upper()} "
        f"with ±{args.amplitude:.1f}° command, deadband {args.deadband*1000:.1f} mm."
    )

    raw = runner.run()
    summary = _summarize_results(raw, args.amplitude)
    pid = _compute_pid_from_rule(summary["Ku"], summary["Tu"], args.rule)

    entry = {
        "timestamp": datetime.utcnow().isoformat() + "Z",
        "axis": args.axis,
        "rule": pid["rule"],
        "Ku": summary["Ku"],
        "Tu": summary["Tu"],
        "relay_amplitude_deg": summary["relay_amplitude"],
        "avg_peak_scaled": summary["avg_peak_scaled"],
        "cycles_used": summary["cycles_used"],
        "Kp": pid["Kp"],
        "Ki": pid["Ki"],
        "Kd": pid["Kd"],
    }

    _write_results(args.results_file, entry)

    print("\n[AUTOTUNE] Completed relay test:")
    print(f"  Ultimate gain Ku      : {summary['Ku']:.3f}")
    print(f"  Ultimate period Tu    : {summary['Tu']:.3f} s")
    print(f"  Avg. ball amplitude   : {summary['avg_peak_scaled'] / 100.0:.4f} m")
    print(f"  PID rule              : {pid['rule']}")
    print(f"  -> Kp={pid['Kp']:.3f}, Ki={pid['Ki']:.3f}, Kd={pid['Kd']:.3f}")

    if args.apply_config:
        backup = _apply_to_config(args.config, pid)
        print(f"[AUTOTUNE] Updated {args.config} (backup at {backup}).")
    else:
        print("[AUTOTUNE] To apply gains, rerun with --apply-config or edit config manually.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[AUTOTUNE] Interrupted by user.")
        sys.exit(1)
    except Exception as exc:
        print(f"[AUTOTUNE] Error: {exc}")
        sys.exit(1)

