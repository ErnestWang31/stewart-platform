"""
Relay autotuning (Åström–Hägglund) controller for the Stewart platform.

Workflow:
1. Run this script.
2. Select axis (roll/pitch), enter the relay amplitude `d` (degrees of tilt), and click Start.
3. The controller toggles the platform tilt between +d and -d, forcing the ball to oscillate.
4. After the response settles, the script measures oscillation amplitude and period, computes
   the equivalent Ku/Tu, and saves everything to `relay_capture_results.json`.
"""

from __future__ import annotations

import json
import math
import queue
import time
from datetime import datetime
from threading import Thread, Lock

import cv2
import numpy as np
import serial
import tkinter as tk
from tkinter import ttk, messagebox

from scipy.signal import find_peaks

from ball_detection_2d import BallDetector2D
from inverse_kinematics import StewartPlatformIK
from calibration_2d import StewartPlatformCalibrator


class RelayAutotuneController:
    """Relay autotuning controller for a selected axis."""

    DEFAULT_CONFIG = {
        "relays": {
            "amplitude_deg": 2.0,       # +/- d
            "settle_time_sec": 5.0,     # Time to settle after each toggle
            "analysis_window_sec": 15.0, # Time window to analyze for oscillations
            "min_amplitude_m": 0.005,    # Minimum amplitude to consider valid
            "period_tolerance": 0.25,    # Tolerance for period variation
            "amplitude_tolerance": 0.30, # Tolerance for amplitude variation
            "min_cycles": 6,             # Minimum number of cycles to consider valid
            "edge_guard_warmup_sec": 3.0, # Time to wait before checking for oscillations
            "max_position_ratio": 0.9,   # Maximum position ratio to consider valid
            "relay_deadband_m": 0.001,    # Deadband for the relay
        }
    }

    def __init__(self, config_file: str = "config_stewart.json") -> None:
        self.config_file = config_file
        self.root = None
        self._ensure_calibration()
        self.config = self._load_config(config_file)

        relay_cfg = {**self.DEFAULT_CONFIG["relays"], **self.config.get("relay_autotune", {})}
        self.relay_amplitude_deg = relay_cfg["amplitude_deg"]
        self.settle_time = relay_cfg["settle_time_sec"]
        self.analysis_window = relay_cfg["analysis_window_sec"]
        self.min_amplitude = relay_cfg["min_amplitude_m"]
        self.period_tolerance = relay_cfg["period_tolerance"]
        self.amplitude_tolerance = relay_cfg["amplitude_tolerance"]
        self.min_cycles = int(max(3, relay_cfg["min_cycles"]))
        self.edge_guard_warmup = relay_cfg["edge_guard_warmup_sec"]
        self.max_position_ratio = relay_cfg["max_position_ratio"]
        self.relay_deadband = relay_cfg["relay_deadband_m"]

        self.detector = BallDetector2D(config_file)
        self.ik_solver = StewartPlatformIK(self.config)

        servo_cfg = self.config.get("servo", {})
        self.servo_port = servo_cfg.get("port") or (servo_cfg.get("ports") or ["COM3"])[0]
        self.servo_baud_rate = servo_cfg.get("baud_rate", 115200)
        self.servo_read_timeout = max(servo_cfg.get("timeout_seconds", 1.0), 0.0)
        write_timeout_ms = servo_cfg.get("write_timeout_ms", 50) or 50
        self.servo_write_timeout = max(write_timeout_ms, 0) / 1000.0
        self.neutral_angles = servo_cfg.get("neutral_angles", [15, 15, 15])
        self.motor_direction_invert = servo_cfg.get("motor_direction_invert", [False, False, False])
        self.roll_direction_invert = self.config.get("platform", {}).get("roll_direction_invert", False)
        self.use_inverse_kinematics = self.config.get("platform", {}).get("use_inverse_kinematics", False)

        self.camera_settings = self.config.get(
            "camera",
            {"index": 0, "frame_width": 640, "frame_height": 480},
        )

        self.position_limit_m = self._compute_position_limit()

        # State
        self.axis_choice = None
        self.amplitude_var = None
        self.status_text = None
        self.start_button = None
        self.axis_label = None

        self.servo_serial = None
        self.detector_queue = queue.Queue(maxsize=1)
        self.running = False
        self.capture_enabled = False
        self.target_axis = None
        self.capture_start_time = None
        self.relay_state = 1
        self.last_toggle_time = None

        self.time_log = []
        self.axis_position_log = []
        self.control_log = []
        self.log_lock = Lock()

        self.results_path = "relay_capture_results.json"
        self.ui_task_queue = queue.Queue()
        self.latest_period = None
        self.latest_amplitude = None
        self.ku_result = None

    # ------------------------------------------------------------------ #
    # Entry point
    # ------------------------------------------------------------------ #
    def run(self) -> None:
        print("[INFO] Starting relay autotune controller")
        self.running = True

        cam_thread = Thread(target=self.camera_thread, daemon=True)
        ctrl_thread = Thread(target=self.control_thread, daemon=True)
        cam_thread.start()
        ctrl_thread.start()

        self._build_gui()
        self.root.mainloop()

        self.running = False
        self.capture_enabled = False
        self.send_platform_tilt(0.0, 0.0)
        if self.servo_serial:
            self.servo_serial.close()
        print("[INFO] Relay autotuning stopped")

    # ------------------------------------------------------------------ #
    # GUI
    # ------------------------------------------------------------------ #
    def _build_gui(self) -> None:
        self.root = tk.Tk()
        self.root.title("Relay Autotune")
        self.root.geometry("420x360")

        self.axis_choice = tk.StringVar(value="x")
        self.amplitude_var = tk.DoubleVar(value=self.relay_amplitude_deg)

        ttk.Label(self.root, text="Relay Autotuning", font=("Arial", 16, "bold")).pack(pady=10)
        ttk.Label(
            self.root,
            text="Select axis and relay amplitude (degrees). Place ball near center,\n"
                 "nudge it slightly along the axis before starting.",
            justify=tk.CENTER,
        ).pack(pady=5)

        axis_frame = ttk.LabelFrame(self.root, text="Axis Selection", padding=10)
        axis_frame.pack(fill=tk.X, padx=20, pady=5)
        ttk.Radiobutton(axis_frame, text="Roll / X-Axis", variable=self.axis_choice, value="x").pack(anchor=tk.W)
        ttk.Radiobutton(axis_frame, text="Pitch / Y-Axis", variable=self.axis_choice, value="y").pack(anchor=tk.W)
        self.axis_label = ttk.Label(axis_frame, text="Current axis: None selected")
        self.axis_label.pack(anchor=tk.W, pady=(8, 0))

        amp_frame = ttk.Frame(self.root, padding=10)
        amp_frame.pack(fill=tk.X, padx=20, pady=5)
        ttk.Label(amp_frame, text="Relay amplitude d (degrees):").pack(side=tk.LEFT)
        amp_entry = ttk.Entry(amp_frame, textvariable=self.amplitude_var, width=10)
        amp_entry.pack(side=tk.LEFT, padx=5)

        btn_frame = ttk.Frame(self.root)
        btn_frame.pack(pady=10)
        self.start_button = ttk.Button(btn_frame, text="Start", command=self._begin_capture)
        self.start_button.pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Stop", command=self._stop).pack(side=tk.LEFT, padx=5)

        status_frame = ttk.LabelFrame(self.root, text="Status", padding=10)
        status_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=5)
        self.status_text = tk.Text(status_frame, height=8, width=44, state=tk.DISABLED)
        self.status_text.pack(fill=tk.BOTH, expand=True)
        self._update_status("Ready. Select axis and relay amplitude, then click Start.")
        self._drain_ui_queue()

    def _update_status(self, message: str) -> None:
        if not self.status_text:
            return
        self.status_text.config(state=tk.NORMAL)
        self.status_text.delete("1.0", tk.END)
        self.status_text.insert("1.0", message)
        self.status_text.config(state=tk.DISABLED)

    def _stop(self) -> None:
        self.capture_enabled = False
        self.relay_state = 1
        self._update_status("Stopping capture...")
        self.start_button.config(state=tk.NORMAL)

    def _begin_capture(self) -> None:
        if self.capture_enabled:
            messagebox.showinfo("Capture running", "Relay capture already in progress.")
            return

        axis = self.axis_choice.get()
        if axis not in ("x", "y"):
            messagebox.showerror("Invalid axis", "Select X or Y axis before starting.")
            return

        amplitude = self.amplitude_var.get()
        if amplitude <= 0 or amplitude > 15:
            messagebox.showerror("Invalid amplitude", "Amplitude must be between 0 and 15 degrees.")
            return

        self.axis_label.config(text=f"Current axis: {'Roll/X' if axis == 'x' else 'Pitch/Y'}")
        self.relay_amplitude_deg = amplitude
        self.target_axis = axis
        self.capture_enabled = True
        self.capture_start_time = None
        self.relay_state = 1
        self.last_toggle_time = None
        self.ku_result = None
        self.latest_period = None
        self.latest_amplitude = None

        with self.log_lock:
            self.time_log.clear()
            self.axis_position_log.clear()
            self.control_log.clear()

        self.start_button.config(state=tk.DISABLED)
        self._update_status(
            f"Relay capture running on {self._axis_label(axis)}\n"
            f"Amplitude d = {self.relay_amplitude_deg:.2f}°.\n"
            "Wait for oscillations to stabilize..."
        )

        monitor_thread = Thread(target=self._monitor_capture, daemon=True)
        monitor_thread.start()

    # ------------------------------------------------------------------ #
    # Capture monitoring
    # ------------------------------------------------------------------ #
    def _monitor_capture(self) -> None:
        time.sleep(self.settle_time)
        while self.running and self.capture_enabled:
            time.sleep(self.analysis_window)
            if not self.capture_enabled or not self.running:
                break
            window = self._get_recent_window(self.analysis_window)
            if not window:
                continue
            times, values = window

            guard_active = (
                self.capture_start_time is not None
                and (time.time() - self.capture_start_time) >= self.edge_guard_warmup
            )
            if guard_active and self.position_limit_m and np.max(np.abs(values)) > self.position_limit_m:
                print("[RELAY] Window rejected: ball hitting platform edge.")
                continue

            amplitude, period = self._analyze_window(times, values)
            if amplitude is None or period is None:
                continue

            self.latest_amplitude = amplitude
            self.latest_period = period
            self.ku_result = (4 * self.relay_amplitude_deg) / (math.pi * amplitude)
            self._run_in_ui(
                self._update_status,
                f"Relay capture complete.\n"
                f"Amplitude a = {amplitude:.4f} m\n"
                f"Period Tu = {period:.4f} s\n"
                f"Ku = {self.ku_result:.4f}\nResults saved to {self.results_path}",
            )
            self._save_results()
            self.capture_enabled = False
            self._run_in_ui(self.start_button.config, state=tk.NORMAL)
            return

        if self.capture_enabled:
            self._run_in_ui(
                self._update_status,
                "Relay capture ended without finding sustained oscillations.",
            )
            self.capture_enabled = False
            self._run_in_ui(self.start_button.config, state=tk.NORMAL)

    def _analyze_window(self, times: np.ndarray, values: np.ndarray):
        if times.size < 10:
            return None, None

        centered = values - np.mean(values)
        amplitude = 0.5 * (centered.max() - centered.min())
        if amplitude < self.min_amplitude:
            return None, None

        peaks, _ = find_peaks(centered, prominence=self.min_amplitude / 3.0)
        troughs, _ = find_peaks(-centered, prominence=self.min_amplitude / 3.0)
        if len(peaks) < self.min_cycles or len(troughs) < self.min_cycles:
            return None, None

        if len(peaks) >= 2:
            periods = np.diff(times[peaks])
            periods = periods[periods > 0]
            if len(periods) < 2:
                return None, None
            mean_period = np.mean(periods)
            spread = np.max(periods) - np.min(periods)
            if mean_period <= 0 or spread / mean_period > self.period_tolerance:
                return None, None
        else:
            return None, None

        recent_peaks = centered[peaks[-self.min_cycles:]]
        recent_troughs = centered[troughs[-self.min_cycles:]]
        peak_span = np.max(recent_peaks) - np.min(recent_peaks)
        trough_span = np.max(recent_troughs) - np.min(recent_troughs)
        tolerance = max(self.amplitude_tolerance * amplitude, self.min_amplitude / 2)
        if peak_span > tolerance or trough_span > tolerance:
            return None, None

        return amplitude, mean_period

    def _get_recent_window(self, window_sec: float):
        with self.log_lock:
            if not self.time_log:
                return None
            cutoff = self.time_log[-1] - window_sec
            if cutoff < 0:
                cutoff = 0
            indices = [i for i, t in enumerate(self.time_log) if t >= cutoff]
            if len(indices) < 5:
                return None
            times = np.array([self.time_log[i] for i in indices])
            values = np.array([self.axis_position_log[i] for i in indices])
            return times, values

    def _save_results(self) -> None:
        payload = {
            "timestamp": datetime.now().isoformat(),
            "axis": self.target_axis,
            "d_degrees": self.relay_amplitude_deg,
            "amplitude_m": self.latest_amplitude,
            "Tu": self.latest_period,
            "Ku": self.ku_result,
            "time": self.time_log,
            "axis_position": self.axis_position_log,
            "control": self.control_log,
        }
        with open(self.results_path, "w") as f:
            json.dump(payload, f, indent=2)
        print(
            f"[RESULT] Relay autotune saved: Ku={self.ku_result:.4f}, "
            f"Tu={self.latest_period:.4f} s -> {self.results_path}"
        )

    # ------------------------------------------------------------------ #
    # Threads
    # ------------------------------------------------------------------ #
    def camera_thread(self) -> None:
        cam = cv2.VideoCapture(self.camera_settings["index"], cv2.CAP_DSHOW)
        cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_settings.get("frame_width", 640))
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_settings.get("frame_height", 480))

        while self.running:
            ret, frame = cam.read()
            if not ret:
                continue
            frame = cv2.resize(
                frame,
                (
                    self.camera_settings.get("frame_width", 640),
                    self.camera_settings.get("frame_height", 480),
                ),
            )
            found, _, _, pos_x, pos_y = self.detector.detect_ball(frame)
            if found:
                try:
                    if self.detector_queue.full():
                        self.detector_queue.get_nowait()
                    self.detector_queue.put_nowait((pos_x, pos_y))
                except queue.Full:
                    pass

            vis_frame, _, _, _ = self.detector.draw_detection(frame, show_info=True)
            cv2.imshow("Relay Autotune - Ball Tracking", vis_frame)
            if cv2.waitKey(1) & 0xFF == 27:
                self._run_in_ui(self._stop)
                break

        cam.release()
        cv2.destroyAllWindows()

    def control_thread(self) -> None:
        if not self.connect_servos():
            print("[WARNING] Could not connect to servos. Running in simulation mode.")
        else:
            self._send_neutral_pose()
            print("[INFO] Platform set to neutral pose. Place ball near center.")

        while self.running:
            try:
                position_x, position_y = self.detector_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            if self.capture_enabled:
                if self.capture_start_time is None:
                    self.capture_start_time = time.time()
                axis_position = position_x if self.target_axis == "x" else position_y
                setpoint = 0.0
                error = setpoint - axis_position

                desired_state = 1 if error >= self.relay_deadband else -1
                if abs(error) < self.relay_deadband and self.relay_state != 0:
                    desired_state = self.relay_state

                if desired_state != self.relay_state:
                    self.relay_state = desired_state
                    self.last_toggle_time = time.time()

                roll_cmd = self.relay_amplitude_deg * self.relay_state if self.target_axis == "x" else 0.0
                pitch_cmd = self.relay_amplitude_deg * self.relay_state if self.target_axis == "y" else 0.0
                self.send_platform_tilt(roll_cmd, pitch_cmd)

                relay_output = roll_cmd if self.target_axis == "x" else pitch_cmd
                timestamp = (
                    time.time() - self.capture_start_time if self.capture_start_time else 0.0
                )
                with self.log_lock:
                    self.time_log.append(timestamp)
                    self.axis_position_log.append(axis_position)
                    self.control_log.append(relay_output)
            else:
                time.sleep(0.02)

        self.send_platform_tilt(0.0, 0.0)

    # ------------------------------------------------------------------ #
    # Hardware helpers
    # ------------------------------------------------------------------ #
    def connect_servos(self) -> bool:
        try:
            self.servo_serial = serial.Serial(
                self.servo_port,
                self.servo_baud_rate,
                timeout=self.servo_read_timeout,
                write_timeout=self.servo_write_timeout,
            )
            time.sleep(2)
            self.servo_serial.reset_input_buffer()
            print(f"[ARDUINO] Connected to {self.servo_port} @ {self.servo_baud_rate} baud")
            return True
        except Exception as exc:
            print(f"[ARDUINO] Failed to connect to {self.servo_port}: {exc}")
            self.servo_serial = None
            return False

    def send_platform_tilt(self, roll_angle: float, pitch_angle: float) -> None:
        roll_angle = float(np.clip(roll_angle, -15, 15))
        pitch_angle = float(np.clip(pitch_angle, -15, 15))

        if self.roll_direction_invert:
            roll_angle = -roll_angle

        if self.use_inverse_kinematics:
            try:
                theta_11, theta_21, theta_31 = self.ik_solver.get_motor_angles(roll_angle, pitch_angle)
                angle_scale = self.config.get("platform", {}).get("ik_angle_scale", 1.0)
                angle_offset = self.config.get("platform", {}).get("ik_angle_offset", 0.0)
                dirs = [-1.0 if flag else 1.0 for flag in self.motor_direction_invert]
                motor_angles = [
                    self.neutral_angles[i] + (theta * angle_scale + angle_offset) * dirs[i]
                    for i, theta in enumerate((theta_11, theta_21, theta_31))
                ]
            except Exception as exc:
                print(f"[IK] Error {exc}, using simplified mapping.")
                motor_angles = self._simplified_motor_mapping(roll_angle, pitch_angle)
        else:
            motor_angles = self._simplified_motor_mapping(roll_angle, pitch_angle)

        motor_packet = bytes(int(np.clip(angle, 0, 30)) for angle in motor_angles)

        if self.servo_serial:
            try:
                if self.servo_serial.in_waiting > 50:
                    self.servo_serial.reset_input_buffer()
                self.servo_serial.write(motor_packet)
            except Exception as exc:
                print(f"[ARDUINO] Send failed: {exc}")

    def _simplified_motor_mapping(self, roll_angle: float, pitch_angle: float):
        motor_angles_deg = self.config.get("motor_angles_deg")
        if motor_angles_deg and len(motor_angles_deg) == 3:
            base_angles = [motor_angles_deg[i] - 180 for i in range(3)]
        else:
            base_angles = [-90, -210, -330]

        base_rads = [np.radians(angle) for angle in base_angles]
        heights = [
            -roll_angle * np.cos(rad) - pitch_angle * np.sin(rad)
            for rad in base_rads
        ]

        scale_factor = self.config.get("platform", {}).get("motor_scale_factor", 1.0)
        dirs = [-1.0 if flag else 1.0 for flag in self.motor_direction_invert]
        return [self.neutral_angles[i] + heights[i] * scale_factor * dirs[i] for i in range(3)]

    def _send_neutral_pose(self) -> None:
        if not self.servo_serial:
            return
        neutral_packet = bytes(int(np.clip(angle, 0, 30)) for angle in self.neutral_angles)
        try:
            self.servo_serial.write(neutral_packet)
            print("[MOTOR] Neutral pose command sent.")
        except Exception as exc:
            print(f"[MOTOR] Failed to send neutral pose: {exc}")

    # ------------------------------------------------------------------ #
    # Utility helpers
    # ------------------------------------------------------------------ #
    @staticmethod
    def _axis_label(axis: str) -> str:
        return "Roll / X-Axis" if axis == "x" else "Pitch / Y-Axis"

    @staticmethod
    def _load_config(config_file: str) -> dict:
        with open(config_file, "r") as f:
            return json.load(f)

    def _ensure_calibration(self) -> None:
        try:
            calibrator = StewartPlatformCalibrator()
            calibrator.run()
        except Exception as exc:
            print(f"[CALIBRATION] Skipping automatic calibration: {exc}")

    def _compute_position_limit(self) -> float:
        calib = self.config.get("calibration", {})
        platform_radius = self.config.get("platform_radius_m") or self.config.get("platform", {}).get(
            "platform_radius_m"
        )
        candidates = [
            abs(calib.get("position_max_x_m", 0.0)),
            abs(calib.get("position_min_x_m", 0.0)),
            abs(calib.get("position_max_y_m", 0.0)),
            abs(calib.get("position_min_y_m", 0.0)),
            platform_radius or 0.0,
        ]
        radius = max(candidates) if any(candidates) else 0.15
        return radius * self.max_position_ratio if radius > 0 else None

    # ------------------------------------------------------------------ #
    # UI helpers
    # ------------------------------------------------------------------ #
    def _run_in_ui(self, func, *args, **kwargs):
        if self.root and self.root.winfo_exists():
            self.root.after(0, func, *args, **kwargs)
        else:
            self.ui_task_queue.put((func, args, kwargs))

    def _drain_ui_queue(self):
        while not self.ui_task_queue.empty():
            func, args, kwargs = self.ui_task_queue.get()
            try:
                func(*args, **kwargs)
            except Exception as exc:
                print(f"[UI] Error executing scheduled task: {exc}")
        if self.root and self.root.winfo_exists():
            self.root.after(50, self._drain_ui_queue)


if __name__ == "__main__":
    try:
        controller = RelayAutotuneController()
        controller.run()
    except Exception as exc:
        print(f"[ERROR] {exc}")
        import traceback

        traceback.print_exc()

