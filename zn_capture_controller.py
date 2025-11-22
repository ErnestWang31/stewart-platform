"""
Automated Ziegler–Nichols data-capture controller.

This script reuses the full Stewart platform stack (ball detection, PID controller,
and servo mapping) but drives a dedicated workflow for finding the ultimate gain
Ku and oscillation period Tu along a single axis. The typical workflow is:

1. Run this script.
2. Select which axis you want to tune (roll/X or pitch/Y) in the GUI.
3. Click "Start Capture".
4. The controller will zero Ki/Kd, slowly ramp Kp, and watch the response.
5. Once sustained oscillations are detected, Ku/Tu are saved to JSON and a plot
   comparing setpoint vs. measured position is displayed.

The resulting JSON can then be consumed by a separate tuning script to compute the
final PID gains via the Ziegler–Nichols formulas.
"""

from __future__ import annotations

import json
import queue
import time
from datetime import datetime
from threading import Thread, Lock

import cv2
import numpy as np
import serial
import tkinter as tk
from tkinter import ttk, messagebox

from matplotlib import pyplot as plt
from scipy.signal import find_peaks

from ball_detection_2d import BallDetector2D
from pid_controller_2d import PIDController2D
from inverse_kinematics import StewartPlatformIK


class ZNCaptureController:
    """Controller that automates Ku/Tu capture for a chosen axis."""

    DEFAULT_CAPTURE_CONFIG = {
        "kp_step": 0.15,
        "kp_max": 20.0,
        "dwell_time_sec": 4.0,
        "analysis_window_sec": 8.0,
        "min_amplitude_m": 0.005,  # 5 mm
        "period_tolerance": 0.25,
        "amplitude_tolerance": 0.35,
    }

    def __init__(self, config_file: str = "config_stewart.json") -> None:
        self.config_file = config_file
        self.config = self._load_config(config_file)

        self.detector = BallDetector2D(config_file)
        self.pid = PIDController2D(
            Kp_x=self.config.get("pid", {}).get("Kp_x", 0.0),
            Ki_x=self.config.get("pid", {}).get("Ki_x", 0.0),
            Kd_x=self.config.get("pid", {}).get("Kd_x", 0.0),
            Kp_y=self.config.get("pid", {}).get("Kp_y", 0.0),
            Ki_y=self.config.get("pid", {}).get("Ki_y", 0.0),
            Kd_y=self.config.get("pid", {}).get("Kd_y", 0.0),
            output_limit_x=self.config.get("platform", {}).get("max_roll_angle", 15.0),
            output_limit_y=self.config.get("platform", {}).get("max_pitch_angle", 15.0),
        )
        self.pid.set_setpoint(0.0, 0.0)

        self.ik_solver = StewartPlatformIK(self.config)
        self.use_inverse_kinematics = False

        servo_config = self.config.get("servo", {})
        self.servo_port = servo_config.get("port") or (servo_config.get("ports") or ["COM3"])[0]
        self.neutral_angles = servo_config.get("neutral_angles", [15, 15, 15])
        self.motor_direction_invert = servo_config.get(
            "motor_direction_invert", [False, False, False]
        )
        self.servo_serial = None

        self.camera_settings = self.config.get(
            "camera",
            {"index": 0, "frame_width": 640, "frame_height": 480},
        )

        capture_cfg = self.config.get("zn_capture", {})
        merged_cfg = {**self.DEFAULT_CAPTURE_CONFIG, **capture_cfg}
        self.kp_step = merged_cfg["kp_step"]
        self.kp_max = merged_cfg["kp_max"]
        self.dwell_time = merged_cfg["dwell_time_sec"]
        self.analysis_window = merged_cfg["analysis_window_sec"]
        self.min_amplitude = merged_cfg["min_amplitude_m"]
        self.period_tolerance = merged_cfg["period_tolerance"]
        self.amplitude_tolerance = merged_cfg["amplitude_tolerance"]
        self.min_prominence = self.min_amplitude / 3.0

        self.axis_choice = None  # Will be created in _build_gui() after root window exists
        self.capture_enabled = False
        self.target_axis = None

        self.time_log = []
        self.position_x_log = []
        self.position_y_log = []
        self.control_x_log = []
        self.control_y_log = []
        self.setpoint_x_log = []
        self.setpoint_y_log = []

        self.capture_start_time = None
        self.current_kp = 0.0
        self.base_gains = {
            "x": (
                self.pid.Kp_x,
                self.pid.Ki_x,
                self.pid.Kd_x,
            ),
            "y": (
                self.pid.Kp_y,
                self.pid.Ki_y,
                self.pid.Kd_y,
            ),
        }

        self.running = False
        self.position_queue = queue.Queue(maxsize=1)
        self.tuning_thread = None
        self.status_text = None
        self.start_button = None
        self.axis_label = None

        self.log_lock = Lock()

        self.ku_found = False
        self.result_ku = None
        self.result_tu = None
        self.results_path = "zn_capture_results.json"

    # ------------------------------------------------------------------ #
    # Public entry point
    # ------------------------------------------------------------------ #
    def run(self) -> None:
        print("[INFO] Starting Ziegler–Nichols capture controller")
        print("Select axis, click 'Start Capture', and let the routine find Ku/Tu.")
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
        print("[INFO] ZN capture controller stopped")

    # ------------------------------------------------------------------ #
    # GUI
    # ------------------------------------------------------------------ #
    def _build_gui(self) -> None:
        self.root = tk.Tk()
        self.root.title("ZN Ku/Tu Capture")
        self.root.geometry("420x320")

        # Create tkinter variables after root window exists
        self.axis_choice = tk.StringVar(value="x")

        header = ttk.Label(
            self.root,
            text="Ziegler–Nichols Ultimate Gain Capture",
            font=("Arial", 14, "bold"),
        )
        header.pack(pady=10)

        instructions = ttk.Label(
            self.root,
            text=(
                "Choose axis, set platform steady, then click Start.\n"
                "The controller will ramp Kp until sustained oscillations."
            ),
            justify=tk.CENTER,
        )
        instructions.pack(pady=5)

        axis_frame = ttk.LabelFrame(self.root, text="Axis Selection", padding=10)
        axis_frame.pack(fill=tk.X, padx=20, pady=5)

        ttk.Radiobutton(
            axis_frame, text="Roll / X-Axis", variable=self.axis_choice, value="x"
        ).pack(anchor=tk.W)
        ttk.Radiobutton(
            axis_frame, text="Pitch / Y-Axis", variable=self.axis_choice, value="y"
        ).pack(anchor=tk.W)

        self.axis_label = ttk.Label(axis_frame, text="Current axis: None selected")
        self.axis_label.pack(anchor=tk.W, pady=(8, 0))

        btn_frame = ttk.Frame(self.root)
        btn_frame.pack(pady=15)

        self.start_button = ttk.Button(
            btn_frame, text="Start Capture", command=self._begin_capture
        )
        self.start_button.pack(side=tk.LEFT, padx=5)

        ttk.Button(btn_frame, text="Stop", command=self._stop).pack(side=tk.LEFT, padx=5)

        status_frame = ttk.LabelFrame(self.root, text="Status", padding=10)
        status_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=5)
        self.status_text = tk.Text(status_frame, height=6, width=38, state=tk.DISABLED)
        self.status_text.pack(fill=tk.BOTH, expand=True)
        self._update_status("Ready. Select axis and press Start.")

    def _update_status(self, message: str) -> None:
        if not self.status_text:
            return
        self.status_text.config(state=tk.NORMAL)
        self.status_text.delete("1.0", tk.END)
        self.status_text.insert("1.0", message)
        self.status_text.config(state=tk.DISABLED)

    def _stop(self) -> None:
        self.capture_enabled = False
        self.running = False
        self._update_status("Stopping...")
        self.root.quit()

    def _begin_capture(self) -> None:
        if self.capture_enabled:
            messagebox.showinfo("Capture in progress", "Capture already running.")
            return

        axis = self.axis_choice.get()
        if axis not in ("x", "y"):
            messagebox.showerror("Invalid axis", "Select X or Y axis before starting.")
            return

        self.axis_label.config(text=f"Current axis: {'Roll/X' if axis == 'x' else 'Pitch/Y'}")

        self._reset_logs()
        self.pid.reset_integral()
        self.capture_enabled = True
        self.target_axis = axis
        self.ku_found = False
        self.result_ku = None
        self.result_tu = None
        self.capture_start_time = None
        self.current_kp = 0.0

        self._apply_axis_gains(axis, 0.0, 0.0, 0.0)
        other_axis = "y" if axis == "x" else "x"
        self._apply_axis_gains(other_axis, *self.base_gains[other_axis])

        if not self.tuning_thread or not self.tuning_thread.is_alive():
            self.tuning_thread = Thread(target=self._run_tuning_loop, daemon=True)
            self.tuning_thread.start()

        self._update_status(
            f"Capturing on {self._axis_label(axis)}.\n"
            "Kp will ramp automatically until sustained oscillations are detected."
        )
        self.start_button.config(state=tk.DISABLED)

    # ------------------------------------------------------------------ #
    # Capture / tuning logic
    # ------------------------------------------------------------------ #
    def _run_tuning_loop(self) -> None:
        time.sleep(1.5)  # allow control loop to settle
        while (
            self.capture_enabled
            and self.running
            and not self.ku_found
            and self.current_kp <= self.kp_max
        ):
            self.current_kp += self.kp_step
            self._apply_axis_gains(self.target_axis, self.current_kp, 0.0, 0.0)
            self._update_status(
                f"{self._axis_label(self.target_axis)} | "
                f"Kp={self.current_kp:.3f}, Ki=0, Kd=0\n"
                "Waiting for response..."
            )
            dwell_start = time.time()
            while (
                self.running
                and self.capture_enabled
                and not self.ku_found
                and (time.time() - dwell_start) < self.dwell_time
            ):
                time.sleep(0.05)

            if not self.running or not self.capture_enabled or self.ku_found:
                break

            if self._check_for_ultimate():
                self.result_ku = round(self.current_kp, 4)
                self.result_tu = round(self.latest_period, 4)
                self.ku_found = True
                self._update_status(
                    f"Ku found: {self.result_ku:.4f}\n"
                    f"Tu: {self.result_tu:.4f} s\nSaving results..."
                )
                self._finalize_capture(success=True)
                return

        if not self.ku_found:
            self._update_status("Capture ended without finding sustained oscillations.")
            self._finalize_capture(success=False)

    def _apply_axis_gains(self, axis: str, kp: float, ki: float, kd: float) -> None:
        if axis == "x":
            self.pid.set_gains_x(kp, ki, kd)
        else:
            self.pid.set_gains_y(kp, ki, kd)

    def _check_for_ultimate(self) -> bool:
        window = self._get_recent_window(self.target_axis, self.analysis_window)
        if window is None:
            return False

        times, values = window
        if len(times) < 10:
            return False

        centered = values - np.mean(values)
        amplitude = 0.5 * (centered.max() - centered.min())
        if amplitude < self.min_amplitude:
            return False

        peaks, _ = find_peaks(centered, prominence=self.min_prominence)
        troughs, _ = find_peaks(-centered, prominence=self.min_prominence)
        min_cycles = 4
        if len(peaks) < min_cycles or len(troughs) < min_cycles:
            return False

        if len(peaks) >= 2:
            periods = np.diff(times[peaks])
            if len(periods) < 2:
                return False
            mean_period = np.mean(periods)
            if mean_period <= 0:
                return False
            spread = np.max(periods) - np.min(periods)
            if spread / mean_period > self.period_tolerance:
                return False
        else:
            return False

        recent_peaks = centered[peaks[-min_cycles:]]
        recent_troughs = centered[troughs[-min_cycles:]]
        peak_span = np.max(recent_peaks) - np.min(recent_peaks)
        trough_span = np.max(recent_troughs) - np.min(recent_troughs)
        if peak_span > amplitude * self.amplitude_tolerance:
            return False
        if trough_span > amplitude * self.amplitude_tolerance:
            return False

        self.latest_period = mean_period
        return True

    def _get_recent_window(self, axis: str, window_sec: float):
        with self.log_lock:
            if not self.time_log:
                return None
            cutoff = self.time_log[-1] - window_sec
            if cutoff < 0:
                cutoff = 0
            indices = [i for i, t in enumerate(self.time_log) if t >= cutoff]
            if len(indices) < 5:
                return None
            if axis == "x":
                values = np.array([self.position_x_log[i] for i in indices])
            else:
                values = np.array([self.position_y_log[i] for i in indices])
            times = np.array([self.time_log[i] for i in indices])
            return times, values

    def _finalize_capture(self, success: bool) -> None:
        self.capture_enabled = False
        self.capture_start_time = None
        self._apply_axis_gains("x", *self.base_gains["x"])
        self._apply_axis_gains("y", *self.base_gains["y"])
        self.send_platform_tilt(0.0, 0.0)
        self.start_button.config(state=tk.NORMAL)

        if success and self.result_ku and self.result_tu:
            self._save_results()
            self._plot_results(self.target_axis)
            self._update_status(
                f"Capture complete.\nKu={self.result_ku:.4f}, Tu={self.result_tu:.4f} s\n"
                f"Results saved to {self.results_path}."
            )
        elif not success:
            messagebox.showwarning(
                "Capture incomplete",
                "Could not detect sustained oscillations before reaching Kp limit.",
            )

    def _save_results(self) -> None:
        payload = {
            "timestamp": datetime.now().isoformat(),
            "axis": self.target_axis,
            "Ku": self.result_ku,
            "Tu": self.result_tu,
            "kp_step": self.kp_step,
            "kp_max": self.kp_max,
            "dwell_time_sec": self.dwell_time,
            "analysis_window_sec": self.analysis_window,
            "min_amplitude_m": self.min_amplitude,
            "period_tolerance": self.period_tolerance,
            "amplitude_tolerance": self.amplitude_tolerance,
            "time": self.time_log,
            "position_x": self.position_x_log,
            "position_y": self.position_y_log,
            "control_x": self.control_x_log,
            "control_y": self.control_y_log,
            "setpoint_x": self.setpoint_x_log,
            "setpoint_y": self.setpoint_y_log,
        }
        with open(self.results_path, "w") as f:
            json.dump(payload, f, indent=2)
        print(f"[RESULT] Ku={self.result_ku:.4f}, Tu={self.result_tu:.4f} saved to {self.results_path}")

    def _plot_results(self, axis: str) -> None:
        if not self.time_log:
            return
        plt.figure(figsize=(10, 5))
        if axis == "x":
            plt.plot(self.time_log, self.position_x_log, label="Ball X Position")
            plt.plot(self.time_log, self.setpoint_x_log, "--", label="Setpoint X")
            plt.ylabel("Position X (m)")
            plt.title("Roll/X Axis Response During Ku Capture")
        else:
            plt.plot(self.time_log, self.position_y_log, color="green", label="Ball Y Position")
            plt.plot(self.time_log, self.setpoint_y_log, "--", color="orange", label="Setpoint Y")
            plt.ylabel("Position Y (m)")
            plt.title("Pitch/Y Axis Response During Ku Capture")
        plt.xlabel("Time (s)")
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.show()

    def _reset_logs(self) -> None:
        with self.log_lock:
            self.time_log.clear()
            self.position_x_log.clear()
            self.position_y_log.clear()
            self.control_x_log.clear()
            self.control_y_log.clear()
            self.setpoint_x_log.clear()
            self.setpoint_y_log.clear()

    # ------------------------------------------------------------------ #
    # Threads: camera + control
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
                    if self.position_queue.full():
                        self.position_queue.get_nowait()
                    self.position_queue.put_nowait((pos_x, pos_y))
                except queue.Full:
                    pass

            vis_frame, _, _, _ = self.detector.draw_detection(frame, show_info=True)
            cv2.imshow("ZN Capture - Ball Tracking", vis_frame)
            if cv2.waitKey(1) & 0xFF == 27:
                self._stop()
                break

        cam.release()
        cv2.destroyAllWindows()

    def control_thread(self) -> None:
        if not self.connect_servos():
            print("[WARNING] Could not connect to servos. Running in simulation mode.")

        while self.running:
            try:
                position_x, position_y = self.position_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            if self.capture_enabled:
                if self.capture_start_time is None:
                    self.capture_start_time = time.time()

                control_x, control_y = self.pid.update(position_x, position_y)
                self.send_platform_tilt(control_x, control_y)
                elapsed = time.time() - self.capture_start_time
                self._log_sample(
                    elapsed, position_x, position_y, control_x, control_y
                )
            else:
                time.sleep(0.02)

        self.send_platform_tilt(0.0, 0.0)

    def _log_sample(
        self, timestamp: float, pos_x: float, pos_y: float, ctrl_x: float, ctrl_y: float
    ) -> None:
        with self.log_lock:
            self.time_log.append(timestamp)
            self.position_x_log.append(pos_x)
            self.position_y_log.append(pos_y)
            self.control_x_log.append(ctrl_x)
            self.control_y_log.append(ctrl_y)
            self.setpoint_x_log.append(self.pid.setpoint_x)
            self.setpoint_y_log.append(self.pid.setpoint_y)

    # ------------------------------------------------------------------ #
    # Hardware helpers
    # ------------------------------------------------------------------ #
    @staticmethod
    def _load_config(config_file: str) -> dict:
        try:
            with open(config_file, "r") as f:
                return json.load(f)
        except FileNotFoundError:
            print(f"[WARNING] Config file {config_file} not found. Using defaults.")
            return {}

    def connect_servos(self) -> bool:
        try:
            self.servo_serial = serial.Serial(self.servo_port, 9600, timeout=1)
            time.sleep(2)
            self.servo_serial.reset_input_buffer()
            print(f"[ARDUINO] Connected to {self.servo_port}")
            return True
        except Exception as exc:
            print(f"[ARDUINO] Failed to connect to {self.servo_port}: {exc}")
            self.servo_serial = None
            return False

    def send_platform_tilt(self, roll_angle: float, pitch_angle: float) -> None:
        roll_angle = float(np.clip(roll_angle, -15, 15))
        pitch_angle = float(np.clip(pitch_angle, -15, 15))

        roll_direction_invert = self.config.get("platform", {}).get(
            "roll_direction_invert", False
        )
        if roll_direction_invert:
            roll_angle = -roll_angle

        if self.use_inverse_kinematics:
            try:
                theta_11, theta_21, theta_31 = self.ik_solver.get_motor_angles(
                    roll_angle, pitch_angle
                )
                angle_scale = self.config.get("platform", {}).get("ik_angle_scale", 1.0)
                angle_offset = self.config.get("platform", {}).get("ik_angle_offset", 0.0)
                motor1_dir = -1.0 if self.motor_direction_invert[0] else 1.0
                motor2_dir = -1.0 if self.motor_direction_invert[1] else 1.0
                motor3_dir = -1.0 if self.motor_direction_invert[2] else 1.0
                motor1_angle = self.neutral_angles[0] + (theta_11 * angle_scale + angle_offset) * motor1_dir
                motor2_angle = self.neutral_angles[1] + (theta_21 * angle_scale + angle_offset) * motor2_dir
                motor3_angle = self.neutral_angles[2] + (theta_31 * angle_scale + angle_offset) * motor3_dir
            except Exception as exc:
                print(f"[IK] Error ({exc}). Falling back to simplified mapping.")
                motor1_angle, motor2_angle, motor3_angle = self._simplified_motor_mapping(
                    roll_angle, pitch_angle
                )
        else:
            motor1_angle, motor2_angle, motor3_angle = self._simplified_motor_mapping(
                roll_angle, pitch_angle
            )

        motor1_angle = int(np.clip(motor1_angle, 0, 30))
        motor2_angle = int(np.clip(motor2_angle, 0, 30))
        motor3_angle = int(np.clip(motor3_angle, 0, 30))

        if self.servo_serial:
            try:
                if self.servo_serial.in_waiting > 50:
                    self.servo_serial.reset_input_buffer()
                self.servo_serial.write(bytes([motor1_angle, motor2_angle, motor3_angle]))
            except Exception as exc:
                print(f"[ARDUINO] Send failed: {exc}")

    def _simplified_motor_mapping(self, roll_angle: float, pitch_angle: float):
        motor_positions = [90, 210, 330]
        motor_rads = [np.radians(angle) for angle in motor_positions]
        heights = []
        for angle_rad in motor_rads:
            height = -roll_angle * np.cos(angle_rad) - pitch_angle * np.sin(angle_rad)
            heights.append(height)

        scale_factor = self.config.get("platform", {}).get("motor_scale_factor", 1.0)
        dirs = [-1.0 if flag else 1.0 for flag in self.motor_direction_invert]

        motor_angles = []
        for i in range(3):
            motor_angles.append(
                self.neutral_angles[i] + heights[i] * scale_factor * dirs[i]
            )
        return motor_angles

    # ------------------------------------------------------------------ #
    # Utility
    # ------------------------------------------------------------------ #
    @staticmethod
    def _axis_label(axis: str) -> str:
        return "X-Axis (Roll)" if axis == "x" else "Y-Axis (Pitch)"


if __name__ == "__main__":
    try:
        controller = ZNCaptureController()
        controller.run()
    except Exception as exc:
        print(f"[ERROR] {exc}")
        import traceback

        traceback.print_exc()

