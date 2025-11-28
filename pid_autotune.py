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
import platform
import shutil
import sys
import threading
import time
from datetime import datetime
from queue import Queue
from typing import Callable, Optional

import cv2
import numpy as np

try:
    from scipy import signal, optimize
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False

try:
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False

from stewart_platform_controller import StewartPlatformController

try:
    import tkinter as tk
    from tkinter import ttk, messagebox
except Exception:  # pragma: no cover - Tk may be unavailable
    tk = None
    ttk = None
    messagebox = None


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


def _compute_pid_from_model(model_type, model_params, rule):
    """Derive PID gains from fitted plant model parameters.
    
    Args:
        model_type: "second_order" or "first_order_delay"
        model_params: Dict with model parameters
        rule: Tuning rule ("zn" or "tyreus")
    
    Returns:
        Dict with PID gains and rule name
    """
    if model_type == "second_order":
        K = model_params.get("K", 1.0)
        zeta = model_params.get("zeta", 0.7)
        omega_n = model_params.get("omega_n", 1.0)

        # Estimate ultimate gain and period from 2nd order model
        # For underdamped 2nd order, approximate Ku and Tu from frequency response
        # At resonance (if underdamped), gain peaks; estimate from model
        if zeta < 1.0:
            # Underdamped: peak at omega_d = omega_n * sqrt(1 - zeta^2)
            omega_d = omega_n * math.sqrt(1.0 - zeta * zeta)
            # Approximate ultimate period from natural frequency
            tu = 2.0 * math.pi / omega_n
            # Approximate ultimate gain (conservative estimate)
            ku = K / (2.0 * zeta) if zeta > 0.1 else K * 5.0
        else:
            # Overdamped: use time constant approximation
            tau = 1.0 / (zeta * omega_n)
            tu = 2.0 * math.pi * tau
            ku = K * 2.0

    elif model_type == "first_order_delay":
        K = model_params.get("K", 1.0)
        tau = model_params.get("tau", 0.5)
        delay_T = model_params.get("delay_T", 0.1)

        # For 1st order + delay, use IMC-like tuning or convert to equivalent Ku/Tu
        # Approximate: Tu ≈ 2π * (tau + delay_T), Ku ≈ K
        tu = 2.0 * math.pi * (tau + delay_T)
        ku = K * 1.5  # Conservative estimate

    else:
        raise ValueError(f"Unsupported model type: {model_type}")

    # Use existing tuning rules
    return _compute_pid_from_rule(ku, tu, rule)


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
        status_callback: Optional[Callable[[str], None]] = None,
        stop_event: Optional[threading.Event] = None,
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
        self._status_callback = status_callback
        self._stop_event = stop_event or threading.Event()
        self._opened_servo = False
        self._axis_vectors = self._compute_axis_vectors()

    def _notify(self, message: str):
        timestamp = datetime.utcnow().strftime("%H:%M:%S")
        payload = f"[AUTOTUNE {timestamp}] {message}"
        print(payload)
        if self._status_callback:
            try:
                self._status_callback(payload)
            except Exception:
                pass

    def connect(self):
        """Connect to servos unless running purely in simulation."""
        if self.simulate:
            self._notify("Simulation mode enabled (no servo connection).")
            return
        self.has_servo = self.controller.connect_servos()
        if not self.has_servo:
            raise RuntimeError("Failed to connect to servos. Use --simulate to skip.")
        self._opened_servo = True
        self._go_to_neutral()
    def _go_to_neutral(self):
        """Command the neutral tilt so hardware starts flat before tuning."""
        try:
            self.controller.send_platform_tilt(0.0, 0.0)
            self._notify("Platform commanded to neutral (roll=0°, pitch=0°).")
        except Exception as exc:
            self._notify(f"Warning: failed to send neutral command ({exc}).")

    def cleanup(self):
        """Return platform to neutral and release hardware resources."""
        try:
            self.controller.send_platform_tilt(0.0, 0.0)
        except Exception:
            pass
        if self._opened_servo and self.controller.servo_serial:
            try:
                self.controller.servo_serial.close()
            except Exception:
                pass
        if self.display:
            cv2.destroyAllWindows()

    def _open_camera(self):
        index = self.camera_cfg.get("index", 0)
        frame_width = self.camera_cfg.get("frame_width", 640)
        frame_height = self.camera_cfg.get("frame_height", 480)
        fps = self.camera_cfg.get("fps", 60)
        backend = self.camera_cfg.get("backend")
        if backend is None and platform.system() == "Windows" and hasattr(cv2, "CAP_DSHOW"):
            backend = cv2.CAP_DSHOW

        if backend is not None:
            cap = cv2.VideoCapture(index, backend)
        else:
            cap = cv2.VideoCapture(index)

        if not cap.isOpened():
            raise RuntimeError(f"Unable to open camera index {index}")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def _compute_axis_vectors(self):
        """Derive roll/pitch unit vectors aligned with motor axes."""
        motor_angles = self.controller.config.get("motor_angles_deg")
        if motor_angles and len(motor_angles) >= 1:
            pitch_deg = motor_angles[0] - 180.0
        else:
            pitch_deg = -90.0  # default setup (motor 0 pointing "down")
        roll_deg = pitch_deg + 90.0

        def to_vec(angle_deg):
            rad = math.radians(angle_deg)
            return (math.cos(rad), math.sin(rad))

        return {
            "pitch": to_vec(pitch_deg),
            "roll": to_vec(roll_deg),
        }

    def request_stop(self):
        """Signal the running autotune loop to exit."""
        self._stop_event.set()

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
        relay_output = self.relay_amplitude
        settle_start = time.time()

        try:
            self._notify("Camera stream ready. Waiting for ball detection...")
            while True:
                if self._stop_event.is_set():
                    raise KeyboardInterrupt("Autotune aborted by user.")

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

                if self.display:
                    vis_frame, _, _, _ = self.detector.draw_detection(frame)

                    # Draw pitch/roll axis overlay aligned with motor frame.
                    axis_origin = (40, vis_frame.shape[0] - 60)
                    axis_scale = 60

                    def axis_endpoint(vec):
                        return (
                            int(axis_origin[0] + axis_scale * vec[0]),
                            int(axis_origin[1] - axis_scale * vec[1]),
                        )

                    roll_vec = self._axis_vectors["roll"]
                    pitch_vec = self._axis_vectors["pitch"]
                    roll_end = axis_endpoint(roll_vec)
                    pitch_end = axis_endpoint(pitch_vec)

                    cv2.arrowedLine(vis_frame, axis_origin, roll_end, (200, 200, 255), 2, tipLength=0.15)
                    cv2.arrowedLine(vis_frame, axis_origin, pitch_end, (200, 200, 255), 2, tipLength=0.15)
                    cv2.putText(
                        vis_frame,
                        "ROLL (X)",
                        (roll_end[0] + 5, roll_end[1] + 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (200, 200, 255),
                        2,
                    )
                    cv2.putText(
                        vis_frame,
                        "PITCH (Y)",
                        (pitch_end[0] - 15, pitch_end[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (200, 200, 255),
                        2,
                    )

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


class FrequencyResponseRunner:
    """Runs a frequency response sweep autotune on a single axis."""

    def __init__(
        self,
        config_path,
        axis,
        freq_min,
        freq_max,
        num_points,
        amplitude_deg,
        cycles_per_freq,
        settle_time,
        display,
        simulate,
        status_callback: Optional[Callable[[str], None]] = None,
        stop_event: Optional[threading.Event] = None,
    ):
        self.config_path = config_path
        self.axis = axis
        self.freq_min = freq_min
        self.freq_max = freq_max
        self.num_points = num_points
        self.amplitude_deg = amplitude_deg
        self.cycles_per_freq = cycles_per_freq
        self.settle_time = settle_time
        self.display = display
        self.simulate = simulate

        self.controller = StewartPlatformController(config_path)
        self.detector = self.controller.detector
        self.camera_cfg = self.controller.config.get("camera", {})
        self.has_servo = False
        self._status_callback = status_callback
        self._stop_event = stop_event or threading.Event()
        self._opened_servo = False
        self._axis_vectors = self._compute_axis_vectors()

    def _notify(self, message: str):
        timestamp = datetime.utcnow().strftime("%H:%M:%S")
        payload = f"[FREQ_RESP {timestamp}] {message}"
        print(payload)
        if self._status_callback:
            try:
                self._status_callback(payload)
            except Exception:
                pass

    def connect(self):
        """Connect to servos unless running purely in simulation."""
        if self.simulate:
            self._notify("Simulation mode enabled (no servo connection).")
            return
        self.has_servo = self.controller.connect_servos()
        if not self.has_servo:
            raise RuntimeError("Failed to connect to servos. Use --simulate to skip.")
        self._opened_servo = True
        self._go_to_neutral()

    def _go_to_neutral(self):
        """Command the neutral tilt so hardware starts flat before tuning."""
        try:
            self.controller.send_platform_tilt(0.0, 0.0)
            self._notify("Platform commanded to neutral (roll=0°, pitch=0°).")
        except Exception as exc:
            self._notify(f"Warning: failed to send neutral command ({exc}).")

    def cleanup(self):
        """Return platform to neutral and release hardware resources."""
        try:
            self.controller.send_platform_tilt(0.0, 0.0)
        except Exception:
            pass
        if self._opened_servo and self.controller.servo_serial:
            try:
                self.controller.servo_serial.close()
            except Exception:
                pass
        if self.display:
            cv2.destroyAllWindows()

    def _open_camera(self):
        index = self.camera_cfg.get("index", 0)
        frame_width = self.camera_cfg.get("frame_width", 640)
        frame_height = self.camera_cfg.get("frame_height", 480)
        fps = self.camera_cfg.get("fps", 60)
        backend = self.camera_cfg.get("backend")
        if backend is None and platform.system() == "Windows" and hasattr(cv2, "CAP_DSHOW"):
            backend = cv2.CAP_DSHOW

        if backend is not None:
            cap = cv2.VideoCapture(index, backend)
        else:
            cap = cv2.VideoCapture(index)

        if not cap.isOpened():
            raise RuntimeError(f"Unable to open camera index {index}")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def _compute_axis_vectors(self):
        """Derive roll/pitch unit vectors aligned with motor axes."""
        motor_angles = self.controller.config.get("motor_angles_deg")
        if motor_angles and len(motor_angles) >= 1:
            pitch_deg = motor_angles[0] - 180.0
        else:
            pitch_deg = -90.0  # default setup (motor 0 pointing "down")
        roll_deg = pitch_deg + 90.0

        def to_vec(angle_deg):
            rad = math.radians(angle_deg)
            return (math.cos(rad), math.sin(rad))

        return {
            "pitch": to_vec(pitch_deg),
            "roll": to_vec(roll_deg),
        }

    def request_stop(self):
        """Signal the running autotune loop to exit."""
        self._stop_event.set()

    def _measure_at_frequency(self, cap, freq):
        """Measure ball response to sinusoidal input at a specific frequency."""
        period = 1.0 / freq
        duration = self.cycles_per_freq * period
        dt_target = 1.0 / 60.0  # Target 60 Hz sampling
        num_samples = int(duration / dt_target)
        if num_samples < 10:
            num_samples = 10

        command_signal = []
        response_signal = []
        time_stamps = []

        start_time = time.time()
        t0 = start_time

        self._notify(f"Measuring at {freq:.3f} Hz ({self.cycles_per_freq} cycles)...")

        while time.time() - start_time < duration + self.settle_time:
            if self._stop_event.is_set():
                raise KeyboardInterrupt("Autotune aborted by user.")

            current_time = time.time()
            elapsed = current_time - t0

            # Generate sinusoidal command
            if elapsed < self.settle_time:
                # Settle period: hold at zero
                cmd_angle = 0.0
            else:
                # Measurement period: sinusoidal excitation
                t_meas = elapsed - self.settle_time
                cmd_angle = self.amplitude_deg * math.sin(2.0 * math.pi * freq * t_meas)

            # Send command to platform
            roll_cmd = cmd_angle if self.axis == "x" else 0.0
            pitch_cmd = cmd_angle if self.axis == "y" else 0.0

            if self.has_servo:
                self.controller.send_platform_tilt(roll_cmd, pitch_cmd)

            # Capture ball position
            ret, frame = cap.read()
            if ret:
                frame = cv2.resize(
                    frame,
                    (
                        self.camera_cfg.get("frame_width", 640),
                        self.camera_cfg.get("frame_height", 480),
                    ),
                )
                found, _, _, pos_x, pos_y = self.detector.detect_ball(frame)

                if found:
                    axis_value = pos_x if self.axis == "x" else pos_y
                    measurement_scaled = axis_value * 100.0  # match PID scaling

                    command_signal.append(cmd_angle)
                    response_signal.append(measurement_scaled)
                    time_stamps.append(current_time)

                    if self.display:
                        vis_frame, _, _, _ = self.detector.draw_detection(frame)
                        cv2.putText(
                            vis_frame,
                            f"Freq: {freq:.3f} Hz | Cmd: {cmd_angle:.2f}°",
                            (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (0, 255, 0),
                            2,
                        )
                        cv2.putText(
                            vis_frame,
                            f"Response: {axis_value:.4f} m",
                            (10, 50),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (0, 255, 255),
                            2,
                        )
                        cv2.imshow("Frequency Response Autotune", vis_frame)
                        if cv2.waitKey(1) & 0xFF == 27:
                            raise KeyboardInterrupt

            time.sleep(dt_target)

        # Extract measurement period only (skip settle)
        settle_samples = int(self.settle_time / dt_target)
        if settle_samples < len(command_signal):
            command_signal = command_signal[settle_samples:]
            response_signal = response_signal[settle_samples:]
            time_stamps = time_stamps[settle_samples:]

        if len(command_signal) < 10:
            raise RuntimeError(f"Insufficient data collected at {freq:.3f} Hz")

        return np.array(command_signal), np.array(response_signal), np.array(time_stamps)

    def _compute_bode_point(self, command_signal, response_signal, freq):
        """Extract gain (dB) and phase (degrees) from command/response signals using FFT."""
        if len(command_signal) < 10 or len(response_signal) < 10:
            return None, None

        # Remove DC offset
        cmd_mean = np.mean(command_signal)
        resp_mean = np.mean(response_signal)
        cmd_centered = command_signal - cmd_mean
        resp_centered = response_signal - resp_mean

        # Use FFT for accurate frequency domain analysis
        if SCIPY_AVAILABLE:
            # Estimate sample rate (assume ~60 Hz from camera)
            dt_est = 1.0 / 60.0
            sample_rate = 1.0 / dt_est
            
            # Compute FFT
            n = len(cmd_centered)
            freqs_fft = np.fft.fftfreq(n, dt_est)
            cmd_fft = np.fft.fft(cmd_centered)
            resp_fft = np.fft.fft(resp_centered)
            
            # Find bin closest to target frequency
            target_idx = np.argmin(np.abs(freqs_fft[:n//2] - freq))
            if target_idx == 0:
                target_idx = 1  # Avoid DC component
            
            # Extract complex values at target frequency
            cmd_complex = cmd_fft[target_idx]
            resp_complex = resp_fft[target_idx]
            
            # Compute transfer function
            if abs(cmd_complex) < 1e-10:
                return None, None
            
            H = resp_complex / cmd_complex
            
            # Extract magnitude and phase
            gain_linear = abs(H)
            phase_rad = math.atan2(H.imag, H.real)
            
            gain_db = 20.0 * math.log10(gain_linear) if gain_linear > 0 else -100.0
            phase_deg = math.degrees(phase_rad)
        else:
            # Fallback: correlation method
            correlation = np.correlate(resp_centered, cmd_centered, mode="full")
            lags = np.arange(-len(cmd_centered) + 1, len(resp_centered))
            max_corr_idx = np.argmax(np.abs(correlation))
            lag_samples = lags[max_corr_idx]

            dt_est = 1.0 / 60.0
            phase_rad = 2.0 * math.pi * freq * lag_samples * dt_est
            phase_rad = math.atan2(math.sin(phase_rad), math.cos(phase_rad))

            cmd_amplitude = np.std(cmd_centered) * math.sqrt(2.0)
            resp_amplitude = np.std(resp_centered) * math.sqrt(2.0)

            if cmd_amplitude < 1e-6:
                return None, None

            gain_linear = resp_amplitude / cmd_amplitude
            gain_db = 20.0 * math.log10(gain_linear) if gain_linear > 0 else -100.0
            phase_deg = math.degrees(phase_rad)

        return gain_db, phase_deg

    def run(self):
        """Execute the frequency response sweep and return Bode data."""
        if not SCIPY_AVAILABLE:
            raise RuntimeError("scipy is required for frequency response autotune. Install with: pip install scipy")

        cap = self._open_camera()
        bode_data = []

        if not self.simulate and not self.has_servo:
            self.connect()

        # Generate log-spaced frequencies
        frequencies = np.logspace(
            math.log10(self.freq_min), math.log10(self.freq_max), self.num_points
        )

        try:
            self._notify(f"Starting frequency sweep: {self.freq_min:.3f} to {self.freq_max:.3f} Hz ({self.num_points} points)")
            self._notify("Camera stream ready. Waiting for ball detection...")

            for idx, freq in enumerate(frequencies):
                if self._stop_event.is_set():
                    raise KeyboardInterrupt("Autotune aborted by user.")

                self._notify(f"Frequency {idx + 1}/{self.num_points}: {freq:.3f} Hz")

                try:
                    cmd_sig, resp_sig, timestamps = self._measure_at_frequency(cap, freq)
                    gain_db, phase_deg = self._compute_bode_point(cmd_sig, resp_sig, freq)

                    if gain_db is not None and phase_deg is not None:
                        bode_data.append({
                            "frequency": float(freq),
                            "gain_db": float(gain_db),
                            "phase_deg": float(phase_deg),
                            "command_signal": cmd_sig.tolist(),
                            "response_signal": resp_sig.tolist(),
                        })
                        self._notify(f"  Gain: {gain_db:.2f} dB, Phase: {phase_deg:.1f}°")
                    else:
                        self._notify(f"  Warning: Could not compute Bode point at {freq:.3f} Hz")
                except Exception as exc:
                    self._notify(f"  Error at {freq:.3f} Hz: {exc}")
                    continue

        finally:
            cap.release()
            self.cleanup()

        if len(bode_data) < 3:
            raise RuntimeError("Insufficient frequency response data collected for model fitting.")

        return {
            "bode_data": bode_data,
            "frequencies": [d["frequency"] for d in bode_data],
            "gains_db": [d["gain_db"] for d in bode_data],
            "phases_deg": [d["phase_deg"] for d in bode_data],
        }


class PRBSAutotuneRunner:
    """Runs PRBS (Pseudo-Random Binary Sequence) autotune with safety clamp."""

    def __init__(
        self,
        config_path,
        axis,
        amplitude,
        prbs_length,
        sample_time,
        clamp_threshold,
        safe_threshold,
        max_duration,
        display,
        simulate,
        status_callback: Optional[Callable[[str], None]] = None,
        stop_event: Optional[threading.Event] = None,
    ):
        self.config_path = config_path
        self.axis = axis
        self.amplitude = amplitude
        self.prbs_length = prbs_length
        self.sample_time = sample_time
        self.clamp_threshold = clamp_threshold  # Stop PRBS when |x| > this
        self.safe_threshold = safe_threshold  # Resume PRBS when |x| < this
        self.max_duration = max_duration
        self.display = display
        self.simulate = simulate

        self.controller = StewartPlatformController(config_path)
        self.detector = self.controller.detector
        self.camera_cfg = self.controller.config.get("camera", {})
        self.has_servo = False
        self._status_callback = status_callback
        self._stop_event = stop_event or threading.Event()
        self._opened_servo = False
        self._axis_vectors = self._compute_axis_vectors()

        # PD controller for recentering (more aggressive to prevent wall hits)
        self.recenter_kp = 5.0  # Increased for faster recentering
        self.recenter_kd = 1.5  # Increased damping
        self.prev_error = 0.0
        self.prev_time = None
        self.prev_position = 0.0  # Track position for velocity estimation

    def _notify(self, message: str):
        timestamp = datetime.utcnow().strftime("%H:%M:%S")
        payload = f"[PRBS {timestamp}] {message}"
        print(payload)
        if self._status_callback:
            try:
                self._status_callback(payload)
            except Exception:
                pass

    def connect(self):
        """Connect to servos unless running purely in simulation."""
        if self.simulate:
            self._notify("Simulation mode enabled (no servo connection).")
            return
        self.has_servo = self.controller.connect_servos()
        if not self.has_servo:
            raise RuntimeError("Failed to connect to servos. Use --simulate to skip.")
        self._opened_servo = True
        self._go_to_neutral()

    def _go_to_neutral(self):
        """Command the neutral tilt so hardware starts flat before tuning."""
        try:
            self.controller.send_platform_tilt(0.0, 0.0)
            self._notify("Platform commanded to neutral (roll=0°, pitch=0°).")
        except Exception as exc:
            self._notify(f"Warning: failed to send neutral command ({exc}).")

    def cleanup(self):
        """Return platform to neutral and release hardware resources."""
        try:
            self.controller.send_platform_tilt(0.0, 0.0)
        except Exception:
            pass
        if self._opened_servo and self.controller.servo_serial:
            try:
                self.controller.servo_serial.close()
            except Exception:
                pass
        if self.display:
            cv2.destroyAllWindows()

    def _open_camera(self):
        index = self.camera_cfg.get("index", 0)
        frame_width = self.camera_cfg.get("frame_width", 640)
        frame_height = self.camera_cfg.get("frame_height", 480)
        fps = self.camera_cfg.get("fps", 60)
        backend = self.camera_cfg.get("backend")
        if backend is None and platform.system() == "Windows" and hasattr(cv2, "CAP_DSHOW"):
            backend = cv2.CAP_DSHOW

        if backend is not None:
            cap = cv2.VideoCapture(index, backend)
        else:
            cap = cv2.VideoCapture(index)

        if not cap.isOpened():
            raise RuntimeError(f"Unable to open camera index {index}")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def _compute_axis_vectors(self):
        """Derive roll/pitch unit vectors aligned with motor axes."""
        motor_angles = self.controller.config.get("motor_angles_deg")
        if motor_angles and len(motor_angles) >= 1:
            pitch_deg = motor_angles[0] - 180.0
        else:
            pitch_deg = -90.0
        roll_deg = pitch_deg + 90.0

        def to_vec(angle_deg):
            rad = math.radians(angle_deg)
            return (math.cos(rad), math.sin(rad))

        return {
            "pitch": to_vec(pitch_deg),
            "roll": to_vec(roll_deg),
        }

    def request_stop(self):
        """Signal the running autotune loop to exit."""
        self._stop_event.set()

    def _generate_prbs(self, length, seed=None):
        """Generate PRBS sequence using linear feedback shift register."""
        if seed is None:
            seed = int(time.time() * 1000) % (2**15 - 1)
        
        # Maximum length PRBS: 2^n - 1
        # Using n=15 for length up to 32767
        prbs = []
        register = seed & 0x7FFF  # 15-bit
        
        for _ in range(length):
            # LFSR: taps at positions 15 and 14 (standard)
            bit = ((register >> 14) ^ (register >> 13)) & 1
            register = ((register << 1) | bit) & 0x7FFF
            prbs.append(1 if bit else -1)
        
        return prbs

    def _recenter_pd(self, error, dt):
        """PD controller to recenter the ball (more aggressive to prevent wall hits)."""
        if self.prev_time is None:
            self.prev_time = time.time()
            derivative = 0.0
        else:
            derivative = (error - self.prev_error) / dt if dt > 0.001 else 0.0
        
        # More aggressive recentering - allow larger output to bring ball back quickly
        output = self.recenter_kp * error + self.recenter_kd * derivative
        # Allow up to 3x amplitude for recentering (need to overcome ball momentum)
        max_recenter = max(self.amplitude * 3.0, 1.0)  # At least 1 degree
        output = np.clip(output, -max_recenter, max_recenter)
        
        self.prev_error = error
        self.prev_time = time.time()
        
        return output

    def run(self):
        """Execute PRBS excitation with safety clamp and return input/output data."""
        cap = self._open_camera()
        
        if not self.simulate and not self.has_servo:
            self.connect()

        # Generate PRBS sequence
        prbs_sequence = self._generate_prbs(self.prbs_length)
        prbs_index = 0

        input_log = []
        output_log = []
        time_log = []
        prbs_active = []  # Boolean: True when PRBS was active (not clamped)

        start_time = time.time()
        last_sample_time = start_time
        prbs_active_flag = True
        clamp_count = 0

        try:
            self._notify(f"Starting PRBS excitation: amplitude={self.amplitude:.3f}°, clamp at ±{self.clamp_threshold*1000:.1f}mm")
            self._notify("Camera stream ready. Waiting for ball detection...")

            while time.time() - start_time < self.max_duration:
                if self._stop_event.is_set():
                    raise KeyboardInterrupt("Autotune aborted by user.")

                current_time = time.time()
                dt = current_time - last_sample_time

                if dt < self.sample_time:
                    # Use smaller sleep for more responsive safety checking
                    time.sleep(0.005)  # Check every 5ms even if not sampling
                    # Still check safety even if not time to sample yet
                    ret, frame = cap.read()
                    if ret:
                        frame = cv2.resize(
                            frame,
                            (
                                self.camera_cfg.get("frame_width", 640),
                                self.camera_cfg.get("frame_height", 480),
                            ),
                        )
                        found, _, _, pos_x, pos_y = self.detector.detect_ball(frame)
                        if found:
                            axis_value_check = pos_x if self.axis == "x" else pos_y
                            # Emergency clamp if very close to wall
                            if abs(axis_value_check) > self.clamp_threshold * 1.2:
                                prbs_active_flag = False
                                error_emergency = -axis_value_check * 100.0
                                cmd_emergency = self._recenter_pd(error_emergency, 0.01)
                                roll_cmd = cmd_emergency if self.axis == "x" else 0.0
                                pitch_cmd = cmd_emergency if self.axis == "y" else 0.0
                                if self.has_servo:
                                    self.controller.send_platform_tilt(roll_cmd, pitch_cmd)
                    continue

                last_sample_time = current_time

                # Capture ball position
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
                        cv2.imshow("PRBS Autotune (no ball)", frame)
                        if cv2.waitKey(1) & 0xFF == 27:
                            raise KeyboardInterrupt
                    continue

                axis_value = pos_x if self.axis == "x" else pos_y
                axis_value_scaled = axis_value * 100.0  # match PID scaling

                # Estimate velocity for predictive clamping
                if self.prev_time is not None and dt > 0:
                    velocity = (axis_value - self.prev_position) / dt
                    # Predict position in next sample
                    predicted_pos = axis_value + velocity * dt
                else:
                    velocity = 0.0
                    predicted_pos = axis_value
                self.prev_position = axis_value

                # Safety clamp logic - use predicted position for earlier clamping
                # Also check if moving fast toward wall
                moving_toward_wall = (axis_value > 0 and velocity > 0) or (axis_value < 0 and velocity < 0)
                effective_threshold = self.clamp_threshold * 0.8 if moving_toward_wall else self.clamp_threshold
                
                if abs(axis_value) > self.clamp_threshold or abs(predicted_pos) > self.clamp_threshold:
                    # Ball too close to wall - stop PRBS and recenter
                    if prbs_active_flag:
                        prbs_active_flag = False
                        clamp_count += 1
                        self._notify(f"Clamp activated: |x|={abs(axis_value)*1000:.1f}mm > {self.clamp_threshold*1000:.1f}mm")
                    
                    # Use PD controller to recenter
                    error = -axis_value_scaled
                    cmd_angle = self._recenter_pd(error, dt)
                    prbs_active.append(False)
                elif abs(axis_value) < self.safe_threshold:
                    # Ball back in safe zone - resume PRBS
                    if not prbs_active_flag:
                        prbs_active_flag = True
                        self._notify(f"Resuming PRBS: |x|={abs(axis_value)*1000:.1f}mm < {self.safe_threshold*1000:.1f}mm")
                        self.prev_error = 0.0  # Reset PD state
                    
                    # Generate PRBS command
                    if prbs_index < len(prbs_sequence):
                        cmd_angle = self.amplitude * prbs_sequence[prbs_index]
                        prbs_index += 1
                    else:
                        # Wrap around PRBS
                        prbs_index = 0
                        cmd_angle = self.amplitude * prbs_sequence[prbs_index]
                        prbs_index += 1
                    prbs_active.append(True)
                else:
                    # In transition zone - continue current mode
                    if prbs_active_flag:
                        if prbs_index < len(prbs_sequence):
                            cmd_angle = self.amplitude * prbs_sequence[prbs_index]
                            prbs_index += 1
                        else:
                            prbs_index = 0
                            cmd_angle = self.amplitude * prbs_sequence[prbs_index]
                            prbs_index += 1
                        prbs_active.append(True)
                    else:
                        error = -axis_value_scaled
                        cmd_angle = self._recenter_pd(error, dt)
                        prbs_active.append(False)

                # Send command to platform
                roll_cmd = cmd_angle if self.axis == "x" else 0.0
                pitch_cmd = cmd_angle if self.axis == "y" else 0.0

                if self.has_servo:
                    self.controller.send_platform_tilt(roll_cmd, pitch_cmd)

                # Log data
                input_log.append(cmd_angle)
                output_log.append(axis_value_scaled)
                time_log.append(current_time)

                if self.display:
                    vis_frame, _, _, _ = self.detector.draw_detection(frame)
                    mode_str = "PRBS" if prbs_active_flag else "RECENTER"
                    color = (0, 255, 0) if prbs_active_flag else (0, 0, 255)
                    cv2.putText(
                        vis_frame,
                        f"Mode: {mode_str} | x={axis_value*1000:.1f}mm | Clamps: {clamp_count}",
                        (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        color,
                        2,
                    )
                    cv2.putText(
                        vis_frame,
                        f"Cmd: {cmd_angle:.3f}° | Active: {sum(prbs_active)}/{len(prbs_active)}",
                        (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 255),
                        2,
                    )
                    cv2.imshow("PRBS Autotune", vis_frame)
                    if cv2.waitKey(1) & 0xFF == 27:
                        raise KeyboardInterrupt

        finally:
            cap.release()
            self.cleanup()

        if len(input_log) < 100:
            raise RuntimeError("Insufficient data collected for PRBS identification.")

        active_samples = sum(prbs_active)
        if active_samples < 50:
            raise RuntimeError(f"Too few PRBS-active samples ({active_samples}). Ball may have been clamped too often.")

        self._notify(f"Collected {len(input_log)} samples, {active_samples} PRBS-active ({100*active_samples/len(input_log):.1f}%)")

        return {
            "input_log": input_log,
            "output_log": output_log,
            "time_log": time_log,
            "prbs_active": prbs_active,
            "clamp_count": clamp_count,
        }


def _compute_impulse_response_prbs(input_signal, output_signal, prbs_active, max_lag=None):
    """Compute impulse response using correlation of PRBS input/output.
    
    Only uses samples where prbs_active is True.
    """
    # Extract only PRBS-active samples
    input_active = np.array([inp for inp, active in zip(input_signal, prbs_active) if active])
    output_active = np.array([out for out, active in zip(output_signal, prbs_active) if active])
    
    if len(input_active) < 50:
        raise RuntimeError("Too few PRBS-active samples for correlation.")
    
    if max_lag is None:
        max_lag = min(len(input_active) // 4, 200)
    
    # Normalize
    input_active = input_active - np.mean(input_active)
    output_active = output_active - np.mean(output_active)
    
    # Cross-correlation
    correlation = np.correlate(output_active, input_active, mode="full")
    lags = np.arange(-len(input_active) + 1, len(output_active))
    
    # Extract positive lags (causal system)
    positive_lags = lags >= 0
    correlation = correlation[positive_lags]
    lags = lags[positive_lags]
    
    # Limit to max_lag
    if len(correlation) > max_lag:
        correlation = correlation[:max_lag]
        lags = lags[:max_lag]
    
    # Normalize by input variance to get impulse response
    input_var = np.var(input_active)
    if input_var > 1e-10:
        impulse_response = correlation / input_var
    else:
        impulse_response = correlation
    
    return impulse_response, lags


def _fit_transfer_function_from_impulse(impulse_response, lags, sample_time, model_type="second_order"):
    """Fit transfer function from impulse response using least squares."""
    if not SCIPY_AVAILABLE:
        raise RuntimeError("scipy is required for transfer function fitting")
    
    time_vector = lags * sample_time
    
    if model_type == "second_order":
        # Fit G(s) = K / (s² + 2ζωₙs + ωₙ²)
        # Convert to time domain: h(t) = K*ωₙ²/(ω_d) * exp(-ζωₙt) * sin(ω_d t) for underdamped
        
        def model_impulse(t, K, zeta, omega_n):
            if zeta < 1.0:
                omega_d = omega_n * math.sqrt(1.0 - zeta * zeta)
                if omega_d < 1e-6:
                    return np.zeros_like(t)
                h = (K * omega_n * omega_n / omega_d) * np.exp(-zeta * omega_n * t) * np.sin(omega_d * t)
            else:
                # Overdamped case
                h = K * omega_n * omega_n * t * np.exp(-omega_n * t)
            return h
        
        # Initial guess
        peak_idx = np.argmax(np.abs(impulse_response))
        if peak_idx > 0:
            omega_n_init = 2.0 * math.pi / (time_vector[peak_idx] * 2.0) if time_vector[peak_idx] > 0 else 1.0
        else:
            omega_n_init = 1.0
        K_init = np.max(np.abs(impulse_response))
        zeta_init = 0.7
        
        try:
            popt, _ = optimize.curve_fit(
                model_impulse,
                time_vector,
                impulse_response,
                p0=[K_init, zeta_init, omega_n_init],
                bounds=([0.01, 0.01, 0.1], [100.0, 2.0, 100.0]),
                maxfev=5000,
            )
            K, zeta, omega_n = popt
            return {"K": float(K), "zeta": float(zeta), "omega_n": float(omega_n)}
        except Exception:
            # Fallback
            return {"K": float(K_init), "zeta": float(zeta_init), "omega_n": float(omega_n_init)}
    
    elif model_type == "first_order_delay":
        # Fit G(s) = K·e^(-Ts) / (τs + 1)
        # Impulse response: h(t) = (K/τ) * exp(-(t-T)/τ) for t >= T, else 0
        
        def model_impulse(t, K, tau, delay_T):
            h = np.zeros_like(t)
            mask = t >= delay_T
            if np.any(mask):
                h[mask] = (K / tau) * np.exp(-(t[mask] - delay_T) / tau)
            return h
        
        # Initial guess
        peak_idx = np.argmax(np.abs(impulse_response))
        delay_T_init = time_vector[peak_idx] if peak_idx > 0 else 0.1
        tau_init = time_vector[-1] / 3.0 if len(time_vector) > 0 else 0.5
        K_init = np.max(np.abs(impulse_response)) * tau_init
        
        try:
            popt, _ = optimize.curve_fit(
                model_impulse,
                time_vector,
                impulse_response,
                p0=[K_init, tau_init, delay_T_init],
                bounds=([0.01, 0.01, 0.0], [100.0, 10.0, 1.0]),
                maxfev=5000,
            )
            K, tau, delay_T = popt
            return {"K": float(K), "tau": float(tau), "delay_T": float(delay_T)}
        except Exception:
            return {"K": float(K_init), "tau": float(tau_init), "delay_T": float(delay_T_init)}
    
    else:
        raise ValueError(f"Unsupported model type: {model_type}")


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


def _fit_second_order_model(frequencies, gains_db, phases_deg):
    """Fit a 2nd order model: G(s) = K / (s² + 2ζωₙs + ωₙ²)
    
    Returns: {K, zeta, omega_n}
    """
    if not SCIPY_AVAILABLE:
        raise RuntimeError("scipy is required for model fitting")

    # Convert to linear gain
    gains_linear = [10.0 ** (g_db / 20.0) if g_db > -100 else 1e-5 for g_db in gains_db]
    phases_rad = [math.radians(p) for p in phases_deg]
    freqs_rad = [2.0 * math.pi * f for f in frequencies]

    def model_response(freq_rad, K, zeta, omega_n):
        """Compute magnitude and phase of 2nd order system at frequency."""
        s = 1j * freq_rad
        denom = s * s + 2 * zeta * omega_n * s + omega_n * omega_n
        if abs(denom) < 1e-10:
            return 1e-5, -math.pi
        H = K / denom
        mag = abs(H)
        phase = math.atan2(H.imag, H.real)
        return mag, phase

    def objective(params, freqs, gains, phases):
        """Objective function for optimization."""
        K, zeta, omega_n = params
        error = 0.0
        for f, g_target, p_target in zip(freqs, gains, phases):
            mag, phase = model_response(f, K, zeta, omega_n)
            # Weight magnitude and phase errors
            mag_error = (math.log10(mag) - math.log10(g_target)) ** 2
            phase_error = (phase - p_target) ** 2
            error += mag_error + 0.1 * phase_error  # Phase less weighted
        return error

    # Initial guess: estimate from data
    K_init = max(gains_linear)
    omega_n_init = 2.0 * math.pi * frequencies[np.argmax(gains_linear)] if len(frequencies) > 0 else 1.0
    zeta_init = 0.7  # Typical damping

    try:
        result = optimize.minimize(
            objective,
            [K_init, zeta_init, omega_n_init],
            args=(freqs_rad, gains_linear, phases_rad),
            method="Nelder-Mead",
            bounds=[(0.01, 100.0), (0.01, 2.0), (0.1, 100.0)],
        )
        K, zeta, omega_n = result.x
        if K < 0.01 or zeta < 0.01 or omega_n < 0.1:
            raise ValueError("Invalid fitted parameters")
        return {"K": float(K), "zeta": float(zeta), "omega_n": float(omega_n)}
    except Exception as exc:
        # Fallback: simple estimation
        K = max(gains_linear)
        omega_n = 2.0 * math.pi * frequencies[len(frequencies) // 2] if frequencies else 1.0
        zeta = 0.7
        return {"K": float(K), "zeta": float(zeta), "omega_n": float(omega_n)}


def _fit_first_order_delay_model(frequencies, gains_db, phases_deg):
    """Fit a 1st order + delay model: G(s) = K·e^(-Ts) / (τs + 1)
    
    Returns: {K, tau, delay_T}
    """
    if not SCIPY_AVAILABLE:
        raise RuntimeError("scipy is required for model fitting")

    # Convert to linear gain
    gains_linear = [10.0 ** (g_db / 20.0) if g_db > -100 else 1e-5 for g_db in gains_db]
    phases_rad = [math.radians(p) for p in phases_deg]
    freqs_rad = [2.0 * math.pi * f for f in frequencies]

    def model_response(freq_rad, K, tau, delay_T):
        """Compute magnitude and phase of 1st order + delay at frequency."""
        s = 1j * freq_rad
        H = K * np.exp(-delay_T * s) / (tau * s + 1.0)
        mag = abs(H)
        phase = math.atan2(H.imag, H.real)
        return mag, phase

    def objective(params, freqs, gains, phases):
        """Objective function for optimization."""
        K, tau, delay_T = params
        error = 0.0
        for f, g_target, p_target in zip(freqs, gains, phases):
            mag, phase = model_response(f, K, tau, delay_T)
            mag_error = (math.log10(mag) - math.log10(g_target)) ** 2
            phase_error = (phase - p_target) ** 2
            error += mag_error + 0.1 * phase_error
        return error

    # Initial guess
    K_init = max(gains_linear)
    tau_init = 1.0 / (2.0 * math.pi * frequencies[len(frequencies) // 2]) if frequencies else 0.5
    delay_T_init = 0.1

    try:
        result = optimize.minimize(
            objective,
            [K_init, tau_init, delay_T_init],
            args=(freqs_rad, gains_linear, phases_rad),
            method="Nelder-Mead",
            bounds=[(0.01, 100.0), (0.01, 10.0), (0.0, 1.0)],
        )
        K, tau, delay_T = result.x
        if K < 0.01 or tau < 0.01:
            raise ValueError("Invalid fitted parameters")
        return {"K": float(K), "tau": float(tau), "delay_T": float(delay_T)}
    except Exception as exc:
        # Fallback
        K = max(gains_linear)
        tau = 0.5
        delay_T = 0.1
        return {"K": float(K), "tau": float(tau), "delay_T": float(delay_T)}


def autotune_once(
    *,
    config_path: str,
    axis: str,
    method: str = "relay",
    amplitude: float = None,  # Will be set method-specific defaults
    deadband: float = 0.002,
    cycles: int = 4,
    max_duration: float = 120.0,
    rule: str = "zn",
    results_file: Optional[str] = None,
    display: bool = False,
    simulate: bool = False,
    apply_config: bool = False,
    status_callback: Optional[Callable[[str], None]] = None,
    stop_event: Optional[threading.Event] = None,
    # Frequency response parameters
    freq_min: float = 0.1,
    freq_max: float = 5.0,
    num_points: int = 12,
    cycles_per_freq: int = 5,
    settle_time: float = 2.0,
    model_type: str = "second_order",
    plot_bode: bool = False,
    # PRBS parameters
        prbs_length: int = 1023,
        sample_time: float = 0.03,  # Faster sampling (33 Hz) for better safety
        clamp_threshold: float = 0.015,  # More conservative (15mm instead of 25mm)
        safe_threshold: float = 0.005,  # Must recenter closer (5mm) before resuming
):
    """Run autotune using specified method (relay, freq_response, or prbs) and optionally write results."""
    # Set method-specific amplitude defaults if not provided
    if amplitude is None:
        if method == "prbs":
            amplitude = 0.06  # Very small for PRBS (0.06° = ~1mm ball movement)
        elif method == "freq_response":
            amplitude = 1.5  # Moderate for frequency sweep
        else:  # relay
            amplitude = 4.0  # Larger for relay test
    
    if method == "relay":
        runner = RelayAutotuneRunner(
            config_path=config_path,
            axis=axis,
            relay_amplitude_deg=amplitude,
            deadband_m=deadband,
            cycles=cycles,
            max_duration=max_duration,
            display=display,
            simulate=simulate,
            status_callback=status_callback,
            stop_event=stop_event,
        )

        raw = runner.run()
        summary = _summarize_results(raw, amplitude)
        pid = _compute_pid_from_rule(summary["Ku"], summary["Tu"], rule)

        entry = {
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "method": "relay",
            "axis": axis,
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

    elif method == "freq_response":
        runner = FrequencyResponseRunner(
            config_path=config_path,
            axis=axis,
            freq_min=freq_min,
            freq_max=freq_max,
            num_points=num_points,
            amplitude_deg=amplitude,
            cycles_per_freq=cycles_per_freq,
            settle_time=settle_time,
            display=display,
            simulate=simulate,
            status_callback=status_callback,
            stop_event=stop_event,
        )

        raw = runner.run()
        frequencies = raw["frequencies"]
        gains_db = raw["gains_db"]
        phases_deg = raw["phases_deg"]

        # Fit model
        if model_type == "second_order":
            model_params = _fit_second_order_model(frequencies, gains_db, phases_deg)
        elif model_type == "first_order_delay":
            model_params = _fit_first_order_delay_model(frequencies, gains_db, phases_deg)
        else:
            raise ValueError(f"Unsupported model type: {model_type}")

        # Derive PID from model
        pid = _compute_pid_from_model(model_type, model_params, rule)

        entry = {
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "method": "freq_response",
            "axis": axis,
            "rule": pid["rule"],
            "model_type": model_type,
            "model_params": model_params,
            "frequencies": frequencies,
            "gains_db": gains_db,
            "phases_deg": phases_deg,
            "Kp": pid["Kp"],
            "Ki": pid["Ki"],
            "Kd": pid["Kd"],
        }

        # Generate Bode plot if requested
        if plot_bode and MATPLOTLIB_AVAILABLE:
            _plot_bode(frequencies, gains_db, phases_deg, model_type, model_params)

        summary = {
            "frequencies": frequencies,
            "gains_db": gains_db,
            "phases_deg": phases_deg,
            "model_type": model_type,
            "model_params": model_params,
        }

    elif method == "prbs":
        # PRBS method uses longer default duration
        max_duration_prbs = max_duration if max_duration > 60 else 300.0  # 5 minutes default

        runner = PRBSAutotuneRunner(
            config_path=config_path,
            axis=axis,
            amplitude=amplitude,
            prbs_length=prbs_length,
            sample_time=sample_time,
            clamp_threshold=clamp_threshold,
            safe_threshold=safe_threshold,
            max_duration=max_duration_prbs,
            display=display,
            simulate=simulate,
            status_callback=status_callback,
            stop_event=stop_event,
        )

        raw = runner.run()
        input_log = raw["input_log"]
        output_log = raw["output_log"]
        time_log = raw["time_log"]
        prbs_active = raw["prbs_active"]
        clamp_count = raw["clamp_count"]

        # Compute sample time from data
        if len(time_log) > 1:
            dt_actual = np.mean(np.diff(time_log))
        else:
            dt_actual = sample_time

        # Compute impulse response via correlation
        impulse_response, lags = _compute_impulse_response_prbs(
            input_log, output_log, prbs_active
        )

        # Fit transfer function from impulse response
        model_params = _fit_transfer_function_from_impulse(
            impulse_response, lags, dt_actual, model_type
        )

        # Derive PID from model
        pid = _compute_pid_from_model(model_type, model_params, rule)

        entry = {
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "method": "prbs",
            "axis": axis,
            "rule": pid["rule"],
            "model_type": model_type,
            "model_params": model_params,
            "amplitude": amplitude,
            "prbs_length": prbs_length,
            "sample_time": dt_actual,
            "clamp_count": clamp_count,
            "prbs_active_samples": sum(prbs_active),
            "total_samples": len(input_log),
            "Kp": pid["Kp"],
            "Ki": pid["Ki"],
            "Kd": pid["Kd"],
        }

        summary = {
            "model_type": model_type,
            "model_params": model_params,
            "clamp_count": clamp_count,
            "prbs_active_ratio": sum(prbs_active) / len(input_log) if input_log else 0.0,
            "impulse_response": impulse_response.tolist()[:50],  # Store first 50 points
        }

    else:
        raise ValueError(f"Unsupported method: {method}")

    if results_file:
        _write_results(results_file, entry)

    backup = None
    if apply_config:
        backup = _apply_to_config(config_path, pid)

    return entry, summary, pid, backup


def _plot_bode(frequencies, gains_db, phases_deg, model_type, model_params):
    """Generate Bode plot with measured data and fitted model overlay."""
    if not MATPLOTLIB_AVAILABLE:
        return

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

    # Plot measured data
    ax1.semilogx(frequencies, gains_db, "bo-", label="Measured", linewidth=2, markersize=6)
    ax1.set_ylabel("Magnitude (dB)")
    ax1.set_title("Bode Plot - Magnitude")
    ax1.grid(True, alpha=0.3)
    ax1.legend()

    ax2.semilogx(frequencies, phases_deg, "bo-", label="Measured", linewidth=2, markersize=6)
    ax2.set_xlabel("Frequency (Hz)")
    ax2.set_ylabel("Phase (degrees)")
    ax2.set_title("Bode Plot - Phase")
    ax2.grid(True, alpha=0.3)
    ax2.legend()

    # Overlay fitted model
    freq_model = np.logspace(math.log10(min(frequencies)), math.log10(max(frequencies)), 200)
    gains_model_db = []
    phases_model_deg = []

    for f in freq_model:
        freq_rad = 2.0 * math.pi * f
        if model_type == "second_order":
            K = model_params["K"]
            zeta = model_params["zeta"]
            omega_n = model_params["omega_n"]
            s = 1j * freq_rad
            denom = s * s + 2 * zeta * omega_n * s + omega_n * omega_n
            H = K / denom
        elif model_type == "first_order_delay":
            K = model_params["K"]
            tau = model_params["tau"]
            delay_T = model_params["delay_T"]
            s = 1j * freq_rad
            H = K * np.exp(-delay_T * s) / (tau * s + 1.0)

        mag_db = 20.0 * math.log10(abs(H)) if abs(H) > 0 else -100.0
        phase_deg = math.degrees(math.atan2(H.imag, H.real))
        gains_model_db.append(mag_db)
        phases_model_deg.append(phase_deg)

    ax1.semilogx(freq_model, gains_model_db, "r--", label=f"Fitted {model_type}", linewidth=2)
    ax2.semilogx(freq_model, phases_model_deg, "r--", label=f"Fitted {model_type}", linewidth=2)
    ax1.legend()
    ax2.legend()

    plt.tight_layout()
    plt.show()


class AutotuneGUI:
    """Simple Tkinter GUI to run autotune interactively."""

    def __init__(self, default_config="config_stewart.json"):
        if tk is None or ttk is None:
            raise RuntimeError("Tkinter is unavailable on this system.")

        self.default_config = default_config
        self.root = tk.Tk()
        self.root.title("Stewart Platform PID Autotune")
        self.root.geometry("700x900")

        self.log_queue = Queue()
        self.worker_thread: Optional[threading.Thread] = None
        self.stop_event: Optional[threading.Event] = None
        self.last_pid = None
        self.last_summary = None
        self.last_entry = None
        self.last_backup = None

        self._build_ui()
        self._poll_log_queue()

    def _build_ui(self):
        form = ttk.LabelFrame(self.root, text="Autotune Parameters", padding=10)
        form.pack(fill=tk.X, padx=12, pady=8)

        self.config_var = tk.StringVar(value=self.default_config)
        self.method_var = tk.StringVar(value="relay")
        self.axis_var = tk.StringVar(value="x")
        self.rule_var = tk.StringVar(value="zn")
        self.amplitude_var = tk.DoubleVar(value=4.0)  # Default for relay, will adjust based on method
        self.deadband_var = tk.DoubleVar(value=0.002)
        self.cycles_var = tk.IntVar(value=4)
        self.max_duration_var = tk.DoubleVar(value=120.0)
        self.results_var = tk.StringVar(value="pid_autotune_results.json")
        self.display_var = tk.BooleanVar(value=False)
        self.simulate_var = tk.BooleanVar(value=False)
        self.apply_var = tk.BooleanVar(value=True)
        # Frequency response parameters
        self.freq_min_var = tk.DoubleVar(value=0.1)
        self.freq_max_var = tk.DoubleVar(value=5.0)
        self.num_points_var = tk.IntVar(value=12)
        self.cycles_per_freq_var = tk.IntVar(value=5)
        self.settle_time_var = tk.DoubleVar(value=2.0)
        self.model_type_var = tk.StringVar(value="second_order")
        self.plot_bode_var = tk.BooleanVar(value=False)

        def add_row(row, label, widget):
            ttk.Label(form, text=label, width=18, anchor="w").grid(row=row, column=0, sticky="w", pady=3)
            widget.grid(row=row, column=1, sticky="ew", pady=3)

        form.columnconfigure(1, weight=1)

        add_row(0, "Config Path", ttk.Entry(form, textvariable=self.config_var))
        method_combo = ttk.Combobox(form, textvariable=self.method_var, values=("relay", "freq_response", "prbs"), state="readonly", width=12)
        method_combo.bind("<<ComboboxSelected>>", lambda e: self._update_method_ui())
        add_row(1, "Method", method_combo)
        axis_combo = ttk.Combobox(form, textvariable=self.axis_var, values=("x", "y"), state="readonly", width=5)
        add_row(2, "Axis", axis_combo)
        
        # Relay-specific parameters
        self.relay_frame = ttk.LabelFrame(form, text="Relay Parameters", padding=5)
        self.relay_frame.grid(row=3, column=0, columnspan=2, sticky="ew", pady=5)
        ttk.Label(self.relay_frame, text="Amplitude (°)").grid(row=0, column=0, sticky="w", padx=5)
        ttk.Entry(self.relay_frame, textvariable=self.amplitude_var, width=10).grid(row=0, column=1, padx=5)
        ttk.Label(self.relay_frame, text="Deadband (m)").grid(row=0, column=2, sticky="w", padx=5)
        ttk.Entry(self.relay_frame, textvariable=self.deadband_var, width=10).grid(row=0, column=3, padx=5)
        ttk.Label(self.relay_frame, text="Cycles").grid(row=1, column=0, sticky="w", padx=5)
        ttk.Entry(self.relay_frame, textvariable=self.cycles_var, width=10).grid(row=1, column=1, padx=5)
        ttk.Label(self.relay_frame, text="Timeout (s)").grid(row=1, column=2, sticky="w", padx=5)
        ttk.Entry(self.relay_frame, textvariable=self.max_duration_var, width=10).grid(row=1, column=3, padx=5)
        
        # Frequency response parameters
        self.freq_frame = ttk.LabelFrame(form, text="Frequency Response Parameters", padding=5)
        self.freq_frame.grid(row=4, column=0, columnspan=2, sticky="ew", pady=5)
        ttk.Label(self.freq_frame, text="Freq Min (Hz)").grid(row=0, column=0, sticky="w", padx=5)
        ttk.Entry(self.freq_frame, textvariable=self.freq_min_var, width=10).grid(row=0, column=1, padx=5)
        ttk.Label(self.freq_frame, text="Freq Max (Hz)").grid(row=0, column=2, sticky="w", padx=5)
        ttk.Entry(self.freq_frame, textvariable=self.freq_max_var, width=10).grid(row=0, column=3, padx=5)
        ttk.Label(self.freq_frame, text="Num Points").grid(row=1, column=0, sticky="w", padx=5)
        ttk.Entry(self.freq_frame, textvariable=self.num_points_var, width=10).grid(row=1, column=1, padx=5)
        ttk.Label(self.freq_frame, text="Cycles/Freq").grid(row=1, column=2, sticky="w", padx=5)
        ttk.Entry(self.freq_frame, textvariable=self.cycles_per_freq_var, width=10).grid(row=1, column=3, padx=5)
        ttk.Label(self.freq_frame, text="Settle Time (s)").grid(row=2, column=0, sticky="w", padx=5)
        ttk.Entry(self.freq_frame, textvariable=self.settle_time_var, width=10).grid(row=2, column=1, padx=5)
        ttk.Label(self.freq_frame, text="Model Type").grid(row=2, column=2, sticky="w", padx=5)
        model_combo = ttk.Combobox(self.freq_frame, textvariable=self.model_type_var, values=("second_order", "first_order_delay"), state="readonly", width=15)
        model_combo.grid(row=2, column=3, padx=5)
        
        # PRBS parameters
        self.prbs_frame = ttk.LabelFrame(form, text="PRBS Parameters", padding=5)
        self.prbs_frame.grid(row=5, column=0, columnspan=2, sticky="ew", pady=5)
        self.prbs_length_var = tk.IntVar(value=1023)
        self.sample_time_var = tk.DoubleVar(value=0.03)
        self.clamp_threshold_var = tk.DoubleVar(value=0.015)
        self.safe_threshold_var = tk.DoubleVar(value=0.005)
        ttk.Label(self.prbs_frame, text="PRBS Length").grid(row=0, column=0, sticky="w", padx=5)
        ttk.Entry(self.prbs_frame, textvariable=self.prbs_length_var, width=10).grid(row=0, column=1, padx=5)
        ttk.Label(self.prbs_frame, text="Sample Time (s)").grid(row=0, column=2, sticky="w", padx=5)
        ttk.Entry(self.prbs_frame, textvariable=self.sample_time_var, width=10).grid(row=0, column=3, padx=5)
        ttk.Label(self.prbs_frame, text="Clamp @ (m)").grid(row=1, column=0, sticky="w", padx=5)
        ttk.Entry(self.prbs_frame, textvariable=self.clamp_threshold_var, width=10).grid(row=1, column=1, padx=5)
        ttk.Label(self.prbs_frame, text="Safe @ (m)").grid(row=1, column=2, sticky="w", padx=5)
        ttk.Entry(self.prbs_frame, textvariable=self.safe_threshold_var, width=10).grid(row=1, column=3, padx=5)
        
        rule_combo = ttk.Combobox(form, textvariable=self.rule_var, values=("zn", "tyreus"), state="readonly")
        add_row(6, "Tuning Rule", rule_combo)
        add_row(7, "Results File", ttk.Entry(form, textvariable=self.results_var))

        options_frame = ttk.Frame(form)
        options_frame.grid(row=8, column=0, columnspan=2, sticky="w", pady=(8, 0))
        ttk.Checkbutton(options_frame, text="Display camera preview", variable=self.display_var).pack(anchor="w")
        ttk.Checkbutton(options_frame, text="Simulation (no servos)", variable=self.simulate_var).pack(anchor="w")
        ttk.Checkbutton(options_frame, text="Apply gains to config", variable=self.apply_var).pack(anchor="w")
        ttk.Checkbutton(options_frame, text="Plot Bode (freq_response only)", variable=self.plot_bode_var).pack(anchor="w")
        
        self._update_method_ui()

        button_frame = ttk.Frame(self.root)
        button_frame.pack(fill=tk.X, padx=12, pady=5)
        self.start_button = ttk.Button(button_frame, text="Start Autotune", command=self.start_autotune)
        self.start_button.pack(side=tk.LEFT, padx=5)
        self.stop_button = ttk.Button(button_frame, text="Stop", state=tk.DISABLED, command=self.stop_autotune)
        self.stop_button.pack(side=tk.LEFT, padx=5)

        self.status_var = tk.StringVar(value="Idle")
        ttk.Label(button_frame, textvariable=self.status_var).pack(side=tk.LEFT, padx=10)

        summary_frame = ttk.LabelFrame(self.root, text="Latest Results", padding=10)
        summary_frame.pack(fill=tk.X, padx=12, pady=8)
        self.summary_labels = {}
        for idx, key in enumerate(["Ku", "Tu", "Kp", "Ki", "Kd"]):
            lbl = ttk.Label(summary_frame, text=f"{key}: --")
            lbl.grid(row=0, column=idx, padx=8)
            self.summary_labels[key] = lbl

        self.log_text = tk.Text(self.root, height=18, state=tk.DISABLED, wrap="word")
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=12, pady=(0, 12))

    def _append_log(self, message: str):
        if not message.endswith("\n"):
            message += "\n"
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, message)
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def _poll_log_queue(self):
        while not self.log_queue.empty():
            message = self.log_queue.get_nowait()
            self._append_log(message)
        self.root.after(120, self._poll_log_queue)

    def _set_running_state(self, running: bool):
        self.start_button.config(state=tk.DISABLED if running else tk.NORMAL)
        self.stop_button.config(state=tk.NORMAL if running else tk.DISABLED)
        self.status_var.set("Running..." if running else "Idle")

    def _update_method_ui(self):
        """Show/hide method-specific parameter frames and update amplitude default."""
        method = self.method_var.get()
        if method == "relay":
            self.relay_frame.grid()
            self.freq_frame.grid_remove()
            self.prbs_frame.grid_remove()
            self.amplitude_var.set(4.0)  # Default for relay
        elif method == "freq_response":
            self.relay_frame.grid_remove()
            self.freq_frame.grid()
            self.prbs_frame.grid_remove()
            self.amplitude_var.set(1.5)  # Default for frequency response
        else:  # prbs
            self.relay_frame.grid_remove()
            self.freq_frame.grid_remove()
            self.prbs_frame.grid()
            self.amplitude_var.set(0.06)  # Very small default for PRBS

    def _collect_params(self):
        config_path = self.config_var.get().strip()
        if not config_path:
            raise ValueError("Config path is required.")
        params = {
            "config_path": config_path,
            "axis": self.axis_var.get(),
            "method": self.method_var.get(),
            "amplitude": float(self.amplitude_var.get()),
            "rule": self.rule_var.get(),
            "results_file": self.results_var.get().strip() or None,
            "display": bool(self.display_var.get()),
            "simulate": bool(self.simulate_var.get()),
            "apply_config": bool(self.apply_var.get()),
        }
        if self.method_var.get() == "relay":
            params.update({
                "deadband": float(self.deadband_var.get()),
                "cycles": int(self.cycles_var.get()),
                "max_duration": float(self.max_duration_var.get()),
            })
        elif self.method_var.get() == "freq_response":
            params.update({
                "freq_min": float(self.freq_min_var.get()),
                "freq_max": float(self.freq_max_var.get()),
                "num_points": int(self.num_points_var.get()),
                "cycles_per_freq": int(self.cycles_per_freq_var.get()),
                "settle_time": float(self.settle_time_var.get()),
                "model_type": self.model_type_var.get(),
                "plot_bode": bool(self.plot_bode_var.get()),
            })
        else:  # prbs
            params.update({
                "prbs_length": int(self.prbs_length_var.get()),
                "sample_time": float(self.sample_time_var.get()),
                "clamp_threshold": float(self.clamp_threshold_var.get()),
                "safe_threshold": float(self.safe_threshold_var.get()),
                "model_type": self.model_type_var.get(),
                "max_duration": 300.0,  # PRBS needs longer time
            })
        return params

    def start_autotune(self):
        if self.worker_thread and self.worker_thread.is_alive():
            return
        try:
            params = self._collect_params()
        except ValueError as exc:
            if messagebox:
                messagebox.showerror("Invalid Parameters", str(exc))
            else:
                self.log_queue.put(f"[ERROR] {exc}")
            return

        method_names = {
            "relay": "relay test",
            "freq_response": "frequency response sweep",
            "prbs": "PRBS identification with safety clamp"
        }
        method_name = method_names.get(params["method"], params["method"])
        self.log_queue.put(f"[AUTOTUNE] Starting {method_name}...")
        self._set_running_state(True)
        self.stop_event = threading.Event()

        def status_cb(msg):
            self.log_queue.put(msg)

        def worker():
            try:
                entry, summary, pid, backup = autotune_once(
                    config_path=params["config_path"],
                    axis=params["axis"],
                    method=params["method"],
                    amplitude=params["amplitude"],
                    rule=params["rule"],
                    results_file=params["results_file"],
                    display=params["display"],
                    simulate=params["simulate"],
                    apply_config=params["apply_config"],
                    status_callback=status_cb,
                    stop_event=self.stop_event,
                    **{k: v for k, v in params.items() if k not in ["config_path", "axis", "method", "amplitude", "rule", "results_file", "display", "simulate", "apply_config"]},
                )
                self.last_entry = entry
                self.last_summary = summary
                self.last_pid = pid
                self.last_backup = backup
                self.log_queue.put("[AUTOTUNE] Completed successfully.")
                if backup:
                    self.log_queue.put(f"[AUTOTUNE] Config updated (backup at {backup}).")
            except Exception as exc:
                self.last_entry = None
                self.log_queue.put(f"[AUTOTUNE] Error: {exc}")
                if messagebox:
                    self.root.after(0, lambda: messagebox.showerror("Autotune failed", str(exc)))
            finally:
                self.root.after(0, self._on_worker_finished)

        self.worker_thread = threading.Thread(target=worker, daemon=True)
        self.worker_thread.start()

    def _on_worker_finished(self):
        self._set_running_state(False)
        if self.last_pid:
            method = self.last_entry.get("method", "relay") if self.last_entry else "relay"
            if method == "freq_response" or method == "prbs":
                # Model-based results (freq_response or prbs)
                model_type = self.last_summary.get("model_type", "unknown")
                self.summary_labels["Ku"].config(text=f"Model: {model_type}")
                params = self.last_summary.get("model_params", {})
                params_str = ", ".join([f"{k}={v:.3f}" for k, v in params.items()])
                self.summary_labels["Tu"].config(text=params_str[:30] + "..." if len(params_str) > 30 else params_str)
                if method == "prbs":
                    clamp_count = self.last_summary.get("clamp_count", 0)
                    active_ratio = self.last_summary.get("prbs_active_ratio", 0.0) * 100.0
                    # Show clamp info in a label if available
            else:
                # Relay test results
                self.summary_labels["Ku"].config(text=f"Ku: {self.last_summary.get('Ku', 0):.3f}")
                self.summary_labels["Tu"].config(text=f"Tu: {self.last_summary.get('Tu', 0):.3f}s")
            self.summary_labels["Kp"].config(text=f"Kp: {self.last_pid['Kp']:.3f}")
            self.summary_labels["Ki"].config(text=f"Ki: {self.last_pid['Ki']:.3f}")
            self.summary_labels["Kd"].config(text=f"Kd: {self.last_pid['Kd']:.3f}")
        else:
            for lbl in self.summary_labels.values():
                lbl.config(text=lbl.cget("text").split(":")[0] + ": --")

    def stop_autotune(self):
        if self.stop_event and not self.stop_event.is_set():
            self.stop_event.set()
            self.log_queue.put("[AUTOTUNE] Stop requested...")

    def run(self):
        self.root.mainloop()


def parse_args():
    parser = argparse.ArgumentParser(description="1D PID autotune for the Stewart platform.")
    parser.add_argument("--config", default="config_stewart.json", help="Path to configuration JSON.")
    parser.add_argument("--axis", choices=["x", "y"], default="x", help="Axis to tune (roll=X, pitch=Y).")
    parser.add_argument(
        "--method",
        choices=["relay", "freq_response", "prbs"],
        default="relay",
        help="Autotune method: relay (limit cycle), freq_response (Bode sweep), or prbs (PRBS with safety clamp).",
    )
    parser.add_argument("--amplitude", type=float, default=None, help="Command amplitude in degrees. Defaults: relay=4.0°, freq_response=1.5°, prbs=0.06°.")
    parser.add_argument("--deadband", type=float, default=0.002, help="Deadband around setpoint in meters (relay only).")
    parser.add_argument("--cycles", type=int, default=4, help="Number of oscillation cycles to capture (relay only).")
    parser.add_argument("--max-duration", type=float, default=120.0, help="Safety timeout in seconds (relay only).")
    parser.add_argument("--rule", choices=["zn", "tyreus"], default="zn", help="PID tuning rule.")
    parser.add_argument("--results-file", default="pid_autotune_results.json", help="Where to store tuning logs.")
    parser.add_argument("--display", action="store_true", help="Show OpenCV window with overlays.")
    parser.add_argument("--simulate", action="store_true", help="Skip servo connection, log data only.")
    parser.add_argument(
        "--apply-config",
        action="store_true",
        help="Write the tuned gains back into config_stewart.json for both axes.",
    )
    parser.add_argument(
        "--gui",
        action="store_true",
        help="Launch interactive autotune GUI instead of running once.",
    )
    # Frequency response parameters
    parser.add_argument("--freq-min", type=float, default=0.1, help="Minimum frequency for sweep (Hz, freq_response only).")
    parser.add_argument("--freq-max", type=float, default=5.0, help="Maximum frequency for sweep (Hz, freq_response only).")
    parser.add_argument("--num-points", type=int, default=12, help="Number of frequency points (freq_response only).")
    parser.add_argument("--cycles-per-freq", type=int, default=5, help="Cycles per frequency (freq_response only).")
    parser.add_argument("--settle-time", type=float, default=2.0, help="Settle time before measurement (s, freq_response only).")
    parser.add_argument(
        "--model-type",
        choices=["second_order", "first_order_delay"],
        default="second_order",
        help="Plant model type for fitting (freq_response only).",
    )
    parser.add_argument("--plot-bode", action="store_true", help="Generate Bode plot (freq_response only).")
    return parser.parse_args()


def main():
    args = parse_args()

    if args.gui:
        if tk is None or ttk is None:
            print("[AUTOTUNE] Tkinter is not available; cannot launch GUI.")
            sys.exit(1)
        gui = AutotuneGUI(default_config=args.config)
        gui.run()
        return

    # Set method-specific amplitude default if not provided
    if args.amplitude is None:
        if args.method == "prbs":
            args.amplitude = 0.06
        elif args.method == "freq_response":
            args.amplitude = 1.5
        else:  # relay
            args.amplitude = 4.0

    if args.method == "relay":
        print(
            f"[AUTOTUNE] Starting relay test on axis {args.axis.upper()} "
            f"with ±{args.amplitude:.1f}° command, deadband {args.deadband*1000:.1f} mm."
        )
    elif args.method == "freq_response":
        print(
            f"[AUTOTUNE] Starting frequency response sweep on axis {args.axis.upper()} "
            f"from {args.freq_min:.3f} to {args.freq_max:.3f} Hz ({args.num_points} points)."
        )
    else:  # prbs
        print(
            f"[AUTOTUNE] Starting PRBS excitation on axis {args.axis.upper()} "
            f"with amplitude {args.amplitude:.3f}°, clamp at ±{args.clamp_threshold*1000:.1f}mm."
        )

    entry, summary, pid, backup = autotune_once(
        config_path=args.config,
        axis=args.axis,
        method=args.method,
        amplitude=args.amplitude,
        deadband=args.deadband,
        cycles=args.cycles,
        max_duration=args.max_duration,
        rule=args.rule,
        results_file=args.results_file,
        display=args.display,
        simulate=args.simulate,
        apply_config=args.apply_config,
        freq_min=args.freq_min,
        freq_max=args.freq_max,
        num_points=args.num_points,
        cycles_per_freq=args.cycles_per_freq,
        settle_time=args.settle_time,
        model_type=args.model_type,
        plot_bode=args.plot_bode,
        prbs_length=args.prbs_length,
        sample_time=args.sample_time,
        clamp_threshold=args.clamp_threshold,
        safe_threshold=args.safe_threshold,
    )

    if args.method == "relay":
        print("\n[AUTOTUNE] Completed relay test:")
        print(f"  Ultimate gain Ku      : {summary['Ku']:.3f}")
        print(f"  Ultimate period Tu    : {summary['Tu']:.3f} s")
        print(f"  Avg. ball amplitude   : {summary['avg_peak_scaled'] / 100.0:.4f} m")
        print(f"  PID rule              : {pid['rule']}")
        print(f"  -> Kp={pid['Kp']:.3f}, Ki={pid['Ki']:.3f}, Kd={pid['Kd']:.3f}")
    elif args.method == "freq_response":
        print("\n[AUTOTUNE] Completed frequency response sweep:")
        print(f"  Model type            : {summary['model_type']}")
        print(f"  Model parameters      : {summary['model_params']}")
        print(f"  PID rule              : {pid['rule']}")
        print(f"  -> Kp={pid['Kp']:.3f}, Ki={pid['Ki']:.3f}, Kd={pid['Kd']:.3f}")
    else:  # prbs
        print("\n[AUTOTUNE] Completed PRBS identification:")
        print(f"  Model type            : {summary['model_type']}")
        print(f"  Model parameters      : {summary['model_params']}")
        print(f"  Clamp activations     : {summary['clamp_count']}")
        print(f"  PRBS active ratio     : {summary['prbs_active_ratio']*100:.1f}%")
        print(f"  PID rule              : {pid['rule']}")
        print(f"  -> Kp={pid['Kp']:.3f}, Ki={pid['Ki']:.3f}, Kd={pid['Kd']:.3f}")

    if args.apply_config:
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

