#!/usr/bin/env python3
"""
Interactive GUI to fine-tune servo neutral angles in very small increments.

Allows nudging each motor by tiny degree steps, immediately applies the new
neutral position (by commanding roll=0, pitch=0 using the Stewart controller),
and provides a button to write the updated neutral angles back into the config.
"""

import argparse
import json
import sys
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

import numpy as np

from stewart_platform_controller import StewartPlatformController


class NeutralTuneApp:
    def __init__(self, config_path: Path, simulate: bool):
        self.config_path = config_path
        self.simulate = simulate

        self.controller = StewartPlatformController(str(self.config_path))

        if not self.simulate:
            if not self.controller.connect_servos():
                raise RuntimeError("Failed to connect to servos. Re-run with --simulate to preview only.")
        else:
            print("[NEUTRAL GUI] Simulation mode enabled (no servo commands).")

        self.current_neutrals = [float(v) for v in self.controller.neutral_angles]

        self.root = tk.Tk()
        self.root.title("Neutral Angle Tuner")
        self.root.geometry("420x300")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.step_var = tk.DoubleVar(master=self.root, value=0.05)
        self.servo_vars = []
        self._build_ui()
        self._apply_neutrals(initial=True)

    def _build_ui(self):
        ttk.Label(self.root, text="Fine Neutral Adjustments", font=("Arial", 14, "bold")).pack(pady=8)

        step_frame = ttk.Frame(self.root)
        step_frame.pack(pady=4)
        ttk.Label(step_frame, text="Adjustment step (deg):").pack(side=tk.LEFT, padx=4)
        tk.Entry(step_frame, textvariable=self.step_var, width=6).pack(side=tk.LEFT)

        servo_frame = ttk.Frame(self.root)
        servo_frame.pack(pady=10, fill=tk.X)

        for idx in range(3):
            frame = ttk.LabelFrame(servo_frame, text=f"Motor {idx + 1}", padding=8)
            frame.pack(side=tk.LEFT, padx=6, fill=tk.BOTH, expand=True)

            value_var = tk.DoubleVar(master=self.root, value=self.current_neutrals[idx])
            self.servo_vars.append(value_var)

            ttk.Label(frame, textvariable=value_var, font=("Consolas", 12, "bold")).pack(pady=4)

            button_frame = ttk.Frame(frame)
            button_frame.pack(pady=4)
            ttk.Button(
                button_frame,
                text="-",
                width=3,
                command=lambda i=idx: self.adjust_servo(i, -self.step_var.get()),
            ).pack(side=tk.LEFT, padx=2)
            ttk.Button(
                button_frame,
                text="+",
                width=3,
                command=lambda i=idx: self.adjust_servo(i, self.step_var.get()),
            ).pack(side=tk.LEFT, padx=2)

            ttk.Button(
                frame,
                text="Set exact…",
                command=lambda i=idx: self.prompt_exact_value(i),
            ).pack(pady=4, fill=tk.X)

        action_frame = ttk.Frame(self.root)
        action_frame.pack(pady=10)

        ttk.Button(action_frame, text="Apply Neutral & Hold", command=self._apply_neutrals).pack(
            side=tk.LEFT, padx=5
        )
        ttk.Button(action_frame, text="Save to Config", command=self.save_to_config).pack(
            side=tk.LEFT, padx=5
        )
        ttk.Button(action_frame, text="Close", command=self.on_close).pack(side=tk.LEFT, padx=5)

        ttk.Label(
            self.root,
            text="Platform is commanded to roll=0°, pitch=0° after each adjustment.\n"
            "Use tiny steps to level the plate, then save when satisfied.",
            font=("Arial", 9),
            anchor="center",
            justify="center",
        ).pack(pady=6)

    def adjust_servo(self, index: int, delta: float):
        new_value = np.clip(self.current_neutrals[index] + delta, 0.0, 30.0)
        self.current_neutrals[index] = new_value
        self.servo_vars[index].set(round(new_value, 3))
        self._apply_neutrals()

    def prompt_exact_value(self, index: int):
        popup = tk.Toplevel(self.root)
        popup.title(f"Set Motor {index + 1} Angle")
        popup.grab_set()

        ttk.Label(popup, text="Angle (deg, 0-30):").pack(pady=6, padx=10)
        value_var = tk.DoubleVar(master=popup, value=self.current_neutrals[index])
        entry = tk.Entry(popup, textvariable=value_var, width=8, justify="center")
        entry.pack(pady=4)
        entry.focus()

        def apply_value():
            try:
                val = float(value_var.get())
            except ValueError:
                messagebox.showerror("Invalid value", "Enter a numeric angle.")
                return
            if not (0.0 <= val <= 30.0):
                messagebox.showerror("Invalid range", "Angle must be between 0 and 30 degrees.")
                return
            self.current_neutrals[index] = val
            self.servo_vars[index].set(round(val, 3))
            popup.destroy()
            self._apply_neutrals()

        ttk.Button(popup, text="Apply", command=apply_value).pack(pady=6)
        popup.bind("<Return>", lambda _: apply_value())

    def _apply_neutrals(self, initial: bool = False):
        self.controller.neutral_angles = list(self.current_neutrals)
        if not self.simulate:
            self.controller.send_platform_tilt(0.0, 0.0)
        if initial:
            # Ensure UI reflects loaded values
            for idx, var in enumerate(self.servo_vars):
                var.set(round(self.current_neutrals[idx], 3))

    def save_to_config(self):
        try:
            with open(self.config_path, "r", encoding="utf-8") as f:
                config = json.load(f)
        except Exception as exc:
            messagebox.showerror("Config error", f"Failed to load config: {exc}")
            return

        config.setdefault("servo", {})["neutral_angles"] = [
            round(value, 3) for value in self.current_neutrals
        ]

        try:
            with open(self.config_path, "w", encoding="utf-8") as f:
                json.dump(config, f, indent=2)
        except Exception as exc:
            messagebox.showerror("Config error", f"Failed to write config: {exc}")
            return

        messagebox.showinfo("Saved", f"Updated neutral angles persisted to {self.config_path}.")

    def on_close(self):
        if messagebox.askokcancel("Exit", "Leave the platform holding current neutral?"):
            print("[NEUTRAL GUI] Holding current neutral until power-off.")
        else:
            print("[NEUTRAL GUI] Returning to neutral before exit.")
            if not self.simulate:
                self.controller.send_platform_tilt(0.0, 0.0)
        if self.controller.servo_serial:
            self.controller.servo_serial.close()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def parse_args():
    parser = argparse.ArgumentParser(description="Interactive neutral angle tuning GUI.")
    parser.add_argument("--config", default="config_stewart.json", help="Path to config JSON.")
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="Skip servo connection; adjustments won't move hardware.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    try:
        app = NeutralTuneApp(Path(args.config), simulate=args.simulate)
    except Exception as exc:
        print(f"[NEUTRAL GUI] Error: {exc}")
        sys.exit(1)
    app.run()


if __name__ == "__main__":
    main()

