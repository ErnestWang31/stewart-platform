"""
Post-processing tool that converts captured Ku/Tu data into PID gains using
classic Ziegler–Nichols formulas.

Usage examples:

    # 1) Preview recommended gains for the latest capture (defaults).
    python zn_gain_calculator.py

    # 2) Specify a capture file, axis, and ZN rule, then update config.
    python zn_gain_calculator.py --results custom_results.json --axis y \
        --rule pessen_integral --write-config

    # 3) Override Ku/Tu manually without a capture file.
    python zn_gain_calculator.py --axis x --ku 1.9 --tu 0.85

The script always prints the computed Kp/Ki/Kd. When --write-config is passed,
those gains are also written into config_stewart.json for the selected axis.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Dict, Tuple


DEFAULT_RESULTS_PATH = Path("zn_capture_results.json")
DEFAULT_CONFIG_PATH = Path("config_stewart.json")

ZN_RULES: Dict[str, Dict[str, float]] = {
    "classic": {"Kp": 0.6, "Ti": 0.5, "Td": 0.125},
    "pessen_integral": {"Kp": 0.7, "Ti": 0.4, "Td": 0.15},
    "some_overshoot": {"Kp": 0.33, "Ti": 0.5, "Td": 0.33},
    "no_overshoot": {"Kp": 0.2, "Ti": 0.5, "Td": 0.33},
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert Ku/Tu data into PID gains using Ziegler–Nichols rules."
    )
    parser.add_argument(
        "--results",
        type=Path,
        default=DEFAULT_RESULTS_PATH,
        help="Path to zn_capture_results.json (default: zn_capture_results.json)",
    )
    parser.add_argument(
        "--axis",
        choices=("x", "y"),
        help="Axis to tune (roll=X or pitch=Y). Required if results contain both.",
    )
    parser.add_argument(
        "--rule",
        choices=ZN_RULES.keys(),
        default="classic",
        help="Ziegler–Nichols rule to apply (default: classic).",
    )
    parser.add_argument(
        "--ku",
        type=float,
        help="Override Ku instead of reading from results file.",
    )
    parser.add_argument(
        "--tu",
        type=float,
        help="Override Tu instead of reading from results file.",
    )
    parser.add_argument(
        "--write-config",
        action="store_true",
        help="Persist computed gains into config_stewart.json for the chosen axis.",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG_PATH,
        help="Path to config_stewart.json (default: config_stewart.json).",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Optional path to write a JSON blob with Ku/Tu and computed gains.",
    )
    return parser.parse_args()


def load_capture_results(results_path: Path):
    if not results_path.exists():
        raise FileNotFoundError(
            f"Results file {results_path} not found. Run zn_capture_controller.py first "
            "or provide --ku/--tu overrides."
        )

    with open(results_path, "r") as f:
        data = json.load(f)

    if isinstance(data, dict):
        return [data]
    if isinstance(data, list):
        return data

    raise ValueError("Unexpected capture results format; expected dict or list.")


def select_capture_entry(entries, axis: str | None):
    if not entries:
        raise ValueError("Capture results are empty.")

    if axis is None:
        if len(entries) == 1:
            return entries[0]
        raise ValueError("Multiple captures found—specify --axis.")

    filtered = [entry for entry in entries if entry.get("axis") == axis]
    if not filtered:
        raise ValueError(f"No capture entry for axis '{axis}'.")

    return filtered[-1]


def compute_pid_gains(ku: float, tu: float, rule: str) -> Tuple[float, float, float]:
    if ku <= 0 or tu <= 0:
        raise ValueError("Ku and Tu must both be positive values.")

    params = ZN_RULES[rule]
    kp = params["Kp"] * ku
    ti = params["Ti"] * tu
    td = params["Td"] * tu

    ki = kp / ti if ti != 0 else 0.0
    kd = kp * td
    return kp, ki, kd


def update_config(
    config_path: Path,
    axis: str,
    kp: float,
    ki: float,
    kd: float,
) -> None:
    if not config_path.exists():
        raise FileNotFoundError(
            f"Cannot write gains: {config_path} does not exist. "
            "Run calibration first or point --config to an existing file."
        )

    with open(config_path, "r") as f:
        config = json.load(f)

    config.setdefault("pid", {})
    if axis == "x":
        config["pid"]["Kp_x"] = round(kp, 6)
        config["pid"]["Ki_x"] = round(ki, 6)
        config["pid"]["Kd_x"] = round(kd, 6)
    else:
        config["pid"]["Kp_y"] = round(kp, 6)
        config["pid"]["Ki_y"] = round(ki, 6)
        config["pid"]["Kd_y"] = round(kd, 6)

    with open(config_path, "w") as f:
        json.dump(config, f, indent=2)

    print(f"[CONFIG] Updated PID gains for axis '{axis}' in {config_path}")


def main() -> None:
    args = parse_args()

    ku = args.ku
    tu = args.tu
    axis = args.axis

    if ku is None or tu is None:
        entries = load_capture_results(args.results)
        selected = select_capture_entry(entries, axis)
        axis_from_file = selected.get("axis")
        if axis is None:
            axis = axis_from_file
        elif axis != axis_from_file:
            print(
                f"[WARN] Requested axis '{axis}' but capture entry axis is '{axis_from_file}'. "
                "Proceeding with requested axis."
            )
        ku = selected.get("Ku")
        tu = selected.get("Tu")
        if ku is None or tu is None:
            raise ValueError("Capture entry missing Ku/Tu. Re-run capture or pass overrides.")
    else:
        if axis is None:
            raise ValueError("When overriding Ku/Tu you must also specify --axis.")

    kp, ki, kd = compute_pid_gains(ku, tu, args.rule)

    print("=== Ziegler–Nichols Gain Recommendation ===")
    print(f"Axis:        {axis.upper()}")
    print(f"Rule:        {args.rule}")
    print(f"Ku:          {ku:.6f}")
    print(f"Tu:          {tu:.6f} s")
    print("------------------------------------------")
    print(f"Kp:          {kp:.6f}")
    print(f"Ki:          {ki:.6f}")
    print(f"Kd:          {kd:.6f}")

    if args.output:
        payload = {
            "axis": axis,
            "rule": args.rule,
            "Ku": ku,
            "Tu": tu,
            "Kp": kp,
            "Ki": ki,
            "Kd": kd,
            "source": str(args.results),
        }
        with open(args.output, "w") as f:
            json.dump(payload, f, indent=2)
        print(f"[OUTPUT] Wrote gain summary to {args.output}")

    if args.write_config:
        update_config(args.config, axis, kp, ki, kd)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"[ERROR] {exc}")
        import traceback

        traceback.print_exc()

