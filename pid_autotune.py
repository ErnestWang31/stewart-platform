# PID Autotuning Module for Stewart Platform
# Implements multiple tuning methods: Relay Feedback, Ziegler-Nichols, and Step Response

import numpy as np
import time
from collections import deque

class PIDAutotuner:
    """PID autotuner with multiple tuning methods."""
    
    def __init__(self, output_limit=15.0, sample_time=0.033):
        """Initialize autotuner.
        
        Args:
            output_limit: Maximum control output (degrees)
            sample_time: Control loop sample time (seconds)
        """
        self.output_limit = output_limit
        self.sample_time = sample_time
        self.tuning_data = {
            'time': [],
            'position': [],
            'control': [],
            'setpoint': []
        }
    
    def relay_feedback_tune(self, get_position_callback, send_control_callback, 
                           axis='x', setpoint=0.0, duration=30.0, 
                           relay_amplitude=None, hysteresis=0.01):
        """Relay feedback autotuning method.
        
        Uses relay (on/off) control to induce oscillations and find ultimate gain/period.
        
        Args:
            get_position_callback: Function that returns current position (meters)
            send_control_callback: Function(control_output) to send control command
            axis: 'x' or 'y' for which axis to tune
            setpoint: Target position (meters)
            duration: Tuning duration (seconds)
            relay_amplitude: Relay output amplitude (None = use output_limit * 0.5)
            hysteresis: Hysteresis band for relay switching (meters)
            
        Returns:
            dict: {'Kp': float, 'Ki': float, 'Kd': float, 'Ku': float, 'Tu': float}
        """
        if relay_amplitude is None:
            relay_amplitude = self.output_limit * 0.5
        
        print(f"[AUTOTUNE] Starting relay feedback tuning for {axis.upper()}-axis")
        print(f"[AUTOTUNE] Setpoint: {setpoint:.4f}m, Duration: {duration}s")
        
        # Clear tuning data
        self.tuning_data = {'time': [], 'position': [], 'control': [], 'setpoint': []}
        
        start_time = time.time()
        last_switch_time = start_time
        relay_state = 1  # 1 = positive, -1 = negative
        last_position = setpoint
        
        # Wait for initial position reading
        try:
            current_pos = get_position_callback()
            if axis == 'x':
                position = current_pos[0] if isinstance(current_pos, tuple) else current_pos
            else:
                position = current_pos[1] if isinstance(current_pos, tuple) else current_pos
        except:
            position = setpoint
        
        oscillations = []
        oscillation_started = False
        last_zero_crossing = None
        
        while time.time() - start_time < duration:
            current_time = time.time()
            dt = current_time - start_time
            
            # Get current position
            try:
                current_pos = get_position_callback()
                if axis == 'x':
                    position = current_pos[0] if isinstance(current_pos, tuple) else current_pos
                else:
                    position = current_pos[1] if isinstance(current_pos, tuple) else current_pos
            except:
                position = last_position
            
            error = setpoint - position
            
            # Relay switching logic with hysteresis
            if relay_state == 1 and error < -hysteresis:
                relay_state = -1
                if oscillation_started and last_zero_crossing is not None:
                    period = current_time - last_zero_crossing
                    oscillations.append(period)
                last_zero_crossing = current_time
                oscillation_started = True
            elif relay_state == -1 and error > hysteresis:
                relay_state = 1
                if oscillation_started and last_zero_crossing is not None:
                    period = current_time - last_zero_crossing
                    oscillations.append(period)
                last_zero_crossing = current_time
                oscillation_started = True
            
            # Apply relay control
            control_output = relay_state * relay_amplitude
            if axis == 'x':
                send_control_callback(control_output, 0.0)
            else:
                send_control_callback(0.0, control_output)
            
            # Log data
            self.tuning_data['time'].append(dt)
            self.tuning_data['position'].append(position)
            self.tuning_data['control'].append(control_output)
            self.tuning_data['setpoint'].append(setpoint)
            
            last_position = position
            time.sleep(self.sample_time)
        
        # Calculate ultimate period (Tu) and amplitude
        if len(oscillations) < 2:
            print("[AUTOTUNE] Warning: Insufficient oscillations detected")
            Tu = 2.0  # Default fallback
            amplitude = 0.01
        else:
            # Use median period for stability
            Tu = np.median(oscillations)
            # Calculate amplitude from position data
            if len(self.tuning_data['position']) > 10:
                pos_array = np.array(self.tuning_data['position'][-int(2*Tu/self.sample_time):])
                amplitude = (np.max(pos_array) - np.min(pos_array)) / 2.0
            else:
                amplitude = 0.01
        
        # Calculate ultimate gain (Ku) using describing function method
        # Ku = 4 * relay_amplitude / (π * amplitude)
        if amplitude > 1e-6:
            Ku = 4.0 * relay_amplitude / (np.pi * amplitude)
        else:
            Ku = 10.0  # Default fallback
        
        print(f"[AUTOTUNE] Relay feedback results: Tu={Tu:.2f}s, Ku={Ku:.2f}, Amplitude={amplitude:.4f}m")
        
        # Apply Ziegler-Nichols PID formulas
        Kp, Ki, Kd = self._ziegler_nichols_pid(Ku, Tu)
        
        return {
            'Kp': Kp,
            'Ki': Ki,
            'Kd': Kd,
            'Ku': Ku,
            'Tu': Tu,
            'amplitude': amplitude
        }
    
    def ziegler_nichols_tune(self, Ku, Tu):
        """Ziegler-Nichols tuning method using provided Ku and Tu.
        
        Args:
            Ku: Ultimate gain
            Tu: Ultimate period (seconds)
            
        Returns:
            dict: {'Kp': float, 'Ki': float, 'Kd': float}
        """
        Kp, Ki, Kd = self._ziegler_nichols_pid(Ku, Tu)
        
        return {
            'Kp': Kp,
            'Ki': Ki,
            'Kd': Kd,
            'Ku': Ku,
            'Tu': Tu
        }
    
    def _ziegler_nichols_pid(self, Ku, Tu):
        """Calculate PID gains using Ziegler-Nichols formulas.
        
        Args:
            Ku: Ultimate gain
            Tu: Ultimate period (seconds)
            
        Returns:
            tuple: (Kp, Ki, Kd)
        """
        # Ziegler-Nichols PID tuning formulas
        Kp = 0.6 * Ku
        Ki = 1.2 * Ku / Tu
        Kd = 3.0 * Ku * Tu / 40.0
        
        return Kp, Ki, Kd
    
    def step_response_tune(self, get_position_callback, send_control_callback,
                          axis='x', setpoint=0.0, step_size=0.05, duration=10.0):
        """Step response autotuning method.
        
        Applies step input and analyzes response characteristics.
        
        Args:
            get_position_callback: Function that returns current position (meters)
            send_control_callback: Function(control_output_x, control_output_y) to send control
            axis: 'x' or 'y' for which axis to tune
            setpoint: Initial setpoint (meters)
            step_size: Step size in meters
            duration: Tuning duration (seconds)
            
        Returns:
            dict: {'Kp': float, 'Ki': float, 'Kd': float, 'overshoot': float, 'settling_time': float}
        """
        print(f"[AUTOTUNE] Starting step response tuning for {axis.upper()}-axis")
        print(f"[AUTOTUNE] Step size: {step_size:.4f}m, Duration: {duration}s")
        
        # Clear tuning data
        self.tuning_data = {'time': [], 'position': [], 'control': [], 'setpoint': []}
        
        start_time = time.time()
        step_applied = False
        step_time = None
        initial_position = setpoint
        
        # Initial settling period
        settle_start = start_time
        settle_duration = 2.0
        
        while time.time() - start_time < duration:
            current_time = time.time()
            dt = current_time - start_time
            
            # Get current position
            try:
                current_pos = get_position_callback()
                if axis == 'x':
                    position = current_pos[0] if isinstance(current_pos, tuple) else current_pos
                else:
                    position = current_pos[1] if isinstance(current_pos, tuple) else current_pos
            except:
                position = initial_position
            
            # Apply step after settling period
            if not step_applied and (current_time - settle_start) > settle_duration:
                step_applied = True
                step_time = current_time
                target_setpoint = setpoint + step_size
                print(f"[AUTOTUNE] Applying step to {target_setpoint:.4f}m")
            else:
                target_setpoint = setpoint + step_size if step_applied else setpoint
            
            # Simple P control to reach setpoint (for step response analysis)
            error = target_setpoint - position
            Kp_temp = 5.0  # Temporary proportional gain for step response
            control_output = np.clip(Kp_temp * error * 100, -self.output_limit, self.output_limit)
            
            if axis == 'x':
                send_control_callback(control_output, 0.0)
            else:
                send_control_callback(0.0, control_output)
            
            # Log data
            self.tuning_data['time'].append(dt)
            self.tuning_data['position'].append(position)
            self.tuning_data['control'].append(control_output)
            self.tuning_data['setpoint'].append(target_setpoint)
            
            time.sleep(self.sample_time)
        
        # Analyze step response
        if len(self.tuning_data['position']) < 10:
            print("[AUTOTUNE] Warning: Insufficient data for analysis")
            return {'Kp': 5.0, 'Ki': 0.0, 'Kd': 1.0, 'overshoot': 0.0, 'settling_time': 0.0}
        
        # Find step response characteristics
        pos_array = np.array(self.tuning_data['position'])
        time_array = np.array(self.tuning_data['time'])
        
        # Find when step was applied
        if step_time:
            step_idx = int((step_time - start_time) / self.sample_time)
        else:
            step_idx = int(settle_duration / self.sample_time)
        
        if step_idx >= len(pos_array):
            step_idx = len(pos_array) // 4
        
        # Extract response after step
        response_pos = pos_array[step_idx:]
        response_time = time_array[step_idx:] - time_array[step_idx]
        
        if len(response_pos) < 5:
            print("[AUTOTUNE] Warning: Insufficient response data")
            return {'Kp': 5.0, 'Ki': 0.0, 'Kd': 1.0, 'overshoot': 0.0, 'settling_time': 0.0}
        
        # Calculate overshoot
        final_value = response_pos[-1] if len(response_pos) > 0 else setpoint + step_size
        peak_value = np.max(response_pos) if len(response_pos) > 0 else final_value
        overshoot = (peak_value - final_value) / step_size if step_size > 1e-6 else 0.0
        
        # Calculate rise time (10% to 90%)
        initial_val = response_pos[0]
        target_10 = initial_val + 0.1 * step_size
        target_90 = initial_val + 0.9 * step_size
        
        rise_time = 0.0
        for i, (t, p) in enumerate(zip(response_time, response_pos)):
            if p >= target_10 and rise_time == 0.0:
                rise_time = t
            if p >= target_90:
                rise_time = t - rise_time
                break
        
        # Calculate settling time (within 2% of final value)
        settling_time = 0.0
        tolerance = 0.02 * abs(step_size)
        for i in range(len(response_pos) - 1, -1, -1):
            if abs(response_pos[i] - final_value) > tolerance:
                if i < len(response_time):
                    settling_time = response_time[i]
                break
        
        # Estimate system parameters and calculate PID gains
        # Simple heuristic tuning based on response characteristics
        if rise_time > 0:
            # Estimate natural frequency and damping
            if overshoot > 0:
                # Underdamped system
                zeta = -np.log(overshoot) / np.sqrt(np.pi**2 + np.log(overshoot)**2)
                wn = np.pi / (rise_time * np.sqrt(1 - zeta**2)) if zeta < 1 else 1.0 / rise_time
            else:
                # Overdamped or critically damped
                zeta = 1.0
                wn = 1.0 / rise_time
            
            # Calculate PID gains using heuristic formulas
            Kp = 2.0 * zeta * wn / (step_size * 100) if step_size > 1e-6 else 5.0
            Ki = wn**2 / (step_size * 100) if step_size > 1e-6 else 0.0
            Kd = (2.0 * zeta * wn - Kp) / (step_size * 100) if step_size > 1e-6 else 1.0
            
            # Clamp to reasonable values
            Kp = np.clip(Kp, 0.1, 100.0)
            Ki = np.clip(Ki, 0.0, 10.0)
            Kd = np.clip(Kd, 0.0, 20.0)
        else:
            # Fallback values
            Kp = 5.0
            Ki = 0.0
            Kd = 1.0
        
        print(f"[AUTOTUNE] Step response results: Overshoot={overshoot:.2%}, "
              f"Rise time={rise_time:.2f}s, Settling time={settling_time:.2f}s")
        print(f"[AUTOTUNE] Recommended gains: Kp={Kp:.2f}, Ki={Ki:.2f}, Kd={Kd:.2f}")
        
        return {
            'Kp': Kp,
            'Ki': Ki,
            'Kd': Kd,
            'overshoot': overshoot,
            'settling_time': settling_time,
            'rise_time': rise_time
        }
    
    def get_tuning_data(self):
        """Get collected tuning data.
        
        Returns:
            dict: Tuning data with time, position, control, and setpoint arrays
        """
        return self.tuning_data.copy()

