# Disturbance Module
# Provides functions to add disturbances to experiments

import numpy as np

class Disturbance:
    """Manages disturbances for experiments."""
    
    def __init__(self, disturbance_type='none', **kwargs):
        """Initialize disturbance.
        
        Args:
            disturbance_type: 'none', 'step', 'impulse', 'sinusoidal', 'manual'
            **kwargs: Disturbance parameters
                - For 'step': time, magnitude
                - For 'impulse': time, magnitude, duration, apply_to ('position' or 'actuator')
                - For 'sinusoidal': start_time, amplitude, frequency, duration
                - For 'manual': magnitude (applied when apply_manual() called)
        """
        self.disturbance_type = disturbance_type
        self.params = kwargs
        self.manual_applied = False
        self.apply_to = kwargs.get('apply_to', 'position')  # 'position' or 'actuator'
        
    def apply(self, position, current_time):
        """Apply disturbance to position (backward compatibility).
        
        Args:
            position: Current ball position (meters)
            current_time: Current time (seconds)
            
        Returns:
            disturbed_position: Position with disturbance applied
        """
        return self._apply_position_disturbance(position, current_time)
    
    def apply_to_position(self, position, current_time):
        """Apply disturbance to position measurement.
        
        Args:
            position: Current ball position (meters)
            current_time: Current time (seconds)
            
        Returns:
            disturbed_position: Position with disturbance applied
        """
        return self._apply_position_disturbance(position, current_time)
    
    def apply_to_actuator(self, control_output, current_time):
        """Apply disturbance to actuator/control output.
        
        Args:
            control_output: Control output (degrees)
            current_time: Current time (seconds)
            
        Returns:
            disturbed_output: Control output with disturbance applied
        """
        if self.disturbance_type == 'none':
            return control_output
        
        elif self.disturbance_type == 'step':
            time = self.params.get('time', 2.0)
            magnitude = self.params.get('magnitude', 2.0)  # degrees
            
            if current_time >= time and not hasattr(self, '_step_applied'):
                self._step_applied = True
                return control_output + magnitude
            return control_output
        
        elif self.disturbance_type == 'impulse':
            time = self.params.get('time', 2.0)
            magnitude = self.params.get('magnitude', 2.0)  # degrees
            duration = self.params.get('duration', 0.5)  # seconds
            
            if time <= current_time <= time + duration:
                return control_output + magnitude
            return control_output
        
        elif self.disturbance_type == 'sinusoidal':
            start_time = self.params.get('start_time', 2.0)
            amplitude = self.params.get('amplitude', 1.0)  # degrees
            frequency = self.params.get('frequency', 2.0)  # Hz
            duration = self.params.get('duration', 1.0)  # seconds
            
            if start_time <= current_time <= start_time + duration:
                t = current_time - start_time
                return control_output + amplitude * np.sin(2 * np.pi * frequency * t)
            return control_output
        
        return control_output
    
    def _apply_position_disturbance(self, position, current_time):
        """Internal method to apply position disturbance."""
        if self.disturbance_type == 'none':
            return position
        
        elif self.disturbance_type == 'step':
            # Step disturbance: sudden position change at specific time
            time = self.params.get('time', 2.0)
            magnitude = self.params.get('magnitude', 0.05)  # meters (5cm)
            
            if current_time >= time and not hasattr(self, '_step_applied'):
                self._step_applied = True
                return position + magnitude
            return position
        
        elif self.disturbance_type == 'impulse':
            # Impulse disturbance: temporary position change
            time = self.params.get('time', 2.0)
            magnitude = self.params.get('magnitude', 0.05)  # meters
            duration = self.params.get('duration', 0.5)  # seconds
            
            if time <= current_time <= time + duration:
                return position + magnitude
            return position
        
        elif self.disturbance_type == 'sinusoidal':
            # Sinusoidal disturbance: oscillating position change
            start_time = self.params.get('start_time', 2.0)
            amplitude = self.params.get('amplitude', 0.02)  # meters (2cm)
            frequency = self.params.get('frequency', 2.0)  # Hz
            duration = self.params.get('duration', 1.0)  # seconds
            
            if start_time <= current_time <= start_time + duration:
                t = current_time - start_time
                return position + amplitude * np.sin(2 * np.pi * frequency * t)
            return position
        
        elif self.disturbance_type == 'manual':
            # Manual disturbance: applied when apply_manual() is called
            if self.manual_applied:
                magnitude = self.params.get('magnitude', 0.05)  # meters
                return position + magnitude
            return position
        
        return position
    
    def apply_manual(self):
        """Apply manual disturbance (for 'manual' type)."""
        self.manual_applied = True
    
    def reset_manual(self):
        """Reset manual disturbance."""
        self.manual_applied = False


def create_step_disturbance(time=2.0, magnitude=0.05):
    """Create a step disturbance.
    
    Args:
        time: When to apply disturbance (seconds)
        magnitude: Position change (meters)
        
    Returns:
        Disturbance object
    """
    return Disturbance('step', time=time, magnitude=magnitude)


def create_impulse_disturbance(time=2.0, magnitude=0.05, duration=0.5, apply_to='position'):
    """Create an impulse disturbance.
    
    Args:
        time: When to apply disturbance (seconds)
        magnitude: Disturbance magnitude (meters for position, degrees for actuator)
        duration: How long disturbance lasts (seconds)
        apply_to: 'position' or 'actuator' (default: 'position')
        
    Returns:
        Disturbance object
    """
    return Disturbance('impulse', time=time, magnitude=magnitude, duration=duration, apply_to=apply_to)


def create_sinusoidal_disturbance(start_time=2.0, amplitude=0.02, frequency=2.0, duration=1.0, apply_to='position'):
    """Create a sinusoidal disturbance.
    
    Args:
        start_time: When to start disturbance (seconds)
        amplitude: Disturbance amplitude (meters for position, degrees for actuator)
        frequency: Oscillation frequency (Hz)
        duration: How long disturbance lasts (seconds)
        apply_to: 'position' or 'actuator' (default: 'position')
        
    Returns:
        Disturbance object
    """
    return Disturbance('sinusoidal', start_time=start_time, amplitude=amplitude, 
                      frequency=frequency, duration=duration, apply_to=apply_to)

