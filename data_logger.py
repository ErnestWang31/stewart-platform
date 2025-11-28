# Data Logger Module
# Logs all experiment signals to CSV files

import csv
import os
from datetime import datetime

class DataLogger:
    """Logs experiment data to CSV files."""
    
    def __init__(self, filename=None, experiment_name="experiment"):
        """Initialize data logger.
        
        Args:
            filename (str, optional): CSV filename. If None, auto-generates from experiment_name
            experiment_name (str): Name of experiment for auto-generated filename
        """
        if filename is None:
            # Create results directory if it doesn't exist
            results_dir = "results"
            if not os.path.exists(results_dir):
                os.makedirs(results_dir)
            
            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(results_dir, f"{experiment_name}_{timestamp}.csv")
        
        self.filename = filename
        self.data = []
        self.fieldnames = [
            'time',
            'position_x',
            'setpoint',
            'error',
            'control_signal',
            'tilt_angle',
            'saturated'
        ]
    
    def log(self, time, position_x, setpoint, error, control_signal, tilt_angle, saturated):
        """Log a data point.
        
        Args:
            time (float): Time in seconds
            position_x (float): Measured ball position (meters)
            setpoint (float): Desired position (meters)
            error (float): Tracking error (meters)
            control_signal (float): Control output (degrees)
            tilt_angle (float): Actual platform tilt angle (degrees)
            saturated (bool or int): Saturation flag (1 if saturated, 0 otherwise)
        """
        # Convert saturated to int if boolean
        saturated_int = 1 if saturated else 0
        
        self.data.append({
            'time': time,
            'position_x': position_x,
            'setpoint': setpoint,
            'error': error,
            'control_signal': control_signal,
            'tilt_angle': tilt_angle,
            'saturated': saturated_int
        })
    
    def save(self):
        """Save logged data to CSV file."""
        if not self.data:
            print(f"[DATA_LOGGER] No data to save to {self.filename}")
            return
        
        # Ensure results directory exists
        results_dir = os.path.dirname(self.filename)
        if results_dir and not os.path.exists(results_dir):
            os.makedirs(results_dir)
        
        with open(self.filename, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=self.fieldnames)
            writer.writeheader()
            writer.writerows(self.data)
        
        print(f"[DATA_LOGGER] Saved {len(self.data)} data points to {self.filename}")
    
    def get_data_arrays(self):
        """Get logged data as numpy arrays for analysis.
        
        Returns:
            dict: Dictionary with arrays for each field
        """
        import numpy as np
        
        if not self.data:
            return {
                'time': np.array([]),
                'position_x': np.array([]),
                'setpoint': np.array([]),
                'error': np.array([]),
                'control_signal': np.array([]),
                'tilt_angle': np.array([]),
                'saturated': np.array([])
            }
        
        return {
            'time': np.array([d['time'] for d in self.data]),
            'position_x': np.array([d['position_x'] for d in self.data]),
            'setpoint': np.array([d['setpoint'] for d in self.data]),
            'error': np.array([d['error'] for d in self.data]),
            'control_signal': np.array([d['control_signal'] for d in self.data]),
            'tilt_angle': np.array([d['tilt_angle'] for d in self.data]),
            'saturated': np.array([d['saturated'] for d in self.data])
        }
    
    def clear(self):
        """Clear all logged data."""
        self.data = []

