# Trajectory Updater Module
# Dynamically regenerates trajectories based on current ball position

import time
from trajectory_generator import generate_trajectory, sample_trajectory

class TrajectoryUpdater:
    """Manages dynamic trajectory replanning every Δt seconds."""
    
    def __init__(self, target_position, time_horizon, update_period=0.2, method='polynomial', order=5):
        """Initialize trajectory updater.
        
        Args:
            target_position (float): Target position (meters)
            time_horizon (float): Time horizon for trajectory (seconds)
            update_period (float): Period between trajectory updates (seconds)
            method (str): Trajectory generation method ('polynomial' or 'linear')
            order (int): Polynomial order (3 or 5, only for polynomial method)
        """
        self.target_position = target_position
        self.time_horizon = time_horizon
        self.update_period = update_period
        self.method = method
        self.order = order
        
        self.current_trajectory = None
        self.trajectory_start_time = None
        self.trajectory_start_position = None
        self.last_update_time = None
    
    def update(self, current_position, current_time):
        """Update trajectory based on current position and time.
        
        Args:
            current_position (float): Current ball position (meters)
            current_time (float): Current time (seconds)
            
        Returns:
            bool: True if trajectory was updated, False otherwise
        """
        # Initialize on first call
        if self.last_update_time is None:
            self.last_update_time = current_time
            self.trajectory_start_time = current_time
            self.trajectory_start_position = current_position
            self.current_trajectory = generate_trajectory(
                current_position,
                self.target_position,
                self.time_horizon,
                method=self.method,
                order=self.order
            )
            return True
        
        # Check if it's time to update
        elapsed_since_update = current_time - self.last_update_time
        if elapsed_since_update >= self.update_period:
            # Regenerate trajectory from current position to target
            self.trajectory_start_time = current_time
            self.trajectory_start_position = current_position
            self.current_trajectory = generate_trajectory(
                current_position,
                self.target_position,
                self.time_horizon,
                method=self.method,
                order=self.order
            )
            self.last_update_time = current_time
            # Debug output (can be commented out if too verbose)
            # print(f"[TRAJ_UPDATE] t={current_time:.2f}s: Regenerated trajectory from {current_position*100:.2f}cm → {self.target_position*100:.2f}cm")
            return True
        
        return False
    
    def get_setpoint(self, current_time):
        """Get setpoint from current trajectory at given time.
        
        Args:
            current_time (float): Current time (seconds)
            
        Returns:
            float: Setpoint position (meters)
        """
        if self.current_trajectory is None:
            return self.target_position
        
        # Calculate time relative to trajectory start
        t_rel = current_time - self.trajectory_start_time
        
        # Sample trajectory
        setpoint = sample_trajectory(self.current_trajectory, t_rel)
        
        return setpoint
    
    def reset(self):
        """Reset trajectory updater state."""
        self.current_trajectory = None
        self.trajectory_start_time = None
        self.trajectory_start_position = None
        self.last_update_time = None

