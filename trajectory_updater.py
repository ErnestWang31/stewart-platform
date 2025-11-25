# Trajectory Updater Module
# Regenerates trajectory periodically based on current measured position

from trajectory_generator import generate_trajectory, Trajectory

class TrajectoryUpdater:
    """Manages trajectory regeneration every Δt based on current position."""
    
    def __init__(self, target_position, trajectory_duration, update_interval, method='linear'):
        """Initialize trajectory updater.
        
        Args:
            target_position: Final target position (meters)
            trajectory_duration: Duration for each trajectory (seconds)
            update_interval: Time between trajectory updates (seconds)
            method: Trajectory method ('linear' or 'polynomial')
        """
        self.target_position = target_position
        self.trajectory_duration = trajectory_duration
        self.update_interval = update_interval
        self.method = method
        
        # Current trajectory
        self.current_trajectory = None
        
        # Timing
        self.last_update_time = None
        self.trajectory_start_time = None  # Time when current trajectory started
        
    def update(self, current_position, current_time):
        """Update trajectory if update_interval has elapsed.
        
        Args:
            current_position: Current measured position (meters)
            current_time: Current time (seconds, relative to experiment start)
            
        Returns:
            bool: True if trajectory was updated, False otherwise
        """
        # Initialize on first call
        if self.last_update_time is None:
            self.last_update_time = current_time
            self.trajectory_start_time = current_time
            # Generate initial trajectory
            self.current_trajectory = generate_trajectory(
                current_position,
                self.target_position,
                self.trajectory_duration,
                method=self.method
            )
            return True
        
        # Check if update interval has elapsed
        time_since_update = current_time - self.last_update_time
        
        if time_since_update >= self.update_interval:
            # Regenerate trajectory from current position
            self.current_trajectory = generate_trajectory(
                current_position,
                self.target_position,
                self.trajectory_duration,
                method=self.method
            )
            self.last_update_time = current_time
            self.trajectory_start_time = current_time
            return True
        
        return False
    
    def get_setpoint(self, current_time):
        """Get setpoint from current trajectory at time t.
        
        The setpoint should always be ahead of current position (closer to target)
        to ensure continuous progress toward the target.
        
        Args:
            current_time: Current time (seconds, relative to experiment start)
            
        Returns:
            setpoint: Desired position from trajectory
        """
        if self.current_trajectory is None:
            return self.target_position
        
        # Time relative to current trajectory start
        trajectory_time = current_time - self.trajectory_start_time
        
        # Use a lookahead: sample the trajectory at a point that's ahead of where we are
        # This ensures we're always moving toward the target, not just following from current position
        # Lookahead = min(trajectory_duration, update_interval * 2)
        lookahead_time = min(self.trajectory_duration, self.update_interval * 2)
        sample_time = trajectory_time + lookahead_time
        
        # Clamp sample_time to trajectory duration to avoid sampling beyond end
        sample_time = min(sample_time, self.trajectory_duration)
        
        # Get position from trajectory
        if sample_time >= self.trajectory_duration:
            # At or beyond trajectory end, return target
            return self.target_position
        else:
            return self.current_trajectory.get_position(sample_time)
    
    def get_trajectory_info(self, current_time):
        """Get information about current trajectory state.
        
        Args:
            current_time: Current time (seconds)
            
        Returns:
            dict: Trajectory information
        """
        if self.current_trajectory is None:
            return {
                'active': False,
                'time_until_update': 0.0,
                'trajectory_progress': 0.0
            }
        
        trajectory_time = current_time - self.trajectory_start_time
        time_until_update = self.update_interval - (current_time - self.last_update_time)
        
        if self.current_trajectory.is_complete(trajectory_time):
            progress = 1.0
        else:
            progress = trajectory_time / self.trajectory_duration if self.trajectory_duration > 0 else 0.0
        
        return {
            'active': True,
            'time_until_update': max(0.0, time_until_update),
            'trajectory_progress': min(1.0, max(0.0, progress))
        }

