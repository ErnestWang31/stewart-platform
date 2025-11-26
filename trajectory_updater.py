# Trajectory Updater Module
# Regenerates trajectory periodically based on current measured position

from trajectory_generator import generate_trajectory, Trajectory

class TrajectoryUpdater:
    """Manages trajectory regeneration every Δt based on current position."""
    
    def __init__(self, target_position, trajectory_duration, update_interval, method='linear',
                 use_remaining_time=False, total_trajectory_duration=None):
        """Initialize trajectory updater.
        
        Args:
            target_position: Final target position (meters)
            trajectory_duration: Duration for each trajectory segment (seconds)
            update_interval: Time between trajectory updates (seconds)
            method: Trajectory method ('linear' or 'polynomial')
            use_remaining_time: If True, each trajectory uses remaining time to target
                               If False, each trajectory uses fixed trajectory_duration
            total_trajectory_duration: Total time to reach target (required if use_remaining_time=True)
        """
        self.target_position = target_position
        self.trajectory_duration = trajectory_duration
        self.update_interval = update_interval
        self.method = method
        self.use_remaining_time = use_remaining_time
        self.total_trajectory_duration = total_trajectory_duration
        
        if use_remaining_time and total_trajectory_duration is None:
            raise ValueError("total_trajectory_duration must be provided when use_remaining_time=True")
        
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
            tuple: (updated: bool, trajectory_info: dict or None)
                trajectory_info contains: x0, xf, start_time, duration, method
        """
        # Calculate trajectory duration for this update
        if self.use_remaining_time:
            # Use remaining time to reach target
            remaining_time = max(0.01, self.total_trajectory_duration - current_time)  # Min 10ms to avoid division by zero
            segment_duration = remaining_time
        else:
            # Use fixed duration for each segment
            segment_duration = self.trajectory_duration
        
        # Initialize on first call
        if self.last_update_time is None:
            self.last_update_time = current_time
            self.trajectory_start_time = current_time
            # Generate initial trajectory
            self.current_trajectory = generate_trajectory(
                current_position,
                self.target_position,
                segment_duration,
                method=self.method
            )
            return True, {
                'x0': current_position,
                'xf': self.target_position,
                'start_time': current_time,
                'duration': segment_duration,
                'method': self.method
            }
        
        # Check if update interval has elapsed
        time_since_update = current_time - self.last_update_time
        
        if time_since_update >= self.update_interval:
            # Regenerate trajectory from current position
            self.current_trajectory = generate_trajectory(
                current_position,
                self.target_position,
                segment_duration,
                method=self.method
            )
            self.last_update_time = current_time
            self.trajectory_start_time = current_time
            return True, {
                'x0': current_position,
                'xf': self.target_position,
                'start_time': current_time,
                'duration': segment_duration,
                'method': self.method
            }
        
        return False, None
    
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
        
        # Get the actual duration of the current trajectory
        # This may differ from self.trajectory_duration if use_remaining_time is True
        if hasattr(self.current_trajectory, 'T'):
            current_traj_duration = self.current_trajectory.T
        else:
            current_traj_duration = self.trajectory_duration
        
        # Time relative to current trajectory start
        trajectory_time = current_time - self.trajectory_start_time
        
        # Use a lookahead: sample the trajectory at a point that's ahead of where we are
        # This ensures we're always moving toward the target, not just following from current position
        # Lookahead = min(trajectory_duration, update_interval * 2)
        lookahead_time = min(current_traj_duration, self.update_interval * 2)
        sample_time = trajectory_time + lookahead_time
        
        # Clamp sample_time to trajectory duration to avoid sampling beyond end
        sample_time = min(sample_time, current_traj_duration)
        
        # Get position from trajectory
        if sample_time >= current_traj_duration:
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
        
        # Get the actual duration of the current trajectory
        if hasattr(self.current_trajectory, 'T'):
            current_traj_duration = self.current_trajectory.T
        else:
            current_traj_duration = self.trajectory_duration
        
        trajectory_time = current_time - self.trajectory_start_time
        time_until_update = self.update_interval - (current_time - self.last_update_time)
        
        if self.current_trajectory.is_complete(trajectory_time):
            progress = 1.0
        else:
            progress = trajectory_time / current_traj_duration if current_traj_duration > 0 else 0.0
        
        return {
            'active': True,
            'time_until_update': max(0.0, time_until_update),
            'trajectory_progress': min(1.0, max(0.0, progress))
        }

