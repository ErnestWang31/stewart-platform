# Physics Engine for Stewart Platform Simulation
# Models ball dynamics, motor actuation, and sensor characteristics

import numpy as np
from collections import deque

class StewartPlatformPhysics:
    """
    Physics simulation for 2D Stewart platform ball balancing.
    Models:
    - Ball dynamics with friction/damping
    - Motor actuation with lag and rate limits
    - Sensor noise and delay
    """
    
    def __init__(self, config=None):
        """Initialize physics engine with configuration.
        
        Args:
            config: Configuration dictionary with simulation parameters
        """
        # Load defaults or from config
        if config is None:
            config = {}
        
        sim_config = config.get('simulation', {})
        
        # Ball physics parameters
        self.ball_mass = sim_config.get('ball_mass', 0.1)  # kg
        self.friction_coefficient = sim_config.get('friction_coefficient', 0.2)  # N*s/m (viscous damping)
        self.g = 9.81  # m/s^2
        self.platform_radius = config.get('platform_radius_m', 0.15)  # m
        
        # Motor actuation parameters
        self.motor_time_constant = sim_config.get('motor_time_constant', 0.1)  # s (first-order lag)
        self.motor_max_rate = np.deg2rad(sim_config.get('motor_max_rate_deg_per_s', 180.0))  # rad/s
        # Max tilt is 30° because with 3 motors, one can be +15° while others are -15°, giving net 30° tilt
        self.tilt_max = np.deg2rad(30.0)  # rad (actual platform limit, not individual motor limit)
        
        # Sensor parameters
        self.sensor_noise_std = sim_config.get('sensor_noise_std', 0.002)  # m
        self.sensor_delay = sim_config.get('sensor_delay_seconds', 0.1)  # s
        self.sensor_rate = sim_config.get('sensor_rate_hz', 30.0)  # Hz
        self.sensor_dt = 1.0 / self.sensor_rate
        
        # Physics integration
        self.physics_dt = sim_config.get('physics_dt', 0.002)  # s (500 Hz physics)
        
        # Initial conditions
        self.initial_x = sim_config.get('initial_position_x', -0.10)  # m
        self.initial_y = sim_config.get('initial_position_y', 0.0)  # m
        
        # Sensor delay buffer (must be initialized before reset())
        self.sensor_buffer = deque()
        self.delay_samples = max(1, int(self.sensor_delay / self.sensor_dt))
        self.next_sample_time = 0.0
        
        # State variables
        self.reset()
        
    def reset(self):
        """Reset physics state to initial conditions."""
        # Ball position and velocity
        self.x = self.initial_x
        self.y = self.initial_y
        self.vx = 0.0
        self.vy = 0.0
        
        # Platform tilt angles (actual and commanded)
        self.theta_x = 0.0  # Roll (affects x-axis)
        self.theta_y = 0.0  # Pitch (affects y-axis)
        self.theta_cmd_x = 0.0
        self.theta_cmd_y = 0.0
        
        # Time
        self.t = 0.0
        
        # Sensor buffer
        self.sensor_buffer.clear()
        self.next_sample_time = 0.0
        
    def update_motors(self, roll_angle_deg, pitch_angle_deg, dt):
        """Update motor actuation (first-order lag with rate limiting).
        
        Args:
            roll_angle_deg: Commanded roll angle (degrees)
            pitch_angle_deg: Commanded pitch angle (degrees)
            dt: Time step (seconds)
        """
        # Clip to platform limits (30° max achievable tilt with 3 motors)
        roll_angle_deg = np.clip(roll_angle_deg, -30.0, 30.0)
        pitch_angle_deg = np.clip(pitch_angle_deg, -30.0, 30.0)
        
        # Convert to radians
        theta_cmd_x = np.deg2rad(roll_angle_deg)
        theta_cmd_y = np.deg2rad(pitch_angle_deg)
        
        # Store commanded angles
        self.theta_cmd_x = theta_cmd_x
        self.theta_cmd_y = theta_cmd_y
        
        # Motor dynamics: Low-pass filter approach (matching overshoot_sim.py)
        # Modeled as: angle_new = angle_old * (1 - servo_speed) + target * servo_speed
        # This matches the overshoot_sim.py approach which shows good PID behavior
        # servo_speed = 0.15 means 15% of the way to target each step
        # Since we run physics at 500 Hz (dt=0.002) vs overshoot_sim at 30 Hz (dt=0.033),
        # we need to scale servo_speed to maintain the same time constant
        # Time constant tau ≈ dt / servo_speed
        # At 30 Hz: tau ≈ 0.033 / 0.15 ≈ 0.22s
        # At 500 Hz with same tau: servo_speed ≈ dt / tau ≈ 0.002 / 0.22 ≈ 0.009
        servo_speed_base = 0.15  # For 30 Hz (dt=0.033)
        control_dt = 1.0 / 30.0  # Control rate timestep
        # Scale to maintain same time constant
        servo_speed = servo_speed_base * (dt / control_dt)
        servo_speed = min(servo_speed, 1.0)  # Don't exceed 100%
        
        # Calculate desired change
        dtheta_x = (theta_cmd_x - self.theta_x) * servo_speed
        dtheta_y = (theta_cmd_y - self.theta_y) * servo_speed
        
        # Apply rate limiting
        max_dtheta = self.motor_max_rate * dt
        dtheta_x = np.clip(dtheta_x, -max_dtheta, max_dtheta)
        dtheta_y = np.clip(dtheta_y, -max_dtheta, max_dtheta)
        
        # Update angles
        self.theta_x += dtheta_x
        self.theta_y += dtheta_y
        
        # Saturate angles
        self.theta_x = np.clip(self.theta_x, -self.tilt_max, self.tilt_max)
        self.theta_y = np.clip(self.theta_y, -self.tilt_max, self.tilt_max)
    
    def update_ball_dynamics(self, dt):
        """Update ball position and velocity based on platform tilt.
        
        Args:
            dt: Time step (seconds)
        """
        # Ball acceleration on tilted plane
        # For a hollow sphere on a plane: a = 0.6 * g * sin(θ) (accounts for moment of inertia)
        # With friction/damping: a = 0.6*g*sin(θ) - (c/m)*v
        # The 0.6 factor comes from the moment of inertia of a hollow sphere
        K_plant = 0.6 * self.g  # Physics constant for hollow sphere
        
        ax = K_plant * np.sin(self.theta_x) - (self.friction_coefficient / self.ball_mass) * self.vx
        ay = K_plant * np.sin(self.theta_y) - (self.friction_coefficient / self.ball_mass) * self.vy
        
        # Euler integration
        self.vx += ax * dt
        self.vy += ay * dt
        self.x += self.vx * dt
        self.y += self.vy * dt
        
        # Boundary constraint: ball cannot go beyond platform radius
        r = np.hypot(self.x, self.y)
        if r > self.platform_radius:
            # Project back to circle and zero velocity (inelastic collision)
            self.x *= self.platform_radius / r
            self.y *= self.platform_radius / r
            self.vx = 0.0
            self.vy = 0.0
    
    def step(self, roll_angle_deg, pitch_angle_deg, dt=None):
        """Advance physics simulation by one time step.
        
        Args:
            roll_angle_deg: Commanded roll angle (degrees)
            pitch_angle_deg: Commanded pitch angle (degrees)
            dt: Time step (seconds). If None, uses physics_dt.
            
        Returns:
            tuple: (x, y) position in meters
        """
        if dt is None:
            dt = self.physics_dt
        
        # Update motors
        self.update_motors(roll_angle_deg, pitch_angle_deg, dt)
        
        # Update ball dynamics
        self.update_ball_dynamics(dt)
        
        # Update time
        self.t += dt
        
        return self.x, self.y
    
    def get_position(self):
        """Get current true ball position (no noise, no delay).
        
        Returns:
            tuple: (x, y) position in meters
        """
        return self.x, self.y
    
    def sample_sensor(self):
        """Sample sensor with noise and delay.
        Should be called at sensor_rate frequency.
        
        Returns:
            tuple: (x, y) measured position in meters, or None if not ready
        """
        # Check if it's time to take a new sample
        if self.t < self.next_sample_time:
            # Not time for a new sample yet, but return delayed measurement if available
            if len(self.sensor_buffer) > self.delay_samples:
                idx = len(self.sensor_buffer) - 1 - self.delay_samples
                delayed_x, delayed_y, _ = self.sensor_buffer[idx]
                return delayed_x, delayed_y
            else:
                # Not enough samples in buffer yet
                return None
        
        # Time to take a new sample
        # Add noise to true position
        noisy_x = self.x + np.random.normal(0.0, self.sensor_noise_std)
        noisy_y = self.y + np.random.normal(0.0, self.sensor_noise_std)
        
        # Add to delay buffer
        self.sensor_buffer.append((noisy_x, noisy_y, self.t))
        
        # Ensure buffer has enough samples for delay (pad with current sample if needed)
        while len(self.sensor_buffer) < self.delay_samples + 1:
            self.sensor_buffer.appendleft((noisy_x, noisy_y, self.t))
        
        # Get delayed measurement
        if len(self.sensor_buffer) > self.delay_samples:
            idx = len(self.sensor_buffer) - 1 - self.delay_samples
            delayed_x, delayed_y, _ = self.sensor_buffer[idx]
        else:
            # Not enough samples yet, return first sample (shouldn't happen due to padding above)
            delayed_x, delayed_y, _ = self.sensor_buffer[0]
        
        # Schedule next sample
        self.next_sample_time += self.sensor_dt
        
        return delayed_x, delayed_y
    
    def get_tilt_angles(self):
        """Get current platform tilt angles.
        
        Returns:
            tuple: (roll_deg, pitch_deg) in degrees
        """
        return np.rad2deg(self.theta_x), np.rad2deg(self.theta_y)
