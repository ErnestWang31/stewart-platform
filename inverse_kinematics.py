# Inverse Kinematics Module for Stewart Platform
# Based on SPV4.py - converts platform orientation (roll/pitch) to motor angles
# Adapted for circular platform with 3 motors

import numpy as np
from scipy.optimize import fsolve

class StewartPlatformIK:
    """Inverse kinematics solver for Stewart platform with 3 motors."""
    
    def __init__(self, config=None):
        """Initialize inverse kinematics solver with platform geometry.
        
        Args:
            config: Configuration dictionary with platform geometry parameters
        """
        # Platform geometry parameters (from SPV4.py)
        # These should match your physical platform dimensions
        # Default values - should be calibrated/configured
        self.l = config.get('platform', {}).get('triangle_side_length', 10.0) if config else 10.0
        
        # Motor 1 parameters (at 90°)
        self.l_1 = config.get('platform', {}).get('motor1_base_length', 10.0) if config else 10.0
        self.l_11 = config.get('platform', {}).get('motor1_link1_length', 8.0) if config else 8.0
        self.l_12 = config.get('platform', {}).get('motor1_link2_length', 8.0) if config else 8.0
        
        # Motor 2 parameters (at 210°)
        self.l_2 = config.get('platform', {}).get('motor2_base_length', 10.0) if config else 10.0
        self.l_21 = config.get('platform', {}).get('motor2_link1_length', 8.0) if config else 8.0
        self.l_22 = config.get('platform', {}).get('motor2_link2_length', 8.0) if config else 8.0
        
        # Motor 3 parameters (at 330°)
        self.l_3 = config.get('platform', {}).get('motor3_base_length', 10.0) if config else 10.0
        self.l_31 = config.get('platform', {}).get('motor3_link1_length', 8.0) if config else 8.0
        self.l_32 = config.get('platform', {}).get('motor3_link2_length', 8.0) if config else 8.0
        
        # Platform center position (Z coordinate)
        self.S_z = config.get('platform', {}).get('platform_center_z', 17.05) if config else 17.05
        self.S = np.array([0, 0, self.S_z])
        
        # Initial guess for solver
        self.initial_guess = 5.0
    
    def roll_pitch_to_normal(self, roll_deg, pitch_deg):
        """Convert roll and pitch angles to platform normal vector.
        
        Args:
            roll_deg: Roll angle in degrees (rotation around X axis, positive = tilt right)
            pitch_deg: Pitch angle in degrees (rotation around Y axis, positive = tilt forward)
            
        Returns:
            nrm: Normal vector [nx, ny, nz] representing platform orientation
        """
        roll_rad = np.radians(roll_deg)
        pitch_rad = np.radians(pitch_deg)
        
        # Create rotation matrices
        # Roll rotation around X axis
        R_roll = np.array([
            [1, 0, 0],
            [0, np.cos(roll_rad), -np.sin(roll_rad)],
            [0, np.sin(roll_rad), np.cos(roll_rad)]
        ])
        
        # Pitch rotation around Y axis
        R_pitch = np.array([
            [np.cos(pitch_rad), 0, np.sin(pitch_rad)],
            [0, 1, 0],
            [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]
        ])
        
        # Initial normal vector (pointing up)
        nrm_initial = np.array([0, 0, 1])
        
        # Apply rotations: first roll, then pitch
        nrm = R_pitch @ R_roll @ nrm_initial
        
        return nrm
    
    def calculate_vectors_and_angles_1(self, l, l_1, l_11, l_12, x1, x2, x3):
        """Calculate motor 1 angles from platform vertex positions."""
        AA_tmp = l_12**2 - (l_1 - x1[1])**2 - l_11**2 - x1[2]**2
        bb_tmp = (2 * (l_1 - x1[1]) * l_11)
        cc_tmp = (2 * x1[2] * l_11)
        denom_AA = np.sqrt(bb_tmp**2 + cc_tmp**2)
        cc = np.arccos(np.clip(AA_tmp / denom_AA, -1, 1))
        bb1 = np.arccos(np.clip(bb_tmp / denom_AA, -1, 1))
        theta_11 = cc - bb1
        theta_12 = np.arcsin(np.clip((x1[2] - l_11 * np.sin(theta_11)) / l_12, -1, 1))
        return {
            "theta_11": theta_11 * 180 / np.pi,
            "theta_12": theta_12 * 180 / np.pi,
        }
    
    def calculate_vectors_and_angles_2(self, l, l_2, l_21, l_22, x1, x2, x3):
        """Calculate motor 2 angles from platform vertex positions."""
        y_tmp = np.sqrt(x2[0]**2 + x2[1]**2)
        z_tmp = x2[2]
        AA_tmp = -(l_22**2 - (l_2 - y_tmp)**2 - l_21**2 - z_tmp**2)
        bb_tmp = (2 * (l_2 - y_tmp) * l_21)
        cc_tmp = (2 * z_tmp * l_21)
        denom_AA = np.sqrt(bb_tmp**2 + cc_tmp**2)
        cc = np.arcsin(np.clip(AA_tmp / denom_AA, -1, 1))
        bb1 = np.arcsin(np.clip(bb_tmp / denom_AA, -1, 1))
        theta_21 = cc + bb1
        theta_22 = np.arcsin(np.clip((x2[2] - l_21 * np.sin(theta_21)) / l_22, -1, 1))
        return {
            "theta_21": theta_21 * 180 / np.pi,
            "theta_22": theta_22 * 180 / np.pi,
        }
    
    def calculate_vectors_and_angles_3(self, l, l_3, l_31, l_32, x1, x2, x3):
        """Calculate motor 3 angles from platform vertex positions."""
        y_tmp = np.sqrt(x3[0]**2 + x3[1]**2)
        z_tmp = x3[2]
        AA_tmp = l_32**2 - (l_3 - y_tmp)**2 - l_31**2 - z_tmp**2
        bb_tmp = (2 * (l_3 - y_tmp) * l_31)
        cc_tmp = (2 * z_tmp * l_31)
        denom_AA = np.sqrt(bb_tmp**2 + cc_tmp**2)
        cc = np.arccos(np.clip(AA_tmp / denom_AA, -1, 1))
        bb2 = np.arcsin(np.clip(cc_tmp / denom_AA, -1, 1))
        theta_31 = cc - bb2
        theta_32 = np.arcsin(np.clip((z_tmp - l_31 * np.sin(theta_31)) / l_32, -1, 1))
        return {
            "theta_31": theta_31 * 180 / np.pi,
            "theta_32": theta_32 * 180 / np.pi,
        }
    
    def position_and_orientation(self, nrm, S):
        """Calculate platform vertex positions from normal vector."""
        vector_i = np.array([1, 0, 0])
        cross_product = np.cross(nrm, vector_i)
        anorm = np.linalg.norm(cross_product)
        if anorm < 1e-6:
            # Handle degenerate case (nrm parallel to [1,0,0])
            vector_i = np.array([0, 1, 0])
            cross_product = np.cross(nrm, vector_i)
            anorm = np.linalg.norm(cross_product)
        v_hat = cross_product / anorm
        a = self.l / (2 * np.cos(30 * np.pi / 180))
        cross_product = np.cross(v_hat, nrm)
        anorm = np.linalg.norm(cross_product)
        u_hat = cross_product / anorm
        w_hat = np.cross(u_hat, v_hat)
        x1 = S + a * v_hat
        x2 = S - a * np.sin(30 * np.pi / 180.0) * v_hat + a * np.cos(30 * np.pi / 180.) * u_hat
        x3 = S - a * np.sin(30 * np.pi / 180.0) * v_hat - a * np.cos(30 * np.pi / 180.) * u_hat
        return {
            "x1": x1,
            "x2": x2,
            "x3": x3,
        }
    
    def eeq1(self, d_1, nrm1):
        """Equation 1 for solving platform orientation."""
        n_x, n_y, n_z = nrm1[0], nrm1[1], nrm1[2]
        if abs(n_z) < 1e-6:
            return 1e6  # Avoid division by zero
        
        A3 = 1 + (-n_x * np.sqrt(3) / 2 - n_y / 2) ** 2 / n_z ** 2
        BBB = d_1 - 2 * n_y * d_1 * (-n_x * np.sqrt(3) / 2 - n_y / 2) / n_z ** 2
        CCC = d_1 ** 2 + n_y ** 2 * d_1 ** 2 / n_z ** 2 - self.l ** 2
        
        A2 = 1 + (n_x * np.sqrt(3) / 2 - n_y / 2) ** 2 / n_z ** 2
        BB = d_1 - 2 * n_y * d_1 * (n_x * np.sqrt(3) / 2 - n_y / 2) / n_z ** 2
        CC = d_1 ** 2 + n_y ** 2 * d_1 ** 2 / n_z ** 2 - self.l ** 2
        
        s2 = np.sqrt(max(0, -4 * A2 * CC + BB ** 2))
        s3 = np.sqrt(max(0, -4 * A3 * CCC + BBB ** 2))
        
        eq1 = (n_x * n_y * ((-BBB + s3) * A2 + A3 * (BB - s2)) * ((-BBB + s3) * A2 - A3 * (BB - s2)) * np.sqrt(3) + 
               2 * (-4 * self.l ** 2 * n_z ** 2 * A3 ** 2 + (-BBB + s3) ** 2 * (n_z ** 2 + 0.75 * n_x ** 2 + n_y ** 2 / 4)) * A2 ** 2 - 
               2 * (BB - s2) * (n_z ** 2 + 1.5 * n_x ** 2 - n_y ** 2 / 2) * (-BBB + s3) * A3 * A2 + 
               2 * (BB - s2) ** 2 * A3 ** 2 * (n_z ** 2 + 0.75 * n_x ** 2 + n_y ** 2 / 4)) / n_z ** 2 / A2 ** 2 / A3 ** 2 / 8
        return eq1
    
    def triangle_orientation_and_location(self, nrm1, S):
        """Calculate motor angles from platform normal vector using inverse kinematics."""
        try:
            root = fsolve(self.eeq1, self.initial_guess, args=(nrm1,))
            d_1 = root[0]
            
            n_x, n_y, n_z = nrm1[0], nrm1[1], nrm1[2]
            if abs(n_z) < 1e-6:
                raise ValueError("Platform normal vector too close to horizontal")
            
            A3 = 1 + (-n_x * np.sqrt(3) / 2 - n_y / 2) ** 2 / n_z ** 2
            BBB = d_1 - 2 * n_y * d_1 * (-n_x * np.sqrt(3) / 2 - n_y / 2) / n_z ** 2
            CCC = d_1 ** 2 + n_y ** 2 * d_1 ** 2 / n_z ** 2 - self.l ** 2
            
            A2 = 1 + (n_x * np.sqrt(3) / 2 - n_y / 2) ** 2 / n_z ** 2
            BB = d_1 - 2 * n_y * d_1 * (n_x * np.sqrt(3) / 2 - n_y / 2) / n_z ** 2
            CC = d_1 ** 2 + n_y ** 2 * d_1 ** 2 / n_z ** 2 - self.l ** 2
            
            s2 = np.sqrt(max(0, -4 * A2 * CC + BB ** 2))
            s3 = np.sqrt(max(0, -4 * A3 * CCC + BBB ** 2))
            
            d_2 = (-BB + s2) / (2 * A2)
            d_3 = (-BBB + s3) / (2 * A3)
            c_1 = 12
            c_2 = c_1 + (1 / n_z) * (-d_2 * np.cos(30 * np.pi / 180) * n_x + d_2 * np.sin(30 * np.pi / 180) * n_y + d_1 * n_y)
            c_3 = c_2 + (1 / n_z) * ((d_2 + d_3) * np.cos(30 * np.pi / 180) * n_x - (-d_3 + d_2) * np.sin(30 * np.pi / 180) * n_y)
            
            X1_hat = np.array([0, 1, 0])
            X2_hat = np.array([np.cos(30 * np.pi / 180), -np.sin(30 * np.pi / 180), 0])
            X3_hat = np.array([-np.cos(30 * np.pi / 180), -np.sin(30 * np.pi / 180), 0])
            z_hat = np.array([0, 0, 1])
            
            P1 = d_1 * X1_hat + c_1 * z_hat
            P2 = d_2 * X2_hat + c_2 * z_hat
            P3 = d_3 * X3_hat + c_3 * z_hat
            
            pp = (P1 + P2 + P3) / 3 - S
            P1 = P1 - pp
            P2 = P2 - pp
            P3 = P3 - pp
            
            result1 = self.calculate_vectors_and_angles_1(self.l, self.l_1, self.l_11, self.l_12, P1, P2, P3)
            result2 = self.calculate_vectors_and_angles_2(self.l, self.l_2, self.l_21, self.l_22, P1, P2, P3)
            result3 = self.calculate_vectors_and_angles_3(self.l, self.l_3, self.l_31, self.l_32, P1, P2, P3)
            
            return {
                "theta_11": result1["theta_11"],
                "theta_12": result1["theta_12"],
                "theta_21": result2["theta_21"],
                "theta_22": result2["theta_22"],
                "theta_31": result3["theta_31"],
                "theta_32": result3["theta_32"]
            }
        except Exception as e:
            print(f"[IK] Error in inverse kinematics: {e}")
            # Return neutral angles on error
            return {
                "theta_11": 0.0,
                "theta_12": 0.0,
                "theta_21": 0.0,
                "theta_22": 0.0,
                "theta_31": 0.0,
                "theta_32": 0.0
            }
    
    def solve(self, roll_deg, pitch_deg):
        """Solve inverse kinematics: convert roll/pitch to motor angles.
        
        Args:
            roll_deg: Roll angle in degrees
            pitch_deg: Pitch angle in degrees
            
        Returns:
            dict: Motor angles in degrees
                - theta_11, theta_21, theta_31: First joint angles for motors 1, 2, 3
                - theta_12, theta_22, theta_32: Second joint angles for motors 1, 2, 3
        """
        # Convert roll/pitch to normal vector
        nrm = self.roll_pitch_to_normal(roll_deg, pitch_deg)
        
        # Solve inverse kinematics
        result = self.triangle_orientation_and_location(nrm, self.S)
        
        return result
    
    def get_motor_angles(self, roll_deg, pitch_deg):
        """Get primary motor angles (theta_11, theta_21, theta_31) for servo control.
        
        Args:
            roll_deg: Roll angle in degrees
            pitch_deg: Pitch angle in degrees
            
        Returns:
            tuple: (theta_11, theta_21, theta_31) in degrees
        """
        result = self.solve(roll_deg, pitch_deg)
        return result["theta_11"], result["theta_21"], result["theta_31"]

