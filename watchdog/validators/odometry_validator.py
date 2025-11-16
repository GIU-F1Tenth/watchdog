#!/usr/bin/env python3

"""
Odometry Validator for Watchdog Sanity Checking

This validator checks odometry data for various anomalies including:
- Velocity limit validation (linear and angular)
- Acceleration limit validation
- Position jump detection
- Velocity smoothing and consistency
- Physical constraint validation

Author: F1TENTH Watchdog Team
License: MIT  
Version: 1.0.0
"""

import time
import math
from typing import Any, List, Dict, Optional
from collections import deque
from nav_msgs.msg import Odometry

try:
    from ..base_validator import BaseValidator, ValidationResult, SeverityLevel
except ImportError:
    import sys
    import os
    sys.path.append(os.path.dirname(os.path.dirname(__file__)))
    from base_validator import BaseValidator, ValidationResult, SeverityLevel


class OdometryValidator(BaseValidator):
    """
    Validator for odometry sensor data (Odometry messages).
    
    Performs comprehensive validation of odometry data including velocity limits,
    acceleration constraints, position consistency, and physical feasibility.
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize odometry validator with configuration parameters.
        
        Args:
            config: Configuration dictionary containing odometry validation parameters
        """
        super().__init__('odometry', config)
        
        # Load odometry-specific configuration
        self.max_linear_velocity = config.get('max_linear_velocity', 10.0)  # m/s
        self.max_angular_velocity = config.get('max_angular_velocity', 5.0)  # rad/s
        self.max_linear_acceleration = config.get('max_linear_acceleration', 8.0)  # m/s²
        self.max_angular_acceleration = config.get('max_angular_acceleration', 10.0)  # rad/s²
        self.velocity_smoothing_window = config.get('odom_velocity_smoothing_window', 5)
        self.position_jump_threshold = config.get('odom_position_jump_threshold', 1.0)  # meters
        
        # State tracking
        self.velocity_history = deque(maxlen=self.velocity_smoothing_window)
        self.angular_velocity_history = deque(maxlen=self.velocity_smoothing_window)
        self.position_history = deque(maxlen=10)
        self.timestamp_history = deque(maxlen=10)
        
        # Previous values for acceleration/jump detection
        self.previous_linear_vel = None
        self.previous_angular_vel = None
        self.previous_position = None
        self.previous_timestamp = None
        
    def validate(self, data: Odometry) -> ValidationResult:
        """
        Validate odometry data.
        
        Args:
            data: Odometry message to validate
            
        Returns:
            ValidationResult with validation outcome
        """
        if not isinstance(data, Odometry):
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.ERROR,
                anomaly_type="invalid_data_type",
                description=f"Expected Odometry, got {type(data).__name__}",
                suggested_action="check_data_source",
                confidence=1.0
            )
        
        current_time = time.time()
        
        # Extract data from odometry message
        try:
            linear_vel = self._extract_linear_velocity(data)
            angular_vel = self._extract_angular_velocity(data)
            position = self._extract_position(data)
        except Exception as e:
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.ERROR,
                anomaly_type="data_extraction_error",
                description=f"Failed to extract odometry data: {str(e)}",
                suggested_action="check_data_format",
                confidence=1.0
            )
        
        # Update history
        self.velocity_history.append(linear_vel)
        self.angular_velocity_history.append(angular_vel)
        self.position_history.append(position)
        self.timestamp_history.append(current_time)
        
        # Perform validation checks
        validation_results = []
        
        # 1. Velocity limit validation
        velocity_result = self._validate_velocity_limits(linear_vel, angular_vel, data)
        if velocity_result:
            validation_results.append(velocity_result)
            
        # 2. Acceleration validation (if we have previous data)
        if (self.previous_linear_vel is not None and 
            self.previous_timestamp is not None):
            
            accel_result = self._validate_accelerations(linear_vel, angular_vel, 
                                                      current_time, data)
            if accel_result:
                validation_results.append(accel_result)
                
        # 3. Position jump detection
        if self.previous_position is not None:
            jump_result = self._detect_position_jumps(position, current_time, data)
            if jump_result:
                validation_results.append(jump_result)
                
        # 4. Velocity consistency check
        if len(self.velocity_history) >= 3:
            consistency_result = self._check_velocity_consistency(data)
            if consistency_result:
                validation_results.append(consistency_result)
                
        # 5. Physical feasibility check
        feasibility_result = self._check_physical_feasibility(linear_vel, angular_vel, data)
        if feasibility_result:
            validation_results.append(feasibility_result)
        
        # Store current values for next validation
        self.previous_linear_vel = linear_vel
        self.previous_angular_vel = angular_vel
        self.previous_position = position
        self.previous_timestamp = current_time
        
        # Determine overall result
        if not validation_results:
            return self._create_result(
                is_valid=True,
                severity=SeverityLevel.INFO,
                anomaly_type="none",
                description="Odometry data appears healthy",
                suggested_action="continue",
                confidence=1.0,
                raw_data=data
            )
        else:
            # Return the most severe issue found
            most_severe = max(validation_results, key=lambda x: x.severity)
            return most_severe
    
    def _extract_linear_velocity(self, data: Odometry) -> float:
        """Extract linear velocity magnitude from odometry data."""
        twist = data.twist.twist
        linear_vel = math.sqrt(twist.linear.x**2 + twist.linear.y**2 + twist.linear.z**2)
        return linear_vel
    
    def _extract_angular_velocity(self, data: Odometry) -> float:
        """Extract angular velocity magnitude from odometry data."""
        twist = data.twist.twist
        angular_vel = abs(twist.angular.z)  # Assuming 2D motion, only z-axis rotation
        return angular_vel
    
    def _extract_position(self, data: Odometry) -> tuple:
        """Extract position (x, y) from odometry data."""
        pose = data.pose.pose
        return (pose.position.x, pose.position.y)
    
    def _validate_velocity_limits(self, linear_vel: float, angular_vel: float, 
                                 data: Odometry) -> Optional[ValidationResult]:
        """
        Validate that velocities are within expected limits.
        
        Args:
            linear_vel: Linear velocity magnitude
            angular_vel: Angular velocity magnitude
            data: Original Odometry message
            
        Returns:
            ValidationResult if limits exceeded, None otherwise
        """
        # Check linear velocity limit
        if linear_vel > self.max_linear_velocity:
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.ERROR,
                anomaly_type="linear_velocity_excessive",
                description=f"Linear velocity {linear_vel:.2f}m/s exceeds maximum {self.max_linear_velocity}m/s",
                suggested_action="check_velocity_commands_or_sensors",
                confidence=0.9,
                raw_data=data
            )
        
        # Check angular velocity limit
        if angular_vel > self.max_angular_velocity:
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.ERROR,
                anomaly_type="angular_velocity_excessive",
                description=f"Angular velocity {angular_vel:.2f}rad/s exceeds maximum {self.max_angular_velocity}rad/s",
                suggested_action="check_steering_commands_or_sensors",
                confidence=0.9,
                raw_data=data
            )
        
        return None
    
    def _validate_accelerations(self, linear_vel: float, angular_vel: float,
                               current_time: float, data: Odometry) -> Optional[ValidationResult]:
        """
        Validate acceleration limits.
        
        Args:
            linear_vel: Current linear velocity
            angular_vel: Current angular velocity
            current_time: Current timestamp
            data: Original Odometry message
            
        Returns:
            ValidationResult if acceleration limits exceeded, None otherwise
        """
        time_delta = current_time - self.previous_timestamp
        if time_delta <= 0 or time_delta > 1.0:  # Skip if time delta is invalid or too large
            return None
        
        # Calculate accelerations
        linear_accel = abs(linear_vel - self.previous_linear_vel) / time_delta
        angular_accel = abs(angular_vel - self.previous_angular_vel) / time_delta
        
        # Check linear acceleration
        if linear_accel > self.max_linear_acceleration:
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.WARNING,
                anomaly_type="linear_acceleration_excessive",
                description=f"Linear acceleration {linear_accel:.2f}m/s² exceeds maximum {self.max_linear_acceleration}m/s²",
                suggested_action="check_for_sudden_movements_or_obstacles",
                confidence=0.8,
                raw_data=data
            )
        
        # Check angular acceleration
        if angular_accel > self.max_angular_acceleration:
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.WARNING,
                anomaly_type="angular_acceleration_excessive",
                description=f"Angular acceleration {angular_accel:.2f}rad/s² exceeds maximum {self.max_angular_acceleration}rad/s²",
                suggested_action="check_steering_system_or_commands",
                confidence=0.8,
                raw_data=data
            )
        
        return None
    
    def _detect_position_jumps(self, position: tuple, current_time: float,
                              data: Odometry) -> Optional[ValidationResult]:
        """
        Detect unrealistic position jumps.
        
        Args:
            position: Current position (x, y)
            current_time: Current timestamp
            data: Original Odometry message
            
        Returns:
            ValidationResult if position jump detected, None otherwise
        """
        if self.previous_position is None or self.previous_timestamp is None:
            return None
            
        time_delta = current_time - self.previous_timestamp
        if time_delta <= 0 or time_delta > 1.0:  # Skip if time delta is invalid
            return None
        
        # Calculate position change
        dx = position[0] - self.previous_position[0]
        dy = position[1] - self.previous_position[1]
        distance_change = math.sqrt(dx*dx + dy*dy)
        
        # Calculate implied velocity
        implied_velocity = distance_change / time_delta
        
        # Check for position jumps
        if distance_change > self.position_jump_threshold:
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.WARNING,
                anomaly_type="position_jump",
                description=f"Large position jump: {distance_change:.3f}m in {time_delta:.3f}s (implied velocity: {implied_velocity:.2f}m/s)",
                suggested_action="check_odometry_sensors_or_calibration",
                confidence=0.9,
                raw_data=data
            )
        
        return None
    
    def _check_velocity_consistency(self, data: Odometry) -> Optional[ValidationResult]:
        """
        Check velocity consistency using moving average.
        
        Args:
            data: Original Odometry message
            
        Returns:
            ValidationResult if inconsistency detected, None otherwise
        """
        if len(self.velocity_history) < 3:
            return None
        
        velocity_list = list(self.velocity_history)
        current_vel = velocity_list[-1]
        
        # Check if current velocity is an outlier
        if self._is_outlier(current_vel, velocity_list[:-1], sigma=2.5):
            moving_avg = self._calculate_moving_average(velocity_list[:-1])
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.WARNING,
                anomaly_type="velocity_inconsistent",
                description=f"Velocity {current_vel:.2f}m/s inconsistent with recent average {moving_avg:.2f}m/s",
                suggested_action="check_velocity_sensor_consistency",
                confidence=0.7,
                raw_data=data
            )
        
        return None
    
    def _check_physical_feasibility(self, linear_vel: float, angular_vel: float,
                                   data: Odometry) -> Optional[ValidationResult]:
        """
        Check if velocity combination is physically feasible for the vehicle.
        
        Args:
            linear_vel: Linear velocity
            angular_vel: Angular velocity
            data: Original Odometry message
            
        Returns:
            ValidationResult if infeasible combination detected, None otherwise
        """
        # For a car-like vehicle, there are physical constraints on the relationship
        # between linear and angular velocity based on wheelbase and maximum steering angle
        
        # Simple check: if we're turning (angular velocity > 0), we should have some forward motion
        # unless we're doing a zero-radius turn (which most F1TENTH cars can't do)
        
        if angular_vel > 0.5 and linear_vel < 0.1:  # Turning but no forward motion
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.WARNING,
                anomaly_type="infeasible_motion",
                description=f"High angular velocity ({angular_vel:.2f}rad/s) with minimal linear velocity ({linear_vel:.2f}m/s)",
                suggested_action="check_motion_model_or_sensors",
                confidence=0.6,
                raw_data=data
            )
        
        # Check for simultaneous high linear and angular velocities that might be unrealistic
        if linear_vel > self.max_linear_velocity * 0.8 and angular_vel > self.max_angular_velocity * 0.8:
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.WARNING,
                anomaly_type="high_combined_velocity",
                description=f"High combined velocities: linear {linear_vel:.2f}m/s, angular {angular_vel:.2f}rad/s",
                suggested_action="verify_motion_is_safe",
                confidence=0.7,
                raw_data=data
            )
        
        return None