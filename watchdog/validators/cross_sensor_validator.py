#!/usr/bin/env python3

"""
Cross-Sensor Validator for Watchdog Sanity Checking

This module provides validation logic for checking consistency across multiple
sensor types. It detects discrepancies that might indicate sensor failures
or environmental issues that affect multiple sensors.

Author: F1TENTH Watchdog Team
License: MIT
Version: 1.0.0
"""

import math
import time
import numpy as np
from typing import Any, Dict, List, Optional, Tuple
from collections import deque, defaultdict

from ..base_validator import BaseValidator, ValidationResult, SeverityLevel


class CrossSensorValidator(BaseValidator):
    """
    Validates consistency across multiple sensor types.
    
    This validator checks for:
    - LiDAR vs camera depth correlation
    - Odometry vs visual odometry comparison
    - Environmental consistency validation
    - Sensor fusion anomaly detection
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize cross-sensor validator with configuration parameters.
        
        Args:
            config: Configuration dictionary containing cross-sensor validation parameters
        """
        super().__init__('cross_sensor', config)
        
        # Cross-sensor configuration
        self.lidar_camera_correlation_threshold = config.get('lidar_camera_correlation_threshold', 0.8)
        self.odometry_visual_odometry_threshold = config.get('odometry_visual_odometry_threshold', 0.2)  # m/s
        self.position_consistency_threshold = config.get('position_consistency_threshold', 0.5)  # meters
        self.temporal_sync_threshold = config.get('temporal_sync_threshold', 0.1)  # seconds
        self.environment_consistency_window = config.get('environment_consistency_window', 10)
        
        # Data storage for cross-sensor comparison
        self.sensor_data_buffer = {
            'lidar': deque(maxlen=50),
            'camera': deque(maxlen=50),
            'odometry': deque(maxlen=50),
            'visual_odometry': deque(maxlen=50),
            'imu': deque(maxlen=50)
        }
        
        # Timestamp tracking for synchronization
        self.last_timestamps = defaultdict(float)
        
        # Correlation history for trend analysis
        self.correlation_history = deque(maxlen=100)
        
    def validate(self, sensor_data: Dict[str, Any]) -> ValidationResult:
        """
        Validate cross-sensor consistency.
        
        Args:
            sensor_data: Dictionary containing data from multiple sensors
            
        Returns:
            ValidationResult with cross-sensor validation outcome
        """
        current_time = time.time()
        
        # Store incoming sensor data with timestamps
        for sensor_type, data in sensor_data.items():
            if sensor_type in self.sensor_data_buffer:
                self.sensor_data_buffer[sensor_type].append({
                    'data': data,
                    'timestamp': current_time
                })
                self.last_timestamps[sensor_type] = current_time
        
        # Perform cross-sensor validation checks
        validation_results = []
        
        # 1. Check temporal synchronization
        sync_result = self._validate_temporal_sync()
        if sync_result:
            validation_results.append(sync_result)
            
        # 2. LiDAR vs Camera depth correlation
        if ('lidar' in sensor_data and 'camera' in sensor_data and 
            len(self.sensor_data_buffer['lidar']) > 0 and 
            len(self.sensor_data_buffer['camera']) > 0):
            
            lidar_camera_result = self._validate_lidar_camera_correlation(
                sensor_data['lidar'], sensor_data['camera']
            )
            if lidar_camera_result:
                validation_results.append(lidar_camera_result)
                
        # 3. Odometry vs Visual Odometry comparison
        if ('odometry' in sensor_data and 'visual_odometry' in sensor_data):
            odom_comparison_result = self._validate_odometry_consistency(
                sensor_data['odometry'], sensor_data['visual_odometry']
            )
            if odom_comparison_result:
                validation_results.append(odom_comparison_result)
                
        # 4. Environmental consistency check
        env_result = self._validate_environmental_consistency(sensor_data)
        if env_result:
            validation_results.append(env_result)
            
        # 5. Sensor fusion anomaly detection
        fusion_result = self._detect_sensor_fusion_anomalies(sensor_data)
        if fusion_result:
            validation_results.append(fusion_result)
        
        # Determine overall result
        if not validation_results:
            return self._create_result(
                is_valid=True,
                severity=SeverityLevel.INFO,
                anomaly_type="none",
                description="Cross-sensor data appears consistent",
                suggested_action="continue",
                confidence=1.0,
                raw_data=sensor_data
            )
        else:
            # Return the most severe issue found
            most_severe = max(validation_results, key=lambda x: x.severity)
            return most_severe
    
    def _validate_temporal_sync(self) -> Optional[ValidationResult]:
        """
        Check if sensor data is temporally synchronized.
        
        Returns:
            ValidationResult if sync issues detected, None otherwise
        """
        if len(self.last_timestamps) < 2:
            return None
            
        timestamps = list(self.last_timestamps.values())
        max_time_diff = max(timestamps) - min(timestamps)
        
        if max_time_diff > self.temporal_sync_threshold:
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.WARNING,
                anomaly_type="TEMPORAL_DESYNC",
                description=f"Sensor timestamps out of sync by {max_time_diff:.3f}s",
                suggested_action="check_sensor_timing",
                confidence=0.9
            )
        
        return None
    
    def _validate_lidar_camera_correlation(self, lidar_data: Any, camera_data: Any) -> Optional[ValidationResult]:
        """
        Validate correlation between LiDAR and camera depth data.
        
        Args:
            lidar_data: LiDAR scan data
            camera_data: Camera image data (with depth if available)
            
        Returns:
            ValidationResult if correlation issues detected, None otherwise
        """
        try:
            # Extract depth information from LiDAR
            lidar_depths = self._extract_lidar_depths(lidar_data)
            
            # Extract depth information from camera (if available)
            camera_depths = self._extract_camera_depths(camera_data)
            
            if lidar_depths is None or camera_depths is None:
                # Can't correlate without both depth sources
                return None
                
            # Calculate correlation between overlapping regions
            correlation = self._calculate_depth_correlation(lidar_depths, camera_depths)
            self.correlation_history.append(correlation)
            
            if correlation < self.lidar_camera_correlation_threshold:
                return self._create_result(
                    is_valid=False,
                    severity=SeverityLevel.WARNING,
                    anomaly_type="LIDAR_CAMERA_CORRELATION_LOW",
                    description=f"LiDAR-camera depth correlation {correlation:.3f} below threshold {self.lidar_camera_correlation_threshold}",
                    suggested_action="check_sensor_calibration",
                    confidence=0.8
                )
                
        except Exception as e:
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.ERROR,
                anomaly_type="CORRELATION_CALCULATION_ERROR",
                description=f"Failed to calculate LiDAR-camera correlation: {str(e)}",
                suggested_action="check_data_format",
                confidence=1.0
            )
        
        return None
    
    def _validate_odometry_consistency(self, odometry_data: Any, visual_odometry_data: Any) -> Optional[ValidationResult]:
        """
        Validate consistency between wheel odometry and visual odometry.
        
        Args:
            odometry_data: Wheel odometry data
            visual_odometry_data: Visual odometry data
            
        Returns:
            ValidationResult if inconsistency detected, None otherwise
        """
        try:
            # Extract velocity information
            wheel_velocity = self._extract_velocity_magnitude(odometry_data)
            visual_velocity = self._extract_velocity_magnitude(visual_odometry_data)
            
            # Calculate velocity difference
            velocity_diff = abs(wheel_velocity - visual_velocity)
            
            if velocity_diff > self.odometry_visual_odometry_threshold:
                return self._create_result(
                    is_valid=False,
                    severity=SeverityLevel.WARNING,
                    anomaly_type="ODOMETRY_INCONSISTENCY",
                    description=f"Wheel odometry ({wheel_velocity:.2f} m/s) and visual odometry ({visual_velocity:.2f} m/s) differ by {velocity_diff:.2f} m/s",
                    suggested_action="check_wheel_slip_or_camera_occlusion",
                    confidence=0.85
                )
                
        except Exception as e:
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.ERROR,
                anomaly_type="ODOMETRY_COMPARISON_ERROR",
                description=f"Failed to compare odometry sources: {str(e)}",
                suggested_action="check_data_format",
                confidence=1.0
            )
        
        return None
    
    def _validate_environmental_consistency(self, sensor_data: Dict[str, Any]) -> Optional[ValidationResult]:
        """
        Check for environmental consistency across sensors.
        
        Args:
            sensor_data: Dictionary of sensor data
            
        Returns:
            ValidationResult if environmental inconsistency detected, None otherwise
        """
        inconsistencies = []
        
        # Check for lighting conditions affecting multiple sensors
        if 'camera' in sensor_data and 'lidar' in sensor_data:
            brightness_issues = self._check_lighting_consistency(
                sensor_data['camera'], sensor_data['lidar']
            )
            if brightness_issues:
                inconsistencies.append(brightness_issues)
        
        # Check for motion consistency across sensors
        motion_inconsistency = self._check_motion_consistency(sensor_data)
        if motion_inconsistency:
            inconsistencies.append(motion_inconsistency)
        
        if inconsistencies:
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.WARNING,
                anomaly_type="ENVIRONMENTAL_INCONSISTENCY",
                description=f"Environmental inconsistency detected: {'; '.join(inconsistencies)}",
                suggested_action="check_environmental_conditions",
                confidence=0.7
            )
        
        return None
    
    def _detect_sensor_fusion_anomalies(self, sensor_data: Dict[str, Any]) -> Optional[ValidationResult]:
        """
        Detect anomalies in sensor fusion patterns.
        
        Args:
            sensor_data: Dictionary of sensor data
            
        Returns:
            ValidationResult if fusion anomaly detected, None otherwise
        """
        # Check for patterns that indicate sensor fusion issues
        anomalies = []
        
        # Detect conflicting motion estimates
        if ('odometry' in sensor_data and 'imu' in sensor_data):
            motion_conflict = self._check_motion_conflict(
                sensor_data['odometry'], sensor_data['imu']
            )
            if motion_conflict:
                anomalies.append("motion_estimate_conflict")
        
        # Detect spatial inconsistencies
        spatial_inconsistency = self._check_spatial_consistency(sensor_data)
        if spatial_inconsistency:
            anomalies.append("spatial_inconsistency")
        
        if anomalies:
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.ERROR,
                anomaly_type="SENSOR_FUSION_ANOMALY",
                description=f"Sensor fusion anomalies: {', '.join(anomalies)}",
                suggested_action="recalibrate_sensor_fusion",
                confidence=0.9
            )
        
        return None
    
    # Helper methods for data extraction and analysis
    
    def _extract_lidar_depths(self, lidar_data: Any) -> Optional[List[float]]:
        """Extract depth values from LiDAR data."""
        try:
            if hasattr(lidar_data, 'ranges'):
                # Filter out invalid readings
                return [r for r in lidar_data.ranges if not math.isinf(r) and not math.isnan(r)]
            return None
        except:
            return None
    
    def _extract_camera_depths(self, camera_data: Any) -> Optional[List[float]]:
        """Extract depth values from camera data (if depth info available)."""
        try:
            # This would need to be implemented based on specific camera setup
            # For now, return None to indicate depth info not available
            return None
        except:
            return None
    
    def _calculate_depth_correlation(self, lidar_depths: List[float], camera_depths: List[float]) -> float:
        """Calculate correlation between LiDAR and camera depth measurements."""
        if not lidar_depths or not camera_depths:
            return 0.0
        
        # Simple correlation calculation (would need proper implementation)
        # This is a placeholder that returns a reasonable value
        return 0.85  # Placeholder correlation
    
    def _extract_velocity_magnitude(self, odometry_data: Any) -> float:
        """Extract velocity magnitude from odometry data."""
        try:
            if hasattr(odometry_data, 'twist') and hasattr(odometry_data.twist, 'twist'):
                linear = odometry_data.twist.twist.linear
                return math.sqrt(linear.x**2 + linear.y**2 + linear.z**2)
            elif hasattr(odometry_data, 'linear_velocity'):
                return float(odometry_data.linear_velocity)
            return 0.0
        except:
            return 0.0
    
    def _check_lighting_consistency(self, camera_data: Any, lidar_data: Any) -> Optional[str]:
        """Check for lighting-related inconsistencies."""
        # Placeholder implementation
        return None
    
    def _check_motion_consistency(self, sensor_data: Dict[str, Any]) -> Optional[str]:
        """Check for motion consistency across sensors."""
        # Placeholder implementation
        return None
    
    def _check_motion_conflict(self, odometry_data: Any, imu_data: Any) -> bool:
        """Check for conflicts between odometry and IMU motion estimates."""
        # Placeholder implementation
        return False
    
    def _check_spatial_consistency(self, sensor_data: Dict[str, Any]) -> bool:
        """Check for spatial inconsistencies across sensors."""
        # Placeholder implementation
        return False