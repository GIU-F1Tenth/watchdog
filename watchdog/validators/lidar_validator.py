#!/usr/bin/env python3

"""
LiDAR Validator for Watchdog Sanity Checking

This validator checks LiDAR sensor data for various anomalies including:
- Range violations (readings outside expected bounds)
- Noise detection (excessive variation in readings)
- Dead zones (consecutive invalid readings)
- Jump detection (sudden large changes)
- Intensity validation (if available)

Author: F1TENTH Watchdog Team
License: MIT
Version: 1.0.0
"""

import numpy as np
from typing import Any, List, Dict
from sensor_msgs.msg import LaserScan

try:
    from ..base_validator import BaseValidator, ValidationResult, SeverityLevel
except ImportError:
    import sys
    import os
    sys.path.append(os.path.dirname(os.path.dirname(__file__)))
    from base_validator import BaseValidator, ValidationResult, SeverityLevel


class LiDARValidator(BaseValidator):
    """
    Validator for LiDAR sensor data (LaserScan messages).
    
    Performs comprehensive validation of LiDAR data including range checks,
    noise detection, dead zone identification, and jump detection.
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize LiDAR validator with configuration parameters.
        
        Args:
            config: Configuration dictionary containing LiDAR validation parameters
        """
        super().__init__('lidar', config)
        
        # Load LiDAR-specific configuration
        self.min_range = config.get('lidar_min_range', 0.08)
        self.max_range = config.get('lidar_max_range', 30.0)
        self.noise_threshold = config.get('lidar_noise_threshold', 0.05)
        self.max_consecutive_invalid = config.get('lidar_max_consecutive_invalid', 5)
        self.dead_zone_threshold = config.get('lidar_dead_zone_threshold', 10)
        self.jump_threshold = config.get('lidar_jump_threshold', 2.0)
        self.intensity_min = config.get('lidar_intensity_min', 0.0)
        self.intensity_max = config.get('lidar_intensity_max', 255.0)
        
        # State tracking
        self.previous_ranges = None
        self.range_history = []
        
    def validate(self, data: LaserScan) -> ValidationResult:
        """
        Validate LiDAR scan data.
        
        Args:
            data: LaserScan message to validate
            
        Returns:
            ValidationResult with validation outcome
        """
        if not isinstance(data, LaserScan):
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.ERROR,
                anomaly_type="invalid_data_type",
                description=f"Expected LaserScan, got {type(data).__name__}",
                suggested_action="check_data_source",
                confidence=1.0
            )
        
        # Convert ranges to numpy array for easier processing
        ranges = np.array(data.ranges)
        
        # Perform all validation checks
        validation_results = []
        
        # 1. Basic range validation
        range_result = self._validate_ranges(ranges, data)
        if range_result:
            validation_results.append(range_result)
            
        # 2. Noise detection
        noise_result = self._detect_noise(ranges, data)
        if noise_result:
            validation_results.append(noise_result)
            
        # 3. Dead zone detection
        dead_zone_result = self._detect_dead_zones(ranges, data)
        if dead_zone_result:
            validation_results.append(dead_zone_result)
            
        # 4. Jump detection (if we have previous data)
        if self.previous_ranges is not None:
            jump_result = self._detect_jumps(ranges, data)
            if jump_result:
                validation_results.append(jump_result)
                
        # 5. Intensity validation (if available)
        if hasattr(data, 'intensities') and len(data.intensities) > 0:
            intensity_result = self._validate_intensities(data.intensities, data)
            if intensity_result:
                validation_results.append(intensity_result)
        
        # Store current ranges for next validation
        self.previous_ranges = ranges.copy()
        self.range_history.append(ranges)
        if len(self.range_history) > 10:  # Keep last 10 scans
            self.range_history.pop(0)
        
        # Determine overall result
        if not validation_results:
            # All checks passed
            return self._create_result(
                is_valid=True,
                severity=SeverityLevel.INFO,
                anomaly_type="none",
                description="LiDAR data appears healthy",
                suggested_action="continue",
                confidence=1.0,
                raw_data=data
            )
        else:
            # Return the most severe issue found
            most_severe = max(validation_results, key=lambda x: x.severity)
            return most_severe
    
    def _validate_ranges(self, ranges: np.ndarray, data: LaserScan) -> ValidationResult:
        """
        Validate that range readings are within expected bounds.
        
        Args:
            ranges: Array of range measurements
            data: Original LaserScan message
            
        Returns:
            ValidationResult if issues found, None otherwise
        """
        # Filter out infinite and NaN values
        valid_ranges = ranges[np.isfinite(ranges)]
        
        if len(valid_ranges) == 0:
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.CRITICAL,
                anomaly_type="no_valid_readings",
                description="No valid range readings in LiDAR scan",
                suggested_action="check_lidar_hardware",
                confidence=1.0,
                raw_data=data
            )
        
        # Check for readings outside expected range
        too_close = np.sum(valid_ranges < self.min_range)
        too_far = np.sum(valid_ranges > self.max_range)
        
        total_valid = len(valid_ranges)
        invalid_percentage = (too_close + too_far) / len(ranges) * 100
        
        if invalid_percentage > 50:  # More than 50% invalid
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.ERROR,
                anomaly_type="range_violation_excessive",
                description=f"LiDAR has {invalid_percentage:.1f}% readings outside valid range ({self.min_range}-{self.max_range}m)",
                suggested_action="check_environment_or_calibration",
                confidence=0.9,
                raw_data=data
            )
        elif invalid_percentage > 20:  # More than 20% invalid
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.WARNING,
                anomaly_type="range_violation_moderate",
                description=f"LiDAR has {invalid_percentage:.1f}% readings outside valid range",
                suggested_action="monitor_closely",
                confidence=0.8,
                raw_data=data
            )
        
        return None  # No issues found
    
    def _detect_noise(self, ranges: np.ndarray, data: LaserScan) -> ValidationResult:
        """
        Detect excessive noise in LiDAR readings.
        
        Args:
            ranges: Array of range measurements
            data: Original LaserScan message
            
        Returns:
            ValidationResult if noise detected, None otherwise
        """
        # Filter out infinite and NaN values
        valid_ranges = ranges[np.isfinite(ranges)]
        
        if len(valid_ranges) < 10:  # Need minimum data for noise analysis
            return None
            
        # Calculate local variation (difference between adjacent readings)
        differences = np.abs(np.diff(valid_ranges))
        avg_difference = np.mean(differences)
        
        if avg_difference > self.noise_threshold:
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.WARNING,
                anomaly_type="excessive_noise",
                description=f"LiDAR showing high noise level: {avg_difference:.3f}m average variation",
                suggested_action="check_vibrations_or_interference",
                confidence=0.7,
                raw_data=data
            )
        
        return None
    
    def _detect_dead_zones(self, ranges: np.ndarray, data: LaserScan) -> ValidationResult:
        """
        Detect dead zones (consecutive invalid readings).
        
        Args:
            ranges: Array of range measurements
            data: Original LaserScan message
            
        Returns:
            ValidationResult if dead zones found, None otherwise
        """
        # Find consecutive invalid readings
        invalid_mask = ~np.isfinite(ranges)
        
        # Find runs of consecutive invalid readings
        consecutive_invalid = []
        current_run = 0
        
        for is_invalid in invalid_mask:
            if is_invalid:
                current_run += 1
            else:
                if current_run > 0:
                    consecutive_invalid.append(current_run)
                current_run = 0
        
        # Check final run
        if current_run > 0:
            consecutive_invalid.append(current_run)
        
        if consecutive_invalid:
            max_consecutive = max(consecutive_invalid)
            
            if max_consecutive >= self.dead_zone_threshold:
                return self._create_result(
                    is_valid=False,
                    severity=SeverityLevel.ERROR,
                    anomaly_type="dead_zone_detected",
                    description=f"LiDAR has dead zone with {max_consecutive} consecutive invalid readings",
                    suggested_action="check_obstruction_or_hardware",
                    confidence=0.9,
                    raw_data=data
                )
        
        return None
    
    def _detect_jumps(self, ranges: np.ndarray, data: LaserScan) -> ValidationResult:
        """
        Detect sudden large changes between consecutive scans.
        
        Args:
            ranges: Current range measurements
            data: Original LaserScan message
            
        Returns:
            ValidationResult if jumps detected, None otherwise
        """
        if self.previous_ranges is None or len(self.previous_ranges) != len(ranges):
            return None
        
        # Calculate differences between current and previous scan
        valid_current = np.isfinite(ranges)
        valid_previous = np.isfinite(self.previous_ranges)
        valid_both = valid_current & valid_previous
        
        if np.sum(valid_both) < 10:  # Need minimum valid points
            return None
        
        differences = np.abs(ranges[valid_both] - self.previous_ranges[valid_both])
        large_jumps = np.sum(differences > self.jump_threshold)
        
        jump_percentage = large_jumps / np.sum(valid_both) * 100
        
        if jump_percentage > 30:  # More than 30% of readings jumped significantly
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.WARNING,
                anomaly_type="sudden_jumps",
                description=f"LiDAR showing {jump_percentage:.1f}% of readings with jumps > {self.jump_threshold}m",
                suggested_action="check_for_moving_objects_or_vibration",
                confidence=0.8,
                raw_data=data
            )
        
        return None
    
    def _validate_intensities(self, intensities: List[float], data: LaserScan) -> ValidationResult:
        """
        Validate LiDAR intensity values if available.
        
        Args:
            intensities: List of intensity values
            data: Original LaserScan message
            
        Returns:
            ValidationResult if issues found, None otherwise
        """
        if not intensities:
            return None
        
        intensities_array = np.array(intensities)
        valid_intensities = intensities_array[np.isfinite(intensities_array)]
        
        if len(valid_intensities) == 0:
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.WARNING,
                anomaly_type="no_valid_intensities",
                description="No valid intensity readings in LiDAR scan",
                suggested_action="check_intensity_calibration",
                confidence=0.6,
                raw_data=data
            )
        
        # Check intensity range
        out_of_range = np.sum((valid_intensities < self.intensity_min) | 
                             (valid_intensities > self.intensity_max))
        
        if out_of_range / len(intensities) > 0.1:  # More than 10% out of range
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.WARNING,
                anomaly_type="intensity_out_of_range",
                description=f"{out_of_range} intensity readings outside expected range ({self.intensity_min}-{self.intensity_max})",
                suggested_action="check_intensity_calibration",
                confidence=0.7,
                raw_data=data
            )
        
        return None