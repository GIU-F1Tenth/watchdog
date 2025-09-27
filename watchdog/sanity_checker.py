#!/usr/bin/env python3

"""
SanityChecker - Main orchestrator for watchdog sanity checking

This module provides the main SanityChecker class that coordinates all
sensor validators, processes validation results, and manages health tracking.

Author: F1TENTH Watchdog Team
License: MIT
Version: 1.0.0
"""

import time
from typing import Dict, List, Any, Optional, Tuple
from collections import defaultdict, deque
import rclpy
from rclpy.node import Node

try:
    from .base_validator import BaseValidator, ValidationResult, SeverityLevel
except ImportError:
    from base_validator import BaseValidator, ValidationResult, SeverityLevel


class SensorHealthTracker:
    """
    Tracks health scores and statistics for individual sensors.
    """
    
    def __init__(self, sensor_name: str, history_size: int = 100):
        """
        Initialize health tracker for a sensor.
        
        Args:
            sensor_name: Name of the sensor to track
            history_size: Number of health scores to keep in history
        """
        self.sensor_name = sensor_name
        self.health_history = deque(maxlen=history_size)
        self.anomaly_counts = defaultdict(int)
        self.last_update = time.time()
        self.is_valid = True
        
    def update_health(self, health_score: float, active_anomalies: List[str]) -> None:
        """
        Update health tracking information.
        
        Args:
            health_score: Current health score (0.0-1.0)
            active_anomalies: List of currently active anomalies
        """
        self.health_history.append(health_score)
        
        # Update anomaly counts
        for anomaly in active_anomalies:
            self.anomaly_counts[anomaly] += 1
            
        # Update validity based on health score
        self.is_valid = health_score > 0.5  # Threshold for "valid" sensor
        self.last_update = time.time()
        
    def get_average_health(self, window_size: int = 10) -> float:
        """
        Get average health score over recent window.
        
        Args:
            window_size: Number of recent scores to average
            
        Returns:
            Average health score
        """
        if not self.health_history:
            return 1.0
            
        recent_scores = list(self.health_history)[-window_size:]
        return sum(recent_scores) / len(recent_scores)
        
    def get_health_trend(self) -> str:
        """
        Determine if health is improving, declining, or stable.
        
        Returns:
            'improving', 'declining', or 'stable'
        """
        if len(self.health_history) < 5:
            return 'stable'
            
        recent_scores = list(self.health_history)[-10:]
        first_half = recent_scores[:len(recent_scores)//2]
        second_half = recent_scores[len(recent_scores)//2:]
        
        first_avg = sum(first_half) / len(first_half)
        second_avg = sum(second_half) / len(second_half)
        
        if second_avg > first_avg + 0.1:
            return 'improving'
        elif second_avg < first_avg - 0.1:
            return 'declining'
        else:
            return 'stable'


class SanityChecker:
    """
    Main sanity checker class that orchestrates all validation operations.
    
    This class manages multiple validators, processes their results, and
    maintains overall system health tracking.
    """
    
    def __init__(self, node: Node, config: Dict[str, Any]):
        """
        Initialize the sanity checker.
        
        Args:
            node: ROS2 node instance for logging
            config: Configuration parameters
        """
        self.node = node
        self.config = config
        self.enabled = config.get('sanity_checks_enabled', True)
        
        # Validator management
        self.validators: Dict[str, BaseValidator] = {}
        self.health_trackers: Dict[str, SensorHealthTracker] = {}
        
        # Warning management
        self.warning_history = deque(maxlen=config.get('warning_history_size', 100))
        self.active_warnings: Dict[str, List[ValidationResult]] = defaultdict(list)
        
        # Statistics
        self.total_validations = 0
        self.total_warnings = 0
        self.start_time = time.time()
        
        # Thresholds
        self.outlier_sigma = config.get('outlier_detection_sigma', 2.0)
        self.health_update_interval = config.get('health_update_interval', 1.0)
        self.last_health_update = 0.0
        
        self.node.get_logger().info("SanityChecker initialized")
        
    def register_validator(self, validator: BaseValidator) -> None:
        """
        Register a new validator with the sanity checker.
        
        Args:
            validator: Validator instance to register
        """
        sensor_name = validator.sensor_name
        self.validators[sensor_name] = validator
        self.health_trackers[sensor_name] = SensorHealthTracker(sensor_name)
        
        self.node.get_logger().info(f"Registered validator for {sensor_name}")
        
    def validate_sensor_data(self, sensor_name: str, data: Any) -> Optional[ValidationResult]:
        """
        Validate data from a specific sensor.
        
        Args:
            sensor_name: Name of the sensor
            data: Sensor data to validate
            
        Returns:
            ValidationResult if validation performed, None if validator not found/disabled
        """
        if not self.enabled:
            return None
            
        validator = self.validators.get(sensor_name)
        if validator is None:
            self.node.get_logger().warn(f"No validator registered for sensor: {sensor_name}")
            return None
            
        if not validator.is_enabled():
            return None
            
        try:
            # Perform validation
            result = validator.validate(data)
            self.total_validations += 1
            
            # Process the result
            self._process_validation_result(result)
            
            return result
            
        except Exception as e:
            self.node.get_logger().error(f"Error validating {sensor_name} data: {str(e)}")
            return None
            
    def validate_all_sensors(self, sensor_data: Dict[str, Any]) -> Dict[str, ValidationResult]:
        """
        Validate data from multiple sensors at once.
        
        Args:
            sensor_data: Dictionary mapping sensor names to their data
            
        Returns:
            Dictionary mapping sensor names to their validation results
        """
        results = {}
        
        for sensor_name, data in sensor_data.items():
            result = self.validate_sensor_data(sensor_name, data)
            if result is not None:
                results[sensor_name] = result
                
        return results
        
    def _process_validation_result(self, result: ValidationResult) -> None:
        """
        Process a validation result and update internal state.
        
        Args:
            result: Validation result to process
        """
        sensor_name = result.sensor_name
        
        # Add to warning history
        self.warning_history.append(result)
        
        # Update active warnings
        if not result.is_valid:
            self.active_warnings[sensor_name].append(result)
            self.total_warnings += 1
            
            # Log the warning
            severity_str = {
                SeverityLevel.INFO: "INFO",
                SeverityLevel.WARNING: "WARN", 
                SeverityLevel.ERROR: "ERROR",
                SeverityLevel.CRITICAL: "CRITICAL"
            }.get(result.severity, "UNKNOWN")
            
            self.node.get_logger().log(
                self._get_log_level(result.severity),
                f"[{severity_str}] {sensor_name}: {result.description}"
            )
            
        # Clean up old active warnings (keep only recent ones)
        cutoff_time = time.time() - 10.0  # Keep warnings from last 10 seconds
        self.active_warnings[sensor_name] = [
            w for w in self.active_warnings[sensor_name] 
            if w.timestamp > cutoff_time
        ]
        
        # Update health tracking
        self._update_sensor_health(sensor_name)
        
    def _update_sensor_health(self, sensor_name: str) -> None:
        """
        Update health tracking for a specific sensor.
        
        Args:
            sensor_name: Name of sensor to update
        """
        current_time = time.time()
        
        # Rate limit health updates
        if current_time - self.last_health_update < self.health_update_interval:
            return
            
        validator = self.validators.get(sensor_name)
        health_tracker = self.health_trackers.get(sensor_name)
        
        if validator and health_tracker:
            health_score = validator.get_health_score()
            active_anomalies = validator.get_active_anomalies()
            health_tracker.update_health(health_score, active_anomalies)
            
        self.last_health_update = current_time
        
    def get_sensor_health(self, sensor_name: str) -> Optional[Dict[str, Any]]:
        """
        Get health information for a specific sensor.
        
        Args:
            sensor_name: Name of sensor
            
        Returns:
            Dictionary with health information or None if sensor not found
        """
        validator = self.validators.get(sensor_name)
        health_tracker = self.health_trackers.get(sensor_name)
        
        if not validator or not health_tracker:
            return None
            
        return {
            'sensor_name': sensor_name,
            'health_score': validator.get_health_score(),
            'is_valid': health_tracker.is_valid,
            'active_anomalies': validator.get_active_anomalies(),
            'last_update': health_tracker.last_update,
            'anomaly_count': len(validator.get_active_anomalies()),
            'average_confidence': self._get_average_confidence(sensor_name),
            'health_trend': health_tracker.get_health_trend()
        }
        
    def get_all_sensor_health(self) -> Dict[str, Dict[str, Any]]:
        """
        Get health information for all registered sensors.
        
        Returns:
            Dictionary mapping sensor names to their health information
        """
        health_info = {}
        
        for sensor_name in self.validators.keys():
            health_info[sensor_name] = self.get_sensor_health(sensor_name)
            
        return health_info
        
    def get_system_summary(self) -> Dict[str, Any]:
        """
        Get overall system sanity summary.
        
        Returns:
            Dictionary with system-wide sanity information
        """
        all_health = self.get_all_sensor_health()
        
        # Determine overall system status
        system_healthy = True
        has_critical_issues = False
        has_errors = False
        has_warnings = False
        sensors_with_issues = []
        max_severity = SeverityLevel.INFO
        critical_issue_summary = ""
        
        for sensor_name, health_info in all_health.items():
            if health_info and not health_info['is_valid']:
                sensors_with_issues.append(sensor_name)
                system_healthy = False
                
        # Check recent warnings for severity levels
        recent_warnings = [w for w in self.warning_history if time.time() - w.timestamp < 30.0]
        
        for warning in recent_warnings:
            if not warning.is_valid:
                if warning.severity == SeverityLevel.CRITICAL:
                    has_critical_issues = True
                    if not critical_issue_summary:
                        critical_issue_summary = f"{warning.sensor_name}: {warning.description}"
                elif warning.severity == SeverityLevel.ERROR:
                    has_errors = True
                elif warning.severity == SeverityLevel.WARNING:
                    has_warnings = True
                    
                max_severity = max(max_severity, warning.severity)
                
        return {
            'system_healthy': system_healthy,
            'has_critical_issues': has_critical_issues,
            'has_errors': has_errors,
            'has_warnings': has_warnings,
            'sensors_with_issues': sensors_with_issues,
            'max_severity_level': int(max_severity),
            'timestamp': time.time(),
            'critical_issue_summary': critical_issue_summary,
            'total_sensors': len(self.validators),
            'healthy_sensors': len([h for h in all_health.values() if h and h['is_valid']]),
            'total_validations': self.total_validations,
            'total_warnings': self.total_warnings
        }
        
    def get_recent_warnings(self, count: int = 10) -> List[ValidationResult]:
        """
        Get recent validation warnings.
        
        Args:
            count: Maximum number of warnings to return
            
        Returns:
            List of recent ValidationResult objects with issues
        """
        recent_invalid = [w for w in self.warning_history if not w.is_valid]
        return recent_invalid[-count:] if len(recent_invalid) > count else recent_invalid
        
    def _get_average_confidence(self, sensor_name: str) -> float:
        """
        Calculate average confidence for recent validations of a sensor.
        
        Args:
            sensor_name: Name of sensor
            
        Returns:
            Average confidence level
        """
        recent_validations = [
            w for w in self.warning_history 
            if w.sensor_name == sensor_name and time.time() - w.timestamp < 60.0
        ]
        
        if not recent_validations:
            return 1.0
            
        return sum(v.confidence for v in recent_validations) / len(recent_validations)
        
    def _get_log_level(self, severity: SeverityLevel) -> int:
        """
        Convert severity level to ROS2 log level.
        
        Args:
            severity: Severity level
            
        Returns:
            ROS2 log level constant
        """
        if severity == SeverityLevel.CRITICAL:
            return rclpy.logging.LoggingSeverity.FATAL
        elif severity == SeverityLevel.ERROR:
            return rclpy.logging.LoggingSeverity.ERROR
        elif severity == SeverityLevel.WARNING:
            return rclpy.logging.LoggingSeverity.WARN
        else:
            return rclpy.logging.LoggingSeverity.INFO
            
    def get_statistics(self) -> Dict[str, Any]:
        """
        Get comprehensive statistics about sanity checking operations.
        
        Returns:
            Dictionary with statistics
        """
        uptime = time.time() - self.start_time
        
        return {
            'enabled': self.enabled,
            'uptime_seconds': uptime,
            'total_validations': self.total_validations,
            'total_warnings': self.total_warnings,
            'registered_validators': len(self.validators),
            'active_sensors': len([v for v in self.validators.values() if v.is_enabled()]),
            'warning_rate': self.total_warnings / max(1, self.total_validations),
            'validations_per_second': self.total_validations / max(1, uptime),
            'validator_statistics': {
                name: validator.get_statistics() 
                for name, validator in self.validators.items()
            }
        }
        
    def is_enabled(self) -> bool:
        """Check if sanity checking is enabled."""
        return self.enabled
        
    def enable(self) -> None:
        """Enable sanity checking."""
        self.enabled = True
        self.node.get_logger().info("Sanity checking enabled")
        
    def disable(self) -> None:
        """Disable sanity checking."""
        self.enabled = False
        self.node.get_logger().info("Sanity checking disabled")