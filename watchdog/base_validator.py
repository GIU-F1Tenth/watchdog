#!/usr/bin/env python3

"""
Base Validator Class for Watchdog Sanity Checking

This module provides the abstract base class that all sensor-specific validators
must inherit from. It defines the common interface and utility methods for
validation operations.

Author: F1TENTH Watchdog Team
License: MIT
Version: 1.0.0
"""

from abc import ABC, abstractmethod
from typing import Any, List, Optional, Dict
from dataclasses import dataclass
from enum import IntEnum
import time
import statistics
from collections import deque


class SeverityLevel(IntEnum):
    """Severity levels for validation results."""
    INFO = 1
    WARNING = 2
    ERROR = 3
    CRITICAL = 4


@dataclass
class ValidationResult:
    """
    Data structure for validation results.
    
    This standardizes the output from all validators to make it easy
    for the SanityChecker to process and publish warnings.
    """
    sensor_name: str
    is_valid: bool
    severity: SeverityLevel
    anomaly_type: str
    description: str
    suggested_action: str
    confidence: float
    timestamp: float
    raw_data: Optional[Any] = None
    
    def to_dict(self) -> Dict:
        """Convert to dictionary for easy serialization."""
        return {
            'sensor_name': self.sensor_name,
            'is_valid': self.is_valid,
            'severity': int(self.severity),
            'anomaly_type': self.anomaly_type,  
            'description': self.description,
            'suggested_action': self.suggested_action,
            'confidence': self.confidence,
            'timestamp': self.timestamp
        }


class BaseValidator(ABC):
    """
    Abstract base class for all sensor validators.
    
    This class defines the common interface that all specific validators
    (LiDAR, camera, battery, etc.) must implement. It also provides
    utility methods for common validation tasks.
    """
    
    def __init__(self, sensor_name: str, config: Dict[str, Any]):
        """
        Initialize the base validator.
        
        Args:
            sensor_name: Name of the sensor this validator handles
            config: Configuration parameters for this validator
        """
        self.sensor_name = sensor_name
        self.config = config
        self.enabled = config.get(f'{sensor_name}_sanity_enabled', True)
        
        # History tracking for statistical analysis
        self.history_size = config.get('moving_average_window_size', 20)
        self.data_history = deque(maxlen=self.history_size)
        self.validation_history = deque(maxlen=100)  # Keep last 100 validations
        
        # Statistics tracking
        self.total_validations = 0
        self.failed_validations = 0
        self.last_validation_time = 0.0
        
    @abstractmethod
    def validate(self, data: Any) -> ValidationResult:
        """
        Validate sensor data and return result.
        
        This is the main method that each specific validator must implement.
        It should analyze the incoming sensor data and return a ValidationResult.
        
        Args:
            data: The sensor data to validate (type depends on sensor)
            
        Returns:
            ValidationResult containing validation outcome
        """
        pass
    
    def is_enabled(self) -> bool:
        """Check if this validator is enabled."""
        return self.enabled
    
    def get_health_score(self) -> float:
        """
        Calculate current health score for this sensor.
        
        Returns:
            Float between 0.0 (completely failed) and 1.0 (perfect health)
        """
        if self.total_validations == 0:
            return 1.0  # No data yet, assume healthy
            
        # Basic health score based on recent validation success rate
        recent_validations = list(self.validation_history)[-20:]  # Last 20 validations
        if not recent_validations:
            return 1.0
            
        success_rate = sum(1 for result in recent_validations if result.is_valid) / len(recent_validations)
        
        # Weight by severity of failures
        severity_penalty = 0.0
        for result in recent_validations:
            if not result.is_valid:
                if result.severity == SeverityLevel.CRITICAL:
                    severity_penalty += 0.3
                elif result.severity == SeverityLevel.ERROR:
                    severity_penalty += 0.2
                elif result.severity == SeverityLevel.WARNING:
                    severity_penalty += 0.1
                    
        health_score = success_rate - (severity_penalty / len(recent_validations))
        return max(0.0, min(1.0, health_score))
    
    def get_active_anomalies(self) -> List[str]:
        """
        Get list of currently active anomalies.
        
        Returns:
            List of anomaly type strings
        """
        # Look at recent validations to find active issues
        recent_validations = list(self.validation_history)[-5:]  # Last 5 validations
        active_anomalies = []
        
        for result in recent_validations:
            if not result.is_valid and result.anomaly_type not in active_anomalies:
                active_anomalies.append(result.anomaly_type)
                
        return active_anomalies
    
    def _record_validation(self, result: ValidationResult) -> None:
        """
        Record a validation result for statistics tracking.
        
        Args:
            result: The validation result to record
        """
        self.validation_history.append(result)
        self.total_validations += 1
        if not result.is_valid:
            self.failed_validations += 1
        self.last_validation_time = time.time()
    
    def _create_result(self, 
                      is_valid: bool,
                      severity: SeverityLevel,
                      anomaly_type: str,
                      description: str,
                      suggested_action: str,
                      confidence: float = 1.0,
                      raw_data: Any = None) -> ValidationResult:
        """
        Helper method to create a ValidationResult.
        
        Args:
            is_valid: Whether the data is valid
            severity: Severity level of any issues
            anomaly_type: Type of anomaly detected
            description: Human-readable description
            suggested_action: Recommended action to take
            confidence: Confidence in the validation (0.0-1.0)
            raw_data: Original sensor data (optional)
            
        Returns:
            ValidationResult object
        """
        result = ValidationResult(
            sensor_name=self.sensor_name,
            is_valid=is_valid,
            severity=severity,
            anomaly_type=anomaly_type,
            description=description,
            suggested_action=suggested_action,
            confidence=confidence,
            timestamp=time.time(),
            raw_data=raw_data
        )
        
        self._record_validation(result)
        return result
    
    def _is_outlier(self, value: float, data_list: List[float], sigma: float = 2.0) -> bool:
        """
        Check if a value is an outlier using statistical analysis.
        
        Args:
            value: Value to check
            data_list: List of recent values for comparison
            sigma: Number of standard deviations for outlier detection
            
        Returns:
            True if value is an outlier
        """
        if len(data_list) < 3:
            return False  # Need at least 3 points for statistics
            
        try:
            mean = statistics.mean(data_list)
            stdev = statistics.stdev(data_list)
            
            if stdev == 0:
                return value != mean
                
            z_score = abs(value - mean) / stdev
            return z_score > sigma
            
        except statistics.StatisticsError:
            return False
    
    def _calculate_moving_average(self, data_list: List[float]) -> float:
        """
        Calculate moving average of recent data.
        
        Args:
            data_list: List of numerical values
            
        Returns:
            Moving average value
        """
        if not data_list:
            return 0.0
        return statistics.mean(data_list)
    
    def _detect_trend(self, data_list: List[float], min_points: int = 5) -> str:
        """
        Detect trend in data (increasing, decreasing, stable).
        
        Args:
            data_list: List of numerical values in chronological order
            min_points: Minimum points needed for trend detection
            
        Returns:
            'increasing', 'decreasing', or 'stable'
        """
        if len(data_list) < min_points:
            return 'stable'
            
        # Simple linear trend detection
        increases = 0
        decreases = 0
        
        for i in range(1, len(data_list)):
            if data_list[i] > data_list[i-1]:
                increases += 1
            elif data_list[i] < data_list[i-1]:
                decreases += 1
                
        if increases > len(data_list) * 0.6:
            return 'increasing'
        elif decreases > len(data_list) * 0.6:
            return 'decreasing'
        else:
            return 'stable'
    
    def get_statistics(self) -> Dict[str, Any]:
        """
        Get validation statistics for this sensor.
        
        Returns:
            Dictionary with validation statistics
        """
        success_rate = 0.0
        if self.total_validations > 0:
            success_rate = (self.total_validations - self.failed_validations) / self.total_validations
            
        return {
            'sensor_name': self.sensor_name,
            'total_validations': self.total_validations,
            'failed_validations': self.failed_validations,
            'success_rate': success_rate,
            'health_score': self.get_health_score(),
            'active_anomalies': self.get_active_anomalies(),
            'last_validation_time': self.last_validation_time,
            'enabled': self.enabled
        }