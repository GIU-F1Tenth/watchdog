#!/usr/bin/env python3

"""
Integration tests for the SanityChecker system in the F1TENTH watchdog.

This module tests the complete sanity checking system including validator
registration, sensor data processing, and health tracking functionality.

Author: F1TENTH Watchdog Team
License: MIT
"""

import unittest
import sys
import os
from unittest.mock import MagicMock, patch
import time

# Add the watchdog module to the path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# Import components to test
from watchdog.sanity_checker import SanityChecker, SensorHealthTracker
from watchdog.base_validator import BaseValidator, ValidationResult, SeverityLevel
from watchdog.validators.lidar_validator import LiDARValidator
from watchdog.validators.battery_validator import BatteryValidator


class MockValidator(BaseValidator):
    """Mock validator for testing purposes."""
    
    def __init__(self, node, should_fail=False, anomaly_type="MOCK_ANOMALY"):
        super().__init__(node)
        self.should_fail = should_fail
        self.anomaly_type = anomaly_type
        self.call_count = 0
    
    def validate(self, data):
        """Mock validation that can be configured to pass or fail."""
        self.call_count += 1
        
        if self.should_fail:
            return [ValidationResult(
                is_valid=False,
                anomaly_type=self.anomaly_type,
                description=f"Mock anomaly detected in call {self.call_count}",
                severity=SeverityLevel.MEDIUM,
                suggested_action="Mock action"
            )]
        else:
            return []


class MockNode:
    """Mock ROS2 node for testing."""
    
    def __init__(self):
        self.params = {
            'sanity.health_decay_rate': 0.1,
            'sanity.health_recovery_rate': 0.05,
            'sanity.min_health_score': 0.0,
            'sanity.max_health_score': 100.0,
        }
        
        # Mock logger
        self.logger = MagicMock()
    
    def get_parameter(self, name):
        """Mock parameter getter."""
        param = MagicMock()
        if name in self.params:
            param.get_parameter_value.return_value.double_value = self.params[name]
        else:
            param.get_parameter_value.return_value.double_value = 0.0
        return param
    
    def get_logger(self):
        """Return mock logger."""
        return self.logger


class TestSensorHealthTracker(unittest.TestCase):
    """Test the SensorHealthTracker component."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.tracker = SensorHealthTracker()
    
    def test_initial_health_score(self):
        """Test that new sensors start with perfect health."""
        score = self.tracker.get_health_score('test_sensor')
        self.assertEqual(score, 100.0)
    
    def test_anomaly_recording(self):
        """Test recording and tracking of anomalies."""
        # Record an anomaly
        self.tracker.record_anomaly('test_sensor', 'TEST_ANOMALY', SeverityLevel.HIGH)
        
        # Check that it's tracked
        anomalies = self.tracker.get_active_anomalies('test_sensor')
        self.assertEqual(len(anomalies), 1)
        self.assertEqual(anomalies[0]['type'], 'TEST_ANOMALY')
        self.assertEqual(anomalies[0]['severity'], SeverityLevel.HIGH)
    
    def test_health_score_degradation(self):
        """Test that health score degrades with anomalies."""
        initial_score = self.tracker.get_health_score('test_sensor')
        
        # Record multiple anomalies
        for i in range(5):
            self.tracker.record_anomaly('test_sensor', f'ANOMALY_{i}', SeverityLevel.MEDIUM)
            self.tracker.update_health_score('test_sensor')
        
        final_score = self.tracker.get_health_score('test_sensor')
        self.assertLess(final_score, initial_score)
    
    def test_anomaly_expiration(self):
        """Test that old anomalies expire."""
        # Record an anomaly with old timestamp
        old_time = time.time() - 3700  # Over an hour ago
        self.tracker.record_anomaly('test_sensor', 'OLD_ANOMALY', SeverityLevel.LOW)
        
        # Manually set old timestamp
        if 'test_sensor' in self.tracker.sensor_anomalies:
            self.tracker.sensor_anomalies['test_sensor'][0]['timestamp'] = old_time
        
        # Clean up expired anomalies
        self.tracker._cleanup_expired_anomalies('test_sensor')
        
        # Should have no active anomalies
        anomalies = self.tracker.get_active_anomalies('test_sensor')
        self.assertEqual(len(anomalies), 0)
    
    def test_health_summary(self):
        """Test generation of health summary."""
        # Add some sensors with different health states
        self.tracker.record_anomaly('sensor1', 'ANOMALY1', SeverityLevel.LOW)
        self.tracker.record_anomaly('sensor2', 'ANOMALY2', SeverityLevel.HIGH)
        self.tracker.update_health_score('sensor1')
        self.tracker.update_health_score('sensor2')
        
        summary = self.tracker.get_health_summary()
        
        # Should include both sensors
        self.assertIn('sensor1', summary)
        self.assertIn('sensor2', summary)
        
        # Should have health scores and anomaly counts
        self.assertIn('health_score', summary['sensor1'])
        self.assertIn('active_anomalies', summary['sensor1'])


class TestSanityChecker(unittest.TestCase):
    """Test the main SanityChecker class."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.mock_node = MockNode()
        self.sanity_checker = SanityChecker(self.mock_node)
    
    def test_validator_registration(self):
        """Test registering validators."""
        # Create a mock validator
        validator = MockValidator(self.mock_node)
        
        # Register it
        self.sanity_checker.register_validator('test_sensor', validator)
        
        # Check it's registered
        self.assertIn('test_sensor', self.sanity_checker.validators)
        self.assertEqual(self.sanity_checker.validators['test_sensor'], validator)
    
    def test_sensor_data_validation_success(self):
        """Test successful sensor data validation."""
        # Register a validator that always passes
        validator = MockValidator(self.mock_node, should_fail=False)
        self.sanity_checker.register_validator('test_sensor', validator)
        
        # Validate some data
        results = self.sanity_checker.validate_sensor_data('test_sensor', {'test': 'data'})
        
        # Should return empty list (no anomalies)
        self.assertEqual(len(results), 0)
        self.assertEqual(validator.call_count, 1)
    
    def test_sensor_data_validation_failure(self):
        """Test sensor data validation with anomalies."""
        # Register a validator that always fails
        validator = MockValidator(self.mock_node, should_fail=True)
        self.sanity_checker.register_validator('test_sensor', validator)
        
        # Validate some data
        results = self.sanity_checker.validate_sensor_data('test_sensor', {'test': 'data'})
        
        # Should return anomaly results
        self.assertEqual(len(results), 1)
        self.assertFalse(results[0].is_valid)
        self.assertEqual(results[0].anomaly_type, 'MOCK_ANOMALY')
    
    def test_unregistered_sensor_handling(self):
        """Test handling of unregistered sensors."""
        # Try to validate data for unregistered sensor
        results = self.sanity_checker.validate_sensor_data('unknown_sensor', {'test': 'data'})
        
        # Should return empty list and log warning
        self.assertEqual(len(results), 0)
        self.mock_node.logger.warn.assert_called()
    
    def test_health_tracking_integration(self):
        """Test integration with health tracking."""
        # Register a failing validator
        validator = MockValidator(self.mock_node, should_fail=True)
        self.sanity_checker.register_validator('test_sensor', validator)
        
        # Validate data multiple times
        for i in range(3):
            self.sanity_checker.validate_sensor_data('test_sensor', {'test': f'data_{i}'})
        
        # Check health summary
        summary = self.sanity_checker.get_health_summary()
        
        # Should show degraded health for test_sensor
        self.assertIn('test_sensor', summary)
        self.assertLess(summary['test_sensor']['health_score'], 100.0)
        self.assertGreater(len(summary['test_sensor']['active_anomalies']), 0)
    
    def test_multiple_validators(self):
        """Test system with multiple validators."""
        # Register multiple validators
        validator1 = MockValidator(self.mock_node, should_fail=False)
        validator2 = MockValidator(self.mock_node, should_fail=True, anomaly_type='ANOMALY_2')
        
        self.sanity_checker.register_validator('sensor1', validator1)
        self.sanity_checker.register_validator('sensor2', validator2)
        
        # Validate data for both sensors
        results1 = self.sanity_checker.validate_sensor_data('sensor1', {'test': 'data1'})
        results2 = self.sanity_checker.validate_sensor_data('sensor2', {'test': 'data2'})
        
        # Check results
        self.assertEqual(len(results1), 0)  # No anomalies
        self.assertEqual(len(results2), 1)  # One anomaly
        
        # Check health summary includes both
        summary = self.sanity_checker.get_health_summary()
        self.assertIn('sensor1', summary)
        self.assertIn('sensor2', summary)
    
    def test_error_handling(self):
        """Test error handling in validation."""
        # Create a validator that raises an exception
        class FailingValidator(BaseValidator):
            def validate(self, data):
                raise Exception("Test exception")
        
        validator = FailingValidator(self.mock_node)
        self.sanity_checker.register_validator('failing_sensor', validator)
        
        # Validate data - should handle exception gracefully
        results = self.sanity_checker.validate_sensor_data('failing_sensor', {'test': 'data'})
        
        # Should return empty list and log error
        self.assertEqual(len(results), 0)
        self.mock_node.logger.error.assert_called()


class TestRealValidatorIntegration(unittest.TestCase):
    """Test integration with real validators."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.mock_node = MockNode()
        # Add validator-specific parameters
        self.mock_node.params.update({
            'sanity.lidar.range_min': 0.1,
            'sanity.lidar.range_max': 12.0,
            'sanity.lidar.noise_threshold': 0.05,
            'sanity.lidar.dead_zone_threshold': 0.1,
            'sanity.lidar.min_valid_points': 100,
            'sanity.battery.voltage_min': 10.0,
            'sanity.battery.voltage_max': 12.6,
            'sanity.battery.voltage_critical': 10.5,
        })
        
        self.sanity_checker = SanityChecker(self.mock_node)
    
    def test_lidar_validator_integration(self):
        """Test integration with LiDAR validator."""
        # Register LiDAR validator
        lidar_validator = LiDARValidator(self.mock_node)
        self.sanity_checker.register_validator('lidar', lidar_validator)
        
        # Create mock LiDAR data with anomaly
        mock_lidar = MagicMock()
        mock_lidar.ranges = [-1.0, 15.0] * 250  # Out of range data
        mock_lidar.range_min = 0.1
        mock_lidar.range_max = 12.0
        mock_lidar.header.stamp.sec = 1234567890
        mock_lidar.header.stamp.nanosec = 123456789
        
        # Validate data
        results = self.sanity_checker.validate_sensor_data('lidar', mock_lidar)
        
        # Should detect anomalies
        self.assertGreater(len(results), 0)
        
        # Check health tracking
        summary = self.sanity_checker.get_health_summary()
        self.assertIn('lidar', summary)
        self.assertLess(summary['lidar']['health_score'], 100.0)
    
    def test_battery_validator_integration(self):
        """Test integration with battery validator."""
        # Register battery validator
        battery_validator = BatteryValidator(self.mock_node)
        self.sanity_checker.register_validator('battery', battery_validator)
        
        # Create battery data with low voltage
        battery_data = {
            'voltage': 9.5,  # Below minimum
            'current': 15.0,
            'temperature': 45.0
        }
        
        # Validate data
        results = self.sanity_checker.validate_sensor_data('battery', battery_data)
        
        # Should detect low voltage
        self.assertGreater(len(results), 0)
        low_voltage_detected = any(r.anomaly_type == "LOW_VOLTAGE" for r in results)
        self.assertTrue(low_voltage_detected)
        
        # Check health tracking
        summary = self.sanity_checker.get_health_summary()
        self.assertIn('battery', summary)
        self.assertGreater(len(summary['battery']['active_anomalies']), 0)


if __name__ == '__main__':
    # Set up test suite
    unittest.main(verbosity=2)