#!/usr/bin/env python3

"""
Unit tests for sensor-specific validators in the F1TENTH watchdog system.

This module tests each validator individually with mock sensor data to ensure
they correctly identify anomalies and generate appropriate validation results.

Author: F1TENTH Watchdog Team
License: MIT
"""

import unittest
import sys
import os
from unittest.mock import MagicMock, patch
from collections import deque

# Add the watchdog module to the path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# Import components to test
from watchdog.base_validator import BaseValidator, ValidationResult, SeverityLevel
from watchdog.validators.lidar_validator import LiDARValidator
from watchdog.validators.battery_validator import BatteryValidator
from watchdog.validators.camera_validator import CameraValidator
from watchdog.validators.odometry_validator import OdometryValidator


class MockNode:
    """Mock ROS2 node for testing validators."""
    
    def __init__(self):
        self.params = {
            # LiDAR parameters
            'sanity.lidar.range_min': 0.1,
            'sanity.lidar.range_max': 12.0,
            'sanity.lidar.noise_threshold': 0.05,
            'sanity.lidar.dead_zone_threshold': 0.1,
            'sanity.lidar.min_valid_points': 100,
            
            # Battery parameters
            'sanity.battery.voltage_min': 10.0,
            'sanity.battery.voltage_max': 12.6,
            'sanity.battery.voltage_critical': 10.5,
            'sanity.battery.current_max': 30.0,
            'sanity.battery.temp_max': 85.0,
            'sanity.battery.rate_change_threshold': 1.0,
            
            # Camera parameters
            'sanity.camera.brightness_min': 50,
            'sanity.camera.brightness_max': 200,
            'sanity.camera.blur_threshold': 100.0,
            'sanity.camera.corruption_threshold': 0.05,
            'sanity.camera.min_resolution': 480,
            
            # Odometry parameters
            'sanity.odometry.max_linear_velocity': 5.0,
            'sanity.odometry.max_angular_velocity': 2.0,
            'sanity.odometry.max_linear_acceleration': 3.0,
            'sanity.odometry.max_angular_acceleration': 1.5,
        }
    
    def get_parameter(self, name):
        """Mock parameter getter."""
        param = MagicMock()
        if name in self.params:
            param.get_parameter_value.return_value.double_value = self.params[name]
            param.get_parameter_value.return_value.integer_value = int(self.params[name])
        else:
            param.get_parameter_value.return_value.double_value = 0.0
            param.get_parameter_value.return_value.integer_value = 0
        return param


class TestBaseValidator(unittest.TestCase):
    """Test the BaseValidator abstract class and utilities."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.mock_node = MockNode()
    
    def test_validation_result(self):
        """Test ValidationResult dataclass."""
        result = ValidationResult(
            is_valid=False,
            anomaly_type="TEST_ANOMALY",
            description="Test description",
            severity=SeverityLevel.HIGH,
            suggested_action="Test action"
        )
        
        self.assertFalse(result.is_valid)
        self.assertEqual(result.anomaly_type, "TEST_ANOMALY")
        self.assertEqual(result.severity, SeverityLevel.HIGH)
    
    def test_severity_levels(self):
        """Test severity level enumeration."""
        self.assertEqual(SeverityLevel.LOW.value, 1)
        self.assertEqual(SeverityLevel.MEDIUM.value, 2)
        self.assertEqual(SeverityLevel.HIGH.value, 3)
        self.assertEqual(SeverityLevel.CRITICAL.value, 4)


class TestLiDARValidator(unittest.TestCase):
    """Test LiDAR sensor validation."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.mock_node = MockNode()
        self.validator = LiDARValidator(self.mock_node)
    
    def create_mock_lidar_msg(self, ranges, range_min=0.1, range_max=12.0):
        """Create a mock LaserScan message."""
        msg = MagicMock()
        msg.ranges = ranges
        msg.range_min = range_min
        msg.range_max = range_max
        msg.header.stamp.sec = 1234567890
        msg.header.stamp.nanosec = 123456789
        return msg
    
    def test_valid_lidar_data(self):
        """Test validation of normal LiDAR data."""
        # Create valid LiDAR data
        ranges = [1.0, 2.0, 3.0, 4.0, 5.0] * 100  # 500 valid points
        msg = self.create_mock_lidar_msg(ranges)
        
        results = self.validator.validate(msg)
        
        # Should return empty list (no anomalies)
        self.assertEqual(len(results), 0)
    
    def test_out_of_range_detection(self):
        """Test detection of out-of-range readings."""
        # Create data with out-of-range values
        ranges = [-1.0, 0.05, 15.0, 2.0, 3.0] * 100
        msg = self.create_mock_lidar_msg(ranges)
        
        results = self.validator.validate(msg)
        
        # Should detect out-of-range anomaly
        self.assertGreater(len(results), 0)
        self.assertEqual(results[0].anomaly_type, "OUT_OF_RANGE")
    
    def test_excessive_noise_detection(self):
        """Test detection of excessive noise in LiDAR data."""
        # Create very noisy data
        ranges = []
        for i in range(500):
            if i % 2 == 0:
                ranges.append(1.0)
            else:
                ranges.append(10.0)  # High variance
        
        msg = self.create_mock_lidar_msg(ranges)
        
        results = self.validator.validate(msg)
        
        # Should detect noise anomaly
        noise_detected = any(r.anomaly_type == "EXCESSIVE_NOISE" for r in results)
        self.assertTrue(noise_detected)
    
    def test_insufficient_data_detection(self):
        """Test detection of insufficient valid data points."""
        # Create data with mostly invalid readings
        ranges = [float('inf')] * 400 + [1.0] * 50  # Only 50 valid points
        msg = self.create_mock_lidar_msg(ranges)
        
        results = self.validator.validate(msg)
        
        # Should detect insufficient data
        insufficient_detected = any(r.anomaly_type == "INSUFFICIENT_DATA" for r in results)
        self.assertTrue(insufficient_detected)


class TestBatteryValidator(unittest.TestCase):
    """Test battery/VESC sensor validation."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.mock_node = MockNode()
        self.validator = BatteryValidator(self.mock_node)
    
    def test_valid_battery_data(self):
        """Test validation of normal battery data."""
        data = {
            'voltage': 11.5,
            'current': 15.0,
            'temperature': 45.0
        }
        
        results = self.validator.validate(data)
        
        # Should return empty list (no anomalies)
        self.assertEqual(len(results), 0)
    
    def test_low_voltage_detection(self):
        """Test detection of low voltage."""
        data = {
            'voltage': 9.5,  # Below minimum
            'current': 15.0,
            'temperature': 45.0
        }
        
        results = self.validator.validate(data)
        
        # Should detect low voltage
        self.assertGreater(len(results), 0)
        self.assertEqual(results[0].anomaly_type, "LOW_VOLTAGE")
        self.assertEqual(results[0].severity, SeverityLevel.HIGH)
    
    def test_critical_voltage_detection(self):
        """Test detection of critical voltage."""
        data = {
            'voltage': 10.0,  # Below critical threshold
            'current': 15.0,
            'temperature': 45.0
        }
        
        results = self.validator.validate(data)
        
        # Should detect critical voltage
        critical_detected = any(r.anomaly_type == "CRITICAL_VOLTAGE" for r in results)
        self.assertTrue(critical_detected)
        critical_result = next(r for r in results if r.anomaly_type == "CRITICAL_VOLTAGE")
        self.assertEqual(critical_result.severity, SeverityLevel.CRITICAL)
    
    def test_high_current_detection(self):
        """Test detection of excessive current draw."""
        data = {
            'voltage': 11.5,
            'current': 35.0,  # Above maximum
            'temperature': 45.0
        }
        
        results = self.validator.validate(data)
        
        # Should detect high current
        high_current_detected = any(r.anomaly_type == "HIGH_CURRENT" for r in results)
        self.assertTrue(high_current_detected)
    
    def test_voltage_trend_detection(self):
        """Test detection of rapid voltage changes."""
        # Add some data points to history
        for voltage in [11.8, 11.7, 11.6, 11.5]:
            data = {'voltage': voltage, 'current': 15.0, 'temperature': 45.0}
            self.validator.validate(data)
        
        # Now add a point with rapid drop
        data = {
            'voltage': 10.0,  # Rapid drop
            'current': 15.0,
            'temperature': 45.0
        }
        
        results = self.validator.validate(data)
        
        # Should detect voltage trend anomaly
        trend_detected = any(r.anomaly_type == "VOLTAGE_TREND" for r in results)
        self.assertTrue(trend_detected)


class TestCameraValidator(unittest.TestCase):
    """Test camera sensor validation."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.mock_node = MockNode()
        self.validator = CameraValidator(self.mock_node)
    
    def create_mock_image_msg(self, width=640, height=480, encoding="rgb8"):
        """Create a mock Image message."""
        msg = MagicMock()
        msg.width = width
        msg.height = height
        msg.encoding = encoding
        msg.step = width * 3  # 3 bytes per pixel for RGB
        msg.data = bytearray([128] * (width * height * 3))  # Mid-gray image
        msg.header.stamp.sec = 1234567890
        msg.header.stamp.nanosec = 123456789
        return msg
    
    def test_valid_camera_data(self):
        """Test validation of normal camera data."""
        msg = self.create_mock_image_msg()
        
        results = self.validator.validate(msg)
        
        # Should return empty list (no anomalies)
        self.assertEqual(len(results), 0)
    
    def test_low_resolution_detection(self):
        """Test detection of low resolution images."""
        msg = self.create_mock_image_msg(width=320, height=240)  # Below minimum
        
        results = self.validator.validate(msg)
        
        # Should detect low resolution
        low_res_detected = any(r.anomaly_type == "LOW_RESOLUTION" for r in results)
        self.assertTrue(low_res_detected)
    
    def test_brightness_anomaly_detection(self):
        """Test detection of brightness anomalies."""
        # Create very dark image
        msg = self.create_mock_image_msg()
        msg.data = bytearray([10] * (640 * 480 * 3))  # Very dark
        
        results = self.validator.validate(msg)
        
        # Should detect brightness anomaly
        brightness_detected = any(r.anomaly_type == "BRIGHTNESS_ANOMALY" for r in results)
        self.assertTrue(brightness_detected)
    
    def test_unsupported_encoding_detection(self):
        """Test detection of unsupported image encodings."""
        msg = self.create_mock_image_msg(encoding="unknown_format")
        
        results = self.validator.validate(msg)
        
        # Should detect unsupported encoding
        encoding_detected = any(r.anomaly_type == "UNSUPPORTED_ENCODING" for r in results)
        self.assertTrue(encoding_detected)


class TestOdometryValidator(unittest.TestCase):
    """Test odometry sensor validation."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.mock_node = MockNode()
        self.validator = OdometryValidator(self.mock_node)
    
    def test_valid_odometry_data(self):
        """Test validation of normal odometry data."""
        data = {
            'linear_velocity': 2.0,
            'angular_velocity': 0.5
        }
        
        results = self.validator.validate(data)
        
        # Should return empty list (no anomalies)
        self.assertEqual(len(results), 0)
    
    def test_excessive_linear_velocity_detection(self):
        """Test detection of excessive linear velocity."""
        data = {
            'linear_velocity': 6.0,  # Above maximum
            'angular_velocity': 0.5
        }
        
        results = self.validator.validate(data)
        
        # Should detect excessive velocity
        self.assertGreater(len(results), 0)
        self.assertEqual(results[0].anomaly_type, "EXCESSIVE_LINEAR_VELOCITY")
    
    def test_excessive_angular_velocity_detection(self):
        """Test detection of excessive angular velocity."""
        data = {
            'linear_velocity': 2.0,
            'angular_velocity': 3.0  # Above maximum
        }
        
        results = self.validator.validate(data)
        
        # Should detect excessive angular velocity
        angular_detected = any(r.anomaly_type == "EXCESSIVE_ANGULAR_VELOCITY" for r in results)
        self.assertTrue(angular_detected)
    
    def test_acceleration_detection(self):
        """Test detection of excessive acceleration."""
        # Add some data points to history
        for velocity in [1.0, 1.5, 2.0]:
            data = {'linear_velocity': velocity, 'angular_velocity': 0.5}
            self.validator.validate(data)
        
        # Now add a point with high acceleration
        data = {
            'linear_velocity': 6.0,  # Large jump
            'angular_velocity': 0.5
        }
        
        results = self.validator.validate(data)
        
        # Should detect excessive acceleration
        accel_detected = any(r.anomaly_type == "EXCESSIVE_LINEAR_ACCELERATION" for r in results)
        self.assertTrue(accel_detected)


if __name__ == '__main__':
    # Set up test suite
    unittest.main(verbosity=2)