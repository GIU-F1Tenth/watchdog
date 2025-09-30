#!/usr/bin/env python3

"""
Integration tests for the WatchdogNode with sanity checking functionality.

This module tests the complete integration of sanity checking into the
main watchdog node, including parameter loading, publisher setup, and
message handling.

Author: F1TENTH Watchdog Team
License: MIT
"""

import unittest
import sys
import os
from unittest.mock import MagicMock, patch, call
import time

# Add the watchdog module to the path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


class MockParameter:
    """Mock ROS2 parameter."""
    
    def __init__(self, value):
        self.value = value
    
    def get_parameter_value(self):
        """Return mock parameter value."""
        mock_value = MagicMock()
        if isinstance(self.value, bool):
            mock_value.bool_value = self.value
        elif isinstance(self.value, int):
            mock_value.integer_value = self.value
        elif isinstance(self.value, float):
            mock_value.double_value = self.value
        elif isinstance(self.value, str):
            mock_value.string_value = self.value
        return mock_value


class TestWatchdogNodeIntegration(unittest.TestCase):
    """Test WatchdogNode integration with sanity checking."""
    
    def setUp(self):
        """Set up test fixtures."""
        # Mock ROS2 components
        self.rclpy_patcher = patch('rclpy.create_node')
        self.mock_rclpy = self.rclpy_patcher.start()
        
        # Mock the Node class
        self.node_patcher = patch('watchdog.watchdog_node.Node')
        self.mock_node_class = self.node_patcher.start()
        
        # Create mock node instance
        self.mock_node = MagicMock()
        self.mock_node_class.return_value = self.mock_node
        
        # Mock sanity checker components
        self.sanity_checker_patcher = patch('watchdog.watchdog_node.SanityChecker')
        self.mock_sanity_checker_class = self.sanity_checker_patcher.start()
        self.mock_sanity_checker = MagicMock()
        self.mock_sanity_checker_class.return_value = self.mock_sanity_checker
        
        # Mock validators
        self.validator_patches = {}
        validator_names = ['LiDARValidator', 'BatteryValidator', 'CameraValidator', 'OdometryValidator']
        for name in validator_names:
            patcher = patch(f'watchdog.watchdog_node.{name}')
            self.validator_patches[name] = patcher
            patcher.start()
        
        # Set up parameters
        self.setup_mock_parameters()
    
    def tearDown(self):
        """Clean up test fixtures."""
        self.rclpy_patcher.stop()
        self.node_patcher.stop()
        self.sanity_checker_patcher.stop()
        for patcher in self.validator_patches.values():
            patcher.stop()
    
    def setup_mock_parameters(self):
        """Set up mock parameters for the node."""
        self.mock_parameters = {
            # Existing parameters
            'critical_voltage': MockParameter(12.0),
            'min_voltage': MockParameter(9.0),
            'max_voltage': MockParameter(52.0),
            'temp_warning_start': MockParameter(80.0),
            'temp_warning_high': MockParameter(90.0),
            'temp_critical': MockParameter(100.0),
            'lidar_timeout': MockParameter(0.5),
            'camera_timeout': MockParameter(0.5),
            'lidar_critical_timeout': MockParameter(1.0),
            'status_publish_interval': MockParameter(1.0),
            'camera_check_interval': MockParameter(0.5),
            'critical_check_interval': MockParameter(0.5),
            'core_topic': MockParameter('/sensors/core'),
            'camera_topic': MockParameter('/camera/camera/color/image_raw'),
            'lidar_topic': MockParameter('/scan'),
            'odom_topic': MockParameter('/odom'),
            'critical_topic': MockParameter('/tmp/watchdog/critical'),
            'status_topic': MockParameter('/watchdog/system/status'),
            'camera_status_topic': MockParameter('/tmp/watchdog/camera_is_live'),
            'subscription_qos_depth': MockParameter(10),
            'publisher_qos_depth': MockParameter(10),
            
            # New sanity checking parameters
            'sanity_warning_topic': MockParameter('/watchdog/sanity/warnings'),
            'sensor_health_topic': MockParameter('/watchdog/sanity/sensor_health'),
            'sanity_summary_topic': MockParameter('/watchdog/sanity/summary'),
            'sanity_check_enabled': MockParameter(True),
            'sanity_check_interval': MockParameter(0.1),
        }
        
        self.mock_node.get_parameter.side_effect = lambda name: self.mock_parameters.get(name, MockParameter(0))
    
    @patch('watchdog.watchdog_node.MESSAGES_AVAILABLE', True)
    def test_watchdog_node_initialization_with_sanity_checking(self):
        """Test WatchdogNode initialization with sanity checking enabled."""
        # Import after mocking
        from watchdog.watchdog_node import WatchdogNode
        
        # Create node instance
        node = WatchdogNode()
        
        # Verify sanity checking was initialized
        self.mock_sanity_checker_class.assert_called_once()
        self.mock_sanity_checker.register_validator.assert_called()
        
        # Check that all validators were registered
        validator_calls = self.mock_sanity_checker.register_validator.call_args_list
        registered_sensors = [call[0][0] for call in validator_calls]
        
        expected_sensors = ['lidar', 'battery', 'camera', 'odometry']
        for sensor in expected_sensors:
            self.assertIn(sensor, registered_sensors)
    
    @patch('watchdog.watchdog_node.MESSAGES_AVAILABLE', False)
    def test_watchdog_node_fallback_without_messages(self):
        """Test WatchdogNode graceful fallback when messages aren't available."""
        # Import after mocking
        from watchdog.watchdog_node import WatchdogNode
        
        # Create node instance
        node = WatchdogNode()
        
        # Should still initialize sanity checker
        self.mock_sanity_checker_class.assert_called_once()
    
    def test_sanity_checking_disabled(self):
        """Test WatchdogNode with sanity checking disabled."""
        # Disable sanity checking
        self.mock_parameters['sanity_check_enabled'] = MockParameter(False)
        
        # Import after mocking
        from watchdog.watchdog_node import WatchdogNode
        
        # Create node instance
        node = WatchdogNode()
        
        # Sanity checker should not be initialized
        self.mock_sanity_checker_class.assert_not_called()
    
    def test_sensor_callback_integration(self):
        """Test that sensor callbacks store data for sanity checking."""
        # Import after mocking
        from watchdog.watchdog_node import WatchdogNode
        
        # Create node instance
        node = WatchdogNode()
        
        # Create mock sensor messages
        mock_lidar_msg = MagicMock()
        mock_camera_msg = MagicMock()
        
        # Call sensor callbacks
        node._lidar_callback(mock_lidar_msg)
        node._camera_callback(mock_camera_msg)
        
        # Check that messages were stored
        self.assertTrue(hasattr(node, '_last_lidar_msg'))
        self.assertTrue(hasattr(node, '_last_camera_msg'))
        self.assertEqual(node._last_lidar_msg, mock_lidar_msg)
        self.assertEqual(node._last_camera_msg, mock_camera_msg)
    
    @patch('watchdog.watchdog_node.MESSAGES_AVAILABLE', True)
    def test_publisher_setup_with_custom_messages(self):
        """Test publisher setup with custom messages available."""
        # Mock message classes
        with patch('watchdog.watchdog_node.SanityWarning') as mock_warning, \
             patch('watchdog.watchdog_node.SensorHealth') as mock_health, \
             patch('watchdog.watchdog_node.SanitySummary') as mock_summary:
            
            # Import after mocking
            from watchdog.watchdog_node import WatchdogNode
            
            # Create node instance
            node = WatchdogNode()
            
            # Check that publishers were created with custom message types
            create_publisher_calls = self.mock_node.create_publisher.call_args_list
            
            # Find sanity checking publisher calls
            sanity_publishers = [call for call in create_publisher_calls 
                               if len(call[0]) > 1 and 'sanity' in str(call[0][1])]
            
            # Should have created sanity checking publishers
            self.assertGreater(len(sanity_publishers), 0)
    
    @patch('watchdog.watchdog_node.MESSAGES_AVAILABLE', False)
    def test_publisher_setup_with_string_fallback(self):
        """Test publisher setup with String message fallback."""
        # Import after mocking
        from watchdog.watchdog_node import WatchdogNode
        
        # Create node instance
        node = WatchdogNode()
        
        # Check that publishers were created
        create_publisher_calls = self.mock_node.create_publisher.call_args_list
        
        # Should have created publishers (with String type fallback)
        self.assertGreater(len(create_publisher_calls), 0)
    
    def test_sanity_check_timer_setup(self):
        """Test that sanity check timer is properly set up."""
        # Import after mocking
        from watchdog.watchdog_node import WatchdogNode
        
        # Create node instance
        node = WatchdogNode()
        
        # Check that timer was created
        create_timer_calls = self.mock_node.create_timer.call_args_list
        
        # Should include sanity check timer
        timer_found = False
        for call in create_timer_calls:
            if len(call[0]) >= 2:
                interval, callback = call[0][0], call[0][1]
                if interval == 0.1 and callback.__name__ == '_run_sanity_checks':
                    timer_found = True
                    break
        
        self.assertTrue(timer_found, "Sanity check timer not found")
    
    def test_run_sanity_checks_method(self):
        """Test the _run_sanity_checks method."""
        # Import after mocking
        from watchdog.watchdog_node import WatchdogNode
        
        # Create node instance
        node = WatchdogNode()
        
        # Set up some mock data
        node._last_lidar_msg = MagicMock()
        node.battery_voltage = 11.5
        node.motor_velocity = 2.0
        node.motor_angular_velocity = 0.5
        
        # Mock validation results
        mock_results = [MagicMock()]
        mock_results[0].is_valid = False
        mock_results[0].description = "Test anomaly"
        mock_results[0].suggested_action = "Test action"
        mock_results[0].severity = 3
        
        self.mock_sanity_checker.validate_sensor_data.return_value = mock_results
        
        # Run sanity checks
        node._run_sanity_checks()
        
        # Check that validation was called
        self.mock_sanity_checker.validate_sensor_data.assert_called()
    
    def test_error_handling_in_sanity_checks(self):
        """Test error handling in sanity check execution."""
        # Import after mocking
        from watchdog.watchdog_node import WatchdogNode
        
        # Create node instance
        node = WatchdogNode()
        
        # Make sanity checker raise an exception
        self.mock_sanity_checker.validate_sensor_data.side_effect = Exception("Test exception")
        
        # Run sanity checks - should not crash
        try:
            node._run_sanity_checks()
        except Exception as e:
            self.fail(f"_run_sanity_checks raised an exception: {e}")
        
        # Should have logged the error
        self.mock_node.get_logger.return_value.error.assert_called()
    
    def test_status_message_includes_sanity_info(self):
        """Test that status message includes sanity check information."""
        # Import after mocking
        from watchdog.watchdog_node import WatchdogNode
        
        # Create node instance
        node = WatchdogNode()
        
        # Mock health summary
        mock_health_summary = {
            'lidar': {'health_score': 95.0, 'active_anomalies': []},
            'battery': {'health_score': 85.0, 'active_anomalies': ['LOW_VOLTAGE']}
        }
        self.mock_sanity_checker.get_health_summary.return_value = mock_health_summary
        
        # Generate status message
        status = node._generate_status_message()
        
        # Should include sanity check information
        self.assertIn("Sanity Check", status)
        self.assertIn("2 sensors monitored", status)


class TestMockSensorData(unittest.TestCase):
    """Test scenarios with mock sensor data containing known anomalies."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.mock_node = MagicMock()
        self.mock_node.get_parameter.return_value.get_parameter_value.return_value.double_value = 1.0
    
    def test_lidar_anomaly_scenario(self):
        """Test LiDAR data with known anomalies."""
        from watchdog.validators.lidar_validator import LiDARValidator
        
        validator = LiDARValidator(self.mock_node)
        
        # Create LiDAR data with out-of-range readings
        mock_msg = MagicMock()
        mock_msg.ranges = [-1.0, 0.05, 15.0] * 200  # Many out-of-range readings
        mock_msg.range_min = 0.1
        mock_msg.range_max = 12.0
        mock_msg.header.stamp.sec = 1234567890
        mock_msg.header.stamp.nanosec = 123456789
        
        results = validator.validate(mock_msg)
        
        # Should detect anomalies
        self.assertGreater(len(results), 0)
        out_of_range_detected = any(r.anomaly_type == "OUT_OF_RANGE" for r in results)
        self.assertTrue(out_of_range_detected)
    
    def test_battery_critical_scenario(self):
        """Test battery data with critical conditions."""
        from watchdog.validators.battery_validator import BatteryValidator
        
        # Set up critical voltage threshold
        self.mock_node.get_parameter.side_effect = lambda name: {
            'sanity.battery.voltage_critical': MockParameter(10.5),
            'sanity.battery.voltage_min': MockParameter(10.0),
        }.get(name, MockParameter(1.0))
        
        validator = BatteryValidator(self.mock_node)
        
        # Create critical battery data
        critical_data = {
            'voltage': 10.0,  # Critical voltage
            'current': 35.0,  # High current
            'temperature': 90.0  # High temperature
        }
        
        results = validator.validate(critical_data)
        
        # Should detect multiple critical issues
        self.assertGreater(len(results), 0)
        
        # Check for critical voltage
        critical_detected = any(r.anomaly_type == "CRITICAL_VOLTAGE" for r in results)
        self.assertTrue(critical_detected)
    
    def test_camera_quality_scenario(self):
        """Test camera data with quality issues."""
        from watchdog.validators.camera_validator import CameraValidator
        
        validator = CameraValidator(self.mock_node)
        
        # Create low quality image
        mock_msg = MagicMock()
        mock_msg.width = 320  # Low resolution
        mock_msg.height = 240
        mock_msg.encoding = "rgb8"
        mock_msg.step = 960
        mock_msg.data = bytearray([10] * (320 * 240 * 3))  # Very dark image
        mock_msg.header.stamp.sec = 1234567890
        mock_msg.header.stamp.nanosec = 123456789
        
        results = validator.validate(mock_msg)
        
        # Should detect quality issues
        self.assertGreater(len(results), 0)
    
    def test_odometry_excessive_values_scenario(self):
        """Test odometry data with excessive values."""
        from watchdog.validators.odometry_validator import OdometryValidator
        
        # Set up velocity limits
        self.mock_node.get_parameter.side_effect = lambda name: {
            'sanity.odometry.max_linear_velocity': MockParameter(5.0),
            'sanity.odometry.max_angular_velocity': MockParameter(2.0),
        }.get(name, MockParameter(1.0))
        
        validator = OdometryValidator(self.mock_node)
        
        # Create excessive velocity data
        excessive_data = {
            'linear_velocity': 7.0,  # Above limit
            'angular_velocity': 3.0  # Above limit
        }
        
        results = validator.validate(excessive_data)
        
        # Should detect excessive velocities
        self.assertGreater(len(results), 0)
        
        linear_detected = any(r.anomaly_type == "EXCESSIVE_LINEAR_VELOCITY" for r in results)
        angular_detected = any(r.anomaly_type == "EXCESSIVE_ANGULAR_VELOCITY" for r in results)
        
        self.assertTrue(linear_detected)
        self.assertTrue(angular_detected)


if __name__ == '__main__':
    # Set up test suite
    unittest.main(verbosity=2)