#!/usr/bin/env python3

"""
F1TENTH Watchdog Node

A comprehensive system monitoring and safety watchdog for F1TENTH autonomous racing vehicles.
This node monitors critical system components including battery voltage, motor temperature,
LiDAR functionality, camera status, and motor operation to ensure safe vehicle operation.

Author: Fam Shihata <fam@awadlouis.com>
Author: Mohammed Azab <mo7ammed3zab@outlook.com>
License: MIT
Version: 1.0.0
"""

import rclpy
import time
from typing import Optional
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_msgs.msg import Bool, String
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from vesc_msgs.msg import VescStateStamped

# Import sanity checking components
try:
    # Try absolute import first (for installed package)
    from watchdog.sanity_checker import SanityChecker
    from watchdog.validators import LiDARValidator, BatteryValidator, CameraValidator, OdometryValidator
except ImportError:
    try:
        # Try relative import (for development)
        from .sanity_checker import SanityChecker
        from .validators import LiDARValidator, BatteryValidator, CameraValidator, OdometryValidator
    except ImportError:
        # Fallback for when running as standalone script
        from sanity_checker import SanityChecker
        from validators import LiDARValidator, BatteryValidator, CameraValidator, OdometryValidator

# Custom watchdog messages are provided via the shared interfaces repo.
# In this package we publish simple String summaries for compatibility.


class WatchdogNode(Node):
    """
    F1TENTH Watchdog Node for system monitoring and safety.

    This node continuously monitors various system components and publishes
    status information and critical alerts to ensure safe vehicle operation.
    """

    def __init__(self):
        """Initialize the watchdog node with parameters and subscriptions."""
        # Automatically declare parameters provided via YAML, including optional vehicle
        # configuration fields carried over from F1TENTH-specific configs.
        super().__init__(
            'watchdog_node',
            automatically_declare_parameters_from_overrides=True
        )

        # Declare and get parameters
        self._declare_parameters()
        self._load_parameters()

        # Initialize state variables
        self._initialize_state_variables()

        # Setup ROS2 subscriptions and publishers
        self._setup_subscriptions()
        self._setup_publishers()
        self._setup_timers()

        self.get_logger().info("F1TENTH Watchdog Node has been started successfully")

    def _declare_parameters(self) -> None:
        """Declare all ROS2 parameters with default values."""
        # Voltage thresholds
        self.declare_parameter('critical_voltage', 12.0)
        self.declare_parameter('min_voltage', 9.0)
        self.declare_parameter('max_voltage', 52.0)

        # Temperature thresholds
        self.declare_parameter('temp_warning_start', 80.0)
        self.declare_parameter('temp_warning_high', 90.0)
        self.declare_parameter('temp_critical', 100.0)

        # Timeout settings
        self.declare_parameter('lidar_timeout', 0.5)
        self.declare_parameter('camera_timeout', 0.5)
        self.declare_parameter('lidar_critical_timeout', 1.0)

        # Timer intervals
        self.declare_parameter('status_publish_interval', 1.0)
        self.declare_parameter('camera_check_interval', 0.5)
        self.declare_parameter('critical_check_interval', 0.5)

        # Topic names
        self.declare_parameter('core_topic', '/sensors/core')
        self.declare_parameter(
            'camera_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('lidar_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('critical_topic', '/tmp/watchdog/critical')
        self.declare_parameter('status_topic', '/watchdog/system/status')
        self.declare_parameter('camera_status_topic',
                               '/tmp/watchdog/camera_is_live')

        # Sanity checking topics
        self.declare_parameter('sanity_warning_topic', '/watchdog/sanity/warnings')
        self.declare_parameter('sensor_health_topic', '/watchdog/sanity/sensor_health')
        self.declare_parameter('sanity_summary_topic', '/watchdog/sanity/summary')

        # Sanity checking parameters
        self.declare_parameter('sanity_check_enabled', True)
        self.declare_parameter('sanity_check_interval', 0.1)

        # QoS settings
        self.declare_parameter('subscription_qos_depth', 10)
        self.declare_parameter('publisher_qos_depth', 10)

    def _load_parameters(self) -> None:
        """Load parameters from ROS2 parameter server."""
        # Voltage thresholds
        self.critical_voltage = self.get_parameter(
            'critical_voltage').get_parameter_value().double_value
        self.min_voltage = self.get_parameter(
            'min_voltage').get_parameter_value().double_value
        self.max_voltage = self.get_parameter(
            'max_voltage').get_parameter_value().double_value

        # Temperature thresholds
        self.temp_warning_start = self.get_parameter(
            'temp_warning_start').get_parameter_value().double_value
        self.temp_warning_high = self.get_parameter(
            'temp_warning_high').get_parameter_value().double_value
        self.temp_critical = self.get_parameter(
            'temp_critical').get_parameter_value().double_value

        # Timeout settings
        self.lidar_timeout = self.get_parameter(
            'lidar_timeout').get_parameter_value().double_value
        self.camera_timeout = self.get_parameter(
            'camera_timeout').get_parameter_value().double_value
        self.lidar_critical_timeout = self.get_parameter(
            'lidar_critical_timeout').get_parameter_value().double_value

        # Timer intervals
        self.status_publish_interval = self.get_parameter(
            'status_publish_interval').get_parameter_value().double_value
        self.camera_check_interval = self.get_parameter(
            'camera_check_interval').get_parameter_value().double_value
        self.critical_check_interval = self.get_parameter(
            'critical_check_interval').get_parameter_value().double_value

        # Topic names
        self.core_topic = self.get_parameter(
            'core_topic').get_parameter_value().string_value
        self.camera_topic = self.get_parameter(
            'camera_topic').get_parameter_value().string_value
        self.lidar_topic = self.get_parameter(
            'lidar_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter(
            'odom_topic').get_parameter_value().string_value
        self.critical_topic = self.get_parameter(
            'critical_topic').get_parameter_value().string_value
        self.status_topic = self.get_parameter(
            'status_topic').get_parameter_value().string_value
        self.camera_status_topic = self.get_parameter(
            'camera_status_topic').get_parameter_value().string_value

        # Sanity checking topics
        self.sanity_warning_topic = self.get_parameter(
            'sanity_warning_topic').get_parameter_value().string_value
        self.sensor_health_topic = self.get_parameter(
            'sensor_health_topic').get_parameter_value().string_value
        self.sanity_summary_topic = self.get_parameter(
            'sanity_summary_topic').get_parameter_value().string_value

        # Sanity checking parameters
        self.sanity_check_enabled = self.get_parameter(
            'sanity_check_enabled').get_parameter_value().bool_value
        self.sanity_check_interval = self.get_parameter(
            'sanity_check_interval').get_parameter_value().double_value

        # QoS settings
        self.qos_depth = self.get_parameter(
            'subscription_qos_depth').get_parameter_value().integer_value
        self.pub_qos_depth = self.get_parameter(
            'publisher_qos_depth').get_parameter_value().integer_value

    def _initialize_state_variables(self) -> None:
        """Initialize all internal state variables."""
        # Battery and motor state
        self.battery_voltage: float = 0.0
        self.motor_current: float = 0.0
        self.motor_temperature: float = 0.0
        self.motor_velocity: float = 0.0
        self.motor_angular_velocity: float = 0.0

        # Component status
        self.lidar_status: Optional[bool] = None
        self.motor_status: bool = False
        self.vesc_status: Optional[bool] = None
        self.camera_is_live: bool = True

        # Timing variables
        self.lidar_previous_time: Optional[Time] = None
        self.last_lidar_msg_time: Optional[Time] = None
        self.last_image_time: float = time.time()

        # Critical state
        self.is_critical: bool = False

        # Initialize sanity checker if enabled
        self.sanity_checker: Optional[SanityChecker] = None
        if self.sanity_check_enabled:
            self._initialize_sanity_checker()

    def _setup_subscriptions(self) -> None:
        """Setup ROS2 subscriptions for sensor data."""
        self.core_subscription = self.create_subscription(
            VescStateStamped,
            self.core_topic,
            self._core_callback,
            self.qos_depth
        )

        self.camera_subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self._camera_callback,
            self.qos_depth
        )

        self.lidar_subscription = self.create_subscription(
            LaserScan,
            self.lidar_topic,
            self._lidar_callback,
            self.qos_depth
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_callback,
            self.qos_depth
        )

    def _setup_publishers(self) -> None:
        """Setup ROS2 publishers for status and alerts."""
        self.critical_publisher = self.create_publisher(
            Bool,
            self.critical_topic,
            self.pub_qos_depth
        )

        self.status_publisher = self.create_publisher(
            String,
            self.status_topic,
            self.pub_qos_depth
        )

        self.camera_status_publisher = self.create_publisher(
            Bool,
            self.camera_status_topic,
            self.pub_qos_depth
        )

        # Sanity checking publishers
        if self.sanity_check_enabled:
            # Publish human-readable summaries using std_msgs/String
            self.sanity_warning_publisher = self.create_publisher(
                String,
                self.sanity_warning_topic,
                self.pub_qos_depth
            )

            self.sensor_health_publisher = self.create_publisher(
                String,
                self.sensor_health_topic,
                self.pub_qos_depth
            )

            self.sanity_summary_publisher = self.create_publisher(
                String,
                self.sanity_summary_topic,
                self.pub_qos_depth
            )

    def _setup_timers(self) -> None:
        """Setup periodic timers for status updates and checks."""
        self.status_timer = self.create_timer(
            self.status_publish_interval,
            self._publish_status_message
        )

        self.camera_check_timer = self.create_timer(
            self.camera_check_interval,
            self._check_camera_status
        )

        self.critical_timer = self.create_timer(
            self.critical_check_interval,
            self._publish_critical_status
        )

        self.camera_status_timer = self.create_timer(
            self.critical_check_interval,
            self._publish_camera_status
        )

        # Sanity checking timer
        if self.sanity_check_enabled:
            self.sanity_check_timer = self.create_timer(
                self.sanity_check_interval,
                self._run_sanity_checks
            )

    def _odom_callback(self, msg: Odometry) -> None:
        """
        Process odometry messages to monitor motor status.

        Args:
            msg: Odometry message containing velocity information
        """
        self.motor_velocity = msg.twist.twist.linear.x
        self.motor_angular_velocity = msg.twist.twist.angular.z

        # Determine if motor is active based on velocity
        self.motor_status = not (
            self.motor_velocity == 0.0 and self.motor_angular_velocity == 0.0
        )

    def _camera_callback(self, msg: Image) -> None:
        """
        Process camera messages to monitor camera status.

        Args:
            msg: Image message from camera
        """
        self.last_image_time = time.time()
        
        # Store latest camera message for sanity checking
        if self.sanity_check_enabled:
            self._last_camera_msg = msg

    def _check_camera_status(self) -> None:
        """Check if camera is providing live data within timeout period."""
        current_time = time.time()

        if current_time - self.last_image_time > self.camera_timeout:
            self.camera_is_live = False
        else:
            self.camera_is_live = True

    def _core_callback(self, msg: VescStateStamped) -> None:
        """
        Process VESC core messages to monitor battery and motor status.

        Args:
            msg: VESC state message containing battery and motor data
        """
        self.battery_voltage = msg.state.voltage_input
        self.motor_current = msg.state.current_motor
        self.motor_temperature = msg.state.temp_motor

        # Check for critical conditions
        if self.battery_voltage < self.critical_voltage:
            self.get_logger().warn(
                f"Battery voltage critically low: {self.battery_voltage:.2f}V"
            )
            self.is_critical = True

        if self.motor_temperature > self.temp_warning_start:
            self.is_critical = True
            self.get_logger().warn(
                f"High motor temperature detected: {self.motor_temperature:.1f}°C"
            )

    def _lidar_callback(self, msg: LaserScan) -> None:
        """
        Process LiDAR messages to monitor LiDAR status and timing.

        Args:
            msg: LaserScan message from LiDAR sensor
        """
        self.last_lidar_msg_time = msg.header.stamp

        # Store latest LiDAR message for sanity checking
        if self.sanity_check_enabled:
            self._last_lidar_msg = msg

        if self.lidar_previous_time is None:
            self.lidar_previous_time = self.last_lidar_msg_time
            return

        # Calculate elapsed time since last message
        elapsed_time = (
            Time.from_msg(self.last_lidar_msg_time) -
            Time.from_msg(self.lidar_previous_time)
        ).nanoseconds * 1e-9

        self.lidar_previous_time = self.last_lidar_msg_time

        # Update LiDAR status based on timing
        if elapsed_time > self.lidar_timeout:
            self.lidar_status = False
        else:
            self.lidar_status = True

        # Check for critical timeout
        if elapsed_time > self.lidar_critical_timeout:
            self.is_critical = True

    def _initialize_sanity_checker(self) -> None:
        """Initialize the sanity checker with validators."""
        try:
            # Create config dictionary from node parameters
            config = {}
            param_names = [
                'sanity_checks_enabled', 'sanity_check_interval', 'warning_history_size',
                'health_update_interval', 'cross_sensor_validation_enabled',
                'lidar_camera_correlation_threshold', 'odometry_visual_odometry_threshold',
                'position_consistency_threshold', 'temporal_sync_threshold',
                'environment_consistency_window', 'outlier_detection_sigma',
                # LiDAR parameters
                'lidar_min_range', 'lidar_max_range', 'lidar_noise_threshold',
                'lidar_dead_zone_threshold', 'lidar_min_valid_points',
                # Battery parameters  
                'min_voltage', 'max_voltage', 'critical_voltage', 'motor_current_max',
                'max_temp', 'battery_max_voltage_change_rate', 'battery_trend_window_size',
                # Camera parameters
                'camera_min_brightness', 'camera_max_brightness', 'camera_blur_threshold',
                'camera_corruption_threshold', 'camera_min_resolution',
                # Odometry parameters
                'odometry_max_linear_velocity', 'odometry_max_angular_velocity',
                'odometry_max_linear_acceleration', 'odometry_max_angular_acceleration'
            ]
            
            for param_name in param_names:
                try:
                    param_value = self.get_parameter(param_name).value
                    config[param_name] = param_value
                except:
                    # Parameter not found, skip
                    pass
            
            self.sanity_checker = SanityChecker(self, config)
            
            # Register validators with config dictionaries
            if config.get('lidar_sanity_enabled', True):
                self.sanity_checker.register_validator(LiDARValidator(config))
            if config.get('battery_sanity_enabled', True):
                self.sanity_checker.register_validator(BatteryValidator(config))
            if config.get('camera_sanity_enabled', True):
                self.sanity_checker.register_validator(CameraValidator(config))
            if config.get('odometry_sanity_enabled', True):
                self.sanity_checker.register_validator(OdometryValidator(config))
            
            self.get_logger().info("Sanity checker initialized with all validators")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize sanity checker: {e}")
            self.sanity_check_enabled = False

    def _run_sanity_checks(self) -> None:
        """Run sanity checks on all sensor data."""
        if not self.sanity_checker:
            return

        try:
            # Check LiDAR data if available
            if hasattr(self, '_last_lidar_msg'):
                results = self.sanity_checker.validate_sensor_data('lidar', self._last_lidar_msg)
                self._process_sanity_results('lidar', results)
                # Update cross-sensor data
                self.sanity_checker.update_cross_sensor_data('lidar', self._last_lidar_msg)

            # Check battery/VESC data
            if self.battery_voltage > 0:
                vesc_data = {
                    'voltage': self.battery_voltage,
                    'current': self.motor_current,
                    'temperature': self.motor_temperature
                }
                results = self.sanity_checker.validate_sensor_data('battery', vesc_data)
                self._process_sanity_results('battery', results)
                # Update cross-sensor data
                self.sanity_checker.update_cross_sensor_data('battery', vesc_data)

            # Check camera data if available
            if hasattr(self, '_last_camera_msg'):
                results = self.sanity_checker.validate_sensor_data('camera', self._last_camera_msg)
                self._process_sanity_results('camera', results)
                # Update cross-sensor data
                self.sanity_checker.update_cross_sensor_data('camera', self._last_camera_msg)

            # Check odometry data
            if self.motor_velocity != 0 or self.motor_angular_velocity != 0:
                odom_data = {
                    'linear_velocity': self.motor_velocity,
                    'angular_velocity': self.motor_angular_velocity
                }
                results = self.sanity_checker.validate_sensor_data('odometry', odom_data)
                self._process_sanity_results('odometry', results)
                # Update cross-sensor data
                self.sanity_checker.update_cross_sensor_data('odometry', odom_data)

            # Perform cross-sensor validation
            cross_sensor_result = self.sanity_checker.validate_cross_sensor_consistency()
            if cross_sensor_result:
                self._process_sanity_results('cross_sensor', cross_sensor_result)

            # Publish sensor health summary
            self._publish_sensor_health()
            
        except Exception as e:
            self.get_logger().error(f"Error during sanity checks: {e}")

    def _process_sanity_results(self, sensor_name: str, results: list) -> None:
        """Process sanity check results and publish warnings if needed."""
        for result in results:
            if not result.is_valid:
                # Create warning message as string
                warning_msg = String()
                warning_content = f"SANITY WARNING - {sensor_name.upper()}: {result.description}"
                if result.suggested_action:
                    warning_content += f" | Action: {result.suggested_action}"
                warning_msg.data = warning_content
                
                # Publish warning
                self.sanity_warning_publisher.publish(warning_msg)
                
                # Log warning
                log_msg = f"SANITY WARNING - {sensor_name.upper()}: {result.description}"
                self.get_logger().warn(log_msg)
                
                # Set critical if severity is high
                if hasattr(result, 'severity') and result.severity >= 3:  # HIGH or CRITICAL
                    self.is_critical = True

    def _publish_sensor_health(self) -> None:
        """Publish sensor health summary."""
        if not self.sanity_checker:
            return
            
        try:
            health_summary = self.sanity_checker.get_health_summary()
            
            # Create health message as string summary
            health_msg = String()
            health_content = f"Sensor Health Summary: {len(health_summary)} sensors monitored"
            for sensor, health in health_summary.items():
                health_content += f" | {sensor}: {health['health_score']:.1f}% "
                if health.get('active_anomalies'):
                    health_content += f"({len(health['active_anomalies'])} anomalies)"
            health_msg.data = health_content
            self.sensor_health_publisher.publish(health_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing sensor health: {e}")

    def _generate_status_message(self) -> str:
        """
        Generate comprehensive system status message.

        Returns:
            Formatted status string with all system information
        """
        status_lines = ["F1TENTH System Status:"]

        # Battery status
        if self.battery_voltage == 0.0:
            status_lines.append("Battery: No data available")
        else:
            battery_status = "Normal"
            if self.battery_voltage < self.min_voltage:
                battery_status = "LOW"
            elif self.battery_voltage > self.max_voltage:
                battery_status = "HIGH"
            elif self.battery_voltage < self.critical_voltage:
                battery_status = "CRITICAL"

            status_lines.append(
                f"Battery: {self.battery_voltage:.2f}V ({battery_status})")

        # Temperature status
        temp_status = "Normal"
        if self.motor_temperature > self.temp_critical:
            temp_status = "CRITICAL"
        elif self.motor_temperature > self.temp_warning_high:
            temp_status = "Very High"
        elif self.motor_temperature > self.temp_warning_start:
            temp_status = "High"

        status_lines.append(
            f"Motor Temperature: {self.motor_temperature:.1f}°C ({temp_status})")

        # Motor status
        motor_state = "Active" if self.motor_status else "Idle"
        status_lines.append(f"Motor: {motor_state}")
        status_lines.append(f"Linear Velocity: {self.motor_velocity:.2f} m/s")
        status_lines.append(
            f"Angular Velocity: {self.motor_angular_velocity:.2f} rad/s")

        # LiDAR status
        if self.lidar_status is None:
            status_lines.append("LiDAR: No data available")
        else:
            lidar_state = "Operational" if self.lidar_status else "Faulty"
            status_lines.append(f"LiDAR: {lidar_state}")

        # Camera status
        camera_state = "Live" if self.camera_is_live else "Not responding"
        status_lines.append(f"Camera: {camera_state}")

        # VESC status
        if self.vesc_status is None:
            status_lines.append("VESC: Status unknown")
        else:
            vesc_state = "Operational" if self.vesc_status else "Faulty"
            status_lines.append(f"VESC: {vesc_state}")

        # Sanity checking status
        if self.sanity_check_enabled and self.sanity_checker:
            try:
                health_summary = self.sanity_checker.get_health_summary()
                status_lines.append(f"Sanity Check: {len(health_summary)} sensors monitored")
                
                # Add brief health summary
                for sensor, health in health_summary.items():
                    anomaly_count = len(health.get('active_anomalies', []))
                    if anomaly_count > 0:
                        status_lines.append(f"  {sensor}: {health['health_score']:.0f}% ({anomaly_count} anomalies)")
            except Exception:
                status_lines.append("Sanity Check: Error retrieving status")
        elif self.sanity_check_enabled:
            status_lines.append("Sanity Check: Initializing...")
        else:
            status_lines.append("Sanity Check: Disabled")

        return "\n".join(status_lines)

    def _publish_status_message(self) -> None:
        """Publish comprehensive system status message."""
        msg = String()
        msg.data = self._generate_status_message()
        self.status_publisher.publish(msg)

    def _publish_critical_status(self) -> None:
        """Publish critical system status flag."""
        msg = Bool()
        msg.data = self.is_critical
        self.critical_publisher.publish(msg)

    def _publish_camera_status(self) -> None:
        """Publish camera live status."""
        msg = Bool()
        msg.data = self.camera_is_live
        self.camera_status_publisher.publish(msg)

        if not self.camera_is_live:
            self.get_logger().warn("Camera is not responding within timeout period")


def main(args=None):
    """
    Main entry point for the watchdog node.

    Args:
        args: Command line arguments (optional)
    """
    rclpy.init(args=args)

    try:
        node = WatchdogNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
