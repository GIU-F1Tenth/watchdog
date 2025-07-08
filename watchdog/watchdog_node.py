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


class WatchdogNode(Node):
    """
    F1TENTH Watchdog Node for system monitoring and safety.

    This node continuously monitors various system components and publishes
    status information and critical alerts to ensure safe vehicle operation.
    """

    def __init__(self):
        """Initialize the watchdog node with parameters and subscriptions."""
        super().__init__('watchdog_node')

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
