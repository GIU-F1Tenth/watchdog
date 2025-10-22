"""
Deprecated duplicate node file.

This file previously exposed an alternate WatchdogNode implementation.
The project now provides a single canonical entry point:

    - console script: watchdog_node
    - module path: watchdog.watchdog_node:main

If you are importing this module, switch to:

    from watchdog.watchdog_node import WatchdogNode

This module will raise at import time to avoid silent duplication.
"""

raise RuntimeError(
        "watchdog/watchdogNode.py is deprecated. Use watchdog.watchdog_node instead."
)
        status_msg += f"Angular Velocity: {self.motor_angularvelocity}\n"

        # LiDAR
        if self.lidar_status is None:
            status_msg += "LiDAR: No data\n"
        else:
            status_msg += f"LiDAR: {'Operational' if self.lidar_status else 'Faulty'}\n"

        # VESC
        if self.vesc_status is None:
            status_msg += "VESC: No data\n"
        else:
            status_msg += f"VESC: {'Operational' if self.vesc_status else 'Faulty'}\n"

        return status_msg

    def generate_warning_message(self):
        # Create a warning string with all health parameters
        warning_msg = "Warning:\n"

        # Check battery status
        if self.battery_voltage < 0.0:
            warning_msg += "Battery Voltage is negative\n"

        if self.battery_voltage < 9.0:
            warning_msg += f"Battery Voltage: {self.battery_voltage}V\n"

        if self.battery_voltage > 52.0:
            warning_msg += f"Battery Voltage: {self.battery_voltage}V (Out of range)\n"

        # Check temperature — order matters
        if self.motor_temperature > 100.0:
            warning_msg += f"Motor Temperature: {self.motor_temperature}°C (Critical)\n"
        elif self.motor_temperature > 90.0:
            warning_msg += f"Motor Temperature: {self.motor_temperature}°C (Very High – More reduction in velocity)\n"
        elif self.motor_temperature > 80.0:
            warning_msg += f"Motor Temperature: {self.motor_temperature}°C (High – Velocity reduced by 15%)\n"

        # Motor
        if not self.motor_status:
            warning_msg += "Motor: Idle\n"

        # LiDAR
        warning_msg += f"LiDAR: {'Operational' if self.lidar_status else 'Faulty'}\n"

        # VESC (only include if you track its status)
        if hasattr(self, 'vesc_status'):
            warning_msg += f"VESC: {'Operational' if self.vesc_status else 'Faulty'}\n"

        return warning_msg

    def publish_camera_status(self):
        msg = Bool()
        msg.data = self.camera_is_live
        self.status_publisher.publish(msg)

        if not self.camera_is_live:
            self.get_logger().warn(f"{red}Camera is not live{reset}")

        # self.get_logger().info("Camera is live")
        # self.get_logger().info(f"Camera is live: {self.camera_is_live}")


def main(args=None):
    rclpy.init(args=args)
    node = WatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# 9V – 52V (Safe for 3S to 12S LiPo/LiIon). Voltage spikes may not exceed 60V

# Start of Thermal Throttling: Typically set around 80°C.
# Complete Shutdown Threshold: Commonly configured at 100°C. default 15%
