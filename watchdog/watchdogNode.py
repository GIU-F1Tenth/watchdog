import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String



class WatchdogNode(Node):
    def __init__(self):
        super().__init__('WatchdogNode')
        
        
        # Internal state variables
        self.battery_voltage = None
        self.battery_percentage = None
        self.lidar_status = None
        self.motor_status = True  # Assuming motor is healthy initially
        self.ssh_connected = self.check_ssh_connection()
        
        # Subscriber
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning
        
        self.get_logger().info(" Watchdog has been started")
        
        # Publisher boolean
        self.publisher_ = self.create_publisher(bool, 'watchdog', 10)
        
        # Publisher
        self.publisher_ = self.create_publisher(String, '/system/status', 10)
        
        # Timer to publish messages
        self.timer = self.create_timer(1.0, self.timer_callback)
        
      
    
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, ROS 2!'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
    
    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')
        
    def generate_status_message(self):
        # Create a status string with all health parameters
        status_msg = "System Status:\n"
        
        # Check battery status
        if self.battery_voltage is None or self.battery_percentage is None:
            status_msg += "Battery: No data\n"
        else:
            status_msg += f"Battery Voltage: {self.battery_voltage}V\n"
            status_msg += f"Battery Percentage: {self.battery_percentage}%\n"
        
        # Check temperature (this can be part of another sensor or data stream)
        # For simplicity, assume it's stable unless you have a separate sensor for temperature.
        status_msg += "Temperature: Stable\n"
        
        # Check motor status (motor doesn't respond)
        if not self.motor_status:
            status_msg += "Motor: Not Responding\n"
        else:
            status_msg += "Motor: Operational\n"
        
        # Check LiDAR status
        if self.lidar_status is None:
            status_msg += "LiDAR: No data\n"
        else:
            status_msg += f"LiDAR: {'Operational' if self.lidar_status else 'Faulty'}\n"
        
        # Check SSH connection status
        status_msg += f"SSH Connection: {'Active' if self.ssh_connected else 'Inactive'}\n"

        # Communication lost check (can implement based on received messages)
        # For example, if no `cmd_vel` message has been received for a certain period
        status_msg += "Communication: Active\n"  # Customize this condition as needed

        return status_msg


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
