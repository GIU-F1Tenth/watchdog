import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32



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
        self.sub_current = self.create_subscription(
            Float32,
            '/vesc/current',
            self.subCurrent_callback,
            10)
        self.sub_current
        
        # Subscriber
        self.sub_voltage = self.create_subscription(
            Float32,
            '/vesc/voltage',
            self.subVoltage_callback,
            10)
        self.sub_voltage  # Prevent unused variable warning
        
        # Subscriber
        self.sub_temperature= self.create_subscription(
            Float32,
            '/vesc/temperature',
            self.subTemperature_callback,
            10)
        self.sub_temperature # Prevent unused variable warning # Subscriber
        
        self.sub_velocity= self.create_subscription(
            Float32,
            '/cmd_vel',
            self.subvelocity_callback,
            10)
        self.sub_velocity # Prevent unused variable warning 
        
        
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
        
    def subCurrent_callback(self, msg):
        self.battery_voltage = msg.data
        self.get_logger().info(f'Battery Voltage: "{self.battery_voltage}"')
        self.check_battery_status()
        self.publish_status_message()
    
    def subVoltage_callback(self, msg):
        pass   
    
    def subTemperature_callback(self, msg):
        self.battery_percentage = msg.data
        self.get_logger().info(f'Battery Percentage: "{self.battery_percentage}"')
        self.check_battery_status()
        self.publish_status_message()
    
    def subvelocity_callback(self, msg):
        # Check if the motor is responding to velocity commands
        if msg.linear.x == 0.0 and msg.angular.z == 0.0:
            self.motor_status = False
        else:
            self.motor_status = True
        self.get_logger().info(f'Motor Status: {"Operational" if self.motor_status else "Not Responding"}')
        self.publish_status_message()
        
        
        
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
