import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Twist
from rclpy.duration import Duration



class WatchdogNode(Node):
    def __init__(self):
        super().__init__('WatchdogNode')
        
        # Internal state variables
        self.battery_voltage = None
        self.motor_current = None
        self.motor_temperature = None
        self.motor_velocity = None
        self.motor_angularvelocity = None
        self.motor_angularvelocity = None
        self.lidar_status = None
        self.motor_status = True  # Assuming motor is healthy initially
 
        
        
        self.get_logger().info(" Watchdog has been started")
        
        # Subscriber
        
        self.sub_laserLidar = self.create_subscription(
            LaserScan,
            '/scan',
            self.check_lidar_status,
            10
        )

        self.sub_diagnosticLidar = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
            10
        )

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
        self.sub_temperature = self.create_subscription(
            Float32,
            '/vesc/temperature',
            self.subTemperature_callback,
            10)
        self.sub_temperature 
        
        self.sub_velocity = self.create_subscription(
            Twist,
            Twist,
            '/cmd_vel',
            self.subvelocity_callback,
            10)
        self.sub_velocity 
        
        
      
        
      
        # Publisher boolean
        self.publisherStop= self.create_publisher(Bool, '/watchdog/critical', 10)
        
        # Publisher 
        self.publisherWarning= self.create_publisher(String, '/watchdog/warning', 10) 
        
        # Publisher
        self.publisherStatus = self.create_publisher(String, '/system/status', 10)
        

    def check_lidar_status(self):
        # Check if we've missed scan messages for >1 second
        elapsed = self.get_clock().now() - self.last_msg_time
        if elapsed > Duration(seconds=1.0):
            if self.lidar_online:
                self.lidar_online = False
                self.get_logger().warn("LiDAR is OFFLINE — no scan received in >1s")
    
    def diagnostics_callback(self, msg):
        for status in msg.status:
            if 'hokuyo' in status.name.lower() or 'lidar' in status.name.lower():
                self.get_logger().info(f"Status: {status.name}")
                self.get_logger().info(f"Level: {status.level} | Message: {status.message}")
                if status.level > 0:
                    self.get_logger().warn(f"⚠️ Problem detected: {status.message}")
                for kv in status.values:
                    self.get_logger().info(f"  {kv.key}: {kv.value}")


    def subCurrent_callback(self, msg):
        self.motor_current = msg.data
        self.get_logger().info(f'Motor Current: "{self.motor_current}"')


    
    def subVoltage_callback(self, msg):
        pass   
    
    def subTemperature_callback(self, msg):
        self.motor_temperature = msg.data
        self.get_logger().info(f'Motor Temperature: "{self.motor_temperature}"')

    
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
        
        status_msg += "Temperature: Stable\n"
        
        # Motor
        if not self.motor_status:
            status_msg += "Motor: Idel\n"
        else:
            status_msg += "Motor: Operational\n"
        
        # LiDAR
        if self.lidar_status is None:
            status_msg += "LiDAR: No data\n"
        else:
            status_msg += f"LiDAR: {'Operational' if self.lidar_status else 'Faulty'}\n"

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


#9V – 52V (Safe for 3S to 12S LiPo/LiIon). Voltage spikes may not exceed 60V

#Start of Thermal Throttling: Typically set around 80°C.
#Complete Shutdown Threshold: Commonly configured at 100°C. default 15%