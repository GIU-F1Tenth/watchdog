import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from sensor_msgs.msg import LaserScan
from rclpy.time import Time
from nav_msgs.msg import Odometry
from vesc_msgs.msg import VescStateStamped

import config

red = '\033[91m'
yellow = '\033[93m'
reset = '\033[0m'


class WatchdogNode(Node):
    def __init__(self):
        super().__init__('WatchdogNode')
        
        # Internal state variables
        self.lidar_timeout = 1.0  
        self.battery_voltage = 0.0
        self.motor_current = 0.0
        self.motor_temperature = 0.0
        self.motor_velocity = 0.0
        self.motor_angularvelocity = 0.0
        self.lidar_status = None
        self.motor_status = False
        self.vesc_status = None
        self.lidar_previous = None
        self.last_msg_time = None
        self.isCritical = False
 
        
        
        self.get_logger().info(" Watchdog has been started ")
        
        # Subscriber
        
        self.sub_core = self.create_subscription(
            VescStateStamped,
            '/sensors/core',
            self.sub_core_callback,
            10)
        self.sub_core

        self.sub_scanLidar = self.create_subscription(
            LaserScan,
            '/scan',
            self.check_lidar_status,
            10)
        
        self.sub_odom=  self.create_subscription(
            Odometry        ,
            '/odom',
            self.odom_callback,
            10)
        self.sub_odom
        
        # Publisher boolean
        self.publisherStop= self.create_publisher(Bool, '/watchdog/critical', 10)
        
        # Publisher 
        self.publisherWarning= self.create_publisher(String, '/watchdog/warning', 10) 
        
        # Publisher
        self.publisherStatus = self.create_publisher(String, '/watchdog/system/status', 10)

        self.timer_status = self.create_timer(1.0, self.publish_status_message)  # every 1 seconds
        self.timer_warning = self.create_timer(1.0, self.publish_warning)  # every 1 seconds
        self.timer_critical = self.create_timer(1.0, self.publish_critical)  # every 1 seconds


    def odom_callback(self, msg):
        self.motor_velocity = msg.twist.twist.linear.x  # forward velocity
        self.motor_angularvelocity = msg.twist.twist.angular.z  # rotation (yaw rate)
        
        if self.motor_velocity == 0.0 and self.motor_angularvelocity == 0.0:
            self.motor_status = False
        else:
            self.motor_status = True
        
        #self.get_logger().info(f"Linear Velocity: {self.motor_velocity} m/s")
        #self.get_logger().info(f"Angular Velocity: {self.motor_angularvelocity} rad/s")

    def sub_core_callback(self, msg: VescStateStamped):
        self.battery_voltage = msg.state.voltage_input
        self.motor_current = msg.state.current_motor
        self.motor_temperature = msg.state.temp_motor
        # Implement safety checks
        if self.battery_voltage < config.criticalVoltage:
            self.get_logger().warn(f"Battery voltage out of range: {self.battery_voltage}V")
            self.isCritical=True
        if self.motor_temperature > 80.0:
            self.isCritical=True
            self.get_logger().warn(f"High motor temperature: {self.motor_temperature}°C")

    def publish_status_message(self):
        msg = String()
        msg.data = self.generate_status_message()
        self.publisherStatus.publish(msg)

    def publish_warning(self):
        msg2 = String()
        msg2.data = f"{yellow}{self.generate_warning_message()}{reset}"
        self.publisherWarning.publish(msg2)
    
    def publish_critical(self):
        msg = Bool()
        msg.data = self.isCritical
        self.publisherStop.publish(msg)

        
    def check_lidar_status(self, msg):
        self.last_msg_time = msg.header.stamp

        if self.lidar_previous is None:
            #self.get_logger().info("Initializing lidar_previous timestamp")
            self.lidar_previous = self.last_msg_time
            return

        # Calculate elapsed time in seconds
        elapsed = (Time.from_msg(self.last_msg_time) - Time.from_msg(self.lidar_previous)).nanoseconds * 1e-9

        #self.get_logger().info(f"Elapsed time since last LiDAR message: {elapsed:.3f} seconds")

        self.lidar_previous = self.last_msg_time

        # Check if elapsed time exceeds the timeout
        #if elapsed > self.lidar_timeout:
            #self.get_logger().warn(f"No LiDAR message received for {elapsed:.2f} seconds!")
            
        if elapsed > self.lidar_timeout:
            self.lidar_status = False
        else:
            self.lidar_status = True
            
        if elapsed > 1.0:
            self.isCritical = True
        
        
    def generate_status_message(self): # VElocity and angular 
        # Create a status string with all health parameters
        status_msg = "System Status:\n"
        
        # Check battery status
        if self.battery_voltage == 0.0:
            status_msg += "Battery: No data\n"
        else:
            status_msg += f"Battery Voltage: {self.battery_voltage}V\n"
        
        status_msg += f"Temperature: {self.motor_temperature}\n"
        
        # Motor
        if not self.motor_status:
            status_msg += "Motor: Idel\n"
        else:
            status_msg += "Motor: Operational\n"

        status_msg += f"Velocity: {self.motor_velocity}\n"
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