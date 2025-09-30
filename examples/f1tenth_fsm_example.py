#!/usr/bin/env python3

"""
Example F1TENTH FSM with Watchdog Integration

This example demonstrates how to integrate watchdog health monitoring
into an F1TENTH Finite State Machine for autonomous racing.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist

# Try to import ackermann_msgs, use a mock if not available
try:
    from ackermann_msgs.msg import AckermannDriveStamped
    ACKERMANN_MSGS_AVAILABLE = True
except ImportError:
    ACKERMANN_MSGS_AVAILABLE = False
    print("Warning: ackermann_msgs not available. Using mock message type.")
    
    # Create a mock AckermannDriveStamped for testing
    class MockAckermannDrive:
        def __init__(self):
            self.speed = 0.0
            self.steering_angle = 0.0
    
    class MockAckermannDriveStamped:
        def __init__(self):
            self.header = None
            self.drive = MockAckermannDrive()
    
    AckermannDriveStamped = MockAckermannDriveStamped

from enum import Enum
from typing import Optional
import time


class RacingState(Enum):
    """F1TENTH Racing FSM States."""
    STARTUP = "startup"
    READY = "ready"
    RACING = "racing"
    CAUTION = "caution"
    EMERGENCY_STOP = "emergency_stop"
    RECOVERY = "recovery"
    SHUTDOWN = "shutdown"


class F1TenthRacingFSM(Node):
    """
    Example F1TENTH Racing FSM with Watchdog Integration.
    
    This FSM demonstrates how to use watchdog health information
    to make racing decisions and maintain safety.
    """
    
    def __init__(self):
        super().__init__('f1tenth_racing_fsm')
        
        # FSM state management
        self.current_state = RacingState.STARTUP
        self.previous_state = RacingState.STARTUP
        self.state_entry_time = time.time()
        
        # Watchdog health tracking
        self.system_healthy = True
        self.health_score = 1.0
        self.watchdog_health_level = "excellent"
        self.emergency_stop_active = False
        self.critical_issues = []
        
        # Racing parameters
        self.max_speed = 6.0  # m/s
        self.current_max_speed = 6.0
        self.safety_margin = 1.0
        
        # QoS for reliable communication
        reliable_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        
        # Publishers
        self.fsm_state_pub = self.create_publisher(String, '/fsm/state', reliable_qos)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.emergency_brake_pub = self.create_publisher(Bool, '/emergency_brake', reliable_qos)
        
        # Subscribers
        self.watchdog_health_sub = self.create_subscription(
            String, '/fsm/watchdog_health', self._watchdog_health_callback, 10)
        
        self.racing_status_sub = self.create_subscription(
            String, '/racing/system_status', self._racing_status_callback, 10)
        
        self.emergency_stop_sub = self.create_subscription(
            Bool, '/emergency_stop', self._emergency_stop_callback, reliable_qos)
        
        # Manual control override
        self.manual_control_sub = self.create_subscription(
            Bool, '/manual_control', self._manual_control_callback, 10)
        
        # Example racing command input
        self.racing_cmd_sub = self.create_subscription(
            AckermannDriveStamped, '/racing/drive_cmd', self._racing_cmd_callback, 10)
        
        # State machine timer
        self.fsm_timer = self.create_timer(0.1, self._fsm_update)  # 10Hz
        
        # Initialize in startup state
        self._enter_state(RacingState.STARTUP)
        
        self.get_logger().info("F1TENTH Racing FSM with Watchdog Integration initialized")
    
    def _fsm_update(self):
        """Main FSM update loop."""
        current_time = time.time()
        time_in_state = current_time - self.state_entry_time
        
        # State machine logic
        if self.current_state == RacingState.STARTUP:
            self._handle_startup_state(time_in_state)
        
        elif self.current_state == RacingState.READY:
            self._handle_ready_state(time_in_state)
        
        elif self.current_state == RacingState.RACING:
            self._handle_racing_state(time_in_state)
        
        elif self.current_state == RacingState.CAUTION:
            self._handle_caution_state(time_in_state)
        
        elif self.current_state == RacingState.EMERGENCY_STOP:
            self._handle_emergency_stop_state(time_in_state)
        
        elif self.current_state == RacingState.RECOVERY:
            self._handle_recovery_state(time_in_state)
        
        elif self.current_state == RacingState.SHUTDOWN:
            self._handle_shutdown_state(time_in_state)
        
        # Publish current state
        self._publish_state()
    
    def _handle_startup_state(self, time_in_state: float):
        """Handle startup state logic."""
        # Wait for watchdog to initialize and report health
        if time_in_state > 3.0:  # 3 second startup delay
            if self.system_healthy and self.health_score > 0.8:
                self._transition_to(RacingState.READY)
            elif time_in_state > 10.0:  # Timeout
                self.get_logger().error("Startup timeout - system not healthy")
                self._transition_to(RacingState.SHUTDOWN)
    
    def _handle_ready_state(self, time_in_state: float):
        """Handle ready state logic."""
        # Wait for racing command or manual intervention
        # In real implementation, this would wait for race start signal
        if self.system_healthy and self.health_score > 0.7:
            # Auto-transition to racing for demo (remove in real implementation)
            if time_in_state > 2.0:
                self._transition_to(RacingState.RACING)
        else:
            self._transition_to(RacingState.CAUTION)
    
    def _handle_racing_state(self, time_in_state: float):
        """Handle racing state logic."""
        # Check health continuously during racing
        if not self.system_healthy or self.emergency_stop_active:
            self._transition_to(RacingState.EMERGENCY_STOP)
        elif self.health_score < 0.5:
            self._transition_to(RacingState.EMERGENCY_STOP)
        elif self.health_score < 0.7:
            self._transition_to(RacingState.CAUTION)
        
        # Adjust racing parameters based on health
        self._adjust_racing_parameters()
    
    def _handle_caution_state(self, time_in_state: float):
        """Handle caution state logic."""
        # Reduce speed and monitor health recovery
        self.current_max_speed = self.max_speed * 0.6
        
        # Check for health recovery
        if self.system_healthy and self.health_score > 0.8:
            if time_in_state > 5.0:  # Wait 5 seconds before returning to racing
                self._transition_to(RacingState.RACING)
        elif not self.system_healthy or self.health_score < 0.4:
            self._transition_to(RacingState.EMERGENCY_STOP)
    
    def _handle_emergency_stop_state(self, time_in_state: float):
        """Handle emergency stop state logic."""
        # Publish emergency brake command
        brake_msg = Bool()
        brake_msg.data = True
        self.emergency_brake_pub.publish(brake_msg)
        
        # Publish zero drive command
        self._publish_stop_command()
        
        # Check for recovery conditions
        if not self.emergency_stop_active and self.system_healthy and self.health_score > 0.6:
            if time_in_state > 5.0:  # Minimum 5 seconds in emergency stop
                self._transition_to(RacingState.RECOVERY)
    
    def _handle_recovery_state(self, time_in_state: float):
        """Handle recovery state logic."""
        # Gradual return to normal operation
        if self.system_healthy and self.health_score > 0.8:
            if time_in_state > 3.0:  # 3 second recovery period
                self._transition_to(RacingState.READY)
        elif not self.system_healthy or self.health_score < 0.5:
            self._transition_to(RacingState.EMERGENCY_STOP)
    
    def _handle_shutdown_state(self, time_in_state: float):
        """Handle shutdown state logic."""
        # Publish stop commands and prepare for shutdown
        self._publish_stop_command()
        
        if time_in_state > 2.0:
            self.get_logger().info("FSM shutdown complete")
    
    def _adjust_racing_parameters(self):
        """Adjust racing parameters based on health score."""
        if self.health_score >= 0.9:
            self.current_max_speed = self.max_speed
            self.safety_margin = 1.0
        elif self.health_score >= 0.8:
            self.current_max_speed = self.max_speed * 0.9
            self.safety_margin = 1.1
        elif self.health_score >= 0.7:
            self.current_max_speed = self.max_speed * 0.8
            self.safety_margin = 1.2
        else:
            self.current_max_speed = self.max_speed * 0.6
            self.safety_margin = 1.5
    
    def _transition_to(self, new_state: RacingState):
        """Transition to a new state."""
        if new_state != self.current_state:
            self.get_logger().info(f"FSM Transition: {self.current_state.value} -> {new_state.value}")
            self._exit_state(self.current_state)
            self.previous_state = self.current_state
            self.current_state = new_state
            self._enter_state(new_state)
    
    def _enter_state(self, state: RacingState):
        """Actions to perform when entering a state."""
        self.state_entry_time = time.time()
        
        if state == RacingState.STARTUP:
            self.get_logger().info("FSM: Starting up, waiting for watchdog health...")
        elif state == RacingState.READY:
            self.get_logger().info("FSM: Ready for racing")
        elif state == RacingState.RACING:
            self.get_logger().info("FSM: Racing mode active")
        elif state == RacingState.CAUTION:
            self.get_logger().warn("FSM: Caution mode - reduced performance")
        elif state == RacingState.EMERGENCY_STOP:
            self.get_logger().error("FSM: EMERGENCY STOP ACTIVE")
        elif state == RacingState.RECOVERY:
            self.get_logger().info("FSM: Recovery mode - returning to normal")
        elif state == RacingState.SHUTDOWN:
            self.get_logger().info("FSM: Shutting down")
    
    def _exit_state(self, state: RacingState):
        """Actions to perform when exiting a state."""
        if state == RacingState.EMERGENCY_STOP:
            # Clear emergency brake
            brake_msg = Bool()
            brake_msg.data = False
            self.emergency_brake_pub.publish(brake_msg)
    
    def _publish_state(self):
        """Publish current FSM state."""
        state_msg = String()
        state_msg.data = self.current_state.value
        self.fsm_state_pub.publish(state_msg)
    
    def _publish_stop_command(self):
        """Publish stop drive command."""
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.speed = 0.0
        drive_msg.drive.steering_angle = 0.0
        self.drive_pub.publish(drive_msg)
    
    # Callback functions for watchdog integration
    
    def _watchdog_health_callback(self, msg):
        """Process watchdog health level updates."""
        self.watchdog_health_level = msg.data
        
        # Update system health based on watchdog report
        if msg.data in ['critical', 'poor']:
            self.system_healthy = False
        elif msg.data in ['degraded']:
            self.system_healthy = True  # Healthy but degraded
        else:  # excellent, good
            self.system_healthy = True
        
        self.get_logger().debug(f"Watchdog health level: {msg.data}")
    
    def _racing_status_callback(self, msg):
        """Process racing status updates from watchdog."""
        if msg.data.startswith('SAFE_TO_RACE:'):
            # Extract health score from message
            try:
                score_str = msg.data.split(':')[1]
                self.health_score = float(score_str)
                self.critical_issues = []
            except (IndexError, ValueError):
                pass
        
        elif msg.data.startswith('UNSAFE_TO_RACE:'):
            self.health_score = 0.0
            # Extract critical issues
            try:
                issues_str = msg.data.split(':', 1)[1]
                self.critical_issues = issues_str.split(':') if issues_str else []
            except IndexError:
                self.critical_issues = ["Unknown critical issue"]
    
    def _emergency_stop_callback(self, msg):
        """Handle emergency stop signals."""
        if msg.data != self.emergency_stop_active:
            self.emergency_stop_active = msg.data
            
            if msg.data:
                self.get_logger().error("Emergency stop signal received!")
                if self.current_state != RacingState.EMERGENCY_STOP:
                    self._transition_to(RacingState.EMERGENCY_STOP)
            else:
                self.get_logger().info("Emergency stop signal cleared")
    
    def _manual_control_callback(self, msg):
        """Handle manual control override."""
        if msg.data:
            self.get_logger().info("Manual control activated")
            # In real implementation, transition to manual control state
    
    def _racing_cmd_callback(self, msg):
        """Process racing drive commands with safety limits."""
        if self.current_state == RacingState.RACING:
            # Apply safety limits based on health
            limited_msg = AckermannDriveStamped()
            limited_msg.header = msg.header
            
            # Limit speed based on current health
            max_allowed_speed = self.current_max_speed
            limited_msg.drive.speed = min(abs(msg.drive.speed), max_allowed_speed)
            if msg.drive.speed < 0:
                limited_msg.drive.speed *= -1
            
            # Apply steering limits if needed
            max_steering = 0.5  # rad
            limited_msg.drive.steering_angle = max(-max_steering, 
                                                  min(max_steering, msg.drive.steering_angle))
            
            # Publish limited command
            self.drive_pub.publish(limited_msg)
        
        elif self.current_state in [RacingState.CAUTION]:
            # Reduced performance mode
            limited_msg = AckermannDriveStamped()
            limited_msg.header = msg.header
            limited_msg.drive.speed = min(abs(msg.drive.speed), self.current_max_speed)
            if msg.drive.speed < 0:
                limited_msg.drive.speed *= -1
            limited_msg.drive.steering_angle = msg.drive.steering_angle * 0.8
            
            self.drive_pub.publish(limited_msg)
        
        else:
            # Not in racing state - stop
            self._publish_stop_command()


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    fsm_node = F1TenthRacingFSM()
    
    try:
        rclpy.spin(fsm_node)
    except KeyboardInterrupt:
        pass
    finally:
        fsm_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()