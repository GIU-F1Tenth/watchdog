#!/usr/bin/env python3

"""
F1TENTH FSM Integration Helper

This module provides integration utilities for connecting the watchdog system
with F1TENTH Finite State Machine architectures.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist

try:
    from watchdog_msgs.msg import SanityWarning, SensorHealth, SanitySummary
    WATCHDOG_MSGS_AVAILABLE = True
except ImportError:
    WATCHDOG_MSGS_AVAILABLE = False
    print("Warning: watchdog_msgs not available. Using mock message types.")

from typing import Dict, List, Callable, Optional
from dataclasses import dataclass
from enum import Enum


class FSMState(Enum):
    """Common F1TENTH FSM states."""
    IDLE = "idle"
    READY = "ready"
    DRIVING = "driving"
    EMERGENCY_STOP = "emergency_stop"
    MANUAL_CONTROL = "manual_control"
    FAILED = "failed"
    RECOVERY = "recovery"


class HealthLevel(Enum):
    """System health levels for FSM decision making."""
    EXCELLENT = "excellent"  # > 0.9
    GOOD = "good"           # 0.7 - 0.9
    DEGRADED = "degraded"   # 0.5 - 0.7
    POOR = "poor"           # 0.3 - 0.5
    CRITICAL = "critical"   # < 0.3


@dataclass
class FSMHealthStatus:
    """Health status information for FSM integration."""
    overall_healthy: bool
    health_score: float
    health_level: HealthLevel
    critical_issues: List[str]
    warnings: List[str]
    affected_sensors: List[str]
    recommended_action: str
    can_continue_racing: bool


class WatchdogFSMIntegration(Node):
    """
    Integration helper for connecting watchdog with F1TENTH FSM.
    
    This class provides:
    - Health status translation for FSM consumption
    - Emergency stop handling
    - State-based configuration adjustment
    - Racing-specific validation logic
    """
    
    def __init__(self, node_name: str = 'watchdog_fsm_integration'):
        """Initialize the FSM integration node."""
        super().__init__(node_name)
        
        # QoS profiles for reliable communication
        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # Current FSM state and health status
        self.current_fsm_state = FSMState.IDLE
        self.current_health_status: Optional[FSMHealthStatus] = None
        self.emergency_stop_active = False
        
        # Callback registrations
        self.state_change_callbacks: List[Callable] = []
        self.health_change_callbacks: List[Callable] = []
        self.emergency_callbacks: List[Callable] = []
        
        # Initialize publishers
        self._init_publishers()
        
        # Initialize subscribers
        self._init_subscribers()
        
        # Configuration
        self.health_thresholds = {
            HealthLevel.EXCELLENT: 0.9,
            HealthLevel.GOOD: 0.7,
            HealthLevel.DEGRADED: 0.5,
            HealthLevel.POOR: 0.3,
            HealthLevel.CRITICAL: 0.0
        }
        
        self.get_logger().info("Watchdog FSM Integration initialized")
    
    def _init_publishers(self):
        """Initialize ROS2 publishers."""
        # FSM command topics
        self.emergency_stop_pub = self.create_publisher(
            Bool, '/emergency_stop', self.reliable_qos)
        
        self.fsm_health_pub = self.create_publisher(
            String, '/fsm/watchdog_health', self.reliable_qos)
        
        self.racing_status_pub = self.create_publisher(
            String, '/racing/system_status', 10)
        
        # Control override for emergency situations
        self.cmd_vel_override_pub = self.create_publisher(
            Twist, '/cmd_vel_emergency', self.reliable_qos)
    
    def _init_subscribers(self):
        """Initialize ROS2 subscribers."""
        # Watchdog sanity checking topics
        if WATCHDOG_MSGS_AVAILABLE:
            self.sanity_summary_sub = self.create_subscription(
                SanitySummary, '/watchdog/sanity_summary',
                self._sanity_summary_callback, 10)
            
            self.sanity_warning_sub = self.create_subscription(
                SanityWarning, '/watchdog/sanity_warnings',
                self._sanity_warning_callback, 10)
            
            self.sensor_health_sub = self.create_subscription(
                SensorHealth, '/watchdog/sensor_health',
                self._sensor_health_callback, 10)
        
        # FSM state topic (assuming standard F1TENTH FSM)
        self.fsm_state_sub = self.create_subscription(
            String, '/fsm/state', self._fsm_state_callback, 10)
        
        # Emergency stop from other sources
        self.emergency_sub = self.create_subscription(
            Bool, '/tmp/watchdog/critical', self._emergency_callback, self.reliable_qos)
    
    def _sanity_summary_callback(self, msg):
        """Process sanity summary messages."""
        if not WATCHDOG_MSGS_AVAILABLE:
            return
            
        # Convert to FSM health status
        health_status = self._convert_to_fsm_health(msg)
        
        # Update current status
        old_status = self.current_health_status
        self.current_health_status = health_status
        
        # Publish FSM-friendly health information
        self._publish_fsm_health(health_status)
        
        # Check for state changes that require FSM action
        self._check_fsm_action_required(health_status, old_status)
        
        # Notify registered callbacks
        for callback in self.health_change_callbacks:
            try:
                callback(health_status, old_status)
            except Exception as e:
                self.get_logger().warn(f"Health callback failed: {e}")
    
    def _sanity_warning_callback(self, msg):
        """Process sanity warning messages."""
        if not WATCHDOG_MSGS_AVAILABLE:
            return
            
        # Check for critical warnings that require immediate action
        if msg.severity >= 3:  # CRITICAL
            self._handle_critical_warning(msg)
        elif msg.severity >= 2:  # ERROR
            self._handle_error_warning(msg)
    
    def _sensor_health_callback(self, msg):
        """Process individual sensor health messages."""
        if not WATCHDOG_MSGS_AVAILABLE:
            return
            
        # Log sensor-specific health changes
        if not msg.is_healthy:
            self.get_logger().warn(
                f"Sensor {msg.sensor_name} unhealthy: {msg.status_description}")
    
    def _fsm_state_callback(self, msg):
        """Process FSM state changes."""
        try:
            new_state = FSMState(msg.data.lower())
            old_state = self.current_fsm_state
            self.current_fsm_state = new_state
            
            # Adjust watchdog behavior based on FSM state
            self._adjust_for_fsm_state(new_state, old_state)
            
            # Notify callbacks
            for callback in self.state_change_callbacks:
                try:
                    callback(new_state, old_state)
                except Exception as e:
                    self.get_logger().warn(f"State callback failed: {e}")
                    
        except ValueError:
            self.get_logger().warn(f"Unknown FSM state: {msg.data}")
    
    def _emergency_callback(self, msg):
        """Handle emergency stop signals."""
        if msg.data and not self.emergency_stop_active:
            self._trigger_emergency_stop("Watchdog critical alert")
        elif not msg.data and self.emergency_stop_active:
            self._clear_emergency_stop()
    
    def _convert_to_fsm_health(self, sanity_summary) -> FSMHealthStatus:
        """Convert watchdog sanity summary to FSM health status."""
        health_level = self._get_health_level(sanity_summary.overall_health_score)
        
        # Determine if racing can continue
        can_continue_racing = (
            sanity_summary.overall_system_healthy and 
            health_level.value not in ['critical', 'poor'] and
            len(sanity_summary.critical_issues) == 0
        )
        
        # Determine recommended action
        recommended_action = self._get_recommended_action(
            health_level, sanity_summary.critical_issues, sanity_summary.warnings)
        
        # Extract affected sensors
        affected_sensors = [
            sensor.sensor_name for sensor in sanity_summary.sensor_healths
            if not sensor.is_healthy
        ]
        
        return FSMHealthStatus(
            overall_healthy=sanity_summary.overall_system_healthy,
            health_score=sanity_summary.overall_health_score,
            health_level=health_level,
            critical_issues=list(sanity_summary.critical_issues),
            warnings=list(sanity_summary.warnings),
            affected_sensors=affected_sensors,
            recommended_action=recommended_action,
            can_continue_racing=can_continue_racing
        )
    
    def _get_health_level(self, health_score: float) -> HealthLevel:
        """Convert numeric health score to health level enum."""
        if health_score >= self.health_thresholds[HealthLevel.EXCELLENT]:
            return HealthLevel.EXCELLENT
        elif health_score >= self.health_thresholds[HealthLevel.GOOD]:
            return HealthLevel.GOOD
        elif health_score >= self.health_thresholds[HealthLevel.DEGRADED]:
            return HealthLevel.DEGRADED
        elif health_score >= self.health_thresholds[HealthLevel.POOR]:
            return HealthLevel.POOR
        else:
            return HealthLevel.CRITICAL
    
    def _get_recommended_action(self, health_level: HealthLevel, 
                               critical_issues: List[str], 
                               warnings: List[str]) -> str:
        """Determine recommended action based on health status."""
        if critical_issues:
            return "emergency_stop"
        elif health_level == HealthLevel.CRITICAL:
            return "emergency_stop"
        elif health_level == HealthLevel.POOR:
            return "transition_to_safe_mode"
        elif health_level == HealthLevel.DEGRADED:
            return "reduce_speed"
        elif warnings:
            return "monitor_closely"
        else:
            return "continue_normal"
    
    def _publish_fsm_health(self, health_status: FSMHealthStatus):
        """Publish health status in FSM-friendly format."""
        # Publish simple health level for FSM consumption
        health_msg = String()
        health_msg.data = health_status.health_level.value
        self.fsm_health_pub.publish(health_msg)
        
        # Publish detailed racing status
        racing_status = String()
        if health_status.can_continue_racing:
            racing_status.data = f"SAFE_TO_RACE:{health_status.health_score:.2f}"
        else:
            racing_status.data = f"UNSAFE_TO_RACE:{':'.join(health_status.critical_issues)}"
        self.racing_status_pub.publish(racing_status)
    
    def _check_fsm_action_required(self, current_status: FSMHealthStatus, 
                                  previous_status: Optional[FSMHealthStatus]):
        """Check if FSM action is required based on health changes."""
        if not current_status.can_continue_racing:
            if self.current_fsm_state == FSMState.DRIVING:
                self.get_logger().warn("Health degraded during driving - recommending FSM action")
        
        # Check for health level changes
        if previous_status and current_status.health_level != previous_status.health_level:
            self.get_logger().info(
                f"Health level changed: {previous_status.health_level.value} -> "
                f"{current_status.health_level.value}")
    
    def _handle_critical_warning(self, warning_msg):
        """Handle critical warnings that require immediate action."""
        self.get_logger().error(
            f"CRITICAL: {warning_msg.sensor_name} - {warning_msg.description}")
        
        if warning_msg.suggested_action == "emergency_stop":
            self._trigger_emergency_stop(f"Critical warning: {warning_msg.description}")
    
    def _handle_error_warning(self, warning_msg):
        """Handle error-level warnings."""
        self.get_logger().warn(
            f"ERROR: {warning_msg.sensor_name} - {warning_msg.description}")
    
    def _adjust_for_fsm_state(self, new_state: FSMState, old_state: FSMState):
        """Adjust watchdog behavior based on FSM state."""
        if new_state == FSMState.DRIVING and old_state != FSMState.DRIVING:
            self.get_logger().info("Entering driving state - enabling racing mode validation")
            # Could adjust parameters here via service calls
        
        elif new_state == FSMState.EMERGENCY_STOP:
            self.get_logger().warn("FSM entered emergency stop state")
            self.emergency_stop_active = True
    
    def _trigger_emergency_stop(self, reason: str):
        """Trigger emergency stop and notify FSM."""
        if not self.emergency_stop_active:
            self.emergency_stop_active = True
            self.get_logger().error(f"EMERGENCY STOP TRIGGERED: {reason}")
            
            # Publish emergency stop signal
            emergency_msg = Bool()
            emergency_msg.data = True
            self.emergency_stop_pub.publish(emergency_msg)
            
            # Send zero velocity command
            stop_cmd = Twist()
            self.cmd_vel_override_pub.publish(stop_cmd)
            
            # Notify callbacks
            for callback in self.emergency_callbacks:
                try:
                    callback(reason)
                except Exception as e:
                    self.get_logger().warn(f"Emergency callback failed: {e}")
    
    def _clear_emergency_stop(self):
        """Clear emergency stop condition."""
        if self.emergency_stop_active:
            self.emergency_stop_active = False
            self.get_logger().info("Emergency stop cleared")
            
            emergency_msg = Bool()
            emergency_msg.data = False
            self.emergency_stop_pub.publish(emergency_msg)
    
    # Public API for FSM integration
    
    def register_state_change_callback(self, callback: Callable[[FSMState, FSMState], None]):
        """Register callback for FSM state changes."""
        self.state_change_callbacks.append(callback)
    
    def register_health_change_callback(self, callback: Callable[[FSMHealthStatus, Optional[FSMHealthStatus]], None]):
        """Register callback for health status changes."""
        self.health_change_callbacks.append(callback)
    
    def register_emergency_callback(self, callback: Callable[[str], None]):
        """Register callback for emergency stop events."""
        self.emergency_callbacks.append(callback)
    
    def get_current_health_status(self) -> Optional[FSMHealthStatus]:
        """Get current system health status."""
        return self.current_health_status
    
    def get_current_fsm_state(self) -> FSMState:
        """Get current FSM state."""
        return self.current_fsm_state
    
    def is_safe_to_race(self) -> bool:
        """Check if system is safe for racing."""
        if not self.current_health_status:
            return False
        return self.current_health_status.can_continue_racing and not self.emergency_stop_active
    
    def force_emergency_stop(self, reason: str):
        """Manually trigger emergency stop."""
        self._trigger_emergency_stop(f"Manual trigger: {reason}")


def main(args=None):
    """Main function for standalone execution."""
    rclpy.init(args=args)
    
    fsm_integration = WatchdogFSMIntegration()
    
    # Example callback registrations
    def on_state_change(new_state, old_state):
        print(f"FSM State changed: {old_state.value} -> {new_state.value}")
    
    def on_health_change(new_health, old_health):
        print(f"Health changed: score={new_health.health_score:.2f}, "
              f"level={new_health.health_level.value}")
    
    def on_emergency(reason):
        print(f"EMERGENCY STOP: {reason}")
    
    fsm_integration.register_state_change_callback(on_state_change)
    fsm_integration.register_health_change_callback(on_health_change)
    fsm_integration.register_emergency_callback(on_emergency)
    
    try:
        rclpy.spin(fsm_integration)
    except KeyboardInterrupt:
        pass
    finally:
        fsm_integration.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()