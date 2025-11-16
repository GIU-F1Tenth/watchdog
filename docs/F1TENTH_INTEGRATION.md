# F1TENTH System Integration Guide

This guide provides detailed instructions for integrating the Watchdog sanity checking system with F1TENTH autonomous racing vehicles.

## Overview

The F1TENTH Watchdog system provides comprehensive health monitoring and safety validation for autonomous racing vehicles. This integration guide covers:

- Standard F1TENTH topic mappings
- FSM integration patterns
- Racing-specific configurations
- Safety protocols
- Performance optimization

## Prerequisites

### Required Packages
- ROS2 Humble or later
- Standard F1TENTH packages:
  - `vesc_msgs` (VESC interface)
  - `ackermann_msgs` (vehicle control)
  - `sensor_msgs` (sensor interfaces)
  - `nav_msgs` (navigation)
  - `robot_localization` (EKF)
  - `realsense2_camera` (Intel RealSense)
  - `urg_node` (Hokuyo LiDAR)

### Hardware Compatibility
- F1TENTH vehicle platform
- VESC motor controller
- LiDAR sensor (Hokuyo UST-10LX recommended)
- Camera (Intel RealSense recommended)
- IMU sensor
- 3S LiPo battery

## Quick Start

### 1. Basic Watchdog Launch
```bash
# Launch watchdog with F1TENTH configuration
ros2 launch watchdog f1tenth_watchdog.launch.py

# With custom configuration
ros2 launch watchdog f1tenth_watchdog.launch.py config_file:=/path/to/custom_config.yaml
```

### 2. Complete System Launch
```bash
# Launch complete F1TENTH system with watchdog
ros2 launch watchdog f1tenth_system.launch.py

# With simulation
ros2 launch watchdog f1tenth_system.launch.py use_sim_time:=true
```

### 3. FSM Integration
```bash
# Launch FSM integration node
ros2 run watchdog fsm_integration_node

# Monitor FSM health communication
ros2 topic echo /fsm/watchdog_health
ros2 topic echo /racing/system_status
```

## Topic Mappings

### Standard F1TENTH Topics
The watchdog system maps to standard F1TENTH topic names:

| Watchdog Subscription | F1TENTH Topic | Message Type | Purpose |
|----------------------|---------------|--------------|---------|
| `/sensors/core` | `/sensors/core` | `vesc_msgs/VescStateStamped` | VESC telemetry |
| `/scan` | `/scan` | `sensor_msgs/LaserScan` | LiDAR data |
| `/camera/image_raw` | `/camera/camera/color/image_raw` | `sensor_msgs/Image` | Camera feed |
| `/odom` | `/odom` | `nav_msgs/Odometry` | Vehicle odometry |
| `/imu` | `/imu` | `sensor_msgs/Imu` | IMU data |

### Watchdog Output Topics
| Topic | Message Type | Purpose |
|-------|--------------|---------|
| `/watchdog/sanity_warnings` | `watchdog_msgs/SanityWarning` | Real-time validation warnings |
| `/watchdog/sensor_health` | `watchdog_msgs/SensorHealth` | Individual sensor health |
| `/watchdog/sanity_summary` | `watchdog_msgs/SanitySummary` | Overall system health |
| `/emergency_stop` | `std_msgs/Bool` | Emergency stop signal |

### FSM Integration Topics
| Topic | Message Type | Purpose |
|-------|--------------|---------|
| `/fsm/state` | `std_msgs/String` | Current FSM state |
| `/fsm/watchdog_health` | `std_msgs/String` | Health level for FSM |
| `/racing/system_status` | `std_msgs/String` | Racing safety status |
| `/cmd_vel_emergency` | `geometry_msgs/Twist` | Emergency control override |

## Configuration

### F1TENTH-Specific Parameters
The `config/f1tenth_params.yaml` provides optimized settings:

```yaml
# Vehicle specifications
vehicle_type: "f1tenth"
max_speed: 8.0  # m/s
wheelbase: 0.33  # m

# Racing-optimized thresholds
critical_voltage: 12.2  # V - Conservative for racing
temp_warning_start: 75.0  # C - Lower for racing
sanity_check_frequency: 10.0  # Hz - High frequency

# LiDAR settings for indoor racing
lidar_range_max: 10.0  # m
lidar_noise_threshold: 0.08

# Cross-sensor validation
temporal_sync_threshold: 0.08  # s - Tight sync
lidar_camera_correlation_threshold: 0.75
```

### Racing Mode Configuration
```yaml
# Enable racing-specific features
racing_mode: true
track_type: "indoor"  # indoor, outdoor, mixed
safety_margin: 1.2
emergency_stop_enabled: true
auto_recovery_enabled: false  # Disable for safety
```

## FSM Integration

### Health-Based State Transitions

The watchdog system provides health information that FSMs can use for decision-making:

```python
def health_callback(self, msg):
    """React to watchdog health updates."""
    if not msg.overall_system_healthy:
        self.transition_to_emergency_stop()
    elif msg.overall_health_score < 0.7:
        self.transition_to_caution_mode()
    elif msg.overall_health_score > 0.9:
        self.enable_full_performance()
```

### Emergency Stop Integration

```python
def emergency_stop_callback(self, msg):
    """Handle emergency stop from watchdog."""
    if msg.data:
        self.immediate_stop()
        self.transition_to_emergency_state()
```

### Racing Parameter Adjustment

```python
def adjust_racing_parameters(self, health_score):
    """Adjust racing based on health."""
    if health_score > 0.9:
        self.max_speed = 8.0  # Full performance
    elif health_score > 0.7:
        self.max_speed = 6.5  # Reduced performance
    else:
        self.max_speed = 4.0  # Conservative mode
```

## Safety Protocols

### Emergency Stop Behavior
1. **Immediate**: Vehicle stops within 100ms of critical alert
2. **Graceful**: Controlled deceleration for non-critical issues
3. **Recovery**: Manual intervention required for restart

### Health Monitoring Levels
- **Excellent (>0.9)**: Full racing performance
- **Good (0.7-0.9)**: Normal operation with monitoring
- **Degraded (0.5-0.7)**: Reduced performance mode
- **Poor (0.3-0.5)**: Caution mode, consider stopping
- **Critical (<0.3)**: Emergency stop recommended

### Racing-Specific Validations
- Battery voltage monitoring for race duration
- Motor temperature tracking during high-speed operation
- Sensor synchronization for accurate perception
- Cross-sensor correlation for redundancy

## Performance Optimization

### Computational Resources
- **CPU Usage**: <5% on F1TENTH hardware
- **Memory**: <50MB RAM
- **Network**: Minimal bandwidth impact
- **Latency**: <10ms for critical alerts

### Configuration Tuning
```bash
# High-performance racing
ros2 param set /watchdog sanity_check_frequency 15.0
ros2 param set /watchdog temporal_sync_threshold 0.05

# Endurance racing
ros2 param set /watchdog sanity_check_frequency 5.0
ros2 param set /watchdog battery_consistency_window 20

# Indoor racing
ros2 param set /watchdog lidar_range_max 8.0
ros2 param set /watchdog camera_brightness_threshold 30
```

## Troubleshooting

### Common Issues

#### High False Positive Rate
```bash
# Check sensor publication rates
ros2 topic hz /scan
ros2 topic hz /odom

# Adjust thresholds
ros2 param set /watchdog temporal_sync_threshold 0.15
ros2 param set /watchdog lidar_noise_threshold 0.12
```

#### Emergency Stops During Racing
```bash
# Check critical issues
ros2 topic echo /watchdog/sanity_warnings --field severity

# Review configuration
ros2 param get /watchdog critical_voltage
ros2 param get /watchdog temp_critical
```

#### Poor Cross-Sensor Correlation
```bash
# Check sensor calibration
ros2 topic echo /watchdog/sanity_warnings --field anomaly_type

# Adjust correlation threshold
ros2 param set /watchdog lidar_camera_correlation_threshold 0.6
```

### Debug Mode
```bash
# Enable debug logging
ros2 param set /watchdog debug_mode true
ros2 run watchdog watchdog_node --ros-args --log-level DEBUG
```

## Example Integration

### Complete FSM Example
See `examples/f1tenth_fsm_example.py` for a complete FSM implementation with watchdog integration.

### Key Features:
- Health-based state transitions
- Emergency stop handling
- Performance adjustment based on health score
- Racing safety protocols

### Launch Example:
```bash
# Terminal 1: Launch watchdog system
ros2 launch watchdog f1tenth_system.launch.py

# Terminal 2: Launch example FSM
ros2 run watchdog f1tenth_fsm_example.py

# Terminal 3: Monitor health
ros2 topic echo /fsm/watchdog_health
```

## Testing and Validation

### System Health Test
```bash
# Test basic health reporting
ros2 topic echo /watchdog/sanity_summary --once

# Test emergency stop
ros2 topic pub /emergency_stop std_msgs/Bool "data: true" --once
```

### Integration Test
```bash
# Launch complete system
ros2 launch watchdog f1tenth_system.launch.py

# Verify topic connections
ros2 node info /watchdog
ros2 node info /watchdog_fsm_integration
```

### Performance Test
```bash
# Monitor update rates
ros2 topic hz /watchdog/sanity_summary
ros2 topic hz /fsm/watchdog_health

# Check computational load
top -p $(pgrep -f watchdog)
```

## Best Practices

### Racing Safety
1. Always test in safe environment before racing
2. Have manual override readily available
3. Set conservative thresholds for competition
4. Monitor health trends during practice
5. Validate emergency stop functionality

### System Integration
1. Use reliable QoS for critical topics
2. Implement proper error handling
3. Test all failure modes
4. Document configuration changes
5. Regular calibration verification

### Performance
1. Tune parameters for specific tracks
2. Monitor computational resources
3. Optimize sensor publication rates
4. Balance responsiveness vs. stability
5. Regular performance profiling

## Support

### Resources
- Configuration examples: `config/examples/`
- Integration code: `examples/`
- API documentation: `docs/API.md`
- Troubleshooting: Main README.md

### Community
- F1TENTH forums
- GitHub issues
- ROS2 community discussions

---

For additional support or questions about F1TENTH integration, please refer to the main documentation or contact the maintainers.