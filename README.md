# F1TENTH Watchdog

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)

A comprehensive system monitoring and safety watchdog for F1TENTH autonomous racing vehicles. This ROS2 package provides real-time monitoring of critical system components, including advanced sanity checking capabilities to ensure safe vehicle operation.

## Features

### Core Monitoring
-   **Battery Monitoring**: Continuous voltage monitoring with configurable thresholds
-   **Motor Temperature Monitoring**: Real-time temperature tracking with thermal protection
-   **LiDAR Health Monitoring**: Timeout detection and status reporting
-   **Camera Status Monitoring**: Live feed verification and timeout detection
-   **Motor Status Tracking**: Velocity and angular velocity monitoring
-   **Critical System Alerts**: Emergency stop signals for unsafe conditions

### Advanced Sanity Checking
-   **Sensor Data Validation**: Real-time validation of sensor readings for anomaly detection
-   **Cross-Sensor Correlation**: Multi-modal sensor consistency checking
-   **Temporal Synchronization**: Detection of sensor timing issues
-   **Environmental Consistency**: Validation of sensor data against expected environmental conditions
-   **Predictive Anomaly Detection**: Early warning system for potential sensor failures
-   **Comprehensive Health Reporting**: Detailed analysis with confidence scores and suggested actions

### Integration Support
-   **FSM Integration**: Direct integration with F1TENTH Finite State Machine
-   **Custom Message Types**: Structured warning and health reporting
-   **Configurable Thresholds**: Easy tuning for different racing scenarios
-   **Real-time Performance**: Low-latency monitoring suitable for high-speed racing

## System Requirements

-   ROS2 Humble or later
-   Python 3.8+
-   VESC interface (vesc_msgs)
-   Standard ROS2 sensor message packages
-   NumPy (for advanced data analysis)

## Installation

1. Clone this repository into your ROS2 workspace:

```bash
cd ~/ros2_ws/src
git clone <repository-url> watchdog
```

2. Install dependencies:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:

```bash
colcon build --packages-select watchdog
```

4. Source the workspace:

```bash
source install/setup.bash
```

## Usage

### Basic Launch

```bash
ros2 launch watchdog watchdog.launch.py
```

### Custom Configuration

```bash
ros2 launch watchdog watchdog.launch.py config_file:=/path/to/custom/config.yaml
```

### Direct Node Execution

```bash
ros2 run watchdog watchdog_node
```

### Sanity Checking Features

The watchdog now includes advanced sanity checking capabilities that validate sensor data in real-time:

```bash
# Enable all sanity checking features
ros2 param set /watchdog sanity_check_enabled true

# Enable specific validators
ros2 param set /watchdog battery_validation_enabled true
ros2 param set /watchdog lidar_validation_enabled true
ros2 param set /watchdog camera_validation_enabled true
ros2 param set /watchdog odometry_validation_enabled true
ros2 param set /watchdog cross_sensor_validation_enabled true
```

## Configuration

The watchdog node is highly configurable through parameters. The default configuration is located in `config/watchdog_params.yaml`.

### Core Monitoring Parameters

-   `critical_voltage`: Battery voltage threshold for critical alerts (default: 12.0V)
-   `temp_warning_start`: Temperature threshold for warnings (default: 80.0°C)
-   `lidar_timeout`: Maximum time between LiDAR messages (default: 0.5s)
-   `camera_timeout`: Maximum time between camera frames (default: 0.5s)

### Sanity Checking Parameters

#### Battery Validation
-   `battery_voltage_min`: Minimum acceptable voltage (default: 11.0V)
-   `battery_voltage_max`: Maximum acceptable voltage (default: 13.0V)
-   `battery_voltage_rate_limit`: Maximum voltage change rate (default: 1.0V/s)

#### LiDAR Validation
-   `lidar_range_min`: Minimum valid range reading (default: 0.1m)
-   `lidar_range_max`: Maximum valid range reading (default: 30.0m)
-   `lidar_noise_threshold`: Maximum acceptable noise level (default: 0.1)

#### Camera Validation
-   `camera_brightness_threshold`: Minimum brightness for valid image (default: 50)
-   `camera_blur_threshold`: Maximum blur metric (default: 100.0)
-   `camera_noise_threshold`: Maximum noise level (default: 30.0)

#### Cross-Sensor Validation
-   `lidar_camera_correlation_threshold`: Minimum correlation between LiDAR and camera depth (default: 0.8)
-   `temporal_sync_threshold`: Maximum acceptable time difference between sensors (default: 0.1s)
-   `position_consistency_threshold`: Maximum position discrepancy (default: 0.5m)

See `config/watchdog_params.yaml` for complete parameter documentation and tuning guidelines.

## ROS2 Topics and Messages

### Subscribed Topics

-   `/sensors/core` (vesc_msgs/VescStateStamped): VESC sensor data
-   `/camera/camera/color/image_raw` (sensor_msgs/Image): Camera feed
-   `/scan` (sensor_msgs/LaserScan): LiDAR data
-   `/odom` (nav_msgs/Odometry): Vehicle odometry

### Published Topics

#### Core Monitoring
-   `/tmp/watchdog/critical` (std_msgs/Bool): Critical system alert
-   `/tmp/watchdog/status` (std_msgs/String): System status information

#### Sanity Checking
-   `/watchdog/sanity_warnings` (watchdog_msgs/SanityWarning): Real-time validation warnings
-   `/watchdog/sensor_health` (watchdog_msgs/SensorHealth): Individual sensor health reports
-   `/watchdog/sanity_summary` (watchdog_msgs/SanitySummary): Comprehensive system health summary

### Custom Message Types

#### SanityWarning
```yaml
Header header
string sensor_name          # Name of the sensor (e.g., "battery", "lidar")
string anomaly_type         # Type of anomaly detected
string description          # Human-readable description
string suggested_action     # Recommended action to take
uint8 severity             # Severity level (INFO=0, WARNING=1, ERROR=2, CRITICAL=3)
float64 confidence         # Confidence score (0.0 to 1.0)
```

#### SensorHealth
```yaml
Header header
string sensor_name          # Name of the sensor
bool is_healthy            # Overall health status
float64 health_score       # Health score (0.0 to 1.0)
string[] active_warnings   # List of active warning types
float64 last_data_time     # Timestamp of last valid data
string status_description  # Detailed status information
```

#### SanitySummary
```yaml
Header header
bool overall_system_healthy    # Overall system health status
float64 overall_health_score   # Combined health score (0.0 to 1.0)
SensorHealth[] sensor_healths  # Individual sensor health reports
string[] critical_issues       # List of critical issues requiring immediate attention
string[] warnings             # List of warnings that should be monitored
string system_status          # Overall system status description
```

## FSM Integration

The watchdog system is designed for seamless integration with F1TENTH Finite State Machines:

### Emergency Stop Integration
```python
# Subscribe to critical alerts for immediate FSM state changes
self.critical_sub = self.create_subscription(
    Bool, '/tmp/watchdog/critical', self.emergency_stop_callback, 10)

def emergency_stop_callback(self, msg):
    if msg.data:
        self.transition_to_emergency_stop()
```

### Health-Based State Transitions
```python
# Monitor overall system health for state decisions
self.health_sub = self.create_subscription(
    SanitySummary, '/watchdog/sanity_summary', self.health_callback, 10)

def health_callback(self, msg):
    if not msg.overall_system_healthy:
        self.handle_degraded_performance(msg.critical_issues)
    elif msg.overall_health_score < 0.7:
        self.enable_conservative_mode()
```

## Testing

### Running Unit Tests

```bash
cd ~/ros2_ws
colcon test --packages-select watchdog
```

### Manual Testing

```bash
# Test core validators
python3 ~/ros2_ws/src/watchdog/test/test_core_components.py

# Test cross-sensor validation
python3 ~/ros2_ws/src/watchdog/test/test_cross_sensor.py

# Test complete system
ros2 run watchdog watchdog_node
```

### Integration Testing

```bash
# Launch with test configuration
ros2 launch watchdog watchdog.launch.py config_file:=$(ros2 pkg prefix watchdog)/share/watchdog/config/test_params.yaml

# Monitor sanity checking topics
ros2 topic echo /watchdog/sanity_warnings
ros2 topic echo /watchdog/sensor_health
ros2 topic echo /watchdog/sanity_summary
```

## Performance

### System Requirements
- **CPU Usage**: < 5% on modern F1TENTH hardware
- **Memory Usage**: < 50MB RAM
- **Update Rate**: 10Hz for core monitoring, 5Hz for sanity checking
- **Latency**: < 10ms for critical alerts

### Optimization Tips
- Adjust validation frequency based on racing scenario
- Disable unused validators to reduce computational load
- Tune correlation thresholds based on track environment
- Use appropriate buffer sizes for cross-sensor validation

## Troubleshooting

### Common Issues

#### Sanity Checking Not Working
```bash
# Check if sanity checking is enabled
ros2 param get /watchdog sanity_check_enabled

# Verify validator configuration
ros2 param dump /watchdog
```

#### High False Positive Rate
```bash
# Adjust validation thresholds
ros2 param set /watchdog battery_voltage_rate_limit 2.0
ros2 param set /watchdog lidar_noise_threshold 0.2
ros2 param set /watchdog temporal_sync_threshold 0.2
```

#### Cross-Sensor Validation Errors
```bash
# Check sensor synchronization
ros2 topic hz /scan
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /odom

# Verify correlation thresholds
ros2 param set /watchdog lidar_camera_correlation_threshold 0.6
```

### Debug Mode
```bash
# Enable verbose logging
ros2 param set /watchdog debug_mode true
ros2 run watchdog watchdog_node --ros-args --log-level DEBUG
```

## Safety Features

The watchdog monitors critical conditions for F1TENTH racing:

1. **Low Battery Voltage**: Triggers critical alert when voltage drops below threshold
2. **High Motor Temperature**: Monitors for thermal overload conditions  
3. **LiDAR Timeout**: Detects sensor communication failures
4. **Camera Timeout**: Monitors camera feed availability
5. **Sensor Anomalies**: Advanced validation of sensor data quality
6. **Cross-Sensor Inconsistencies**: Detection of multi-modal sensor conflicts

When critical conditions are detected, the node publishes emergency stop signals to prevent unsafe operation.

## Development

### Architecture

The watchdog system uses a modular validator architecture:

```
WatchdogNode
├── SanityChecker (orchestration)
│   ├── BatteryValidator
│   ├── LiDARValidator  
│   ├── CameraValidator
│   ├── OdometryValidator
│   └── CrossSensorValidator
└── Traditional monitoring (legacy)
```

### Adding New Validators

1. Inherit from `BaseValidator`
2. Implement the `validate()` method
3. Add configuration parameters
4. Register in `SanityChecker`
5. Add appropriate tests

### Code Quality

This package follows strict development standards:

-   PEP 8 style guidelines with type hints
-   Comprehensive docstrings and comments
-   Extensive error handling and logging
-   Modular design for maintainability
-   Unit and integration test coverage
-   Performance optimization for real-time operation

## Authors

-   **Fam Shihata** - _Maintainer_ - fam@awadlouis.com
-   **Mohammed Azab** - _Original co-author_ - mo7ammed3zab@outlook.com
-   **Nada Mahmoud** - _Original co-author_

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch
3. Add comprehensive tests
4. Follow coding standards
5. Submit a pull request

## Support

For questions and support:

- **Issues**: Open an issue on the repository
- **Documentation**: Check `config/watchdog_params.yaml` for parameter details
- **Community**: Join the F1TENTH community discussions
- **Maintainers**: Contact the maintainers directly

---

**Note**: This watchdog system is designed specifically for F1TENTH autonomous racing. Always test thoroughly in a safe environment before deployment in racing scenarios.
