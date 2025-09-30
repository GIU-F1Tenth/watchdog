# F1TENTH Watchdog API Reference

## Message Types

### SanityWarning

Real-time validation warnings published when anomalies are detected.

**Topic**: `/watchdog/sanity_warnings`  
**Type**: `watchdog_msgs/SanityWarning`

```yaml
Header header                  # Standard ROS header with timestamp
string sensor_name            # Name of the sensor (e.g., "battery", "lidar", "camera", "odometry", "cross_sensor")
string anomaly_type           # Type of anomaly detected
string description            # Human-readable description of the issue
string suggested_action       # Recommended action to take
uint8 severity               # Severity level (INFO=0, WARNING=1, ERROR=2, CRITICAL=3)
float64 confidence           # Confidence score (0.0 to 1.0)
```

**Example Usage**:
```python
def sanity_warning_callback(self, msg):
    if msg.severity >= 2:  # ERROR or CRITICAL
        self.log_critical_issue(msg.description)
        if msg.suggested_action == "emergency_stop":
            self.trigger_emergency_stop()
```

**Common Anomaly Types**:
- `VOLTAGE_OUT_OF_RANGE`
- `VOLTAGE_RATE_LIMIT_EXCEEDED`
- `RANGE_OUT_OF_BOUNDS`
- `HIGH_NOISE_LEVEL`
- `BRIGHTNESS_TOO_LOW`
- `BLUR_DETECTED`
- `VELOCITY_EXCEEDED`
- `ACCELERATION_EXCEEDED`
- `TEMPORAL_DESYNC`
- `LIDAR_CAMERA_CORRELATION_LOW`

---

### SensorHealth

Individual sensor health status reports.

**Topic**: `/watchdog/sensor_health`  
**Type**: `watchdog_msgs/SensorHealth`

```yaml
Header header                 # Standard ROS header with timestamp
string sensor_name           # Name of the sensor
bool is_healthy             # Overall health status (true/false)
float64 health_score        # Health score (0.0 = unhealthy, 1.0 = perfect health)
string[] active_warnings    # List of active warning types for this sensor
float64 last_data_time      # Timestamp of last valid data received
string status_description   # Detailed status information
```

**Example Usage**:
```python
def sensor_health_callback(self, msg):
    if not msg.is_healthy:
        self.handle_unhealthy_sensor(msg.sensor_name, msg.active_warnings)
    
    if msg.health_score < 0.7:
        self.enable_degraded_mode(msg.sensor_name)
```

**Health Score Interpretation**:
- `1.0`: Perfect operation, no issues detected
- `0.8-0.99`: Good operation, minor warnings
- `0.6-0.79`: Degraded operation, monitoring recommended
- `0.4-0.59`: Poor operation, intervention may be needed
- `0.0-0.39`: Critical issues, immediate attention required

---

### SanitySummary

Comprehensive system health summary combining all sensor health reports.

**Topic**: `/watchdog/sanity_summary`  
**Type**: `watchdog_msgs/SanitySummary`

```yaml
Header header                     # Standard ROS header with timestamp
bool overall_system_healthy       # Overall system health status
float64 overall_health_score      # Combined health score (0.0 to 1.0)
SensorHealth[] sensor_healths     # Array of individual sensor health reports
string[] critical_issues          # List of critical issues requiring immediate attention
string[] warnings                # List of warnings that should be monitored
string system_status             # Overall system status description
```

**Example Usage**:
```python
def sanity_summary_callback(self, msg):
    if not msg.overall_system_healthy:
        self.handle_system_degradation(msg.critical_issues)
    
    # Update FSM based on overall health
    if msg.overall_health_score < 0.5:
        self.transition_to_safe_mode()
    elif msg.overall_health_score < 0.8:
        self.enable_conservative_driving()
```

**System Status Values**:
- `"HEALTHY"`: All systems operating normally
- `"DEGRADED"`: Some issues detected, operation possible with caution
- `"CRITICAL"`: Serious issues detected, immediate attention required
- `"EMERGENCY"`: Critical failure, emergency stop recommended

---

## Service Interfaces

### Get System Health

**Service**: `/watchdog/get_system_health`  
**Type**: `watchdog_msgs/GetSystemHealth`

```yaml
# Request (empty)
---
# Response
bool success
string message
SanitySummary current_health
```

### Reset Validator

**Service**: `/watchdog/reset_validator`  
**Type**: `watchdog_msgs/ResetValidator`

```yaml
# Request
string sensor_name      # Name of validator to reset, or "all"
---
# Response
bool success
string message
```

---

## Parameter Interface

### Core Parameters

```bash
# Enable/disable sanity checking
ros2 param set /watchdog sanity_check_enabled true

# Adjust validation frequency
ros2 param set /watchdog sanity_check_frequency 5.0

# Enable specific validators
ros2 param set /watchdog battery_validation_enabled true
ros2 param set /watchdog lidar_validation_enabled true
ros2 param set /watchdog camera_validation_enabled true
ros2 param set /watchdog odometry_validation_enabled true
ros2 param set /watchdog cross_sensor_validation_enabled true
```

### Validation Thresholds

```bash
# Battery validation
ros2 param set /watchdog battery_voltage_min 11.0
ros2 param set /watchdog battery_voltage_max 13.0
ros2 param set /watchdog battery_voltage_rate_limit 1.0

# LiDAR validation
ros2 param set /watchdog lidar_range_min 0.1
ros2 param set /watchdog lidar_range_max 30.0
ros2 param set /watchdog lidar_noise_threshold 0.1

# Camera validation
ros2 param set /watchdog camera_brightness_threshold 50
ros2 param set /watchdog camera_blur_threshold 100.0

# Cross-sensor validation
ros2 param set /watchdog lidar_camera_correlation_threshold 0.8
ros2 param set /watchdog temporal_sync_threshold 0.1
```

---

## Topic Monitoring

### Essential Topics to Monitor

```bash
# Real-time warnings
ros2 topic echo /watchdog/sanity_warnings

# Overall health summary
ros2 topic echo /watchdog/sanity_summary

# Individual sensor health
ros2 topic echo /watchdog/sensor_health

# Critical alerts
ros2 topic echo /tmp/watchdog/critical
```

### Topic Frequency Monitoring

```bash
# Check update rates
ros2 topic hz /watchdog/sanity_warnings
ros2 topic hz /watchdog/sensor_health
ros2 topic hz /watchdog/sanity_summary
```

---

## Integration Patterns

### FSM Integration

```python
class F1TENTHStateMachine:
    def __init__(self):
        # Subscribe to critical alerts
        self.critical_sub = self.create_subscription(
            Bool, '/tmp/watchdog/critical', 
            self.emergency_stop_callback, 10)
        
        # Subscribe to health summary
        self.health_sub = self.create_subscription(
            SanitySummary, '/watchdog/sanity_summary',
            self.health_callback, 10)
        
        # Subscribe to individual warnings
        self.warning_sub = self.create_subscription(
            SanityWarning, '/watchdog/sanity_warnings',
            self.warning_callback, 10)
    
    def emergency_stop_callback(self, msg):
        if msg.data:
            self.transition_to_emergency_stop()
    
    def health_callback(self, msg):
        if not msg.overall_system_healthy:
            self.handle_degraded_performance(msg.critical_issues)
        elif msg.overall_health_score < 0.7:
            self.enable_conservative_mode()
    
    def warning_callback(self, msg):
        if msg.severity >= 2:  # ERROR or CRITICAL
            self.log_issue(f"{msg.sensor_name}: {msg.description}")
            
        if msg.suggested_action == "reduce_speed":
            self.set_max_velocity(self.current_max_velocity * 0.8)
```

### Telemetry Integration

```python
class TelemetryLogger:
    def __init__(self):
        self.health_sub = self.create_subscription(
            SanitySummary, '/watchdog/sanity_summary',
            self.log_health_data, 10)
        
        self.warning_sub = self.create_subscription(
            SanityWarning, '/watchdog/sanity_warnings',
            self.log_warning, 10)
    
    def log_health_data(self, msg):
        # Log to database/file
        health_data = {
            'timestamp': msg.header.stamp,
            'overall_healthy': msg.overall_system_healthy,
            'health_score': msg.overall_health_score,
            'sensor_count': len(msg.sensor_healths),
            'critical_issues': len(msg.critical_issues),
            'warnings': len(msg.warnings)
        }
        self.database.log_health(health_data)
    
    def log_warning(self, msg):
        warning_data = {
            'timestamp': msg.header.stamp,
            'sensor': msg.sensor_name,
            'type': msg.anomaly_type,
            'severity': msg.severity,
            'confidence': msg.confidence,
            'description': msg.description
        }
        self.database.log_warning(warning_data)
```

---

## Error Handling

### Common Error Scenarios

1. **Validator Initialization Failure**
   ```python
   try:
       validator = BatteryValidator(config)
   except ValueError as e:
       self.get_logger().error(f"Battery validator config error: {e}")
       # Disable battery validation or use defaults
   ```

2. **Data Processing Errors**
   ```python
   try:
       result = validator.validate(sensor_data)
   except Exception as e:
       self.get_logger().warn(f"Validation error for {sensor_name}: {e}")
       # Return default "healthy" result or skip validation
   ```

3. **Message Publishing Failures**
   ```python
   try:
       self.sanity_warning_pub.publish(warning_msg)
   except Exception as e:
       self.get_logger().error(f"Failed to publish warning: {e}")
       # Log to file or try alternative communication
   ```

### Recovery Strategies

1. **Graceful Degradation**: Disable problematic validators
2. **Default Values**: Use conservative defaults when configuration is invalid
3. **Retry Logic**: Implement retry for transient failures
4. **Fallback Communication**: Use multiple channels for critical alerts

---

## Performance Considerations

### Computational Efficiency

- **Validation Frequency**: Balance between responsiveness and CPU usage
- **Buffer Sizes**: Larger buffers improve accuracy but increase memory usage
- **Correlation Calculations**: Most CPU-intensive operation in cross-sensor validation
- **Message Publishing**: High-frequency publishing can impact network performance

### Memory Management

- **Circular Buffers**: Used to limit memory growth
- **Data Retention**: Configure appropriate buffer sizes for available memory
- **Message Queues**: Monitor queue depths to prevent memory leaks

### Real-time Performance

- **Update Rates**: Typical rates: 10Hz core monitoring, 5Hz sanity checking
- **Latency**: Critical alerts should propagate within 10ms
- **Jitter**: Monitor timing consistency for critical operations
- **Priority**: Consider process/thread priority for time-critical components

---

For more detailed information, see the [Configuration Guide](CONFIGURATION.md) and main [README](../README.md).