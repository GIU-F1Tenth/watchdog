# F1TENTH Watchdog Configuration Guide

This guide provides detailed information on configuring the F1TENTH Watchdog system for optimal performance in different racing scenarios.

## Configuration File Structure

The watchdog system uses a hierarchical YAML configuration file located at `config/watchdog_params.yaml`. All parameters are organized by functional category.

## Core Monitoring Parameters

### Battery Monitoring
```yaml
# Battery voltage thresholds
critical_voltage: 12.0        # Critical alert threshold (V)
low_voltage_threshold: 12.5   # Warning threshold (V)
very_low_voltage: 11.5        # Emergency stop threshold (V)
```

**Tuning Guidelines:**
- **Track Racing**: Use stricter thresholds (12.5V critical) for safety
- **Practice/Testing**: More relaxed thresholds (11.8V critical) for extended sessions
- **Indoor Tracks**: Standard settings work well
- **Outdoor Tracks**: Consider temperature effects on battery performance

### Temperature Monitoring
```yaml
# Motor temperature limits (°C)
temp_warning_start: 80.0      # Start warning alerts
temp_warning_stop: 75.0       # Stop warning alerts (hysteresis)
temp_critical: 90.0           # Critical temperature threshold
temp_emergency: 95.0          # Emergency stop threshold
```

**Tuning Guidelines:**
- **High-Speed Racing**: Lower thresholds (75°C warning, 85°C critical)
- **Endurance Racing**: Monitor trends, allow higher peaks (85°C warning, 95°C critical)
- **Hot Environments**: Reduce thresholds by 5-10°C
- **Cold Environments**: Standard settings appropriate

### Timeout Settings
```yaml
# Sensor timeout values (seconds)
lidar_timeout: 0.5            # LiDAR message timeout
camera_timeout: 0.5           # Camera frame timeout
odom_timeout: 0.1             # Odometry timeout
sensor_timeout: 1.0           # General sensor timeout
```

**Tuning Guidelines:**
- **High-Frequency Racing**: Reduce timeouts (0.2s for critical sensors)
- **Development/Debug**: Increase timeouts (1.0s) to avoid false alarms
- **Network Issues**: Increase timeouts progressively
- **Real-time Requirements**: Keep as low as possible without false positives

## Sanity Checking Parameters

### Global Sanity Check Settings
```yaml
# Master enable/disable
sanity_check_enabled: true    # Enable all sanity checking
sanity_check_frequency: 5.0   # Validation frequency (Hz)
sanity_check_buffer_size: 50  # Data buffer size for analysis
```

### Battery Validation
```yaml
# Battery sanity checking
battery_validation_enabled: true
battery_voltage_min: 11.0         # Minimum acceptable voltage (V)
battery_voltage_max: 13.0         # Maximum acceptable voltage (V)
battery_voltage_rate_limit: 1.0   # Max voltage change rate (V/s)
battery_current_max: 50.0         # Maximum current draw (A)
battery_consistency_window: 10    # Samples for consistency check
```

**Tuning Guidelines:**
- **Aggressive Racing**: Tighter rate limits (0.5V/s) to catch sudden drops
- **Gradual Degradation**: Wider limits (2.0V/s) to avoid false alarms
- **New Batteries**: Standard limits work well
- **Aging Batteries**: Relax voltage_min slightly (10.8V)

### LiDAR Validation
```yaml
# LiDAR sanity checking
lidar_validation_enabled: true
lidar_range_min: 0.1              # Minimum valid range (m)
lidar_range_max: 30.0             # Maximum valid range (m)
lidar_angle_min: -3.14159         # Minimum scan angle (rad)
lidar_angle_max: 3.14159          # Maximum scan angle (rad)
lidar_noise_threshold: 0.1        # Noise level threshold
lidar_dropout_threshold: 0.05     # Max fraction of dropout points
lidar_consistency_window: 5       # Samples for consistency check
```

**Tuning Guidelines:**
- **Indoor Tracks**: Reduce range_max (10.0m), increase noise_threshold (0.15)
- **Outdoor Tracks**: Standard settings, monitor for environmental interference
- **Dusty Conditions**: Relax noise_threshold (0.2) and dropout_threshold (0.1)
- **Clean Environments**: Tighten thresholds for better anomaly detection

### Camera Validation
```yaml
# Camera sanity checking
camera_validation_enabled: true
camera_brightness_threshold: 50      # Minimum brightness
camera_contrast_threshold: 20        # Minimum contrast
camera_blur_threshold: 100.0         # Maximum blur metric
camera_noise_threshold: 30.0         # Maximum noise level
camera_consistency_window: 10        # Samples for consistency check
```

**Tuning Guidelines:**
- **Variable Lighting**: Lower brightness_threshold (30), adjust contrast (15)
- **Consistent Lighting**: Higher thresholds for stricter validation
- **High-Speed Racing**: Adjust blur_threshold based on motion blur tolerance
- **Image Quality Critical**: Tighten all thresholds for better quality assurance

### Odometry Validation
```yaml
# Odometry sanity checking
odometry_validation_enabled: true
odometry_velocity_max: 10.0          # Maximum linear velocity (m/s)
odometry_angular_velocity_max: 5.0   # Maximum angular velocity (rad/s)
odometry_acceleration_max: 15.0      # Maximum acceleration (m/s²)
odometry_jerk_threshold: 50.0        # Maximum jerk (m/s³)
odometry_consistency_window: 10      # Samples for consistency check
```

**Tuning Guidelines:**
- **High-Speed Racing**: Increase velocity_max (15.0m/s), acceleration_max (20.0m/s²)
- **Technical Tracks**: Standard settings with tight turning limits
- **Aggressive Driving**: Relax jerk_threshold (75.0) for rapid maneuvers
- **Smooth Driving**: Tighten thresholds for anomaly detection

### Cross-Sensor Validation
```yaml
# Cross-sensor correlation
cross_sensor_validation_enabled: true
lidar_camera_correlation_threshold: 0.8      # Min LiDAR-camera correlation
odometry_visual_odometry_threshold: 0.2      # Max odom-visual odom difference
position_consistency_threshold: 0.5          # Max position discrepancy (m)
temporal_sync_threshold: 0.1                 # Max time difference (s)
environment_consistency_window: 10           # Samples for environment check
cross_sensor_buffer_size: 50                 # Buffer size for correlation
```

**Tuning Guidelines:**
- **Calibrated Systems**: Use strict correlation_threshold (0.85)
- **Approximate Calibration**: Relax threshold (0.7) to avoid false alarms
- **High-Frequency Operation**: Reduce temporal_sync_threshold (0.05s)
- **Variable Conditions**: Increase position_consistency_threshold (0.8m)

## Scenario-Specific Configurations

### Competition Racing Configuration
```yaml
# Aggressive settings for maximum safety
critical_voltage: 12.5
temp_warning_start: 75.0
lidar_timeout: 0.2
camera_timeout: 0.2
sanity_check_frequency: 10.0
battery_voltage_rate_limit: 0.5
lidar_noise_threshold: 0.05
temporal_sync_threshold: 0.05
```

### Practice/Development Configuration
```yaml
# Relaxed settings for extended operation
critical_voltage: 12.0
temp_warning_start: 85.0
lidar_timeout: 0.5
camera_timeout: 0.5
sanity_check_frequency: 5.0
battery_voltage_rate_limit: 1.5
lidar_noise_threshold: 0.15
temporal_sync_threshold: 0.1
```

### Endurance Racing Configuration
```yaml
# Balanced settings for long-term operation
critical_voltage: 12.2
temp_warning_start: 80.0
lidar_timeout: 0.3
camera_timeout: 0.3
sanity_check_frequency: 5.0
battery_voltage_rate_limit: 1.0
battery_consistency_window: 20
lidar_consistency_window: 10
```

## Dynamic Configuration

### Runtime Parameter Changes
```bash
# Adjust parameters during operation
ros2 param set /watchdog critical_voltage 12.3
ros2 param set /watchdog sanity_check_frequency 8.0
ros2 param set /watchdog temporal_sync_threshold 0.08

# Save current parameters
ros2 param dump /watchdog > current_config.yaml
```

### Conditional Configurations
```yaml
# Use different configurations based on conditions
track_type: "indoor"  # indoor, outdoor, technical, high_speed
lighting_conditions: "variable"  # consistent, variable, low_light
performance_mode: "race"  # race, practice, endurance, debug
```

## Troubleshooting Configuration Issues

### High False Positive Rate
1. Check actual sensor performance vs. thresholds
2. Increase noise_threshold and dropout_threshold gradually
3. Extend consistency_window for more stable validation
4. Relax correlation thresholds if sensors are not well-calibrated

### Missed Real Anomalies
1. Tighten validation thresholds gradually
2. Increase sanity_check_frequency for faster detection
3. Reduce consistency_window for more responsive validation
4. Enable additional validators if available

### Performance Issues
1. Reduce sanity_check_frequency
2. Decrease buffer sizes
3. Disable unused validators
4. Optimize consistency_window sizes

### Sensor Synchronization Issues
1. Check actual sensor publication rates
2. Adjust temporal_sync_threshold based on measured jitter
3. Verify system clock synchronization
4. Consider network latency in timeout values

## Validation and Testing

### Configuration Validation
```bash
# Test configuration before racing
ros2 launch watchdog watchdog.launch.py config_file:=test_config.yaml

# Monitor validation results
ros2 topic echo /watchdog/sanity_warnings
ros2 topic echo /watchdog/sanity_summary
```

### Performance Monitoring
```bash
# Check CPU and memory usage
top -p $(pgrep -f watchdog_node)

# Monitor update rates
ros2 topic hz /watchdog/sensor_health
ros2 topic hz /watchdog/sanity_summary
```

### Threshold Tuning Process
1. Start with conservative (relaxed) thresholds
2. Collect baseline performance data
3. Gradually tighten thresholds while monitoring false positives
4. Test with known failure scenarios
5. Validate in actual racing conditions
6. Document final configuration with rationale

## Best Practices

1. **Version Control**: Keep configuration files in version control
2. **Documentation**: Document all threshold changes with rationale
3. **Testing**: Test configuration changes in safe environments first
4. **Monitoring**: Continuously monitor validation performance
5. **Backup**: Keep working configurations as backups
6. **Gradual Changes**: Make incremental threshold adjustments
7. **Scenario-Specific**: Use different configs for different scenarios
8. **Team Communication**: Share configuration changes with team members

## Support and Resources

- **Default Configuration**: `config/watchdog_params.yaml`
- **Example Configurations**: `config/examples/`
- **Documentation**: README.md and inline code comments
- **Community**: F1TENTH forums and Discord
- **Issues**: GitHub repository issues

---

For additional configuration support, please refer to the main README.md or contact the maintainers.