# SensorHealth.msg
# Custom message for reporting overall health status of individual sensors

# Name of the sensor (e.g., "lidar", "camera", "battery", "odometry")
string sensor_name

# Overall health score from 0.0 (completely failed) to 1.0 (perfect health)
float64 health_score

# Whether the sensor data is currently considered valid/usable
bool is_valid

# List of currently active anomalies for this sensor
string[] active_anomalies

# Timestamp of the last health assessment
builtin_interfaces/Time last_update

# Number of anomalies detected in the last assessment window
uint32 anomaly_count

# Average confidence of recent assessments
float64 average_confidence