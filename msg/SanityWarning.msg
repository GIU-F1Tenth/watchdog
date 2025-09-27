# SanityWarning.msg
# Custom message for reporting sensor data anomalies detected by the watchdog

# Which sensor detected the anomaly
string sensor_name

# Type of anomaly detected (e.g., "range_violation", "noise_excessive", "data_corrupted")
string anomaly_type

# Severity level: 1=INFO, 2=WARNING, 3=ERROR, 4=CRITICAL
uint8 severity

# Human-readable description of the problem
string description

# When the anomaly was detected
builtin_interfaces/Time timestamp

# Recommended action (e.g., "switch_to_backup", "reduce_speed", "stop_vehicle")
string suggested_action

# Confidence level of the detection (0.0 to 1.0)
float64 confidence

# Constants for severity levels
uint8 SEVERITY_INFO = 1
uint8 SEVERITY_WARNING = 2
uint8 SEVERITY_ERROR = 3
uint8 SEVERITY_CRITICAL = 4