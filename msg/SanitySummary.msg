# SanitySummary.msg
# Quick summary message for FSM decision making

# Overall system sanity status
bool system_healthy

# Are there any critical issues requiring immediate action?
bool has_critical_issues

# Are there any errors that require fallback actions?
bool has_errors

# Are there any warnings to be aware of?
bool has_warnings

# List of sensors with issues
string[] sensors_with_issues

# Highest severity level currently active (1-4)
uint8 max_severity_level

# Timestamp of this summary
builtin_interfaces/Time timestamp

# Brief description of most critical issue (empty if system_healthy=true)
string critical_issue_summary