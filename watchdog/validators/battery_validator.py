#!/usr/bin/env python3

"""
Battery/VESC Validator for Watchdog Sanity Checking

This validator checks battery and motor controller (VESC) data for anomalies including:
- Voltage rate of change validation
- Current spike detection  
- Temperature trend analysis
- Historical trend monitoring
- Cross-correlation validation

Author: F1TENTH Watchdog Team
License: MIT
Version: 1.0.0
"""

import time
from typing import Any, List, Dict, Optional
from collections import deque

try:
    from ..base_validator import BaseValidator, ValidationResult, SeverityLevel
except ImportError:
    import sys
    import os
    sys.path.append(os.path.dirname(os.path.dirname(__file__)))
    from base_validator import BaseValidator, ValidationResult, SeverityLevel


class BatteryValidator(BaseValidator):
    """
    Validator for battery and VESC controller data.
    
    Monitors voltage, current, and temperature readings for anomalies
    that could indicate hardware issues or unsafe operating conditions.
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize battery validator with configuration parameters.
        
        Args:
            config: Configuration dictionary containing battery validation parameters
        """
        super().__init__('battery', config)
        
        # Load battery-specific configuration
        self.max_voltage_change_rate = config.get('battery_max_voltage_change_rate', 1.0)  # V/s
        self.trend_window_size = config.get('battery_trend_window_size', 10)
        self.spike_threshold = config.get('battery_spike_threshold', 2.0)  # V
        self.temp_rate_threshold = config.get('motor_temp_rate_threshold', 5.0)  # °C/s
        self.current_max = config.get('motor_current_max', 50.0)  # A
        self.current_spike_threshold = config.get('motor_current_spike_threshold', 10.0)  # A
        
        # State tracking
        self.voltage_history = deque(maxlen=self.trend_window_size)
        self.current_history = deque(maxlen=self.trend_window_size)
        self.temperature_history = deque(maxlen=self.trend_window_size)
        self.timestamp_history = deque(maxlen=self.trend_window_size)
        
        # Previous values for rate calculations
        self.previous_voltage = None
        self.previous_current = None
        self.previous_temperature = None
        self.previous_timestamp = None
        
    def validate(self, data: Any) -> ValidationResult:
        """
        Validate battery/VESC data.
        
        Args:
            data: VESC state data (expected to have voltage_input, current_motor, temp_motor attributes)
            
        Returns:
            ValidationResult with validation outcome
        """
        current_time = time.time()
        
        # Extract values from data (assuming VescStateStamped structure)
        try:
            voltage = self._extract_voltage(data)
            current = self._extract_current(data)  
            temperature = self._extract_temperature(data)
        except Exception as e:
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.ERROR,
                anomaly_type="data_extraction_error",
                description=f"Failed to extract battery data: {str(e)}",
                suggested_action="check_data_format",
                confidence=1.0
            )
        
        # Update history
        self.voltage_history.append(voltage)
        self.current_history.append(current)
        self.temperature_history.append(temperature)
        self.timestamp_history.append(current_time)
        
        # Perform validation checks
        validation_results = []
        
        # 1. Basic range validation
        range_result = self._validate_ranges(voltage, current, temperature, data)
        if range_result:
            validation_results.append(range_result)
            
        # 2. Rate of change validation (if we have previous data)
        if (self.previous_voltage is not None and 
            self.previous_timestamp is not None):
            
            rate_result = self._validate_rates(voltage, current, temperature, 
                                             current_time, data)
            if rate_result:
                validation_results.append(rate_result)
                
        # 3. Spike detection
        spike_result = self._detect_spikes(voltage, current, data)
        if spike_result:
            validation_results.append(spike_result)
            
        # 4. Trend analysis (if we have enough history)
        if len(self.voltage_history) >= 5:
            trend_result = self._analyze_trends(data)
            if trend_result:
                validation_results.append(trend_result)
        
        # Store current values for next validation
        self.previous_voltage = voltage
        self.previous_current = current
        self.previous_temperature = temperature
        self.previous_timestamp = current_time
        
        # Determine overall result
        if not validation_results:
            return self._create_result(
                is_valid=True,
                severity=SeverityLevel.INFO,
                anomaly_type="none",
                description="Battery/VESC data appears healthy",
                suggested_action="continue",
                confidence=1.0,
                raw_data=data
            )
        else:
            # Return the most severe issue found
            most_severe = max(validation_results, key=lambda x: x.severity)
            return most_severe
    
    def _extract_voltage(self, data: Any) -> float:
        """Extract voltage from data object."""
        if hasattr(data, 'state') and hasattr(data.state, 'voltage_input'):
            return float(data.state.voltage_input)
        elif hasattr(data, 'voltage_input'):
            return float(data.voltage_input)
        elif hasattr(data, 'voltage'):
            return float(data.voltage)
        else:
            raise ValueError("Could not find voltage field in data")
    
    def _extract_current(self, data: Any) -> float:
        """Extract current from data object."""
        if hasattr(data, 'state') and hasattr(data.state, 'current_motor'):
            return float(data.state.current_motor)
        elif hasattr(data, 'current_motor'):
            return float(data.current_motor)
        elif hasattr(data, 'current'):
            return float(data.current)
        else:
            raise ValueError("Could not find current field in data")
    
    def _extract_temperature(self, data: Any) -> float:
        """Extract temperature from data object."""
        if hasattr(data, 'state') and hasattr(data.state, 'temp_motor'):
            return float(data.state.temp_motor)
        elif hasattr(data, 'temp_motor'):
            return float(data.temp_motor)
        elif hasattr(data, 'temperature'):
            return float(data.temperature)
        else:
            raise ValueError("Could not find temperature field in data")
    
    def _validate_ranges(self, voltage: float, current: float, 
                        temperature: float, data: Any) -> Optional[ValidationResult]:
        """
        Validate that readings are within expected ranges.
        
        Args:
            voltage: Battery voltage (V)
            current: Motor current (A)
            temperature: Motor temperature (°C)
            data: Original data
            
        Returns:
            ValidationResult if issues found, None otherwise
        """
        # Check voltage range (using existing watchdog thresholds)
        min_voltage = self.config.get('min_voltage', 9.0)
        max_voltage = self.config.get('max_voltage', 52.0)
        critical_voltage = self.config.get('critical_voltage', 12.0)
        
        if voltage < critical_voltage:
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.CRITICAL,
                anomaly_type="voltage_critically_low",
                description=f"Battery voltage {voltage:.2f}V is critically low (< {critical_voltage}V)",
                suggested_action="immediate_shutdown_or_charge",
                confidence=1.0,
                raw_data=data
            )
        elif voltage < min_voltage:
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.ERROR,
                anomaly_type="voltage_too_low",
                description=f"Battery voltage {voltage:.2f}V is below minimum ({min_voltage}V)",
                suggested_action="charge_battery_soon",
                confidence=0.9,
                raw_data=data
            )
        elif voltage > max_voltage:
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.ERROR,
                anomaly_type="voltage_too_high",
                description=f"Battery voltage {voltage:.2f}V exceeds maximum ({max_voltage}V)",
                suggested_action="check_charging_system",
                confidence=0.9,
                raw_data=data
            )
        
        # Check current range
        if abs(current) > self.current_max:
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.WARNING,
                anomaly_type="current_excessive",
                description=f"Motor current {current:.1f}A exceeds maximum ({self.current_max}A)",
                suggested_action="reduce_load_or_check_motor",
                confidence=0.8,
                raw_data=data
            )
        
        # Check temperature range (using existing watchdog thresholds)
        temp_critical = self.config.get('temp_critical', 100.0)
        temp_warning_high = self.config.get('temp_warning_high', 90.0)
        
        if temperature > temp_critical:
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.CRITICAL,
                anomaly_type="temperature_critical",
                description=f"Motor temperature {temperature:.1f}°C is critical (> {temp_critical}°C)",
                suggested_action="immediate_shutdown_cooling",
                confidence=1.0,
                raw_data=data
            )
        elif temperature > temp_warning_high:
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.WARNING,
                anomaly_type="temperature_high",
                description=f"Motor temperature {temperature:.1f}°C is high (> {temp_warning_high}°C)",
                suggested_action="reduce_load_monitor_cooling",
                confidence=0.9,
                raw_data=data
            )
        
        return None
    
    def _validate_rates(self, voltage: float, current: float, temperature: float,
                       current_time: float, data: Any) -> Optional[ValidationResult]:
        """
        Validate rates of change for voltage, current, and temperature.
        
        Args:
            voltage: Current voltage
            current: Current current
            temperature: Current temperature  
            current_time: Current timestamp
            data: Original data
            
        Returns:
            ValidationResult if issues found, None otherwise
        """
        time_delta = current_time - self.previous_timestamp
        if time_delta <= 0:
            return None  # Invalid time delta
        
        # Calculate rates of change
        voltage_rate = abs(voltage - self.previous_voltage) / time_delta
        current_rate = abs(current - self.previous_current) / time_delta
        temp_rate = abs(temperature - self.previous_temperature) / time_delta
        
        # Check voltage rate of change
        if voltage_rate > self.max_voltage_change_rate:
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.WARNING,
                anomaly_type="voltage_rate_excessive",
                description=f"Voltage changing too rapidly: {voltage_rate:.2f}V/s (max: {self.max_voltage_change_rate}V/s)",
                suggested_action="check_connections_or_load",
                confidence=0.8,
                raw_data=data
            )
        
        # Check temperature rate of change
        if temp_rate > self.temp_rate_threshold:
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.WARNING,
                anomaly_type="temperature_rate_excessive",
                description=f"Temperature changing too rapidly: {temp_rate:.1f}°C/s (max: {self.temp_rate_threshold}°C/s)",
                suggested_action="check_cooling_system",
                confidence=0.8,
                raw_data=data
            )
        
        return None
    
    def _detect_spikes(self, voltage: float, current: float, 
                      data: Any) -> Optional[ValidationResult]:
        """
        Detect sudden spikes in voltage or current.
        
        Args:
            voltage: Current voltage
            current: Current current
            data: Original data
            
        Returns:
            ValidationResult if spikes detected, None otherwise
        """
        if self.previous_voltage is None:
            return None
            
        voltage_change = abs(voltage - self.previous_voltage)
        current_change = abs(current - self.previous_current)
        
        # Check voltage spike
        if voltage_change > self.spike_threshold:
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.WARNING,
                anomaly_type="voltage_spike",
                description=f"Voltage spike detected: {voltage_change:.2f}V change (threshold: {self.spike_threshold}V)",
                suggested_action="check_power_supply_stability",
                confidence=0.9,
                raw_data=data
            )
        
        # Check current spike
        if current_change > self.current_spike_threshold:
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.WARNING,
                anomaly_type="current_spike",
                description=f"Current spike detected: {current_change:.1f}A change (threshold: {self.current_spike_threshold}A)",
                suggested_action="check_motor_or_load",
                confidence=0.9,
                raw_data=data
            )
        
        return None
    
    def _analyze_trends(self, data: Any) -> Optional[ValidationResult]:
        """
        Analyze trends in voltage, current, and temperature.
        
        Args:
            data: Original data
            
        Returns:
            ValidationResult if concerning trends found, None otherwise
        """
        voltage_list = list(self.voltage_history)
        temperature_list = list(self.temperature_history)
        
        # Analyze voltage trend
        voltage_trend = self._detect_trend(voltage_list)
        if voltage_trend == 'decreasing':
            # Check if voltage is consistently decreasing (battery drain)
            recent_drop = voltage_list[-1] - voltage_list[0]
            if recent_drop < -1.0:  # More than 1V drop over trend window
                return self._create_result(
                    is_valid=False,
                    severity=SeverityLevel.WARNING,
                    anomaly_type="voltage_declining_trend",
                    description=f"Battery voltage declining trend: {recent_drop:.2f}V drop over {len(voltage_list)} readings",
                    suggested_action="monitor_battery_closely",
                    confidence=0.7,
                    raw_data=data
                )
        
        # Analyze temperature trend
        temp_trend = self._detect_trend(temperature_list)
        if temp_trend == 'increasing':
            recent_rise = temperature_list[-1] - temperature_list[0]
            if recent_rise > 10.0:  # More than 10°C rise
                return self._create_result(
                    is_valid=False,
                    severity=SeverityLevel.WARNING,
                    anomaly_type="temperature_rising_trend",
                    description=f"Motor temperature rising trend: {recent_rise:.1f}°C rise over {len(temperature_list)} readings",
                    suggested_action="check_cooling_reduce_load",
                    confidence=0.8,
                    raw_data=data
                )
        
        return None