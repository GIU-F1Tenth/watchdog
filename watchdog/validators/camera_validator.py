#!/usr/bin/env python3

"""
Camera Validator for Watchdog Sanity Checking

This validator checks camera sensor data for various anomalies including:
- Image quality assessment (corruption, blank frames)
- Brightness validation (over/under exposure)
- Blur detection using variance of Laplacian
- Dimension validation
- Timestamp consistency checking
- Noise detection

Author: F1TENTH Watchdog Team
License: MIT
Version: 1.0.0
"""

import time
import numpy as np
from typing import Any, List, Dict, Optional
from collections import deque
from sensor_msgs.msg import Image

try:
    from ..base_validator import BaseValidator, ValidationResult, SeverityLevel
except ImportError:
    import sys
    import os
    sys.path.append(os.path.dirname(os.path.dirname(__file__)))
    from base_validator import BaseValidator, ValidationResult, SeverityLevel


class CameraValidator(BaseValidator):
    """
    Validator for camera sensor data (Image messages).
    
    Performs comprehensive validation of camera data including quality checks,
    brightness validation, blur detection, and timestamp consistency.
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize camera validator with configuration parameters.
        
        Args:
            config: Configuration dictionary containing camera validation parameters
        """
        super().__init__('camera', config)
        
        # Load camera-specific configuration
        self.min_brightness = config.get('camera_min_brightness', 15)
        self.max_brightness = config.get('camera_max_brightness', 240)
        self.blur_threshold = config.get('camera_blur_threshold', 100.0)
        self.noise_threshold = config.get('camera_noise_threshold', 0.1)
        self.min_width = config.get('camera_min_width', 320)
        self.min_height = config.get('camera_min_height', 240)
        self.max_timestamp_gap = config.get('camera_max_timestamp_gap', 0.2)
        
        # State tracking
        self.previous_timestamp = None
        self.brightness_history = deque(maxlen=10)
        self.blur_history = deque(maxlen=5)
        self.frame_count = 0
        
    def validate(self, data: Image) -> ValidationResult:
        """
        Validate camera image data.
        
        Args:
            data: Image message to validate
            
        Returns:
            ValidationResult with validation outcome
        """
        if not isinstance(data, Image):
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.ERROR,
                anomaly_type="invalid_data_type",
                description=f"Expected Image, got {type(data).__name__}",
                suggested_action="check_data_source",
                confidence=1.0
            )
        
        self.frame_count += 1
        current_time = time.time()
        
        # Perform all validation checks
        validation_results = []
        
        # 1. Basic image validation (dimensions, encoding)
        basic_result = self._validate_basic_properties(data)
        if basic_result:
            validation_results.append(basic_result)
            
        # 2. Timestamp validation
        if self.previous_timestamp is not None:
            timestamp_result = self._validate_timestamp(data, current_time)
            if timestamp_result:
                validation_results.append(timestamp_result)
        
        # 3. Image content validation (if we can decode the image)
        try:
            image_array = self._decode_image(data)
            if image_array is not None:
                
                # Brightness validation
                brightness_result = self._validate_brightness(image_array, data)
                if brightness_result:
                    validation_results.append(brightness_result)
                
                # Blur detection
                blur_result = self._detect_blur(image_array, data)
                if blur_result:
                    validation_results.append(blur_result)
                    
                # Noise detection
                noise_result = self._detect_noise(image_array, data)
                if noise_result:
                    validation_results.append(noise_result)
                    
                # Corruption detection
                corruption_result = self._detect_corruption(image_array, data)
                if corruption_result:
                    validation_results.append(corruption_result)
                    
        except Exception as e:
            validation_results.append(self._create_result(
                is_valid=False,
                severity=SeverityLevel.WARNING,
                anomaly_type="image_decode_error",
                description=f"Failed to decode image for content analysis: {str(e)}",
                suggested_action="check_image_encoding",
                confidence=0.8,
                raw_data=data
            ))
        
        # Update state
        self.previous_timestamp = current_time
        
        # Determine overall result
        if not validation_results:
            return self._create_result(
                is_valid=True,
                severity=SeverityLevel.INFO,
                anomaly_type="none",
                description="Camera image appears healthy",
                suggested_action="continue",
                confidence=1.0,
                raw_data=data
            )
        else:
            # Return the most severe issue found
            most_severe = max(validation_results, key=lambda x: x.severity)
            return most_severe
    
    def _validate_basic_properties(self, data: Image) -> Optional[ValidationResult]:
        """
        Validate basic image properties like dimensions and encoding.
        
        Args:
            data: Image message
            
        Returns:
            ValidationResult if issues found, None otherwise
        """
        # Check dimensions
        if data.width < self.min_width or data.height < self.min_height:
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.ERROR,
                anomaly_type="invalid_dimensions",
                description=f"Image dimensions {data.width}x{data.height} below minimum {self.min_width}x{self.min_height}",
                suggested_action="check_camera_configuration",
                confidence=1.0,
                raw_data=data
            )
        
        # Check if image data exists
        if not data.data or len(data.data) == 0:
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.CRITICAL,
                anomaly_type="empty_image",
                description="Image contains no data",
                suggested_action="check_camera_connection",
                confidence=1.0,
                raw_data=data
            )
        
        # Check expected data size
        expected_size = data.width * data.height * (3 if 'rgb' in data.encoding.lower() else 1)
        if len(data.data) < expected_size * 0.8:  # Allow some tolerance
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.ERROR,
                anomaly_type="incomplete_image_data",
                description=f"Image data size {len(data.data)} much smaller than expected {expected_size}",
                suggested_action="check_image_transmission",
                confidence=0.9,
                raw_data=data
            )
        
        return None
    
    def _validate_timestamp(self, data: Image, current_time: float) -> Optional[ValidationResult]:
        """
        Validate timestamp consistency.
        
        Args:
            data: Image message
            current_time: Current system time
            
        Returns:
            ValidationResult if issues found, None otherwise
        """
        time_gap = current_time - self.previous_timestamp
        
        if time_gap > self.max_timestamp_gap:
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.WARNING,
                anomaly_type="timestamp_gap",
                description=f"Large gap between frames: {time_gap:.3f}s (max: {self.max_timestamp_gap}s)",
                suggested_action="check_camera_framerate",
                confidence=0.8,
                raw_data=data
            )
        
        return None
    
    def _decode_image(self, data: Image) -> Optional[np.ndarray]:
        """
        Decode ROS Image message to numpy array.
        
        Args:
            data: Image message
            
        Returns:
            Numpy array of image data, or None if decode fails
        """
        try:
            # Convert ROS Image to numpy array
            if data.encoding == 'rgb8':
                image_array = np.frombuffer(data.data, dtype=np.uint8)
                image_array = image_array.reshape((data.height, data.width, 3))
            elif data.encoding == 'bgr8':
                image_array = np.frombuffer(data.data, dtype=np.uint8)
                image_array = image_array.reshape((data.height, data.width, 3))
                # Convert BGR to RGB
                image_array = image_array[:, :, [2, 1, 0]]
            elif data.encoding == 'mono8':
                image_array = np.frombuffer(data.data, dtype=np.uint8)
                image_array = image_array.reshape((data.height, data.width))
            else:
                # Unsupported encoding, but try anyway
                image_array = np.frombuffer(data.data, dtype=np.uint8)
                if len(image_array) >= data.width * data.height:
                    image_array = image_array[:data.width * data.height]
                    image_array = image_array.reshape((data.height, data.width))
                else:
                    return None
            
            return image_array
            
        except Exception:
            return None
    
    def _validate_brightness(self, image_array: np.ndarray, data: Image) -> Optional[ValidationResult]:
        """
        Validate image brightness levels.
        
        Args:
            image_array: Decoded image as numpy array
            data: Original Image message
            
        Returns:
            ValidationResult if issues found, None otherwise
        """
        # Calculate average brightness
        if len(image_array.shape) == 3:  # Color image
            # Convert to grayscale for brightness calculation
            brightness = np.mean(np.dot(image_array[...,:3], [0.2989, 0.5870, 0.1140]))
        else:  # Grayscale image
            brightness = np.mean(image_array)
        
        self.brightness_history.append(brightness)
        
        # Check brightness levels
        if brightness < self.min_brightness:
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.WARNING,
                anomaly_type="image_too_dark",
                description=f"Image too dark: average brightness {brightness:.1f} < {self.min_brightness}",
                suggested_action="check_lighting_or_exposure",
                confidence=0.8,
                raw_data=data
            )
        elif brightness > self.max_brightness:
            return self._create_result(
                is_valid=False,
                severity=SeverityLevel.WARNING,
                anomaly_type="image_too_bright",
                description=f"Image too bright: average brightness {brightness:.1f} > {self.max_brightness}",
                suggested_action="check_exposure_settings",
                confidence=0.8,
                raw_data=data
            )
        
        return None
    
    def _detect_blur(self, image_array: np.ndarray, data: Image) -> Optional[ValidationResult]:
        """
        Detect blur using variance of Laplacian.
        
        Args:
            image_array: Decoded image as numpy array
            data: Original Image message
            
        Returns:
            ValidationResult if blur detected, None otherwise
        """
        try:
            # Convert to grayscale if needed
            if len(image_array.shape) == 3:
                gray = np.dot(image_array[...,:3], [0.2989, 0.5870, 0.1140])
            else:
                gray = image_array
            
            # Calculate Laplacian variance (blur metric)
            # Simple approximation of Laplacian using differences
            laplacian_var = np.var(np.diff(gray, axis=0)) + np.var(np.diff(gray, axis=1))
            
            self.blur_history.append(laplacian_var)
            
            if laplacian_var < self.blur_threshold:
                return self._create_result(
                    is_valid=False,
                    severity=SeverityLevel.WARNING,
                    anomaly_type="image_blurred",
                    description=f"Image appears blurred: blur metric {laplacian_var:.1f} < {self.blur_threshold}",
                    suggested_action="check_focus_or_vibration",
                    confidence=0.7,
                    raw_data=data
                )
                
        except Exception:
            # Blur detection failed, but don't fail validation
            pass
        
        return None
    
    def _detect_noise(self, image_array: np.ndarray, data: Image) -> Optional[ValidationResult]:
        """
        Detect excessive noise in the image.
        
        Args:
            image_array: Decoded image as numpy array
            data: Original Image message
            
        Returns:
            ValidationResult if noise detected, None otherwise
        """
        try:
            # Convert to grayscale if needed
            if len(image_array.shape) == 3:
                gray = np.dot(image_array[...,:3], [0.2989, 0.5870, 0.1140])
            else:
                gray = image_array
            
            # Simple noise detection using local standard deviation
            # Calculate standard deviation in small patches
            patch_size = 10
            noise_levels = []
            
            for i in range(0, gray.shape[0] - patch_size, patch_size):
                for j in range(0, gray.shape[1] - patch_size, patch_size):
                    patch = gray[i:i+patch_size, j:j+patch_size]
                    noise_levels.append(np.std(patch))
            
            if noise_levels:
                avg_noise = np.mean(noise_levels)
                max_noise = np.max(noise_levels)
                
                # Check if noise is excessive
                if avg_noise > self.noise_threshold * 255:  # Convert to 0-255 scale
                    return self._create_result(
                        is_valid=False,
                        severity=SeverityLevel.WARNING,
                        anomaly_type="excessive_noise",
                        description=f"Image has high noise level: {avg_noise:.1f} average, {max_noise:.1f} max",
                        suggested_action="check_camera_sensor_or_gain",
                        confidence=0.6,
                        raw_data=data
                    )
                    
        except Exception:
            # Noise detection failed, but don't fail validation
            pass
        
        return None
    
    def _detect_corruption(self, image_array: np.ndarray, data: Image) -> Optional[ValidationResult]:
        """
        Detect image corruption or completely blank frames.
        
        Args:
            image_array: Decoded image as numpy array
            data: Original Image message
            
        Returns:
            ValidationResult if corruption detected, None otherwise
        """
        try:
            # Check for completely black image
            if np.max(image_array) == 0:
                return self._create_result(
                    is_valid=False,
                    severity=SeverityLevel.ERROR,
                    anomaly_type="blank_image",
                    description="Image is completely black",
                    suggested_action="check_camera_lens_or_power",
                    confidence=0.9,
                    raw_data=data
                )
            
            # Check for completely white image
            max_val = 255 if np.max(image_array) <= 255 else np.max(image_array)
            if np.min(image_array) == max_val:
                return self._create_result(
                    is_valid=False,
                    severity=SeverityLevel.ERROR,
                    anomaly_type="saturated_image",
                    description="Image is completely saturated",
                    suggested_action="check_exposure_or_lighting",
                    confidence=0.9,
                    raw_data=data
                )
            
            # Check for very low variance (possibly corrupted/frozen frame)
            variance = np.var(image_array)
            if variance < 1.0:  # Very low variance
                return self._create_result(
                    is_valid=False,
                    severity=SeverityLevel.WARNING,
                    anomaly_type="low_variance_image",
                    description=f"Image has very low variance: {variance:.2f}, possibly corrupted or frozen",
                    suggested_action="check_camera_connection",
                    confidence=0.7,
                    raw_data=data
                )
                
        except Exception:
            # Corruption detection failed, but don't fail validation
            pass
        
        return None