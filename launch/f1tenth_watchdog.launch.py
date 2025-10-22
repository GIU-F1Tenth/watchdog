#!/usr/bin/env python3

"""
Deprecated duplicate launch file.

Please use the single canonical launch file:
  ros2 launch watchdog watchdog.launch.py
"""

from launch import LaunchDescription


def generate_launch_description():
    # Intentionally fail fast to guide users to the canonical launch file
    raise RuntimeError(
        "f1tenth_watchdog.launch.py is deprecated. Use 'watchdog.launch.py' instead."
    )