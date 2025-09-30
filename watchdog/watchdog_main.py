#!/usr/bin/env python3

"""
F1TENTH Watchdog Node Main Entry Point

This is the main entry point for the F1TENTH watchdog node, designed to work
properly when installed as a ROS2 package.

Author: F1TENTH Watchdog Team
License: MIT
Version: 1.0.0
"""

import sys
import os

# Ensure proper import paths
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

# Import and run the main function
try:
    from watchdog.watchdog_node import main
except ImportError:
    # Fallback for development environment
    from watchdog_node import main

if __name__ == '__main__':
    main()