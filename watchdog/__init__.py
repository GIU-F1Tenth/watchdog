#!/usr/bin/env python3

"""
F1TENTH Watchdog Package

A comprehensive system monitoring and safety watchdog for F1TENTH autonomous racing vehicles.

Author: Fam Shihata <fam@awadlouis.com>
Author: Mohammed Azab <mo7ammed3zab@outlook.com>
License: MIT
Version: 1.0.0
"""

__version__ = "1.0.0"
__author__ = "Fam Shihata, Mohammed Azab"
__email__ = "fam@awadlouis.com, mo7ammed3zab@outlook.com"
__license__ = "MIT"

# Import main components for easier access
from .watchdog_node import WatchdogNode

__all__ = ['WatchdogNode']
