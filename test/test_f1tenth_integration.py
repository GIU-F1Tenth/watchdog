"""
Deprecated F1TENTH-specific integration tests.

This module previously validated F1TENTH launch/config and FSM helpers that are
no longer part of this package. The watchdog is now independent with a single
canonical launch (launch/watchdog.launch.py) and config (config/watchdog_params.yaml).

These F1TENTH integration checks are intentionally skipped to avoid confusion.
"""

import pytest

pytest.skip(
    "F1TENTH-specific integration tests are deprecated. Use launch/watchdog.launch.py "
    "with config/watchdog_params.yaml; FSM integration and f1tenth_* assets were removed.",
    allow_module_level=True,
)