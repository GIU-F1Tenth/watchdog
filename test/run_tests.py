#!/usr/bin/env python3

"""
Deprecated: Use standard test runners instead.

Run tests with:
  - pytest (python -m pytest)
  - or colcon test --packages-select watchdog
"""

import sys

def main():
    raise SystemExit(
        "test/run_tests.py is deprecated. Use 'pytest' or 'colcon test' instead."
    )

if __name__ == "__main__":
    main()