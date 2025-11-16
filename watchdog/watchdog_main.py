#!/usr/bin/env python3

"""
Deprecated: Use the main node entry point instead.

Run the node with:
  ros2 run watchdog watchdog_node
or via the launch file.
"""

def main():
    raise RuntimeError(
        "watchdog/watchdog_main.py is deprecated. Use 'watchdog.watchdog_node:main'."
    )

if __name__ == "__main__":
    main()