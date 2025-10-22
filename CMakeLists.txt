# This file is intentionally minimal.
#
# The watchdog package uses ament_python as its build type (see package.xml) and
# setup.py handles installation of the Python package, launch files, and config.
# A CMake-based build (ament_cmake) is NOT used here to avoid coupling with
# custom message generation. Custom interfaces live in giu_f1t_interfaces.

# Keeping this file empty ensures colcon does not try to treat this as an
# ament_cmake package while still allowing tooling that expects a CMakeLists.txt
# to coexist harmlessly. All installation is driven by setup.py.