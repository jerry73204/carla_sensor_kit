#!/usr/bin/env python3
"""
Allow running package generator as a module:
python -m carla_sensor_kit_utils.package_generator
"""

from .package_generator import main

if __name__ == "__main__":
    main()
