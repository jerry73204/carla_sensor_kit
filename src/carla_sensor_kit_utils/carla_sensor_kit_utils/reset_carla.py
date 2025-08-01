#!/usr/bin/env python3
"""
Script to reload CARLA world and clean up all actors.
This is useful when CARLA gets into a bad state with leftover actors.
"""

import carla
import argparse
import sys


def cleanup_carla(host="localhost", port=2000):
    """Connect to CARLA and reload the world to clean up all actors."""
    try:
        # Connect to CARLA
        client = carla.Client(host, port)
        client.set_timeout(10.0)

        print(f"Connecting to CARLA at {host}:{port}...")

        # Get current world
        world = client.get_world()
        current_map = world.get_map().name
        print(f"Current map: {current_map}")

        # Get all actors
        actors = world.get_actors()
        vehicles = actors.filter("vehicle.*")
        sensors = actors.filter("sensor.*")
        walkers = actors.filter("walker.*")

        print(f"\nCurrent actors:")
        print(f"  Vehicles: {len(vehicles)}")
        print(f"  Sensors: {len(sensors)}")
        print(f"  Walkers: {len(walkers)}")

        # Option 1: Destroy all actors individually
        print("\nDestroying all actors...")
        destroyed_count = 0

        # Destroy sensors first (they might be attached to vehicles)
        for sensor in sensors:
            try:
                sensor.destroy()
                destroyed_count += 1
            except:
                pass

        # Then destroy vehicles
        for vehicle in vehicles:
            try:
                vehicle.destroy()
                destroyed_count += 1
            except:
                pass

        # Finally destroy walkers
        for walker in walkers:
            try:
                walker.destroy()
                destroyed_count += 1
            except:
                pass

        print(f"Destroyed {destroyed_count} actors")

        # Option 2: Reload the world (more thorough cleanup)
        print(f"\nReloading world '{current_map}'...")
        world = client.reload_world()

        # Verify cleanup
        actors = world.get_actors()
        vehicles = actors.filter("vehicle.*")
        sensors = actors.filter("sensor.*")
        walkers = actors.filter("walker.*")

        print(f"\nAfter cleanup:")
        print(f"  Vehicles: {len(vehicles)}")
        print(f"  Sensors: {len(sensors)}")
        print(f"  Walkers: {len(walkers)}")

        # Reset settings to default
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        print("\nWorld reloaded and cleaned successfully!")
        print("Settings reset to asynchronous mode")

    except Exception as e:
        print(f"Error: {e}")
        return 1

    return 0


def main():
    parser = argparse.ArgumentParser(description="Cleanup CARLA world and actors")
    parser.add_argument(
        "--host",
        type=str,
        default="localhost",
        help="CARLA server host (default: localhost)",
    )
    parser.add_argument(
        "--port", type=int, default=2000, help="CARLA server port (default: 2000)"
    )

    args = parser.parse_args()

    return cleanup_carla(args.host, args.port)


if __name__ == "__main__":
    sys.exit(main())
