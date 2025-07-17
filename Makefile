.PHONY: help build clean spawn monitor reset

help:
	@echo "Available targets:"
	@echo "  help    - Show this help message"
	@echo "  build   - Build the project using colcon"
	@echo "  clean   - Remove build/, install/, and log/ directories"
	@echo "  spawn   - Spawn a vehicle with sensors in CARLA (port 3000)"
	@echo "  monitor - Monitor the spawned vehicle with visualization"
	@echo "  reset   - Clean up all actors and reload CARLA world"

build:
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

clean:
	rm -rf build/ install/ log/

spawn:
	. install/setup.sh && \
	ros2 launch carla_sensor_kit_utils vehicle_spawner.launch.py port:=3000

monitor:
	. install/setup.sh && \
	ros2 run carla_sensor_kit_utils vehicle_monitor --port 3000

reset:
	python3 scripts/cleanup_carla.py --port 3000
