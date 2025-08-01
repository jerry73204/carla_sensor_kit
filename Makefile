SHELL := /bin/bash
.PHONY: help build-all build-utils build-generated clean clean-utils clean-generated spawn gui reset generate launch monitor-topics list-topics

# Configuration
CARLA_PORT ?= 3000
VEHICLE_ROLE ?= ego_vehicle
SENSOR_KIT_NAME ?= carla_sensor_kit
VEHICLE_MODEL_NAME ?= carla_vehicle
WORKSPACE_DIR ?= workspace

help:
	@echo "CARLA-Autoware Integration Tool"
	@echo "==============================="
	@echo ""
	@echo "QUICK START:"
	@echo "  1. Start CARLA server (in another terminal)"
	@echo "  2. make build-all           # Complete build: utils, spawn vehicle, generate & build packages"
	@echo "  3. make launch              # Launch Autoware with generated packages"
	@echo ""
	@echo "BUILD TARGETS:"
	@echo "  build-all         - Complete workflow: build utils, spawn, generate, and build packages"
	@echo "  build-utils       - Build carla_sensor_kit_utils only"
	@echo "  build-generated   - Build generated ROS2 packages only (requires prior generation)"
	@echo "  clean             - Remove all build artifacts"
	@echo "  clean-utils       - Remove utils build artifacts only"
	@echo "  clean-generated   - Remove generated packages only"
	@echo ""
	@echo "CARLA CONTROL:"
	@echo "  spawn             - Spawn a vehicle with sensors in CARLA"
	@echo "  gui               - Monitor the spawned vehicle with visualization"
	@echo "  reset             - Clean up all actors and reload CARLA world"
	@echo ""
	@echo "PACKAGE GENERATION:"
	@echo "  generate          - Generate ROS2 packages from CARLA vehicle (requires spawned vehicle)"
	@echo ""
	@echo "AUTOWARE INTEGRATION:"
	@echo "  launch            - Launch Autoware with generated packages"
	@echo "  monitor-topics    - Monitor ROS2 topic rates (live)"
	@echo "  list-topics       - List all available ROS2 topics"
	@echo ""
	@echo "ENVIRONMENT VARIABLES:"
	@echo "  CARLA_PORT         = $(CARLA_PORT) (CARLA server port)"
	@echo "  VEHICLE_ROLE       = $(VEHICLE_ROLE) (Vehicle role name)"
	@echo "  SENSOR_KIT_NAME    = $(SENSOR_KIT_NAME) (Sensor kit package name)"
	@echo "  VEHICLE_MODEL_NAME = $(VEHICLE_MODEL_NAME) (Vehicle model package name)"
	@echo "  WORKSPACE_DIR      = $(WORKSPACE_DIR) (Generated packages directory)"
	@echo "  AUTOWARE_PATH      = $(if $(AUTOWARE_PATH),$(AUTOWARE_PATH),[NOT SET - Required for launch/monitor])"
	@echo ""
	@echo "PREREQUISITES:"
	@echo "  - CARLA server running (default port: 3000)"
	@echo "  - ROS2 Humble installed"
	@echo "  - Autoware installed (set AUTOWARE_PATH for launch/monitor targets)"

build-utils:
	colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

build-generated:
	. /opt/ros/humble/setup.sh && \
	cd workspace && \
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

build-all: build-utils reset spawn generate build-generated
	@echo ""
	@echo "Complete build finished! You can now run:"
	@echo "  make launch      # Launch Autoware with the generated packages"

clean-utils:
	rm -rf build/ install/ log/

clean-generated:
	rm -rf $(WORKSPACE_DIR)

clean: clean-utils clean-generated

spawn:
	@if [ ! -f install/setup.bash ]; then echo "Error: Run 'make build' first"; exit 1; fi
	. install/setup.bash && \
	spawn_vehicle --port $(CARLA_PORT) --config src/carla_sensor_kit_utils/config/spawn_vehicle_config.yaml

gui:
	@if [ ! -f install/setup.bash ]; then echo "Error: Run 'make build' first"; exit 1; fi
	. install/setup.bash && \
	vehicle_monitor --port $(CARLA_PORT) --role-name $(VEHICLE_ROLE)

reset:
	@if [ ! -f install/setup.bash ]; then echo "Error: Run 'make build' first"; exit 1; fi
	. install/setup.bash && \
	reset_carla --port $(CARLA_PORT)

generate:
	@if [ ! -f install/setup.bash ]; then echo "Error: Run 'make build' first"; exit 1; fi
	@echo "Generating packages from CARLA vehicle '$(VEHICLE_ROLE)'..."
	. install/setup.bash && \
	carla_package_generator \
		--port $(CARLA_PORT) \
		--vehicle $(VEHICLE_ROLE) \
		--sensor-kit $(SENSOR_KIT_NAME) \
		--vehicle-model $(VEHICLE_MODEL_NAME) \
		--output $(WORKSPACE_DIR) \
		--clean
	@echo ""
	@echo "Packages generated in $(WORKSPACE_DIR)/"
	@echo "To build the generated packages:"
	@echo "  cd $(WORKSPACE_DIR) && colcon build"

launch:
	@if [ ! -f $(WORKSPACE_DIR)/install/setup.bash ]; then \
		echo "Error: Generated packages not built. Run the following first:"; \
		echo "  1. make generate"; \
		echo "  2. cd $(WORKSPACE_DIR) && colcon build"; \
		exit 1; \
	fi
	@if [ -z "$$AUTOWARE_PATH" ]; then \
		echo "Error: AUTOWARE_PATH environment variable not set"; \
		echo ""; \
		echo "Please set it to your Autoware installation directory:"; \
		echo "  export AUTOWARE_PATH=/path/to/autoware"; \
		echo ""; \
		echo "Or if you have Autoware installed in the default location:"; \
		echo "  export AUTOWARE_PATH=$$HOME/autoware"; \
		exit 1; \
	fi
	@echo "Launching Autoware with generated packages..."
	@echo "  Vehicle model: $(VEHICLE_MODEL_NAME)"
	@echo "  Sensor model: $(SENSOR_KIT_NAME)"
	. $$AUTOWARE_PATH/install/setup.bash && \
	. $(WORKSPACE_DIR)/install/setup.bash && \
	ros2 launch autoware_launch planning_simulator.launch.xml \
		map_path:=$$HOME/autoware_map/sample-map-planning \
		vehicle_model:=$(VEHICLE_MODEL_NAME) \
		sensor_model:=$(SENSOR_KIT_NAME)


monitor-topics:
	@if [ -z "$$AUTOWARE_PATH" ]; then \
		echo "Error: AUTOWARE_PATH environment variable not set"; \
		exit 1; \
	fi
	@echo "Monitoring ROS2 topics from Autoware..."
	@echo "This will show topic rates for key sensor and control topics"
	@echo "Press Ctrl+C to stop"
	@echo ""
	. $$AUTOWARE_PATH/install/setup.bash && \
	. $(WORKSPACE_DIR)/install/setup.bash 2>/dev/null || true && \
	parallel --will-cite --line-buffer --tag ::: \
		"ros2 topic hz /sensing/lidar/top/pointcloud_raw" \
		"ros2 topic hz /sensing/camera/camera_front/image_raw" \
		"ros2 topic hz /sensing/camera/camera_rear/image_raw" \
		"ros2 topic hz /sensing/imu/imu_data" \
		"ros2 topic hz /sensing/gnss/gnss_data" \
		"ros2 topic hz /vehicle/status/velocity_status" \
		"ros2 topic hz /vehicle/status/steering_status" \
		"ros2 topic hz /control/command/control_cmd" \
		"ros2 topic hz /planning/scenario_planning/trajectory" \
		"ros2 topic hz /localization/kinematic_state"

list-topics:
	@if [ -z "$$AUTOWARE_PATH" ]; then \
		echo "Error: AUTOWARE_PATH environment variable not set"; \
		exit 1; \
	fi
	@echo "Listing all ROS2 topics..."
	. $$AUTOWARE_PATH/install/setup.bash && \
	. $(WORKSPACE_DIR)/install/setup.bash 2>/dev/null || true && \
	ros2 topic list | sort
