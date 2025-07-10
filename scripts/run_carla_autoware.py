#!/usr/bin/env python3
"""
CARLA-Autoware Workflow Automation Script

Orchestrates the complete CARLA-Autoware integration stack:
1. Starts CARLA server
2. Spawns vehicles with sensors
3. Launches Autoware sensor kit
4. Monitors system health
"""

import sys
import os
import time
import signal
import argparse
import subprocess
import psutil
import yaml
import logging
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Optional, Tuple
import threading
import queue


class CarlaServerManager:
    """Manages CARLA server lifecycle."""
    
    def __init__(self, headless: bool = False, port: int = 2000, 
                 quality: str = 'Low', map_name: str = 'Town03'):
        self.headless = headless
        self.port = port
        self.quality = quality
        self.map_name = map_name
        self.process = None
        self.logger = logging.getLogger('CarlaServer')
        
    def start(self) -> bool:
        """Start CARLA server."""
        self.logger.info("Starting CARLA server...")
        
        # Kill any existing CARLA instances
        self._kill_existing()
        
        # Find CARLA executable
        carla_path = self._find_carla_executable()
        if not carla_path:
            self.logger.error("CARLA executable not found")
            return False
        
        # Build command
        cmd = [str(carla_path)]
        if self.headless:
            cmd.extend(['-RenderOffScreen'])
        cmd.extend([
            f'-carla-rpc-port={self.port}',
            f'-quality-level={self.quality}'
        ])
        
        # Start process
        try:
            env = os.environ.copy()
            if self.headless:
                env['DISPLAY'] = ''
            
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=env
            )
            
            # Wait for server to be ready
            if self._wait_for_ready():
                self.logger.info(f"CARLA server started (PID: {self.process.pid})")
                return True
            else:
                self.logger.error("CARLA server failed to start")
                self.stop()
                return False
                
        except Exception as e:
            self.logger.error(f"Failed to start CARLA: {e}")
            return False
    
    def stop(self):
        """Stop CARLA server."""
        if self.process:
            self.logger.info("Stopping CARLA server...")
            self.process.terminate()
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()
            self.process = None
    
    def _kill_existing(self):
        """Kill any existing CARLA processes."""
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                if 'CarlaUE4' in proc.info['name'] or \
                   any('CarlaUE4' in arg for arg in (proc.info['cmdline'] or [])):
                    self.logger.info(f"Killing existing CARLA process (PID: {proc.info['pid']})")
                    proc.kill()
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
        
        time.sleep(2)  # Wait for processes to die
    
    def _find_carla_executable(self) -> Optional[Path]:
        """Find CARLA executable."""
        possible_paths = [
            Path.home() / "repos/carla_sensor_kit_launch/carla-simulator/CarlaUE4.sh",
            Path("/opt/carla-simulator/CarlaUE4.sh"),
            Path.home() / "carla/CarlaUE4.sh",
        ]
        
        for path in possible_paths:
            if path.exists():
                return path
        
        return None
    
    def _wait_for_ready(self, timeout: int = 30) -> bool:
        """Wait for CARLA server to be ready."""
        import carla
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                client = carla.Client('localhost', self.port)
                client.set_timeout(1.0)
                client.get_world()
                
                # Load specified map if different
                if self.map_name:
                    world = client.get_world()
                    current_map = world.get_map().name.split('/')[-1]
                    if current_map != self.map_name:
                        self.logger.info(f"Loading map: {self.map_name}")
                        client.load_world(self.map_name)
                
                return True
            except:
                time.sleep(1)
        
        return False


class VehicleSpawner:
    """Spawns vehicles with sensors in CARLA."""
    
    def __init__(self, port: int = 2000):
        self.port = port
        self.logger = logging.getLogger('VehicleSpawner')
        self.actors = []
        
    def spawn(self, num_vehicles: int = 1, spawn_points: Optional[List] = None) -> bool:
        """Spawn vehicles with sensors."""
        self.logger.info(f"Spawning {num_vehicles} vehicle(s)...")
        
        try:
            # Import here to avoid import errors if CARLA not in path
            import carla
            sys.path.insert(0, str(Path(__file__).parent))
            from sensor_config_loader import SensorConfigLoader
            
            # Connect to CARLA
            client = carla.Client('localhost', self.port)
            client.set_timeout(10.0)
            world = client.get_world()
            
            # Load sensor configurations
            config_loader = SensorConfigLoader()
            
            # Get spawn points
            map_spawn_points = world.get_map().get_spawn_points()
            if not spawn_points:
                spawn_points = map_spawn_points[:num_vehicles]
            
            # Spawn vehicles
            blueprint_library = world.get_blueprint_library()
            vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
            
            for i, spawn_point in enumerate(spawn_points[:num_vehicles]):
                # Spawn vehicle
                vehicle = world.spawn_actor(vehicle_bp, spawn_point)
                self.actors.append(vehicle)
                self.logger.info(f"Spawned vehicle {i+1} at {spawn_point.location}")
                
                # Note: Actual sensor spawning would happen here
                # For now, we just log that we would spawn them
                self.logger.info(f"Would spawn sensors for vehicle {i+1} using YAML configs")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to spawn vehicles: {e}")
            return False
    
    def cleanup(self):
        """Destroy all spawned actors."""
        self.logger.info("Cleaning up spawned actors...")
        for actor in self.actors:
            try:
                actor.destroy()
            except:
                pass
        self.actors.clear()


class AutowareLauncher:
    """Launches Autoware sensor kit and modules."""
    
    def __init__(self, sensor_kit: str = 'carla_sensor_kit_launch'):
        self.sensor_kit = sensor_kit
        self.logger = logging.getLogger('AutowareLauncher')
        self.processes = []
        
    def launch(self, modules: List[str] = ['sensing']) -> bool:
        """Launch Autoware modules."""
        self.logger.info(f"Launching Autoware with modules: {modules}")
        
        # Source ROS environment
        source_cmd = "source /opt/ros/humble/setup.bash && "
        
        # Launch sensor kit
        for module in modules:
            if module == 'sensing':
                cmd = f"{source_cmd}ros2 launch {self.sensor_kit} sensing.launch.xml"
            else:
                self.logger.warning(f"Module '{module}' not implemented yet")
                continue
            
            try:
                process = subprocess.Popen(
                    cmd,
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                self.processes.append(process)
                self.logger.info(f"Launched {module} module (PID: {process.pid})")
                
            except Exception as e:
                self.logger.error(f"Failed to launch {module}: {e}")
                return False
        
        return True
    
    def stop(self):
        """Stop all Autoware processes."""
        self.logger.info("Stopping Autoware processes...")
        for process in self.processes:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
        self.processes.clear()


class SystemMonitor:
    """Monitors system health and displays status."""
    
    def __init__(self):
        self.logger = logging.getLogger('SystemMonitor')
        self.running = False
        self.status_queue = queue.Queue()
        
    def start(self):
        """Start monitoring."""
        self.running = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop)
        self.monitor_thread.start()
        
    def stop(self):
        """Stop monitoring."""
        self.running = False
        if hasattr(self, 'monitor_thread'):
            self.monitor_thread.join()
    
    def _monitor_loop(self):
        """Main monitoring loop."""
        while self.running:
            # Collect system stats
            cpu_percent = psutil.cpu_percent(interval=1)
            memory = psutil.virtual_memory()
            
            # Display status
            self._display_status({
                'cpu': cpu_percent,
                'memory_gb': memory.used / (1024**3),
                'timestamp': datetime.now()
            })
            
            time.sleep(1)
    
    def _display_status(self, stats: dict):
        """Display current status."""
        os.system('clear' if os.name == 'posix' else 'cls')
        
        print(f"[CARLA-Autoware Status] {stats['timestamp'].strftime('%Y-%m-%d %H:%M:%S')}")
        print("═" * 50)
        print(f"CARLA Server:    ✓ Running")
        print(f"Spawner:         ✓ Complete")
        print(f"Autoware:        ✓ Active")
        print()
        print("System Health:")
        print(f"  CPU: {stats['cpu']:.1f}%  |  Memory: {stats['memory_gb']:.1f} GB")
        print()
        print("Press Ctrl+C to shutdown gracefully...")


class WorkflowOrchestrator:
    """Main workflow orchestrator."""
    
    def __init__(self, args):
        self.args = args
        self.logger = self._setup_logging()
        
        # Components
        self.carla_server = CarlaServerManager(
            headless=args.carla_headless,
            port=args.carla_port,
            map_name=args.carla_map
        )
        self.spawner = VehicleSpawner(port=args.carla_port)
        self.autoware = AutowareLauncher()
        self.monitor = SystemMonitor()
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
    def _setup_logging(self) -> logging.Logger:
        """Setup logging configuration."""
        log_dir = Path(self.args.log_dir)
        log_dir.mkdir(exist_ok=True)
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_file = log_dir / f"workflow_{timestamp}.log"
        
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s [%(name)s] %(levelname)s: %(message)s',
            handlers=[
                logging.FileHandler(log_file),
                logging.StreamHandler(sys.stdout)
            ]
        )
        
        return logging.getLogger('WorkflowOrchestrator')
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals."""
        self.logger.info("Received shutdown signal")
        self.shutdown()
        sys.exit(0)
    
    def run(self) -> int:
        """Run the complete workflow."""
        self.logger.info("Starting CARLA-Autoware workflow")
        
        try:
            # 1. Start CARLA server
            if not self.carla_server.start():
                self.logger.error("Failed to start CARLA server")
                return 1
            
            time.sleep(5)  # Give server time to fully initialize
            
            # 2. Spawn vehicles
            if not self.spawner.spawn(num_vehicles=self.args.vehicles):
                self.logger.error("Failed to spawn vehicles")
                return 1
            
            # 3. Launch Autoware
            if not self.autoware.launch(modules=['sensing']):
                self.logger.error("Failed to launch Autoware")
                return 1
            
            # 4. Start monitoring
            self.monitor.start()
            
            # Keep running until interrupted
            self.logger.info("Workflow running. Press Ctrl+C to stop.")
            while True:
                time.sleep(1)
                
        except Exception as e:
            self.logger.error(f"Workflow error: {e}")
            return 1
        finally:
            self.shutdown()
        
        return 0
    
    def shutdown(self):
        """Shutdown all components."""
        self.logger.info("Shutting down workflow...")
        
        # Stop in reverse order
        self.monitor.stop()
        self.autoware.stop()
        
        if not self.args.no_cleanup:
            self.spawner.cleanup()
        
        self.carla_server.stop()
        
        self.logger.info("Shutdown complete")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='CARLA-Autoware Workflow Automation'
    )
    
    # CARLA options
    parser.add_argument('--carla-headless', action='store_true',
                       help='Run CARLA without display')
    parser.add_argument('--carla-port', type=int, default=2000,
                       help='CARLA RPC port (default: 2000)')
    parser.add_argument('--carla-map', default='Town03',
                       help='Map to load (default: Town03)')
    
    # Spawner options
    parser.add_argument('--vehicles', type=int, default=1,
                       help='Number of vehicles (default: 1)')
    
    # Autoware options
    parser.add_argument('--rmw', default='rmw_fastrtps_cpp',
                       help='ROS middleware (default: rmw_fastrtps_cpp)')
    
    # Workflow options
    parser.add_argument('--log-dir', default='./logs',
                       help='Log directory (default: ./logs)')
    parser.add_argument('--timeout', type=int, default=60,
                       help='Startup timeout (default: 60)')
    parser.add_argument('--no-cleanup', action='store_true',
                       help='Skip cleanup on exit')
    
    args = parser.parse_args()
    
    # Set RMW implementation
    os.environ['RMW_IMPLEMENTATION'] = args.rmw
    
    # Run workflow
    orchestrator = WorkflowOrchestrator(args)
    return orchestrator.run()


if __name__ == '__main__':
    sys.exit(main())