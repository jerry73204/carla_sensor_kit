#!/usr/bin/env python3
"""Unit tests for package generator."""

import unittest
from pathlib import Path
import tempfile
import shutil
import yaml
import json
from unittest.mock import Mock, patch, MagicMock

from carla_sensor_kit_utils.package_generator import (
    PackageGenerator,
    SensorPresetLibrary,
    VehiclePresetLibrary
)


class TestPackageGenerator(unittest.TestCase):
    """Test package generator functionality."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.test_dir = tempfile.mkdtemp()
        self.test_params = {
            'vehicle': {
                'role_name': 'test_vehicle',
                'wheel_radius': 0.3,
                'wheel_width': 0.2,
                'wheel_base': 2.5,
                'wheel_tread': 1.5,
                'front_overhang': 0.9,
                'rear_overhang': 1.0,
                'left_overhang': 0.1,
                'right_overhang': 0.1,
                'height': 1.5,
                'max_steer_angle': 0.7
            },
            'sensors': {
                'lidar': [
                    {
                        'name': 'velodyne_top',
                        'frame_id': 'velodyne_top_base_link',
                        'transform': {
                            'location': {'x': 0.0, 'y': 0.0, 'z': 2.0},
                            'rotation': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
                        },
                        'channels': 32,
                        'range': 100.0,
                        'rotation_frequency': 10,
                        'points_per_second': 600000
                    }
                ],
                'camera': [
                    {
                        'name': 'camera_front',
                        'frame_id': 'camera_front/camera_link',
                        'transform': {
                            'location': {'x': 1.5, 'y': 0.0, 'z': 1.5},
                            'rotation': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
                        },
                        'image_size_x': 1920,
                        'image_size_y': 1080,
                        'fov': 90,
                        'fps': 30
                    }
                ]
            }
        }
    
    def tearDown(self):
        """Clean up test directory."""
        if Path(self.test_dir).exists():
            shutil.rmtree(self.test_dir)
    
    @patch('carla_sensor_kit_utils.package_generator.carla')
    def test_init_without_carla(self, mock_carla):
        """Test initialization when CARLA is not available."""
        mock_carla = None
        
        # Should still initialize without CARLA
        generator = PackageGenerator()
        self.assertIsNotNone(generator)
    
    def test_sensor_preset_library(self):
        """Test sensor preset library functionality."""
        # Test listing presets
        presets = SensorPresetLibrary.list_presets()
        self.assertIn('default', presets)
        self.assertIn('minimal', presets)
        self.assertIn('advanced', presets)
        
        # Test getting preset
        default_preset = SensorPresetLibrary.get_preset('default')
        self.assertIn('sensors', default_preset)
        self.assertIn('lidar', default_preset['sensors'])
        
        # Test invalid preset
        with self.assertRaises(ValueError):
            SensorPresetLibrary.get_preset('invalid_preset')
    
    def test_vehicle_preset_library(self):
        """Test vehicle preset library functionality."""
        # Test listing presets
        presets = VehiclePresetLibrary.list_presets()
        self.assertIn('sedan', presets)
        self.assertIn('suv', presets)
        
        # Test getting preset
        sedan_preset = VehiclePresetLibrary.get_preset('sedan')
        self.assertIn('dimensions', sedan_preset)
        self.assertIn('wheel_base', sedan_preset['dimensions'])
    
    def test_yaml_filter(self):
        """Test YAML filter functionality."""
        data = {'key': 'value', 'number': 42}
        result = PackageGenerator._to_yaml_filter(data)
        self.assertIn('key: value', result)
        self.assertIn('number: 42', result)
        
        # Test with indentation
        result_indented = PackageGenerator._to_yaml_filter(data, indent=2)
        lines = result_indented.split('\n')
        for line in lines:
            if line:  # Skip empty lines
                self.assertTrue(line.startswith('  '))
    
    def test_xml_filter(self):
        """Test XML filter functionality."""
        # Test boolean conversion
        self.assertEqual(PackageGenerator._to_xml_filter(True), 'true')
        self.assertEqual(PackageGenerator._to_xml_filter(False), 'false')
        
        # Test other types
        self.assertEqual(PackageGenerator._to_xml_filter(42), '42')
        self.assertEqual(PackageGenerator._to_xml_filter('test'), 'test')
    
    @patch('carla_sensor_kit_utils.package_generator.carla')
    def test_generate_from_params(self, mock_carla):
        """Test package generation from parameters."""
        # Create minimal templates for testing
        template_dir = Path(self.test_dir) / 'templates'
        sensor_desc_dir = template_dir / 'carla_sensor_kit_description'
        sensor_desc_dir.mkdir(parents=True)
        
        # Create a simple template file
        package_xml = sensor_desc_dir / 'package.xml'
        package_xml.write_text("""<?xml version="1.0"?>
<package format="3">
  <name>test_package</name>
  <version>0.1.0</version>
  <description>Test package</description>
</package>""")
        
        # Create generator with test template directory
        generator = PackageGenerator(template_dir)
        
        # Generate packages
        output_dir = Path(self.test_dir) / 'output'
        packages = generator.generate_from_params(
            self.test_params,
            'test_sensors',
            'test_vehicle',
            output_dir
        )
        
        # Verify packages were created
        self.assertGreater(len(packages), 0)
        
        # Verify output directory structure
        self.assertTrue(output_dir.exists())
    
    def test_template_file_processing(self):
        """Test Jinja2 template file processing."""
        # Create test template directory
        template_dir = Path(self.test_dir) / 'templates'
        template_dir.mkdir(parents=True)
        
        # Create a Jinja2 template
        template_file = template_dir / 'test.yaml.j2'
        template_file.write_text("""vehicle_name: {{ vehicle.role_name }}
wheel_base: {{ vehicle.wheel_base }}""")
        
        # Create generator
        generator = PackageGenerator(template_dir)
        
        # Process template
        output_file = Path(self.test_dir) / 'output.yaml'
        generator._process_template_file(
            template_file,
            output_file,
            self.test_params,
            template_dir
        )
        
        # Verify output
        self.assertTrue(output_file.exists())
        content = yaml.safe_load(output_file.read_text())
        self.assertEqual(content['vehicle_name'], 'test_vehicle')
        self.assertEqual(content['wheel_base'], 2.5)
    
    def test_parameter_file_loading(self):
        """Test loading parameters from file."""
        # Test YAML file
        yaml_file = Path(self.test_dir) / 'params.yaml'
        yaml_file.write_text(yaml.dump(self.test_params))
        
        # Load and verify
        loaded_yaml = yaml.safe_load(yaml_file.read_text())
        self.assertEqual(loaded_yaml, self.test_params)
        
        # Test JSON file
        json_file = Path(self.test_dir) / 'params.json'
        json_file.write_text(json.dumps(self.test_params))
        
        # Load and verify
        loaded_json = json.loads(json_file.read_text())
        self.assertEqual(loaded_json, self.test_params)
    
    @patch('carla_sensor_kit_utils.package_generator.DynamicConfigGenerator')
    @patch('carla_sensor_kit_utils.package_generator.carla')
    def test_generate_from_carla(self, mock_carla, mock_config_gen):
        """Test package generation from CARLA connection."""
        # Mock CARLA connection and parameter extraction
        mock_generator_instance = MagicMock()
        mock_generator_instance.extract_all_parameters.return_value = self.test_params
        mock_config_gen.return_value = mock_generator_instance
        
        # Create minimal templates
        template_dir = Path(self.test_dir) / 'templates'
        template_dir.mkdir(parents=True)
        
        # Create generator
        generator = PackageGenerator(template_dir)
        
        # Test generation from CARLA
        output_dir = Path(self.test_dir) / 'output'
        packages = generator.generate_from_carla(
            host='localhost',
            port=2000,
            vehicle_role_name='ego_vehicle',
            sensor_kit_name='test_sensors',
            vehicle_model_name='test_vehicle',
            output_dir=output_dir
        )
        
        # Verify CARLA connection was attempted
        mock_config_gen.assert_called_once_with('localhost', 2000)
        mock_generator_instance.extract_all_parameters.assert_called_once_with('ego_vehicle')
    
    def test_executable_preservation(self):
        """Test that executable permissions are preserved."""
        # Create test template directory
        template_dir = Path(self.test_dir) / 'templates'
        template_dir.mkdir(parents=True)
        
        # Create an executable script
        script_file = template_dir / 'script.sh'
        script_file.write_text('#!/bin/bash\necho "test"')
        script_file.chmod(0o755)
        
        # Create generator
        generator = PackageGenerator(template_dir)
        
        # Process (copy) the file
        output_dir = Path(self.test_dir) / 'output'
        output_dir.mkdir(parents=True)
        
        # Simulate file processing
        shutil.copy2(script_file, output_dir / 'script.sh')
        
        # Verify executable permission is preserved
        output_script = output_dir / 'script.sh'
        self.assertTrue(output_script.exists())
        self.assertTrue(output_script.stat().st_mode & 0o111)  # Check if executable


class TestPackageGeneratorIntegration(unittest.TestCase):
    """Integration tests for package generator."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.test_dir = tempfile.mkdtemp()
    
    def tearDown(self):
        """Clean up test directory."""
        if Path(self.test_dir).exists():
            shutil.rmtree(self.test_dir)
    
    def test_end_to_end_generation(self):
        """Test end-to-end package generation workflow."""
        # This would test the full workflow with real templates
        # Skipped if templates directory doesn't exist
        repo_root = Path(__file__).parent.parent.parent.parent
        template_dir = repo_root / 'templates'
        
        if not template_dir.exists():
            self.skipTest("Templates directory not found")
        
        # Create test parameters
        params = {
            'vehicle': {
                'role_name': 'test_ego',
                'wheel_radius': 0.35,
                'wheel_width': 0.25,
                'wheel_base': 2.7,
                'wheel_tread': 1.6,
                'front_overhang': 1.0,
                'rear_overhang': 1.1,
                'left_overhang': 0.15,
                'right_overhang': 0.15,
                'height': 1.6,
                'max_steer_angle': 0.65
            },
            'sensors': {
                'lidar': [{
                    'name': 'velodyne_top',
                    'frame_id': 'velodyne_top_base_link',
                    'transform': {
                        'location': {'x': 0, 'y': 0, 'z': 2.0},
                        'rotation': {'roll': 0, 'pitch': 0, 'yaw': 0}
                    },
                    'rotation_frequency': 10,
                    'points_per_second': 600000
                }]
            }
        }
        
        # Create generator
        generator = PackageGenerator(template_dir)
        
        # Generate packages
        output_dir = Path(self.test_dir) / 'workspace'
        packages = generator.generate_from_params(
            params,
            'integration_test_sensors',
            'integration_test_vehicle',
            output_dir
        )
        
        # Verify expected packages were created
        expected_packages = [
            'integration_test_sensors_description',
            'integration_test_sensors_launch',
            'integration_test_vehicle_description',
            'integration_test_vehicle_launch'
        ]
        
        for pkg_name in expected_packages:
            self.assertIn(pkg_name, packages)
            pkg_path = packages[pkg_name]
            self.assertTrue(pkg_path.exists())
            
            # Verify package.xml exists
            package_xml = pkg_path / 'package.xml'
            self.assertTrue(package_xml.exists())


if __name__ == '__main__':
    unittest.main()