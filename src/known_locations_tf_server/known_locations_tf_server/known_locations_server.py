#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import yaml
import math
from pathlib import Path
from typing import Dict, Optional, Tuple

# TF2 imports
import tf2_ros
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

# Service import

from robotino_interfaces.srv import LocationsServer


class KnownLocationsServer(Node):
    def __init__(self):
        super().__init__('known_locations_server')
        
        # Use reentrant callback group for concurrent service calls
        self.callback_group = ReentrantCallbackGroup()
        
        # Declare parameters with defaults
        self.declare_parameter('locations_file', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        
        # Get parameters
        self.locations_file_param = self.get_parameter('locations_file').value
        self.frame_id = self.get_parameter('frame_id').value
        self.robot_frame = self.get_parameter('robot_frame').value
        
        # Setup TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # Find and load known locations
        self.locations_file_path = self.find_locations_file()
        self.known_locations = self.load_locations_from_yaml()
        
        # Broadcast static TFs from YAML
        self.broadcast_known_locations_tfs()
        
        # Create service
        self.service = self.create_service(
            LocationsServer,
            '/known_location_add',
            self.handle_add_location,
            callback_group=self.callback_group
        )
        
        self.get_logger().info(f'Known Locations Server started')
        self.get_logger().info(f'Loaded {len(self.known_locations)} locations from: {self.locations_file_path}')
        self.get_logger().info('Service /known_location_add available')
    
    def find_locations_file(self) -> Path:
        """
        Resolve a portable, user-editable locations yaml path.

        Priority:
        1) locations_file param (explicit override)
        2) ~/.robotino/known_locations.yaml (auto-created + seeded from robotino_bts default)

        Never falls back to CWD.
        Never writes into install/share.
        """
        from ament_index_python.packages import get_package_share_directory
        import shutil

        # 1) Explicit override
        if self.locations_file_param:
            p = Path(self.locations_file_param).expanduser()
            p = p if p.is_absolute() else (Path.cwd() / p)
            if p.exists():
                return p
            raise FileNotFoundError(f"locations_file parameter points to missing file: {p}")

        # 2) Default user path
        user_dir = Path.home() / ".robotino"
        user_dir.mkdir(parents=True, exist_ok=True)
        user_yaml = user_dir / "known_locations.yaml"

        # Seed from package default if missing
        if not user_yaml.exists():
            share_dir = Path(get_package_share_directory("robotino_bts"))
            packaged_yaml = share_dir / "config" / "known_locations.yaml"
            if not packaged_yaml.exists():
                raise FileNotFoundError(f"Packaged default yaml not found: {packaged_yaml}")

            shutil.copyfile(packaged_yaml, user_yaml)
            self.get_logger().info(f"Seeded user known locations: {user_yaml}")

        return user_yaml

    
    def load_locations_from_yaml(self) -> Dict:
        """Load locations from YAML file with simple format."""
        locations = {}
        
        self.get_logger().info(f'Loading locations from: {self.locations_file_path}')
        
        if not self.locations_file_path.exists():
            self.get_logger().warn(f'YAML file not found: {self.locations_file_path}')
            self.get_logger().info('Starting with empty locations database')
            return locations
        
        try:
            with open(self.locations_file_path, 'r') as file:
                data = yaml.safe_load(file) or {}
            
            self.get_logger().info(f'Successfully loaded YAML file with {len(data)} entries')
            
            # Parse simple format: name -> {x, y, yaw}
            for location_name, location_data in data.items():
                if isinstance(location_data, dict):
                    # Extract values with defaults
                    x = float(location_data.get('x', 0.0))
                    y = float(location_data.get('y', 0.0))
                    yaw = float(location_data.get('yaw', 0.0))
                    
                    # Convert yaw to quaternion
                    qx, qy, qz, qw = self.yaw_to_quaternion(yaw)
                    
                    locations[location_name] = {
                        'x': x,
                        'y': y,
                        'z': 0.0,  # Assuming 2D
                        'yaw': yaw,
                        'qx': qx,
                        'qy': qy,
                        'qz': qz,
                        'qw': qw
                    }
                    self.get_logger().debug(f'Loaded location: {location_name} at ({x}, {y}, {yaw})')
                    
        except yaml.YAMLError as e:
            self.get_logger().error(f'YAML parsing error: {e}')
        except Exception as e:
            self.get_logger().error(f'Failed to load YAML file: {e}')
            
        return locations
    
    def broadcast_known_locations_tfs(self):
        """Broadcast all known locations as static TFs."""
        transforms = []
        
        for location_name, location_data in self.known_locations.items():
            transform = self.create_transform_msg(
                location_data,
                child_frame=location_name
            )
            transforms.append(transform)
        
        if transforms:
            self.static_broadcaster.sendTransform(transforms)
            self.get_logger().info(f'Broadcast {len(transforms)} static TFs')
    
    def create_transform_msg(self, location_data: Dict, child_frame: str) -> TransformStamped:
        """Create a TransformStamped message from location data."""
        transform = TransformStamped()
        
        # Set header
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.frame_id
        transform.child_frame_id = child_frame
        
        # Set translation
        transform.transform.translation.x = float(location_data['x'])
        transform.transform.translation.y = float(location_data['y'])
        transform.transform.translation.z = float(location_data.get('z', 0.0))
        
        # Set rotation (from quaternion)
        transform.transform.rotation.x = float(location_data['qx'])
        transform.transform.rotation.y = float(location_data['qy'])
        transform.transform.rotation.z = float(location_data['qz'])
        transform.transform.rotation.w = float(location_data['qw'])
        
        return transform
    
    def yaw_to_quaternion(self, yaw: float) -> Tuple[float, float, float, float]:
        """Convert yaw angle (radians) to quaternion."""
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qx, qy, qz, qw
    
    def quaternion_to_yaw(self, qx: float, qy: float, qz: float, qw: float) -> float:
        """Convert quaternion to yaw angle (simplified for flat surfaces)."""
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def get_robot_pose(self) -> Optional[Tuple]:
        """Get current robot pose (x, y, z, qx, qy, qz, qw)."""
        try:
            # Lookup transform from map to base_link
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.robot_frame,
                now
            )
            
            pose = (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            )
            
            return pose
            
        except (tf2_ros.LookupException, 
                tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None
        except Exception as e:
            self.get_logger().error(f'Unexpected error in get_robot_pose: {e}')
            return None
    
    def handle_add_location(self, request, response):
        """Service callback to add new location at robot's current pose."""
        location_name = request.location_name
        
        self.get_logger().info(f'Attempting to add new location: {location_name}')
        
        # Check if location already exists
        #if location_name in self.known_locations:
        #    self.get_logger().warn(f'Location {location_name} already exists')
        #    response.success = False
        #    return response
        #
        # Get robot's current pose
        pose = self.get_robot_pose()
        
        if pose is None:
            self.get_logger().error('Could not get robot pose - is TF available?')
            response.success = False
            return response
        
        # Extract yaw from quaternion
        yaw = self.quaternion_to_yaw(pose[3], pose[4], pose[5], pose[6])
        
        # Add to known locations
        self.known_locations[location_name] = {
            'x': round(pose[0], 3),
            'y': round(pose[1], 3),
            'z': round(pose[2], 3),
            'yaw': round(yaw, 3),
            'qx': pose[3],
            'qy': pose[4],
            'qz': pose[5],
            'qw': pose[6]
        }
        
        # Update YAML file
        success = self.save_locations_to_yaml()
        
        if success:
            # Broadcast new TF
            transform = self.create_transform_msg(
                self.known_locations[location_name],
                child_frame=location_name
            )
            self.static_broadcaster.sendTransform([transform])
            self.get_logger().info(f'Successfully added location: {location_name} at ({pose[0]:.2f}, {pose[1]:.2f}, {yaw:.2f})')
        else:
            self.get_logger().error(f'Failed to save location {location_name} to YAML')
        
        response.success = success
        return response
    
    def save_locations_to_yaml(self) -> bool:
        """Save current locations to YAML file in simple format."""
        try:
            # Convert to simple format: name -> {x, y, yaw}
            yaml_data = {}
            for location_name, data in self.known_locations.items():
                yaml_data[location_name] = {
                    'x': data['x'],
                    'y': data['y'],
                    'yaw': data['yaw']
                }
            
            # Write to the file we loaded from
            with open(self.locations_file_path, 'w') as file:
                yaml.dump(yaml_data, file, default_flow_style=False)
            
            self.get_logger().info(f'Saved {len(self.known_locations)} locations to {self.locations_file_path}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to save YAML file: {e}')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = KnownLocationsServer()
        
        # Use MultiThreadedExecutor for concurrent service handling
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            node.get_logger().info('Node shutdown requested')
        finally:
            executor.shutdown()
            node.destroy_node()
            
    except Exception as e:
        print(f'Error in main: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()