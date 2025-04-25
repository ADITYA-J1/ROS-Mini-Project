#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import os
import time
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class LidarMapper(Node):
    def __init__(self):
        super().__init__('lidar_mapper')
        
        # Create subscription to laser scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        # Create TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create directory for saving data if it doesn't exist
        self.save_dir = os.path.join(os.path.expanduser('~'), 'robot_mapping')
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            
        self.get_logger().info(f'Saving mapping data to: {self.save_dir}')
        
        # Initialize variables
        self.last_save_time = time.time()
        self.save_interval = 5.0  # Save every 5 seconds
        self.objects = []

    def scan_callback(self, msg):
        try:
            # Get robot's current position
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            
            # Process laser scan data
            ranges = np.array(msg.ranges)
            angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
            
            # Convert to cartesian coordinates
            x = ranges * np.cos(angles)
            y = ranges * np.sin(angles)
            
            # Filter out invalid readings
            valid = ~np.isnan(ranges) & ~np.isinf(ranges)
            x = x[valid]
            y = y[valid]
            
            # Group points into objects using simple clustering
            objects = self.cluster_points(x, y)
            
            # Convert to map frame
            for obj in objects:
                # Get object center
                center_x = np.mean(obj['x'])
                center_y = np.mean(obj['y'])
                
                # Transform to map frame
                map_x = center_x + transform.transform.translation.x
                map_y = center_y + transform.transform.translation.y
                
                # Calculate dimensions
                width = np.max(obj['x']) - np.min(obj['x'])
                depth = np.max(obj['y']) - np.min(obj['y'])
                
                # Calculate orientation (angle from robot to object)
                angle = math.atan2(map_y - transform.transform.translation.y,
                                 map_x - transform.transform.translation.x)
                
                # Store object information
                self.objects.append({
                    'x': map_x,
                    'y': map_y,
                    'width': width,
                    'depth': depth,
                    'angle': angle,
                    'timestamp': time.time()
                })
            
            # Save data periodically
            current_time = time.time()
            if current_time - self.last_save_time >= self.save_interval:
                self.save_data()
                self.last_save_time = current_time
                
        except Exception as e:
            self.get_logger().error(f'Error processing scan: {str(e)}')

    def cluster_points(self, x, y):
        # Simple clustering algorithm
        clusters = []
        current_cluster = {'x': [], 'y': []}
        
        # Distance threshold for clustering
        dist_threshold = 0.5
        
        for i in range(len(x)):
            if i == 0:
                current_cluster['x'].append(x[i])
                current_cluster['y'].append(y[i])
                continue
                
            # Calculate distance to previous point
            dist = math.sqrt((x[i] - x[i-1])**2 + (y[i] - y[i-1])**2)
            
            if dist < dist_threshold:
                current_cluster['x'].append(x[i])
                current_cluster['y'].append(y[i])
            else:
                if len(current_cluster['x']) > 3:  # Minimum points for an object
                    clusters.append(current_cluster)
                current_cluster = {'x': [x[i]], 'y': [y[i]]}
        
        # Add last cluster if it has enough points
        if len(current_cluster['x']) > 3:
            clusters.append(current_cluster)
            
        return clusters

    def save_data(self):
        try:
            # Generate filename with timestamp
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            filename = os.path.join(self.save_dir, f'mapping_{timestamp}.txt')
            
            with open(filename, 'w') as f:
                f.write("# Object mapping data\n")
                f.write("# Format: x(m) y(m) width(m) depth(m) angle(rad) timestamp\n")
                
                for obj in self.objects:
                    f.write(f"{obj['x']:.3f} {obj['y']:.3f} {obj['width']:.3f} "
                           f"{obj['depth']:.3f} {obj['angle']:.3f} {obj['timestamp']:.3f}\n")
            
            self.get_logger().info(f'Saved mapping data to: {filename}')
            self.objects = []  # Clear saved objects
            
        except Exception as e:
            self.get_logger().error(f'Error saving data: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    lidar_mapper = LidarMapper()
    
    try:
        rclpy.spin(lidar_mapper)
    except KeyboardInterrupt:
        pass
    finally:
        lidar_mapper.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 