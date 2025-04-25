#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import time

class LidarMapper(Node):
    def __init__(self):
        super().__init__('lidar_mapper')
        
        # Create subscription to laser scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        self.scan_received = False
        self.get_logger().info('LiDAR mapper started. Checking topic status...')
        
        # Check if topic exists and has publishers
        self.timer = self.create_timer(1.0, self.check_topic_status)
        
    def check_topic_status(self):
        topic_info = self.get_topic_names_and_types()
        scan_topics = [topic for topic, _ in topic_info if '/scan' in topic]
        
        if not scan_topics:
            self.get_logger().warn('No /scan topic found!')
        else:
            self.get_logger().info(f'Found scan topics: {scan_topics}')
            
        # Check publishers on /scan topic
        publishers = self.count_publishers('/scan')
        self.get_logger().info(f'Number of publishers on /scan: {publishers}')

    def scan_callback(self, msg):
        if self.scan_received:
            return
            
        try:
            # Buffer the output
            output_lines = []
            
            output_lines.append('\n' + '='*50)
            output_lines.append('LiDAR Scan Analysis')
            output_lines.append('='*50)
            
            output_lines.append(f'\nScan Statistics:')
            output_lines.append(f'  Total points: {len(msg.ranges)}')
            output_lines.append(f'  Angle range: {math.degrees(msg.angle_min):.1f}° to {math.degrees(msg.angle_max):.1f}°')
            
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
            ranges = ranges[valid]
            angles = angles[valid]
            
            output_lines.append(f'  Valid points: {len(x)}')
            
            if len(x) == 0:
                output_lines.append('\nNo valid points found in scan!')
                for line in output_lines:
                    self.get_logger().info(line)
                return
            
            # Group points into objects using simple clustering
            objects = self.cluster_points(x, y)
            
            # Find walls (continuous segments with similar distances)
            walls = self.find_walls(ranges, angles)
            
            # Print wall information
            output_lines.append('\n' + '-'*50)
            output_lines.append('Wall Detection Results')
            output_lines.append('-'*50)
            
            for i, wall in enumerate(walls, 1):
                output_lines.append(f'\nWall {i}:')
                output_lines.append(f'  Distance: {wall["distance"]:.2f} meters')
                output_lines.append(f'  Length: {wall["length"]:.2f} meters')
                output_lines.append(f'  Angle range: {math.degrees(wall["start_angle"]):.1f}° to {math.degrees(wall["end_angle"]):.1f}°')
            
            # Print object information
            output_lines.append('\n' + '-'*50)
            output_lines.append('Object Detection Results')
            output_lines.append('-'*50)
            
            for i, obj in enumerate(objects, 1):
                # Get object center
                center_x = np.mean(obj['x'])
                center_y = np.mean(obj['y'])
                
                # Calculate dimensions
                width = np.max(obj['x']) - np.min(obj['x'])
                depth = np.max(obj['y']) - np.min(obj['y'])
                
                # Calculate angle from robot
                angle = math.atan2(center_y, center_x)
                
                # Calculate distance from robot
                distance = math.sqrt(center_x**2 + center_y**2)
                
                output_lines.append(f'\nObject {i}:')
                output_lines.append(f'  Position:')
                output_lines.append(f'    Distance: {distance:.2f} meters')
                output_lines.append(f'    Angle: {math.degrees(angle):.1f}°')
                output_lines.append(f'  Dimensions:')
                output_lines.append(f'    Width: {width:.2f} meters')
                output_lines.append(f'    Depth: {depth:.2f} meters')
            
            output_lines.append('\n' + '='*50)
            output_lines.append('Scan Complete')
            output_lines.append('='*50 + '\n')
            
            # Print all output at once
            for line in output_lines:
                self.get_logger().info(line)
            
            # Add a small delay to ensure output is displayed
            time.sleep(0.5)
            
            self.scan_received = True
            self.get_logger().info('Shutting down...')
            rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'\nError processing scan: {str(e)}')
            rclpy.shutdown()

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

    def find_walls(self, ranges, angles):
        walls = []
        current_wall = {
            'start_angle': angles[0],
            'end_angle': angles[0],
            'distance': ranges[0],
            'length': 0,
            'points': [(ranges[0], angles[0])]
        }
        
        # Parameters for wall detection
        distance_threshold = 0.2  # Increased threshold to be more tolerant of small variations
        min_wall_length = 1.0    # Increased minimum wall length
        angle_threshold = math.radians(10)  # Maximum angle change to consider as same wall
        
        for i in range(1, len(ranges)):
            # Calculate angle change
            angle_change = abs(angles[i] - angles[i-1])
            
            # Check if current point belongs to the same wall
            if (abs(ranges[i] - current_wall['distance']) < distance_threshold and 
                angle_change < angle_threshold):
                current_wall['end_angle'] = angles[i]
                current_wall['points'].append((ranges[i], angles[i]))
            else:
                # Calculate wall length using all points
                if len(current_wall['points']) > 1:
                    # Convert points to cartesian coordinates
                    x = [r * math.cos(a) for r, a in current_wall['points']]
                    y = [r * math.sin(a) for r, a in current_wall['points']]
                    
                    # Calculate wall length using endpoints
                    start_x = x[0]
                    start_y = y[0]
                    end_x = x[-1]
                    end_y = y[-1]
                    wall_length = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
                    
                    if wall_length > min_wall_length:
                        current_wall['length'] = wall_length
                        current_wall['distance'] = np.mean([r for r, _ in current_wall['points']])
                        walls.append(current_wall.copy())
                
                # Start new wall
                current_wall = {
                    'start_angle': angles[i],
                    'end_angle': angles[i],
                    'distance': ranges[i],
                    'length': 0,
                    'points': [(ranges[i], angles[i])]
                }
        
        # Add last wall if it's long enough
        if len(current_wall['points']) > 1:
            x = [r * math.cos(a) for r, a in current_wall['points']]
            y = [r * math.sin(a) for r, a in current_wall['points']]
            wall_length = math.sqrt((x[-1] - x[0])**2 + (y[-1] - y[0])**2)
            if wall_length > min_wall_length:
                current_wall['length'] = wall_length
                current_wall['distance'] = np.mean([r for r, _ in current_wall['points']])
                walls.append(current_wall)
        
        # Merge walls that are likely part of the same wall
        merged_walls = []
        if walls:
            current = walls[0]
            for wall in walls[1:]:
                # Check if walls are close in distance and angle
                if (abs(wall['distance'] - current['distance']) < distance_threshold and
                    abs(wall['start_angle'] - current['end_angle']) < angle_threshold):
                    # Merge walls
                    current['end_angle'] = wall['end_angle']
                    current['length'] += wall['length']
                    current['points'].extend(wall['points'])
                else:
                    merged_walls.append(current)
                    current = wall
            merged_walls.append(current)
        
        return merged_walls

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