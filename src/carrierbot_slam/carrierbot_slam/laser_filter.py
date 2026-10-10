#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LaserFilterNode(Node):
    def __init__(self):
        super().__init__('laser_filter')
        
        self.get_logger().info('Laser Filter Node started')
        
        # Declare parameters
        self.declare_parameter('min_range', 1.0)
        self.declare_parameter('max_range', 12.0)
        
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        
        self.get_logger().info(f'Filtering range: {self.min_range} - {self.max_range} meters')
        
        # Create subscription and publisher
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        
        self.scan_pub = self.create_publisher(
            LaserScan,
            'scan_filtered',
            10
        )

    def scan_callback(self, msg):
        # Create filtered scan message
        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max
        filtered_scan.ranges = []
        filtered_scan.intensities = []
        
        # Filter ranges: keep only those within [min_range, max_range]
        for i, range_val in enumerate(msg.ranges):
            if (not math.isnan(range_val) and 
                self.min_range <= range_val <= self.max_range):
                filtered_scan.ranges.append(range_val)
            else:
                # Set out-of-range values to NaN
                filtered_scan.ranges.append(float('nan'))
            
            # Copy intensities if available
            if i < len(msg.intensities):
                filtered_scan.intensities.append(msg.intensities[i])
        
        self.scan_pub.publish(filtered_scan)


def main(args=None):
    rclpy.init(args=args)
    laser_filter = LaserFilterNode()
    try:
        rclpy.spin(laser_filter)
    except KeyboardInterrupt:
        pass
    finally:
        laser_filter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
