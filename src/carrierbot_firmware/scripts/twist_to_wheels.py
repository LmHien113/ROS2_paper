#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class TwistToWheels(Node):
    def __init__(self):
        super().__init__('twist_to_wheels')
        
        # Declare parameters
        self.declare_parameter('wheel_distance', 0.5)
        self.wheel_distance = self.get_parameter('wheel_distance').value
        
        self.get_logger().info(f'Initialized twist_to_wheels node')
        self.get_logger().info(f'wheel_distance: {self.wheel_distance}')
        
        # Subscriber to /carrierbot_controller/cmd_vel_unstamped (from teleop_twist_keyboard or nav2)
        self.sub = self.create_subscription(
            Twist, 
            '/carrierbot_controller/cmd_vel_unstamped', 
            self.on_twist, 
            10
        )
        
        # Publisher to /cmd_v_wheels
        self.pub = self.create_publisher(
            Float64MultiArray, 
            '/cmd_v_wheels', 
            10
        )
    
    def on_twist(self, msg: Twist):
        """
        Convert Twist message to wheel velocities.
        
        For this robot (right wheel opposite direction):
        v_left = linear_x - (angular_z * wheel_distance / 2)
        v_right = -(linear_x + (angular_z * wheel_distance / 2))
        """
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Differential drive kinematics (right wheel opposite direction)
        v_left = linear - (angular * self.wheel_distance / 2.0)
        v_right = -(linear + (angular * self.wheel_distance / 2.0))
        
        # Create Float64MultiArray message
        wheels_msg = Float64MultiArray()
        wheels_msg.data = [v_left, v_right]
        
        self.pub.publish(wheels_msg)
        self.get_logger().debug(f'Twist: linear={linear:.2f}, angular={angular:.2f} -> wheels: [{v_left:.2f}, {v_right:.2f}]')


def main(args=None):
    rclpy.init(args=args)
    node = TwistToWheels()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
