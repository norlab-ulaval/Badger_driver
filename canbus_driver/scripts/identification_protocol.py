#!/usr/bin/env python3

import math
import os
import struct
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Identification(Node):

    def __init__(self):
        super().__init__('Motor_identification_protocole')

        self.speed_pub = self.create_publisher(Twist, '/cmd_vel', 20)
        
        self.counter = 0.1

        self.timer_inc = self.create_timer(10, self.inc)

        
        self.timer_pub = self.create_timer(0.1, self.pub_speed)

    def inc(self):

        self.counter += 0.5

    def pub_speed(self):
        msg = Twist()

        # msg.linear.x = float(self.counter)
        msg.linear.x = 2.0
        msg.linear.y = 0.0   
        msg.linear.z = 0.0   
        
        msg.angular.x = 0.0  
        msg.angular.y = 0.0  
        msg.angular.z = 0.0

        self.speed_pub.publish(msg)
        self.get_logger().info(f'Publishing speed: {msg.linear.x:.2f} m/s')


def main(args=None):
    rclpy.init(args=args)
    node = Identification()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot when shutting down
        stop_msg = Twist()
        node.speed_pub.publish(stop_msg)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()