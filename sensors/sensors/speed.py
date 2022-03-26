#!/usr/bin/env python
# -*-coding:utf-8 -*-
'''
@File    :   speed.py
@Time    :   2022/03/24 17:40:43
@Author  :   Cheng Liu 
@Version :   1.0
@Contact :   cliu@umd.edu
@License :   (C)Copyright 2022-2023, Cheng Liu
@Desc    :   None
'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import random

class SpeedSensorNode(Node):
    """_Speed publisher class_

    Args:
        Node (Speed): _publish random speed at 35Hz_
    """
    def __init__(self):
        #initialize the node and set the time 35Hz
        super().__init__("speedPub")
        self.speed_publisher_ = self.create_publisher(
            TwistStamped, "speed", 10)
        self.speed_timer_ = self.create_timer(
            1/35, self.publish_speed)

    def publish_speed(self):
        """_generate random linear speed and angular speed_
        """
        speed = random.uniform(2.0, 4.0)
        t = self.get_clock().now()
        msg = TwistStamped()
        msg.twist.linear.x = speed
        msg.twist.linear.y = speed
        msg.twist.angular.z = random.uniform(1.0, 2.0)
        msg.header.stamp = t.to_msg()
        self.speed_publisher_.publish(msg)
        self.get_logger().info('Publishing Speed x: "%.2f", Speed y: "%.2f"' % (msg.twist.linear.x, msg.twist.linear.y))

def main(args=None):
    rclpy.init(args=args)
    node = SpeedSensorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()