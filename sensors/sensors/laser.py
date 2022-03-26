#!/usr/bin/env python
# -*-coding:utf-8 -*-
'''
@File    :   laser.py
@Time    :   2022/03/26 00:14:04
@Author  :   Cheng Liu 
@Version :   1.0
@Contact :   cliu@umd.edu
@License :   (C)Copyright 2022-2023, Cheng Liu
@Desc    :   None
'''


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import random

class LaserSensorNode(Node):
    """_Laser publisher class_

    Args:
        Node (Laser): _publish random single laser range at 80Hz_
    """
    def __init__(self):
        #initialize the node and set the 80Hz
        super().__init__("laserPub")
        self.laser_publisher_ = self.create_publisher(
            Range, "laser", 10)
        self.laser_timer_ = self.create_timer(
            1/80, self.publish_laser)

    def publish_laser(self):
        """_generate random laser gauss distribution with avg 10 and standard 3_
        """
        laser = random.gauss(10.0, 3.0)
        t = self.get_clock().now()
        msg = Range()
        msg.range = laser
        msg.header.stamp = t.to_msg()
        self.laser_publisher_.publish(msg)
        self.get_logger().info('Publishing Laser: "%.2f"' % msg.range)

def main(args=None):
    rclpy.init(args=args)
    node = LaserSensorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()