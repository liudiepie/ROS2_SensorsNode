#!/usr/bin/env python
# -*-coding:utf-8 -*-
'''
@File    :   temp_py.py
@Time    :   2022/03/23 16:14:54
@Author  :   Cheng Liu 
@Version :   1.0
@Contact :   cliu@umd.edu
@License :   (C)Copyright 2022-2023, Cheng Liu
@Desc    :   None
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
import random

class TemperatureSensorNode(Node):
    """_Temp publisher class_

    Args:
        Node (Temp): _publish random temperature at 30Hz_
    """
    def __init__(self):
        #initialize the node and set the 30Hz
        super().__init__("tempPub")
        self.temperature_publisher_ = self.create_publisher(
            Temperature, "temp", 10)
        self.temperature_timer_ = self.create_timer(
            1/30, self.publish_temperature)



    def publish_temperature(self):
        """_generate random temp between 20.0 to 30.0_
        """
        temperature = random.uniform(20.0, 30.0)
        t = self.get_clock().now()
        msg = Temperature()
        msg.temperature = temperature
        msg.header.stamp = t.to_msg()
        self.temperature_publisher_.publish(msg)
        self.get_logger().info('Publishing Temperature: "%.2f"' % msg.temperature)

def main(args=None):
    rclpy.init(args=args)
    node = TemperatureSensorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()