#!/usr/bin/env python
# -*-coding:utf-8 -*-
'''
@File    :   laserSub.py
@Time    :   2022/03/26 00:22:10
@Author  :   Cheng Liu 
@Version :   1.0
@Contact :   cliu@umd.edu
@License :   (C)Copyright 2022-2023, Cheng Liu
@Desc    :   None
'''


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import matplotlib.pyplot as plt
import numpy as np

laser_data = []
laser_time_sec = []
laser_time_nsec = []
laser_time = []
class laserSub(Node):
    """_Subscribe the topic laser and get range data and time_

    Args:
        Node (_laserSub_): _get data from sensor_msgs_
    """
    def __init__(self):
        super().__init__('laserSub')
        self.subscription = self.create_subscription(
            Range,
            'laser',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%.2f"' % msg.range)
        global laser_data
        global laser_time_sec
        global laser_time_nsec
        laser_data.append(msg.range)
        laser_time_sec.append(msg.header.stamp.sec)
        laser_time_nsec.append(msg.header.stamp.nanosec)

def printlist():
    """_Print and Visualize the data after ros2 shutdown_
    """
    global laser_data
    global laser_time
    global laser_time_sec
    global laser_time_nsec

    if len(laser_data)>20:
        #get start time and change to float type
        start_sec = float(laser_time_sec[0])
        start_nsec = float(laser_time_nsec[0])/(10**9)
        #get the last 20 data of laser range and time
        laser_data = laser_data[-20:]
        laser_time_sec = laser_time_sec[-20:]
        laser_time_nsec = laser_time_nsec[-20:]
        #change time to float type and calculate during time
        laser_time_sec = [float(x) for x in laser_time_sec]
        laser_time_nsec = [float(x)/(10**9) for x in laser_time_nsec]
        for (item1, item2) in zip(laser_time_sec, laser_time_nsec):
            laser_time.append((item1+item2)-(start_sec+start_nsec)) 
        #remove the number after third decimal places
        laser_data_show = [float(str(x)[:len(str(x))-13]) for x in laser_data]
        laser_time_show = [float(str(x)[:len(str(x))-13]) for x in laser_time]
        #show the dataset
        print('Last 20 data of laser: ',laser_data_show)
        print('Last 20 data of run time: ', laser_time_show)
        #plot the image
        plt.plot(laser_time, laser_data)
        plt.xlabel('Time (s)')
        plt.ylabel('Range (m)')
        plt.show()


def main(args=None):
    rclpy.init(args=args)

    laser_subscriber = laserSub()

    try:
        rclpy.spin(laser_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        printlist()
        laser_subscriber.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()