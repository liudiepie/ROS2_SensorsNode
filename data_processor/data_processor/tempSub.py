#!/usr/bin/env python
# -*-coding:utf-8 -*-
'''
@File    :   tempSub.py
@Time    :   2022/03/23 16:56:40
@Author  :   Cheng Liu 
@Version :   1.0
@Contact :   cliu@umd.edu
@License :   (C)Copyright 2022-2023, Cheng Liu
@Desc    :   Temperature Subscriber and visualization
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
import matplotlib.pyplot as plt
import numpy as np
from data_processor import ransac

temp_data = []
temp_time_sec = []
temp_time_nsec = []
temp_time = []
class tempSub(Node):
    """_Subscribe the topic temp and get temperature data and time_

    Args:
        Node (_tempSub_): _get data from sensor_msgs_
    """
    def __init__(self):
        super().__init__('tempSub')
        self.subscription = self.create_subscription(
            Temperature,
            'temp',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%.2f"' % msg.temperature)
        global temp_data
        global temp_time_sec
        global temp_time_nsec
        temp_data.append(msg.temperature)
        temp_time_sec.append(msg.header.stamp.sec)
        temp_time_nsec.append(msg.header.stamp.nanosec)

def printlist():
    """_Print and Visualize the data after ros2 shutdown_
    """
    global temp_data
    global temp_time
    global temp_time_sec
    global temp_time_nsec

    if len(temp_data)>20:
        #get start time and change to float type
        start_sec = float(temp_time_sec[0])
        start_nsec = float(temp_time_nsec[0])/(10**9)
        #get the last 20 data of laser range and time
        temp_data = temp_data[-20:]
        temp_time_sec = temp_time_sec[-20:]
        temp_time_nsec = temp_time_nsec[-20:]
        #change time to float type and calculate during time
        temp_time_sec = [float(x) for x in temp_time_sec]
        temp_time_nsec = [float(x)/(10**9) for x in temp_time_nsec]
        for (item1, item2) in zip(temp_time_sec, temp_time_nsec):
            temp_time.append((item1+item2)-(start_sec+start_nsec)) 
        #remove the number after third decimal places
        temp_data_show = [float(str(x)[:len(str(x))-13]) for x in temp_data]
        temp_time_show = [float(str(x)[:len(str(x))-13]) for x in temp_time]
        #show the dataset
        print('Last 20 data of temperature: ',temp_data_show)
        print('Last 20 data of run time: ', temp_time_show)
        #ransac
        ransac_y = ransac.ransac(temp_time[:np.newaxis], temp_data[:np.newaxis])
        #plot the image
        plt.scatter(temp_time, temp_data)
        plt.plot(temp_time, ransac_y, '-b', label='RANSAC regression')
        plt.xlabel('Time (s)')
        plt.ylabel('Temperature (C)')
        plt.show()


def main(args=None):
    rclpy.init(args=args)

    temp_subscriber = tempSub()

    try:
        rclpy.spin(temp_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        printlist()
        temp_subscriber.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()