#!/usr/bin/env python
# -*-coding:utf-8 -*-
'''
@File    :   speedSub.py
@Time    :   2022/03/24 17:47:37
@Author  :   Cheng Liu 
@Version :   1.0
@Contact :   cliu@umd.edu
@License :   (C)Copyright 2022-2023, Cheng Liu
@Desc    :   None
'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import matplotlib.pyplot as plt
import numpy as np

speed_lin_x = []
speed_lin_y = []
speed_ang_z = []
speed_time_sec = []
speed_time_nsec = []
class speedSub(Node):
    """_Subscribe the topic speed and get speed data and time_

    Args:
        Node (_speedSub_): _get data from sensor_msgs_
    """
    def __init__(self):
        super().__init__('speedSub')
        self.subscription = self.create_subscription(
            TwistStamped,
            'speed',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard linear x: "%.2f", linear y: "%.2f"' % (msg.twist.linear.x, msg.twist.linear.y))
        global speed_lin_x
        global speed_lin_y
        global speed_ang_z
        global speed_time_sec
        global speed_time_nsec
        #make msg data be accessed by global variable
        speed_lin_x.append(msg.twist.linear.x)
        speed_lin_y.append(msg.twist.linear.y)
        speed_ang_z.append(msg.twist.angular.z)
        speed_time_sec.append(msg.header.stamp.sec)
        speed_time_nsec.append(msg.header.stamp.nanosec)

def printlist():
    """_Print and Visualize the data after ros2 shutdown_
    """
    global speed_lin_x
    global speed_lin_y
    global speed_ang_z
    global speed_time_sec
    global speed_time_nsec
    speed_time = []
    speed_lin = []

    if len(speed_lin_x)>20:
        #get the start time
        start_sec = float(speed_time_sec[0])
        start_nsec = float(speed_time_nsec[0])/(10**9)
        #get the last 20 speed data
        speed_lin_x = speed_lin_x[-20:]
        speed_lin_y = speed_lin_y[-20:]
        speed_ang_z = speed_ang_z[-20:]
        #get the last 20 time data
        speed_time_sec = speed_time_sec[-20:]
        speed_time_nsec = speed_time_nsec[-20:]
        #transform the time data into float
        speed_time_sec = [float(x) for x in speed_time_sec]
        speed_time_nsec = [float(x)/(10**9) for x in speed_time_nsec]
        #make current time - start time and get linear velocity by sqrt(x^2 + y^2)
        for (item1, item2) in zip(speed_time_sec, speed_time_nsec):
            speed_time.append((item1+item2)-(start_sec+start_nsec)) 
        for (lin1, lin2) in zip(speed_lin_x, speed_lin_y):
            speed_lin.append(np.sqrt(lin1**2 + lin2**2))
        #remove the number after third decimal places
        speed_lin_show = [float(str(x)[:len(str(x))-13]) for x in speed_lin]
        speed_time_show = [float(str(x)[:len(str(x))-13]) for x in speed_time]
        #print the last 20 values
        print('Last 20 data of linear velocity: ',speed_lin_show)
        print('Last 20 data of run time: ', speed_time_show)
        #plot the image
        plt.subplot(2,1,1)
        plt.plot(speed_time, speed_lin)
        plt.title('linear velocity')
        plt.xlabel('Time (s)'), plt.ylabel('Speed (m/s)')
        plt.subplot(2,1,2)
        plt.plot(speed_time, speed_ang_z)
        plt.title('angular velocity')
        plt.xlabel('Time (s)'), plt.ylabel('Speed (rad/s)')
        plt.show()

def main(args=None):
    rclpy.init(args=args)

    speed_subscriber = speedSub()

    try:
        rclpy.spin(speed_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        printlist()
        speed_subscriber.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()