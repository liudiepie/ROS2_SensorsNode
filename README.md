# ROS2_SensorsNode
This repo is to create three different sensor publisher and subscriber.  
The following files are written in python and implement temperature, speed, and laser sensor.  

## Prerequirement
Create a file named "based on user", for example, "Task1". Under "Task1", create another file named "src". Then move the zip files under "src".   
Open a terminal under "Task1", and type  
```bash
colcon build
```
When it ends, it will show "Summary: 2 packages finish".   
As always, don’t forget to source ROS 2 in every new terminal you open.   
Now, we can move on next step.  

## Structure
```
├── data_processor
│   ├── data_processor
│   │   └── utils
│   ├── resource
│   └── test
├── image
└── sensors
    ├── resource
    ├── sensors
    ├── launch
    └── test
```

## Temp
For publisher and subscriber of temperature sensor, random uniform temperature data (20-30) are generated to publihser.  
Publisher will write the data and time into sensor_msgs/msg/Temperature and publish them to topic "temp".  
To run the publisher, type  
```bash
ros2 run sensors temp
```
Terminal will show the data which it publishes.  

Subscriber will get the data from publisher and implement RANSAC to estimate the temperature.  
Due to only twenty dataset, the desired probability is defined as 70%.  
To run the subscriber, type  
```bash
ros2 run data_processor tempSub
```
Terminal will print the last twenty dataset, and a window will plot those dataset and ransac measurement.  
Notice terminal may get error due to calculation in the RANSAC, it may be hard to find ransac estimation because of less data volume.  
For this error, it seems to show no difference between gaussian distribution and uniform distribution.  
For launch publisher and subscriber together, type  
```bash
ros2 launch sensors temp_launch.py
```
Result  
![](<image/Temperature_ransac.png>)  

## Speed 
For publisher and subscriber of speed sensor, random uniform speed data linear velocity x, y (2.0-4.0) and angular velocity z (1.0,2.0) are generated to publihser.  
Publisher will write the data and time into geometry_msgs/msg/TwistStamped and publish them to topic "speed".  
To run the publisher, type  
```bash
ros2 run sensors speed
```
Terminal will show the data which it publishes.  

Subscriber will get the data from publisher.  
To run the subscriber, type  
```bash
ros2 run data_processor speedSub
```
Terminal will print the last twenty linear velocity dataset, and a window will plot those dataset.  
For launch publisher and subscriber together, type  
```bash
ros2 launch sensors speed_launch.py
```
Again, due to small volume of data, the error will show sometimes.  
The result will show no more than restarting five times.  
Result  
![](<image/Speed.png>)  

## Laser
For publisher and subscriber of speed sensor, random gaussian distribution 1D laser data (10, 3) are generated to publihser.  
Publisher will write the data and time into sensor_msgs/msg/Range and publish them to topic "laser".  
To run the publisher, type  
```bash
ros2 run sensors laser
```
Terminal will show the data which it publishes.  

Subscriber will get the data from publisher.  
To run the subscriber, type  
```bash
ros2 run data_processor laserSub
```
Terminal will print the last twenty laser dataset, and a window will plot those dataset.  
For launch publisher and subscriber together, type  
```bash
ros2 launch sensors laser_launch.py
```
Result  
![](<image/Laser.png>)  

## Interesting issues
1. To get the last 20 data, the data should be obtained after Ctrl-C.  
   When trying on_shutdown(), ROS2 gets error on it.  
   This issue might be in the developer state.  
   For alternative method, KeyboardInterrupt can capture Ctrl-C to continue the code before shutdown.  
2. To get the time, we should publish get_clock().now() to store the time in stamp.  
   Minus the first time data in all dataset, the last 20 time data should be the time since subscriber created.  
3. The way to import ransac.py is to check the folder install/python3.8/data_processor/.  
   Though ransac.py is actually under "utils" folder, it is under "data_processor" in the "install" folder.  
   Therefore, when importing ransac.py, it should be "from data_processor import ransac".  
   RANSAC is homework in ENPM673 Perception, it is about removing outliers to receive a better linear fit.  
