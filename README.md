# Hesai QT128 Lidar Guidance
- [Lidar Wiring](#lidar-wiring)
   * [Overview](#overview)
   * [Details for the Connector of Lidar](#details-for-the-connector-of-lidar)
- [Network Setting](#network-setting)
- [Web Control](#web-control)
- [Hesai Lidar ROS Driver Installation](#hesai-lidar-ros-driver-installation)
   * [Environment Requirement](#environment-requirement)
   * [Install dependencies](#install-dependencies)
   * [ROS1](#ros1)
   * [ROS2](#ros2)
- [Data Collection](#data-collection)
- [Introduction to the configuration file `config.yaml` parameters](#introduction-to-the-configuration-file-configyaml-parameters)
- [References](#references)

## Lidar Wiring

### Overview

![1](https://raw.githubusercontent.com/Gonglitian/images/main/img/20250103134137726.png)

### Details for the Connector of Lidar

Refer to the [user manual](https://www.hesaitech.com/wp-content/uploads/QT128C2X_User_manual_Q03-en-241210.pdf) (page 23), after connected, **disconnection is not recommended**

>  The connector is designed to withstand at least 10 mating cycles; exceeding this number may increase the risk of connector damage.

![2](https://raw.githubusercontent.com/Gonglitian/images/main/img/20250103134414380.png)
- Connection
1. Turn off the power source.
2. Ensure the plug's red CPA is on the same side as the socket's locking nose.
3. Push the plug straight into the socket until you feel and hear a click.
4. Push the red CPA towards the socket until you feel and hear a click.

- Disconnection
1. Turn off the power source.
2. Pull the red CPA away from the socket until you feel and hear a click.
3. Depress the water blue locking latch, then pull the plug from the socket.

## Network Setting

You must access lidar via ethernet.

1. Run this command in the terminal:

```bash
sudo ifconfig ${interface_name} ${ip_addr}
```

2.  Replace **\${interface_name}** with the host computer's network interface name.
   + Enter "ifconfig" in the terminal.

![3](https://raw.githubusercontent.com/Gonglitian/images/main/img/20250103134640567.png)

3. Replace **\${ip_addr}** with the host computer's IP address.
   + **${ip_addr}** can be selected from **2 to 200** and from **202 to 254**, e.g. 192.168.1.100..

## Web Control

If needed, web control is used for setting parameters, checking device info, and upgrading.

Enter this URL in your web browser: 192.168.1.201.

![4](https://raw.githubusercontent.com/Gonglitian/images/main/img/20250103134652689.png)

## Hesai Lidar ROS Driver Installation

### Environment Requirement

- Ubuntu 16.04 - ROS Kinetic desktop
- Ubuntu 18.04 - ROS Melodic desktop
- Ubuntu 18.04 - ROS2 Dashing desktop
- Ubuntu 20.04 - ROS Noetic desktop
- Ubuntu 20.04 - ROS2 Foxy desktop
- Ubuntu 22.04 - ROS2 Humble desktop

### Install dependencies

```bash
sudo apt-get update

sudo apt-get install libboost-all-dev

sudo apt-get install -y libyaml-cpp-dev 
```

### ROS1

> Tested under ros-noetic

+ Clone driver source code and compile.

```bash
mkdir ${lidar_ws_name} && mkdir ${lidar_ws_name}/src

cd ${lidar_ws_name}/src

git clone --recurse-submodules https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git

cd ..

catkin_make

source devel/setup.bash
```

+ run and visualize in `rviz`

```bash
roslaunch hesai_ros_driver start.launch
```

### ROS2

>  Tested under ros2-foxy

+ Clone driver source code and compile.

```bash
mkdir ${lidar_ws_name} && mkdir ${lidar_ws_name}/src

cd ${lidar_ws_name}/src

git clone --recurse-submodules https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git

cd ..

colcon build --symlink-install

. install/local_setup.bash
```

+ run and visualize in `rviz`

```bash
ros2 launch hesai_ros_driver start.py
```

## Data Collection

You can get Lidar data by checking or subscribing to the topic.

+ topic: `/lidar_points` (topic name can be modified in `${lidar_ws_name}/src/HesaiLidar_ROS_2.0/config/config.yaml`)
  + msg type: [sensor_msgs/PointCloud2](https://docs.ros.org/en/jade/api/sensor_msgs/html/msg/PointCloud2.html)
  + number of points for single frame: 230400
  + frequency: 10 Hz
+ use `rosbag` to record this topic:
  + speed of data generation: **~ 3.4 GB/min**

## Introduction to the configuration file `config.yaml` parameters

```yaml
lidar:
- driver:
    udp_port: 2368                                       #UDP port of lidar
    ptc_port: 9347                                       #PTC port of lidar
    device_ip_address: 192.168.1.201                     #IP address of lidar
    pcap_path: "<Your PCAP file path>"                   #The path of pcap file (set during offline playback)
    correction_file_path: "<Your correction file path>"  #LiDAR angle file, required for offline playback of pcap/packet rosbag
    firetimes_path: "<Your firetime file path>"          #The path of firetimes file
    source_type: 2                                       #The type of data source, 1: real-time lidar connection, 2: pcap, 3: packet rosbag
    pcap_play_synchronization: true                      #Pcap play rate synchronize with the host time
    x: 0                                                 #Calibration parameter
    y: 0                                                 #Calibration parameter
    z: 0                                                 #Calibration parameter
    roll: 0                                              #Calibration parameter
    pitch: 0                                             #Calibration parameter
    yaw: 0                                               #Calibration parameter
ros:
    ros_frame_id: hesai_lidar                            #Frame id of packet message and point cloud message
    ros_recv_packet_topic: /lidar_packets                #Topic used to receive lidar packets from ROS
    ros_send_packet_topic: /lidar_packets                #Topic used to send lidar packets through ROS
    ros_send_point_cloud_topic: /lidar_points            #Topic used to send point cloud through ROS
    send_packet_ros: true                                #true: Send packets through ROS 
    send_point_cloud_ros: true                           #true: Send point cloud through ROS 
```

## References

+ [Hesai Lidar Driver 2.0 repo](https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0)
+ [Official User Manual-en](https://www.hesaitech.com/wp-content/uploads/QT128C2X_User_manual_Q03-en-241210.pdf) (click to open pdf)
+ [Official User Manual-zh](https://wwwcms.hesaitech.com/uploads/QT_128_C2_X_Q03_zh_241210_1a3f39016c.pdf) (click to open pdf)
