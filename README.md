# Hesai QT128 Lidar Quick Guide

- [Lidar Wiring](#lidar-wiring)
   * [Overview](#overview)
   * [Lidar Connector Details](#lidar-connector-details)
- [Network Configuration](#network-configuration)
- [Web Interface](#web-interface)
- [Hesai Lidar ROS Driver Installation](#hesai-lidar-ros-driver-installation)
   * [System Requirements](#system-requirements)
   * [Install Dependencies](#install-dependencies)
   * [ROS1 Instructions](#ros1-instructions)
   * [ROS2 Instructions](#ros2-instructions)
- [Data Collection](#data-collection)
- [Configuration File: `config.yaml` Parameters](#configuration-file-configyaml-parameters)
- [References](#references)

## Lidar Wiring

### Overview

![1](https://raw.githubusercontent.com/Gonglitian/images/main/img/20250103134137726.png)

### Lidar Connector Details

Refer to the [user manual](https://www.hesaitech.com/wp-content/uploads/QT128C2X_User_manual_Q03-en-241210.pdf) (page 23). Once connected, **avoid frequent disconnections**.

> The connector supports at least 10 mating cycles; exceeding this may damage the connector.

![2](https://raw.githubusercontent.com/Gonglitian/images/main/img/20250103134414380.png)
- To Connect:
1. Power off the device.
2. Align the plug's red CPA with the socket's locking nose.
3. Insert the plug into the socket until you hear and feel a click.
4. Push the red CPA toward the socket until it clicks.

- To Disconnect:
1. Power off the device.
2. Pull the red CPA away from the socket until it clicks.
3. Press the blue locking latch and pull the plug out.

## Network Configuration

Connect to the lidar using Ethernet.

### Step 1: Identify your network interface

```bash
ifconfig
```
Look for the name of the interface connected to the LiDAR (e.g., eth0, enp3s0, etc.).

![3](https://raw.githubusercontent.com/Gonglitian/images/main/img/20250103134640567.png)

### Step 2: Choose a static IP address

Pick an IP address in the range:

`192.168.1.2`–`192.168.1.200`, or

`192.168.1.202`–`192.168.1.254`

> ⚠️ Avoid using 192.168.1.201 — it may be the default IP of the LiDAR.

Example:
`192.168.1.100`



### Step 3: Assign the IP address to your interface

Run the following command, replacing the placeholders:

```bash
sudo ifconfig <interface_name> <ip_addr>
```

```bash
sudo ifconfig eth0 192.168.1.100
```


## Web Interface

To configure parameters, check device info, or upgrade the device, use the web interface.

Open a browser and go to: 192.168.1.201

![4](https://raw.githubusercontent.com/Gonglitian/images/main/img/20250103134652689.png)

## Hesai Lidar ROS Driver Installation

### System Requirements

- Ubuntu 16.04 - ROS Kinetic
- Ubuntu 18.04 - ROS Melodic
- Ubuntu 18.04 - ROS2 Dashing
- Ubuntu 20.04 - ROS Noetic
- Ubuntu 20.04 - ROS2 Foxy
- Ubuntu 22.04 - ROS2 Humble

### Install Dependencies

```bash
sudo apt-get update
sudo apt-get install libboost-all-dev
sudo apt-get install -y libyaml-cpp-dev 
```

### ROS1 Instructions

> Tested on ROS Noetic

1. Clone the driver and build:

```bash
mkdir ${lidar_ws_name} && mkdir ${lidar_ws_name}/src
cd ${lidar_ws_name}/src
git clone --recurse-submodules https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git
cd ..
catkin_make
source devel/setup.bash
```

2. Run and visualize in `rviz`:

```bash
roslaunch hesai_ros_driver start.launch
```

### ROS2 Instructions

> Tested on ROS2 Foxy

1. Clone the driver and build:

```bash
mkdir ${lidar_ws_name} && mkdir ${lidar_ws_name}/src
cd ${lidar_ws_name}/src
git clone --recurse-submodules https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git
cd ..
colcon build --symlink-install
. install/local_setup.bash
```

2. Run and visualize in `rviz`:

```bash
ros2 launch hesai_ros_driver start.py
```

## Data Collection

Lidar data is available by subscribing to the topic:

- Topic: `/lidar_points` (can be changed in `${lidar_ws_name}/src/HesaiLidar_ROS_2.0/config/config.yaml`)
  - Msg type: [sensor_msgs/PointCloud2](https://docs.ros.org/en/jade/api/sensor_msgs/html/msg/PointCloud2.html)
  - Points per frame: 230,400
  - Frequency: 10 Hz
- Record data with `rosbag`:
  - Data rate: **~3.4 GB/min**

## Configuration File: `config.yaml` Parameters

```yaml
lidar:
- driver:
    udp_port: 2368                                       # UDP port
    ptc_port: 9347                                       # PTC port
    device_ip_address: 192.168.1.201                     # Lidar IP
    pcap_path: "<Your PCAP file path>"                   # PCAP file path (for offline playback)
    correction_file_path: "<Your correction file path>"  # Angle file for offline playback
    firetimes_path: "<Your firetime file path>"          # Firetime file path
    source_type: 2                                       # Data source: 1=live, 2=pcap, 3=rosbag
    pcap_play_synchronization: true                      # Sync PCAP playback with host time
    x: 0                                                 # Calibration: X
    y: 0                                                 # Calibration: Y
    z: 0                                                 # Calibration: Z
    roll: 0                                              # Calibration: Roll
    pitch: 0                                             # Calibration: Pitch
    yaw: 0                                               # Calibration: Yaw
ros:
    ros_frame_id: hesai_lidar                            # Frame ID for messages
    ros_recv_packet_topic: /lidar_packets                # Topic to receive packets
    ros_send_packet_topic: /lidar_packets                # Topic to send packets
    ros_send_point_cloud_topic: /lidar_points            # Topic to send point cloud
    send_packet_ros: true                                # Enable ROS packet publishing
    send_point_cloud_ros: true                           # Enable ROS point cloud publishing
```

## References

- [Hesai Lidar Driver 2.0 GitHub](https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0)
- [Official User Manual (EN)](https://www.hesaitech.com/wp-content/uploads/QT128C2X_User_manual_Q03-en-241210.pdf)
- [Official User Manual (ZH)](https://wwwcms.hesaitech.com/uploads/QT_128_C2_X_Q03_zh_241210_1a3f39016c.pdf)
