# leica_streaming_app

## Overview

The leica_streaming_app ROS package can be used with the Leica Captivate TS Survey Streaming application to publish the position of a tracked prism as a ROS message as well as the corresponding transformation.

**Keywords:** ROS, Leica total station, localization

## Installation

### Installation from Packages

To install all packages from the this repository as Debian packages use

    sudo apt-get install ros-kinetic-desktop-full

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [Boost] (boost C++ libraries)

		sudo apt-get install libboost-dev 


#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone git@github.com:AndreasAZiegler/leica_streaming_app.git
	git checkout serial
	cd ../
	catkin build leica_streaming_app


## Usage

Run the main node with

	rosrun leica_streaming_app leica_streaming_app_tcp_node _ip:="<IP address of the total station>" _port:=<port number of the streaming app>
	rosrun leica_streaming_app leica_streaming_app_serial_node _port:=<port number of the streaming app>

## Nodes

### leica_streaming_app_tcp_node

Interfaces the streaming app with a tcp connection.

### leica_streaming_app_serial_node

Interfaces the streaming app with a serial connection.


#### Subscribed Topics

* **`/paintcopter/position`** ([nav_msgs/Odometry])

	The pose from Rovio or Vicon.

* **`/leica/start_stop`** ([std_msgs/Bool])

  Start/stop the streaming of positions.


#### Published Topics

* **`/leica/position`** ([geometry_msgs/PointStamped])

	The position of the tracked prism.


#### Parameters

* **`ip`** (string, default: "10.2.86.55")

	The IP address of the total station.

* **`port`** (int, default: 5001)

	The port number of the streaming app.
