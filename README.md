# dvl1000_ros
ROS package, based on python, to read data from the Nortek DVL1000 and feed it to ROS

## Input and Output

Publishes
* __/dvl/dvl_msg__ linear velocities, altitude and beam data (custom DVL msg)
* __/dvl/twist__ linear velocities (nav_msgs/Odometry)
* __/dvl/pressure__ fluid pressure (sensor_msgs/FluidPressure)
* __/dvl/altitude__ distance above ground (std_msgs/Float64)

dvl/msg contains extreme outliers when the dvl struggles to lock on ground. This can happen when the distance to ground is outside the working range of 20cm to 70m. 

Linear velocities and publsihed as odometry because robot_localization does not accept twist msgs. 


## Parameters

* __/dvl/websocket_address__ websocket address of DVL. Default: _ws://192.168.0.96:10100_

## Installation

    cd ~/catkin_ws/src
    git clone -b master https://github.com/vortexntnu/dvl1000_ros.git?
    cd ~/catkin_ws
    catkin_make

### Prerequisites
Install the websocket-client library for your python workspace.
If you have pip installed use:

`pip install websocket-client`

Else:
1. Download the file from https://pypi.org/project/websocket-client/#files
2. Extract the files and navigate to the extracted folder
3. Use the following command:

    python setup.py install

Once the library is ready, download the ROS package

### Usage
Make sure you remebered to update the IP adress in the script, so that it points to the DVL.
Run the script using:

    #To run the publisher:
    rosrun dvl1000_ros publisherDVL1000.py
    
    #To run the listener:

    rosrun dvl1000_ros subscriberDVL1000.py
    
