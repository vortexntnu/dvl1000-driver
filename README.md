# dvl1000_ros
ROS package, based on python, to read data from the Nortek DVL1000 and feed it to ROS

## Prerequisites
Install the websocket-client library for your python workspace.
If you have pip installed use:

`pip install websocket-client`

Else:
1. Download the file from https://pypi.org/project/websocket-client/#files
2. Extract the files and navigate to the extracted folder
3. Use the following command:

    python setup.py install

Once the library is ready, download the ROS package

# Installation

    cd ~/catkin_ws/src
    git clone -b master https://github.com/vortexntnu/dvl1000_ros.git
    cd ~/catkin_ws
    catkin_make


ghese
## Usage
Make sure you remebered to update the IP adress in the script, so that it points to the DVL.
Run the script using:

    #To run the publisher:
    rosrun dvl1000_ros publisherDVL1000.os_em7180 py
    
    #To run the listener:

    rosrun dvl1000_ros subscriberDVL1000.py
    
# Documentationa
The node publishes data to the following topics: "manta/dvl", "manta/Pressure" and "nav_msgs/Odometry"
