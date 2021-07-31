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

## Usage
Make sure you remebered to update the IP adress in the script, so that it points to the DVL.
Run the script using:

    #To run the publisher:
    rosrun dvl1000_ros publisherDVL1000.py
    
    #To run the listener:

    rosrun dvl1000_ros subscriberDVL1000.py
    
# Documentation
The data retrieved from the sensor is put in a JSON format. The available fields are:
Parsing Variables
BottomID = dvl_data["id"]  
BottomMode = dvl_data["name"]  
BottomTime = dvl_data["timeStampStr"]  
BottomStatus = dvl_data["frames"][1]["inputs"][0]["lines"][0]["data"][0]  

Speed of Sound variables
BottomSpeedOfSoundMin = dvl_data["frames"][2]["inputs"][0]["min"]  
BottomSpeedOfSoundMax = dvl_data["frames"][2]["inputs"][0]["max"]  
BottomSpeedOfSoundUnit = dvl_data["frames"][2]["inputs"][0]["units"]  
BottomSpeedOfSoundData = dvl_data["frames"][2]["inputs"][0]["lines"][0]["data"][0]  

Temperature varliables
BottomTempMin = dvl_data["frames"][3]["inputs"][0]["min"]  
BottomTempMax = dvl_data["frames"][3]["inputs"][0]["max"]  
BottomTempUnit = dvl_data["frames"][3]["inputs"][0]["units"]  
BottomTempData = dvl_data["frames"][3]["inputs"][0]["lines"][0]["data"][0]  

Pressure variables
BottomPressureName = dvl_data["frames"][4]["inputs"][0]["name"]  
BottomPressureMin = dvl_data["frames"][4]["inputs"][0]["min"]  
BottomPressureMax = dvl_data["frames"][4]["inputs"][0]["max"]  
measured_pressure = dvl_data["frames"][4]["inputs"][0]["lines"][0]["data"][0]  
BottomPressureUnit = dvl_data["frames"][4]["inputs"][0]["units"]  

Beam Velocity Variables
BottomBeamVelMin = dvl_data["frames"][5]["inputs"][0]["min"]  
BottomBeamVelMax = dvl_data["frames"][5]["inputs"][0]["max"]  
BottomBeamVelUnit = dvl_data["frames"][5]["inputs"][0]["units"]  
measured_velocity_beam1 = dvl_data["frames"][5]["inputs"][0]["lines"][0]["data"][0]  
measured_velocity_beam2 = dvl_data["frames"][5]["inputs"][0]["lines"][1]["data"][0]  
measured_velocity_beam3 = dvl_data["frames"][5]["inputs"][0]["lines"][2]["data"][0]  
measured_velocity_beam4 = dvl_data["frames"][5]["inputs"][0]["lines"][3]["data"][0]  
BottomBeamVel1Valid = dvl_data["frames"][5]["inputs"][0]["lines"][0]["valid"]  
BottomBeamVel2Valid = dvl_data["frames"][5]["inputs"][0]["lines"][1]["valid"]  
BottomBeamVel3Valid = dvl_data["frames"][5]["inputs"][0]["lines"][2]["valid"]  
BottomBeamVel4Valid = dvl_data["frames"][5]["inputs"][0]["lines"][3]["valid"]  

Beam FOM Variables
BottomBeamFomMin = dvl_data["frames"][5]["inputs"][1]["min"]  
BottomBeamFomMax = dvl_data["frames"][5]["inputs"][1]["max"]  
BottomBeamFomUnit = dvl_data["frames"][5]["inputs"][1]["units"]  
fom_beam1 = dvl_data["frames"][5]["inputs"][1]["lines"][0]["data"][0]  
fom_beam2 = dvl_data["frames"][5]["inputs"][1]["lines"][1]["data"][0]  
fom_beam3 = dvl_data["frames"][5]["inputs"][1]["lines"][2]["data"][0]  
fom_beam4 = dvl_data["frames"][5]["inputs"][1]["lines"][3]["data"][0]  
BottomBeamFom1Valid = dvl_data["frames"][5]["inputs"][1]["lines"][0]["valid"]  
BottomBeamFom2Valid = dvl_data["frames"][5]["inputs"][1]["lines"][1]["valid"]  
BottomBeamFom3Valid = dvl_data["frames"][5]["inputs"][1]["lines"][2]["valid"]  
BottomBeamFom4Valid = dvl_data["frames"][5]["inputs"][1]["lines"][3]["valid"]  

Beam Dist Variables
BottomBeamDistMin = dvl_data["frames"][5]["inputs"][2]["min"]  
BottomBeamDistMax = dvl_data["frames"][5]["inputs"][2]["max"]  
BottomBeamDistUnit = dvl_data["frames"][5]["inputs"][2]["units"]  
measured_altitude_beam1 = dvl_data["frames"][5]["inputs"][2]["lines"][0]["data"][0]  
measured_altitude_beam2 = dvl_data["frames"][5]["inputs"][2]["lines"][1]["data"][0]  
measured_altitude_beam3 = dvl_data["frames"][5]["inputs"][2]["lines"][2]["data"][0]  
measured_altitude_beam4 = dvl_data["frames"][5]["inputs"][2]["lines"][3]["data"][0]  
BottomBeamDist1Valid = dvl_data["frames"][5]["inputs"][2]["lines"][0]["valid"]  
BottomBeamDist2Valid = dvl_data["frames"][5]["inputs"][2]["lines"][1]["valid"]  
BottomBeamDist3Valid = dvl_data["frames"][5]["inputs"][2]["lines"][2]["valid"]  
BottomBeamDist4Valid = dvl_data["frames"][5]["inputs"][2]["lines"][3]["valid"]  

XYZ Velocity Variables
BottomXyzVelMin = dvl_data["frames"][6]["inputs"][0]["min"]  
BottomXyzVelMax = dvl_data["frames"][6]["inputs"][0]["max"]  
BottomXyzVelUnit = dvl_data["frames"][6]["inputs"][0]["units"]  
measured_velocity_x = dvl_data["frames"][6]["inputs"][0]["lines"][0]["data"][0]  
measured_velocity_y = dvl_data["frames"][6]["inputs"][0]["lines"][1]["data"][0]  
measured_velocity_z_1 = dvl_data["frames"][6]["inputs"][0]["lines"][2]["data"][0]  
measured_velocity_z_2 = dvl_data["frames"][6]["inputs"][0]["lines"][3]["data"][0]  
BottomXyzVel1Valid = dvl_data["frames"][6]["inputs"][0]["lines"][0]["valid"]  
BottomXyzVel2Valid = dvl_data["frames"][6]["inputs"][0]["lines"][1]["valid"]  
BottomXyzVel3Valid = dvl_data["frames"][6]["inputs"][0]["lines"][2]["valid"]  
BottomXyzVel4Valid = dvl_data["frames"][6]["inputs"][0]["lines"][3]["valid"]  

XYZ FOM Variables
BottomXyzFomMin = dvl_data["frames"][6]["inputs"][1]["min"]  
BottomXyzFomMax = dvl_data["frames"][6]["inputs"][1]["max"]  
BottomXyzFomUnit = dvl_data["frames"][6]["inputs"][1]["units"]  

FOM = Figure Of Merit - indicates performance/quality of the measurements
fom_velocity_x = dvl_data["frames"][6]["inputs"][1]["lines"][0]["data"][0]  
fom_velocity_y = dvl_data["frames"][6]["inputs"][1]["lines"][1]["data"][0]  
fom_velocity_z_1 = dvl_data["frames"][6]["inputs"][1]["lines"][2]["data"][0]  
fom_velocity_z_2 = dvl_data["frames"][6]["inputs"][1]["lines"][3]["data"][0]  
BottomXyzFom1Valid = dvl_data["frames"][6]["inputs"][1]["lines"][0]["valid"]  
BottomXyzFom2Valid = dvl_data["frames"][6]["inputs"][1]["lines"][1]["valid"]  
BottomXyzFom3Valid = dvl_data["frames"][6]["inputs"][1]["lines"][2]["valid"]  
BottomXyzFom4Valid = dvl_data["frames"][6]["inputs"][1]["lines"][3]["valid"]  