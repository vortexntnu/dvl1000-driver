#!/usr/bin/env python
import rospy
import json
import time
from websocket import create_connection
from std_msgs.msg import String
from dvl1000_ros.msg import NORTEK

#Websocket connection stuff. Make sure the websocket-client python library is installed.
ws = create_connection("ws://10.42.0.96:10100", subprotocols=["data-transfer-nortek"])
print("Connecting to DVL1000 via websocket...")
result = ws.recv()
print("Status: '%s'" % result)

#Initiating Variables
BottomID = 0
BottomMode = "0"
BottomTime = "0"
BottomStatus = 0

BottomSpeedOfSoundMin = 0
BottomSpeedOfSoundMax = 0
BottomSpeedOfSoundUnit = "0"
BottomSpeedOfSoundData = 0

BottomTempMin = 0
BottomTempMax = 0
BottomTempUnit = "0"
BottomTempData = 0

BottomPressureName = "0"
BottomPressureMin = 0
BottomPressureMax = 0
BottomPressureData = 0
BottomPressureUnit = "0"

BottomBeamVelMin = 0
BottomBeamVelMax = 0
BottomBeamVelUnit = "0"
BottomBeamVel1Data = 0
BottomBeamVel2Data = 0
BottomBeamVel3Data = 0
BottomBeamVel4Data = 0
BottomBeamVel1Valid = 0
BottomBeamVel2Valid = 0
BottomBeamVel3Valid = 0
BottomBeamVel4Valid = 0

BottomBeamFomMin = 0
BottomBeamFomMax = 0
BottomBeamFomUnit = "0"
BottomBeamFom1Data = 0
BottomBeamFom2Data = 0
BottomBeamFom3Data = 0
BottomBeamFom4Data = 0
BottomBeamFom1Valid = 0
BottomBeamFom2Valid = 0
BottomBeamFom3Valid = 0
BottomBeamFom4Valid = 0

BottomBeamDistMin = 0
BottomBeamDistMax = 0
BottomBeamDistUnit = "0"
BottomBeamDist1Data = 0
BottomBeamDist2Data = 0
BottomBeamDist3Data = 0
BottomBeamDist4Data = 0
BottomBeamDist1Valid = 0
BottomBeamDist2Valid = 0
BottomBeamDist3Valid = 0
BottomBeamDist4Valid = 0

BottomXyzVelMin = 0
BottomXyzVelMax = 0
BottomXyzVelUnit = "0"
BottomXyzVel1Data = 0
BottomXyzVel2Data = 0
BottomXyzVel3Data = 0
BottomXyzVel4Data = 0
BottomXyzVel1Valid = 0
BottomXyzVel2Valid = 0
BottomXyzVel3Valid = 0
BottomXyzVel4Valid = 0

BottomXyzFomMin = 0
BottomXyzFomMax = 0
BottomXyzFomUnit = "0"
BottomXyzFom1Data = 0
BottomXyzFom2Data = 0
BottomXyzFom3Data = 0
BottomXyzFom4Data = 0
BottomXyzFom1Valid = 0
BottomXyzFom2Valid = 0
BottomXyzFom3Valid = 0
BottomXyzFom4Valid = 0

WaterID = 0
WaterMode = "0"
WaterTime = "0"
WaterStatus = 0

WaterSpeedOfSoundMin = 0
WaterSpeedOfSoundMax = 0
WaterSpeedOfSoundUnit = "0"
WaterSpeedOfSoundData = 0

WaterTempMin = 0
WaterTempMax = 0
WaterTempUnit = "0"
WaterTempData = 0

WaterPressureName = "0"
WaterPressureMin = 0
WaterPressureMax = 0
WaterPressureData = 0
WaterPressureUnit = "0"

WaterBeamVelMin = 0
WaterBeamVelMax = 0
WaterBeamVelUnit = "0"
WaterBeamVel1Data = 0
WaterBeamVel2Data = 0
WaterBeamVel3Data = 0
WaterBeamVel4Data = 0
WaterBeamVel1Valid = 0
WaterBeamVel2Valid = 0
WaterBeamVel3Valid = 0
WaterBeamVel4Valid = 0

WaterBeamFomMin = 0
WaterBeamFomMax = 0
WaterBeamFomUnit = "0"
WaterBeamFom1Data = 0
WaterBeamFom2Data = 0
WaterBeamFom3Data = 0
WaterBeamFom4Data = 0
WaterBeamFom1Valid = 0
WaterBeamFom2Valid = 0
WaterBeamFom3Valid = 0
WaterBeamFom4Valid = 0

WaterBeamDistMin = 0
WaterBeamDistMax = 0
WaterBeamDistUnit = "0"
WaterBeamDist1Data = 0
WaterBeamDist2Data = 0
WaterBeamDist3Data = 0
WaterBeamDist4Data = 0
WaterBeamDist1Valid = 0
WaterBeamDist2Valid = 0
WaterBeamDist3Valid = 0
WaterBeamDist4Valid = 0

WaterXyzVelMin = 0
WaterXyzVelMax = 0
WaterXyzVelUnit = "0"
WaterXyzVel1Data = 0
WaterXyzVel2Data = 0
WaterXyzVel3Data = 0
WaterXyzVel4Data = 0
WaterXyzVel1Valid = 0
WaterXyzVel2Valid = 0
WaterXyzVel3Valid = 0
WaterXyzVel4Valid = 0

WaterXyzFomMin = 0
WaterXyzFomMax = 0
WaterXyzFomUnit = "0"
WaterXyzFom1Data = 0
WaterXyzFom2Data = 0
WaterXyzFom3Data = 0
WaterXyzFom4Data = 0
WaterXyzFom1Valid = 0
WaterXyzFom2Valid = 0
WaterXyzFom3Valid = 0
WaterXyzFom4Valid = 0

def publishDVLdata():
	
	global BottomID
	global BottomMode
	global BottomTime
	global BottomStatus

	global BottomSpeedOfSoundMin
	global BottomSpeedOfSoundMax
	global BottomSpeedOfSoundUnit
	global BottomSpeedOfSoundData

	global BottomTempMin
	global BottomTempMax
	global BottomTempUnit
	global BottomTempData

	global BottomPressureName
	global BottomPressureMin
	global BottomPressureMax
	global BottomPressureData
	global BottomPressureUnit

	global BottomBeamVelMin
	global BottomBeamVelMax
	global BottomBeamVelUnit
	global BottomBeamVel1Data
	global BottomBeamVel2Data
	global BottomBeamVel3Data
	global BottomBeamVel4Data
	global BottomBeamVel1Valid
	global BottomBeamVel2Valid
	global BottomBeamVel3Valid
	global BottomBeamVel4Valid

	global BottomBeamFomMin
	global BottomBeamFomMax
	global BottomBeamFomUnit
	global BottomBeamFom1Data
	global BottomBeamFom2Data
	global BottomBeamFom3Data
	global BottomBeamFom4Data
	global BottomBeamFom1Valid
	global BottomBeamFom2Valid
	global BottomBeamFom3Valid
	global BottomBeamFom4Valid

	global BottomBeamDistMin
	global BottomBeamDistMax
	global BottomBeamDistUnit
	global BottomBeamDist1Data
	global BottomBeamDist2Data
	global BottomBeamDist3Data
	global BottomBeamDist4Data
	global BottomBeamDist1Valid
	global BottomBeamDist2Valid
	global BottomBeamDist3Valid
	global BottomBeamDist4Valid

	global BottomXyzVelMin
	global BottomXyzVelMax
	global BottomXyzVelUnit
	global BottomXyzVel1Data
	global BottomXyzVel2Data
	global BottomXyzVel3Data
	global BottomXyzVel4Data
	global BottomXyzVel1Valid
	global BottomXyzVel2Valid
	global BottomXyzVel3Valid
	global BottomXyzVel4Valid

	global BottomXyzFomMin
	global BottomXyzFomMax
	global BottomXyzFomUnit
	global BottomXyzFom1Data
	global BottomXyzFom2Data
	global BottomXyzFom3Data
	global BottomXyzFom4Data
	global BottomXyzFom1Valid
	global BottomXyzFom2Valid
	global BottomXyzFom3Valid
	global BottomXyzFom4Valid
	
	global WaterID
	global WaterMode
	global WaterTime
	global WaterStatus

	global WaterSpeedOfSoundMin
	global WaterSpeedOfSoundMax
	global WaterSpeedOfSoundUnit
	global WaterSpeedOfSoundData

	global WaterTempMin
	global WaterTempMax
	global WaterTempUnit
	global WaterTempData

	global WaterPressureName
	global WaterPressureMin
	global WaterPressureMax
	global WaterPressureData
	global WaterPressureUnit

	global WaterBeamVelMin
	global WaterBeamVelMax
	global WaterBeamVelUnit
	global WaterBeamVel1Data
	global WaterBeamVel2Data
	global WaterBeamVel3Data
	global WaterBeamVel4Data
	global WaterBeamVel1Valid
	global WaterBeamVel2Valid
	global WaterBeamVel3Valid
	global WaterBeamVel4Valid

	global WaterBeamFomMin
	global WaterBeamFomMax
	global WaterBeamFomUnit
	global WaterBeamFom1Data
	global WaterBeamFom2Data
	global WaterBeamFom3Data
	global WaterBeamFom4Data
	global WaterBeamFom1Valid
	global WaterBeamFom2Valid
	global WaterBeamFom3Valid
	global WaterBeamFom4Valid

	global WaterBeamDistMin
	global WaterBeamDistMax
	global WaterBeamDistUnit
	global WaterBeamDist1Data
	global WaterBeamDist2Data
	global WaterBeamDist3Data
	global WaterBeamDist4Data
	global WaterBeamDist1Valid
	global WaterBeamDist2Valid
	global WaterBeamDist3Valid
	global WaterBeamDist4Valid

	global WaterXyzVelMin
	global WaterXyzVelMax
	global WaterXyzVelUnit
	global WaterXyzVel1Data
	global WaterXyzVel2Data
	global WaterXyzVel3Data
	global WaterXyzVel4Data
	global WaterXyzVel1Valid
	global WaterXyzVel2Valid
	global WaterXyzVel3Valid
	global WaterXyzVel4Valid

	global WaterXyzFomMin
	global WaterXyzFomMax
	global WaterXyzFomUnit
	global WaterXyzFom1Data
	global WaterXyzFom2Data
	global WaterXyzFom3Data
	global WaterXyzFom4Data
	global WaterXyzFom1Valid
	global WaterXyzFom2Valid
	global WaterXyzFom3Valid
	global WaterXyzFom4Valid

	result = ws.recv()

	#Get JSON Data
	theData = json.loads(result)

	#Alternating data each time data is recieved. Each with unique ID(4123 and 8219) and dataset. Using IF to sort this out
	#8219 seem to have additional information like Status, Speed of Sound, Temperature
	#More data if more modes of tracking is turned on. IDs 4125 and 8221 belongs to Water Tracking mode.
	#IDs 4123 and 8219 belongs to Bottom Track mode.

	pub = rospy.Publisher('sensors/dvl/dvl1000', NORTEK, queue_size=10)
	rospy.init_node('DVL1000', anonymous=True)
	rate = rospy.Rate(10) # 10hz

	theDVL = NORTEK()

	#Bottom-Tracking
	if theData["id"] == 8219:
		#Parsing Variables
		BottomID = theData["id"]
		BottomMode = theData["name"]
		BottomTime = theData["timeStampStr"]
		BottomStatus = theData["frames"][1]["inputs"][0]["lines"][0]["data"][0]
		
		#Speed of Sound variables
		BottomSpeedOfSoundMin = theData["frames"][2]["inputs"][0]["min"]
		BottomSpeedOfSoundMax = theData["frames"][2]["inputs"][0]["max"]
		BottomSpeedOfSoundUnit = theData["frames"][2]["inputs"][0]["units"]
		BottomSpeedOfSoundData = theData["frames"][2]["inputs"][0]["lines"][0]["data"][0]
		
		#Temperature varliables
		BottomTempMin = theData["frames"][3]["inputs"][0]["min"]
		BottomTempMax = theData["frames"][3]["inputs"][0]["max"]
		BottomTempUnit = theData["frames"][3]["inputs"][0]["units"]
		BottomTempData = theData["frames"][3]["inputs"][0]["lines"][0]["data"][0]
		
		#Pressure variables
		BottomPressureName = theData["frames"][4]["inputs"][0]["name"]
		BottomPressureMin = theData["frames"][4]["inputs"][0]["min"]
		BottomPressureMax = theData["frames"][4]["inputs"][0]["max"]
		BottomPressureData = theData["frames"][4]["inputs"][0]["lines"][0]["data"][0]
		BottomPressureUnit = theData["frames"][4]["inputs"][0]["units"]
		
		#Beam Velocity Variables
		BottomBeamVelMin = theData["frames"][5]["inputs"][0]["min"]
		BottomBeamVelMax = theData["frames"][5]["inputs"][0]["max"]
		BottomBeamVelUnit = theData["frames"][5]["inputs"][0]["units"]
		BottomBeamVel1Data = theData["frames"][5]["inputs"][0]["lines"][0]["data"][0]
		BottomBeamVel2Data = theData["frames"][5]["inputs"][0]["lines"][1]["data"][0]
		BottomBeamVel3Data = theData["frames"][5]["inputs"][0]["lines"][2]["data"][0]
		BottomBeamVel4Data = theData["frames"][5]["inputs"][0]["lines"][3]["data"][0]
		BottomBeamVel1Valid = theData["frames"][5]["inputs"][0]["lines"][0]["valid"]
		BottomBeamVel2Valid = theData["frames"][5]["inputs"][0]["lines"][1]["valid"]
		BottomBeamVel3Valid = theData["frames"][5]["inputs"][0]["lines"][2]["valid"]
		BottomBeamVel4Valid = theData["frames"][5]["inputs"][0]["lines"][3]["valid"]
		
		#Beam FOM Variables
		BottomBeamFomMin = theData["frames"][5]["inputs"][1]["min"]
		BottomBeamFomMax = theData["frames"][5]["inputs"][1]["max"]
		BottomBeamFomUnit = theData["frames"][5]["inputs"][1]["units"]
		BottomBeamFom1Data = theData["frames"][5]["inputs"][1]["lines"][0]["data"][0]
		BottomBeamFom2Data = theData["frames"][5]["inputs"][1]["lines"][1]["data"][0]
		BottomBeamFom3Data = theData["frames"][5]["inputs"][1]["lines"][2]["data"][0]
		BottomBeamFom4Data = theData["frames"][5]["inputs"][1]["lines"][3]["data"][0]
		BottomBeamFom1Valid = theData["frames"][5]["inputs"][1]["lines"][0]["valid"]
		BottomBeamFom2Valid = theData["frames"][5]["inputs"][1]["lines"][1]["valid"]
		BottomBeamFom3Valid = theData["frames"][5]["inputs"][1]["lines"][2]["valid"]
		BottomBeamFom4Valid = theData["frames"][5]["inputs"][1]["lines"][3]["valid"]
		
		#Beam Dist Variables
		BottomBeamDistMin = theData["frames"][5]["inputs"][2]["min"]
		BottomBeamDistMax = theData["frames"][5]["inputs"][2]["max"]
		BottomBeamDistUnit = theData["frames"][5]["inputs"][2]["units"]
		BottomBeamDist1Data = theData["frames"][5]["inputs"][2]["lines"][0]["data"][0]
		BottomBeamDist2Data = theData["frames"][5]["inputs"][2]["lines"][1]["data"][0]
		BottomBeamDist3Data = theData["frames"][5]["inputs"][2]["lines"][2]["data"][0]
		BottomBeamDist4Data = theData["frames"][5]["inputs"][2]["lines"][3]["data"][0]
		BottomBeamDist1Valid = theData["frames"][5]["inputs"][2]["lines"][0]["valid"]
		BottomBeamDist2Valid = theData["frames"][5]["inputs"][2]["lines"][1]["valid"]
		BottomBeamDist3Valid = theData["frames"][5]["inputs"][2]["lines"][2]["valid"]
		BottomBeamDist4Valid = theData["frames"][5]["inputs"][2]["lines"][3]["valid"]
		
		#XYZ Velocity Variables
		BottomXyzVelMin = theData["frames"][6]["inputs"][0]["min"]
		BottomXyzVelMax = theData["frames"][6]["inputs"][0]["max"]
		BottomXyzVelUnit = theData["frames"][6]["inputs"][0]["units"]
		BottomXyzVel1Data = theData["frames"][6]["inputs"][0]["lines"][0]["data"][0]
		BottomXyzVel2Data = theData["frames"][6]["inputs"][0]["lines"][1]["data"][0]
		BottomXyzVel3Data = theData["frames"][6]["inputs"][0]["lines"][2]["data"][0]
		BottomXyzVel4Data = theData["frames"][6]["inputs"][0]["lines"][3]["data"][0]
		BottomXyzVel1Valid = theData["frames"][6]["inputs"][0]["lines"][0]["valid"]
		BottomXyzVel2Valid = theData["frames"][6]["inputs"][0]["lines"][1]["valid"]
		BottomXyzVel3Valid = theData["frames"][6]["inputs"][0]["lines"][2]["valid"]
		BottomXyzVel4Valid = theData["frames"][6]["inputs"][0]["lines"][3]["valid"]
		
		#XYZ FOM Variables
		BottomXyzFomMin = theData["frames"][6]["inputs"][1]["min"]
		BottomXyzFomMax = theData["frames"][6]["inputs"][1]["max"]
		BottomXyzFomUnit = theData["frames"][6]["inputs"][1]["units"]
		BottomXyzFom1Data = theData["frames"][6]["inputs"][1]["lines"][0]["data"][0]
		BottomXyzFom2Data = theData["frames"][6]["inputs"][1]["lines"][1]["data"][0]
		BottomXyzFom3Data = theData["frames"][6]["inputs"][1]["lines"][2]["data"][0]
		BottomXyzFom4Data = theData["frames"][6]["inputs"][1]["lines"][3]["data"][0]
		BottomXyzFom1Valid = theData["frames"][6]["inputs"][1]["lines"][0]["valid"]
		BottomXyzFom2Valid = theData["frames"][6]["inputs"][1]["lines"][1]["valid"]
		BottomXyzFom3Valid = theData["frames"][6]["inputs"][1]["lines"][2]["valid"]
		BottomXyzFom4Valid = theData["frames"][6]["inputs"][1]["lines"][3]["valid"]
		
		theDVL.time_stamp = BottomTime
		theDVL.dvl_status = BottomStatus
		theDVL.temperature = BottomTempData
		theDVL.temperature_min = BottomTempMin
		theDVL.temperature_max = BottomTempMax
		theDVL.temperature_unit = BottomTempUnit
		theDVL.sound_speed = BottomSpeedOfSoundData
		theDVL.sound_speed_min = BottomSpeedOfSoundMin
		theDVL.sound_speed_max = BottomSpeedOfSoundMax
		theDVL.sound_speed_unit = BottomSpeedOfSoundUnit
		theDVL.pressure = BottomPressureData
		theDVL.pressure_min = BottomPressureMin
		theDVL.pressure_max = BottomPressureMax
		theDVL.pressure_unit = BottomPressureUnit
		
	#Water-Tracking
	if theData["id"] == 8221:
		#Parsing Variables
		WaterID = theData["id"]
		WaterMode = theData["name"]
		WaterTime = theData["timeStampStr"]
		WaterStatus = theData["frames"][1]["inputs"][0]["lines"][0]["data"][0]
		
		#Speed of Sound variables
		WaterSpeedOfSoundMin = theData["frames"][2]["inputs"][0]["min"]
		WaterSpeedOfSoundMax = theData["frames"][2]["inputs"][0]["max"]
		WaterSpeedOfSoundUnit = theData["frames"][2]["inputs"][0]["units"]
		WaterSpeedOfSoundData = theData["frames"][2]["inputs"][0]["lines"][0]["data"][0]
		
		#Temperature varliables
		WaterTempMin = theData["frames"][3]["inputs"][0]["min"]
		WaterTempMax = theData["frames"][3]["inputs"][0]["max"]
		WaterTempUnit = theData["frames"][3]["inputs"][0]["units"]
		WaterTempData = theData["frames"][3]["inputs"][0]["lines"][0]["data"][0]
		
		#Pressure variables
		WaterPressureName = theData["frames"][4]["inputs"][0]["name"]
		WaterPressureMin = theData["frames"][4]["inputs"][0]["min"]
		WaterPressureMax = theData["frames"][4]["inputs"][0]["max"]
		WaterPressureData = theData["frames"][4]["inputs"][0]["lines"][0]["data"][0]
		WaterPressureUnit = theData["frames"][4]["inputs"][0]["units"]
		
		#Beam Velocity Variables
		WaterBeamVelMin = theData["frames"][5]["inputs"][0]["min"]
		WaterBeamVelMax = theData["frames"][5]["inputs"][0]["max"]
		WaterBeamVelUnit = theData["frames"][5]["inputs"][0]["units"]
		WaterBeamVel1Data = theData["frames"][5]["inputs"][0]["lines"][0]["data"][0]
		WaterBeamVel2Data = theData["frames"][5]["inputs"][0]["lines"][1]["data"][0]
		WaterBeamVel3Data = theData["frames"][5]["inputs"][0]["lines"][2]["data"][0]
		WaterBeamVel4Data = theData["frames"][5]["inputs"][0]["lines"][3]["data"][0]
		WaterBeamVel1Valid = theData["frames"][5]["inputs"][0]["lines"][0]["valid"]
		WaterBeamVel2Valid = theData["frames"][5]["inputs"][0]["lines"][1]["valid"]
		WaterBeamVel3Valid = theData["frames"][5]["inputs"][0]["lines"][2]["valid"]
		WaterBeamVel4Valid = theData["frames"][5]["inputs"][0]["lines"][3]["valid"]
		
		#Beam FOM Variables
		WaterBeamFomMin = theData["frames"][5]["inputs"][1]["min"]
		WaterBeamFomMax = theData["frames"][5]["inputs"][1]["max"]
		WaterBeamFomUnit = theData["frames"][5]["inputs"][1]["units"]
		WaterBeamFom1Data = theData["frames"][5]["inputs"][1]["lines"][0]["data"][0]
		WaterBeamFom2Data = theData["frames"][5]["inputs"][1]["lines"][1]["data"][0]
		WaterBeamFom3Data = theData["frames"][5]["inputs"][1]["lines"][2]["data"][0]
		WaterBeamFom4Data = theData["frames"][5]["inputs"][1]["lines"][3]["data"][0]
		WaterBeamFom1Valid = theData["frames"][5]["inputs"][1]["lines"][0]["valid"]
		WaterBeamFom2Valid = theData["frames"][5]["inputs"][1]["lines"][1]["valid"]
		WaterBeamFom3Valid = theData["frames"][5]["inputs"][1]["lines"][2]["valid"]
		WaterBeamFom4Valid = theData["frames"][5]["inputs"][1]["lines"][3]["valid"]
		
		#Beam Dist Variables
		WaterBeamDistMin = theData["frames"][5]["inputs"][2]["min"]
		WaterBeamDistMax = theData["frames"][5]["inputs"][2]["max"]
		WaterBeamDistUnit = theData["frames"][5]["inputs"][2]["units"]
		WaterBeamDist1Data = theData["frames"][5]["inputs"][2]["lines"][0]["data"][0]
		WaterBeamDist2Data = theData["frames"][5]["inputs"][2]["lines"][1]["data"][0]
		WaterBeamDist3Data = theData["frames"][5]["inputs"][2]["lines"][2]["data"][0]
		WaterBeamDist4Data = theData["frames"][5]["inputs"][2]["lines"][3]["data"][0]
		WaterBeamDist1Valid = theData["frames"][5]["inputs"][2]["lines"][0]["valid"]
		WaterBeamDist2Valid = theData["frames"][5]["inputs"][2]["lines"][1]["valid"]
		WaterBeamDist3Valid = theData["frames"][5]["inputs"][2]["lines"][2]["valid"]
		WaterBeamDist4Valid = theData["frames"][5]["inputs"][2]["lines"][3]["valid"]
		
		#XYZ Velocity Variables
		WaterXyzVelMin = theData["frames"][6]["inputs"][0]["min"]
		WaterXyzVelMax = theData["frames"][6]["inputs"][0]["max"]
		WaterXyzVelUnit = theData["frames"][6]["inputs"][0]["units"]
		WaterXyzVel1Data = theData["frames"][6]["inputs"][0]["lines"][0]["data"][0]
		WaterXyzVel2Data = theData["frames"][6]["inputs"][0]["lines"][1]["data"][0]
		WaterXyzVel3Data = theData["frames"][6]["inputs"][0]["lines"][2]["data"][0]
		WaterXyzVel4Data = theData["frames"][6]["inputs"][0]["lines"][3]["data"][0]
		WaterXyzVel1Valid = theData["frames"][6]["inputs"][0]["lines"][0]["valid"]
		WaterXyzVel2Valid = theData["frames"][6]["inputs"][0]["lines"][1]["valid"]
		WaterXyzVel3Valid = theData["frames"][6]["inputs"][0]["lines"][2]["valid"]
		WaterXyzVel4Valid = theData["frames"][6]["inputs"][0]["lines"][3]["valid"]
		
		#XYZ FOM Variables
		WaterXyzFomMin = theData["frames"][6]["inputs"][1]["min"]
		WaterXyzFomMax = theData["frames"][6]["inputs"][1]["max"]
		WaterXyzFomUnit = theData["frames"][6]["inputs"][1]["units"]
		WaterXyzFom1Data = theData["frames"][6]["inputs"][1]["lines"][0]["data"][0]
		WaterXyzFom2Data = theData["frames"][6]["inputs"][1]["lines"][1]["data"][0]
		WaterXyzFom3Data = theData["frames"][6]["inputs"][1]["lines"][2]["data"][0]
		WaterXyzFom4Data = theData["frames"][6]["inputs"][1]["lines"][3]["data"][0]
		WaterXyzFom1Valid = theData["frames"][6]["inputs"][1]["lines"][0]["valid"]
		WaterXyzFom2Valid = theData["frames"][6]["inputs"][1]["lines"][1]["valid"]
		WaterXyzFom3Valid = theData["frames"][6]["inputs"][1]["lines"][2]["valid"]
		WaterXyzFom4Valid = theData["frames"][6]["inputs"][1]["lines"][3]["valid"]
		
		theDVL.time_stamp = WaterTime
		theDVL.dvl_status = WaterStatus
		theDVL.temperature = WaterTempData
		theDVL.temperature_min = WaterTempMin
		theDVL.temperature_max = WaterTempMax
		theDVL.temperature_unit = WaterTempUnit
		theDVL.sound_speed = WaterSpeedOfSoundData
		theDVL.sound_speed_min = WaterSpeedOfSoundMin
		theDVL.sound_speed_max = WaterSpeedOfSoundMax
		theDVL.sound_speed_unit = WaterSpeedOfSoundUnit
		theDVL.pressure = WaterPressureData
		theDVL.pressure_min = WaterPressureMin
		theDVL.pressure_max = WaterPressureMax
		theDVL.pressure_unit = WaterPressureUnit
	
    
	while not rospy.is_shutdown():
		#Bottom Beam Vel
		theDVL.bottom_vel_min = BottomBeamVelMin
		theDVL.bottom_vel_max = BottomBeamVelMax
		theDVL.bottom_vel_unit = BottomBeamVelUnit
		theDVL.bottom_vel_1_data = BottomBeamVel1Data
		theDVL.bottom_vel_2_data = BottomBeamVel2Data
		theDVL.bottom_vel_3_data = BottomBeamVel3Data
		theDVL.bottom_vel_4_data = BottomBeamVel4Data
		theDVL.bottom_vel_1_status = BottomBeamVel1Valid
		theDVL.bottom_vel_2_status = BottomBeamVel2Valid
		theDVL.bottom_vel_3_status = BottomBeamVel3Valid
		theDVL.bottom_vel_4_status = BottomBeamVel4Valid
		#Bottom Beam FOM
		theDVL.bottom_fom_min = BottomBeamFomMin
		theDVL.bottom_fom_max = BottomBeamFomMax
		theDVL.bottom_fom_unit = BottomBeamFomUnit
		theDVL.bottom_fom_1_data = BottomBeamFom1Data
		theDVL.bottom_fom_2_data = BottomBeamFom2Data
		theDVL.bottom_fom_3_data = BottomBeamFom3Data
		theDVL.bottom_fom_4_data = BottomBeamFom4Data
		theDVL.bottom_fom_1_status = BottomBeamFom1Valid
		theDVL.bottom_fom_2_status = BottomBeamFom2Valid
		theDVL.bottom_fom_3_status = BottomBeamFom3Valid
		theDVL.bottom_fom_4_status = BottomBeamFom4Valid
		#Bottom Beam Dist
		theDVL.bottom_dist_min = BottomBeamDistMin
		theDVL.bottom_dist_max = BottomBeamDistMax
		theDVL.bottom_dist_unit = BottomBeamDistUnit
		theDVL.bottom_dist_1_data = BottomBeamDist1Data
		theDVL.bottom_dist_2_data = BottomBeamDist2Data
		theDVL.bottom_dist_3_data = BottomBeamDist3Data
		theDVL.bottom_dist_4_data = BottomBeamDist4Data
		theDVL.bottom_dist_1_status = BottomBeamDist1Valid
		theDVL.bottom_dist_2_status = BottomBeamDist2Valid
		theDVL.bottom_dist_3_status = BottomBeamDist3Valid
		theDVL.bottom_dist_4_status = BottomBeamDist4Valid
		#Bottom XYZ Vel
		theDVL.bottom_xyz_vel_min = BottomXyzVelMin
		theDVL.bottom_xyz_vel_max = BottomXyzVelMax
		theDVL.bottom_xyz_vel_unit = BottomXyzVelUnit
		theDVL.bottom_xyz_vel_1_data = BottomXyzVel1Data
		theDVL.bottom_xyz_vel_2_data = BottomXyzVel2Data
		theDVL.bottom_xyz_vel_3_data = BottomXyzVel3Data
		theDVL.bottom_xyz_vel_4_data = BottomXyzVel4Data
		theDVL.bottom_xyz_vel_1_status = BottomXyzVel1Valid
		theDVL.bottom_xyz_vel_2_status = BottomXyzVel2Valid
		theDVL.bottom_xyz_vel_3_status = BottomXyzVel3Valid
		theDVL.bottom_xyz_vel_4_status = BottomXyzVel4Valid
		#Bottom XYZ FOM
		theDVL.bottom_xyz_fom_min = BottomXyzFomMin
		theDVL.bottom_xyz_fom_max = BottomXyzFomMax
		theDVL.bottom_xyz_fom_unit = BottomXyzFomUnit
		theDVL.bottom_xyz_fom_1_data = BottomXyzFom1Data
		theDVL.bottom_xyz_fom_2_data = BottomXyzFom2Data
		theDVL.bottom_xyz_fom_3_data = BottomXyzFom3Data
		theDVL.bottom_xyz_fom_4_data = BottomXyzFom4Data
		theDVL.bottom_xyz_fom_1_status = BottomXyzFom1Valid
		theDVL.bottom_xyz_fom_2_status = BottomXyzFom2Valid
		theDVL.bottom_xyz_fom_3_status = BottomXyzFom3Valid
		theDVL.bottom_xyz_fom_4_status = BottomXyzFom4Valid
		#
		#Water Beam Vel
		theDVL.water_vel_min = WaterBeamVelMin
		theDVL.water_vel_max = WaterBeamVelMax
		theDVL.water_vel_unit = WaterBeamVelUnit
		theDVL.water_vel_1_data = WaterBeamVel1Data
		theDVL.water_vel_2_data = WaterBeamVel2Data
		theDVL.water_vel_3_data = WaterBeamVel3Data
		theDVL.water_vel_4_data = WaterBeamVel4Data
		theDVL.water_vel_1_status = WaterBeamVel1Valid
		theDVL.water_vel_2_status = WaterBeamVel2Valid
		theDVL.water_vel_3_status = WaterBeamVel3Valid
		theDVL.water_vel_4_status = WaterBeamVel4Valid
		#Water Beam FOM
		theDVL.water_fom_min = WaterBeamFomMin
		theDVL.water_fom_max = WaterBeamFomMax
		theDVL.water_fom_unit = WaterBeamFomUnit
		theDVL.water_fom_1_data = WaterBeamFom1Data
		theDVL.water_fom_2_data = WaterBeamFom2Data
		theDVL.water_fom_3_data = WaterBeamFom3Data
		theDVL.water_fom_4_data = WaterBeamFom4Data
		theDVL.water_fom_1_status = WaterBeamFom1Valid
		theDVL.water_fom_2_status = WaterBeamFom2Valid
		theDVL.water_fom_3_status = WaterBeamFom3Valid
		theDVL.water_fom_4_status = WaterBeamFom4Valid
		#Water Beam Dist
		theDVL.water_dist_min = WaterBeamDistMin
		theDVL.water_dist_max = WaterBeamDistMax
		theDVL.water_dist_unit = WaterBeamDistUnit
		theDVL.water_dist_1_data = WaterBeamDist1Data
		theDVL.water_dist_2_data = WaterBeamDist2Data
		theDVL.water_dist_3_data = WaterBeamDist3Data
		theDVL.water_dist_4_data = WaterBeamDist4Data
		theDVL.water_dist_1_status = WaterBeamDist1Valid
		theDVL.water_dist_2_status = WaterBeamDist2Valid
		theDVL.water_dist_3_status = WaterBeamDist3Valid
		theDVL.water_dist_4_status = WaterBeamDist4Valid
		#Water XYZ Vel
		theDVL.water_xyz_vel_min = WaterXyzVelMin
		theDVL.water_xyz_vel_max = WaterXyzVelMax
		theDVL.water_xyz_vel_unit = WaterXyzVelUnit
		theDVL.water_xyz_vel_1_data = WaterXyzVel1Data
		theDVL.water_xyz_vel_2_data = WaterXyzVel2Data
		theDVL.water_xyz_vel_3_data = WaterXyzVel3Data
		theDVL.water_xyz_vel_4_data = WaterXyzVel4Data
		theDVL.water_xyz_vel_1_status = WaterXyzVel1Valid
		theDVL.water_xyz_vel_2_status = WaterXyzVel2Valid
		theDVL.water_xyz_vel_3_status = WaterXyzVel3Valid
		theDVL.water_xyz_vel_4_status = WaterXyzVel4Valid
		#Water XYZ FOM
		theDVL.water_xyz_fom_min = WaterXyzFomMin
		theDVL.water_xyz_fom_max = WaterXyzFomMax
		theDVL.water_xyz_fom_unit = WaterXyzFomUnit
		theDVL.water_xyz_fom_1_data = WaterXyzFom1Data
		theDVL.water_xyz_fom_2_data = WaterXyzFom2Data
		theDVL.water_xyz_fom_3_data = WaterXyzFom3Data
		theDVL.water_xyz_fom_4_data = WaterXyzFom4Data
		theDVL.water_xyz_fom_1_status = WaterXyzFom1Valid
		theDVL.water_xyz_fom_2_status = WaterXyzFom2Valid
		theDVL.water_xyz_fom_3_status = WaterXyzFom3Valid
		theDVL.water_xyz_fom_4_status = WaterXyzFom4Valid
		
        rospy.loginfo("Publishing sensor data from DVL %s" % rospy.get_time())
        pub.publish(theDVL)
        rate.sleep()

if __name__ == '__main__':
    try:
        publishDVLdata()
    except rospy.ROSInterruptException:
        pass
