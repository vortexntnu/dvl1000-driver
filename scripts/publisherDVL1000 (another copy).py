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


def publishDVLdata():
	global ws
	result = ws.recv()

	#Get JSON Data
	theData = json.loads(result)

	#Alternating data each time data is recieved. Each with unique ID(4123 and 8219) and dataset. Using IF to sort this out
	#8219 seem to have additional information like Status, Speed of Sound, Temperature
	#More data if more modes of tracking is turned on. IDs 4125 and 8221 belongs to Water Tracking mode.
	#IDs 4123 and 8219 belongs to Bottom Track mode.

	pubBottom = rospy.Publisher('sensors/dvl/bottom', NORTEK, queue_size=10)
	pubWater = rospy.Publisher('sensors/dvl/water', NORTEK, queue_size=10)
	rospy.init_node('DVL1000', anonymous=True)
	rate = rospy.Rate(1000) # 1000hz

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
		#Bottom Beam Vel
		theDVL.vel_min = BottomBeamVelMin
		theDVL.vel_max = BottomBeamVelMax
		theDVL.vel_unit = BottomBeamVelUnit
		theDVL.vel_1_data = BottomBeamVel1Data
		theDVL.vel_2_data = BottomBeamVel2Data
		theDVL.vel_3_data = BottomBeamVel3Data
		theDVL.vel_4_data = BottomBeamVel4Data
		theDVL.vel_1_status = BottomBeamVel1Valid
		theDVL.vel_2_status = BottomBeamVel2Valid
		theDVL.vel_3_status = BottomBeamVel3Valid
		theDVL.vel_4_status = BottomBeamVel4Valid
		#Bottom Beam FOM
		theDVL.fom_min = BottomBeamFomMin
		theDVL.fom_max = BottomBeamFomMax
		theDVL.fom_unit = BottomBeamFomUnit
		theDVL.fom_1_data = BottomBeamFom1Data
		theDVL.fom_2_data = BottomBeamFom2Data
		theDVL.fom_3_data = BottomBeamFom3Data
		theDVL.fom_4_data = BottomBeamFom4Data
		theDVL.fom_1_status = BottomBeamFom1Valid
		theDVL.fom_2_status = BottomBeamFom2Valid
		theDVL.fom_3_status = BottomBeamFom3Valid
		theDVL.fom_4_status = BottomBeamFom4Valid
		#Bottom Beam Dist
		theDVL.dist_min = BottomBeamDistMin
		theDVL.dist_max = BottomBeamDistMax
		theDVL.dist_unit = BottomBeamDistUnit
		theDVL.dist_1_data = BottomBeamDist1Data
		theDVL.dist_2_data = BottomBeamDist2Data
		theDVL.dist_3_data = BottomBeamDist3Data
		theDVL.dist_4_data = BottomBeamDist4Data
		theDVL.dist_2_status = BottomBeamDist2Valid
		theDVL.dist_3_status = BottomBeamDist3Valid
		theDVL.dist_4_status = BottomBeamDist4Valid
		#Bottom XYZ Vel
		theDVL.xyz_vel_min = BottomXyzVelMin
		theDVL.xyz_vel_max = BottomXyzVelMax
		theDVL.xyz_vel_unit = BottomXyzVelUnit
		theDVL.xyz_vel_1_data = BottomXyzVel1Data
		theDVL.xyz_vel_2_data = BottomXyzVel2Data
		theDVL.xyz_vel_3_data = BottomXyzVel3Data
		theDVL.xyz_vel_4_data = BottomXyzVel4Data
		theDVL.xyz_vel_1_status = BottomXyzVel1Valid
		theDVL.xyz_vel_2_status = BottomXyzVel2Valid
		theDVL.xyz_vel_3_status = BottomXyzVel3Valid
		theDVL.xyz_vel_4_status = BottomXyzVel4Valid
		#Bottom XYZ FOM
		theDVL.xyz_fom_min = BottomXyzFomMin
		theDVL.xyz_fom_max = BottomXyzFomMax
		theDVL.xyz_fom_unit = BottomXyzFomUnit
		theDVL.xyz_fom_1_data = BottomXyzFom1Data
		theDVL.xyz_fom_2_data = BottomXyzFom2Data
		theDVL.xyz_fom_3_data = BottomXyzFom3Data
		theDVL.xyz_fom_4_data = BottomXyzFom4Data
		theDVL.xyz_fom_1_status = BottomXyzFom1Valid
		theDVL.xyz_fom_2_status = BottomXyzFom2Valid
		theDVL.xyz_fom_3_status = BottomXyzFom3Valid
		theDVL.xyz_fom_4_status = BottomXyzFom4Valid
		#pub = rospy.Publisher('sensors/dvl/bottom', NORTEK, queue_size=10)
		rospy.loginfo("Publishing sensor data from DVL Bottom-Track %s" % rospy.get_time())
        pubBottom.publish(theDVL)
		
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
		#Water Beam Vel
		theDVL.vel_min = WaterBeamVelMin
		theDVL.vel_max = WaterBeamVelMax
		theDVL.vel_unit = WaterBeamVelUnit
		theDVL.vel_1_data = WaterBeamVel1Data
		theDVL.vel_2_data = WaterBeamVel2Data
		theDVL.vel_3_data = WaterBeamVel3Data
		theDVL.vel_4_data = WaterBeamVel4Data
		theDVL.vel_1_status = WaterBeamVel1Valid
		theDVL.vel_2_status = WaterBeamVel2Valid
		theDVL.vel_3_status = WaterBeamVel3Valid
		theDVL.vel_4_status = WaterBeamVel4Valid
		#Water Beam FOM
		theDVL.fom_min = WaterBeamFomMin
		theDVL.fom_max = WaterBeamFomMax
		theDVL.fom_unit = WaterBeamFomUnit
		theDVL.fom_1_data = WaterBeamFom1Data
		theDVL.fom_2_data = WaterBeamFom2Data
		theDVL.fom_3_data = WaterBeamFom3Data
		theDVL.fom_4_data = WaterBeamFom4Data
		theDVL.fom_1_status = WaterBeamFom1Valid
		theDVL.fom_2_status = WaterBeamFom2Valid
		theDVL.fom_3_status = WaterBeamFom3Valid
		theDVL.fom_4_status = WaterBeamFom4Valid
		#Water Beam Dist
		theDVL.dist_min = WaterBeamDistMin
		theDVL.dist_max = WaterBeamDistMax
		theDVL.dist_unit = WaterBeamDistUnit
		theDVL.dist_1_data = WaterBeamDist1Data
		theDVL.dist_2_data = WaterBeamDist2Data
		theDVL.dist_3_data = WaterBeamDist3Data
		theDVL.dist_4_data = WaterBeamDist4Data
		theDVL.dist_1_status = WaterBeamDist1Valid
		theDVL.dist_2_status = WaterBeamDist2Valid
		theDVL.dist_3_status = WaterBeamDist3Valid
		theDVL.dist_4_status = WaterBeamDist4Valid
		#Water XYZ Vel
		theDVL.xyz_vel_min = WaterXyzVelMin
		theDVL.xyz_vel_max = WaterXyzVelMax
		theDVL.xyz_vel_unit = WaterXyzVelUnit
		theDVL.xyz_vel_1_data = WaterXyzVel1Data
		theDVL.xyz_vel_2_data = WaterXyzVel2Data
		theDVL.xyz_vel_3_data = WaterXyzVel3Data
		theDVL.xyz_vel_4_data = WaterXyzVel4Data
		theDVL.xyz_vel_1_status = WaterXyzVel1Valid
		theDVL.xyz_vel_2_status = WaterXyzVel2Valid
		theDVL.xyz_vel_3_status = WaterXyzVel3Valid
		theDVL.xyz_vel_4_status = WaterXyzVel4Valid
		#Water XYZ FOM
		theDVL.xyz_fom_min = WaterXyzFomMin
		theDVL.xyz_fom_max = WaterXyzFomMax
		theDVL.xyz_fom_unit = WaterXyzFomUnit
		theDVL.xyz_fom_1_data = WaterXyzFom1Data
		theDVL.xyz_fom_2_data = WaterXyzFom2Data
		theDVL.xyz_fom_3_data = WaterXyzFom3Data
		theDVL.xyz_fom_4_data = WaterXyzFom4Data
		theDVL.xyz_fom_1_status = WaterXyzFom1Valid
		theDVL.xyz_fom_2_status = WaterXyzFom2Valid
		theDVL.xyz_fom_3_status = WaterXyzFom3Valid
		theDVL.xyz_fom_4_status = WaterXyzFom4Valid
		#pub = rospy.Publisher('sensors/dvl/water', NORTEK, queue_size=10)
		rospy.loginfo("Publishing sensor data from DVL Water-Track %s" % rospy.get_time())
        pubWater.publish(theDVL)
	
    
	#while not rospy.is_shutdown():
		#rospy.loginfo("Publishing sensor data from DVL %s" % rospy.get_time())
		#pub.publish(theDVL)
		#rate.sleep()
	rate.sleep()
if __name__ == '__main__':
	try:
		while not rospy.is_shutdown():
			publishDVLdata()
			#time.sleep(1)
	except rospy.ROSInterruptException:
		pass
        
ws.close
