#!/usr/bin/env python
import rospy
import json
import time
from websocket import create_connection
from std_msgs.msg import String
from dvl1000_ros.msg import DVL
from dvl1000_ros.msg import DVLBeam
from nav_msgs.msg import Odometry
from sensor_msgs.msg import FluidPressure

#Websocket connection stuff. Make sure the websocket-client python library is installed.
ws = create_connection("ws://10.42.0.96:10100", subprotocols=["data-transfer-nortek"])
print("Connecting to DVL1000 via websocket...")
result = ws.recv()
print("Status: '%s'" % result)

backupjson = ''
unknown = -1 #The arbitrary value set for unknown values throughout the file

def cycleDVL():
	global ws
	retVal = ''
	for n in range(5):
		result = ws.recv()
		theData = json.loads(result)
		if theData["id"] == 8219:
			retVal = result
	return retVal



def publishDVLdata():
	global unknown
	global backupjson
	#Get JSON Data
	getJson = cycleDVL()
	try:
		theData = json.loads(getJson)
	except:
		theData = json.loads(backupjson)

	#Alternating data each time data is recieved. Each with unique ID(4123 and 8219) and dataset. Using IF to sort this out
	#8219 seem to have additional information like Status, Speed of Sound, Temperature
	#More data if more modes of tracking is turned on. IDs 4125 and 8221 belongs to Water Tracking mode.
	#IDs 4123 and 8219 belongs to Bottom Track mode.

	pubBottom = rospy.Publisher('manta/dvl_twist', DVL, queue_size=10)
	pubOdo = rospy.Publisher('/manta/odom', Odometry, queue_size=10)
	pubPressure = rospy.Publisher('manta/Pressure', FluidPressure, queue_size=10)
	#pubWater = rospy.Publisher('sensors/dvl/water', DVL, queue_size=10)
	rospy.init_node('DVL1000', anonymous=False)
	rate = rospy.Rate(8) # 8hz

	theDVL = DVL()
	theDVLBeam = DVLBeam()
	theOdo = Odometry()
	thePressure = FluidPressure()

	#Bottom-Trackingnumpy square
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
		
		theDVL.header.stamp = rospy.Time.now()
		theDVL.header.frame_id = "dvl_link"
		theDVL.velocity.x = BottomXyzVel1Data
		theDVL.velocity.y = BottomXyzVel2Data
		if BottomXyzFom3Data > BottomXyzFom4Data:
			theDVL.velocity.z = BottomXyzVel4Data
			BottomXyzFomZbest = BottomXyzFom4Data
		else:
			theDVL.velocity.z = BottomXyzVel3Data
			BottomXyzFomZbest = BottomXyzFom3Data
			
		#Doing covariances
		velocity_covariance = [BottomXyzFom1Data * BottomXyzFom1Data, unknown, unknown, unknown, BottomXyzFom2Data * BottomXyzFom2Data, unknown, unknown, unknown, BottomXyzFomZbest * BottomXyzFomZbest]
		
		#Feeding message
		theDVL.velocity_covariance = velocity_covariance
		theDVL.altitude = ((BottomBeamDist1Data + BottomBeamDist2Data + BottomBeamDist3Data + BottomBeamDist4Data) / 4)
		
		#Individual beam data
		beam1 = theDVLBeam
		beam1.range = BottomBeamDist1Data
		beam1.range_covariance = 0.0001 #TODO: find accurate value
		beam1.velocity = BottomBeamVel1Data
		beam1.velocity_covariance = BottomBeamFom1Data * BottomBeamFom1Data
		beam1.pose.header.stamp = rospy.Time.now()
		beam1.pose.pose.position.x = 0.283
		beam1.pose.pose.position.y = 0.283
		beam1.pose.pose.position.z = 0
		beam1.pose.pose.orientation.x = 0.211 #Estimating values here. 25 degree tilt and 4 cm from origo
		beam1.pose.pose.orientation.y = 0.211
		beam1.pose.pose.orientation.z = -0.047
		beam1.pose.pose.orientation.w = 0.953
		
		beam2 = theDVLBeam
		beam2.range = BottomBeamDist2Data
		beam2.range_covariance = 0.0001 #TODO: find accurate value
		beam2.velocity = BottomBeamVel2Data
		beam2.velocity_covariance = BottomBeamFom2Data * BottomBeamFom2Data
		beam2.pose.header.stamp = rospy.Time.now()
		beam2.pose.pose.position.x = 0.283
		beam2.pose.pose.position.y = -0.283
		beam2.pose.pose.position.z = 0
		beam2.pose.pose.orientation.x = -0.211 #Estimating values here. 25 degree tilt and 4 cm from origo
		beam2.pose.pose.orientation.y = -0.211
		beam2.pose.pose.orientation.z = 0.047
		beam2.pose.pose.orientation.w = -0.953
		
		beam3 = theDVLBeam
		beam3.range = BottomBeamDist3Data
		beam3.range_covariance = 0.0001 #TODO: find accurate value
		beam3.velocity = BottomBeamVel3Data
		beam3.velocity_covariance = BottomBeamFom3Data * BottomBeamFom3Data
		beam3.pose.header.stamp = rospy.Time.now()
		beam3.pose.pose.position.x = -0.283
		beam3.pose.pose.position.y = -0.283
		beam3.pose.pose.position.z = 0
		beam3.pose.pose.orientation.x = -0.299 #Estimating values here. 25 degree tilt and 4 cm from origo
		beam3.pose.pose.orientation.y = 0
		beam3.pose.pose.orientation.z = 0.707
		beam3.pose.pose.orientation.w = -0.641
		
		beam4 = theDVLBeam
		beam4.range = BottomBeamDist4Data
		beam4.range_covariance = 0.0001 #TODO: find accurate value
		beam4.velocity = BottomBeamVel4Data
		beam4.velocity_covariance = BottomBeamFom4Data * BottomBeamFom4Data
		beam4.pose.header.stamp = rospy.Time.now()
		beam4.pose.pose.position.x = -0.283
		beam4.pose.pose.position.y = 0.283
		beam4.pose.pose.position.z = 0
		beam4.pose.pose.orientation.x = 0 #Estimating values here. 25 degree tilt and 4 cm from origo
		beam4.pose.pose.orientation.y = 0.299
		beam4.pose.pose.orientation.z = 0.641
		beam4.pose.pose.orientation.w = 0.707
		
		theDVL.beams = [beam1, beam2, beam3, beam4]
			
		#Check page 49 for Z1 and Z2 and use FOM to evaluate each, 
		#Covariance is a matrix with variance in the diagonal fields
		#[var(x), cov(x,y) cov(x,z)|cov(x,y), var(y), cov(y,z)|cov(x,z), cov(y,z), var(z)]
		#Concluded with using FOB*FOB as an estimate for variance
		#Try to get Variance out of FOM
		
		#pub = rospy.Publisher('sensors/dvl/bottom', NORTEK, queue_size=10)
		rospy.loginfo("Publishing sensor data from DVL Bottom-Track %s" % rospy.get_time())
        pubBottom.publish(theDVL)
        backupjson = getJson
        
        #Odometry topic
        theOdo.header.stamp = rospy.Time.now()
        theOdo.header.frame_id = "dvl_link"
        theOdo.child_frame_id = "dvl_link"
        theOdo.twist.twist.linear.x = theDVL.velocity.x
        theOdo.twist.twist.linear.y = theDVL.velocity.y
        theOdo.twist.twist.linear.z = theDVL.velocity.z
        theOdo.twist.twist.angular.x = unknown
        theOdo.twist.twist.angular.y = unknown
        theOdo.twist.twist.angular.z = unknown
	theOdo.pose.pose.position.z=-((BottomPressuseData*10000)-101325)/(997*9.81)
        theOdo.twist.covariance = [BottomXyzFom1Data * BottomXyzFom1Data, unknown, unknown, unknown, unknown, unknown, unknown, BottomXyzFom2Data * BottomXyzFom2Data, unknown, unknown, unknown, unknown, unknown, unknown, BottomXyzFomZbest * BottomXyzFomZbest, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown]
        pubOdo.publish(theOdo)


        
        #Pressure topic
        thePressure.header.stamp = rospy.Time.now()
        thePressure.header.frame_id = "dvl_link"
        thePressure.fluid_pressure = BottomPressureData * 10000 #Convert dbar to Pascal
        thePressure.variance = 30*30 #Should do a more accurate meassurement of the variance
        pubPressure.publish(thePressure)
	
    
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
