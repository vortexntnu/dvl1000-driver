#!/usr/bin/python3
import rospy
import json
import time
from websocket import create_connection
from std_msgs.msg import String
from dvl1000_ros.msg import DVL
from dvl1000_ros.msg import DVLBeam
from nav_msgs.msg import Odometry
from sensor_msgs.msg import FluidPressure

unknown = 0 # The arbitrary value set for unknown values throughout the file

class DVL1000:

	def __init__(self, rate=1):
		
		websocket_address = rospy.get_param("/dvl/websocket_address", "ws://192.168.0.96:10100")
		self.ws = create_connection(websocket_address, subprotocols=["data-transfer-nortek"])
		print("Connecting to DVL1000 via websocket...")
		result = self.ws.recv()
		print("Status: '%s'" % result)

		self.rate = rate
		self.data_id = 8219
		self.json_dvl_data_backup = ''
  
		self.bottom_track_pub = rospy.Publisher('/auv/dvl_twist', DVL, queue_size=10)
		self.odometry_pub = rospy.Publisher('/auv/odom', Odometry, queue_size=10)
		self.pressure_pub = rospy.Publisher('/auv/Pressure', FluidPressure, queue_size=10)

	def __del__(self):
		self.ws.close()


	def cycleDVL(self):
		return_data = ''
		for n in range(5):
			result = self.ws.recv()
			dvl_data = json.loads(result)
			if dvl_data["id"] == self.data_id:
				return_data = result
		return return_data


	def spin(self):

		#Get JSON Data
		json_dvl_data = self.cycleDVL()
		try:
			dvl_data = json.loads(json_dvl_data)
		except:
			dvl_data = json.loads(self.json_dvl_data_backup)

		#Alternating data each time data is recieved. Each with unique ID(4123 and 8219) and dataset. Using IF to sort this out
		#8219 seem to have additional information like Status, Speed of Sound, Temperature
		#More data if more modes of tracking is turned on. IDs 4125 and 8221 belongs to Water Tracking mode.
		#IDs 4123 and 8219 belongs to Bottom Track mode.
		#pubWater = rospy.Publisher('sensors/dvl/water', DVL, queue_size=10)

		dvl_msg = DVL()
		odometry_msg = Odometry()
		pressure_msg = FluidPressure()

		#Bottom-Trackingnumpy square
		if dvl_data["id"] == self.data_id:
			#Parsing Variables
			BottomID = dvl_data["id"]
			BottomMode = dvl_data["name"]
			BottomTime = dvl_data["timeStampStr"]
			BottomStatus = dvl_data["frames"][1]["inputs"][0]["lines"][0]["data"][0]
			
			#Speed of Sound variables
			BottomSpeedOfSoundMin = dvl_data["frames"][2]["inputs"][0]["min"]
			BottomSpeedOfSoundMax = dvl_data["frames"][2]["inputs"][0]["max"]
			BottomSpeedOfSoundUnit = dvl_data["frames"][2]["inputs"][0]["units"]
			BottomSpeedOfSoundData = dvl_data["frames"][2]["inputs"][0]["lines"][0]["data"][0]
			
			#Temperature varliables
			BottomTempMin = dvl_data["frames"][3]["inputs"][0]["min"]
			BottomTempMax = dvl_data["frames"][3]["inputs"][0]["max"]
			BottomTempUnit = dvl_data["frames"][3]["inputs"][0]["units"]
			BottomTempData = dvl_data["frames"][3]["inputs"][0]["lines"][0]["data"][0]
			
			#Pressure variables
			BottomPressureName = dvl_data["frames"][4]["inputs"][0]["name"]
			BottomPressureMin = dvl_data["frames"][4]["inputs"][0]["min"]
			BottomPressureMax = dvl_data["frames"][4]["inputs"][0]["max"]
			BottomPressureData = dvl_data["frames"][4]["inputs"][0]["lines"][0]["data"][0]
			BottomPressureUnit = dvl_data["frames"][4]["inputs"][0]["units"]
			
			#Beam Velocity Variables
			BottomBeamVelMin = dvl_data["frames"][5]["inputs"][0]["min"]
			BottomBeamVelMax = dvl_data["frames"][5]["inputs"][0]["max"]
			BottomBeamVelUnit = dvl_data["frames"][5]["inputs"][0]["units"]
			BottomBeamVel1Data = dvl_data["frames"][5]["inputs"][0]["lines"][0]["data"][0]
			BottomBeamVel2Data = dvl_data["frames"][5]["inputs"][0]["lines"][1]["data"][0]
			BottomBeamVel3Data = dvl_data["frames"][5]["inputs"][0]["lines"][2]["data"][0]
			BottomBeamVel4Data = dvl_data["frames"][5]["inputs"][0]["lines"][3]["data"][0]
			BottomBeamVel1Valid = dvl_data["frames"][5]["inputs"][0]["lines"][0]["valid"]
			BottomBeamVel2Valid = dvl_data["frames"][5]["inputs"][0]["lines"][1]["valid"]
			BottomBeamVel3Valid = dvl_data["frames"][5]["inputs"][0]["lines"][2]["valid"]
			BottomBeamVel4Valid = dvl_data["frames"][5]["inputs"][0]["lines"][3]["valid"]
			
			#Beam FOM Variables
			BottomBeamFomMin = dvl_data["frames"][5]["inputs"][1]["min"]
			BottomBeamFomMax = dvl_data["frames"][5]["inputs"][1]["max"]
			BottomBeamFomUnit = dvl_data["frames"][5]["inputs"][1]["units"]
			BottomBeamFom1Data = dvl_data["frames"][5]["inputs"][1]["lines"][0]["data"][0]
			BottomBeamFom2Data = dvl_data["frames"][5]["inputs"][1]["lines"][1]["data"][0]
			BottomBeamFom3Data = dvl_data["frames"][5]["inputs"][1]["lines"][2]["data"][0]
			BottomBeamFom4Data = dvl_data["frames"][5]["inputs"][1]["lines"][3]["data"][0]
			BottomBeamFom1Valid = dvl_data["frames"][5]["inputs"][1]["lines"][0]["valid"]
			BottomBeamFom2Valid = dvl_data["frames"][5]["inputs"][1]["lines"][1]["valid"]
			BottomBeamFom3Valid = dvl_data["frames"][5]["inputs"][1]["lines"][2]["valid"]
			BottomBeamFom4Valid = dvl_data["frames"][5]["inputs"][1]["lines"][3]["valid"]
			
			#Beam Dist Variables
			BottomBeamDistMin = dvl_data["frames"][5]["inputs"][2]["min"]
			BottomBeamDistMax = dvl_data["frames"][5]["inputs"][2]["max"]
			BottomBeamDistUnit = dvl_data["frames"][5]["inputs"][2]["units"]
			BottomBeamDist1Data = dvl_data["frames"][5]["inputs"][2]["lines"][0]["data"][0]
			BottomBeamDist2Data = dvl_data["frames"][5]["inputs"][2]["lines"][1]["data"][0]
			BottomBeamDist3Data = dvl_data["frames"][5]["inputs"][2]["lines"][2]["data"][0]
			BottomBeamDist4Data = dvl_data["frames"][5]["inputs"][2]["lines"][3]["data"][0]
			BottomBeamDist1Valid = dvl_data["frames"][5]["inputs"][2]["lines"][0]["valid"]
			BottomBeamDist2Valid = dvl_data["frames"][5]["inputs"][2]["lines"][1]["valid"]
			BottomBeamDist3Valid = dvl_data["frames"][5]["inputs"][2]["lines"][2]["valid"]
			BottomBeamDist4Valid = dvl_data["frames"][5]["inputs"][2]["lines"][3]["valid"]
			
			#XYZ Velocity Variables
			BottomXyzVelMin = dvl_data["frames"][6]["inputs"][0]["min"]
			BottomXyzVelMax = dvl_data["frames"][6]["inputs"][0]["max"]
			BottomXyzVelUnit = dvl_data["frames"][6]["inputs"][0]["units"]
			BottomXyzVel1Data = dvl_data["frames"][6]["inputs"][0]["lines"][0]["data"][0]
			BottomXyzVel2Data = dvl_data["frames"][6]["inputs"][0]["lines"][1]["data"][0]
			BottomXyzVel3Data = dvl_data["frames"][6]["inputs"][0]["lines"][2]["data"][0]
			BottomXyzVel4Data = dvl_data["frames"][6]["inputs"][0]["lines"][3]["data"][0]
			BottomXyzVel1Valid = dvl_data["frames"][6]["inputs"][0]["lines"][0]["valid"]
			BottomXyzVel2Valid = dvl_data["frames"][6]["inputs"][0]["lines"][1]["valid"]
			BottomXyzVel3Valid = dvl_data["frames"][6]["inputs"][0]["lines"][2]["valid"]
			BottomXyzVel4Valid = dvl_data["frames"][6]["inputs"][0]["lines"][3]["valid"]
			
			#XYZ FOM Variables
			BottomXyzFomMin = dvl_data["frames"][6]["inputs"][1]["min"]
			BottomXyzFomMax = dvl_data["frames"][6]["inputs"][1]["max"]
			BottomXyzFomUnit = dvl_data["frames"][6]["inputs"][1]["units"]
			BottomXyzFom1Data = dvl_data["frames"][6]["inputs"][1]["lines"][0]["data"][0]
			BottomXyzFom2Data = dvl_data["frames"][6]["inputs"][1]["lines"][1]["data"][0]
			BottomXyzFom3Data = dvl_data["frames"][6]["inputs"][1]["lines"][2]["data"][0]
			BottomXyzFom4Data = dvl_data["frames"][6]["inputs"][1]["lines"][3]["data"][0]
			BottomXyzFom1Valid = dvl_data["frames"][6]["inputs"][1]["lines"][0]["valid"]
			BottomXyzFom2Valid = dvl_data["frames"][6]["inputs"][1]["lines"][1]["valid"]
			BottomXyzFom3Valid = dvl_data["frames"][6]["inputs"][1]["lines"][2]["valid"]
			BottomXyzFom4Valid = dvl_data["frames"][6]["inputs"][1]["lines"][3]["valid"]
			
			dvl_msg.header.stamp = rospy.Time.now()
			dvl_msg.header.frame_id = "dvl_link"
			dvl_msg.velocity.x = BottomXyzVel1Data
			dvl_msg.velocity.y = BottomXyzVel2Data
			if BottomXyzFom3Data > BottomXyzFom4Data:
				dvl_msg.velocity.z = BottomXyzVel4Data
				BottomXyzFomZbest = BottomXyzFom4Data
			else:
				dvl_msg.velocity.z = BottomXyzVel3Data
				BottomXyzFomZbest = BottomXyzFom3Data
				
			#Doing covariances
			velocity_covariance = [BottomXyzFom1Data * BottomXyzFom1Data, unknown, unknown, unknown, BottomXyzFom2Data * BottomXyzFom2Data, unknown, unknown, unknown, BottomXyzFomZbest * BottomXyzFomZbest]
			
			#Feeding message
			dvl_msg.velocity_covariance = velocity_covariance
			dvl_msg.altitude = ((BottomBeamDist1Data + BottomBeamDist2Data + BottomBeamDist3Data + BottomBeamDist4Data) / 4)
			
			#Individual beam data
			beam1 = DVLBeam()
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
			
			beam2 = DVLBeam()
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
			
			beam3 = DVLBeam()
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
			
			beam4 = DVLBeam()
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
			
			dvl_msg.beams = [beam1, beam2, beam3, beam4]
				
			#Check page 49 for Z1 and Z2 and use FOM to evaluate each, 
			#Covariance is a matrix with variance in the diagonal fields
			#[var(x), cov(x,y) cov(x,z)|cov(x,y), var(y), cov(y,z)|cov(x,z), cov(y,z), var(z)]
			#Concluded with using FOB*FOB as an estimate for variance
			#Try to get Variance out of FOM

			rospy.loginfo("Publishing sensor data from DVL Bottom-Track %s" % rospy.get_time())
			self.bottom_track_pub.publish(dvl_msg)
			self.json_dvl_data_backup = json_dvl_data
			
			#Odometry topic
			odometry_msg.header.stamp = rospy.Time.now()
			odometry_msg.header.frame_id = "dvl_link"
			odometry_msg.child_frame_id = "dvl_link"
			if abs(dvl_msg.velocity.x) < 10 and abs(dvl_msg.velocity.y) < 10 and abs(dvl_msg.velocity.z) < 10:
				odometry_msg.twist.twist.linear.x = dvl_msg.velocity.x
				odometry_msg.twist.twist.linear.y = dvl_msg.velocity.y
				odometry_msg.twist.twist.linear.z = dvl_msg.velocity.z
			else:
				rospy.logdebug(
					"Invalid DVL velocity data x: %f, y: %f, z: %f" % (dvl_msg.velocity.x, dvl_msg.velocity.y, dvl_msg.velocity.z)
				)
			
			odometry_msg.twist.twist.angular.x = unknown
			odometry_msg.twist.twist.angular.y = unknown	
			odometry_msg.twist.twist.angular.z = unknown

			odometry_msg.pose.pose.position.z=-((BottomPressureData*10000)*10)/(997*9.81)
				
			if (BottomXyzFom1Data != 10) and (BottomXyzFom2Data != 10) and (BottomXyzFomZbest != 10):
				odometry_msg.twist.covariance = [BottomXyzFom1Data * BottomXyzFom1Data, unknown, unknown, unknown, unknown, unknown, unknown, BottomXyzFom2Data * BottomXyzFom2Data, unknown, unknown, unknown, unknown, unknown, unknown, BottomXyzFomZbest * BottomXyzFomZbest, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown]
			self.odometry_pub.publish(odometry_msg)


			
			#Pressure topic
			pressure_msg.header.stamp = rospy.Time.now()
			pressure_msg.header.frame_id = "dvl_link"
			pressure_msg.fluid_pressure = BottomPressureData * 10000 #Convert dbar to Pascal
			pressure_msg.variance = 30*30 #Should do a more accurate meassurement of the variance
			self.pressure_pub.publish(pressure_msg)
		
		self.rate.sleep()


if __name__ == '__main__':
	try:
		rospy.init_node('DVL1000', anonymous=False)
		rate = rospy.Rate(8) # 8hz

		dvl = DVL1000(rate)

		while not rospy.is_shutdown():
			dvl.spin()
			
	except rospy.ROSInterruptException:
		pass
