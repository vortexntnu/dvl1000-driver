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
			# Pressure measurements
			measured_pressure = dvl_data["frames"][4]["inputs"][0]["lines"][0]["data"][0]
			
			# Beam velocities and FOM (Figure Of Merit)
			measured_velocity_beam1 = dvl_data["frames"][5]["inputs"][0]["lines"][0]["data"][0]
			measured_velocity_beam2 = dvl_data["frames"][5]["inputs"][0]["lines"][1]["data"][0]
			measured_velocity_beam3 = dvl_data["frames"][5]["inputs"][0]["lines"][2]["data"][0]
			measured_velocity_beam4 = dvl_data["frames"][5]["inputs"][0]["lines"][3]["data"][0]
			fom_beam1 = dvl_data["frames"][5]["inputs"][1]["lines"][0]["data"][0]
			fom_beam2 = dvl_data["frames"][5]["inputs"][1]["lines"][1]["data"][0]
			fom_beam3 = dvl_data["frames"][5]["inputs"][1]["lines"][2]["data"][0]
			fom_beam4 = dvl_data["frames"][5]["inputs"][1]["lines"][3]["data"][0]

			# Beam altitude measurements and FOM (Figure Of Merit)
			measured_altitude_beam1 = dvl_data["frames"][5]["inputs"][2]["lines"][0]["data"][0]
			measured_altitude_beam2 = dvl_data["frames"][5]["inputs"][2]["lines"][1]["data"][0]
			measured_altitude_beam3 = dvl_data["frames"][5]["inputs"][2]["lines"][2]["data"][0]
			measured_altitude_beam4 = dvl_data["frames"][5]["inputs"][2]["lines"][3]["data"][0]

			# Velocity measurements and FOM in XYZ. Note the two measurements in Z
			measured_velocity_x = dvl_data["frames"][6]["inputs"][0]["lines"][0]["data"][0]
			measured_velocity_y = dvl_data["frames"][6]["inputs"][0]["lines"][1]["data"][0]
			measured_velocity_z_1 = dvl_data["frames"][6]["inputs"][0]["lines"][2]["data"][0]
			measured_velocity_z_2 = dvl_data["frames"][6]["inputs"][0]["lines"][3]["data"][0]
			fom_velocity_x = dvl_data["frames"][6]["inputs"][1]["lines"][0]["data"][0]
			fom_velocity_y = dvl_data["frames"][6]["inputs"][1]["lines"][1]["data"][0]
			fom_velocity_z_1 = dvl_data["frames"][6]["inputs"][1]["lines"][2]["data"][0]
			fom_velocity_z_2 = dvl_data["frames"][6]["inputs"][1]["lines"][3]["data"][0]
			
			# Fill in fields in DVL twist message
			dvl_msg.header.stamp = rospy.Time.now()
			dvl_msg.header.frame_id = "dvl_link"
			dvl_msg.velocity.x = measured_velocity_x
			dvl_msg.velocity.y = measured_velocity_y

			# Since we have two measurements for velocity in z, choose the best one
			if fom_velocity_z_1 > fom_velocity_z_2:
				dvl_msg.velocity.z = measured_velocity_z_2
				fom_velocity_z_best = fom_velocity_z_2
			else:
				dvl_msg.velocity.z = measured_velocity_z_1
				fom_velocity_z_best = fom_velocity_z_1
				
			velocity_covariance = [fom_velocity_x * fom_velocity_x, unknown, unknown, unknown, fom_velocity_y * fom_velocity_y, unknown, unknown, unknown, fom_velocity_z_best * fom_velocity_z_best]
			dvl_msg.velocity_covariance = velocity_covariance
			dvl_msg.altitude = ((measured_altitude_beam1 + measured_altitude_beam2 + measured_altitude_beam3 + measured_altitude_beam4) / 4)
			
			#Individual beam data
			beam1 = DVLBeam()
			beam1.range = measured_altitude_beam1
			beam1.range_covariance = 0.0001 #TODO: find accurate value
			beam1.velocity = measured_velocity_beam1
			beam1.velocity_covariance = fom_beam1 * fom_beam1
			beam1.pose.header.stamp = rospy.Time.now()
			beam1.pose.pose.position.x = 0.283
			beam1.pose.pose.position.y = 0.283
			beam1.pose.pose.position.z = 0
			beam1.pose.pose.orientation.x = 0.211 #Estimating values here. 25 degree tilt and 4 cm from origo
			beam1.pose.pose.orientation.y = 0.211
			beam1.pose.pose.orientation.z = -0.047
			beam1.pose.pose.orientation.w = 0.953
			
			beam2 = DVLBeam()
			beam2.range = measured_altitude_beam2
			beam2.range_covariance = 0.0001 #TODO: find accurate value
			beam2.velocity = measured_velocity_beam2
			beam2.velocity_covariance = fom_beam2 * fom_beam2
			beam2.pose.header.stamp = rospy.Time.now()
			beam2.pose.pose.position.x = 0.283
			beam2.pose.pose.position.y = -0.283
			beam2.pose.pose.position.z = 0
			beam2.pose.pose.orientation.x = -0.211 #Estimating values here. 25 degree tilt and 4 cm from origo
			beam2.pose.pose.orientation.y = -0.211
			beam2.pose.pose.orientation.z = 0.047
			beam2.pose.pose.orientation.w = -0.953
			
			beam3 = DVLBeam()
			beam3.range = measured_altitude_beam3
			beam3.range_covariance = 0.0001 #TODO: find accurate value
			beam3.velocity = measured_velocity_beam3
			beam3.velocity_covariance = fom_beam3 * fom_beam3
			beam3.pose.header.stamp = rospy.Time.now()
			beam3.pose.pose.position.x = -0.283
			beam3.pose.pose.position.y = -0.283
			beam3.pose.pose.position.z = 0
			beam3.pose.pose.orientation.x = -0.299 #Estimating values here. 25 degree tilt and 4 cm from origo
			beam3.pose.pose.orientation.y = 0
			beam3.pose.pose.orientation.z = 0.707
			beam3.pose.pose.orientation.w = -0.641
			
			beam4 = DVLBeam()
			beam4.range = measured_altitude_beam4
			beam4.range_covariance = 0.0001 #TODO: find accurate value
			beam4.velocity = measured_velocity_beam4
			beam4.velocity_covariance = fom_beam4 * fom_beam4
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

			odometry_msg.pose.pose.position.z=-((measured_pressure*10000)*10)/(997*9.81)
				
			if (fom_velocity_x != 10) and (fom_velocity_y != 10) and (fom_velocity_z_best != 10):
				odometry_msg.twist.covariance = [fom_velocity_x * fom_velocity_x, unknown, unknown, unknown, unknown, unknown, unknown, fom_velocity_y * fom_velocity_y, unknown, unknown, unknown, unknown, unknown, unknown, fom_velocity_z_best * fom_velocity_z_best, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown, unknown]
			self.odometry_pub.publish(odometry_msg)

			#Pressure topic
			pressure_msg.header.stamp = rospy.Time.now()
			pressure_msg.header.frame_id = "dvl_link"
			pressure_msg.fluid_pressure = measured_pressure * 10000 #Convert dbar to Pascal
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
