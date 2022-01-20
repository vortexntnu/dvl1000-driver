#!/usr/bin/env python

import json
from websocket import create_connection

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TwistWithCovariance

class DVL1000_Ros_Driver:

	def __init__(self, rate=None):
		
		if rate is None:
			self.rate = rospy.Rate(1)
		else:
			self.rate = rate

		self.seq = 0
		self.data_id = 8219 # ID 8219 is for Bottom Track mode. Alternative is 4123 (contains less data); see data sheet for more info
		self.invalid_velocity = -32

		self.odom_pub = rospy.Publisher('/dvl/odom', Odometry, queue_size=10)

		websocket_address = rospy.get_param("/dvl/websocket_address", "ws://192.168.0.96:10100")
		self.ws = create_connection(websocket_address, subprotocols=["data-transfer-nortek"])
		print("Connecting to DVL1000 via websocket...")
		result = self.ws.recv()
		print("Status: '%s'" % result)

	def __del__(self):
		self.ws.close()

	def get_dvl_measurements(self):

		dvl_data = self.ws.recv()
		dvl_data_json = json.loads(dvl_data)

		dvl_data_valid = False
		if dvl_data_json["id"] == self.data_id:
			dvl_data_valid = True

		return dvl_data_json, dvl_data_valid

	def pressure_to_depth(self, pressure):
		return -((pressure*10000)*10)/(997*9.81)


	def spin(self):

		dvl_data, dvl_data_valid = self.get_dvl_measurements()

		if dvl_data_valid:
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
			
			# Since we have two measurements for velocity in z, choose the best one
			fom_velocity_z_best = 0
			measured_velocity_z_best = 0
			if fom_velocity_z_1 > fom_velocity_z_2:
				fom_velocity_z_best = fom_velocity_z_2
				measured_velocity_z_best = measured_velocity_z_2
			else:
				fom_velocity_z_best = fom_velocity_z_1
				measured_velocity_z_best = measured_velocity_z_1

			# Create twist part of odometry message
			twist_msg = TwistWithCovariance()
			twist_msg.twist.linear.x = measured_velocity_x
			twist_msg.twist.linear.y = measured_velocity_y
			twist_msg.twist.linear.z = measured_velocity_z_best

			# Assume zero covariance, i.e. all non-diagonal elements are 0
			# Squaring the FOM is used here as an estimate for variance
			var_x = fom_velocity_x * fom_velocity_x
			var_y = fom_velocity_y * fom_velocity_y
			var_z = fom_velocity_z_best * fom_velocity_z_best
			twist_msgcovariance = [var_x,  0,      0,   0, 0, 0,
			  					     0,   var_y,   0,   0, 0, 0,
			  					     0,    0,    var_z, 0, 0, 0,
			  					     0,    0,      0,   0, 0, 0,
			  					     0,    0,      0,   0, 0, 0,
			  					     0,    0,      0,   0, 0, 0]

			# Create pose part of odometry message
			# We only get z from the pressure measurement for the pose
			# The pressure covariance is currently undetermined, but is
			# set as exact for now. If the variance is found, it will be 
			# element the third diagonal element of the covariance matrix
			pose_msg = PoseWithCovariance()
			pose_msg.pose.position.z = self.pressure_to_depth(measured_pressure)
			pose_msg.covariance = [0, 0, 0, 0, 0, 0,
								   0, 0, 0, 0, 0, 0,
								   0, 0, 0, 0, 0, 0,
								   0, 0, 0, 0, 0, 0,
								   0, 0, 0, 0, 0, 0,
								   0, 0, 0, 0, 0, 0]


			odometry_msg = Odometry()
			odometry_msg.header.seq = self.seq
			odometry_msg.header.stamp = rospy.Time.now()
			odometry_msg.header.frame_id = "dvl_link"

			odometry_msg.pose = pose_msg
			odometry_msg.twist = twist_msg

			# Only publish odometry if valid velocity data
			if (measured_velocity_x < self.invalid_velocity or 
			    measured_velocity_y < self.invalid_velocity or 
			    measured_velocity_z < self.invalid_velocity
			):
				rospy.logwarn("DVL velocity measurement invalid! Skipping.")
			else:
				self.odom_pub.publish(odometry_msg)

		self.seq += 1
		self.rate.sleep()

if __name__ == '__main__':
	try:
		rospy.init_node('DVL1000', anonymous=False)
		rate = rospy.Rate(8) # 8hz

		dvl = DVL1000_Ros_Driver(rate)

		while not rospy.is_shutdown():
			dvl.spin()
			
	except rospy.ROSInterruptException:
		pass
