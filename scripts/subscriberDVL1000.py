#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from dvl1000_ros.msg import DVL

def callBottom(data):
    rospy.loginfo(rospy.get_caller_id() + "From Bottom-Tracking: %s", data.header.stamp)
      
def subscriberDVL1000():

	# In ROS, nodes are uniquely named. If two nodes with the same
	# name are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('subscriberDVL1000', anonymous=False)

	rospy.Subscriber("/auv/dvl", DVL, callBottom)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
    subscriberDVL1000()
