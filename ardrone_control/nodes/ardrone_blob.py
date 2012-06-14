#!/usr/bin/env python
import roslib; roslib.load_manifest('ardrone_blob')
import rospy
from geometry_msgs.msg import Twist
from cmvision.msg import Blobs

K = 0.01

def compare_blobs(a, b):
	return cmp(a.area, b.area) 

def callback(data):
	command = Twist()
	if data.blob_count > 0:
		barray = sorted(data.blobs, compare_blobs)
		command.linear.x = barray[0].x
		command.linear.y = barray[0].y
		command.linear.z = barray[0].right - barray[0].left
		pub.publish(command)

def talker():
	global pub

	rospy.init_node('ardrone_blob')
	rospy.Subscriber('/blobs', Blobs, callback);
	pub = rospy.Publisher('/teites', Twist)
	rospy.spin();
if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException: pass
