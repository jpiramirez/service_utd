#!/usr/bin/env python
import roslib; 
import rospy
import math
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

pub = rospy.Publisher('/cmd_vel', Twist)
state = 0

def callback(data):
	global pub
	global state
	mustpub = False
	pmsg = Twist()
	pmsg.linear.x = data.axes[1]
	pmsg.linear.y = data.axes[0]
	pmsg.linear.z = data.axes[3]
	pmsg.angular.z = data.axes[2]
	if data.buttons[3] > 0 and state == 0:
		epub = rospy.Publisher('/ardrone/takeoff', Empty)
		epub.publish(Empty())
		state = 1
		rospy.loginfo(rospy.get_name() + " Taking off...")
	if data.buttons[0] > 0 and state == 1:
		epub = rospy.Publisher('/ardrone/land', Empty)
		epub.publish(Empty())
		state = 0
		rospy.loginfo(rospy.get_name() + " Landing...")
	if state == 1 and data.buttons[0] != 1:
	#	if abs(pmsg.linear.x) > 1e-3 or abs(pmsg.linear.y) > 1e-3 or abs(pmsg.linear.z) > 1e-3:
		pub.publish(pmsg)

def listener():
	rospy.init_node('joy2drone')
	rospy.Subscriber("joy", Joy, callback, queue_size=1)
	rospy.spin()
 
if __name__ == '__main__':
	listener()
