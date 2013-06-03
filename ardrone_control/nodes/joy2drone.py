#!/usr/bin/env python
import roslib; roslib.load_manifest('ardrone_control')
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

pub = rospy.Publisher('/cmd_vel', Twist)
state = 0

def callback(data):
	global pub
	global state
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
		pub.publish(pmsg)

def listener():
	rospy.init_node('joy2drone')
	rospy.Subscriber("joy", Joy, callback, queue_size=1)
	rospy.spin()
 
if __name__ == '__main__':
	listener()
