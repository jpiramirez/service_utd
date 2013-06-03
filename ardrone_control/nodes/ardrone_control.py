#!/usr/bin/env python
import roslib; roslib.load_manifest('ardrone_control')
import rospy
from math import *
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy

Kz = 0.3
Kphi = 0.3
Ktheta = 0.1
Kpsi = 0.3
KDz = 0.01
KDphi = 0.02
KDtheta = 0.01
KDpsi = 0.02
distance = 1.0
command = Twist()
automode = False
xprev = 0
yprev = 0
zprev = 0
psiprev = 0
state = 0
autoflight = 0
pub = rospy.Publisher('/cmd_vel', Twist)

def jscallback(data):
	global pub
	global state
	global autoflight
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
	autoflight = data.buttons[10]
	if state == 1 and autoflight == 0:
		pub.publish(pmsg)

def callback(data):
	global xprev
	global yprev
	global zprev
	global psiprev
	global state
	global pub
	global autoflight

	q = data.pose.orientation
	theta = asin(2*(q.w*q.y-q.z*q.x))
	rospy.loginfo("Yaw %s  Rads    %s  Deg", theta, theta*180.0/3.141592)

	

	command = Twist()

	command.linear.z = -Kz*data.pose.position.y - KDz*(data.pose.position.y - yprev)
	command.linear.y = -Kphi*data.pose.position.x - KDphi*(data.pose.position.x - xprev)
	command.linear.x = Ktheta*(data.pose.position.z - distance) + KDtheta*(data.pose.position.z - distance - zprev)
	command.angular.z = Kpsi*theta

	
	xprev = data.pose.position.x
	yprev = data.pose.position.y
	zprev = data.pose.position.z - distance

	if autoflight == 0:
		return

	pub.publish(command)
	# For the first tests
	#command.linear.x = 0.0


def talker():
	global pub
	global automode
	
	rospy.init_node('ardrone_control')
	rospy.Subscriber("/joy", Joy, jscallback, queue_size=1)
	rospy.Subscriber('/vispose/pose', PoseStamped, callback)
	while not rospy.is_shutdown():
		rospy.spin()
if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException: pass
