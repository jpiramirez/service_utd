#!/usr/bin/env python
import roslib; roslib.load_manifest('ardrone_control')
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty

Kz = 0.3
Kphi = 0.3
Ktheta = 0.1
Kpsi = 0.7
KDz = 0.1
KDphi = 0.2
KDtheta = 0.1
KDpsi = 0.2
distance = 0.5
command = Twist()
automode = False
xprev = 0
yprev = 0
zprev = 0
psiprev = 0


def callback(data):
	global xprev
	global yprev
	global zprev
	global psiprev

	command.linear.z = Kz*data.position.y + KDz*(data.position.y - yprev)
	command.linear.y = Kphi*data.position.x + KDphi*(data.position.x - xprev)
	command.linear.x = - Ktheta*(data.position.z - distance) - KDtheta*(data.position.z - distance - zprev)
	command.angular.z = -Kpsi*data.position.x

	xprev = data.position.x
	yprev = data.position.y
	zprev = data.position.z - distance

	# For the first tests
	#command.linear.x = 0.0

def autocb(data):
	global automode
	if not automode:
		rospy.loginfo('Enabling automatic flight')
		automode = True
	else:
		rospy.loginfo('Disabling automatic flight')
		automode = False


def talker():
	global pub
	global automode
	
	rospy.init_node('ardrone_control')
	rospy.Subscriber('/vispose/pose', Pose, callback)
	rospy.Subscriber('/ardrone/auto', Empty, autocb)
	pub = rospy.Publisher('/cmd_vel', Twist)
	while not rospy.is_shutdown():
		if automode:
			pub.publish(command)
			#command.linear.x = 0.0
			#command.linear.y = 0.0
			#command.linear.z = 0.0
		rospy.sleep(0.02)
if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException: pass
