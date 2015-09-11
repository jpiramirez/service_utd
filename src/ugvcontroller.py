#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry

pub = 0

def cbfunc(data):
	global pub
	T = rospy.get_time()
	a = 1;
	control = Twist()
	control.linear.x = (a**2.0)*math.sin(a*T)**2.0 + 4.0*(a**2)*math.cos(2.0*a*T)**2.0
	control.linear.x = 0.5*math.sqrt(control.linear.x)
	control.angular.z = (-4.0*a*math.sin(a*T))/(1.0+16.0*math.cos(a*T)**2)
	control.angular.z = math.atan2(2*a*math.cos(2*a*T), -a*math.sin(a*T))
	pub.publish(control)

def ugvcontroller():
	global pub
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	rospy.init_node('ugvcontroller', anonymous=True)
	rospy.Subscriber('/odom', Odometry, cbfunc)
	rospy.spin()

if __name__ == '__main__':
	try:
		ugvcontroller()
	except rospy.ROSInterruptException:
		pass