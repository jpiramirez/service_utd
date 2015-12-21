#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import Joy
from service_utd.msg import SoftMeasure
from std_msgs.msg import Empty

pub = rospy.Publisher('/ground_obs', SoftMeasure, queue_size=1)
state = 0

def callback(data):
	global pub
	global state

	if data.buttons[0] == 0:
		return

	pmsg = SoftMeasure()
	pmsg.measurement = True
	pmsg.ul.x = 20
	pmsg.ul.y = 20
	pmsg.br.x = 10
	pmsg.br.y = 10
	pmsg.alpha = 1
	pmsg.beta = 0
	pmsg.header.stamp = rospy.get_rostime() - rospy.Duration(1.0)
	print "Sending measurement at " + str(pmsg.header.stamp)
	pub.publish(pmsg)

def listener():
	rospy.init_node('groundobserver')
	rospy.Subscriber("joy", Joy, callback, queue_size=1)
	rospy.spin()
 
if __name__ == '__main__':
	listener()
