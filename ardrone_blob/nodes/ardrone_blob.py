#!/usr/bin/env python
import roslib; roslib.load_manifest('ardrone_blob')
import rospy
from geometry_msgs.msg import Twist
from cmvision.msg import Blobs

K = 0.01

def callback(data):
    command = Twist()
    if data.blob_count == 1:
    	command.linear.x = data.blobs[0].x
	command.linear.y = data.blobs[0].y
	command.linear.z = data.blobs[0].right - data.blobs[0].left
	pub = rospy.Publisher('/teites', Twist)
        pub.publish(command)

def talker():
    rospy.init_node('ardrone_blob')
    rospy.Subscriber('/blobs', Blobs, callback);
    rospy.spin();
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
