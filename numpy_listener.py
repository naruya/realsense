#!/usr/bin/env python

import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

def callback(data):
    print rospy.get_name(), "I heard %s"%str(data.data)

def listener():
    rospy.init_node('listener')
    rospy.Subscriber("contours", numpy_msg(Floats), callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
