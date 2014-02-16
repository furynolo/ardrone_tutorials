#!/usr/bin/env python

import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy
from std_msgs.msg import String


def callback(data):
    rospy.loginfo(rospy.get_name() + ": I heard %s" % data.data)


def subDecision():
    rospy.init_node('ardrone_decisions_subscriber', anonymous=True)
    #rospy.Subscriber("ardrone_decisions", String, callback)
    #rospy.Subscriber("/ardrone/entities", String, callback)
    rospy.Subscriber("/ardrone/commands", String, callback)
    #rospy.Timer(rospy.Duration(10), test)
    rospy.spin()

def test(event):
    rospy.loginfo("Test")

if __name__ == '__main__':
    subDecision()