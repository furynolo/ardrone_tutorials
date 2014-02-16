#!/usr/bin/env python

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy

# Import String library
from std_msgs.msg import String


# Define what the decision publisher does
def pubDecisions():
    pub = rospy.Publisher("/ardrone/commands", String)
    rospy.init_node('ardrone_decisions_publisher')
    while not rospy.is_shutdown():
        cmd = raw_input("Your command here: ")
#         str = "ardrone_decisions test %s" % rospy.get_time()
#         rospy.loginfo(str)
#         pub.publish(String(str))
        rospy.loginfo(cmd)
        pub.publish(String(cmd))
#         rospy.sleep(1.0)

if __name__ == '__main__':
    pubDecisions()