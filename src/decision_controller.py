#!/usr/bin/env python

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy

# Import String library
from std_msgs.msg import String

# Load the DroneController class, which handles interactions with the drone, and the DroneVideoDisplay class, which handles video display
from drone_controller import BasicDroneController
from drone_video_display import DroneVideoDisplay

ACCEPT_UPPER_MID_X = 180
ACCEPT_UPPER_MID_Y = 180
ACCEPT_LEFT_TWO_Y_DIFFERENTIAL = 52

class Decisioneer(object):
    def __init__(self):
        self.subEntitydata = rospy.Subscriber("/ardrone/entities", String, self.ReceiveEntities)
        self.pubCommand = rospy.Publisher('/ardrone/commands', String)
        
        # wait for a second so the Publisher may get properly set up
        rospy.sleep(1.0)
        
        # Initialize decision making variables
        # 0->2
        self.level = 0
        # 0->7
        self.angle = 0
        # Flag set to True when 
        self.flag = False
        # Stores the most recent (x,y) values of the identified targets
        self.lastKnownArrayValues = []
        # Set to true when the mission is successfully completed
        self.missionSuccess = False

    def ReceiveEntities(self, inputString):
        dataArray = eval(inputString.data)
        if len(dataArray) >= 3:
            self.lastKnownArrayValues = dataArray
            if self.flag == False:
                self.pubCommand.publish("!set self.flag = True")
            self.flag = True
            # Publish the command to the Flight Controller
        else:
            self.flag = False
        
    def Takeoff(self):
        self.pubCommand.publish("!takeoff")
        rospy.sleep(5)
        self.level = 1
        
    def Land(self):
        self.pubCommand.publish("!land")
        self.level = 0
    
    def RiseOneLevel(self):
        self.pubCommand.publish("!rise")
        self.level += 1
        rospy.sleep(5)
    
    def LowerOneLevel(self):
        self.pubCommand.publish("!lower")
        self.level -= 1
        rospy.sleep(5)
    
    # Search for tags: rotate until find, increase altitude
    def FindTarget(self):
#         counter = 0
        done = False
        # Spin eight times about 45 degrees each time
        while (done == False):
            counter = 0
            while counter <= 8:
                if self.flag == True:
                    return
                self.pubCommand.publish("!twist")
                rospy.sleep(3)
                #rospy.sleep(12)
                counter += 1
            if self.level == 2:
                self.RiseOneLevel()
            elif self.level >= 3:
                self.LowerOneLevel()
        
    # We see 3-6 tags and want to start moving (overall) towards the target until ~3ft away.
    def ApproachTarget(self):
        try:
            # Reverify that we see 3+ tags
            while self.flag == True:
#                 rospy.sleep(3)
#                 self.pubCommand.publish("#Approaching Target")
                # Gets the upper of the two left-most targets
                leftUpper, leftLower = self.GetLeftTwo()
                # Gets the upper of the two right-most targets
                rightUpper, rightLower = self.GetRightTwo()
                
                # Try to center the tags horizontally within the frame
                # If we are facing too far to the left
                if ((leftUpper[0] < (ACCEPT_UPPER_MID_X-25)) and (self.flag == True)):
                    self.RotateSlightlyLeft()
#                     leftUpper, leftLower = self.GetLeftTwo()
#                     rightUpper, rightLower = self.GetRightTwo()
                # If we are facing too far to the right
                if ((leftUpper[0] > (ACCEPT_UPPER_MID_X+25)) and (self.flag == True)):
                    self.RotateSlightlyRight()
#                     leftUpper, leftLower = self.GetLeftTwo()
#                     rightUpper, rightLower = self.GetRightTwo()
                
                # If we can see 6 targets then we are close enough, otherwise we need to move forward for better resolution
                if len(self.lastKnownArrayValues) == 6:
                    self.pubCommand.publish("#6 tags seen")
                    # Gets the upper of the two left-most targets
                    leftUpper, leftLower = self.GetLeftTwo()
                    # Gets the upper of the two right-most targets
                    rightUpper, rightLower = self.GetRightTwo()
                    
                    # Try to center the ardrone horizontally with targets so it is perpendicular to surface
                    if ((leftUpper[1] - leftLower[1]) < (rightUpper[1] - rightLower[1] + 5)):
                        self.MoveSlightlyLeft()
                    elif  ((leftUpper[1] - leftLower[1] + 5) > (rightUpper[1] - rightLower[1])):
                        self.MoveSlightlyRight()
                    else:
                        if (((leftUpper[1] - leftLower[1]) < 57) and ((leftUpper[1] - leftLower[1]) > 47)):
                            self.missionSuccess = True
                            self.pubCommand.publish("!missionSuccess")
                            return
                    
                    # Gets the upper of the two left-most targets
                    leftUpper, leftLower = self.GetLeftTwo()
                    # Try to center the tags vertically within the frame
                    if (leftUpper[1] > ACCEPT_UPPER_MID_Y + 25):
                        self.MoveSlightlyUp()
                    elif (leftUpper[1] < ACCEPT_UPPER_MID_Y - 25):
                        self.MoveSlightlyDown()
                    
                    # Gets the upper of the two left-most targets
                    leftUpper, leftLower = self.GetLeftTwo()
                    # Try to move within range of targets
                    if (leftUpper[1] - leftLower[1]) < ACCEPT_LEFT_TWO_Y_DIFFERENTIAL - 5:
                        self.MoveSlightlyForward()
                    elif (leftUpper[1] - leftLower[1]) > ACCEPT_LEFT_TWO_Y_DIFFERENTIAL + 5:
                        self.MoveSlightlyBackward()
                else:
                    self.MoveSlightlyForward()
        except:
            pass
                
        
    def RotateSlightlyRight(self):
        self.pubCommand.publish("!slightTwistRight")
        rospy.sleep(0.75)
    
    def RotateSlightlyLeft(self):
        self.pubCommand.publish("!slightTwistLeft")
        rospy.sleep(0.75)
    
    def MoveSlightlyForward(self):
        self.pubCommand.publish("!slightMoveForward")
        rospy.sleep(0.75)
    
    def MoveSlightlyBackward(self):
        self.pubCommand.publish("!slightMoveBackward")
        rospy.sleep(0.75)
    
    def MoveSlightlyRight(self):
        self.pubCommand.publish("!slightMoveRight")
        rospy.sleep(0.75)
    
    def MoveSlightlyLeft(self):
        self.pubCommand.publish("!slightMoveLeft")
        rospy.sleep(0.75)
    
    def MoveSlightlyUp(self):
        self.pubCommand.publish("!slightMoveUp")
        rospy.sleep(0.75)
    
    def MoveSlightlyDown(self):
        self.pubCommand.publish("!slightMoveDown")
        rospy.sleep(0.75)
#
    def GetLeftTwo(self):
        self.lastKnownArrayValues.sort()
        leftMost1 = self.lastKnownArrayValues[0]
        leftMost2 = self.lastKnownArrayValues[1]
        if leftMost2[1] < leftMost2[1]:
            return leftMost2, leftMost1
        else:
            return leftMost1, leftMost2
        
    def GetRightTwo(self):
#         self.lastKnownArrayValues.sort()
        rightMost1 = self.lastKnownArrayValues[4]
        rightMost2 = self.lastKnownArrayValues[5]
        if rightMost2[1] < rightMost1[1]:
            return rightMost2, rightMost1
        else:
            return rightMost1, rightMost2
        
#     def GetRightMost(self):
#         pass
#     
#     def GetUpperMost(self):
#         pass
        
if __name__ == '__main__':
    rospy.init_node('ardrone_decisioneer')
    controller = BasicDroneController()
    decisioneer = Decisioneer()
    
    decisioneer.Takeoff()
    decisioneer.RiseOneLevel()
    
    while decisioneer.missionSuccess == False:
        decisioneer.FindTarget()
        decisioneer.ApproachTarget()
    
    decisioneer.Land()

    rospy.signal_shutdown('Weeeeeeeeeeee!')

