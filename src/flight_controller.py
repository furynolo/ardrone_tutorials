#!/usr/bin/env python

# The Flight Controller Node

# This controller extends the base DroneVideoDisplay class, adding a keypress handler to enable keyboard control of the drone

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy
import time

# Import String library
from std_msgs.msg import String

# Load the DroneController class, which handles interactions with the drone, and the DroneVideoDisplay class, which handles video display
from drone_controller import BasicDroneController
from drone_video_display import DroneVideoDisplay

# Finally the GUI libraries
from PySide import QtCore, QtGui

# Define Globals
medium_length = 2.0
short_length = 0.5
long_length = 5.0

# Here we define the keyboard map for our controller (note that python has no enums, so we use a class)
class KeyMapping(object):
    PitchForward = QtCore.Qt.Key.Key_E
    PitchBackward = QtCore.Qt.Key.Key_D
    RollLeft = QtCore.Qt.Key.Key_S
    RollRight = QtCore.Qt.Key.Key_F
    YawLeft = QtCore.Qt.Key.Key_W
    YawRight = QtCore.Qt.Key.Key_R
    IncreaseAltitude = QtCore.Qt.Key.Key_Q
    DecreaseAltitude = QtCore.Qt.Key.Key_A
    Takeoff = QtCore.Qt.Key.Key_Y
    Land = QtCore.Qt.Key.Key_H
    Emergency = QtCore.Qt.Key.Key_Space


# Our controller definition, note that we extend the DroneVideoDisplay class
class FlightController(DroneVideoDisplay):
    def __init__(self):
        super(FlightController, self).__init__()
        self.subCommanddata = rospy.Subscriber("/ardrone/commands", String, self.ReceiveCommands)
        # Initialize pitch, roll, yaw_velocity, z_velocity
        self.pitch = 0
        self.roll = 0
        self.yaw_velocity = 0
        self.z_velocity = 0

        # Initialize flight variables
        self.state = 0  # 0=start/takeoff, 1=search/spin, 2=calculate/move, 3=close/land, 4=final/done

        # Initialize target information variables
        self.waitCounter = 1
#        self.twistTimer = rospy.Timer(rospy.Duration(2), self.test)

# We add a keyboard handler to the DroneVideoDisplay to react to keypresses
    def keyPressEvent(self, event):
        key = event.key()

        # If we have constructed the drone controller and the key is not generated from an auto-repeating key
        if controller is not None and not event.isAutoRepeat():
            # Handle the important cases first!
            if key == KeyMapping.Emergency:
                controller.SendEmergency()
            elif key == KeyMapping.Takeoff:
                controller.SendTakeoff()
            elif key == KeyMapping.Land:
                controller.SendLand()
            else:
                # Now we handle moving, notice that this section is the opposite (+=) of the keyrelease section
                if key == KeyMapping.YawLeft:
                    self.yaw_velocity += 1
                elif key == KeyMapping.YawRight:
                    self.yaw_velocity += -1

                elif key == KeyMapping.PitchForward:
                    self.pitch += 1
                elif key == KeyMapping.PitchBackward:
                    self.pitch += -1

                elif key == KeyMapping.RollLeft:
                    self.roll += 1
                elif key == KeyMapping.RollRight:
                    self.roll += -1

                elif key == KeyMapping.IncreaseAltitude:
                    self.z_velocity += 1
                elif key == KeyMapping.DecreaseAltitude:
                    self.z_velocity += -1

            # finally we set the command to be sent. The controller handles sending this at regular intervals
            controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)


    def keyReleaseEvent(self, event):
        key = event.key()

        # If we have constructed the drone controller and the key is not generated from an auto-repeating key
        if controller is not None and not event.isAutoRepeat():
            # Note that we don't handle the release of emergency/takeoff/landing keys here, there is no need.
            # Now we handle moving, notice that this section is the opposite (-=) of the keypress section
            if key == KeyMapping.YawLeft:
                self.yaw_velocity -= 1
            elif key == KeyMapping.YawRight:
                self.yaw_velocity -= -1

            elif key == KeyMapping.PitchForward:
                self.pitch -= 1
            elif key == KeyMapping.PitchBackward:
                self.pitch -= -1

            elif key == KeyMapping.RollLeft:
                self.roll -= 1
            elif key == KeyMapping.RollRight:
                self.roll -= -1

            elif key == KeyMapping.IncreaseAltitude:
                self.z_velocity -= 1
            elif key == KeyMapping.DecreaseAltitude:
                self.z_velocity -= -1

            # finally we set the command to be sent. The controller handles sending this at regular intervals
            controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

    def ReceiveCommands(self, inputString):
        # inputString = str(inputString)
        rospy.loginfo("I have recieved a command.")
        if inputString.data == "!takeoff":
            controller.SendTakeoff()
        elif inputString.data == "!land":
            controller.SendLand()
        elif inputString.data == "!twist":
            # twist @ ~45deg turns
            rospy.Timer(rospy.Duration(medium_length), self.twist, oneshot=True)
            # rospy.sleep(11)
        elif inputString.data == "!rise":
            rospy.Timer(rospy.Duration(medium_length), self.rise, oneshot=True)
        elif inputString.data == "!lower":
            rospy.Timer(rospy.Duration(medium_length), self.lower, oneshot=True)
            
        elif inputString.data == "!slightTwistRight":
            rospy.Timer(rospy.Duration(short_length), self.slightTwistRight, oneshot=True)
        elif inputString.data == "!slightTwistLeft":
            rospy.Timer(rospy.Duration(short_length), self.slightTwistLeft, oneshot=True)
        elif inputString.data == "!slightMoveForward":
            rospy.Timer(rospy.Duration(short_length), self.slightMoveForward, oneshot=True)
        elif inputString.data == "!slightMoveBackward":
            rospy.Timer(rospy.Duration(short_length), self.slightMoveBackward, oneshot=True)
        elif inputString.data == "!slightMoveRight":
            rospy.Timer(rospy.Duration(short_length), self.slightMoveRight, oneshot=True)
        elif inputString.data == "!slightMoveLeft":
            rospy.Timer(rospy.Duration(short_length), self.slightMoveLeft, oneshot=True)
        elif inputString.data == "!slightMoveUp":
            rospy.Timer(rospy.Duration(short_length), self.slightMoveUp, oneshot=True)
        elif inputString.data == "!slightMoveDown":
            rospy.Timer(rospy.Duration(short_length), self.slightMoveDown, oneshot=True)
        
              
        else:
            pass
#             rospy.loginfo("%s" % inputString.data)
#             dataArray will be in format of (roll,pitch,yaw_velocity,z_velocity)
#             dataArray = eval(inputString.data)

# controller.SetCommand(<roll>, <pitch>, <yaw_velocity>, <z_velocity>)

    def twist(self, event):
        #controller.SetCommand(0, 0, 10, 0)
        controller.SetCommand(0, 0, 1000, 0)
        rospy.sleep(1)
        #rospy.sleep(10)
        controller.SetCommand(0, 0, 0, 0)
        
    def rise(self, event):
        #controller.SetCommand(0, 0, 0, 150)
        controller.SetCommand(0, 0, 0, 400)
        rospy.sleep(4)
        controller.SetCommand(0, 0, 0, 0)
    
    def lower(self, event):
        #controller.SetCommand(0, 0, 0, -150)
        controller.SetCommand(0, 0, 0, -400)
        rospy.sleep(4)
        controller.SetCommand(0, 0, 0, 0)
        
    
    # Slight movement control
    def slightTwistRight(self, event):
        controller.SetCommand(0, 0, -5, 0)
        rospy.sleep(0.5)
        controller.SetCommand(0, 0, 0, 0)
    
    def slightTwistLeft(self, event):
        controller.SetCommand(0, 0, 5, 0)
        rospy.sleep(0.5)
        controller.SetCommand(0, 0, 0, 0)
    
    def slightMoveForward(self, event):
        controller.SetCommand(0, 5, 0, 0)
        rospy.sleep(0.5)
        controller.SetCommand(0, 0, 0, 0)
    
    def slightMoveBackward(self, event):
        controller.SetCommand(0, -5, 0, 0)
        rospy.sleep(0.5)
        controller.SetCommand(0, 0, 0, 0)
    
    def slightMoveRight(self, event):
        controller.SetCommand(-5, 0, 0, 0)
        rospy.sleep(0.5)
        controller.SetCommand(0, 0, 0, 0)
    
    def slightMoveLeft(self, event):
        controller.SetCommand(5, 0, 0, 0)
        rospy.sleep(0.5)
        controller.SetCommand(0, 0, 0, 0)
    
    def slightMoveUp(self, event):
        controller.SetCommand(0, 0, 0, 80)
        rospy.sleep(0.5)
        controller.SetCommand(0, 0, 0, 0)
    
    def slightMoveDown(self, event):
        controller.SetCommand(0, 0, 0, -80)
        rospy.sleep(0.5)
        controller.SetCommand(0, 0, 0, 0)
    

    def test(self, event):
        rospy.loginfo("This is a test!")
#         controller.SendEmergency()
        if self.twistCounter:
            self.twistCounter -= 1

# Setup the application
if __name__ == '__main__':
    import sys
    # Firstly we setup a ros node, so that we can communicate with the other packages
    rospy.init_node('ardrone_flight_controller')

    # Now we construct our Qt Application and associated controllers and windows
    app = QtGui.QApplication(sys.argv)
    controller = BasicDroneController()
    display = FlightController()

    display.show()

    # executes the QT application
    status = app.exec_()

    # and only progresses to here once the application has been shutdown
    rospy.signal_shutdown('Great Flying!')
    sys.exit(status)
