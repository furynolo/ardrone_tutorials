#!/usr/bin/env python

# A basic video display window for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This display window listens to the drone's video feeds and updates the display at regular intervals
# It also tracks the drone's status and any connection problems, displaying them in the window's status bar
# By default it includes no control functionality. The class can be extended to implement key or mouse listeners if required

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy

# Import the two types of messages we're interested in plus string
from sensor_msgs.msg import Image  # for receiving the video feed
from ardrone_autonomy.msg import Navdata  # for receiving navdata feedback
from std_msgs.msg import String

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
from threading import Lock

# An enumeration of Drone Statuses
from drone_status import DroneStatus

# The GUI libraries
from PySide import QtCore, QtGui

# For importing opencv
from cv_bridge import CvBridge, CvBridgeError
# import cv
import cv2
import cv2.cv as cv
import numpy as np

# Some Constants
CONNECTION_CHECK_PERIOD = 250  # ms
GUI_UPDATE_PERIOD = 20  # ms
DETECT_RADIUS = 4  # the radius of the circle drawn when a tag is detected

# HSV Tuning values
# Joel's Apt. Nighttime
g_hue_low = 0
g_sat_low = 100
# g_val_low = 165
g_val_low = 185
g_hue_high = 180
g_sat_high = 255
g_val_high = 255

# Joel's Apt. Daytime (Blinds closed)
# g_hue_low = 0
# g_sat_low = 120
# g_val_low = 200
# g_hue_high = 180
# g_sat_high = 255
# g_val_high = 255

# Matt's Lab Values
# g_hue_low = 0
# g_sat_low = 34
# g_val_low = 192
# g_hue_high = 11
# g_sat_high = 255
# g_val_high = 255
# Matt's Lab (slightly overcast?)
# g_hue_low = 0
# g_sat_low = 100
# g_val_low = 185
# g_hue_high = 11
# g_sat_high = 255
# g_val_high = 255

g_tuning_flag = False
# g_tuning_flag = True

class DroneVideoDisplay(QtGui.QMainWindow):
    StatusMessages = {
        DroneStatus.Emergency : 'Emergency',
        DroneStatus.Inited    : 'Initialized',
        DroneStatus.Landed    : 'Landed',
        DroneStatus.Flying    : 'Flying',
        DroneStatus.Hovering  : 'Hovering',
        DroneStatus.Test      : 'Test (?)',
        DroneStatus.TakingOff : 'Taking Off',
        DroneStatus.GotoHover : 'Going to Hover Mode',
        DroneStatus.Landing   : 'Landing',
        DroneStatus.Looping   : 'Looping (?)'
        }
    DisconnectedMessage = 'Disconnected'
    UnknownMessage = 'Unknown Status'
    
    def __init__(self):
        # Construct the parent class
        super(DroneVideoDisplay, self).__init__()

        # Setup our very basic GUI - a label which fills the whole window and holds our image
        self.setWindowTitle('Flying Mongoose')
        
        ####
        # Create named windows for displaying cv2 windows
#        cv2.namedWindow("HSV Image", cv2.CV_WINDOW_AUTOSIZE)
#        cv2.namedWindow("Binary Image", cv2.CV_WINDOW_AUTOSIZE)
#        cv2.namedWindow("Blur Image", cv2.CV_WINDOW_AUTOSIZE)
#        cv2.namedWindow("GaussianBlur Image", cv2.CV_WINDOW_AUTOSIZE)
#        cv2.namedWindow("Erode Image", cv2.CV_WINDOW_AUTOSIZE)
#        cv2.namedWindow("Dilate Image", cv2.CV_WINDOW_AUTOSIZE)
#        cv2.namedWindow("Contour", cv2.CV_WINDOW_AUTOSIZE)
#        cv2.namedWindow("Mean Mask", cv2.CV_WINDOW_AUTOSIZE)
        ####
        
        self.imageBox = QtGui.QLabel(self)
        self.setCentralWidget(self.imageBox)

        # Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
        self.subNavdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.ReceiveNavdata)
        
        # Subscribe to the drone's video feed, calling self.ReceiveImage when a new frame is received
        self.subVideo = rospy.Subscriber('/ardrone/image_raw', Image, self.ReceiveImage)
        
        # Holds the image frame received from the drone and later processed by the GUI
        self.image = None
        self.imageLock = Lock()

        self.tags = []
        self.tagLock = Lock()
        
        # Holds the status message to be displayed on the next GUI update
        self.statusMessage = ''

        # Tracks whether we have received data since the last connection check
        # This works because data comes in at 50Hz but we're checking for a connection at 4Hz
        self.communicationSinceTimer = False
        self.connected = False

        # A timer to check whether we're still connected
        self.connectionTimer = QtCore.QTimer(self)
        self.connectionTimer.timeout.connect(self.ConnectionCallback)
        self.connectionTimer.start(CONNECTION_CHECK_PERIOD)
        
        # A timer to redraw the GUI
        self.redrawTimer = QtCore.QTimer(self)
        self.redrawTimer.timeout.connect(self.RedrawCallback)
        self.redrawTimer.start(GUI_UPDATE_PERIOD)
        
        # initialize bridge
        self.bridge = CvBridge()

        # Initialize entities publisher (x,y,size) of contour
        self.pubEntities = rospy.Publisher('/ardrone/entities', String)

    # Called every CONNECTION_CHECK_PERIOD ms, if we haven't received anything since the last callback, will assume we are having network troubles and display a message in the status bar
    def ConnectionCallback(self):
        self.connected = self.communicationSinceTimer
        self.communicationSinceTimer = False

    def RedrawCallback(self):
        if self.image is not None:
            # We have some issues with locking between the display thread and the ros messaging thread due to the size of the image, so we need to lock the resources
            self.imageLock.acquire()
            try:            
                    # Convert the ROS image into a QImage which we can display
                    image = QtGui.QPixmap.fromImage(QtGui.QImage(self.image.data, self.image.width, self.image.height, QtGui.QImage.Format_RGB888))
                    if len(self.tags) > 0:
                        self.tagLock.acquire()
                        try:
                            painter = QtGui.QPainter()
                            painter.begin(image)
                            painter.setPen(QtGui.QColor(0, 255, 0))
                            painter.setBrush(QtGui.QColor(0, 255, 0))
                            for (x, y, d) in self.tags:
                                r = QtCore.QRectF((x * image.width()) / 1000 - DETECT_RADIUS, (y * image.height()) / 1000 - DETECT_RADIUS, DETECT_RADIUS * 2, DETECT_RADIUS * 2)
                                painter.drawEllipse(r)
                                painter.drawText((x * image.width()) / 1000 + DETECT_RADIUS, (y * image.height()) / 1000 - DETECT_RADIUS, str(d / 100)[0:4] + 'm')
                            painter.end()
                        finally:
                            self.tagLock.release()
            finally:
                self.imageLock.release()

            # We could  do more processing (eg OpenCV) here if we wanted to, but for now lets just display the window.
            self.resize(image.width(), image.height())
            self.imageBox.setPixmap(image)

        # Update the status bar to show the current drone status & battery level
        self.statusBar().showMessage(self.statusMessage if self.connected else self.DisconnectedMessage)

    def ReceiveImage(self, data):
        # Indicate that new data has been received (thus we are connected)
        self.communicationSinceTimer = True

        # We have some issues with locking between the GUI update thread and the ROS messaging thread due to the size of the image, so we need to lock the resources
        self.imageLock.acquire()
        
        try:

            # Inside of ReceiveImage(), create a new image in open CV by converting the ROS image:
            try:
                cv_image = self.bridge.imgmsg_to_cv(data, "rgb8")
            except CvBridgeError, e:
                print e
             
            if g_tuning_flag:
                self.FilterImageByRange(cv_image)
                cv.WaitKey()
                
            # Convert the cv image to a numpy array
            the_array = np.array(cv_image, dtype=np.uint8)

            # Convert image from RGB to HSV
            np_img_hsv = cv2.cvtColor(the_array, cv.CV_RGB2HSV)
            # Display HSV image
#            cv2.imshow("HSV Image",np_img_hsv)
            
            # Convert cv image to binary image based on HSV Lower & Upper bounds
#             binary_img = cv2.inRange(np_img_hsv, np.array([1, 111, 234], np.uint8), np.array([6, 190, 255], np.uint8))
            binary_img = cv2.inRange(np_img_hsv, np.array([g_hue_low, g_sat_low, g_val_low], np.uint8), np.array([g_hue_high, g_sat_high, g_val_high], np.uint8))

            # Display binary image
#            cv2.imshow("Binary Image",binary_img)

            # Erode the binary image
            erode = cv2.erode(binary_img, None, iterations=1)
            # Display eroded image
#            cv2.imshow("Erode Image", erode)

            # Dilate the eroded image
            dilate = cv2.dilate(erode, None, iterations=5)
            # Display dilated image
#             cv2.imshow("Dilate Image", dilate)
            
            # Calculate contours of dilated image
            contours, hierarchy = cv2.findContours(dilate, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            
            try:
                # Sort the contours by size
                contours = sorted(contours, key=lambda contour:cv2.contourArea(contour), reverse=True)
                try:
                    contours = contours[:6]
                except:
                    pass
                
                # Initialize entity array which will store (x, y, size) of each of the (up to 6) conours it identifies
                entities = []

                cnt_set = []
                largest = cv2.contourArea(contours[0])
                for cnt in contours:
                    if cv2.contourArea(cnt) > largest * 0.5:
                        cnt_set.append(cnt)
                
                for cnt in cnt_set:
                    # Calculate minimum rectangle for the contour
                    rect = cv2.minAreaRect(cnt)
                    box = cv2.cv.BoxPoints(rect)
                    box = np.int0(box)
                
                    # Draw rotated rectangle
                    cv2.drawContours(the_array, [box], 0, (0, 0, 255), 2)
                    
                    moments = cv2.moments(cnt)
                    # Calculate centroid
                    centroid_x = int(moments['m10'] / moments['m00'])
                    centroid_y = int(moments['m01'] / moments['m00'])
                    
                    # Draw circle at the center of mass
                    cv2.circle(the_array, (centroid_x, centroid_y), 3, (255, 0, 0), 1)
                
                    # Print x, y coordinates of center of mass
                    cv2.putText(the_array, "x = " + str(centroid_x) + ", y = " + str(centroid_y) + ".", (abs(centroid_x), abs(centroid_y + 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.3, 255)
                    
                    # Add entity to list of entities
                    #entities.append((centroid_x, centroid_y, cv2.contourArea(cnt)))
                    entities.append((centroid_x, centroid_y))

                # Draw exact contour to final image
                #cv2.drawContours(the_array, cnt_set, -1, (0, 255, 0), 2)

                # Publish entities
                self.pubEntities.publish(repr(entities))
                # Define what the decision publisher does
            except:
                pass
            
            # Convert numpy array to cv image message
            src = cv.fromarray(the_array)
            cv.Convert(src, cv_image)

            # Convert cv image message to ros image message 
            ros_img = self.bridge.cv_to_imgmsg(cv_image, encoding="rgb8")
            self.image = ros_img
        finally:
            self.imageLock.release()

    def ReceiveNavdata(self, navdata):
        # Indicate that new data has been received (thus we are connected)
        self.communicationSinceTimer = True

        # Update the message to be displayed
        msg = self.StatusMessages[navdata.state] if navdata.state in self.StatusMessages else self.UnknownMessage
        self.statusMessage = '{} (Battery: {}%)'.format(msg, int(navdata.batteryPercent))

        self.tagLock.acquire()
        try:
            if navdata.tags_count > 0:
                self.tags = [(navdata.tags_xc[i], navdata.tags_yc[i], navdata.tags_distance[i]) for i in range(0, navdata.tags_count)]
            else:
                self.tags = []
        finally:
            self.tagLock.release()
            
    def FilterImageByRange(self, image, color_range=cv.CV_RGB2HSV, top_range=180, window_name = "temp_window"):
        image2 = cv.CreateImage(cv.GetSize(image), 8, 3)
        thresh = cv.CreateImage(cv.GetSize(image), 8, 1)
        def FilterCallback(value):
            h = cv.GetTrackbarPos("h", window_name)
            s = cv.GetTrackbarPos("s", window_name)
            v = cv.GetTrackbarPos("v", window_name)
            h2 = cv.GetTrackbarPos("h2", window_name)
            s2 = cv.GetTrackbarPos("s2", window_name)
            v2 = cv.GetTrackbarPos("v2", window_name)
            lowerBound = cv.Scalar(h,s,v)
            upperBound = cv.Scalar(h2,s2,v2)
            cv.CvtColor(image, image2, color_range)
            cv.InRangeS(image2, lowerBound, upperBound, thresh)
            cv.ShowImage(window_name, thresh)
        cv.NamedWindow(window_name, cv.CV_WINDOW_NORMAL)
        cv.CreateTrackbar( "h", window_name, 0, top_range, FilterCallback)
        cv.CreateTrackbar( "s", window_name, 0, 255, FilterCallback)
        cv.CreateTrackbar( "v", window_name, 0, 255, FilterCallback)
        cv.CreateTrackbar( "h2", window_name, top_range, top_range, FilterCallback)
        cv.CreateTrackbar( "s2", window_name, 255, 255, FilterCallback)
        cv.CreateTrackbar( "v2", window_name, 255, 255, FilterCallback)
        cv.WaitKey()

if __name__ == '__main__':
    import sys
    rospy.init_node('ardrone_video_display')
    app = QtGui.QApplication(sys.argv)
    display = DroneVideoDisplay()
    display.show()
    status = app.exec_()
    rospy.signal_shutdown('Great Flying!')
    sys.exit(status)
