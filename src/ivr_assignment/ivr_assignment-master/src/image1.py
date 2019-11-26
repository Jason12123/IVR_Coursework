#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image1_processing', anonymous=True)
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    
    # initialize a publisher to send robot spheres position y
    self.spheres_y_pub = rospy.Publisher("spheres_position_y",Float64MultiArray, queue_size=10)
    # initialize a publisher to send robot spheres position z
    self.spheres_z_pub = rospy.Publisher("spheres_position_z",Float64MultiArray, queue_size=10)
    self.target_y_pub = rospy.Publisher("target_position_y",Float64, queue_size=10)
    self.obstacle_y_pub = rospy.Publisher("obstacle_position_y",Float64, queue_size=10)
    self.target_z_pub = rospy.Publisher("target_position_z",Float64, queue_size=10)
    self.obstacle_z_pub = rospy.Publisher("obstacle_position_z",Float64, queue_size=10)
    # initialize a subscriber to recieve messages from a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    
    # initialize publishers to send joints' angular position to the robot
    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
    
    # initialize pixel2meter related parameters
    self.pixel2meter_flag = 3
    self.pixel2meter_ratio = 1.0
    
    # initialize start flag
    self.initIter = 30

    # initialize position parameters
    self.center_y = 0.0
    self.circle1Pos_y = 1.0
    self.circle2Pos_y = 1.0
    self.circle3Pos_y = 1.0
    self.center_z = 0.0
    self.circle1Pos_z = 1.0
    self.circle2Pos_z = 1.0
    self.circle3Pos_z = 1.0
    self.obstaclePos_y = 1.0
    self.targetPos_y = 1.0
    self.obstaclePos_z = 1.0
    self.targetPos_z = 1.0
  ######################################################################
  def detect_red (self, image):
    # Image thresholding
    mask = cv2.inRange(image, (0,0,100), (0,0,255))
    # Morphological transformations
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    # Obtain the moments
    try: 
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
    except:
      cx = 0
      cy = 0
    return np.array([cx,cy])
  ######################################################################  
  def detect_yellow (self, image):
    # Image thresholding
    mask = cv2.inRange(image, (0,100,100), (0,255,255))
    # Morphological transformations
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    # Obtain the moments
    try: 
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
    except:
      cx = 0
      cy = 0
    return np.array([cx,cy])
  ######################################################################  
  def detect_blue (self, image):
    # Image thresholding
    mask = cv2.inRange(image, (100,0,0), (255,0,0))
    # Morphological transformations
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    # Obtain the moments
    try: 
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
    except:
      cx = 0
      cy = 0
    return np.array([cx,cy])
  ######################################################################  
  def detect_green (self, image):
    # Image thresholding
    mask = cv2.inRange(image, (0,100,0), (0,255,0))
    # Morphological transformations
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    # Obtain the moments
    try: 
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
    except:
      cx = 0
      cy = 0
    return np.array([cx,cy])
  ######################################################################  
  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)

    im1=cv2.imshow('window1', self.cv_image1)
    cv2.waitKey(1)
    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
    except CvBridgeError as e:
      print(e)

# call the class
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


