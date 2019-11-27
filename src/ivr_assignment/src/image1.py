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
  def detect_target_obstacle_yz(self, image):
    # Image thresholding
    mask = cv2.inRange(image, (85,120,120), (90,180,255))
    # Morphological transformations
    kernel = np.ones((5,5), np.uint8)
    #mask = cv2.dilate(mask, kernel, iterations=1)
    # find contours
    contours= cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]
    tar_y = 0
    obs_y = 0
    tar_z = 0
    obs_z = 0
    if len(contours)>0:
      for i in range(len(contours)):
        hull = cv2.convexHull(contours[i])
        if len(hull)<6:
          obs_y = (sum(hull)/len(hull))[0][0]
          obs_z = (sum(hull)/len(hull))[0][1]
        elif len(hull)>8:
          x,y,w,h = cv2.boundingRect(contours[i])
          tar_y = x+w//2 
          tar_z = y+h//2
          #tar_z = y+h//2 - 1 # COMPENSATION
    return tar_y, tar_z, obs_y, obs_z
  ######################################################################
  def pixel2meter(self, image):
    #circle1Pos = self.detect_blue(image)
    circle2Pos = self.detect_green(image)
    circle3Pos = self.detect_red(image)
    #dist = np.sum((circle2Pos - circle1Pos)**2)
    #print(dist)
    #pixel2meter_flag = 0
    #return 3.0 / np.sqrt(dist)
    dist = np.sum((circle3Pos - circle2Pos)**2)
    print(circle3Pos)
    print(circle2Pos)
    print(dist)
    self.pixel2meter_flag = self.pixel2meter_flag-1
    return 1.8 / np.sqrt(dist)# with compensation
  ######################################################################  
  def detect_position_yz(self,image):####################
    if(self.pixel2meter_flag!=0):
      self.pixel2meter_ratio = self.pixel2meter(image)
    # Obtain the centre of each blob
    self.center_y = 0
    self.center_z = 0
    
    yellow_y = self.detect_yellow(image)[0]
    blue_y = self.detect_blue(image)[0]
    green_y = self.detect_green(image)[0]
    red_y = self.detect_red(image)[0]
    
    yellow_z = self.detect_yellow(image)[1]
    blue_z = self.detect_blue(image)[1]
    green_z = self.detect_green(image)[1]
    red_z = self.detect_red(image)[1]
    
    if (blue_y != 0):
      self.circle1Pos_y = self.pixel2meter_ratio * (blue_y - yellow_y)
      self.circle1Pos_z = self.pixel2meter_ratio * (yellow_z - blue_z)
    if (green_y != 0):
      self.circle2Pos_y = self.pixel2meter_ratio * (green_y - yellow_y)
      self.circle2Pos_z = self.pixel2meter_ratio * (yellow_z - green_z)
    if (red_y != 0):
      self.circle3Pos_y = self.pixel2meter_ratio * (red_y - yellow_y)
      self.circle3Pos_z = self.pixel2meter_ratio * (yellow_z - red_z)
      
    tar_y, tar_z, obs_y, obs_z = self.detect_target_obstacle_yz(image)
    if(tar_y != 0):
      self.targetPos_y = self.pixel2meter_ratio * (tar_y - yellow_y)
      self.targetPos_z = self.pixel2meter_ratio * (yellow_z - tar_z)
    if(obs_y != 0):
      self.obstaclePos_y = self.pixel2meter_ratio * (obs_y - yellow_y)
      self.obstaclePos_z = self.pixel2meter_ratio * (yellow_z - obs_z)

    return np.array([self.center_y,self.circle1Pos_y,self.circle2Pos_y,self.circle3Pos_y,self.center_z,self.circle1Pos_z,self.circle2Pos_z,self.circle3Pos_z]), self.targetPos_y, self.obstaclePos_y, self.targetPos_z, self.obstaclePos_z
    
  ######################################################################
  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # initialize
    if(self.initIter>0):
      print("yeah")
      self.robot_joint1_pub.publish(0)
      self.robot_joint2_pub.publish(0.0)
      self.robot_joint3_pub.publish(0.0)
      self.robot_joint4_pub.publish(0.0)
      self.n = np.array([0,1,0])
      self.initIter = self.initIter-1
      print("initIter = {}".format(self.initIter))
    else:
      spheresPosition_yz, targetPosition_y, obstaclePosition_y, targetPosition_z, obstaclePosition_z = self.detect_position_yz(self.cv_image1)
      
      spheresPosition_y = spheresPosition_yz[0:4]
      #print(spheresPosition_y)
      spheresPosition_z = spheresPosition_yz[4:8]
      #print(spheresPosition_z)

      im1=cv2.imshow('window1', self.cv_image1)
      cv2.waitKey(1)
      
      self.spheres_y = Float64MultiArray()
      self.spheres_y.data = spheresPosition_y
      self.spheres_z = Float64MultiArray()
      self.spheres_z.data = spheresPosition_z
      
      self.target_y = Float64()
      self.target_y.data = targetPosition_y
      
      self.obstacle_y = Float64()
      self.obstacle_y.data = obstaclePosition_y
      
      self.target_z = Float64()
      self.target_z.data = targetPosition_z # WITHOUT OFFSET
      #self.target_z.data = targetPosition_z+1.05 # WITH OFFSET
      
      self.obstacle_z = Float64()
      self.obstacle_z.data = obstaclePosition_z # WITHOUT OFFSET
      #self.obstacle_z.data = obstaclePosition_z+1.05 # WITH OFFSET
      
      
      # Publish the results
      try: 
        self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
        self.spheres_y_pub.publish(self.spheres_y)
        self.spheres_z_pub.publish(self.spheres_z)
        self.target_y_pub.publish(self.target_y)
        self.obstacle_y_pub.publish(self.obstacle_y)
        self.target_z_pub.publish(self.target_z)
        self.obstacle_z_pub.publish(self.obstacle_z)
      except CvBridgeError as e:
        print(e)

######################################################################
###################################################################### 
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


