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
<<<<<<< HEAD
=======

>>>>>>> 4945bcb0ab263b7942ba01974bda1823137411ee
  ######################################################################
  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image2_processing', anonymous=True)
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    
    # initialize a publisher to send robot spheres position x
    self.spheres_x_pub = rospy.Publisher("spheres_position_x",Float64MultiArray, queue_size=10)
    self.target_x_pub = rospy.Publisher("target_position_x",Float64, queue_size=10)
    self.obstacle_x_pub = rospy.Publisher("obstacle_position_x",Float64, queue_size=10)
    
    # initialize a subscriber to recieve messages from a topic named /camera2/robot/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
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
    self.center_x = 0.0
    self.circle1Pos_x = 1.0
    self.circle2Pos_x = 1.0
    self.circle3Pos_x = 1.0
    self.obstaclePos_x = 1.0
    self.targetPos_x = 1.0
    
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
<<<<<<< HEAD
  ######################################################################  
  def pixel2meter(self, image):
    #circle1Pos = self.detect_blue(image)
    circle2Pos = self.detect_green(image)
    circle3Pos = self.detect_red(image)
    dist = np.sum((circle3Pos - circle2Pos)**2)
    print(circle3Pos)
    print(circle2Pos)
    print(dist)
    self.pixel2meter_flag = self.pixel2meter_flag-1
    return 2.0 / np.sqrt(dist)
  ######################################################################  
  def detect_target_obstacle_x(self, image):
    # Image thresholding
    mask = cv2.inRange(image, (85,120,120), (90,180,255))
    # Morphological transformations
    kernel = np.ones((5,5), np.uint8)
    #mask = cv2.dilate(mask, kernel, iterations=1)
    # find contours
    contours= cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]
    tar_x = 0
    obs_x = 0
    if len(contours)>0:
      for i in range(len(contours)):
        hull = cv2.convexHull(contours[i])
        if len(hull)<6:
          obs_x = (sum(hull)/len(hull))[0][0]
        elif len(hull)>8:
          x,y,w,h = cv2.boundingRect(contours[i])
          tar_x = x+w//2
    return tar_x, obs_x
  ######################################################################
  def detect_position_x(self,image):
    if(self.pixel2meter_flag!=0):
      self.pixel2meter_ratio = self.pixel2meter(image)
    # Obtain the centre of each blob
    self.center_x = 0.0
    yellow_x = self.detect_yellow(image)[0]
    blue_x = self.detect_blue(image)[0]
    green_x = self.detect_green(image)[0]
    red_x = self.detect_red(image)[0]
    if (blue_x != 0):
      self.circle1Pos_x = self.pixel2meter_ratio * (blue_x - yellow_x)
    if (green_x != 0):
      self.circle2Pos_x = self.pixel2meter_ratio * (green_x - yellow_x)
    if (red_x != 0):
      self.circle3Pos_x = self.pixel2meter_ratio * (red_x - yellow_x)
    
    tar_x, obs_x = self.detect_target_obstacle_x(image)
    if(tar_x != 0):
      self.targetPos_x = self.pixel2meter_ratio * (tar_x - yellow_x)
    if(obs_x != 0):
      self.obstaclePos_x = self.pixel2meter_ratio * (obs_x - yellow_x)
    
    return np.array([self.center_x,self.circle1Pos_x,self.circle2Pos_x,self.circle3Pos_x]), self.targetPos_x, self.obstaclePos_x
    



  ######################################################################
  # Recieve data from camera 2, process it, and publish
=======
  ######################################################################
  # Recieve data, process it, and publish
>>>>>>> 4945bcb0ab263b7942ba01974bda1823137411ee
  def callback2(self,data):
    # Recieve the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
<<<<<<< HEAD
      
    # initialize
    if(self.initIter>0):
      self.robot_joint1_pub.publish(0)
      self.robot_joint2_pub.publish(0.0)
      self.robot_joint3_pub.publish(0.0)
      self.robot_joint4_pub.publish(0.0)
      self.n = np.array([0,1,0])
      self.initIter = self.initIter-1
      print("initIter = {}".format(self.initIter))
    
    else:
      spheresPosition_x, targetPosition_x, obstaclePosition_x = self.detect_position_x(self.cv_image2)
      
      
      im2=cv2.imshow('window2', self.cv_image2)
      cv2.waitKey(1)
      
      self.spheres_x = Float64MultiArray()
      self.spheres_x.data = spheresPosition_x
      
      self.target_x = Float64()
      self.target_x.data = targetPosition_x
      
      self.obstacle_x = Float64()
      self.obstacle_x.data = obstaclePosition_x
      
      # Publish the results
      try: 
        self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
        self.spheres_x_pub.publish(self.spheres_x)
        self.target_x_pub.publish(self.target_x)
        self.obstacle_x_pub.publish(self.obstacle_x)
      except CvBridgeError as e:
        print(e)

######################################################################    
######################################################################
=======
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)
    im2=cv2.imshow('window2', self.cv_image2)
    cv2.waitKey(1)

    # Publish the results
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
    except CvBridgeError as e:
      print(e)

>>>>>>> 4945bcb0ab263b7942ba01974bda1823137411ee
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


