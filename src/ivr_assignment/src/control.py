#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
import math
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class robot_controller:
  ############################################################################
  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named control
    rospy.init_node('control', anonymous=True)
    
    self.debug_pub = rospy.Publisher("debug", Float64, queue_size=10)
    
    # initialize a publisher to send joints' angular position to a topic called joints_pos
    self.joints_pub = rospy.Publisher("joints_pos",Float64MultiArray, queue_size=10)

    # initialize a publisher to send end_effector position
    self.end_effector_pub = rospy.Publisher("end_effector_pos",Float64MultiArray, queue_size=10)
    
    
    # initialize publishers to send joints' angular position to the robot
    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)


    # initialize subscribers to recieve messages from topics named "spheres_position_x", "spheres_position_y" and "spheres_position_z", and use callback function to recieve data
    self.x_sub = rospy.Subscriber("spheres_position_x",Float64MultiArray,self.callback_x)
    self.y_sub = rospy.Subscriber("spheres_position_y",Float64MultiArray,self.callback_y)
    self.z_sub = rospy.Subscriber("spheres_position_z",Float64MultiArray,self.callback_z)
    
    # initialize subscribers to recieve targets' position from topics named "target_position_x" "target_position_y" "target_position_z"
    self.target_x_sub = rospy.Subscriber("target_position_x",Float64,self.callback_target_x)
    self.target_y_sub = rospy.Subscriber("target_position_y",Float64,self.callback_target_y)
    self.target_z_sub = rospy.Subscriber("target_position_z",Float64,self.callback_target_z)
    
    
    # initialize the normal vector
    self.n = np.array([0,1,0])
    
    # initialize theta to avoid singularity 
    self.theta = np.array([0.0,0.0,0.0,0.0])
    
    # initialize start flag
    self.initIter = 30

    # initialize errors
    self.time_previous_step = np.array([rospy.get_time()], dtype='float64')
    self.error = np.array([0.0,0.0,0.0], dtype='float64')
    self.error_d = np.array([0.0,0.0,0.0], dtype='float64')
    
    #
    self.q_d_previous = np.array([1.0,1.0,1.0,1.0], dtype='float64')
    self.pid_p = 0.0
    self.pid_d = 0.0
    
    # record the begining time
    self.time_trajectory = rospy.get_time()
  
  
  ############################################################################
      

# call the class
def main(args):
  ic = robot_controller()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


