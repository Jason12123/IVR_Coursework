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
  def calculate_joint_angles(self,X,Y,Z):
    
    try:
      #l1 = np.array([X[1]-X[0], Y[1]-Y[0], Z[1]-Z[0]])
      l1 = np.array([0, 0, 1])
      l2 = np.array([X[2]-X[1], Y[2]-Y[1], Z[2]-Z[1]])
      l3 = np.array([X[3]-X[2], Y[3]-Y[2], Z[3]-Z[2]])
      
      # Calculate normal vector
      if (math.acos(l2.dot(l3)/(np.linalg.norm(l2) * np.linalg.norm(l3)))<0.1):
        print("Singular! theta4 is 0")
      else:
        #print(math.acos(l2.dot(l3)/(np.linalg.norm(l2) * np.linalg.norm(l3))))
        aa = np.cross(l2, np.cross(l2,l3))
        aa = aa/np.linalg.norm(aa)
        #if(np.linalg.norm(aa - self.n)<1):
        if(aa.dot(self.n)>0):
          self.n = aa
        else:
          self.n = -aa

      # Calculate theta
      if (l3.dot(self.n)<0):
        self.theta[3] = math.acos(l2.dot(l3)/(np.linalg.norm(l2) * np.linalg.norm(l3)))
      else:
        self.theta[3] = -math.acos(l2.dot(l3)/(np.linalg.norm(l2) * np.linalg.norm(l3)))
        
      self.theta[1] = math.pi/2 - math.acos(self.n.dot(l1)/(np.linalg.norm(self.n) * np.linalg.norm(l1)))
      
      self.theta[0] = math.atan(-self.n[0]/self.n[1])
      
      bb = np.cross(self.n, l1)
      try:
        self.theta[2] = math.pi/2 - math.acos(bb.dot(l2)/(np.linalg.norm(bb) * np.linalg.norm(l2)))
      except:
        print("Singular! theta3 is +-pi/2")
      
    except:
      print("angle computing failed")
      
    return np.array([self.theta[0],self.theta[1],self.theta[2],self.theta[3]])
  
  ############################################################################
  def forward_kinematics(self, joint_angles):
    L1 = 2
    L2 = 3
    L3 = 2
    
    x_e = L2*np.cos(joint_angles[2])*np.sin(joint_angles[0])*np.sin(joint_angles[1]) + L2*np.cos(joint_angles[0])*np.sin(joint_angles[2]) + L3*np.cos(joint_angles[3])*(np.cos(joint_angles[2])*np.sin(joint_angles[0])*np.sin(joint_angles[1]) + np.cos(joint_angles[0])*np.sin(joint_angles[2])) + L3*np.cos(joint_angles[1])*np.sin(joint_angles[0])*np.sin(joint_angles[3])

    y_e = -L2*np.cos(joint_angles[0])*np.cos(joint_angles[2]/end_effector_pos)*np.sin(joint_angles[1]) + L2*np.sin(joint_angles[0])*np.sin(joint_angles[2]) + L3*np.cos(joint_angles[3])*(-np.cos(joint_angles[0])*np.cos(joint_angles[2])*np.sin(joint_angles[1]) + np.sin(joint_angles[0])*np.sin(joint_angles[2])) - L3*np.cos(joint_angles[0])*np.cos(joint_angles[1])*np.sin(joint_angles[3])

    z_e = L1 + L2*np.cos(joint_angles[1])*np.cos(joint_angles[2]) + L3*np.cos(joint_angles[1])*np.cos(joint_angles[2])*np.cos(joint_angles[3]) - L3*np.sin(joint_angles[1])*np.sin(joint_angles[3])
    #print(x_e,y_e,z_e)
    return np.array([x_e,y_e,z_e])
    
  ############################################################################
  def calculate_jacobian(self, joint_angles):
    L1 = 2
    L2 = 3
    L3 = 2
    
    dx_dtheta1=(L2+L3*np.cos(joint_angles[3]))*np.sin(joint_angles[2])*-np.sin(joint_angles[0])+(np.cos(joint_angles[2])*(L2+L3*np.cos(joint_angles[3]))*np.sin(joint_angles[1])+L3*np.cos(joint_angles[1])*np.sin(joint_angles[3]))*np.cos(joint_angles[0])

    dx_dtheta2=np.sin(joint_angles[0])*(L3*np.sin(joint_angles[3])*-np.sin(joint_angles[1])+np.cos(joint_angles[2])*(L2+L3*np.cos(joint_angles[3]))*np.cos(joint_angles[1]))

    dx_dtheta3=(L2+L3*np.cos(joint_angles[3]))*(np.sin(joint_angles[0])*np.sin(joint_angles[1])*-np.sin(joint_angles[2])+np.cos(joint_angles[0])*np.cos(joint_angles[2]))

    dx_dtheta4=L3*(np.cos(joint_angles[2])*np.sin(joint_angles[0])*np.sin(joint_angles[1])+np.cos(joint_angles[0])*np.sin(joint_angles[2]))*-np.sin(joint_angles[3])+L3*np.cos(joint_angles[1])*np.sin(joint_angles[0])*np.cos(joint_angles[3])

    dy_dtheta1=-(np.cos(joint_angles[2])*(L2+L3*np.cos(joint_angles[3]))*np.sin(joint_angles[1])+L3*np.cos(joint_angles[1])*np.sin(joint_angles[3]))*-np.sin(joint_angles[0])+(L2+L3*np.cos(joint_angles[3]))*np.sin(joint_angles[2])*np.cos(joint_angles[0])

    dy_dtheta2=-np.cos(joint_angles[0])*(L3*np.sin(joint_angles[3])*-np.sin(joint_angles[1])+np.cos(joint_angles[2])*(L2+L3*np.cos(joint_angles[3]))*np.cos(joint_angles[1]))

    dy_dtheta3=-(L2+L3*np.cos(joint_angles[3]))*(np.cos(joint_angles[0])*np.sin(joint_angles[1])*-np.sin(joint_angles[2])-np.sin(joint_angles[0])*np.cos(joint_angles[2]))

    dy_dtheta4=L3*np.sin(joint_angles[0])*np.sin(joint_angles[2])*-np.sin(joint_angles[3])-L3*np.cos(joint_angles[0])*(np.cos(joint_angles[2])*np.sin(joint_angles[1])*-np.sin(joint_angles[3])+np.cos(joint_angles[1])*np.cos(joint_angles[3]))

    dz_dtheta1=0

    dz_dtheta2=np.cos(joint_angles[2])*(L2+L3*np.cos(joint_angles[3]))*-np.sin(joint_angles[1])-L3*np.sin(joint_angles[3])*np.cos(joint_angles[1])

    dz_dtheta3=np.cos(joint_angles[1])*(L2+L3*np.cos(joint_angles[3]))*-np.sin(joint_angles[2])

    dz_dtheta4=L3*(np.cos(joint_angles[1])*np.cos(joint_angles[2])*-np.sin(joint_angles[3])-np.sin(joint_angles[1])*np.cos(joint_angles[3]))
    
    
    
    jacobian = np.array([[dx_dtheta1,dx_dtheta2,dx_dtheta3,dx_dtheta4], [dy_dtheta1,dy_dtheta2,dy_dtheta3,dy_dtheta4], [dz_dtheta1, dz_dtheta2, dz_dtheta3, dz_dtheta4]])
    
    #print(np.linalg.norm(jacobian))
    #print(jacobian)
    #self.debug = Float64()
    #self.debug.data = np.linalg.norm(jacobian)
    #self.debug_pub.publish(self.debug)
    
    return jacobian
    
  ############################################################################
  def control_incomplete_differential(self, joint_angles,end_effector_position):
    KP = 10
    KD = 0.08
    ALPHA = 0
    
    k_p = np.array([[KP,0,0], [0,KP,0], [0,0,KP]])
    k_d = np.array([[KD,0,0], [0,KD,0], [0,0,KD]])
    
    cur_time = np.array([rospy.get_time()])
    dt = cur_time - self.time_previous_step
    self.time_previous_step = cur_time
    
    pos = end_effector_position
    try:
      norm = np.linalg.norm(np.array([self.target_x,self.target_y,(self.target_z-2)]))
      # keep away from singularity
      if norm > 4.5:
        pos_target = 4.5/norm*np.array([self.target_x,self.target_y,self.target_z-2])+np.array([0,0,2])
      else:
        pos_target = np.array([self.target_x,self.target_y,self.target_z])
          
      self.error_d = ((pos_target - pos) - self.error) / dt
      self.error = pos_target - pos
    
      try:
        J_inv = np.linalg.pinv(self.calculate_jacobian(joint_angles))
        
        self.pid_p = np.dot(k_p, self.error.transpose())
        self.pid_d = np.dot((1-ALPHA)*k_d, self.error_d.transpose()) + ALPHA*self.pid_d
        dq_d = np.dot( J_inv, (self.pid_p + self.pid_d) )

        q_d = joint_angles+(dt*dq_d)  
        
        self.q_d_previous = q_d
      except:
        print("J_inv compute failed")
        q_d = self.q_d_previous
        
    except:
      print("Haven't recieved target_position")
    
    return q_d
    
  ############################################################################
  def control_closed(self, joint_angles,end_effector_position):
    KP = 10
    KD = 0.1
    k_p = np.array([[KP,0,0], [0,KP,0], [0,0,KP]])
    k_d = np.array([[KD,0,0], [0,KD,0], [0,0,KD]])
    
    cur_time = np.array([rospy.get_time()])
    dt = cur_time - self.time_previous_step
    self.time_previous_step = cur_time
    
    pos = end_effector_position
    try:
      pos_target = np.array([self.target_x,self.target_y,self.target_z])
    except:
      print("Haven't recieved target_position")

    self.error_d = ((pos_target - pos) - self.error) / dt
    self.error = pos_target - pos
    
    J_inv = np.linalg.pinv(self.calculate_jacobian(joint_angles))
    dq_d = np.dot( J_inv, ( np.dot(k_d, self.error_d.transpose()) + np.dot(k_p, self.error.transpose()) ) )
    
    q_d = joint_angles+(dt*dq_d)
    
    # LOCK IF SINGULAR
    #if(q_d[3]<0.01):
      #q_d = self.q_d_previous

    self.q_d_previous = q_d
    
    return q_d 


  ############################################################################    
  def callback_target_x(self,msg):
    try:
      self.target_x = msg.data
    except:
      print("Not recieving target_position_x")
  ############################################################################    
  def callback_target_y(self,msg):
    try:
      self.target_y = msg.data
    except:
      print("Not recieving target_position_y")
  ############################################################################    
  def callback_target_z(self,msg):
    try:
      self.target_z = msg.data
    except:
      print("Not recieving target_position_z")
  ############################################################################
  # Recieve data, process it, and publish
  def callback_x(self,msg):
    # Recieve spheres_position_x
    try:
      self.spheres_x = msg.data
      #print("1")
    except:
      print("Not recieving spheres_position_x")
  ############################################################################    
  def callback_y(self,msg):
    # Recieve spheres_position_y
    try:
      self.spheres_y = msg.data
    except:
      print("Not recieving spheres_position_y")
  ############################################################################
  # Calculate joint angles and ...    
  def callback_z(self,msg):
    
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
      # Recieve spheres_position_z
      try:
        self.spheres_z = msg.data
      except:
        print("Not recieving spheres_position_z")
        
      try:
        X = self.spheres_x
        Y = self.spheres_y
        Z = self.spheres_z
        
      except:
        print("Not recieving spheres' position")
        
      # Calculate joint angles  ###################################################
      joint_angles = self.calculate_joint_angles(X,Y,Z)
      #joint_angles[0] = 0 # LOCK FREEDOM: JOINT ESTIMATION!!!!!!!!!!!!!!!!!!!!!
      # Publishjoint angles
      self.joints = Float64MultiArray()
      self.joints.data = joint_angles
      self.joints_pub.publish(self.joints)

      # Calculate end-effector position ###########################################
      end_effector_position = np.array([X[3],Y[3],Z[3]])
      #end_effector_position = self.forward_kinematics(joint_angles)
      # Publish end-effector position
      self.end_effector = Float64MultiArray()
      #self.end_effector.data = end_effector_position
      self.end_effector.data = np.array([X[3],Y[3],Z[3]+1.05])
      self.end_effector_pub.publish(self.end_effector)
      

      # PD control ###########################################
      q_d = self.control_incomplete_differential(joint_angles,end_effector_position)
      
      #q_d = self.control_closed(joint_angles,end_effector_position)
      
      # send control commands to joints (for lab 3)
      self.joint1=Float64()
      self.joint1.data= q_d[0]
      #self.joint1.data= 0 # LOCK FREEDOM: JOINT CONTROL!!!!!!!!!!!!!!!!!!!!!
      self.joint2=Float64()
      self.joint2.data= q_d[1]
      #self.joint2.data= 0 # LOCK FREEDOM: JOINT CONTROL!!!!!!!!!!!!!!!!!!!!!
      self.joint3=Float64()
      self.joint3.data= q_d[2]
      #self.joint3.data= 0 # LOCK FREEDOM: JOINT CONTROL!!!!!!!!!!!!!!!!!!!!!
      self.joint4=Float64()
      self.joint4.data= q_d[3]
      #self.joint4.data= 0.5 # LOCK FREEDOM: JOINT CONTROL!!!!!!!!!!!!!!!!!!!!!
      
      # publish
      self.robot_joint1_pub.publish(self.joint1)
      self.robot_joint2_pub.publish(self.joint2)
      self.robot_joint3_pub.publish(self.joint3)
      self.robot_joint4_pub.publish(self.joint4)
      

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


