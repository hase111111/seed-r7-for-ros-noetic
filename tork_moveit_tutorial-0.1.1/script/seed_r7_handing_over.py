#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy
import math
import copy
import tf
import time
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Quaternion, Pose, PoseStamped, Vector3

##-- for hand control
from seed_r7_ros_controller.srv import*

##-- for objects
from geometry_msgs.msg import TransformStamped

#################################################
#Main Part
#################################################

class Motion():
  def __init__(self):
    #Configuration for MoveIt!
    robot = moveit_commander.RobotCommander()

    #Store the objects of each group name
    self.upper_body = moveit_commander.MoveGroupCommander("upper_body")
    self.lifter = moveit_commander.MoveGroupCommander("lifter")
    self.rarm_with_torso = moveit_commander.MoveGroupCommander("rarm_with_torso")

    self.upper_body.set_max_velocity_scaling_factor(1)
    self.lifter.set_max_velocity_scaling_factor(1)

    rospy.loginfo('waiting service')
    rospy.wait_for_service('/seed_r7_ros_controller/hand_control')
    self.service = rospy.ServiceProxy('/seed_r7_ros_controller/hand_control', HandControl)

    self.tf_listener_ = tf.TransformListener()
    self.br = tf.TransformBroadcaster()

  def display_target(self,pose,_parent="base_link"):
    position = [pose.position.x,pose.position.y,pose.position.z]
    orientation = [pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
    self.br.sendTransform(position, orientation, rospy.Time.now(),"target", _parent)
    rospy.loginfo("set target pose")

  def get_transform(self,_parent,_child):
    object_pose = Pose()
    while(not rospy.is_shutdown()):
      try:
        (position, quaternion) \
          = self.tf_listener_.lookupTransform(_parent,_child, rospy.Time(0) )
        break
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

    object_pose.position.x = position[0]
    object_pose.position.y = position[1]
    object_pose.position.z = position[2]

    object_pose.orientation.x = quaternion[0]
    object_pose.orientation.y = quaternion[1]
    object_pose.orientation.z = quaternion[2]
    object_pose.orientation.w = quaternion[3]

    return object_pose

  def grasp(self,on):
    try:
      if(on):
        response = self.service(0,'grasp',100)
        #response = self.service(1,'grasp',100)
      else:
        response = self.service(0,'release',100)
        #response = self.service(1,'release',100)
      return True
    except rospy.ServiceException as e:
      rospy.logerr('Service call failed: {}'.format(e))
      return False

  def init_pose(self,lifter=True,top=True):
    #Set Pose to Home Position
    joint_length = len(self.upper_body.get_current_joint_values())
    self.upper_body.set_joint_value_target(joint_length * [0]) #all joints are initialized at 0
    self.upper_body.set_joint_value_target('r_elbow_joint',-2.8)
    self.upper_body.set_joint_value_target('l_elbow_joint',-2.8)
    self.upper_body.go(wait=True)

    if(lifter and top):
      self.lifter.set_joint_value_target('ankle_joint',0)
      self.lifter.set_joint_value_target('knee_joint',0)
      self.lifter.go(wait=True)
    elif(lifter and not top):
      self.lifter.set_joint_value_target('ankle_joint',0.785)
      self.lifter.set_joint_value_target('knee_joint',-0.785)
      self.lifter.go(wait=True)

  def hello(self):
    self.upper_body.set_joint_value_target('r_elbow_joint',-2.0)
    self.upper_body.set_joint_value_target('l_elbow_joint',-2.8)
    self.upper_body.set_joint_value_target('r_shoulder_p_joint',-0.9)
    self.upper_body.set_joint_value_target('r_shoulder_r_joint',0)
    self.upper_body.set_joint_value_target('r_shoulder_y_joint',-0.5)
    self.upper_body.go(wait=True)

    self.upper_body.set_joint_value_target('waist_y_joint',0.5)
    self.upper_body.go(wait=True)
    for i in range(0,2):
      self.upper_body.set_joint_value_target('r_shoulder_y_joint',0.3)
      self.upper_body.go(wait=True)
      self.upper_body.set_joint_value_target('r_shoulder_y_joint',-0.3)
      self.upper_body.go(wait=True)

    self.upper_body.set_joint_value_target('waist_y_joint',-0.5)
    self.upper_body.go(wait=True)
    for i in range(0,2):
      self.upper_body.set_joint_value_target('r_shoulder_y_joint',0.3)
      self.upper_body.go(wait=True)
      self.upper_body.set_joint_value_target('r_shoulder_y_joint',-0.3)
      self.upper_body.go(wait=True)

    self.init_pose(lifter=False)

  def set_grasp_position(self, x, y, z, vel=1.0,direction="side",ik="torso"):
    if(ik=="torso"): self.group = moveit_commander.MoveGroupCommander("rarm_with_torso")
    elif(ik=="waist"): self.group = moveit_commander.MoveGroupCommander("rarm_with_waist")
    elif(ik=="arm"): self.group = moveit_commander.MoveGroupCommander("rarm")

    self.group.set_pose_reference_frame("base_link")
    self.group.set_planner_id( "RRTConnectkConfigDefault" )
    self.group.allow_replanning( True )

    target_pose = Pose()
    if(direction == "side"):
      self.group.set_end_effector_link("r_eef_grasp_link")
      quat = tf.transformations.quaternion_from_euler(0,0,0)
    elif(direction == "top"):
      self.group.set_end_effector_link("r_eef_pick_link")
      quat = tf.transformations.quaternion_from_euler(-1.57,0.79,0)

    target_pose.orientation.x = quat[0]
    target_pose.orientation.y = quat[1]
    target_pose.orientation.z = quat[2]
    target_pose.orientation.w = quat[3]

    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z

    self.group.set_pose_target(target_pose)
    self.group.set_max_velocity_scaling_factor(vel)
    plan = self.group.plan()
    if type(plan) is tuple: # for noetic
        plan = plan[1]

    if(len(plan.joint_trajectory.points)==0):
      rospy.logwarn("IK can't be solved")
      self.group.clear_pose_targets()
      return 'aborted'
    else:
      self.group.execute(plan)
      return 'succeeded'

  def work(self):
    TargetPosition1 = [ 0.3, -0.3, 1.1 ]
    TargetPosition2 = [ 0.3, -0.3, 1.0 ]
    TargetPosition3 = [ 0.3, -0.25, 1.0 ]
    TargetPosition4 = [ 0.3, -0.25, 1.1 ]
    self.set_grasp_position(TargetPosition1[0], TargetPosition1[1], TargetPosition1[2], ik="arm")
    self.set_grasp_position(TargetPosition2[0], TargetPosition2[1], TargetPosition2[2], ik="arm")
    self.set_grasp_position(TargetPosition3[0], TargetPosition3[1], TargetPosition3[2], ik="arm")
    self.set_grasp_position(TargetPosition4[0], TargetPosition4[1], TargetPosition4[2], ik="arm")
    self.set_grasp_position(TargetPosition1[0], TargetPosition1[1], TargetPosition1[2], ik="arm")
    self.init_pose()

if __name__ == '__main__':
  rospy.init_node('motion_node')

  motion = Motion()

  try:
    motion.init_pose()
    motion.work()
  except rospy.ROSInterruptException:
    pass
