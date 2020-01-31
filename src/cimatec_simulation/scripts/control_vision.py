#!/usr/bin/env python2.7
import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray
from actionlib_msgs.msg import GoalID

from sensor_msgs.msg import CameraInfo

import time

from control_pid import ControlPid

import math

class ControlVision:
  control_pid_x = None
  control_pid_yaw = None
  pub_cmd_vel = None
  msg_twist = None
  camera_info = None
  pub_quaternion = None
  odometry_data = None
  rpy_angle = None
  flag_move_to_goal = False
  flag_orientation = True
  flag_ajustment = False
  flag_find = False
  pub_move_to_goal = None
  msg_move_to_goal = None
  move_base_info = None
  
  def __init__ (self):
    rospy.loginfo("INIT CONTROL VISION")
    rospy.init_node("robot_vision", anonymous=True)
    self.control_pid_x = ControlPid(5, -5, 0.01, 0, 0)
    self.control_pid_yaw = ControlPid(3, -3, 0.001, 0, 0)
    self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    self.msg_twist = Twist()
    self.pub_quaternion = rospy.Publisher("/rotation_quaternion", Quaternion, queue_size=1)
    self.pub_move_to_goal = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=1)
    self.msg_move_to_goal = MoveBaseActionGoal()
    rospy.Subscriber("/odometry/filtered", Odometry, self.callback_odometry)
    rospy.Subscriber("/rpy_angles", Vector3, self.callback_rpy_angles)
    rospy.Subscriber("/diff/camera_top/camera_info", CameraInfo, self.callback_camera_info)
    rospy.Subscriber("move_base/status", GoalStatusArray, self.callback_move_base_info)
    
  def publisher_move_to_goal(self, data):
    rospy.loginfo("Entrou no move base")
    factor_x = 1 if (self.rpy_angle.z <= 0 and self.rpy_angle.z >= -1.57) or self.rpy_angle.z >= 0 and self.rpy_angle.z <= 1.57 else -1
    factor_y = 1 if self.rpy_angle.z >= 0 and self.rpy_angle.z <= 3.14 else -1
    angle = self.rpy_angle.z if self.rpy_angle.z >= 0 else self.rpy_angle.z * -1
    self.msg_move_to_goal.goal.target_pose.pose.position.x = self.odometry_data.pose.pose.position.x + (data.y * math.cos(angle)) * factor_x
    self.msg_move_to_goal.goal.target_pose.pose.position.y = self.odometry_data.pose.pose.position.y + (data.y * math.sin(angle)) * factor_y
    self.msg_move_to_goal.goal.target_pose.header.frame_id = 'odom'
    self.msg_move_to_goal.goal.target_pose.pose.orientation.z = self.odometry_data.pose.pose.orientation.z
    self.msg_move_to_goal.goal.target_pose.pose.orientation.w = self.odometry_data.pose.pose.orientation.w
    self.pub_move_to_goal.publish(self.msg_move_to_goal)

  def orientation_to_obj(self, data):
    self.msg_twist.angular.z = self.control_pid_yaw.pid_calculate(0.5, self.camera_info.width/2, int(data.x))
    self.pub_cmd_vel.publish(self.msg_twist)
    rospy.loginfo(round(self.msg_twist.angular.z, 1))
    if round(self.msg_twist.angular.z, 1) == 0:
      self.flag_orientation = False
      self.flag_move_to_goal = True

  def goal_ajustment(self, data):
    self.msg_twist.angular.z = self.control_pid_yaw.pid_calculate(0.5, self.camera_info.width/2, int(data.x))
    self.msg_twist.linear.x = self.control_pid_x.pid_calculate(0.5, 180, int(data.z))
    self.pub_cmd_vel.publish(self.msg_twist)
    if round(self.msg_twist.angular.z, 1) == 0 and round(self.msg_twist.linear.x, 1) == 0:
      self.flag_ajustment = False
      self.flag_orientation = True

  def callback(self, data):
    if data.x != -1:
      if not self.move_base_info.status_list and self.flag_orientation:
        self.orientation_to_obj(data)
      elif self.flag_move_to_goal:
        self.publisher_move_to_goal(data)
        self.flag_move_to_goal = False
        self.flag_ajustment = True
        self.flag_find = True
      elif (self.move_base_info.status_list and self.move_base_info.status_list[0].status != 1) and self.flag_ajustment:
        self.goal_ajustment(data)

      if (self.move_base_info.status_list and self.move_base_info.status_list[0].status == 1) and data.y <= 4:
        rospy.Publisher('/move_base/cancel', GoalID, queue_size=1).publish(GoalID())
      
    else:
      if self.flag_find and (self.move_base_info.status_list and self.move_base_info.status_list[0].status != 1):
        self.msg_twist.angular.z = 0.5
        self.pub_cmd_vel.publish(self.msg_twist.angular.z)

  def callback_camera_info(self, data):
    self.camera_info = data
  
  def callback_odometry(self, data):
    self.odometry_data = data
    quaternion = Quaternion()
    quaternion.x = data.pose.pose.orientation.x
    quaternion.y = data.pose.pose.orientation.y
    quaternion.z = data.pose.pose.orientation.z
    quaternion.w = data.pose.pose.orientation.w
    self.pub_quaternion.publish(quaternion)
  
  def callback_rpy_angles(self, data):
    self.rpy_angle = data

  def callback_move_base_info(self, data):
    self.move_base_info = data

  def run(self):
    self.msg = rospy.Subscriber("/camera/obj/coordinates", Vector3, self.callback)

if __name__ == "__main__":
  rospy.loginfo("Init Control")
  ctrl_vision = ControlVision()
  ctrl_vision.run()
  while not rospy.is_shutdown():
    rospy.spin()    