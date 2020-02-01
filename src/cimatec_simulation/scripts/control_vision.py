#!/usr/bin/env python2.7
import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import Odometry

from nav2d_navigator.msg import GetFirstMapActionGoal
from nav2d_navigator.msg import ExploreActionGoal

from actionlib_msgs.msg import GoalStatusArray
from actionlib_msgs.msg import GoalID
from actionlib_msgs.msg import GoalStatusArray

from sensor_msgs.msg import CameraInfo

import time

from control_pid import ControlPid

import math

import os

class Robot:
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
  flag_explore = False
  pub_move_to_goal = None
  msg_move_to_goal = None
  move_base_info = None

  pub_first_map_goal = None

  pub_explore_goal = None
  status_explore_goal = None
  flag = True
  time_old = None
  
  def __init__ (self):
    rospy.loginfo("INIT CONTROL VISION")
    rospy.init_node("robot_vision", anonymous=True)
    self.control_pid_x = ControlPid(5, -5, 0.01, 0, 0)
    self.control_pid_yaw = ControlPid(3, -3, 0.001, 0, 0)
    self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    self.msg_twist = Twist()
    self.pub_quaternion = rospy.Publisher("/rotation_quaternion", Quaternion, queue_size=1)
    self.pub_move_to_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    self.msg_move_to_goal = PoseStamped()

    rospy.Subscriber("/camera/obj/coordinates", Vector3, self.callback)
    rospy.Subscriber("/odometry/filtered", Odometry, self.callback_odometry)
    rospy.Subscriber("/rpy_angles", Vector3, self.callback_rpy_angles)
    rospy.Subscriber("/diff/camera_top/camera_info", CameraInfo, self.callback_camera_info)
    rospy.Subscriber("/move_base/status", GoalStatusArray, self.callback_move_base_info)
    rospy.Subscriber("/Explore/status", GoalStatusArray, self.callback_explore_status)
    
    self.pub_first_map_goal = rospy.Publisher("/GetFirstMap/goal", GetFirstMapActionGoal, queue_size=1)
    cancel_first_map = rospy.Publisher("/GetFirstMap/cancel", GoalID, queue_size=1)
    self.pub_explore_goal = rospy.Publisher("/Explore/goal", ExploreActionGoal, queue_size=1)
    time.sleep(1)
    self.pub_first_map_goal.publish()
    time.sleep(1)
    cancel_first_map.publish(GoalID())

  def callback(self, data):
    self.image = data

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

  def callback_explore_status(self, data):
    if data.status_list:
      self.status_explore_goal = data.status_list[0].status

  def move_goal_to_object(self):
    if not self.time_old or (self.time_old and time.time() - self.time_old > 10):
      distance = (1 * 937.8194580078125) / (self.image.z * 2)
      y_move_base = -(self.image.x - self.camera_info.width/2) / (self.image.z*2) 
      x_move_base = distance if abs(y_move_base) < 0.006 else math.sqrt(distance**2 - y_move_base**2)
      self.msg_move_to_goal.pose.position.x = x_move_base
      self.msg_move_to_goal.pose.position.y = y_move_base
      self.msg_move_to_goal.pose.orientation.w = 1
      self.msg_move_to_goal.header.frame_id = self.camera_info.header.frame_id
      self.pub_move_to_goal.publish(self.msg_move_to_goal)
      self.time_old = time.time()

  def goal_ajustment(self):
    while round(self.msg_twist.angular.z, 1) != 0 and round(self.msg_twist.linear.x, 1) != 0:
      self.msg_twist.angular.z = self.control_pid_yaw.pid_calculate(0.5, self.camera_info.width/2, int(self.image.x))
      self.msg_twist.linear.x = self.control_pid_x.pid_calculate(0.5, 180, int(self.image.z))
      self.pub_cmd_vel.publish(self.msg_twist)
    rospy.loginfo("Find the ball!")
    rospy.loginfo(time.ctime())
    exit

  def run(self):
    if self.image.x != -1:
      self.flag_find = True
      if (self.move_base_info.status_list and self.move_base_info.status_list[0].status == 1) and self.image.y <= 4:
        rospy.Publisher('/move_base/cancel', GoalID, queue_size=1).publish(GoalID())
        self.goal_ajustment()
      else:
        self.move_goal_to_object()

      if self.flag_explore and self.status_explore_goal == 1:
        rospy.loginfo("Stop Explore and kill Operator")
        rospy.Publisher("/Explore/cancel", GoalID, queue_size=1).publish(GoalID())
        os.system("rosnode kill /Operator")
        time.sleep(5)
        self.flag_explore = False
    else:
      if not self.flag_find and not self.flag_explore and self.status_explore_goal != 1:
        rospy.loginfo("Wait..")
        time.sleep(5)
        self.pub_explore_goal.publish(ExploreActionGoal())
        rospy.loginfo("Start Explore")
        self.flag_explore = True

if __name__ == "__main__":
  rospy.loginfo("Init Control")
  ctrl_vision = Robot()
  ctrl_vision.run()
  while not rospy.is_shutdown():
    rospy.spin()    