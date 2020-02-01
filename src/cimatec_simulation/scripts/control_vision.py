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

    # rospy.Publisher("/GetFirstMap/cancel", GoalID, queue_size=1).publish()
    
  # def publisher_move_to_goal(self, data):
  #   rospy.loginfo("Entrou no move base")
  #   factor_x = 1 if (self.rpy_angle.z <= 0 and self.rpy_angle.z >= -1.57) or self.rpy_angle.z >= 0 and self.rpy_angle.z <= 1.57 else -1
  #   factor_y = 1 if self.rpy_angle.z >= 0 and self.rpy_angle.z <= 3.14 else -1
  #   angle = self.rpy_angle.z if self.rpy_angle.z >= 0 else self.rpy_angle.z * -1
  #   self.msg_move_to_goal.pose.position.x = self.odometry_data.pose.pose.position.x + (data.y * math.cos(angle)) * factor_x
  #   self.msg_move_to_goal.pose.position.y = self.odometry_data.pose.pose.position.y + (data.y * math.sin(angle)) * factor_y
  #   self.msg_move_to_goal.header.frame_id = 'odom'
  #   self.msg_move_to_goal.pose.orientation.z = self.odometry_data.pose.pose.orientation.z
  #   self.msg_move_to_goal.pose.orientation.w = self.odometry_data.pose.pose.orientation.w
  #   self.pub_move_to_goal.publish(self.msg_move_to_goal)

  def move_goal_to_object(self, position_x, radius):
    if not self.time_old or (self.time_old and time.time() - self.time_old > 10):
      distance = (1 * 937.8194580078125) / (radius * 2)
      y_move_base = -(position_x - self.camera_info.width/2) / (radius*2) 
      x_move_base = distance if abs(y_move_base) < 0.006 else math.sqrt(distance**2 - y_move_base**2)
      self.msg_move_to_goal.pose.position.x = x_move_base
      self.msg_move_to_goal.pose.position.y = y_move_base
      self.msg_move_to_goal.pose.orientation.w = 1
      self.msg_move_to_goal.header.frame_id = self.camera_info.header.frame_id
      self.pub_move_to_goal.publish(self.msg_move_to_goal)
      self.time_old = time.time()

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
      rospy.loginfo("Find the ball!")

  def callback(self, data):
    if data.x != -1:
      self.flag_find = True
      self.move_goal_to_object(data.x, data.z)

      if self.flag_explore and self.status_explore_goal == 1:
        rospy.loginfo("CANCELOU")
        rospy.Publisher("/Explore/cancel", GoalID, queue_size=1).publish(GoalID())
        time.sleep(5)
        os.system("rosnode kill /Operator")
        self.flag_explore = False

      # if self.status_explore_goal != 1:
        # if not self.move_base_info.status_list and self.flag_orientation:
        #   rospy.loginfo("entrou no Orientation")
        #   self.orientation_to_obj(data)
        # elif self.flag_move_to_goal:
        #   rospy.loginfo("entrou no MOVIE")
        #   self.publisher_move_to_goal(data)
        #   self.flag_move_to_goal = False
        #   self.flag_ajustment = True
        # elif (self.move_base_info.status_list and self.move_base_info.status_list[0].status != 1) and self.flag_ajustment:
        #   self.goal_ajustment(data)
        # if not self.flag_move_to_goal:
        #   self.flag_move_to_goal = True
        #   self.goal_move_base(data.x, data.z)

      # if (self.move_base_info.status_list and self.move_base_info.status_list[0].status == 1) and data.y <= 4:
      #   rospy.Publisher('/move_base/cancel', GoalID, queue_size=1).publish(GoalID())
      #   rospy.loginfo("CHEGUEI NA BOLA")
  
    else:
      # if self.flag_find and not self.move_base_info.status_list:
      #   rospy.loginfo("Rotacionando")
      #   self.msg_twist = Twist()
      #   time.sleep(1)
      #   self.msg_twist.angular.z = 0.1
      #   self.pub_cmd_vel.publish(self.msg_twist.angular.z)
      # else:
      if not self.flag_find and not self.flag_explore and self.status_explore_goal != 1:
        rospy.loginfo("AGUARDANDO..")
        time.sleep(5)
        self.pub_explore_goal.publish(ExploreActionGoal())
        rospy.loginfo("INICIOU O EXPLORE")
        self.flag_explore = True


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

  def run(self):
    self.msg = rospy.Subscriber("/camera/obj/coordinates", Vector3, self.callback)

if __name__ == "__main__":
  rospy.loginfo("Init Control")
  ctrl_vision = ControlVision()
  ctrl_vision.run()
  while not rospy.is_shutdown():
    rospy.spin()    