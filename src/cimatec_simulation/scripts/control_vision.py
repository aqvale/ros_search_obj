#!/usr/bin/env python2.7
import os
import rospy
import time
import math
import constant

from geometry_msgs.msg import (
  Vector3,
  Twist,
  PoseStamped
)
from nav2d_navigator.msg import (
  GetFirstMapActionGoal,
  ExploreActionGoal
)
from actionlib_msgs.msg import (
  GoalStatusArray,
  GoalID
) 
from sensor_msgs.msg import CameraInfo
from control_pid import ControlPid

class Robot:
  camera_info = None
  obj_coordinates = None
  move_base_info = None
  status_explore_goal = None
  time_old = None
  
  def __init__ (self):
    rospy.loginfo("INIT CONTROL VISION")
    rospy.init_node("robot_vision", anonymous=True)
    self.control_pid_x = ControlPid(5, -5, 0.01, 0, 0)
    self.control_pid_yaw = ControlPid(3, -3, 0.001, 0, 0)
    self.flag_find = False
    self.flag_explore = False
    
    rospy.Subscriber("/camera/obj/coordinates", Vector3, self.callback)
    rospy.Subscriber("/diff/camera_top/camera_info", CameraInfo, self.callback_camera_info)
    rospy.Subscriber("/move_base/status", GoalStatusArray, self.callback_move_base_info)
    rospy.Subscriber("/Explore/status", GoalStatusArray, self.callback_explore_status)
    
    self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    self.pub_move_to_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    self.pub_first_map_goal = rospy.Publisher("/GetFirstMap/goal", GetFirstMapActionGoal, queue_size=1)
    self.pub_explore_goal = rospy.Publisher("/Explore/goal", ExploreActionGoal, queue_size=1)

    cancel_first_map = rospy.Publisher("/GetFirstMap/cancel", GoalID, queue_size=1)
    time.sleep(1)
    self.pub_first_map_goal.publish()
    time.sleep(1)
    cancel_first_map.publish(GoalID())

  def callback(self, data):
    self.obj_coordinates = data

  def callback_camera_info(self, data):
    self.camera_info = data
  
  def callback_move_base_info(self, data):
    self.move_base_info = data

  def callback_explore_status(self, data):
    if data.status_list:
      self.status_explore_goal = data.status_list[0].status

  def move_goal_to_object(self):
    msg_move_to_goal = PoseStamped()
    if not self.time_old or (self.time_old and time.time() - self.time_old > 10):
      distance = (1 * constant.FOCAL_LENGHT) / (self.obj_coordinates.z * 2)
      y_move_base = -(self.obj_coordinates.x - self.camera_info.width/2) / (self.obj_coordinates.z*2) 
      x_move_base = distance if abs(y_move_base) < 0.006 else math.sqrt(distance**2 - y_move_base**2)
      msg_move_to_goal.pose.position.x = x_move_base
      msg_move_to_goal.pose.position.y = y_move_base
      msg_move_to_goal.pose.orientation.w = 1
      msg_move_to_goal.header.frame_id = self.camera_info.header.frame_id
      self.pub_move_to_goal.publish(msg_move_to_goal)
      self.time_old = time.time()

  def goal_ajustment(self):
    msg_twist = Twist()
    while round(msg_twist.angular.z, 1) != 0 and round(msg_twist.linear.x, 1) != 0:
      msg_twist.angular.z = self.control_pid_yaw.pid_calculate(0.5, self.camera_info.width/2, int(self.obj_coordinates.x))
      msg_twist.linear.x = self.control_pid_x.pid_calculate(0.5, 180, int(self.obj_coordinates.z))
      self.pub_cmd_vel.publish(msg_twist)
    rospy.loginfo("Find the ball!")
    rospy.loginfo(time.ctime())
    exit

  def run(self):
    if self.obj_coordinates.x != -1:
      self.flag_find = True
      if (self.move_base_info.status_list and self.move_base_info.status_list[0].status == 1) and self.obj_coordinates.y <= 4:
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
  time.sleep(2)
  while not rospy.is_shutdown():
    ctrl_vision.run()
    rospy.spin()    