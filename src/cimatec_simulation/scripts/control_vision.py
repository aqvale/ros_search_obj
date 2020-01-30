#!/usr/bin/env python2.7
import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

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
  pub_move_to_goal = None
  msg_move_to_goal = None

  def __init__ (self):
    rospy.loginfo("INIT CONTROL VISION")
    self.control_pid_x = ControlPid(5, -5, 0.01, 0, 0)
    self.control_pid_yaw = ControlPid(3, -3, 0.001, 0, 0)
    self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    self.msg_twist = Twist()
    self.pub_quaternion = rospy.Publisher("/rotation_quaternion", Quaternion, queue_size=1)
    self.pub_move_to_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    self.msg_move_to_goal = PoseStamped()
    rospy.init_node("robot_vision", anonymous=True)
    rospy.Subscriber("/odometry/filtered", Odometry, self.callback_odometry)
    rospy.Subscriber("/rpy_angles", Vector3, self.callback_rpy_angles)
    rospy.Subscriber("/diff/camera_top/camera_info", CameraInfo, self.callback_camera_info)

  def callback(self, data):
    # msg ="\nx - " + str(self.odometry_data.pose.pose.position.x) + "\ny - " + str(self.odometry_data.pose.pose.position.y) + "\n" + str(data.y) + " - " + str((self.rpy_angle.z*180)/3.1415)
    # rospy.loginfo(msg)
    if data.x != -1:
      if not self.flag_move_to_goal and self.flag_orientation:
        self.msg_twist.angular.z = self.control_pid_yaw.pid_calculate(0.5, self.camera_info.width/2, int(data.x))
        # self.msg_twist.linear.x = self.control_pid_x.pid_calculate(0.5, 180, int(data.z))
        self.pub_cmd_vel.publish(self.msg_twist)
        rospy.loginfo(round(self.msg_twist.angular.z, 1))

      if not self.flag_move_to_goal and round(self.msg_twist.angular.z, 1) == 0:
        self.flag_move_to_goal = True
        self.flag_orientation = False
        self.publisher_move_to_goal(data)
      
      msg = str(round(self.msg_move_to_goal.pose.position.x, 2) + " - " + round(self.odometry_data.pose.pose.position.x, 2))
      rospy.loginfo(msg)
      if round(self.msg_move_to_goal.pose.position.x, 2) == round(self.odometry_data.pose.pose.position.x, 2) and \
         round(self.msg_move_to_goal.pose.position.y, 2) == round(self.odometry_data.pose.pose.position.y, 2):
        rospy.loginfo("Achou!!")
        self.flag_move_to_goal = False
        self.flag_orientation = True

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

  def publisher_move_to_goal(self, data):
    rospy.loginfo("Entrou no move base")
    if self.rpy_angle.z <= 0 and self.rpy_angle.z >= -1.57:
      angle = self.rpy_angle.z * -1
      self.msg_move_to_goal.pose.position.x = self.odometry_data.pose.pose.position.x + (data.y * math.cos(angle))
      self.msg_move_to_goal.pose.position.y = self.odometry_data.pose.pose.position.y - (data.y * math.sin(angle))
      self.msg_move_to_goal.header.frame_id = 'odom'
      self.msg_move_to_goal.pose.orientation.z = self.odometry_data.pose.pose.orientation.z
      self.msg_move_to_goal.pose.orientation.w = self.odometry_data.pose.pose.orientation.w
      msg = str(self.msg_move_to_goal.pose.position.x) + " - " + str(self.msg_move_to_goal.pose.position.y)
      self.pub_move_to_goal.publish(self.msg_move_to_goal)
    rospy.loginfo(msg)

  def run(self):
    self.msg = rospy.Subscriber("/camera/obj/coordinates", Vector3, self.callback)

if __name__ == "__main__":
  rospy.loginfo("Init Control")
  ctrl_vision = ControlVision()
  ctrl_vision.run()
  while not rospy.is_shutdown():
    rospy.spin()    