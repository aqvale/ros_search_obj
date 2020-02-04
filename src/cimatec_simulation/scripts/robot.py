#!/usr/bin/env python2.7
import os
import rospy
import time
import math
import constant

from geometry_msgs.msg import Quaternion
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

from nav_msgs.msg import Odometry

#
# Robot controler
#
class Robot:
  camera_info = None
  obj_coordinates = None
  move_base_info = None
  status_explore_goal = None
  time_old = None
  time_start = None
  odometry_data = None
  pub_quaternion = None
  rpy_angle = None
  last_reference_coordinate = None
  
  def __init__ (self):
    rospy.loginfo("INIT CONTROL VISION")
    rospy.init_node("robot_vision", anonymous=True)
    self.control_pid_x = ControlPid(5, -5, 0.01, 0, 0)
    self.control_pid_yaw = ControlPid(3, -3, 0.001, 0, 0)
    self.flag_find = False
    self.flag_explore = True
    self.flag_orientation = False
    self.flag_last_ref = False
    self.stop = False
    self.lost = False
    self.time_start = time.time()
    
    rospy.Subscriber("/odometry/filtered", Odometry, self.callback_odometry)
    rospy.Subscriber("/diff/camera_top/camera_info", CameraInfo, self.callback_camera_info)
    rospy.Subscriber("/move_base/status", GoalStatusArray, self.callback_move_base_info)
    rospy.Subscriber("/Explore/status", GoalStatusArray, self.callback_explore_status)
    rospy.Subscriber("/rpy_angles", Vector3, self.callback_rpy_angles)
    
    self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    self.pub_move_to_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    self.pub_quaternion = rospy.Publisher("/rotation_quaternion", Quaternion, queue_size=1)

  #
  # Init of the robot
  # Param data: Object parameters in the camera
  #             data.x - Position of the object in the camera
  #             data.y - Distance of the object to camera
  #             data.z - Radius of the object
  #
  def callback_main(self, data):
    if not self.stop:
      if data.x != -1:
        self.flag_find = True
        if not self.time_old or (self.time_old and time.time() - self.time_old > 20):
          self.last_reference_obj(data.x, data.z)
          self.time_old = time.time() 
        if self.flag_last_ref:
          self.stop_move_base()
          self.flag_last_ref = False

        if self.flag_explore:
          rospy.loginfo("Stop Explore and kill Operator")
          os.system("rosnode kill /explore")
          time.sleep(1)
          self.stop_move_base()
          self.flag_explore = False
          self.flag_orientation = True

        if not self.flag_explore:
          if data.y <= 4:
            self.stop_move_base()
            self.goal_ajustment(data)
            self.flag_move_to_goal = False
          elif self.flag_orientation:
            self.stop_move_base()
            self.orientation_to_obj(data)
          elif self.flag_move_to_goal:
            self.move_goal_to_object(data)
            self.flag_move_to_goal = False
      else:
        if self.flag_orientation and (not self.time_old or (self.time_old and time.time() - self.time_old > 20)):
          msg_move_to_goal = PoseStamped()
          msg_move_to_goal.pose.position.x = self.last_reference_coordinate[0]
          msg_move_to_goal.pose.position.y = self.last_reference_coordinate[1]
          msg_move_to_goal.pose.orientation.w = 1
          msg_move_to_goal.header.frame_id = self.camera_info.header.frame_id
          self.pub_move_to_goal.publish(msg_move_to_goal)
          self.flag_last_ref = True
          self.time_old = time.time() 

  #
  # Camera information
  #
  def callback_camera_info(self, data):
    self.camera_info = data
  
  #
  # Move-base topic information
  #
  def callback_move_base_info(self, data):
    self.move_base_info = data

  #
  # Status of the topic explore
  #
  def callback_explore_status(self, data):
    if data.status_list:
      self.status_explore_goal = data.status_list[0].status

  #
  # Odometry
  #
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

  #
  # Moves the robot to the object's position on the map
  # Param position_x: Position of the object in the camera
  #       radius: Radius of the object
  #
  def move_goal_to_object(self, data):
    rospy.loginfo("#####MOVE GOAL TO OBJ######")
    factor_x = 1 if (self.rpy_angle.z <= 0 and self.rpy_angle.z >= -1.57) or self.rpy_angle.z >= 0 and self.rpy_angle.z <= 1.57 else -1
    factor_y = 1 if self.rpy_angle.z >= 0 and self.rpy_angle.z <= 3.14 else -1
    angle = self.rpy_angle.z if self.rpy_angle.z >= 0 else self.rpy_angle.z * -1
    msg_move_to_goal = PoseStamped()
    msg_move_to_goal.pose.position.x = self.odometry_data.pose.pose.position.x + (data.y * math.cos(angle)) * factor_x
    msg_move_to_goal.pose.position.y = self.odometry_data.pose.pose.position.y + (data.y * math.sin(angle)) * factor_y
    msg_move_to_goal.header.frame_id = 'odom'
    msg_move_to_goal.pose.orientation.z = self.odometry_data.pose.pose.orientation.z
    msg_move_to_goal.pose.orientation.w = self.odometry_data.pose.pose.orientation.w
    self.pub_move_to_goal.publish(msg_move_to_goal)

  def orientation_to_obj(self, data):
    rospy.loginfo("######ORIENTATION TO OBJ#######")
    msg_twist = Twist()
    msg_twist.angular.z = self.control_pid_yaw.pid_calculate(0.5, self.camera_info.width/2, int(data.x))
    self.pub_cmd_vel.publish(msg_twist)
    if round(msg_twist.angular.z, 1) == 0:
      self.flag_orientation = False
      self.flag_move_to_goal = True

  def last_reference_obj(self, position_x, radius):
    rospy.loginfo("######LAST REFERENCE OBJ#######")
    distance = (1 * 937.8194580078125) / (radius * 2)
    y_move_base = -(position_x - self.camera_info.width/2) / (radius*2) 
    x_move_base = distance if abs(y_move_base) < 0.006 else math.sqrt(distance**2 - y_move_base**2)
    self.last_reference_coordinate = [x_move_base, y_move_base]
  #
  # Center the object in the camera
  # Param data: Information of the topic /camera/obj/coordinates
  #
  def goal_ajustment(self, data):
    rospy.loginfo("######GOAL AJUSTMENT######")
    msg_twist = Twist()
    msg_twist.angular.z = self.control_pid_yaw.pid_calculate(0.5, self.camera_info.width/2, int(data.x))
    msg_twist.linear.x = self.control_pid_x.pid_calculate(0.5, 180, int(data.z))
    self.pub_cmd_vel.publish(msg_twist)
    if round(msg_twist.angular.z, 1) == 0 and round(msg_twist.linear.x, 1) == 0:
      rospy.loginfo("Find the ball!")
      rospy.loginfo(time.time() - self.time_start)
      self.stop = True

  def stop_move_base(self):
    rospy.Publisher('/move_base/cancel', GoalID, queue_size=1).publish(GoalID())
    move_msg = PoseStamped()
    move_msg.pose.position.x = self.odometry_data.pose.pose.position.x
    move_msg.pose.position.y = self.odometry_data.pose.pose.position.y
    move_msg.header.frame_id = 'odom'
    move_msg.pose.orientation.z = self.odometry_data.pose.pose.orientation.z
    move_msg.pose.orientation.w = self.odometry_data.pose.pose.orientation.w
    self.pub_move_to_goal.publish(move_msg)
  #
  # Start the robot
  #
  def run(self):
    rospy.Subscriber("/camera/obj/coordinates", Vector3, self.callback_main)

if __name__ == "__main__":
  rospy.loginfo("Start Robot")
  robot = Robot()
  robot.run()
  while not rospy.is_shutdown():
    rospy.spin()    