#!/usr/bin/env python2.7
import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
import time

from control_pid import ControlPid

class ControlVision:
  control_pid_x = None
  control_pid_yaw = None
  pub_cmd_vel = None
  msg_twist = None

  def __init__ (self):
    rospy.init_node("robot_vision", anonymous=True)
    self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    self.control_pid_x = ControlPid(5, -5, 0.01, 0, 0)
    self.control_pid_yaw = ControlPid(3, -3, 0.001, 0, 0)
    self.msg_twist = Twist()
       
  def callback(self, data):
    if data.x != -1:
      self.msg_twist.angular.z = self.control_pid_yaw.pid_calculate(0.5, 600, int(data.x))
      self.msg_twist.linear.x = self.control_pid_x.pid_calculate(0.5, 180, int(data.z))
      self.pub_cmd_vel.publish(self.msg_twist)


  def run(self):
    self.msg = rospy.Subscriber("/camera/obj/coordinates", Vector3, self.callback)

if __name__ == "__main__":
  rospy.loginfo("Init Vision")
  ctrl_vision = ControlVision()
  ctrl_vision.run()
  while not rospy.is_shutdown():
    rospy.spin()
    


    