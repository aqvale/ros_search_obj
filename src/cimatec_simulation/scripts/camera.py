#!/usr/bin/env python2.7

import rospy
import cv2
import argparse
import sys
import numpy as np
import constant

from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from collections import deque
from cv_bridge import CvBridge, CvBridgeError


class Camera:
  yellow_range = [(20, 100, 100), (32, 255, 255)]

  def __init__(self):
    rospy.init_node('opencv_camera', anonymous=True)
    self.pub = rospy.Publisher('camera/obj/coordinates', Vector3, queue_size=1)
    self.pub_image = rospy.Publisher('/camera/mission', Image, queue_size=1)
    self.bridge = CvBridge()
    rospy.loginfo("Init Camera!")

  #
  # Publisher image in the topic camera
  # Param img: Imagem in rbg
  #
  def show_image(self, img):
    # cv2.namedWindow("Image Window", 1)
    # cv2.imshow("Image Window", img)
    img_pub = self.bridge.cv2_to_imgmsg(img, "bgr8")
    self.pub_image.publish(img_pub)
    cv2.waitKey(3)
  
  #
  # Detect a yellow circle
  # Param cv_image: Image in format opencv
  #
  def object_color_detector(self, cv_image):
    img_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    hsv = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2HSV)

    mask_yellow = cv2.inRange(hsv, self.yellow_range[0], self.yellow_range[1])
    mask_yellow = cv2.erode(mask_yellow, None, iterations=2)
    mask_yellow = cv2.dilate(mask_yellow, None, iterations=2)
    cnt_yellow = cv2.findContours(mask_yellow.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2]

    contours_poly = []
    centers = []
    radius = []
    coordinates = [-1, -1, -1]
    for index, obj_cnt in enumerate(cnt_yellow):
      contours_poly.append(cv2.approxPolyDP(obj_cnt, 0.009 * cv2.arcLength(obj_cnt, True), True))
      aux1, aux2 = cv2.minEnclosingCircle(contours_poly[index])
      centers.append(aux1)
      radius.append(aux2)
      if(len(contours_poly[index]) >= 9):
        coordinates = self.obj_coordinate(cnt_yellow)
        coordinates[1] = self.distance_to_camera(coordinates[2])
        cv2.circle(img_rgb, (int(centers[index][0]), int(centers[index][1])), int(radius[index]), (255, 255, 0), 2)
        cv2.putText(img_rgb, 'Ball detected!', (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 5)
    self.pub.publish(coordinates[0], coordinates[1], coordinates[2])
    return img_rgb

  #
  # Get coordinate of the object
  # Param cnts: Contour area of the object
  # Return the position in x, y and the radius
  #
  def obj_coordinate(self, cnts):
    c = max(cnts, key=cv2.contourArea)
    ((x, y), radius) = cv2.minEnclosingCircle(c)
    M = cv2.moments(c)
    return [int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]), radius]
  
  #
  # Compute the distance from the maker to the camera
  # Param radius: Radius of the object
  # Return the distance
  #
  def distance_to_camera(self, radius):
  	return constant.FOCAL_LENGHT / (radius * 2)

  #
  # Callback of the subscriber 
  #
  def image_callback(self, img_msg):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
      self.show_image(self.object_color_detector(cv_image))
    except CvBridgeError, e:
      rospy.logerr("CvBridge Error: {0}".format(e))

  #
  # Start the camera
  #
  def run(self):
    self.sub_image = rospy.Subscriber("/diff/camera_top/image_raw", Image, self.image_callback)

if __name__ == "__main__":
  rospy.loginfo("Init Vision")
  cam = Camera()
  cam.run()
  while not rospy.is_shutdown():
    rospy.spin()   