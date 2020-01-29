 #!/usr/bin/env python2.7

import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

class Camera:
  yellow_range = ((25, 50, 50), (32, 255, 255))

  def __init__(self):
    rospy.init_node('opencv_camera', anonymous=True)
    self.bridge = CvBridge()
    
    rospy.loginfo("Init Camera!")

  # Define a function to show the image in an OpenCV Window
  def show_image(self, img):
    # im_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) #Converte BGR (Gazebo) to RGB (OpenCv)
    cv2.namedWindow("Image Window", 1)
    cv2.imshow("Image Window", img)
    cv2.waitKey(3)
  
  def color_detector(self, cv_image):
    # create NumPy arrays from the boundaries
    img_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    hsv = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2HSV)
    
    mask_yellow = cv2.inRange(hsv, self.yellow_range[0], self.yellow_range[1])
    mask_yellow = cv2.erode(mask_yellow, None, iterations=2)
    mask_yellow = cv2.dilate(mask_yellow, None, iterations=2)
    cnt_yellow = cv2.findContours(mask_yellow.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2]

    contours_poly = []
    centers = []
    radius = []
    for index, obj_cnt in enumerate(cnt_yellow):
      # contours_poly.append(cv2.approxPolyDP(obj_cnt, 0.009 * cv2.arcLength(obj_cnt, True), True))
      contours_poly.append(cv2.approxPolyDP(obj_cnt, 3, True))
      aux1, aux2 = cv2.minEnclosingCircle(contours_poly[index])
      centers.append(aux1)
      radius.append(aux2)

      if(len(contours_poly[index]) > 10):
        center_yellow = max(cnt_yellow, key=cv2.contourArea)
        rect_yellow = cv2.minAreaRect(center_yellow)
        box_yellow = cv2.boxPoints(rect_yellow)
        box_yellow = np.int0(box_yellow)
        # m_yellow = cv2.moments(center_yellow)
        # center_yellow = (int(m_yellow["m10"] / m_yellow["m00"]), int(m_yellow["m01"] / m_yellow["m00"]))
        # cv2.drawContours(img_rgb, contours_poly, 0, (0, 255, 255), 2)
        cv2.circle(img_rgb, (int(centers[index][0]), int(centers[index][1])), int(radius[index]), (0, 255, 255), 2)

    return img_rgb

  def circle_detector(self, cv_image):
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    # detect circles in the image
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20, np.array([]), param1=30,param2=50,minRadius=0,maxRadius=1000)
    # ensure at least some circles were found
    if circles is not None:
      # convert the (x, y) coordinates and radius of the circles to integers
      circles = np.round(circles[0, :]).astype("int")
    
      # loop over the (x, y) coordinates and radius of the circles
      for (x, y, r) in circles:
        # draw the circle in the output image, then draw a rectangle
        # corresponding to the center of the circle
        cv2.circle(cv_image, (x, y), r, (0, 255, 0), 4)
        cv2.rectangle(cv_image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

    return cv_image

  # Define a callback for the Image message
  def image_callback(self, img_msg):
    # log some info about the image topic
    # rospy.loginfo(img_msg.header)

    # Try to convert the ROS Image message to a CV2 Image
    try:
      cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError, e:
      rospy.logerr("CvBridge Error: {0}".format(e))

    img = self.color_detector(cv_image)
    # self.circle_detector(cv_image)
    self.show_image(img)

  def run(self):
    self.sub_image = rospy.Subscriber("/diff/camera_top/image_raw", Image, self.image_callback)

if __name__ == "__main__":
  cam = Camera()
  cam.run()
  while not rospy.is_shutdown():
    rospy.spin()