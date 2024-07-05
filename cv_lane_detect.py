#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import time as t
import math
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import UInt8, Float64
from sensor_msgs.msg import Image, CompressedImage
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Twist

rospy.init_node('opencv_example', anonymous=True)

lower_yellow = np.array([20,100,100])
upper_yellow = np.array([30,255,255])

lower_blue = np.array([110,50,50])
upper_blue = np.array([130,255,255])
 
bridge = CvBridge()

def show_image(img, text):
	    cv2.imshow(text, img)
	    cv2.waitKey(3)


def move_robot(x,w):
    cmd_vel=rospy.Publisher('cmd_vel',Twist, queue_size = 10)
    move_cmd=Twist()
    move_cmd.linear.x=x	 
    move_cmd.angular.z=w
    cmd_vel.publish(move_cmd)
    t.sleep(0.1)

def turn(left):
    angular = -math.pi
    if left:
        angular = math.pi
    cmd_vel=rospy.Publisher('cmd_vel',Twist, queue_size = 10)
    move_cmd=Twist()
    move_cmd.linear.x=0.7
    t.sleep(2)
    move_cmd.angular.z=angular
    t.sleep(0.5)
    cmd_vel.publish(move_cmd)
    t.sleep(0.6)
        

def scan(left_ratio, right_ratio, cx_l, cx_r):
    if abs(left_ratio - right_ratio) <= 0.0007:
        if cx_l < 200 and cx_r > 120:
            move_robot(0.7,0)
        elif cx_l >= 200:
            move_robot(0, 0.65)
        else:
            move_robot(0, -0.65)
    elif left_ratio < 0.0012:
        turn(1)
    elif right_ratio < 0.0012:
        turn(0)
    elif left_ratio > right_ratio:
        move_robot(0, 0.65)
    elif left_ratio < right_ratio:
        move_robot(0, -0.65)

    
def image_callback(img_msg):

    rospy.loginfo(img_msg.header)
    #Get Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")

    except CvBridgeError as e:
        print(e)
    #show_image(cv_image, "hehe")

    #Masking to separate yellow lines
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_blue , upper_blue )
    result = cv2.bitwise_and(cv_image, cv_image, mask= mask)

    #Create Bird's Eye Top Down view
    cv_image_original = cv2.GaussianBlur(result, (5, 5), 0)
    pts_src = np.array([[200, 280], [440, 280], [640, 440], [0,440]])
    pts_dst = np.array([[200, 0], [800, 0], [800, 600], [200, 600]])
    h, status = cv2.findHomography(pts_src, pts_dst)
    cv_image_homography = cv2.warpPerspective(cv_image_original, h, (1000, 600))
    triangle1 = np.array([[0, 599], [0, 340], [200, 599]], np.int32)
    triangle2 = np.array([[999, 599], [999, 340], [799, 599]], np.int32)
    black = (0, 0, 0)
    white = (255, 255, 255)
    result = cv2.fillPoly(cv_image_homography, [triangle1, triangle2], black)


    #Split image into two halves to separate left and right lanes
    h, w, channels = result.shape
    half = w//2
    left_part = result[:, :half]
    right_part = result[:, half:]


    #Find contours in image
    left_mask = cv2.inRange(left_part, lower_blue, upper_blue)
    right_mask = cv2.inRange(right_part, lower_blue, upper_blue)
    ret, thresh = cv2.threshold(left_part, 127, 255, 0)
    contoursl, hierarchyl = cv2.findContours(left_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    contoursr, hierarchyr = cv2.findContours(right_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

    #Select Lane Contours on both left and right
    cl = max(contoursl, key = cv2.contourArea)
    cr = max(contoursr, key = cv2.contourArea)

    #Calculate Percentage of Lane Area in frame:
    ratio_yellow_left = cv2.countNonZero(left_mask)/(left_part.size/3)
    ratio_yellow_right = cv2.countNonZero(right_mask)/(left_part.size/3)

    #Find Centers of Lane Contours
    cx_l = 0 
    cx_r = 0
    M = cv2.moments(cl)
    if M['m00'] != 0:
        cx_l = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cv2.circle(left_part, (cx_l, cy), 7, (0, 0, 255), -1)

    M = cv2.moments(cr)
    if M['m00'] != 0:
        cx_r = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cv2.circle(right_part, (cx_r, cy), 7, (0, 0, 255), -1)

    #This Function handles movement of the bot
    scan(ratio_yellow_left, ratio_yellow_right, cx_l, cx_r)

    show_image(left_part, "left")
    show_image(right_part, "right")
    #print(ratio_yellow_left)
    #print(ratio_yellow_right)
    
    #if cx_l >= half//2:
    #    print("Turn Right!!")
    #elif cx_r <= half//2:
    #    print("Turn Left!!")
        


#Input image from bot
cv_image_compressed = rospy.Subscriber('/camera/color/image_raw', Image, image_callback)

while not rospy.is_shutdown():
    rospy.spin()
    
    
