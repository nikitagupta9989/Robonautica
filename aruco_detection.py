#!/usr/bin/env python3.8
import rospy
from sensor_msgs.msg import Image
import time
import tf
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
import numpy as np 
from std_msgs.msg import String, Float32
import math
import pickle
 
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

array = [[[], [], 1e7, [], []], 
         [[], [], 1e7, [], []],
         [[], [], 1e7, [], []],
         [[], [], 1e7, [], []],
         [[], [], 1e7, [], []],
         [[], [], 1e7, [], []],
         [[], [], 1e7, [], []],
         [[], [], 1e7, [], []],
         [[], [], 1e7, [], []],
         [[], [], 1e7, [], []]]


camera_width = 640
camera_height = 480
camera_frame_rate = 40
marker_size = 100
obj_marker_id = 1
ee_marker_id = 0  

K = [[462.1379699707031, 0.0, 320.0],[ 0.0, 462.1379699707031, 240.0],[ 0.0, 0.0, 1.0]]
D = [[]]
K = np.array(K)
D = np.array(D)

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

rospy.init_node('opencv_example', anonymous=True)

bridge = CvBridge()

def show_image(img):
	    cv2.imshow("Image Window", img)
	    cv2.waitKey(3)

pose_pub = rospy.Publisher("object_pose", Float32, queue_size=10)
marker_pub = rospy.Publisher('marker_id', String, queue_size=10)

listener = tf.TransformListener()
rate = rospy.Rate(1.0)

def image_callback(img_msg):
    rospy.loginfo(img_msg.header)
    ee_marker = []
    obj_marker = []

    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError as e:
        print(e)
    gray_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    arucoParams = aruco.DetectorParameters_create()
    (corners, ids, rejected) = aruco.detectMarkers(gray_frame, aruco_dict, K,D)
    
    if ids is not None:
        rvec, tvec, __ = aruco.estimatePoseSingleMarkers(corners,marker_size, K, D)
        tvec2 = tvec[0][0]
        rvec2 = rvec[0][0]
        tvec_str = "x=%4.0f y=%4.0f z=%4.0f"%(tvec2[0], tvec2[1], tvec2[2])
        cv2.putText(cv_image, tvec_str, (20, 460), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 0), 2, cv2.LINE_AA)

        for marker in range(len(ids)):
            cv2.putText(cv_image, str(ids[marker][0]), (int(corners[marker][0][0][0])-30, int(corners[marker][0][0][1])), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 0), 2, cv2.LINE_AA)
            ee_marker = tvec[marker]
            obj_marker = tvec[marker]
            aruco.drawAxis(cv_image, K, D, rvec[marker], tvec[marker], 10)
            R, _ = cv2.Rodrigues(rvec[0])
            cameraPose = -R.T * tvec2[0]
            dist = math.sqrt(tvec2[0]**2 + tvec2[1]**2 + tvec2[2]**2)
            (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            print(euler_from_quaternion(rot[0], rot[1], rot[2], rot[3]))
            print(tvec2)
            if array[ids[marker][0]][2] > dist:
                array[ids[marker][0]][0] = rvec2;
                array[ids[marker][0]][1] = tvec2;
                array[ids[marker][0]][2] = dist;
                array[ids[marker][0]][3] = trans;
                array[ids[marker][0]][4] = euler_from_quaternion(rot[0], rot[1], rot[2], rot[3]);
        aruco.drawDetectedMarkers(cv_image, corners, ids)
			
        if len(ee_marker) > 0 and len(obj_marker) > 0 :
            delta_pose = [(x - y)*10.0 for x, y in zip(obj_marker, ee_marker)] # Prints in milimeters. Default is cm.
          #  print(delta_pose)
            pose_pub.publish(delta_pose)
            marker_pub.publish(str(obj_marker_id))
        else: 
            print("No Marker Found")
    f = open("file.txt", "w")
    for items in array:
    	f.write(str(items[0])+str(items[1])+"\n")
    f.close()
    
    with open("file2.pkl","wb") as f:
    	pickle.dump(array,f)
    f.close()

    show_image(cv_image)

sub_image = rospy.Subscriber("/camera/color/image_raw", Image, image_callback)

cv2.namedWindow("Image Window", 1)


while not rospy.is_shutdown():
    rospy.spin()
