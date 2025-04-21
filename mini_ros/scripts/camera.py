#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
import cv2
import numpy as np
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def camera():
    pub = rospy.Publisher('/robotis_mini/camera1/image_raw', Image, queue_size=10)
    rospy.init_node('camera', anonymous=True)
    rate = rospy.Rate(15) # 15hz
    br = CvBridge()

    cap = cv2.VideoCapture(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)

    #start time 
    while not rospy.is_shutdown():

        _, frame = cap.read()

        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
   
        if hsv_image is not None:
            pub.publish(br.cv2_to_imgmsg(hsv_image))
        
        rate.sleep()

if __name__ == '__main__':
    try:
        camera()
    except rospy.ROSInterruptException:
        pass
