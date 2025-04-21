#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
import cv2
import numpy as np
import time
from cv_bridge import CvBridge

class BallDetector: 
    
    def __init__(self):
        rospy.init_node('ball_detector', anonymous=True)
        self.calibrate = False
        self.KNOWN_WIDTH = 0.02 * 2
        self.KNOWN_DISTANCE = 0.5

        self.pub = rospy.Publisher('/detected_ball_pos', Point, queue_size=10)
        self.im_pub = rospy.Publisher('/detected_ball_img', Image, queue_size=10)
        sub = rospy.Subscriber('/robotis_mini/camera1/image_raw', Image, self.img_callback)
        self.bridge = CvBridge()

        rate = rospy.Rate(100)
        rospy.spin()

    def img_callback(self, msg):

        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        im = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        '''
        TODO: Implement blob detector to detect the keypoints for target object. The keypoint
        is used to estimate the distance from the robot to target object.
        You can debug the image output on rviz or save the processed img with drawn keypoints. 

        How does Blob detection work?
        SimpleBlobDetector is based on a rather simple algorithm controlled by parameters and has the following steps.
            Thresholding : Convert the source images to several binary images by thresholding the source image with 
                            thresholds starting at minThreshold. These thresholds are incremented by thresholdStep 
                            until maxThreshold. So the first threshold is minThreshold, the second is minThreshold +
                            thresholdStep, the third is minThreshold + 2 x thresholdStep, and so on.
            Grouping : In each binary image, connected white pixels are grouped together. Let's call these binary blobs.
            Merging  : The centers of the binary blobs in the binary images are computed, and  blobs located closer 
                        than minDistBetweenBlobs are merged.
            Center & Radius Calculation :  The centers and radii of the new merged blobs are computed and returned.

        Reference tutorial below to learn more about blob detection and its parameters.
        https://learnopencv.com/blob-detection-using-opencv-python-c/
        '''
        
        gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Detect blobs.
        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()
        
        # Change thresholds 
        params.minThreshold = 10
        params.maxThreshold = 200

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 5

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.3

        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.87

        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.01

        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(gray_image)

        if keypoints and (len(keypoints) > 0) :

            # Draw detected blobs as red circles.
            # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
            # the size of the circle corresponds to the size of blob
            im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,255,), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            imgmsg_with_keypoints = self.bridge.cv2_to_imgmsg(im_with_keypoints)
            self.im_pub.publish(imgmsg_with_keypoints)

            if self.calibrate:
                focalLength = (keypoints[0].size * self.KNOWN_DISTANCE) / self.KNOWN_WIDTH
            else:
                focalLength = 435
            distance = (self.KNOWN_WIDTH * focalLength) / (keypoints[0].size)

            ball_pos = Point()
            ball_pos.x = round(keypoints[0].pt[0])
            ball_pos.y = round(keypoints[0].pt[1])
            ball_pos.z = distance #use z coord to send size 
            self.pub.publish(ball_pos)

            if self.calibrate:
                print("focal_length: ", focalLength, "\nball_pos: \n", ball_pos)
        else: 
            imgmsg_without_keypoints = self.bridge.cv2_to_imgmsg(im)
            self.im_pub.publish(imgmsg_without_keypoints)

if __name__ == '__main__':
    ball_detector = BallDetector()
