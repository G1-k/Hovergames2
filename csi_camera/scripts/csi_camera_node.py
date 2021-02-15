#!/usr/bin/env python
import rospy
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

GSTREAMER_PIPELINE = 'v4l2src ! video/x-raw,width=640,height=480 ! decodebin ! videoconvert ! appsink'

def start_node():
    rospy.init_node('image_pub')
    rospy.loginfo('image_pub node started')
    pub = rospy.Publisher("csi_camera/image" ,Image,queue_size=10)
    video_capture = cv2.VideoCapture(GSTREAMER_PIPELINE, cv2.CAP_GSTREAMER)
    #video_capture = cv2.VideoCapture(0)
    #if video_capture.isOpened():
       # cv2.namedWindow("Face Detection Window", cv2.WINDOW_AUTOSIZE)

    while not rospy.is_shutdown():
        return_key, image = video_capture.read()
        if not return_key:
            rospy.loginfo('Break')
            break
        rospy.loginfo('Pubsishing')                
        bridge = CvBridge()
        img_msg = bridge.cv2_to_imgmsg(image,"bgr8")
        pub.publish(img_msg)
        rospy.Rate(10).sleep()

        #grayscale_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        #detected_faces = face_cascade.detectMultiScale(grayscale_image, 1.3, 5)
        # Create rectangle around the face in the image canvas
        #for (x_pos, y_pos, width, height) in detected_faces:
        #  cv2.rectangle(image, (x_pos, y_pos), (x_pos + width, y_pos + height), (0, 0, 0), 2)

        #cv2.imshow("Face Detection Window", image)
        

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        video_capture.release()
        cv2.destroyAllWindows()
