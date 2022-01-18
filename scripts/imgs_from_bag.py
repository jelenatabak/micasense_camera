#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

path = "/home/jelena/catkin_ws_old/src/micasense_camera/pepper5_front_44/images/pepper5f_"
i = 0
bridge = CvBridge()

def main():
    rospy.init_node("Saver")
    rospy.Subscriber('/dataset_collection/image_save', Image, cbck)
    rospy.spin()

def cbck(msg):
    cv_image = bridge.imgmsg_to_cv2(msg, 'bgr8')
    global i
    cv2.imwrite(path + str(i) + ".jpg", cv_image)
    print("Saved image number " + str(i))
    i += 1

if __name__ == "__main__":
    main()
