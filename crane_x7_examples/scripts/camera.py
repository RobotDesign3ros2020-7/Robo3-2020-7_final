#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import UInt8
import cv2
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge

def cb(message):
    rospy.loginfo(message.data)

def process_image(msg):                                 #画像処理
    try:
        bridge = CvBridge()
        orig = bridge.imgmsg_to_cv2(msg, "bgr8")
        img = cv2.cvtColor(orig, cv2.COLOR_BGR2GRAY)    
        cv2.imshow('image', img)
        cv2.waitKey(1)
    except Exception as err:
        print err



def detect_red_color(img):
   
    
    
    img_min = np.array([0,64,0])                # 赤色のHSVの値域1
    img_max = np.array([30,255,255])
    mask1 = cv2.inRange(img, img_min, img_max)

    
    img_min = np.array([150,64,0])              # 赤色のHSVの値域2
    img_max = np.array([179,255,255])
    mask2 = cv2.inRange(img, img_min, img_max)

        
    mask = mask1 + mask2                       # 赤色領域のマスク（255：赤色、0：赤色以外）

   
    masked_img = cv2.bitwise_and(img, img, mask=mask)            # マスキング処理

    return mask, masked_img

    red_mask, red_masked_img = detect_red_color(img)

    cv2.imwrite("C:\prog\python\\test\\red_mask.png", red_mask)
    cv2.imwrite("C:\prog\python\\test\\red_masked_img.png", red_masked_img)

def start_node():
    rospy.init_node('img_proc')
    rospy.loginfo('img_proc node started')
    sub1 = rospy.Subscriber("image_raw", Image, process_image, queue_size=1)
    sub2 = rospy.Subscriber("ude", UInt8, cb, queue_size=1)
    pub = rospy.Publisher("hanko", Image, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
        n = 2
        pub.publish(n)
        rate.sleep()
    except rospy.ROSInterruptException:
        pass