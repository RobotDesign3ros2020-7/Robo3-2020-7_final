#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window",1)
        self.image_sub = rospy.Subscriber('camera/color/image_raw', Image, self.image_callback)
        self.twist = Twist()

    def image_callback(self, msg):
        #image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image = self.bridge.imgmsg_to_cv2(msg)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red = numpy.array([115,100,100])
        upper_red = numpy.array([185,255,255])
        mask = cv2.inRange(hsv, lower_red, upper_red)

        #masked = cv2.bitwise_and(image, image, mask=mask)

        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 5, (0,0,125), -1)
            print(cx,cy)

            #msg.cx = cx
            #msg.cy = cy

            pub1.publish(cx)
            pub2.publish(cy)

        cv2.imshow("window", mask)
        cv2.imshow("image", image)
        #cv2.imshow("window1", hsv)
        cv2.waitKey(3)

rospy.init_node('follower')
pub1 = rospy.Publisher("point_x", Int32, queue_size=1)
pub2 = rospy.Publisher("point_y", Int32, queue_size=1)
rate = rospy.Rate(0.2)
follower = Follower()
rospy.spin()
