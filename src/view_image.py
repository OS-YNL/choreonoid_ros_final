#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node('image_listener', anonymous=True)
        rospy.Subscriber("/SimpleTank/Camera2/image", Image, self.callback)
        rospy.loginfo("Subscribed to /SimpleTank/Camera2/image")
    
    def callback(self, data):
        try:
            # ROSの画像メッセージをOpenCV画像に変換
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))
            return
        
        # 画像を表示
        cv2.imshow("Image Window", cv_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        ImageSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
