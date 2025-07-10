#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from std_msgs.msg import Int16MultiArray

class CyanExtractor:
    def __init__(self):
        rospy.init_node('cyan_extractor', anonymous=True)
        self.bridge = CvBridge()
        rospy.Subscriber("/MobileRobot/Camera/image", Image, self.callback) 
        rospy.loginfo("Subscribed to /MobileRobot/Camera/image")
        self.pub_edges = rospy.Publisher('/cyan_edges', Int16MultiArray, queue_size=10)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_cyan = np.array([85, 100, 100])
        upper_cyan = np.array([100, 255, 255])
        mask = cv2.inRange(hsv, lower_cyan, upper_cyan)
        result = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        # 画像のサイズを取得
        height, width = mask.shape
        center_x = width // 2

        # 画像中心ラインにおけるマスクの値を取得
        vertical_line = mask[:, center_x]

        edges_msg = Int16MultiArray()

        # 水色（255）になっている部分のy座標を取得
        indices = np.where(vertical_line == 255)[0]

        if len(indices) > 0:
            top_y = indices[0]
            bottom_y = indices[-1]

            # 画像中心をy=0とした相対座標に変換
            rel_top_y = top_y - (height // 2)
            rel_bottom_y = bottom_y - (height // 2)

            rospy.loginfo(f"水色の曲線と縦線の交点範囲：上端 y={rel_top_y}, 下端 y={rel_bottom_y}")

            edges_msg.data = [rel_top_y, rel_bottom_y]
            self.pub_edges.publish(edges_msg)
            
            # 確認用に赤い線を引く（上下端の点）
            cv2.circle(result, (center_x, top_y), 5, (0,0,255), -1)
            cv2.circle(result, (center_x, bottom_y), 5, (0,0,255), -1)

        else:
            rospy.loginfo("縦線上に水色部分なし")
            edges_msg.data = [1000,-1000]
            self.pub_edges.publish(edges_msg)

        # 中央の縦線を描画（見やすく）
        cv2.line(result, (center_x, 0), (center_x, height), (0, 255, 0), 2)


        cv2.imshow("Original", cv_image)
        cv2.imshow("Cyan Mask", mask)
        cv2.imshow("Cyan Extracted", result)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        CyanExtractor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
