#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist

class EdgeToCmdVel:
    def __init__(self):
        rospy.init_node('edge_to_cmd_vel', anonymous=True)
        self.sub_edges = rospy.Subscriber('/cyan_edges', Int16MultiArray, self.edge_callback)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()

    def edge_callback(self, msg):
        if len(msg.data) == 2 :
            top, bottom = msg.data
            #rospy.loginfo(f"Received edges: top={top}, bottom={bottom}")

            # 上下端の差（太さ）を前進速度に利用
            thickness = bottom - top
            max_thickness = 240  
            linear_x = 0.15 + max(0.0, min(0.30, (max_thickness - thickness) / float(max_thickness) * 0.30))
            

            # 上下端の平均を角速度に利用（画像中心が0なので偏差で曲がる）
            avg_y = (top + bottom) / 2.0
            max_offset = 120 
            angular_z =  - max(-8.0, min(8.0, avg_y / max_offset * 4 )) 

            wide = bottom - top

            if wide < 50:
                angular_z += 1.0
                

            self.twist.linear.x = linear_x
            self.twist.angular.z = angular_z




        else:
            rospy.loginfo("No valid edge data received, stopping.")
            top = 1000
            bottom = 1000
            self.twist.linear.x = 0.3
            self.twist.angular.z = 0

        rospy.loginfo(f"cmd_vel: lin={self.twist.linear.x}, ang={self.twist.angular.z}")

        self.pub_cmd_vel.publish(self.twist)

if __name__ == '__main__':
    try:
        node = EdgeToCmdVel()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
