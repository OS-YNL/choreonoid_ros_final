#!/usr/bin/env python
import rospy
from rosgraph_msgs.msg import Log
from geometry_msgs.msg import Point
import re

def callback(msg):
    # 例: msg.msg = "Position: [52.515, 45.080, -6100.316]"
    match = re.search(r'Position: \[([-\d.]+), ([\-\d.]+), ([\-\d.]+)\]', msg.msg)
    if match:
        x = float(match.group(1))
        y = float(match.group(2))
        z = float(match.group(3))

        point_msg = Point()
        point_msg.x = x
        point_msg.y = y
        point_msg.z = z

        pub.publish(point_msg)
        rospy.loginfo("Published position -> X: %.3f, Y: %.3f, Z: %.3f", x, y, z)

def listener():
    rospy.init_node('rosout_position_publisher', anonymous=True)
    rospy.Subscriber("/rosout", Log, callback)
    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('/position_from_log', Point, queue_size=10)
    listener()
