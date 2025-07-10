#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def main():
    rospy.init_node('move_forward_with_safe_shutdown')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


    forward = Twist()
    forward.linear.x = 0.4 

    stop = Twist()

    rate = rospy.Rate(10)  # 10Hz

    rospy.loginfo("Moving forward. Press Ctrl+C to stop.")
    try:
        while not rospy.is_shutdown():
            pub.publish(forward)
            rate.sleep()
    except KeyboardInterrupt:
        rospy.loginfo("Interrupted! Stopping the robot...")
    finally:
   
        for _ in range(10):
            pub.publish(stop)
            rate.sleep()
        rospy.loginfo("Robot stopped.")

if __name__ == '__main__':
    main()
