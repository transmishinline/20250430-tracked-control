#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TwistStamped

def main():
    rospy.init_node('setpoint_streamer')
    pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',
                          TwistStamped, queue_size=1)
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        # zero-motion (dummy) or real commands
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main()

