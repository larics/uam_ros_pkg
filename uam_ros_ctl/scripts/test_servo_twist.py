#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import TwistStamped

def publish_twist():
    rospy.init_node('sinusoidal_twist_publisher', anonymous=True)
    pub = rospy.Publisher('/control_arm/delta_twist_cmds', TwistStamped, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    start_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        elapsed_time = current_time.to_sec() - start_time

        phase = int(elapsed_time // 10) % 3  # 0: x, 1: y, 2: z

        twist_msg = TwistStamped()
        twist_msg.header.stamp = current_time
        twist_msg.header.frame_id = "end_effector_base"

        sinusoid_value = math.sin(elapsed_time)
        
        if phase == 0:
            twist_msg.twist.linear.x = sinusoid_value
        elif phase == 1:
            twist_msg.twist.linear.y = sinusoid_value
        else:
            twist_msg.twist.linear.z = sinusoid_value * 0.5

        pub.publish(twist_msg)
        rospy.loginfo(f'Publishing TwistStamped: {twist_msg}')
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_twist()
    except rospy.ROSInterruptException:
        pass
