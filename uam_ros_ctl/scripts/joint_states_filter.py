#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

def joint_state_callback(msg):
    rospy.loginfo_once("Recieved joint states msg!")
    filtered_msg = JointState()
    filtered_msg.header = msg.header
    
    for i, name in enumerate(msg.name):
        if "arm_joint" in name:
            filtered_msg.name.append(name)
            filtered_msg.position.append(msg.position[i])
            filtered_msg.velocity.append(msg.velocity[i] if msg.velocity else 0.0)
            filtered_msg.effort.append(msg.effort[i] if msg.effort else 0.0)
    
    if filtered_msg.name:  # Only publish if there's relevant data
        pub.publish(filtered_msg)

if __name__ == "__main__":
    rospy.init_node("joint_state_filter")
    pub = rospy.Publisher("/arm_joint_states", JointState, queue_size=5)
    rospy.Subscriber("/red/joint_states", JointState, joint_state_callback)
    rospy.spin()
