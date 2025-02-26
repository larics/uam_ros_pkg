import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class ArmDynamicsNode:
    def __init__(self):
        rospy.init_node('robot_dynamics_node', anonymous=True, log_level=rospy.DEBUG)
        
        # Subscriber to get joint states (positions, velocities)
        self.joint_state_sub = rospy.Subscriber(
            '/arm_joint_states', JointState, self.joint_state_callback)
        # Storage for joint state data
        self.q_p = []
        self.q_v = []
        # Timer for periodic computation
        self.rate = rospy.Rate(50)  # 100 Hz
        l1 = 0.1
        l2 = 0.1
        l3 = 0.1
        l4 = 0.1
        self.l = [l1, l2, l3, l4]
        self.reciv_jnt = False
    
    def joint_state_callback(self, msg):
        self.reciv_jnt = True
        """Callback function to update joint state values."""
        self.joint_positions = msg.position
        self.joint_velocities = msg.velocity
        self.q_p = np.array(self.joint_positions)
        self.q_v = np.array(self.joint_velocities)
    
    def forward_NE(self):
        """Forward Newton-Euler calculations."""
        n = len(self.q_p)
        w = np.zeros((n, 3))  # Angular velocity
        v = np.zeros((n, 3))  # Linear velocity
        
        for i in range(n):
            if i == 0:
                w[i] = np.array([0, 0, self.q_v[i]])
                v[i] = np.array([0, 0, 0])
            else:
                w[i] = w[i-1] + np.array([0, 0, self.q_v[i]])
                v[i] = v[i-1] + np.cross(w[i], np.array([self.l[i-1], 0, 0]))
        rospy.logdebug(f"Current w: {w}")
        rospy.logdebug(f"Current v: {v}")

        return w, v
    
    def back_NE(): 
        F = []
        t = []
        return F, t

    def run(self):
        while not rospy.is_shutdown():
            self.forward_NE()
            self.rate.sleep()

if __name__ == '__main__':
    aD = ArmDynamicsNode()
    aD.run()
