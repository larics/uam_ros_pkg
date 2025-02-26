import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import tf

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

        self.tf_listener = tf.TransformListener()

    def lookup_transform(self, target_frame, source_frame):
        """Lookup transform between target and source frames."""
        try:
            self.tf_listener.waitForTransform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
            return np.array(trans), np.array(rot)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logwarn(f"Could not lookup transform from {source_frame} to {target_frame}")
            return None, None

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
                v[i] = v[i-1] + np.cross(w[i], np.array(self.dS[i-1]))
        #rospy.logdebug(f"Current w: {w}")
        #rospy.logdebug(f"Current v: {v}")
        return w, v
    
    def back_NE(): 
        F = []
        t = []
        return F, t

    def run(self):
        while not rospy.is_shutdown():
            self.forward_NE()
            self.rate.sleep()
            p1, R1 = self.lookup_transform('link1', 'base')
            p2, R2 = self.lookup_transform('link2', 'link1')
            p3, R3 = self.lookup_transform('link3', 'link2')
            p4, R4 = self.lookup_transform('link4', 'link3')
            p5, R5 = self.lookup_transform('link5', 'link4')
            self.dS = np.round([p2 - p1, p3 - p2, p4 - p3, p5 - p4], 3)
            print(self.dS)

if __name__ == '__main__':
    aD = ArmDynamicsNode()
    aD.run()
