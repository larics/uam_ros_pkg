import rospy
import numpy as np
from scipy.spatial.transform import Rotation 
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
        self.reciv_jnt = False
        self.reciv_odom = False
        self.first_jnt = True

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

        """Callback function to update joint state values."""
        self.joint_positions = msg.position
        self.joint_velocities = msg.velocity
        self.joint_accelerations = None
        self.q_p = np.array(self.joint_positions)
        self.q_v = np.array(self.joint_velocities)
        if self.first_jnt:
            self.prev_q_v = np.zeros(len(self.q_v))
            self.jt = msg.header.stamp.to_sec()
            self.first_jnt = False
        else: 
            jt_ = msg.header.stamp.to_sec()
            dt = self.jt - jt_
            self.q_a = self.calc_q_acc(self.prev_q_v, self.q_v, dt)
            self.prev_q_v = self.q_v
            self.jt = jt_

        # TODO: Add acceleration calculation
        # self.q_a = np.array(calculate_accelerations())
        self.reciv_jnt = True


    def odom_callback(self, msg): 
        self.uav_w = np.array([msg.twist.twist.angular.x, 
                               msg.twist.twist.angular.y, 
                               msg.twist.twist.angular.z])
        self.uav_v = np.array([msg.twist.twist.linear.x, 
                               msg.twist.twist.linear.y, 
                               msg.twist.twist.linear.z])
        self.reciv_odom = True

    def imu_callback(self, msg): 
        self.uav_a = np.array([msg.linear_acceleration.x,
                               msg.linear_acceleration.y,
                               msg.linear_acceleration.z])
        self.reciv_imu = True
                            
    
    def forward_NE(self, dS, R):
        """Forward Newton-Euler calculations."""
        n = len(self.q_p)
        w = np.zeros((n, 3))  # Angular velocity
        v = np.zeros((n, 3))  # Linear velocity
        alpha = np.zeros((n, 3)) # Angular acceleration
        a = np.zeros((n, 3)) # Linear acceleration
        # TODO: Add implementation of the linear and angular accs
        for i in range(n):
            if i == 0:
                if self.reciv_odom:
                    w[i] = self.uav_w
                    v[i] = self.uav_v
                else:
                    w[i] = np.zeros(3)
                    v[i] = np.zeros(3)
                    # CAN PROBABLY USE IMU DATA
                    # Check what happens with gravity HERE! 
            else:
                # Angular
                # Velocity
                w[i] = w[i-1] + np.dot(self.q_v[i], R[i-1][:, 2])
                # Acceleration
                alpha[i] = alpha[i-1] + np.dot(self.q_a[i], R[i-1][:, 2]) + np.cross(w[i-1], np.dot(self.q_v[i], R[i-1][:, 2]))  

                # Linear
                # Velocity
                v[i] = v[i-1] + np.cross(w[i], np.array(dS[i-1])) + np.dot(self.q_v[i], R[i-1][:, 2])
                # Acceleration
                wxS = np.cross(w[i], dS[i-1])
                alphaxS = np.cross(alpha[i], dS[i-1])
                wxwxS = np.cross(w[i], wxS)
                a[i] = a[i-1] + alphaxS + wxwxS

        #rospy.logdebug(f"Current w: {w}")
        #rospy.logdebug(f"Current v: {v}")
        return w, alpha, v, a
    
    def back_NE(): 
        F = []
        t = []
        return F, t

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            p1, R1 = self.lookup_transform('link1', 'base')
            p2, R2 = self.lookup_transform('link2', 'link1')
            p3, R3 = self.lookup_transform('link3', 'link2')
            p4, R4 = self.lookup_transform('link4', 'link3')
            p5, R5 = self.lookup_transform('link5', 'link4')
            # Check if distances are consistent:
            dS = np.round([p2 - p1, p3 - p2, p4 - p3, p5 - p4], 3)
            dC = -dS/2
            Rs = [R1, R2, R3, R4, R5]
            R_ = [Rotation.from_quat((R_[0], R_[1], R_[2], R_[3])).as_matrix() for R_ in Rs]
            if self.reciv_jnt:
                w, alpha, v, a = self.forward_NE(dS, R_)
            #print(w, v)

            #print(self.dS)

    def calc_q_acc(self, q_k, q_k1, dt):
        q_a = np.round(np.array((q_k1-q_k)/dt), 3 )
        return q_a

if __name__ == '__main__':
    aD = ArmDynamicsNode()
    aD.run()
