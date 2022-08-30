import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3

def callback(state):
    theta1 = state.position[0]
    theta2 = state.position[1]
    d3 = state.position[2]
    c1 = np.cos(theta1)
    s1 = np.sin(theta1)
    c2 = np.cos(theta2)
    s2 = np.sin(theta2)
    d1 = 1
    a1 = 1
    a2 = 1
    T = [[c1*c2-s1*s2, c1*s2+s1*c2, 0, a1*c1+a2*c1*c2],
        [s1*c2+c1*s2, s1*s2-c1*c2, 0, a1*s1+a2*np.sin(theta1-theta2)],
        [0, 0, 1, d1+d3],
        [0, 0, 0, 1]]

    pos = Vector3(T[0][3], T[1][3], T[2][3])
    
    rospy.Publisher('scara_fwd_kinematics', Vector3).publish(pos)
    
def scara_fwd_kinematics():
    rospy.init_node('scara_fwd_kinematics', anonymous=True)
    rospy.Subscriber("scara/joint_states", JointState, callback)
    rospy.spin()

if __name__ == '__main__':
    scara_fwd_kinematics()