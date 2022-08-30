#!/usr/bin/env python
from geometry_msgs.msg import Vector3
import rospy
import numpy as np
from scara_kinematics.srv import InverseKinematics

def handle_inverse_kinematics(pose):
    a1 = 1
    a2 = 1
    d1 = 1

    D = (pose.x**2+pose.y**2-a1**2-a2**2)/(2*a1*a2)
    theta2 = np.arctan2(D, np.sqrt(1-D**2))
    theta1 = np.arctan2(pose.x, pose.y) - np.arctan2(a1+a2*np.cos(theta2), a2*np.sin(theta2))
    d3 = d1 - pose.z

    return Vector3(theta1, theta2, d3)

def scara_inverse_server():
    rospy.init_node('scara_inverse_server')
    s = rospy.Service('scara_inverse', InverseKinematics, handle_inverse_kinematics)
    print("Ready to compute inverse kinematics.")
    rospy.spin()

if __name__ == "__main__":
    scara_inverse_server()