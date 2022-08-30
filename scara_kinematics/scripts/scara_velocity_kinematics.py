#!/usr/bin/env python
from geometry_msgs.msg import Vector3
import rospy
import numpy as np
from scara_kinematics.srv import InvVelocityKinematics, FwdVelocityKinematics

# Input needs to be a FwdVelocityKinematics object,
# where omega1, omega2, and v3 are the desired velocities for the three joints,
# and where theta1 and theta2 are the current angles of the first two joints
def handle_fwd_velocity_kinematics(state):
    a1 = 1
    a2 = 1

    s1 = np.sin(state.theta1)
    c1 = np.cos(state.theta1)
    s12 = s1*np.sin(state.theta2)
    c12 = c1*np.cos(state.theta2)

    vx = -state.omega1*(a1*s1+a2*s12) - state.omega2*a2*s12
    vy = state.omega1*(a1*c1+a2*c12) - state.omega2*a2*c12
    vz = -state.v3

    return Vector3(vx, vy, vz)

# Input needs to be an InvVelocityKinematics object,
# where vx, vy, and vz are the desired velocities in the x, y, and z directions
# and where theta1 and theta2 are the current angles of the first two joints
def handle_inv_velocity_kinematics(state):
    a1 = 1
    a2 = 1

    s2 = np.sin(state.theta2)
    c2 = np.cos(state.theta2)
    sec1 = 1/np.cos(state.theta1)
    csc1 = 1/np.sin(state.theta1)

    omega1 = -state.vx*(c2*csc1)/(a1*(c2-s2)) + state.vy*(s2*sec1)/(a1*(s2-c2))
    omega2 = state.vx*(a1*csc1 + a2*c2*csc1)/(a1*a2*(c2-s2)) + state.vy*(a1*sec1+a2*s2*sec1)/(a1*a2*(c2-s2))
    v3 = -state.vz

    return Vector3(omega1, omega2, v3)


# Function to in initialize the two services
def scara_velocity_server():
    rospy.init_node('scara_velocity_server')
    inv = rospy.Service('scara_inv_velocity', InvVelocityKinematics, handle_inv_velocity_kinematics)
    fwd = rospy.Service('scara_fwd_velocity', FwdVelocityKinematics, handle_fwd_velocity_kinematics)
    print("Ready to compute velocity kinematics.")
    rospy.spin()

if __name__ == "__main__":
    scara_velocity_server()