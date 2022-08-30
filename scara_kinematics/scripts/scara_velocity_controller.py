#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from scara_kinematics.srv import InvVelocityKinematics
import numpy as np

pub1 = rospy.Publisher('/scara/joint1_velocity_controller/command', Float64)
pub2 = rospy.Publisher('/scara/joint2_velocity_controller/command', Float64)
pub3 = rospy.Publisher('/scara/joint3_velocity_controller/command', Float64)
vx = 0
vy = 0.5
vz = 0

omega1 = 0
omega2 = 0
v3 = 0

def get_joint_velocities(data):
    theta1 = data.position[0]
    theta2 = data.position[1]
    print("Theta1: " + str(theta1) + " Theta2: " + str(theta2))

    handle_inv_velocity_kinematics(theta1, theta2)
    print("Omega1: " + str(omega1) + " Omega2: " + str(omega2))

    pub1.publish(Float64(omega1))
    pub2.publish(Float64(omega2))
    pub3.publish(Float64(v3))

def handle_inv_velocity_kinematics(theta1, theta2):
    a1 = 1
    a2 = 1

    if(theta1 == 0 or theta2 == 0):
        return

    s2 = np.sin(theta2)
    c2 = np.cos(theta2)
    sec1 = 1/np.cos(theta1)
    csc1 = 1/np.sin(theta1)

    global omega1
    global omega2
    global v3

    omega1 = -vx*(c2*csc1)/(a1*(c2-s2)) + vy*(s2*sec1)/(a1*(s2-c2))
    omega2 = vx*(a1*csc1 + a2*c2*csc1)/(a1*a2*(c2-s2)) + vy*(a1*sec1+a2*s2*sec1)/(a1*a2*(c2-s2))
    v3 = -vz

    return
    
def listener():

    rospy.init_node('scara_velocity_controller', anonymous=True)

    rospy.Subscriber("scara/joint_states", JointState, get_joint_velocities)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
