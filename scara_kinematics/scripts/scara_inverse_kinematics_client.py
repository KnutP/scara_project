from __future__ import print_function

import sys
import rospy
from scara_kinematics.srv import InverseKinematics

def scara_inverse_kinematics_callback(x, y, z):
    rospy.wait_for_service('scara_inverse')
    try:
        inverse_kinematics = rospy.ServiceProxy('scara_inverse', InverseKinematics)
        resp1 = inverse_kinematics(x, y, z)
        return resp1.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y z]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 4:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting pose: %s %s %s"%(x, y, z))
    print(scara_inverse_kinematics_callback(x, y, z))