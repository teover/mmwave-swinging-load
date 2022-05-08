#!/usr/bin/env python2.7

## Converts /qtm_data matrix to euler angles for "pile-setup.urdf"
## Publishes for in urdf "radar pile" for now
## Sets /joint_states

import rospy
import numpy as np
from ti_mmwave_rospkg.msg import RadarScan
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_matrix
from math import pi
from math import atan
from motionlab_udp.msg import QTM

class CommandToJointState:
    def __init__(self):
        self.joint_state = JointState()
        self.joint_state.name.append("base_to_pipe_x_radar")
        self.joint_state.name.append("base_to_pipe_y_radar")
        self.joint_state.name.append("base_to_pipe_z_radar")
        rospy.loginfo("Publishing joint_states for " + str(self.joint_state.name))
        self.joint_state.position.append([0.0, 0.0, 0.0])
        self.joint_state.velocity.append(0.0)
        self.joint_pub = rospy.Publisher("joint_states", JointState, queue_size=1)
        self.command_sub = rospy.Subscriber("/QUM_data", QTM, self.command_callback, queue_size=1)



    def command_callback(self, msg):
        # Obtain quaternions xyzw from message
        matrix = [msg.r[0], msg.r[1], msg.r[2], msg.r[3], msg.r[4], msg.r[5], msg.r[6], msg.r[7], msg.r[8]]
        np.array(matrix).reshape(3, 3)
        # Convert quaternions to euler angles  
        (roll, pitch, yaw) = euler_from_matrix(matrix)
        # Log degrees every second
        rospy.loginfo_throttle(1, "Roll: " + str(roll*(180/pi)) + " Pitch: " + str(pitch*(180/pi)) + " Yaw: " + str(yaw*(180/pi)))
        # Set joint states from angles
        self.joint_state.position = (roll, -pitch, yaw)
        self.joint_state.header.stamp = rospy.Time.now()
        # Publish to topic
        self.joint_pub.publish(self.joint_state)


if __name__ == '__main__':
    rospy.init_node('command_to_joint_state_qtm')
    command_to_joint_stateX = CommandToJointState()
    rospy.spin()



