#!/usr/bin/env python2.7

## Converts /imu_iphone quaternions to euler angles for "pile-setup.urdf"
## Sets /joint_states

import rospy
from ti_mmwave_rospkg.msg import RadarScan
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from math import pi
from math import atan


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
        self.command_sub = rospy.Subscriber("/ti_mmwave/radar_scan", RadarScan, self.command_callback, queue_size=1)



    def command_callback(self, msg):
        # Filter based on  x value and velocity 
        if msg.x < 1.3 and msg.y > -1 and msg.y < 1 and (msg.velocity > 0 or msg.velocity < 0):

            # Calculate position with atan(x/ pipe length)
            # msg.x-1 : 1 meter is subtracted as the pile is placed 1 meter from the radar
            self.joint_state.position = (atan(msg.y/0.5), -atan((msg.x-1)/0.5), 0)
            rospy.loginfo("thetaX:" + str(-atan((msg.x-1)/0.5)*(180/pi)) + " \tthetaY: " + str(atan(msg.y/0.5)*(180/pi)) + " \t\tIntensity: " + str(msg.intensity) + " point_id: " + str(msg.point_id) + " velocity: " + str(msg.velocity))
            #rospy.loginfo_throttle(1, "X: " + str(msg.x) + " Y: " + str(msg.y))
            self.joint_state.header.stamp = rospy.Time.now()
            # Publish to topic
            self.joint_pub.publish(self.joint_state)


if __name__ == '__main__':
    rospy.init_node('command_to_joint_state_radar')
    command_to_joint_stateX = CommandToJointState()
    rospy.spin()



