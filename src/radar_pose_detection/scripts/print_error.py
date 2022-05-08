
import rospy
from sensor_msgs.msg import JointState
from math import pi


class CommandToJointState:
    def __init__(self):
        self.command_sub = rospy.Subscriber("/joint_states", JointState, self.command_callback, queue_size=1)
        self.imu_angles = (0, 0)
        self.radar_angles = (0, 0)
        self.count = 0
        self.sumX = 0
        self.sumY = 0
        self.diffX = 0
        self.diffY = 0


    def command_callback(self, msg):
        
        if(msg.name[0] == "base_to_pipe_x"):
            self.imu_angles = (round(msg.position[0]*(180/pi), 1), round(msg.position[1]*(180/pi),1))

        if(msg.name[0] == "base_to_pipe_x_radar"):
            self.radar_angles = (round(msg.position[0]*(180/pi), 1), round(msg.position[1]*(180/pi), 1))

        

        self.diffX = self.radar_angles[0] - self.imu_angles[0]
        self.diffY = self.radar_angles[1] - self.imu_angles[1]
        self.count = self.count + 1
        self.sumX = self.sumX + abs(self.diffX)
        self.sumY = self.sumY + abs(self.diffY)

        avg_errX = round(self.sumX/self.count, 2)
        avg_errY = round(self.sumY/self.count, 2)



        rospy.loginfo_throttle(0.25, "radar_angles: " + str(self.radar_angles) + "\t imu_angles: " + str(self.imu_angles) + "\t diffX: " + str(self.diffX)+ "\t diffY: " + str(self.diffY) + "\t avg_errX: " + str(avg_errX) + "\t avg_errY: " + str(avg_errY))




if __name__ == '__main__':
    rospy.init_node('angle_error_printer')
    command_to_joint_stateX = CommandToJointState()
    rospy.spin()
