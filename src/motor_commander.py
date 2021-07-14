#!/usr/bin/env python3
#publishes absolute motor commands message from a keyboard input

import sys
from select import select
import rospy
from geometry_msgs.msg import Pose, PoseStamped

AXIS_1_COUNTS_PER_REV = rospy.get_param("AXIS_1_COUNTS_PER_REV")
AXIS_2_COUNTS_PER_REV = rospy.get_param("AXIS_2_COUNTS_PER_REV")
AXIS_3_COUNTS_PER_REV = rospy.get_param("AXIS_3_COUNTS_PER_REV")
RAD_TO_REV = 1/(2*3.1415)

class Commander():
    def __init__(self):
        rospy.init_node('motor_commander')
        self.pub=rospy.Publisher('move_axis_absolute', Pose, queue_size=1)
        self.sub=rospy.Subscriber('command_joints', PoseStamped, self.joints_callback)
        self.rate=rospy.Rate(1)

    def joints_callback(self, msg):
      command = Pose()
      command.position.x = int(msg.pose.position.x * AXIS_1_COUNTS_PER_REV * RAD_TO_REV)
      command.position.y = int(msg.pose.position.y * AXIS_2_COUNTS_PER_REV * RAD_TO_REV)
      command.position.z = int(msg.pose.position.z * AXIS_3_COUNTS_PER_REV * RAD_TO_REV)
      self.pub.publish(command)

    def start(self):

        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        commander = Commander()
        commander.start()
    except rospy.ROSInterruptException:
        pass