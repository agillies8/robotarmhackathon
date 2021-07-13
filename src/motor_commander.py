#!/usr/bin/env python3
#publishes absolute motor commands message from a keyboard input

import sys
from select import select
import rospy
from geometry_msgs.msg import Pose, PoseStamped

RAD_TO_STEP = 1018

class Commander():
    def __init__(self):
        rospy.init_node('motor_commander')
        self.pub=rospy.Publisher('move_axis_absolute', Pose, queue_size=1)
        self.sub=rospy.Subscriber('command_joints', PoseStamped, self.joints_callback)
        self.rate=rospy.Rate(1)

    def joints_callback(self, msg):
      command = Pose()
      command.position.x = int(msg.pose.position.x * RAD_TO_STEP)
      command.position.y = int(msg.pose.position.y * RAD_TO_STEP)
      command.position.z = int(msg.pose.position.z * RAD_TO_STEP)
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