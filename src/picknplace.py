#!/usr/bin/env python3
#publishes a series of messages to the robot causing it to pick and place an object from a to b.
#to home the robot before beginning, open a terminal and enter this command: rostopic pub home_axis std_msgs/Bool True -1
#to start the path, enter this command: rostopic pub start_path std_msgs/Bool True -1

import rospy
from std_msgs.msg import String, Bool
import math
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int16, String



class PathPub():
    def __init__(self):
        rospy.init_node('path_pub')    
        self.sub=rospy.Subscriber('start_path', Bool, self.path_callback)
        self.pub=rospy.Publisher('command_pose', PoseStamped, queue_size=1)
        self.gripperpub = rospy.Publisher('gripper', Bool, queue_size = 1)
        self.gripper_msg = Bool()
        self.gripper_msg.data = True
        if self.gripper_msg.data == True:
                self.gripper_msg.data = False
        else:
            self.gripper_msg.data = True
        self.gripperpub.publish(self.gripper_msg)

    def path_wait(self):
        while not rospy.is_shutdown():
            rospy.spin()

    def path_callback(self, msg):
        pose_command = PoseStamped()  
        pose_command.header.frame_id='world'
        approach_a = [[0.2,0,0.2],
                [0.2,0.08,0.2],
                [0.2,0.08,0]]

        approach_b = [[0.2,0.1,0.2],
                [0.2,-0.08,0.2],
                [0.2,-0.08,0]]

        approach_home = [[0.2,-0.08,0.2],
                        [0.2,0.0,0.2]]

        rate=rospy.Rate(0.3) #rate of it will be published in Hz   

        self.gripper_msg.data = True
        self.gripperpub.publish(self.gripper_msg)
        rate.sleep()

        for i in range(len(approach_a)):
            pose_command.header.stamp=rospy.Time.now()
            pose_command.header.frame_id = 'world'
            pose_command.pose.position.x = approach_a[i][0]
            pose_command.pose.position.y = approach_a[i][1]
            pose_command.pose.position.z = approach_a[i][2]
            self.pub.publish(pose_command) 
            rate.sleep()

        self.gripper_msg.data = False
        self.gripperpub.publish(self.gripper_msg)
        rate.sleep()

        for i in range(len(approach_b)):
            pose_command.header.stamp=rospy.Time.now()
            pose_command.header.frame_id = 'world'
            pose_command.pose.position.x = approach_b[i][0]
            pose_command.pose.position.y = approach_b[i][1]
            pose_command.pose.position.z = approach_b[i][2]
            self.pub.publish(pose_command) 
            rate.sleep()

        self.gripper_msg.data = True
        self.gripperpub.publish(self.gripper_msg)
        rate.sleep()

        for i in range(len(approach_home)):
            pose_command.header.stamp=rospy.Time.now()
            pose_command.header.frame_id = 'world'
            pose_command.pose.position.x = approach_home[i][0]
            pose_command.pose.position.y = approach_home[i][1]
            pose_command.pose.position.z = approach_home[i][2]
            self.pub.publish(pose_command) 
            rate.sleep()

if __name__ == '__main__':
    try:
        pathpub= PathPub()
        pathpub.path_wait()
    except rospy.ROSInterruptException:
        pass