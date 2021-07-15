#!/usr/bin/env python3
#publishes a series of messages to the robot causing it to follow a path.
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

    def path_wait(self):
        while not rospy.is_shutdown():
            rospy.spin()

    def path_callback(self, msg):
        pose_command = PoseStamped()  
        pose_command.header.frame_id='world'
        path = [[0.2,0,0.2],
                [0.2,0,0],
                [0.2,-0.05,0],
                [0.15,-0.05,0],
                [0.15,0,0],
                [0.2,0,0],
                [0.2,0,0.2]]

        rate=rospy.Rate(0.3) #rate of it will be published in Hz   
    
        for i in range(len(path)):
            pose_command.header.stamp=rospy.Time.now()
            pose_command.header.frame_id = 'world'
            pose_command.pose.position.x = path[i][0]
            pose_command.pose.position.y = path[i][1]
            pose_command.pose.position.z = path[i][2]
            self.pub.publish(pose_command) 
            rate.sleep()

if __name__ == '__main__':
    try:
        pathpub= PathPub()
        pathpub.path_wait()
    except rospy.ROSInterruptException:
        pass