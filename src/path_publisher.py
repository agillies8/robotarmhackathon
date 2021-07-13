#!/usr/bin/env python3
#publishes a message containting string "hello world"

import rospy
from std_msgs.msg import String
import math
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int16, String

def path_pub():
    rospy.init_node('path_pub')
    pub=rospy.Publisher('command_pose', PoseStamped, queue_size=1)
    pose_command = PoseStamped()  
    pose_command.header.frame_id='world'
    path = [[0.1,0.2,0.2],
            [0.2,0.2,0.1],
            [0.1,-0.2,0.1],
            [0.1,-0.2,0.2],
            [0.2,0.1,0.15]]


    rate=rospy.Rate(1) #rate of it will be published in Hz   
    while not rospy.is_shutdown():
        for i in range(len(path)):
            now = rospy.get_time()
            pose_command.header.stamp=rospy.Time.now()
            pose_command.header.frame_id = 'world'
            pose_command.pose.position.x = path[i][0]
            pose_command.pose.position.y = path[i][1]
            pose_command.pose.position.z = path[i][2]
            pub.publish(pose_command)  
            rate.sleep()

if __name__ == '__main__':
    try:
        path_pub()
    except rospy.ROSInterruptException:
        pass