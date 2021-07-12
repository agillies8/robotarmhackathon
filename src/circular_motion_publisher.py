#!/usr/bin/env python3
#publishes a message containting string "hello world"

import rospy
from std_msgs.msg import String
import math
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int16, String

def circular_motion():
    rospy.init_node('Hello_world')
    pub=rospy.Publisher('command_pose', PoseStamped, queue_size=1)
    pose_command = PoseStamped()
    circle_center = [0.2,0]
    circle_radius = 0.05    
    pose_command.header.frame_id='world'


    rate=rospy.Rate(20) #rate of it will be published in Hz   
    while not rospy.is_shutdown():

        now = rospy.get_time()
        pose_command.header.stamp=rospy.Time.now()
        pose_command.header.frame_id = 'world'
        pose_command.pose.position.x = circle_center[0]+circle_radius*math.sin(now)
        pose_command.pose.position.y = circle_center[1]+circle_radius*math.cos(now)
        pose_command.pose.position.z = 0

        pub.publish(pose_command)  
        rate.sleep()

if __name__ == '__main__':
    try:
        circular_motion()
    except rospy.ROSInterruptException:
        pass