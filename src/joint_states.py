#!/usr/bin/env python3

#This note subscribes to the Image message and translates that into joint positions for the panels

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped

def joint_states_callback(msg):

    #setup the publisher
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)

    #create joint state message and publish
    js = JointState()
    js.header = Header()
    js.header.stamp = rospy.Time.now()
    js.name = ['axis_1_child', 'arm1', 'arm2', 'endeffector1']
    js.position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.y +msg.pose.position.z, -1*msg.pose.position.z]
    js.velocity = []
    js.effort = []
    pub.publish(js)

def commander():
    #create node and setup subscriber for image message
    rospy.init_node('joint_states')
    sub=rospy.Subscriber('command_joints', PoseStamped, joint_states_callback)

    while not rospy.is_shutdown():
        rospy.spin() #keep active

if __name__ == '__main__':
    try:
        commander()
    except rospy.ROSInterruptException:
        pass